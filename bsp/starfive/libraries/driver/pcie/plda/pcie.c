
#include <rthw.h>
#include <rtthread.h>

#include <riscv_io.h>
#include "hal_pcie.h"
#include "../pci.h"

#include "../pci_ids.h"
#include "board.h"
#include "pcie.h"

static struct plda_pcie plda_inst[2];

extern uint32_t pci_size, offset;
#define ECAM_BUS_SHIFT			20
#define ECAM_DEV_SHIFT			15
#define ECAM_FUNC_SHIFT			12
/* Secondary bus number offset in config space */
#define PCI_SECONDARY_BUS		0x19

static inline void plda_pcie_enable_root_port(void *bridge_addr)
{
	sys_setbits(bridge_addr + GEN_SETTINGS, RP_ENABLE);
}

static inline void plda_pcie_set_standard_class(void *bridge_addr)
{
	/* set class code and reserve revision id */
	sys_writel((PCI_CLASS_BRIDGE_PCI << IDS_CLASS_CODE_SHIFT), bridge_addr + PCIE_PCI_IDS_DW1);
}

static inline void plda_pcie_set_pref_win_64bit(void *bridge_addr)
{
	sys_setbits(bridge_addr + PCIE_WINROM, PREF_MEM_WIN_64_SUPPORT);
}

static inline void plda_pcie_disable_ltr(void *bridge_addr)
{
	sys_clrbits(bridge_addr + PMSG_SUPPORT_RX, PMSG_LTR_SUPPORT);
}

static inline void plda_pcie_clear_rc_bar(void *bridge_addr)
{
	sys_writel(0, bridge_addr + 0x1000 + PCI_BASE_ADDRESS_0);
	sys_writel(0, bridge_addr + 0x1000 + PCI_BASE_ADDRESS_1);
}

void xr3pci_set_atr_entry(unsigned long base, unsigned long src_addr,
			unsigned long trsl_addr, int window_size,
			int trsl_param)
{
	/* X3PCI_ATR_SRC_ADDR_LOW:
	     - bit 0: enable entry,
	     - bits 1-6: ATR window size: total size in bytes: 2^(ATR_WSIZE + 1)
	     - bits 7-11: reserved
	     - bits 12-31: start of source address
	*/
	sys_writel((unsigned int)(src_addr & 0xfffff000) | (window_size - 1) << 1 | 1,
	       base + XR3PCI_ATR_SRC_ADDR_LOW);
	sys_writel((unsigned int)(src_addr >> 32), base + XR3PCI_ATR_SRC_ADDR_HIGH);
	sys_writel((unsigned int)(trsl_addr & 0xfffff000), base + XR3PCI_ATR_TRSL_ADDR_LOW);
	sys_writel((unsigned int)(trsl_addr >> 32), base + XR3PCI_ATR_TRSL_ADDR_HIGH);
	sys_writel(trsl_param, base + XR3PCI_ATR_TRSL_PARAM);

	hal_printf("ATR entry: 0x%010lx %s 0x%010lx [0x%010llx] (param: 0x%06x)\n",
	       src_addr, (trsl_param & 0x400000) ? "<-" : "->", trsl_addr,
	       ((unsigned long)1) << window_size, trsl_param);
}

void xr3pci_setup_atr(struct plda_pcie *pcie)
{
    unsigned long csr_base = (unsigned long)pcie->bridge_base;
    unsigned long config_base = (unsigned long) pcie->cfg_base;

    /* setup CPU to PCIe address translation table */
    unsigned long base = csr_base + XR3PCI_ATR_AXI4_SLV0;

    /* setup ECAM space to bus configuration interface */
    xr3pci_set_atr_entry(base, config_base, 0, XR3_PCI_ECAM_SIZE,
			     XR3PCI_ATR_TRSLID_PCIE_CONF);

    pcie->atr_num++;
    base += XR3PCI_ATR_TABLE_SIZE;

	/* setup 64bit MEM space translation */
    xr3pci_set_atr_entry(base, pcie->pcie->cfg.mem64_base,  pcie->pcie->cfg.mem64_base,
			      pcie->pcie->cfg.mem64_log, XR3PCI_ATR_TRSLID_PCIE_MEMORY);

    pcie->atr_num++;
    base += XR3PCI_ATR_TABLE_SIZE;

	/* setup 32bit MEM space translation */
    xr3pci_set_atr_entry(base, pcie->pcie->cfg.mem32_base, pcie->pcie->cfg.mem32_base,
			     pcie->pcie->cfg.mem32_log, XR3PCI_ATR_TRSLID_PCIE_MEMORY);

    pcie->atr_num++;
#if 0
    unsigned int primary_bus = 0;
    int devices =  PCI_BDF(0,0,0);

    pci_generic_ecam_read_config(pcie, devices, PCI_SECONDARY_BUS, &primary_bus,PCI_SIZE_8);
    primary_bus = (1 << 0);
    pci_generic_ecam_write_config(pcie, devices, PCI_SECONDARY_BUS, primary_bus,PCI_SIZE_8);
#endif
}

static void plda_pcie_handle_errors_irq(struct plda_pcie *pcie, uint32_t status)
{
	if (status & INT_DMA_ERROR) {
		uint32_t reg_val = sys_readl(pcie->bridge_base + XR3PCI_ISTATUS_DMA0);
		hal_printf("DMA ERROR, DMA_STATUS: 0x%x\n", reg_val);
	}
	if (status & INT_AXI_POST_ERROR)
		hal_printf( "AXI post error\n");
	if (status & INT_AXI_FETCH_ERROR)
		hal_printf( "AXI fetch error\n");
	if (status & INT_AXI_DISCARD_ERROR)
		hal_printf( "AXI discard error\n");
	if (status & INT_PCIE_POST_ERROR)
		hal_printf( "PCIe post error\n");
	if (status & INT_PCIE_FETCH_ERROR)
		hal_printf( "PCIe fetch error\n");
	if (status & INT_PCIE_DISCARD_ERROR)
		hal_printf( "PCIe discard error\n");

}

static void plda_handle_msi(struct plda_pcie *pcie)
{
	uint32_t reg_val;
	int ret, i;

	reg_val = sys_readl(pcie->bridge_base + XR3PCI_ISTATUS_MSI);
	if (reg_val) {
		for (i = 0; i < 8; i++) {
			if (reg_val & BIT(i)) {
				sys_writel(BIT(i), pcie->bridge_base  + XR3PCI_ISTATUS_MSI);
				if (pcie->msi[i].msi_handler)
					pcie->msi[i].msi_handler(i, pcie->msi[i].arg);
			}
		}
	}
}

static void plda_compose_msi_msg(void *priv, void *msi)
{
	struct msi_msg *msg = (void *)msi;

	msg->address_lo = IMSI_ADDR;
	msg->address_hi = 0;
	//msg->data = data->hwirq;

	hal_printf("msi#%x address_hi %#x address_lo %#x\n",
		(int)msg->data, msg->address_hi, msg->address_lo);
}

static void plda_enable_int(struct plda_pcie *pcie)
{
	unsigned int reg_val;

	sys_writel(0, pcie->bridge_base + IMASK_LOCAL);

	reg_val =  INT_ERRORS | INT_PCIE_MSI;

	sys_writel(reg_val, pcie->bridge_base + IMASK_LOCAL);
}

static void plda_register_msi(void *instance, void *arg, void *handler, int irq)
{
	struct plda_pcie *pcie = instance;
	int msi_index = irq;

	pcie->msi[msi_index].arg = arg;
	pcie->msi[msi_index].msi_handler = handler;
	pcie->msi[msi_index].irqno = irq;
}

static int plda_alloc_msi_irq_num(void *instance, int num)
{
	struct plda_pcie *pcie = instance;
	int start_num = pcie->allocate_irq_num;

	if (start_num + num > 8)
		return -1;

	pcie->allocate_irq_num += num;

	return start_num;
}

static int pcie_irq_handle(void *priv)
{
    struct plda_pcie *pcie = (struct plda_pcie *)priv;
    unsigned int reg_val = 0;

    reg_val = sys_readl(pcie->bridge_base + XR3PCI_ISTATUS_LOCAL);
//     printf("XR3PCI_ISTATUS_LOCAL = 0x%x\r\n", reg_val);
    sys_writel(reg_val, pcie->bridge_base + XR3PCI_ISTATUS_LOCAL);

    if (reg_val & INT_ERRORS)
	plda_pcie_handle_errors_irq(pcie, reg_val);

    plda_handle_msi(pcie);

    return 0;
}

static int starfive_pcie_addr_valid(pci_dev_t bdf, int first_busno)
{
	if ((PCI_BUS(bdf) == first_busno) && (PCI_DEV(bdf) > 0))
		return 0;
	if ((PCI_BUS(bdf) == first_busno + 1) && (PCI_DEV(bdf) > 0))
		return 0;

	return 1;
}

static int starfive_pcie_off_conf(pci_dev_t bdf, unsigned int offset, int first_busno)
{
	unsigned int bus = PCI_BUS(bdf) - first_busno;
	unsigned int dev = PCI_DEV(bdf);
	unsigned int func = PCI_FUNC(bdf);

	return (bus << ECAM_BUS_SHIFT) | (dev << ECAM_DEV_SHIFT) |
			(func << ECAM_FUNC_SHIFT) | (offset & ~0x3);
}

static int plda_pcie_hide_rc_bar(pci_dev_t bdf, int offset, int first_busno)
{
	if ((PCI_BUS(bdf) == first_busno) &&
	    (offset == PCI_BASE_ADDRESS_0 || offset == PCI_BASE_ADDRESS_1))
		return 1;

	return 0;
}

static int starfive_pcie_config_read(const struct udevice *udev, pci_dev_t bdf,
				unsigned int offset, unsigned long *valuep,
				enum pci_size_t size)
{
	void *addr;
	unsigned long value;
	struct plda_pcie *priv = (void *)udev->priv;
	int where = starfive_pcie_off_conf(bdf, offset, priv->first_busno);

	if (!starfive_pcie_addr_valid(bdf, priv->first_busno)) {
		//hal_printf("Out of range\n");
		*valuep = 0xffff;
		//*valuep = pci_get_ff(size);
		return 0;
	}

	addr = priv->cfg_base;
	addr += where;

	if (!addr)
		return -1;

	/* Make sure the LAST TLP is finished, before reading vendor ID. */
	//if (offset == PCI_VENDOR_ID)
		//sys_mdelay(20);

	value = sys_readl(addr);
	*valuep = pci_conv_32_to_size(value, offset, size);
	//hal_printf("config read where 0x%x bdf 0x%x offset 0x%x val 0x%x\n", where, bdf, offset, *valuep);

	return 0;

}

int starfive_pcie_config_write(struct udevice *udev, pci_dev_t bdf,
				 unsigned int offset, unsigned long value,
				 enum pci_size_t size)
{
	void *addr;
	unsigned long old;
	struct plda_pcie *priv = (void *)udev->priv;
	int where = starfive_pcie_off_conf(bdf, offset, priv->first_busno);

	if (plda_pcie_hide_rc_bar(bdf, offset, priv->first_busno))
		return -1;

	if (!starfive_pcie_addr_valid(bdf, priv->first_busno)) {
		//hal_printf("Out of range\n");
		return 0;
	}

	//hal_printf("config write where 0x%x bdf 0x%x offset 0x%x value 0x%x\n",  where, bdf, offset, value);
	addr = priv->cfg_base;
	addr += where;

	if (!addr)
		return -1;

	old = sys_readl(addr);
	value = pci_conv_size_to_32(old, value, offset, size);
	sys_writel(value, addr);

	return 0;
}


static const struct dm_pci_ops starfive_pcie_ops = {
	.read_config	= starfive_pcie_config_read,
	.write_config	= starfive_pcie_config_write,
	.register_msi   = plda_register_msi,
	.compose_msi    = plda_compose_msi_msg,
	.alloc_msi_irq	= plda_alloc_msi_irq_num,
};

#if 0

void pcie_msi_enable(struct generic_ecam_pcie *pcie)
{
    int bdf = PCI_BDF(0,0,0);
    //pci_generic_ecam_write_config(pcie, bdf, PCI_BASE_ADDRESS_0, 0x00000000,PCI_SIZE_32);
    //pci_generic_ecam_write_config(pcie, bdf, PCI_BASE_ADDRESS_1, 0x00000002,PCI_SIZE_32);
}

void mask_mult_func(struct generic_ecam_pcie *pcie)
{
    sys_pcie_mask_mult_func(pcie->index);
}


int pcie_ltssm_state_get(struct generic_ecam_pcie *pcie)
{
    return sys_pcie_get_ltssm_state(pcie->index);
}
#endif

int pcie_init(struct pcie *pcie)
{
    struct plda_pcie *plda = &plda_inst[pcie->index];

    plda->pcie = pcie;

    pcie->priv = (void *)plda;
    plda->bridge_base = pcie->cfg.bridge_base;
    plda->cfg_base = pcie->cfg.cfg_base;

    plda_pcie_enable_root_port(pcie->cfg.bridge_base);

    plda_pcie_clear_rc_bar(pcie->cfg.bridge_base);
	//plda_pcie_write_rc_bar(plda, 0);

	/* PCIe PCI Standard Configuration Identification Settings. */
    plda_pcie_set_standard_class(pcie->cfg.bridge_base);

	/*
	 * The LTR message forwarding of PCIe Message Reception was set by core
	 * as default, but the forward id & addr are also need to be reset.
	 * If we do not disable LTR message forwarding here, or set a legal
	 * forwarding address, the kernel will get stuck after the driver probe.
	 * To workaround, disable the LTR message forwarding support on
	 * PCIe Message Reception.
	 */
    plda_pcie_disable_ltr(pcie->cfg.bridge_base);

	/* Prefetchable memory window 64-bit addressing support */
    plda_pcie_set_pref_win_64bit(pcie->cfg.bridge_base);

    pcie->cfg.pcie_post_init_cb(pcie);

    xr3pci_setup_atr(plda);

    plda_enable_int(plda);
    pcie->irq_handle = pcie_irq_handle;

    if (pcie->link_up) {
	plda->first_busno = pci_scan_bus(pcie, (void *)&starfive_pcie_ops);
    }
    return 0;
}


