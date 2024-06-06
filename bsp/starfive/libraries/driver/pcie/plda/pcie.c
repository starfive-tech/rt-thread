
#include <rthw.h>
#include <rtthread.h>

#include <riscv_io.h>
#include "hal_pcie.h"
#include "../pci_ids.h"
#include "board.h"
#include "pcie.h"

static struct plda_pcie plda_inst[2];

extern uint32_t pci_size, offset;

#if 0
static inline void plda_set_default_msi(struct plda_msi *msi)
{
	msi->vector_phy = IMSI_ADDR;
	msi->num_vectors = PLDA_MAX_NUM_MSI_IRQS;
}
#endif

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

#if 0
static inline void plda_pcie_write_rc_bar(void *bridge_addr, uint64_t val)
{
	void __iomem *addr = plda->bridge_addr + CONFIG_SPACE_ADDR_OFFSET;

	sys_writel(val && 0xffffffff, addr + PCI_BASE_ADDRESS_0);
	writel_relaxed(upper_32_bits(val), addr + PCI_BASE_ADDRESS_1);
}
#endif

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

static int pcie_irq_handle(void *priv)
{
    struct plda_pcie *pcie = (struct plda_pcie *)priv;
    unsigned int reg_val = 0;

    reg_val = sys_readl(pcie->bridge_base + XR3PCI_ISTATUS_LOCAL);
//     printf("XR3PCI_ISTATUS_LOCAL = 0x%x\r\n", reg_val);
    sys_writel(reg_val, pcie->bridge_base + XR3PCI_ISTATUS_LOCAL);

	if (reg_val & INT_ERRORS)
		plda_pcie_handle_errors_irq(pcie, reg_val);

    reg_val = sys_readl(pcie->bridge_base + XR3PCI_ISTATUS_MSI);
    if (reg_val)
	hal_printf("msi interrupt\n");
	//     printf("XR3PCI_ISTATUS_MSI = 0x%x\r\n", reg_val);
    sys_writel(reg_val, pcie->bridge_base + XR3PCI_ISTATUS_MSI);

    return 0;
}

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

    pcie->irq_handle = pcie_irq_handle;

    return 0;
}


