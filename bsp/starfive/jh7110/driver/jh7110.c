
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "jh7110.h"
#include "board.h"
#include <riscv_io.h>
#include "tick.h"
#include "gpio.h"
#include "drv_uart.h"
#ifdef BSP_USING_RPMSG_LITE
#include "rpmsg_platform.h"
#endif
#if defined(BSP_USING_GMAC)
#include "hal_gmac.h"
#endif
#if defined(BSP_USING_CAN)
#include "hal_can.h"
#endif
#if defined(BSP_USING_PCIE)
#include "hal_pcie.h"
#endif

static rt_uint64_t sys_crg_base;
static rt_uint64_t sys_syscon_base;
static rt_uint64_t aon_crg_base;
unsigned long sys_iomux_base;
unsigned long stg_crg_base;
unsigned long stg_syscon_base;

#if defined(BSP_USING_PCIE)
unsigned int STG_ARFUNC_OFFSET[2] = {0xc0, 0x270};
unsigned int STG_AWFUNC_OFFSET[2] = {0xc4, 0x274};
unsigned int STG_RP_REP_OFFSET[2] = {0x130, 0x2e0};
unsigned int STG_LIN_ST_OFFSET[2] = {0x1b8, 0x368};
#endif

#define ROUNDUP(a, b) ((((a)-1)/(b) +1)*(b))
#define CAN_DEAULT_RATE 40000000

struct jh7110_pll_regvals {
        rt_uint32_t dacpd;
        rt_uint32_t dsmpd;
        rt_uint32_t fbdiv;
        rt_uint32_t frac;
        rt_uint32_t postdiv1;
        rt_uint32_t prediv;
};

struct jh7110_pll_info {
        struct {
                unsigned int pd;
                unsigned int fbdiv;
                unsigned int frac;
                unsigned int prediv;
        } offsets;
        struct {
                rt_uint32_t dacpd;
                rt_uint32_t dsmpd;
                rt_uint32_t fbdiv;
        } masks;
        struct {
                char dacpd;
                char dsmpd;
                char fbdiv;
        } shifts;
};

#define JH7110_PLL(_idx)                              			\
        [_idx] = {                                                      \
                .offsets = {                                            \
                        .pd = JH7110_PLL##_idx##_PD_OFFSET,             \
                        .fbdiv = JH7110_PLL##_idx##_FBDIV_OFFSET,       \
                        .frac = JH7110_PLL##_idx##_FRAC_OFFSET,         \
                        .prediv = JH7110_PLL##_idx##_PREDIV_OFFSET,     \
                },                                                      \
                .masks = {                                              \
                        .dacpd = JH7110_PLL##_idx##_DACPD_MASK,         \
                        .dsmpd = JH7110_PLL##_idx##_DSMPD_MASK,         \
                        .fbdiv = JH7110_PLL##_idx##_FBDIV_MASK,         \
                },                                                      \
                .shifts = {                                             \
                        .dacpd = JH7110_PLL##_idx##_DACPD_SHIFT,        \
                        .dsmpd = JH7110_PLL##_idx##_DSMPD_SHIFT,        \
                        .fbdiv = JH7110_PLL##_idx##_FBDIV_SHIFT,        \
                },                                                      \
        }

static const struct jh7110_pll_info jh7110_plls[] = {
        JH7110_PLL(0),
        JH7110_PLL(1),
        JH7110_PLL(2),
};

#ifdef BSP_USING_GMAC1
static void gmac1_plat_init()
{
	/* clk init */
	sys_setbits(sys_crg_base + GMAC1_CLK_AHB, BIT(31));
	sys_setbits(sys_crg_base + GMAC1_CLK_AXI, BIT(31));
	sys_setbits(sys_crg_base + GMAC1_CLK_PTP, BIT(31));
	sys_setbits(sys_crg_base + GMAC1_CLK_GTXC, BIT(31));
	sys_setbits(sys_crg_base + GMAC1_CLK_TX, BIT(31));
	sys_clrbits(sys_crg_base + SYS_CRG_RESET2, GMAC_CLK_AXI_RST);
	sys_clrbits(sys_crg_base + SYS_CRG_RESET2, GMAC_CLK_AHB_RST);
}
#endif

static void jh7110_syscon_init()
{
#ifdef RT_USING_SMART
	sys_crg_base = rt_ioremap(SYS_CRG_BASE, 4096);
	sys_syscon_base = rt_ioremap(SYS_SYSCON_BASE, 4096);
	aon_crg_base = rt_ioremap(AON_CRG_BASE, 4096);
	sys_iomux_base = rt_ioremap(SYS_IOMUX_BASE, 4096);
	stg_crg_base = rt_ioremap(STG_CRG_BASE, 4096);
	stg_syscon_base = rt_ioremap(STG_SYSCON_BASE, 4096);
#else
	sys_crg_base = SYS_CRG_BASE;
	sys_syscon_base = SYS_SYSCON_BASE;
	aon_crg_base = AON_CRG_BASE;
	sys_iomux_base = SYS_IOMUX_BASE;
	stg_crg_base = STG_CRG_BASE;
	stg_syscon_base = STG_SYSCON_BASE;
#endif
}

static void jh7110_pll_regvals_get(const struct jh7110_pll_info *info,
                                   struct jh7110_pll_regvals *ret)
{
        rt_uint32_t val;

        val = sys_readl(sys_syscon_base + info->offsets.pd);
        ret->dacpd = (val & info->masks.dacpd) >> info->shifts.dacpd;
        ret->dsmpd = (val & info->masks.dsmpd) >> info->shifts.dsmpd;

        val = sys_readl(sys_syscon_base + info->offsets.fbdiv);
        ret->fbdiv = (val & info->masks.fbdiv) >> info->shifts.fbdiv;

        val = sys_readl(sys_syscon_base + info->offsets.frac);
        ret->frac = (val & JH7110_PLL_FRAC_MASK) >> JH7110_PLL_FRAC_SHIFT;
        ret->postdiv1 = (val & JH7110_PLL_POSTDIV1_MASK) >> JH7110_PLL_POSTDIV1_SHIFT;

        val = sys_readl(sys_syscon_base + info->offsets.prediv);
        ret->prediv = (val & JH7110_PLL_PREDIV_MASK) >> JH7110_PLL_PREDIV_SHIFT;
}

unsigned long jh7110_pll_get_rate(int index)
{
        unsigned long rate, parent_rate = JH7110_PLL_OSC_RATE;
	struct jh7110_pll_regvals val;

        jh7110_pll_regvals_get(&jh7110_plls[index], &val);

        /*
         * dacpd = dsmpd = 0: fraction mode
         * dacpd = dsmpd = 1: integer mode, frac value ignored
         *
         * rate = parent * (fbdiv + frac/2^24) / prediv / 2^postdiv1
         *      = (parent * fbdiv + parent * frac / 2^24) / (prediv * 2^postdiv1)
         */
        if (val.dacpd == 0 && val.dsmpd == 0)
                rate = parent_rate * val.frac / (1UL << 24);
        else if (val.dacpd == 1 && val.dsmpd == 1)
                rate = 0;
        else
                return 0;

        rate += parent_rate * val.fbdiv;
        rate /= val.prediv << val.postdiv1;

        return rate;
}

unsigned long get_bus_root_rate(void)
{
	unsigned long rate = jh7110_pll_get_rate(2);
	int div;

	div = sys_readl(sys_crg_base + PERI_ROOT) >> 24;
	return rate / div;
}

unsigned long get_axi_cfg_rate(void)
{
	unsigned long rate = get_bus_root_rate();
	rt_uint32_t div;

	div = sys_readl(sys_crg_base + AXI_CFG) & GENMASK(23, 0);
	return rate / div;
}

unsigned long get_stg_axi_ahb_rate(void)
{
	unsigned long rate = get_axi_cfg_rate();
	rt_uint32_t div;

	div = sys_readl(sys_crg_base + STG_AXI_AHB) & GENMASK(23, 0);
	return rate / div;
}

unsigned long get_peri_root_rate(void)
{
	unsigned long rate = get_bus_root_rate();
	int div;

	div = sys_readl(sys_crg_base + PERI_ROOT) & GENMASK(23, 0);
	return rate / div;
}

unsigned long get_iomux_base(void)
{
	return sys_iomux_base;
}

unsigned long sys_cur_time_ms(void)
{
	return get_ticks() / (CPUTIME_TIMER_FREQ / 1000);
}

void sys_udelay(int us)
{
   rt_uint64_t us_cnt;

   us_cnt = CPUTIME_TIMER_FREQ / 1000000 * us;
   us_cnt = get_ticks() + us_cnt;
   while (get_ticks() < us_cnt);
}

void sys_mdelay(int ms)
{
   rt_uint64_t ms_cnt;

   ms_cnt = CPUTIME_TIMER_FREQ / 1000 * ms;
   ms_cnt = get_ticks() + ms_cnt;
   while (get_ticks() < ms_cnt);
}

void sys_tick_sleep(unsigned int tick)
{
    rt_thread_delay(tick);
}

unsigned int sys_gmac_get_csr_clk(int id)
{
    return get_stg_axi_ahb_rate();
}

#if defined(BSP_USING_GMAC)
static struct gmac_config gmac_config[] = {
    {
	.speed_mode = GMAC_PHY_SPEED_1000M,
	.speed = 1000,
	.phy_addr = 0,
	.irq = 7, /* mac irq */
	.duplex = GMAC_PHY_FULL_DUPLEX,
    },
    {
	.speed_mode = GMAC_PHY_SPEED_1000M,
	.speed = 1000,
	.phy_addr = 0,
	.irq = 78, /* mac irq */
	.duplex = GMAC_PHY_FULL_DUPLEX,
    },
};

void gmac_plat_init(gmac_handle_t *gmac)
{
    if (gmac->id == 0) {
#ifdef RT_USING_SMART
	gmac->base = rt_ioremap(0x16030000, 0x4000);
#else
	gmac->base = (void *)0x16030000;
#endif
	rt_memcpy(&gmac->gmac_config, &gmac_config[0], sizeof(struct gmac_config));
    } else {
#ifdef RT_USING_SMART
	gmac->base = rt_ioremap(0x16040000, 0x4000);
#else
	gmac->base = (void *)0x16040000;
#endif
	rt_memcpy(&gmac->gmac_config, &gmac_config[1], sizeof(struct gmac_config));
    }
    /* todo  get mac addr from share ram gmac 1*/
    gmac_set_board_config(gmac);

#if defined(BSP_USING_GMAC1)
    gmac1_plat_init();
#endif
}
#endif

#if defined(BSP_USING_CAN)
void can_plat_init(struct ipms_canfd *ipms)
{
    unsigned int root_rate = get_peri_root_rate();
    unsigned int div;
    int can_rate = CAN_DEAULT_RATE;

    ipms->cfg.clk_freq = can_rate;

    root_rate = ROUNDUP(root_rate, can_rate);
    div = root_rate / can_rate;

    can_set_board_config(ipms);
    if (ipms->index == 0) {
#ifdef RT_USING_SMART
	ipms->base = rt_ioremap(0x130d0000, 0x4000);
#else
	ipms->base  = (void *)0x130d0000;
#endif
	ipms->irq = 112;
	sys_clrsetbits(sys_crg_base + SYS_CRG_RESET3,
			    GENMASK(17, 15), 0);
	sys_setbits(sys_crg_base + CAN0_CTRL_CLK_APB, BIT(31));
	sys_setbits(sys_crg_base + CAN0_CTRL_CLK_TIMER, BIT(31));
	sys_setbits(sys_crg_base + CAN0_CTRL_CLK_CORE, BIT(31) | div);
	if (ipms->cfg.is_canfd) {
	    sys_setbits(sys_syscon_base + CAN0_FD_OFFSET, BIT(3));
	}
    } else {
#ifdef RT_USING_SMART
	ipms->base = rt_ioremap(0x130e0000, 0x4000);
#else
	ipms->base = (void *)0x130e0000;
#endif
	ipms->irq = 113;
	sys_clrsetbits(sys_crg_base + SYS_CRG_RESET3,
			    GENMASK(20, 18), 0);
	sys_setbits(sys_crg_base + CAN1_CTRL_CLK_APB, BIT(31));
	sys_setbits(sys_crg_base + CAN1_CTRL_CLK_TIMER, BIT(31));
	sys_setbits(sys_crg_base + CAN1_CTRL_CLK_CORE, BIT(31) | div);
	if (ipms->cfg.is_canfd) {
	    sys_setbits(sys_syscon_base + CAN1_FD_OFFSET, BIT(18));
	}
    }
}
#endif

void *get_ipi_handler()
{
#ifdef BSP_USING_RPMSG_LITE
    return (void *)rpmsg_handler;
#else
    return NULL;
#endif
}

static void jh7110_uart_init()
{
    struct uart_config *conf;
    int i;

    /* assert uart 0 as linux uart */
    for (i = 0; i < get_uart_config_num(); i++) {
	conf = get_uart_config(i);
	sys_setbits(sys_crg_base + CLK_UART0_APB_OFFSET + conf->index * 0x8,
			BIT(31));
	sys_setbits(sys_crg_base + CLK_UART0_CORE_OFFSET + conf->index * 0x8,
			BIT(31));
	sys_clrsetbits(sys_crg_base + SYS_CRG_RESET2,
			    BIT(19 + conf->index *2) | BIT(20 + conf->index *2), 0);
	if (conf->pinctrl)
		uart_set_pinctrl(conf->index);
   }
   rt_hw_uart_init();
}

void uart_config_fixup(int id)
{
    struct uart_config *conf;
    int root_rate;
    int div;

    conf = get_uart_config(id);

    if (conf->index < 3)
	return;

    root_rate = get_peri_root_rate();

    if (conf->control_uart) {
	while (!env_is_ready());
    }

    div = sys_readl(sys_crg_base + CLK_UART3_CORE_OFFSET + (conf->index - 3) * 0x8);
    div = (div >> 8) & 0xffff;
    conf->uart8250_in_freq = root_rate / div;
}

static void jh7110_env_init(void)
{
    jh7110_syscon_init();
    jh7110_uart_init();
}

#if defined(BSP_USING_PCIE)
static int starfive_pcie_link_up(struct pcie *pcie)
{
#define DATA_LINK_ACTIVE			BIT(5)
	int ret;
	unsigned int stg_reg_val;

	stg_reg_val = sys_readl(stg_syscon_base + STG_LIN_ST_OFFSET[pcie->index]);

	return (stg_reg_val & DATA_LINK_ACTIVE);
}

static int pcie_post_init(struct pcie *pcie)
{

    int i, retries;
    /*
     * Ensure that PERST has been asserted for at least 100 ms,
    * the sleep value is T_PVPERL from PCIe CEM spec r2.0 (Table 2-4)
    */

    sys_mdelay(100);

    jh7110_gpio_direction_output(pcie->cfg.reset_gpio, 1);

   /*
    * With a Downstream Port (<=5GT/s), software must wait a minimum
    * of 100ms following exit from a conventional reset before
    * sending a configuration request to the device.
    */
    sys_mdelay(100);

    /* Check if the link is up or not */
    for (retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
	    if (starfive_pcie_link_up(pcie)) {
		    hal_printf("pcie port link up\n");
		    pcie->link_up = 1;
		    return 0;
	    }
	    sys_udelay(LINK_WAIT_USLEEP_MIN);
    }

    pcie->link_up = 0;

    hal_printf("port link down\n");

    return 0;
}

void pcie_plat_init(struct pcie *pcie)
{
#define PLDA_FUNC_NUM 4
#define PCI_MISC 0xb4
    int i;

    sys_setbits(sys_crg_base + NOC_BUS_STG_AXI, BIT(31));

    sys_clrsetbits(stg_syscon_base + STG_RP_REP_OFFSET[pcie->index],
		STG_SYSCON_K_RP_NEP_MASK,
		STG_SYSCON_K_RP_NEP_MASK);
    sys_clrsetbits(stg_syscon_base + STG_AWFUNC_OFFSET[pcie->index],
		STG_SYSCON_CKREF_SRC_MASK,
		2 << STG_SYSCON_CKREF_SRC_SHIFT);
    sys_clrsetbits(stg_syscon_base + STG_AWFUNC_OFFSET[pcie->index],
		STG_SYSCON_CLKREQ_MASK,
		STG_SYSCON_CLKREQ_MASK);

    pcie->cfg.reset_gpio = get_pcie_reset_gpio(pcie->index);
    if (pcie->index == 0) {
	/* todo RT_USING_SMART */
	//pcie->cfg.bridge_base = rt_ioremap(0x2b000000, 0x10000);
	pcie->cfg.bridge_base  = (void *)0x2b000000;
	pcie->cfg.cfg_base = (void *)0x940000000;
	pcie->cfg.mem32_base = 0x30000000;
	pcie->cfg.mem64_base = 0x900000000;

	pcie->irq = 56;

	sys_clrsetbits(stg_crg_base + STG_PCIE_RESET_OFFSET,
			    GENMASK(16, 11), 0);

	sys_setbits(stg_crg_base + JH7110_STGCLK_PCIE0_AXI_MST0, BIT(31));
	sys_setbits(stg_crg_base + JH7110_STGCLK_PCIE0_APB, BIT(31));
	sys_setbits(stg_crg_base + JH7110_STGCLK_PCIE0_TL, BIT(31));
    } else {
	/* todo RT_USING_SMART */
	//pcie->bridge_base = rt_ioremap(0x2c000000, 0x10000);
	pcie->cfg.bridge_base = (void *)0x2c000000;
	pcie->cfg.cfg_base = (void *)0x9c0000000;
	pcie->cfg.mem32_base = 0x38000000;
	pcie->cfg.mem64_base = 0x980000000;

	pcie->irq = 57;
	sys_clrsetbits(stg_crg_base + STG_PCIE_RESET_OFFSET,
			    GENMASK(22, 17), 0);
	sys_setbits(stg_crg_base + JH7110_STGCLK_PCIE1_AXI_MST0, BIT(31));
	sys_setbits(stg_crg_base + JH7110_STGCLK_PCIE1_APB, BIT(31));
	sys_setbits(stg_crg_base + JH7110_STGCLK_PCIE1_TL, BIT(31));
    }
    pcie->cfg.mem64_log = 30;
    pcie->cfg.mem32_log = 27;
    jh7110_gpio_direction_output(pcie->cfg.reset_gpio, 0);

    /* Disable physical functions except #0 */
    for (i = 1; i < PLDA_FUNC_NUM; i++) {
	    sys_clrsetbits(stg_syscon_base + STG_ARFUNC_OFFSET[pcie->index],
		    STG_SYSCON_AXI4_SLVL_ARFUNC_MASK,
		    (i << PLDA_PHY_FUNC_SHIFT) <<
		    STG_SYSCON_AXI4_SLVL_ARFUNC_SHIFT);
	    sys_clrsetbits(stg_syscon_base + STG_AWFUNC_OFFSET[pcie->index],
		    STG_SYSCON_AXI4_SLVL_AWFUNC_MASK,
		    (i << PLDA_PHY_FUNC_SHIFT) <<
		    STG_SYSCON_AXI4_SLVL_AWFUNC_SHIFT);
	    sys_setbits(pcie->cfg.bridge_base + PCI_MISC, BIT(15));
    }

    sys_clrsetbits(stg_syscon_base + STG_ARFUNC_OFFSET[pcie->index],
		    STG_SYSCON_AXI4_SLVL_ARFUNC_MASK, 0);
    sys_clrsetbits(stg_syscon_base + STG_AWFUNC_OFFSET[pcie->index],
		    STG_SYSCON_AXI4_SLVL_AWFUNC_MASK, 0);

    pcie->cfg.pcie_post_init_cb = pcie_post_init;
}
#endif

void jh7110_driver_init(void)
{
    while (!env_is_ready()) {
	rt_schedule();
    }

    jh7110_env_init();
    hw_pin_init();
#if defined(BSP_USING_PCIE)
    rt_hw_pcie_init();
#endif
#if defined(BSP_USING_GMAC)
    rt_hw_gmac_init();
#endif
#if defined(BSP_USING_CAN)
    rt_hw_canfd_init();
#endif
}
