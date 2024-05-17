
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

static rt_uint64_t sys_crg_base;
static rt_uint64_t sys_syscon_base;
static rt_uint64_t aon_crg_base;
unsigned long sys_iomux_base;

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
#else
	sys_crg_base = SYS_CRG_BASE;
	sys_syscon_base = SYS_SYSCON_BASE;
	aon_crg_base = AON_CRG_BASE;
	sys_iomux_base = SYS_IOMUX_BASE;
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
	.speed = GMAC_PHY_SPEED_1000M,
	.phy_addr = 0,
	.irq = 7, /* mac irq */
	.duplex = GMAC_PHY_FULL_DUPLEX,
    },
    {
	.speed = GMAC_PHY_SPEED_100M,
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
}
#endif

#if defined(BSP_USING_CAN)
void can_plat_init(struct ipms_canfd *ipms)
{
    unsigned int root_rate = get_peri_root_rate();
    unsigned int div;
    int can_rate = CAN_DEAULT_RATE;

    ipms->cfg.clk_freq = can_rate;

    root_rate = ROUNDUP(root_rate, div);
    div = root_rate / can_rate;

    can_set_board_config(ipms);
    if (ipms->index == 0) {
#ifdef RT_USING_SMART
	ipms->base = rt_ioremap(0x130d0000, 0x4000);
#else
	ipms->base  = (void *)0x130d0000;
#endif
	ipms->irq = 112;
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

    for (i = 0; i < get_uart_config_num(); i++) {
	conf = get_uart_config(i);
	if (conf->index == 1) {
		sys_setbits(sys_crg_base + CLK_UART1_APB_OFFSET, BIT(31));
		sys_setbits(sys_crg_base + CLK_UART1_CORE_OFFSET, BIT(31));
		sys_clrsetbits(sys_crg_base + SYS_CRG_RESET2,
			    BIT(21) | BIT(22), 0);
	}
	else if (conf->index == 2) {
	    sys_setbits(sys_crg_base + CLK_UART2_APB_OFFSET, BIT(31));
	    sys_setbits(sys_crg_base + CLK_UART2_CORE_OFFSET, BIT(31));
	    sys_clrsetbits(sys_crg_base + SYS_CRG_RESET2,
			BIT(23) | BIT(24), 0);
	}
	if (conf->pinctrl)
		uart_set_pinctrl(i);
   }
}

static void jh7110_env_init(void)
{
    jh7110_syscon_init();
    jh7110_uart_init();
}

void jh7110_driver_init(void)
{
    while (!env_is_ready()) {
	rt_schedule();
    }

    jh7110_env_init();
    hw_pin_init();
#if defined(BSP_USING_GMAC)
#if defined(BSP_USING_GMAC1)
    gmac1_plat_init();
#endif
    rt_hw_gmac_init();
#endif
#if defined(BSP_USING_CAN)
    rt_hw_canfd_init();
#endif
}
