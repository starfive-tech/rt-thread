
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "drv_uart.h"
#include "encoding.h"
#include "sbi.h"
#if defined(BSP_USING_GMAC)
#include "hal_gmac.h"
#endif

static struct uart_config uart_config[] = {
    {
        .hw_base = 0x10010000,
	.remap_base = (void *)0x10010000,
	.irqno = 33,
        .uart8250_in_freq = 24000000,
        .uart8250_baudrate = 115200,
        .uart8250_reg_width = 4,
        .uart8250_reg_shift = 2,
        .index = 1,
        .control_uart = 1,
    },
};

#if defined(BSP_USING_GMAC)
static struct phy_dts_config phy_dts[] = {
    {
	.rgmii_sw_dr_2 = 0x0,
	.rgmii_sw_dr = 0x3,
	.rgmii_sw_dr_rxc = 0x6,
	.tx_delay_sel_fe = 5,
	.tx_delay_sel = 0xa,
	.rxc_dly_en = 0,
	.rx_delay_sel = 0xa,
	.tx_inverted_10 = 0x1,
	.tx_inverted_100 = 0x1,
	.tx_inverted_1000 = 0x1,
    },
    {
	.rgmii_sw_dr_2 = 0x0,
	.rgmii_sw_dr = 0x3,
	.rgmii_sw_dr_rxc = 0x6,
	.tx_delay_sel_fe = 5,
	.tx_delay_sel = 0x0,
	.rxc_dly_en = 0,
	.rx_delay_sel = 0x2,
	.tx_inverted_10 = 0x1,
	.tx_inverted_100 = 0x1,
	.tx_inverted_1000 = 0,
    }
};

void gmac_set_board_config(gmac_handle_t *gmac)
{
    rt_memcpy(&gmac->phy_config, &phy_dts[gmac->id], sizeof(struct phy_dts_config));
}
#endif
/*
 * UART Initiation
 */
int get_uart_config_num()
{
	return sizeof(uart_config)/sizeof(uart_config[0]);
}

struct uart_config *get_uart_config(int i)
{
	return &uart_config[i];
}

void *get_rpmsg_sharemem_base()
{
	return (void *)0x6e400000;
}

#ifdef BSP_USING_RPMSG_LITE
void *get_rpmsg_mbox_base()
{
	return (void *)0x6e401000;
}
#endif

