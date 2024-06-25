
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "drv_uart.h"
#include "encoding.h"
#include "sbi.h"
#if defined(BSP_USING_GMAC)
#include "hal_gmac.h"
#endif

static struct uart_8250_data uart_config[] = {
#ifdef BSP_USING_DEVKITS
    {
        .hw_base = 0x10010000,
	.remap_base = (void *)0x10010000,
	.irqno = 33,
        .uart8250_in_freq = 24000000,
        .uart8250_baudrate = 115200,
        .uart8250_reg_width = 4,
        .uart8250_reg_shift = 2,
        .index = 1,
        .control_uart = 0,
        .pinctrl = 1,
    },
#endif
    {
        .hw_base = 0x10020000,
	.remap_base = (void *)0x10020000,
	.irqno = 34,
        .uart8250_in_freq = 24000000,
        .uart8250_baudrate = 115200,
        .uart8250_reg_width = 4,
        .uart8250_reg_shift = 2,
        .index = 2,
        .control_uart = 1,
        .pinctrl = 0,
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
	.disable_llp = 0,
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
	.disable_llp = 0,
    }
};

void gmac_set_board_config(gmac_handle_t *gmac)
{
    memcpy(&gmac->phy_config, &phy_dts[gmac->id], sizeof(struct phy_dts_config));
}
#endif
/*
 * UART Initiation
 */
int get_uart_config_num()
{
	return sizeof(uart_config)/sizeof(uart_config[0]);
}

struct uart_8250_data *get_uart_config(int i)
{
	return &uart_config[i];
}

#ifndef BSP_USING_DEVKITS
void uart_set_pinctrl(int id)
{
}
#endif

#if defined(BSP_USING_PCIE)
int get_pcie_reset_gpio(int i)
{
	if (i == 0)
		return 26;
	else
		return 28;
}
#endif

void *get_rpmsg_sharemem_base()
{
	return (void *)0x6e400000;
}

unsigned long get_heap_base(void)
{
	return 0x6f000000;
}

unsigned long get_heap_size(void)
{
	return 16 * 1024 * 1024;
}

#ifdef BSP_USING_RPMSG_LITE
void *get_rpmsg_mbox_base()
{
	return (void *)0x6e402000;
}

void *get_rpmsg_base(void)
{
	return (void *)0x6e404000;
}
#endif

