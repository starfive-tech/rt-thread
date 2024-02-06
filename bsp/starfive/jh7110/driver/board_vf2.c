
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "drv_uart.h"
#include "encoding.h"
#include "sbi.h"

struct uart_config uart_config[] = {
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

#ifdef BSP_USING_RPMSG_LITE
void *get_rpmsg_mbox_base()
{
	return (void *)0x6e400000;
}
#endif

