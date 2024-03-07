/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>

#include "board.h"
#include "drv_uart.h"

#include <stdio.h>
#include "sbi.h"

static rt_uint32_t get_reg(struct uart_config *config, rt_uint32_t num)
{
	rt_uint32_t offset = num << config->uart8250_reg_shift;

	if (config->uart8250_reg_width == 1)
		return __raw_readb(config->remap_base + offset);
	else if (config->uart8250_reg_width == 2)
		return __raw_readw(config->remap_base + offset);
	else
		return __raw_readl(config->remap_base + offset);
}

static void set_reg(struct uart_config *config, rt_uint32_t num, rt_uint32_t val)
{
	rt_uint32_t offset = num << config->uart8250_reg_shift;

	if (config->uart8250_reg_width == 1)
		__raw_writeb(val, config->remap_base + offset);
	else if (config->uart8250_reg_width == 2)
		__raw_writew(val, config->remap_base + offset);
	else
		__raw_writel(val, config->remap_base + offset);
}

void uart_init(struct uart_config *config)
{
    int bdiv;

    bdiv = (config->uart8250_in_freq + 8 * config->uart8250_baudrate) /
	(16 * config->uart8250_baudrate);

    /* Disable all interrupts */
    set_reg(config, UART_IER, 0x00);
    /* Enable DLAB */
    set_reg(config, UART_LCR, 0x80);
    
    if (bdiv) {
	    /* Set divisor low byte */
	    set_reg(config, UART_DLL, bdiv & 0xff);
	    /* Set divisor high byte */
	    set_reg(config, UART_DLH, (bdiv >> 8) & 0xff);
    }

    /* 8 bits, no parity, one stop bit */
    set_reg(config, UART_LCR, 0x03);
    /* Enable FIFO */
    set_reg(config, UART_FCR, 0x01);
    /* No modem control DTR RTS */
    set_reg(config, UART_MCR, 0x00);
    /* Clear line status */
    get_reg(config, UART_LSR);
    /* Read receive buffer */
    get_reg(config, UART_RHR);
    /* Set scratchpad */
    set_reg(config, UART_SCR, 0x00);

    return;
}

static rt_err_t _uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct uart_config *config
        = rt_container_of(serial, struct uart_config, serial);

    uart_init(config);
    return (RT_EOK);
}

static rt_err_t _uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
   struct uart_config *config
	    = rt_container_of(serial, struct uart_config, serial);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        if ((size_t)arg == RT_DEVICE_FLAG_INT_RX)
        {
            rt_uint8_t value = get_reg(config, UART_IER);
            set_reg(config, UART_IER, value & ~UART_IER_RX_ENABLE);
        }
        break;

    case RT_DEVICE_CTRL_SET_INT:
        if ((size_t)arg == RT_DEVICE_FLAG_INT_RX)
        {
            rt_uint8_t value = get_reg(config, UART_IER);
            set_reg(config, UART_IER, value | UART_IER_RX_ENABLE);
        }
        break;
    }

    return (RT_EOK);
}

static int _uart_putc(struct rt_serial_device *serial, char c)
{
    struct uart_config *config
	= rt_container_of(serial, struct uart_config, serial);

    // wait for Transmit Holding Empty to be set in LSR.
    while((get_reg(config, UART_LSR) & UART_LSR_TX_IDLE) == 0)
        ;
    set_reg(config, UART_THR, c);

    return (1);
}

static int _uart_getc(struct rt_serial_device *serial)
{
    struct uart_config *config
        = rt_container_of(serial, struct uart_config, serial);
    volatile rt_uint32_t lsr;
    int ch = -1;

    lsr = get_reg(config, UART_LSR);

    if (lsr & UART_LSR_RX_READY)
    {
        ch = get_reg(config, UART_RHR);
    }
    return ch;
}

const struct rt_uart_ops _uart_ops = {
    _uart_configure,
    _uart_control,
    _uart_putc,
    _uart_getc,
    // TODO: add DMA support
    RT_NULL};

void rt_hw_uart_isr(int irqno, void *param)
{
    rt_ubase_t level = rt_hw_interrupt_disable();

    struct rt_serial_device *serial = (struct rt_serial_device *)param;

    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);

    rt_hw_interrupt_enable(level);
}

int rt_hw_uart_init(void)
{
    struct rt_serial_device *serial;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    char console_name[] = "uart0";
    struct uart_config *uart_config;
    int i, uart_num;

    for (i = 0; i < get_uart_config_num(); i++)
    {
        uart_config = get_uart_config(i);
#ifdef RT_USING_SMART
	uart_config->remap_base = rt_ioremap(uart_config->hw_base, 4096);
#endif
	console_name[4] = uart_config->index + '0';
	serial = &uart_config->serial;
        serial->ops = &_uart_ops;
        serial->config = config;
	serial->config.baud_rate = uart_config->uart8250_baudrate;
	rt_hw_serial_register(serial,
			      RT_CONSOLE_DEVICE_NAME,
			      RT_DEVICE_FLAG_STREAM | RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
			      uart_config);
	rt_hw_interrupt_install(uart_config->irqno, rt_hw_uart_isr, uart_config, console_name);
	rt_hw_interrupt_umask(uart_config->irqno);
    }

    // register device

    return 0;
}

/* WEAK for SDK 0.5.6 */
rt_weak void uart_debug_init(int uart_channel)
{
}
