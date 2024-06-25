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
#include <riscv_io.h>

#include "board.h"
#include "drv_uart.h"

#include <stdio.h>
#include "sbi.h"

struct uart_config uart_config[6];

static rt_err_t _uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct uart_config *config
        = rt_container_of(serial, struct uart_config, serial);

    config->ops->configure(config->priv);
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
	    config->ops->control(config->priv, UART_CLR_INT);
        break;

    case RT_DEVICE_CTRL_SET_INT:
        if ((size_t)arg == RT_DEVICE_FLAG_INT_RX)
	    config->ops->control(config->priv, UART_SET_INIT);
        break;
    }

    return (RT_EOK);
}

static int _uart_putc(struct rt_serial_device *serial, char c)
{
    struct uart_config *config
	= rt_container_of(serial, struct uart_config, serial);

    config->ops->putc(config->priv, c);
    return (1);
}

static int _uart_getc(struct rt_serial_device *serial)
{
    struct uart_config *config
        = rt_container_of(serial, struct uart_config, serial);

    return config->ops->getc(config->priv);
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

static void rt_register_uart(struct uart_config *uart,
		const char *console_name)
{
    struct rt_serial_device *serial;
    struct uart_8250_data *data = uart->priv;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef RT_USING_SMART
    data->remap_base = rt_ioremap(data->hw_base, 4096);
#endif
    serial = &uart->serial;
    serial->ops = &_uart_ops;
    serial->config = config;
    serial->config.baud_rate = data->uart8250_baudrate;
    rt_hw_serial_register(serial,
			console_name,
			RT_DEVICE_FLAG_STREAM | RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
			uart);
    rt_hw_interrupt_install(data->irqno, rt_hw_uart_isr, uart, console_name);
    rt_hw_interrupt_umask(data->irqno);
}

int rt_hw_uart_init(void)
{
    char console_name[] = "uart0";
    struct uart_config *uart;
    struct uart_8250_data *data;
    int i;

    for (i = 0; i < get_uart_config_num(); i++)
    {
        uart = &uart_config[i];
        data = get_uart_config(i);
        if (data->control_uart)
	    continue;
	uart_config_fixup(i);
	uart_8250_set_ops(uart);
	uart->priv = data;
	console_name[4] = data->index + '0';
	rt_register_uart(uart, console_name);
    }
    return 0;
}

int rt_control_uart_init(void)
{
    struct uart_config *uart;
    struct uart_8250_data *data;
    int i;

    for (i = 0; i < get_uart_config_num(); i++)
    {
        uart = &uart_config[i];
        data = get_uart_config(i);
	if (data->control_uart) {
	    uart_config_fixup(i);
	    uart_8250_set_ops(uart);
	    uart->priv = data;
	    rt_register_uart(uart, RT_CONSOLE_DEVICE_NAME);
	    break;
	}
    }

    return 0;
}

/* WEAK for SDK 0.5.6 */
rt_weak void uart_debug_init(int uart_channel)
{
}
