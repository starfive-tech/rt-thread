/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#ifndef __DRV_UART_H__
#define __DRV_UART_H__

#include "riscv_io.h"

enum uart_cmd {
    UART_CLR_INT,
    UART_SET_INIT,
};

struct uart_config {
    struct rt_serial_device serial;
    struct uart_ops *ops;
    void *priv;
};

struct uart_8250_data {
    uint32_t hw_base;
    void *remap_base;
    uint32_t irqno;
    uint32_t uart8250_in_freq;
    uint32_t uart8250_baudrate;
    uint32_t uart8250_reg_width;
    uint32_t uart8250_reg_shift;
    uint8_t index;
    uint8_t control_uart;
    uint8_t pinctrl;
};

struct uart_ops {
    int (*control)(void *config, int cmd);
    int (*putc)(void *config, char c);
    int (*getc)(void *config);
    void (*configure)(void *config);
};

void rt_hw_uart_start_rx_thread(void);
int rt_hw_uart_init(void);
int rt_control_uart_init(void);
int get_uart_config_num();
struct uart_8250_data *get_uart_config(int i);
void uart_8250_set_ops(struct uart_config *config);

#endif /* __DRV_UART_H__ */
