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

/**
 * uart ns16550a
 * http://byterunner.com/16550.html
 */

/* TRANSMIT AND RECEIVE HOLDING REGISTER */
#define UART_RHR 0
#define UART_THR 0
#define UART_DLL 0	/* Out: Divisor Latch Low */

/* INTERRUPT ENABLE REGISTER */
#define UART_IER 1
#define UART_IER_RX_ENABLE (1 << 0)
#define UART_IER_TX_ENABLE (1 << 1)
#define UART_DLH 1	/* Out: Divisor Latch High */

/* FIFO CONTROL REGISTER */
#define UART_FCR 2
#define UART_FCR_FIFO_ENABLE (1 << 0)
#define UART_FCR_FIFO_CLEAR (3 << 1)

/* INTERRUPT STATUS REGISTER */
#define UART_ISR 2

/* LINE CONTROL REGISTER */
#define UART_LCR 3
#define UART_LCR_EIGHT_BITS (3 << 0)
// special mode to set baud rate
#define UART_LCR_BAUD_LATCH (1 << 7)

#define UART_MCR 4
/* LINE STATUS REGISTER */
#define UART_LSR 5
// input is waiting to be read from RHR
#define UART_LSR_RX_READY (1 << 0)
// THR can accept another character to send
#define UART_LSR_TX_IDLE (1 << 5)

#define UART_SCR 7
#define UART_DEFAULT_BAUDRATE 115200

struct uart_config {
    struct rt_serial_device serial;
    rt_uint32_t hw_base;
    void *remap_base;
    rt_uint32_t irqno;
    rt_uint32_t uart8250_in_freq;
    rt_uint32_t uart8250_baudrate;
    rt_uint32_t uart8250_reg_width;
    rt_uint32_t uart8250_reg_shift;
    rt_uint8_t index;
    rt_uint8_t control_uart;
    rt_uint8_t pinctrl;
};

void rt_hw_uart_start_rx_thread();
void drv_uart_puts(char *str); // for syscall
void rt_hw_uart_isr(int irqno, void *param);
int rt_hw_uart_init(void);
int rt_control_uart_init(void);
int get_uart_config_num();
struct uart_config *get_uart_config(int i);

#endif /* __DRV_UART_H__ */
