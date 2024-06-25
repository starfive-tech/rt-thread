
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <riscv_io.h>

#include "board.h"
#include "drv_uart.h"

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

static uint32_t get_reg(struct uart_8250_data *config, uint32_t num)
{
	uint32_t offset = num << config->uart8250_reg_shift;

	if (config->uart8250_reg_width == 1)
		return sys_readb(config->remap_base + offset);
	else if (config->uart8250_reg_width == 2)
		return sys_readw(config->remap_base + offset);
	else
		return sys_readl(config->remap_base + offset);
}

static void set_reg(struct uart_8250_data *config, uint32_t num, uint32_t val)
{
	uint32_t offset = num << config->uart8250_reg_shift;

	if (config->uart8250_reg_width == 1)
		sys_writeb(val, config->remap_base + offset);
	else if (config->uart8250_reg_width == 2)
		sys_writew(val, config->remap_base + offset);
	else
		sys_writel(val, config->remap_base + offset);
}

static int uart_8250_control(void *priv, int cmd)
{
    struct uart_8250_data *config = (void *)priv;
    uint8_t value;

    switch (cmd)
    {
    case UART_CLR_INT:
        value = get_reg(config, UART_IER);
        set_reg(config, UART_IER, value & ~UART_IER_RX_ENABLE);
        break;

    case UART_SET_INIT:
        value = get_reg(config, UART_IER);
        set_reg(config, UART_IER, value | UART_IER_RX_ENABLE);
        break;
    }

    return 0;
}

static int uart_8250_putc(void *priv, char c)
{
    struct uart_8250_data *config = (void *)priv;
    // wait for Transmit Holding Empty to be set in LSR.
    while((get_reg(config, UART_LSR) & UART_LSR_TX_IDLE) == 0)
        ;
    set_reg(config, UART_THR, c);

    return (1);
}

static int uart_8250_getc(void *priv)
{
    struct uart_8250_data *config = (void *)priv;

    volatile uint32_t lsr;
    int ch = -1;

    lsr = get_reg(config, UART_LSR);

    if (lsr & UART_LSR_RX_READY)
    {
        ch = get_reg(config, UART_RHR);
    }
    return ch;
}

void uart_8250_configure(void *priv)
{
    struct uart_8250_data *config = (void *)priv;
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

struct uart_ops uart_8250_ops = {
    .configure =  uart_8250_configure,
    .control =  uart_8250_control,
    .getc    = uart_8250_getc,
    .putc    = uart_8250_putc,
};

void uart_8250_set_ops(struct uart_config *config)
{
     config->ops = &uart_8250_ops;
}

