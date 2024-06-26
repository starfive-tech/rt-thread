
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <hal_can.h>
#include "jh7110.h"
#include "gpio.h"
#include <riscv_io.h>

#ifdef BSP_USING_CAN
struct can_pin {
    char tx_pins;
    char rx_pins;
    char stb_pins;
    char tx_select;
    char stb_select;
    char rx_select;
};

struct uart_pin {
    char id;
    char tx_pins;
    char rx_pins;
    char cts_pin;
    char rts_pin;
    char tx_select;
    char rx_select;
    char cts_select;
    char rts_select;
};

static struct can_pin can_pins_array[] = {

    {
	.tx_pins = 25,
	.rx_pins = 26,
	.stb_pins = 28,
	.tx_select = 0x6,
	.stb_select = 0x3,
	.rx_select = 1,
    },

    {
	.tx_pins = 25,
	.rx_pins = 26,
	.stb_pins = 28,
	.tx_select = 0x32,
	.stb_select = 0x2f,
	.rx_select = 0x26,
    }
};

static struct uart_pin uart_pins_array[] = {
    {
	.id = 1,
	.tx_pins = 53,
	.rx_pins = 54,
	.cts_pin = 56,
	.rts_pin = 55,
	.tx_select = 68,
	.rx_select = 55,
	.cts_select = 54,
	.rts_select = 67,
    },
#if 0
    {
	.id = 3,
	.tx_pins = 32,
	.rx_pins = 34,
	.tx_select = 85,
	.rx_select = 68,
    },
#endif
};

static void can_set_pinctrl(int id)
{
    jh7110_set_gpiomux(can_pins_array[id].tx_pins, GPI_NONE,
	can_pins_array[id].tx_select, GPOEN_ENABLE);

    jh7110_set_gpiomux(can_pins_array[id].stb_pins, GPI_NONE,
	can_pins_array[id].stb_select, GPOEN_ENABLE);

    jh7110_set_gpiomux(can_pins_array[id].rx_pins,
	can_pins_array[id].rx_select,
	GPOUT_LOW, GPOEN_DISABLE);
}

void uart_set_pinctrl(int id)
{
    int i;

    for (i = 0; i < sizeof(uart_pins_array)/sizeof(uart_pins_array[0]); i++) {
	if (i == uart_pins_array[i].id) {
	    jh7110_set_gpiomux(uart_pins_array[id].tx_pins, GPI_NONE,
		uart_pins_array[id].tx_select, GPOEN_ENABLE);

	    jh7110_set_gpiomux(uart_pins_array[id].rx_pins,
			uart_pins_array[id].rx_select,
			GPOUT_LOW, GPOEN_DISABLE);

	    jh7110_set_gpiomux(uart_pins_array[id].rts_pin, GPI_NONE,
			uart_pins_array[id].rts_select, GPOEN_ENABLE);

	    jh7110_set_gpiomux(uart_pins_array[id].cts_pin,
			uart_pins_array[id].cts_select,
			GPOUT_LOW, GPOEN_DISABLE);
	}
    }
}

void can_set_board_config(struct hal_canfd *ipms)
{
    can_set_pinctrl(ipms->index);
    ipms->cfg.is_canfd = 0;
}
#endif
