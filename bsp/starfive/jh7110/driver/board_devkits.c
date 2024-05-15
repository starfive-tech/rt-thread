
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <hal_can.h>
#include "jh7110.h"
#include "gpio.h"
#include <riscv_io.h>

#ifdef BSP_USING_CAN
struct can_pin {
    int tx_pins;
    int rx_pins;
    int stb_pins;
    int tx_select;
    int stb_select;
    int rx_select;
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

static void can_set_pinctrl(int id)
{
    jh7110_set_gpiomux(can_pins_array[id].tx_pins, GPI_NONE,
	can_pins_array[id].tx_select, GPOEN_ENABLE);

    jh7110_set_gpiomux(can_pins_array[id].stb_pins, GPI_NONE,
	can_pins_array[id].stb_select, GPOEN_ENABLE);

    jh7110_set_gpiomux(can_pins_array[id].rx_pins, can_pins_array[id].rx_select,
	GPOUT_LOW, GPOEN_DISABLE);
}

void can_set_board_config(struct ipms_canfd *ipms)
{
    can_set_pinctrl(ipms->index);
    ipms->cfg.is_canfd = 0;
}
#endif
