
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <hal_can.h>
#include "jh7110.h"
#include <riscv_io.h>

#ifdef BSP_USING_CAN
struct can_pin {
    int tx_pins;
    int rx_pins;
    int stb_pins;
};

static struct can_pin can_pins_array[] = {

    {
	.tx_pins = 25,
	.rx_pins = 26,
	.stb_pins = 28,
    },

    {
	.tx_pins = 25,
	.rx_pins = 26,
	.stb_pins = 28,
    }
};

static void can_set_pinctrl(int id)
{
     int index;
     unsigned long addr;
     uint32_t mask, val, shift;

     index = can_pins_array[id].tx_pins & 0x3;
     addr = (SYS_IOMUX_BASE + can_pins_array[id].tx_pins) & ~0x3;
     mask = 0xff << (index * 8);
     sys_clrbits(addr, mask);
     addr += OUTPUT_BASE;
     if (id == 0)
	val = 0x6 << (index * 8);
     else
	val = 0x32 << (index * 8);
     sys_clrsetbits(addr, mask, val);

     index = can_pins_array[id].stb_pins & 0x3;
     addr = (SYS_IOMUX_BASE + can_pins_array[id].stb_pins) & ~0x3;
     mask = 0xff << (index * 8);
     sys_clrbits(addr, mask);
     addr += OUTPUT_BASE;
     if (id == 0)
	val = 0x3 << (index * 8);
     else
	val = 0x2f << (index * 8);
     sys_clrsetbits(addr, mask, val);

     index = can_pins_array[id].rx_pins & 0x3;
     addr = (SYS_IOMUX_BASE + can_pins_array[id].rx_pins) & ~0x3;
     mask = 0xff << (index * 8);
     val = 0x1 << (index * 8);
     sys_clrsetbits(addr, mask, val);

     if (id == 0) {
	shift = 8;
	addr = SYS_IOMUX_BASE + INPUT_BASE;
     } else if (id == 1) {
	shift = 16;
	addr = SYS_IOMUX_BASE + INPUT_BASE + 0x24;
     }
     mask = 0xff << shift;
     val = (can_pins_array[id].rx_pins + 2) << shift;
     sys_clrsetbits(addr, mask, val);
}

void can_set_board_config(struct ipms_canfd *ipms)
{
    can_set_pinctrl(ipms->index);
    ipms->cfg.is_canfd = 0;
}
#endif
