
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "jh7110.h"
#include "board.h"
#include "gpio.h"
#include <riscv_io.h>
#include <interrupt.h>
#include <drivers/pin.h>

#define GENERAL_MASK		(0xff)

#define GPIO_LINE_DIRECTION_OUT 0
#define GPIO_LINE_DIRECTION_IN  1

#define JH7110_SYS_GPO_PDA_0_74_CFG	0x120
#define JH7110_SYS_GPO_PDA_89_94_CFG	0x284

#define PAD_GMAC1_MDC		75
#define PAD_GMAC1_TXC		88
#define PAD_QSPI_DATA3		94

/* pad control bits */
#define JH7110_PADCFG_POS	BIT(7)
#define JH7110_PADCFG_SMT	BIT(6)
#define JH7110_PADCFG_SLEW	BIT(5)
#define JH7110_PADCFG_PD	BIT(4)
#define JH7110_PADCFG_PU	BIT(3)
#define JH7110_PADCFG_BIAS	(JH7110_PADCFG_PD | JH7110_PADCFG_PU)
#define JH7110_PADCFG_DS_MASK	GENMASK(2, 1)
#define JH7110_PADCFG_DS_2MA	(0U << 1)
#define JH7110_PADCFG_DS_4MA	BIT(1)
#define JH7110_PADCFG_DS_8MA	(2U << 1)
#define JH7110_PADCFG_DS_12MA	(3U << 1)
#define JH7110_PADCFG_IE	BIT(0)

#define IOMUX_GPIO_IRQ		86

#define GPIO_NUM		64

struct gpio_irq {
    void *arg;
    void (*irq_cb)(void *param);
    int mode;
};

#ifdef BSP_USING_GPIO
static struct gpio_irq gpio_irq[GPIO_NUM];
#endif

static int jh7110_sys_get_padcfg_base(unsigned int pin)
{
	if (pin < PAD_GMAC1_MDC)
		return JH7110_SYS_GPO_PDA_0_74_CFG;
	else if (pin > PAD_GMAC1_TXC && pin <= PAD_QSPI_DATA3)
		return JH7110_SYS_GPO_PDA_89_94_CFG;
	else
		return -1;
}

static void jh7110_padcfg_rmw(unsigned int pin, unsigned int mask, unsigned int value)
{
	unsigned long base = get_iomux_base();
	void *reg;
	int padcfg_base;

	padcfg_base = jh7110_sys_get_padcfg_base(pin);
	if (padcfg_base < 0)
		return;

	reg = (void *)(base + padcfg_base + 4 * pin);
	value &= mask;

	sys_clrsetbits(reg, mask, value);
}

void jh7110_set_gpiomux(unsigned int pin, unsigned int din,
		unsigned int dout, unsigned int doen)
{
	unsigned long base = get_iomux_base();
	unsigned int offset = 4 * (pin / 4);
	unsigned int shift  = 8 * (pin % 4);
	unsigned int dout_mask = GENERAL_MASK << shift;
	unsigned int done_mask = GENERAL_MASK << shift;
	unsigned int ival, imask;
	void *reg_dout;
	void *reg_doen;
	void *reg_din;

	reg_dout = (void *)(base + OUTPUT_BASE + offset);
	reg_doen = (void *)(base + OUTEN_BASE + offset);
	dout <<= shift;
	doen <<= shift;
	if (din != GPI_NONE) {
		unsigned int ioffset = 4 * (din / 4);
		unsigned int ishift  = 8 * (din % 4);

		reg_din = (void *)(base + INPUT_BASE + ioffset);
		ival = (pin + 2) << ishift;
		imask = GENERAL_MASK << ishift;
	} else {
		reg_din = NULL;
	}

	sys_clrsetbits(reg_dout, dout_mask, dout);
	sys_clrsetbits(reg_doen, done_mask, doen);
	if (reg_din)
		sys_clrsetbits(reg_din, imask, ival);
}

void jh7110_gpio_set(unsigned int gpio, int value)
{
	unsigned long base = get_iomux_base();
	unsigned int offset = 4 * (gpio / 4);
	unsigned int shift  = 8 * (gpio % 4);
	void *reg_dout = (void *)(base + OUTPUT_BASE + offset);
	unsigned int dout = (value ? GPOUT_HIGH : GPOUT_LOW) << shift;
	unsigned int mask = GENERAL_MASK << shift;

	sys_clrsetbits(reg_dout, mask, dout);
}

int jh7110_gpio_get_direction(unsigned int gpio)
{
	unsigned long base = get_iomux_base();
	unsigned int offset = 4 * (gpio / 4);
	unsigned int shift  = 8 * (gpio % 4);
	void *addr;

	addr = (void *)(base + OUTEN_BASE + offset);
	unsigned int doen = sys_readl(addr);

	doen = (doen >> shift) & GENERAL_MASK;

	return doen == GPOEN_ENABLE ?
		GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

void jh7110_gpio_direction_input(unsigned int gpio)
{
	/* enable input and schmitt trigger */
	jh7110_padcfg_rmw(gpio,
			  JH7110_PADCFG_IE | JH7110_PADCFG_SMT,
			  JH7110_PADCFG_IE | JH7110_PADCFG_SMT);

	jh7110_set_gpiomux(gpio, GPI_NONE, GPOUT_LOW, GPOEN_DISABLE);
}

void jh7110_gpio_direction_output(unsigned int gpio, int value)
{
	jh7110_set_gpiomux(gpio,
			   GPI_NONE, value ? GPOUT_HIGH : GPOUT_LOW,
			   GPOEN_ENABLE);

	/* disable input, schmitt trigger and bias */
	jh7110_padcfg_rmw(gpio,
			  JH7110_PADCFG_IE | JH7110_PADCFG_SMT |
			  JH7110_PADCFG_BIAS, 0);
}

int jh7110_gpio_input_get(unsigned int gpio)
{
	unsigned long base = get_iomux_base();
	void *reg = (void *) (base + JH7110_SYS_GPIOIN
			+ 4 * (gpio / 32));

	return !!(sys_readl(reg) & BIT(gpio % 32));
}

int jh7110_gpio_output_get(unsigned int gpio)
{
	unsigned long base = get_iomux_base();
	unsigned int offset = 4 * (gpio / 4);
	unsigned int shift  = 8 * (gpio % 4);
	unsigned int dout_mask = GENERAL_MASK << shift;
	unsigned int dout;
	void *addr;

	addr = (void *)(base + OUTPUT_BASE + offset);
	dout = (sys_readl(addr) >> shift) & GENERAL_MASK;

	return dout;
}


#ifdef RT_USING_PIN
#ifdef BSP_USING_GPIO
static void jh7110_irq_ack(int gpio)
{
	void *base =  (void *)(get_iomux_base() + 4 * (gpio / 32));
	unsigned int mask = BIT(gpio % 32);

	sys_clrsetbits(base + JH7110_SYS_GPIOIC0, mask, ~mask);
	sys_clrsetbits(base + JH7110_SYS_GPIOIC0, mask, mask);
}

static void jh7110_irq_mask(int gpio)
{
	void *base = (void *)(get_iomux_base() + 4 * (gpio / 32));
	unsigned int mask = BIT(gpio % 32);

	sys_clrbits(base + JH7110_SYS_GPIOIE0, mask);
}

static void jh7110_irq_unmask(int gpio)
{
	void *base = (void *)(get_iomux_base() + 4 * (gpio / 32));
	unsigned int mask = BIT(gpio % 32);

	sys_clrsetbits(base + JH7110_SYS_GPIOIE0, mask, mask);
}

static void jh7110_irq_set_type(int pin, unsigned int mode,
	void (*hdr)(void *args), void *args)
{
	void *base = (void *)(get_iomux_base() + 4 * (pin / 32));
	unsigned int mask = BIT(pin % 32);
	unsigned int irq_type, edge_both, polarity;

	if (mode == gpio_irq[pin].mode) {
		rt_hw_plic_irq_disable(IOMUX_GPIO_IRQ);
		if (hdr != gpio_irq[pin].irq_cb)
			gpio_irq[pin].irq_cb = hdr;
		gpio_irq[pin].arg = args;
		rt_hw_plic_irq_enable(IOMUX_GPIO_IRQ);
		return;
	}

	switch (mode) {
	case PIN_IRQ_MODE_RISING:
		irq_type  = mask; /* 1: edge triggered */
		edge_both = 0;    /* 0: single edge */
		polarity  = mask; /* 1: rising edge */
		break;
	case PIN_IRQ_MODE_FALLING:
		irq_type  = mask; /* 1: edge triggered */
		edge_both = 0;    /* 0: single edge */
		polarity  = 0;    /* 0: falling edge */
		break;
	case PIN_IRQ_MODE_RISING_FALLING:
		irq_type  = mask; /* 1: edge triggered */
		edge_both = mask; /* 1: both edges */
		polarity  = 0;    /* 0: ignored */
		break;
	case PIN_IRQ_MODE_HIGH_LEVEL:
		irq_type  = 0;    /* 0: level triggered */
		edge_both = 0;    /* 0: ignored */
		polarity  = mask; /* 1: high level */
		break;
	case PIN_IRQ_MODE_LOW_LEVEL:
		irq_type  = 0;    /* 0: level triggered */
		edge_both = 0;    /* 0: ignored */
		polarity  = 0;    /* 0: low level */
		break;
	default:
		return;
	}

	rt_hw_plic_irq_disable(IOMUX_GPIO_IRQ);
	sys_clrsetbits(base + JH7110_SYS_GPIOIS0, mask, irq_type);
	sys_clrsetbits(base + JH7110_SYS_GPIOIBE0, mask, edge_both);
	sys_clrsetbits(base + JH7110_SYS_GPIOIEV0, mask, polarity);
	if (hdr != gpio_irq[pin].irq_cb)
		gpio_irq[pin].irq_cb = hdr;
	gpio_irq[pin].arg = args;
	rt_hw_plic_irq_enable(IOMUX_GPIO_IRQ);

	return;
}

static void jh7110_sys_irq_handler(int vector, void *param)
{
	void *base = (void *)get_iomux_base();
	unsigned int mis, pin;
	int i;

	mis = sys_readl(base + JH7110_SYS_GPIOMIS0);
	for (i = 0; i < 32; i++) {
		if (mis & BIT(i)) {
			jh7110_irq_ack(i);
			if (gpio_irq[i].irq_cb)
				gpio_irq[i].irq_cb(gpio_irq[i].arg);
		}
	}

	mis = sys_readl(base + JH7110_SYS_GPIOMIS1);
	for (i = 0; i < 32; i++) {
		if (mis & BIT(i)) {
			jh7110_irq_ack(i + 32);
			if (gpio_irq[i + 32].irq_cb)
				gpio_irq[i + 32].irq_cb(gpio_irq[i].arg);
		}
	}
}

static int jh7110_sys_init_hw(void)
{
	void *base = (void *)get_iomux_base();
	int i;

	/* mask all GPIO interrupts */
	sys_writel(0U, base + JH7110_SYS_GPIOIE0);
	sys_writel(0U, base + JH7110_SYS_GPIOIE1);
	/* clear edge interrupt flags */
	sys_writel(~0U, base + JH7110_SYS_GPIOIC0);
	sys_writel(~0U, base + JH7110_SYS_GPIOIC1);
	/* enable GPIO interrupts */
	sys_writel(1U, base + JH7110_SYS_GPIOEN);

	for (i = 0; i < GPIO_NUM; i++)
		gpio_irq[i].mode = -1;

	rt_hw_interrupt_install(IOMUX_GPIO_IRQ, jh7110_sys_irq_handler, NULL, "sys_gpio_irq");
	rt_hw_interrupt_umask(IOMUX_GPIO_IRQ);

	return 0;
}
#endif

void jh7110_pin_mode(struct rt_device *device, rt_base_t pin, rt_uint8_t mode)
{
    unsigned int gpio = pin;

    if (pin >= GPIO_NUM)
	return;

    if (PIN_MODE_OUTPUT == mode)
    {
	jh7110_gpio_direction_output(pin, GPOUT_LOW);
    }
    else if (PIN_MODE_INPUT == mode)
    {
	jh7110_gpio_direction_input(pin);
    }

    return;
}

void jh7110_pin_write(struct rt_device *device, rt_base_t pin, rt_uint8_t value)
{
    unsigned int gpio = pin;

    if (pin >= GPIO_NUM)
	return;

    jh7110_gpio_set(pin, value);

    return ;
}

rt_int8_t jh7110_pin_read(struct rt_device *device, rt_base_t pin)
{
    int ret;

    if (pin >= GPIO_NUM)
	return RT_ERROR;

    if (jh7110_gpio_get_direction(pin) == GPIO_LINE_DIRECTION_OUT) {
	ret = jh7110_gpio_output_get(pin);
	RT_ASSERT(ret < 2);
    } else
	ret = jh7110_gpio_input_get(pin);

    return ret;
}

#ifdef BSP_USING_GPIO
rt_err_t jh7110_pin_attach_irq(struct rt_device *device, rt_base_t pin,
                             rt_uint8_t mode, void (*hdr)(void *args), void *args)
{
    unsigned int gpio = pin;
    char irq_name[10];
    rt_uint32_t type;

    if (gpio >= GPIO_NUM)
	return RT_ERROR;

    if ((gpio_irq[gpio].mode == mode)
	&& (gpio_irq[gpio].irq_cb == hdr)
	&& (gpio_irq[gpio].arg == args))
	return RT_EOK;

    jh7110_irq_set_type(gpio, mode, hdr, args);

    return RT_EOK;
}

rt_err_t jh7110_pin_detach_irq(struct rt_device *device, rt_base_t pin)
{
    rt_hw_plic_irq_disable(IOMUX_GPIO_IRQ);
    jh7110_irq_mask(pin);
    jh7110_irq_ack(pin);
    rt_hw_plic_irq_enable(IOMUX_GPIO_IRQ);

    gpio_irq[pin].mode = -1;
    gpio_irq[pin].irq_cb = NULL;
    gpio_irq[pin].arg = NULL;

    return RT_EOK;
}

rt_err_t jh7110_pin_irq_enable(struct rt_device *device, rt_base_t pin, rt_uint8_t enabled)
{
    unsigned int gpio = pin;

    rt_hw_plic_irq_disable(IOMUX_GPIO_IRQ);
    if (enabled) {
	jh7110_irq_unmask(pin);
    } else {
	jh7110_irq_mask(pin);
    }
    rt_hw_plic_irq_enable(IOMUX_GPIO_IRQ);
    return RT_EOK;
}
#endif

const static struct rt_pin_ops jh7110_pin_ops =
{
    jh7110_pin_mode,
    jh7110_pin_write,
    jh7110_pin_read,
#ifdef BSP_USING_GPIO
    jh7110_pin_attach_irq,
    jh7110_pin_detach_irq,
    jh7110_pin_irq_enable,
#endif
    RT_NULL, /* pins get */
};


int hw_pin_init(void)
{
    int ret = RT_EOK;

#ifdef BSP_USING_GPIO
    jh7110_sys_init_hw();
#endif

    ret = rt_device_pin_register("pin", &jh7110_pin_ops, RT_NULL);

    return ret;
}

#endif /*RT_USING_PIN */

