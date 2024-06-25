
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "hal_gpio.h"
#include <riscv_io.h>
#include <interrupt.h>
#include <drivers/pin.h>

struct gpio_hal_data gpio_data;

#define GPIO_NUM		128

#ifdef BSP_USING_GPIO
static struct gpio_irq gpio_irq[GPIO_NUM];
#endif

#ifdef RT_USING_PIN
#ifdef BSP_USING_GPIO
struct gpio_irq *get_gpio_instance(int i)
{
	return &gpio_irq[i];
}

static void gpio_irq_set_type(struct gpio_hal_data *data, int pin, unsigned int mode,
		void (*hdr)(void *args), void *args)
{
	unsigned int new_mode;

	if (mode == gpio_irq[pin].mode) {
		rt_hw_plic_irq_disable(data->irqno);
		if (hdr != gpio_irq[pin].irq_cb)
			gpio_irq[pin].irq_cb = hdr;
		gpio_irq[pin].arg = args;
		rt_hw_plic_irq_enable(data->irqno);
		return;
	}

	rt_hw_plic_irq_disable(data->irqno);

	switch (mode) {
	case PIN_IRQ_MODE_RISING:
		new_mode = IRQ_MODE_RISING;
		break;
	case PIN_IRQ_MODE_FALLING:
		new_mode = IRQ_MODE_FALLING;
		break;
	case PIN_IRQ_MODE_RISING_FALLING:
		new_mode = IRQ_MODE_RISING_FALLING;
		break;
	case PIN_IRQ_MODE_HIGH_LEVEL:
		new_mode = IRQ_MODE_HIGH_LEVEL;
		break;
	case PIN_IRQ_MODE_LOW_LEVEL:
		new_mode = IRQ_MODE_LOW_LEVEL;
		break;
	default:
		return;
	}
	data->ops->set_irq_mode(pin, new_mode);
	if (hdr != gpio_irq[pin].irq_cb)
		gpio_irq[pin].irq_cb = hdr;
	gpio_irq[pin].arg = args;
	rt_hw_plic_irq_enable(data->irqno);

	return;
}

static void gpio_sys_irq_handler(int vector, void *param)
{
        struct gpio_hal_data *data = (void *)param;

	data->ops->gpio_irq_handler(NULL);
}

static int gpio_irq_sys_init_hw(void)
{
	int i;

	gpio_data.ops->init_gpio_irq();

	for (i = 0; i < GPIO_NUM; i++)
		gpio_irq[i].mode = -1;

	rt_hw_interrupt_install(gpio_data.irqno, gpio_sys_irq_handler, &gpio_data, "sys_gpio_irq");
	rt_hw_interrupt_umask(gpio_data.irqno);

	return 0;
}
#endif

static void pin_mode(struct rt_device *device, rt_base_t pin, rt_uint8_t mode)
{
    struct gpio_hal_data *data = (void *)device->user_data;
    unsigned int gpio = pin;

    if (pin >= GPIO_NUM)
	return;

    if (PIN_MODE_OUTPUT == mode)
	data->ops->set_pin_mode(gpio, 0); /*output */
    else if (PIN_MODE_INPUT == mode)
	data->ops->set_pin_mode(gpio, 1); /*input */

    return;
}

static void pin_write(struct rt_device *device, rt_base_t pin, rt_uint8_t value)
{
    struct gpio_hal_data *data = (void *)device->user_data;
    unsigned int gpio = pin;

    if (pin >= GPIO_NUM)
	return;

    data->ops->pin_write(pin, value);
}

static rt_int8_t pin_read(struct rt_device *device, rt_base_t pin)
{
    struct gpio_hal_data *data = (void *)device->user_data;
    int ret;

    if (pin >= GPIO_NUM)
	return RT_ERROR;

    return data->ops->pin_read(pin);
}

#ifdef BSP_USING_GPIO
static rt_err_t pin_attach_irq(struct rt_device *device, rt_base_t pin,
                             rt_uint8_t mode, void (*hdr)(void *args), void *args)
{
    struct gpio_hal_data *data = (void *)device->user_data;
    unsigned int gpio = pin;
    char irq_name[10];
    rt_uint32_t type;

    if (gpio >= GPIO_NUM)
	return RT_ERROR;

    if ((gpio_irq[gpio].mode == mode)
	&& (gpio_irq[gpio].irq_cb == hdr)
	&& (gpio_irq[gpio].arg == args))
	return RT_EOK;

    gpio_irq_set_type(data, gpio, mode, hdr, args);

    return RT_EOK;
}

static rt_err_t pin_detach_irq(struct rt_device *device, rt_base_t pin)
{
    unsigned int gpio = pin;
    struct gpio_hal_data *data = (void *)device->user_data;

    rt_hw_plic_irq_disable(data->irqno);
    data->ops->pin_detach_irq(gpio);
    rt_hw_plic_irq_enable(data->irqno);

    gpio_irq[pin].mode = -1;
    gpio_irq[pin].irq_cb = NULL;
    gpio_irq[pin].arg = NULL;

    return RT_EOK;
}

static rt_err_t pin_irq_enable(struct rt_device *device, rt_base_t pin, rt_uint8_t enabled)
{
    unsigned int gpio = pin;
    struct gpio_hal_data *data = (void *)device->user_data;

    rt_hw_plic_irq_disable(data->irqno);
    data->ops->pin_irq_enable(gpio, enabled);
    rt_hw_plic_irq_enable(data->irqno);
    return RT_EOK;
}
#endif

const static struct rt_pin_ops pin_ops =
{
    pin_mode,
    pin_write,
    pin_read,
#ifdef BSP_USING_GPIO
    pin_attach_irq,
    pin_detach_irq,
    pin_irq_enable,
#endif
    RT_NULL, /* pins get */
};

int hw_pin_init(void)
{
    int ret = RT_EOK;

    gpio_7110_set_ops(&gpio_data);
#ifdef BSP_USING_GPIO
    gpio_irq_sys_init_hw();
#endif

    ret = rt_device_pin_register("pin", &pin_ops, &gpio_data);

    return ret;
}

#endif /*RT_USING_PIN */

