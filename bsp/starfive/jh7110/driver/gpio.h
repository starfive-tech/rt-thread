#ifndef __BSP_JH7110_GPIO_H_
#define __BSP_JH7110_GPIO_H_

#define GPI_NONE		255

#define GPOUT_HIGH		1
#define GPOUT_LOW		0

#define GPOEN_ENABLE		0
#define	GPOEN_DISABLE		1

int hw_pin_init(void);
void jh7110_set_gpiomux(unsigned int pin, unsigned int din,
		unsigned int dout, unsigned int doen);
void jh7110_gpio_set(unsigned int gpio, int value);
int jh7110_gpio_get_direction(unsigned int gpio);
void jh7110_gpio_direction_input(unsigned int gpio);
void jh7110_gpio_direction_output(unsigned int gpio, int value);
int jh7110_gpio_get(unsigned int gpio);

#endif
