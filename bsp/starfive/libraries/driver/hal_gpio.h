struct gpio_irq {
    void *arg;
    void (*irq_cb)(void *param);
    int mode;
};

enum gpio_irq_mode {
    IRQ_MODE_RISING,
    IRQ_MODE_FALLING,
    IRQ_MODE_RISING_FALLING,
    IRQ_MODE_HIGH_LEVEL,
    IRQ_MODE_LOW_LEVEL,
};

struct gpio_hal_ops {
    void (*init_gpio_irq)(void);
    void (*set_pin_mode)(unsigned int pin, unsigned int mode);
    int (*pin_write)(unsigned int pin, unsigned int value);
    int (*pin_read)(unsigned int pin);
    void (*gpio_irq_handler)(void *arg);
    void (*set_irq_mode)(int pin, unsigned int mode);
    int (*pin_detach_irq)(unsigned int pin);
    int (*pin_irq_enable)(unsigned int pin, unsigned int enabled);
};

struct gpio_hal_data {
    struct gpio_hal_ops *ops;
    int irqno;
    int gpio_num;
};

struct gpio_irq *get_gpio_instance(int index);
void gpio_7110_set_ops(struct gpio_hal_data *data);

