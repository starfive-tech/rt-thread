/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-06-28     misonyo     the first version.
 */

#ifndef HAL_CAN_H__
#define HAL_CAN_H__

#define CAN_RX_IND  0x1
#define CAN_TX_DONE 0x2
#define CAN_RX_OVERFLOW 0x4
#define CAN_ERROR 0x8

struct canfd_cfg {
    char *name;
    int clk_freq;
    int is_canfd;
};

struct hal_canfd;

struct hal_can_ops {
    int (*configure)(void *can, struct can_configure *cfg);
    int (*start_xmit)(const void *data, void *ipms_priv, int canfd);
    int (*rx_poll)(void *can, void *buf);
    unsigned int (*can_isr)(void *);
    int (*canfd_init)(struct hal_canfd *, int);
};

struct hal_canfd {
    struct rt_can_device dev;
    struct canfd_cfg cfg;
    struct hal_can_ops *can_ops;
    void *base;
    int index;
    int irq;
    uint32_t int_flag;
    uint32_t baudrate;
    void *priv;
    //CANFD_FD_T sCANFD_Config;
};

#define CAN_CTRLMODE_LOOPBACK		0x01	/* Loopback mode */
#define CAN_CTRLMODE_LISTENONLY		0x02	/* Listen-only mode */
#define CAN_CTRLMODE_3_SAMPLES		0x04	/* Triple sampling mode */
#define CAN_CTRLMODE_ONE_SHOT		0x08	/* One-Shot mode */
#define CAN_CTRLMODE_BERR_REPORTING	0x10	/* Bus-error reporting */
#define CAN_CTRLMODE_FD			0x20	/* CAN FD mode */
#define CAN_CTRLMODE_PRESUME_ACK	0x40	/* Ignore missing CAN ACKs */
#define CAN_CTRLMODE_FD_NON_ISO		0x80	/* CAN FD in non-ISO mode */
#define CAN_CTRLMODE_CC_LEN8_DLC	0x100	/* Classic CAN DLC option */

void can_plat_init(struct hal_canfd *ipms);
int rt_hw_can_init(void);
void can_set_board_config(struct hal_canfd *ipms);
#endif /* DRV_CAN_H__ */
