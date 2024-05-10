/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <rtthread.h>
#include <rthw.h>
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "rpmsg_lite.h"

extern void jh7110_driver_init(void);
#ifdef USING_RPMSG_ECHO_TEST
int rpmsg_test();
#endif
#ifdef BSP_USING_RPMSG_LITE
struct rpmsg_lite_instance *rpmsg_lite[16];
#define VIRTIO_INSTANCE_SIZE 0x4000
#define RPMSG_CTRL_NUM	12
void rpmsg_lite_insance_init(void)
{
    int i;
    void *rpmsg_mem = get_rpmsg_base();

    for (i = 0; i < RPMSG_CTRL_NUM; i++) {
	rpmsg_lite[i] = rpmsg_lite_remote_init((void *)rpmsg_mem,
		    i, RL_NO_FLAGS);
	rpmsg_mem += VIRTIO_INSTANCE_SIZE;
    }
}

struct rpmsg_lite_instance* get_rpmsg_lite_instance(int id)
{
    return rpmsg_lite[id];
}
#endif

void jh7110_app_init(void)
{
#ifdef USING_RPMSG_ECHO_TEST
    rpmsg_test();
#endif
}

int main(void)
{
    printf("Hello RISC-V\n");

#ifdef BSP_USING_RPMSG_LITE
    rpmsg_lite_insance_init();
#endif
    jh7110_driver_init();
    jh7110_app_init();
    return 0;
}
