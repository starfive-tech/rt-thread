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

extern void jh7110_driver_init(void);
#ifdef USING_RPMSG_ECHO_TEST
int rpmsg_test();
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
    jh7110_driver_init();
    jh7110_app_init();
    return 0;
}
