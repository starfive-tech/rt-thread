/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     lizhirui     first version
 */

#ifndef BOARD_H__
#define BOARD_H__

#include <rtconfig.h>

extern unsigned int __bss_start;
extern unsigned int __bss_end;

#ifndef RT_USING_SMART
#define KERNEL_VADDR_START 0x0
#endif

#define HEAP_PHY_START 0x6f000000

#define RT_HW_HEAP_BEGIN ((void *)HEAP_PHY_START)
#define RT_HW_HEAP_END   ((void *)(RT_HW_HEAP_BEGIN + 16 * 1024 * 1024))
/*
#define RT_HW_PAGE_START RT_HW_HEAP_END
#define RT_HW_PAGE_END   ((void *)(KERNEL_VADDR_START + 8 * 1024 * 1024))
*/

void rt_hw_board_init(void);
void rt_init_user_mem(struct rt_thread *thread, const char *name,
                      unsigned long *entry);
void rt_plic_init(void);
void *get_rpmsg_sharemem_base();
#ifdef BSP_USING_RPMSG_LITE
void *get_rpmsg_mbox_base();
void *get_rpmsg_base(void);
#endif
void *get_ipi_handler();
int get_uart_config_num();

#if defined(BSP_USING_GMAC)
int rt_hw_gmac_init(void);
#endif
#if defined(BSP_USING_CAN)
int rt_hw_canfd_init(void);
#endif

unsigned long sys_cur_time_ms(void);
void sys_udelay(int us);
void sys_mdelay(int ms);
unsigned int sys_gmac_get_csr_clk(int id);
void sys_gmac_invalidate_cache_range(unsigned long start, unsigned long end);
void sys_gmac_flush_dcache_range(unsigned long start, unsigned long end);
void sys_tick_sleep(unsigned int tick);

#define ALIGN(addr, align) (((addr) + (align) - 1) & ~(align -1))
#define BIT(bitnum) (1 << (bitnum % 32))
#define GENMASK(h, l) \
  (((~0UL) - (1UL << (l)) + 1) & (~0UL >> (64 - 1 - (h))))

#define hal_printf rt_kprintf
#define hal_malloc rt_malloc
#define hal_free   rt_free

#endif
