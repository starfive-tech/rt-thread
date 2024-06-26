/*
 *
 */

#include <stdio.h>
#include <string.h>
#include "rpmsg_platform.h"
#include "rpmsg_env.h"
#include <rthw.h>
#include <rtdevice.h>
#include <rtatomic.h>
#include <interrupt.h>
#include "board.h"
#include "sbi.h"

#define IPI_MB_CHANS		2
#define MAX_DEV_PER_CHAN	8

#define RX_MBOX_OFFSET		0x400

#define RX_DONE_OFFSET		0x100

/* Please not change TX & RX */
enum ipi_mb_chan_type {
	IPI_MB_TYPE_TX   	= 0, /* Revert it*/
	IPI_MB_TYPE_RX		= 1, /* Rx */
};

struct ipi_mb_rpmsg {
	void *			tx_mbase;
	void *			rx_mbase;
	int			mem_size;
};

#define REMOTE_HART_ID	1
struct ipi_mb_rpmsg ipi_mb;

static struct rt_mutex plat_mutex;

int rpmsg_handler(unsigned long msg_type)
{
	rt_atomic_t *mb_base, msg;
	void *rx_done_base;
	int i, queue;

	env_set_ready();

	if (!msg_type)
		return 0;

	mb_base = ipi_mb.rx_mbase;
	rx_done_base = ipi_mb.rx_mbase + RX_DONE_OFFSET;
	if (msg_type & BIT(IPI_MB_TYPE_RX)) {
		for (i = 0; i < IPI_MB_CHANS * MAX_DEV_PER_CHAN; i++) {
			msg = rt_hw_atomic_exchange(&mb_base[i], 0);
			if (msg)
				env_isr(msg >> 16);
		}
	}
	if (msg_type & BIT(IPI_MB_TYPE_TX)) {
		mb_base = (void *)rx_done_base;
		for (i = 0; i < IPI_MB_CHANS * MAX_DEV_PER_CHAN; i++) {
			msg = rt_hw_atomic_exchange(&mb_base[i], 0);
			if (msg)
				env_isr(msg >> 16);
		}
	}

	return 0;
}

static void gen_sw_mailbox_init()
{
    ipi_mb.tx_mbase = get_rpmsg_mbox_base();
    ipi_mb.rx_mbase = ipi_mb.tx_mbase + RX_MBOX_OFFSET;
}

static void gen_sw_mbox_sendmsg(uint32_t ch, uint32_t msg)
{
    unsigned long hart_mask = 0x2;
    rt_atomic_t *mb_base = ipi_mb.tx_mbase;
    void *tx_done_base = ipi_mb.tx_mbase + RX_DONE_OFFSET;

    if (!(msg >> 16)) { /* tx queue */
	rt_hw_atomic_exchange(&mb_base[ch >> 1], msg);
	env_mb();
	sbi_send_ipi(&hart_mask, IPI_MB_TYPE_TX, REMOTE_HART_ID);
    } else {
	mb_base = tx_done_base;
	rt_hw_atomic_exchange(&mb_base[ch >> 1], msg);
	env_mb();
	sbi_send_ipi(&hart_mask, IPI_MB_TYPE_RX, REMOTE_HART_ID);
    }
}

int32_t platform_init_interrupt(uint32_t vector_id, void *isr_data)
{
    env_lock_mutex(&plat_mutex);
    /* Register ISR to environment layer */
    env_register_isr(vector_id, isr_data);
    env_unlock_mutex(&plat_mutex);
    return 0;
}

int32_t platform_deinit_interrupt(uint32_t vector_id)
{
    env_lock_mutex(&plat_mutex);
    /* Unregister ISR from environment layer */
    env_unregister_isr(vector_id);
    env_unlock_mutex(&plat_mutex);
    return 0;
}

void platform_notify(uint32_t vector_id)
{
   /* todo */
    uint32_t msg = (uint32_t)(vector_id << 16) | 0x1;

    env_lock_mutex(&plat_mutex);
    gen_sw_mbox_sendmsg(vector_id, msg);
    env_unlock_mutex(&plat_mutex);
}

/**
 * platform_map_mem_region
 *
 * Dummy implementation
 *
 */
void platform_map_mem_region(uint32_t vrt_addr, uint32_t phy_addr, uint32_t size, uint32_t flags)
{
}

/**
 * platform_cache_all_flush_invalidate
 *
 * Dummy implementation
 *
 */
void platform_cache_all_flush_invalidate(void)
{
}

/**
 * platform_cache_disable
 *
 * Dummy implementation
 *
 */
void platform_cache_disable(void)
{
}

/**
 * platform_init
 *
 * platform/environment init
 */
int32_t platform_init(void)
{
    gen_sw_mailbox_init();

    if (rt_mutex_init(&plat_mutex, "rpmsg", 0) != RT_EOK)
    	return -1;

    return 0;
}

/**
 * platform_deinit
 *
 * platform/environment deinit process
 */
int32_t platform_deinit(void)
{
    return 0;
}

