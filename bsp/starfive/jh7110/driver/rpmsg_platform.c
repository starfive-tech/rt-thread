/*
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "rpmsg_platform.h"
#include "rpmsg_env.h"
#include <rthw.h>
#include <rtdevice.h>
#include <interrupt.h>
#include "board.h"

/*
 * Generic software mailbox Registers:
 *
 * RX_STATUS[n]: RX channel n status
 * TX_STATUS[n]: TX channel n status
 * 	0: indicates message in T/RX_CH[n] is invalid and channel ready.
 * 	1: indicates message in T/RX_CH[n] is valid and channel busy.
 * 	2: indicates message in T/RX_CH[n] has been received by the peer.
 * RX_CH[n]: Receive data register for channel n
 * TX_CH[n]: Transmit data register for channel n
 *
 * To send a message:
 * Update the data register TX_CH[n] with the message, then set the
 * TX_STATUS[n] to 1, inject a interrupt to remote side; after the
 * transmission done set the TX_STATUS[n] back to 0.
 *
 * When received a message:
 * Get the received data from RX_CH[n] and then set the RX_STATUS[n] to
 * 2 and inject a interrupt to notify the remote side transmission done.
 */

#define MAX_CH (16)
#define RPMSG_DFT_CHANNEL 0

struct gen_sw_mbox
{
    uint32_t tx_status[MAX_CH];
    uint32_t rx_status[MAX_CH];
    uint32_t reserved[MAX_CH];
    uint32_t tx_ch[MAX_CH];
    uint32_t rx_ch[MAX_CH];
};

enum sw_mbox_channel_status
{
    S_READY,
    S_BUSY,
    S_DONE,
};

static struct rt_mutex plat_mutex;
int rpmsg_handler(unsigned long param) /* maybe put this handler to thread */
{

    struct gen_sw_mbox *base = (struct gen_sw_mbox *)get_rpmsg_mbox_base();
    unsigned long hart_mask = 0x2;
    uint32_t vector_id;

    env_set_ready();
    //rt_kprintf("int tx stat %d rx stat %d\n", base->tx_status[0], base->rx_status[0]);

    if (base->tx_status[RPMSG_DFT_CHANNEL] == S_DONE) {
    	env_isr(0);
	base->tx_status[RPMSG_DFT_CHANNEL] = S_READY;
    }
    /* Check if the interrupt is for us */
    if (base->rx_status[RPMSG_DFT_CHANNEL] != S_BUSY) {
    	return 0;
    }
    /* todo one vector id only */
    //vector_id = base->rx_ch[RPMSG_MBOX_CHANNEL];
	
    base->rx_status[RPMSG_DFT_CHANNEL] = S_DONE;
    env_mb();

    env_isr(1); /* recv queue */

    return 0;
    //sbi_send_ipi(&hart_mask, 1, 1); /* todo set type */
}

static void gen_sw_mailbox_init(struct gen_sw_mbox *base)
{
    /* Clear status register */
    base->rx_status[RPMSG_DFT_CHANNEL] = 0;
    base->tx_status[RPMSG_DFT_CHANNEL] = 0;
}

static void gen_sw_mbox_sendmsg(struct gen_sw_mbox *base, uint32_t ch, uint32_t msg)
{
    unsigned long hart_mask = 0x2;


    //rt_kprintf("set tx stat %d rx stat %d msg %d\n", base->tx_status[0], base->tx_status[0], msg);

    base->tx_ch[ch]     = msg;
    if (msg == 0)
    	base->tx_status[ch] = S_BUSY;
    /* sync before trigger interrupt to remote */

    /* todo */
    env_mb();

    sbi_send_ipi(&hart_mask, 0, 1); /* todo set type */
}

int32_t platform_init_interrupt(uint32_t vector_id, void *isr_data)
{
    //rt_mutex_take(&plat_mutex, 0);
    env_lock_mutex(&plat_mutex);
    /* Register ISR to environment layer */
    env_register_isr(vector_id, isr_data);
    //rt_mutex_release(&plat_mutex);
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
    uint32_t msg = (uint32_t)(vector_id << 16);

    env_lock_mutex(&plat_mutex);
    gen_sw_mbox_sendmsg((struct gen_sw_mbox *)get_rpmsg_mbox_base(), RPMSG_DFT_CHANNEL, msg);
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
    gen_sw_mailbox_init((struct gen_sw_mbox *)get_rpmsg_mbox_base());

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

