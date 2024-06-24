

#include "rtconfig.h"

#if defined(BSP_USING_GMAC)

#include <rtthread.h>
#include <rtdevice.h>
#include <netif/ethernetif.h>
#include <netif/etharp.h>
#include <lwip/sys.h>
#include <lwip/icmp.h>
#include <lwip/pbuf.h>
#include "lwipopts.h"
#include "board.h"
#include "interrupt.h"
#include "hal_gmac.h"
#include "plic.h"

#if 0
#define DBG_ENABLE
//#undef  DBG_ENABLE
#define DBG_LEVEL  LOG_LVL_INFO
#define DBG_SECTION_NAME  "gmac"
#define DBG_COLOR
#include <rtdbg.h>
#endif

enum
{
    GMAC_START = -1,
#if defined(BSP_USING_GMAC0)
    GMAC0_IDX,
#endif
#if defined(BSP_USING_GMAC1)
    GMAC1_IDX,
#endif
#if defined(BSP_USING_EXT_GMAC)
    GMAC_EXT_IDX,
#endif
    GMAC_CNT,
};

struct gmac_lwip_pbuf {
    struct pbuf_custom p;
    void *buf;
    gmac_handle_t *gmacdev;
};

#define HAL_ETHERNET_MTU	1536

/* Private variables ------------------------------------------------------------*/
#if defined(BSP_USING_GMAC0)
    LWIP_MEMPOOL_DECLARE(gmac0_rx, HAL_EQOS_DESC_NUM, sizeof(struct gmac_lwip_pbuf), "GMAC0 RX PBUF pool");
#endif

#if defined(BSP_USING_GMAC1)
    LWIP_MEMPOOL_DECLARE(gmac1_rx, HAL_EQOS_DESC_NUM, sizeof(struct gmac_lwip_pbuf), "GMAC1 RX PBUF pool");
#endif

#if defined(BSP_USING_EXT_GMAC)
    LWIP_MEMPOOL_DECLARE(gmac_ext_rx, HAL_EQOS_DESC_NUM, sizeof(struct gmac_lwip_pbuf), "pcie gmac RX PBUF pool");
#endif

static gmac_handle_t dw_gmac[] =
{
#if defined(BSP_USING_GMAC0)
{
	.name		 =  "g0",
	.id		 =  0,
	.memp_rx_pool	 =  &memp_gmac0_rx,
},
#endif
#if defined(BSP_USING_GMAC1)
{
	.name		 =  "g1",
	.id		 =  1,
	.memp_rx_pool	 =  &memp_gmac1_rx,
},
#endif
#if defined(BSP_USING_EXT_GMAC) /* pcie gmac */
{
	.name		 =  "ge",
	.id		 =  2,
	.memp_rx_pool	 =  &memp_gmac_ext_rx,
	.pcie_iobase	 = 0,
},
#endif
};

void sys_gmac_invalidate_cache_range(unsigned long start, unsigned long end){}
void sys_gmac_flush_dcache_range(unsigned long start, unsigned long end) {}

void dw_gmac_pkt_dump(const char *msg, const struct pbuf *p)
{
    rt_uint32_t i;
    rt_uint8_t *ptr = p->payload;

    DW_GMAC_TRACE("%s %d byte\n", msg, p->tot_len);

    for (i = 0; i < p->tot_len; i++)
    {
        if ((i % 8) == 0)
        {
            DW_GMAC_TRACE("  ");
        }
        if ((i % 16) == 0)
        {
            DW_GMAC_TRACE("\r\n");
        }
        DW_GMAC_TRACE("%02x ", *ptr);
        ptr++;
    }
    DW_GMAC_TRACE("\n\n");
}

void gmac_link_change(gmac_handle_t *dev,int up)
{
    if (up) {
        rt_kprintf("gmac %s link up\n", dev->name);
	if (dev->phy_dev->mode_changed) {
	    dev->ops->set_speed_and_duplex(dev->priv);
	    dev->phy_dev->mode_changed = 0;
	}
	eth_device_linkchange(&dev->eth, RT_TRUE);
	dev->phy_dev->link_status = RT_TRUE;
	if (!dev->phy_dev->pcie_netcard) {
	    plic_set_priority(dev->gmac_config.irq, 2);
	    rt_hw_plic_irq_enable(dev->gmac_config.irq);
	}
    }
    else {
        rt_kprintf("gmac %s link down\n", dev->name);
	if (!dev->phy_dev->pcie_netcard)
	    rt_hw_plic_irq_disable(dev->gmac_config.irq);
	dev->phy_dev->link_status = RT_FALSE;
        eth_device_linkchange(&dev->eth, RT_FALSE);
    }
}

static void rt_hw_gmac_isr(int irq, void *arg)
{
    gmac_handle_t *gmac = (gmac_handle_t *)arg;
    rt_uint32_t status_dma_reg = 0;
    int ret;

    rt_ubase_t level = rt_hw_interrupt_disable();
    ret = gmac->ops->gmac_isr(gmac->priv);
    if (ret < 0) {
	rt_kprintf("isr: tx hard_error\n");
	return;
    }
    if (ret & INT_RX) {
	eth_device_ready(&gmac->eth);
    }
    rt_hw_interrupt_enable(level);
}

void phy_link_detect(void *param)
{
    unsigned short bmsr = 0;
    unsigned short link_status = 0;
    unsigned short phy_val;
    int ret = -1, i;

    while(1)
    {
        for (i = (GMAC_START + 1); i < GMAC_CNT; i++) {
	    gmac_handle_t *gmac = &dw_gmac[i];
	    if (gmac->phy_dev && gmac->phy_dev->phy_detect_start)
		gmac->phy_dev->ops->check_link_status(gmac);
	}
	sys_tick_sleep(RT_TICK_PER_SECOND);
    }
}

static rt_err_t dw_gmac_init(rt_device_t device)
{
    gmac_handle_t *gmac = (gmac_handle_t *)device;
    rt_thread_t link_detect;
    char gmac_name[8] = "gmac0";
    int ret;
    RT_ASSERT(device);

    if (gmac->pcie_iobase)
	gmac->msi_handler = rt_hw_gmac_isr;
    gmac = gmac->ops->open(gmac);

    if (!gmac)
	return -RT_ERROR;

    gmac_name[4] = '0' + gmac->id;
    if (!gmac->pcie_iobase)
	rt_hw_interrupt_install(gmac->gmac_config.irq, rt_hw_gmac_isr, gmac, gmac_name);

    if (gmac->phy_dev) {
	if (gmac->phy_dev->link_status)
		gmac_link_change(gmac, 1);
	gmac->phy_dev->phy_detect_start = 1;
    }

    return 0;
}

static rt_err_t dw_gmac_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t dw_gmac_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_ssize_t dw_gmac_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_ssize_t dw_gmac_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t dw_gmac_control(rt_device_t device, int cmd, void *args)
{
    gmac_handle_t *gmac = (gmac_handle_t *)device;
    RT_ASSERT(device);

    switch (cmd)
    {
    case NIOCTL_GADDR:
        if (args)
		rt_memcpy(args, &gmac->gmac_config.enetaddr[0], 6);
        else
		return -RT_ERROR;
        break;
    default :
        break;
    }

    return RT_EOK;
}

void dw_gmac_pbuf_free(struct pbuf *p)
{
    struct gmac_lwip_pbuf *lwip_buf = (void *)p;
    gmac_handle_t *gmac_dev = lwip_buf->gmacdev;

    //SYS_ARCH_DECL_PROTECT(old_level);
    //SYS_ARCH_PROTECT(old_level);
    gmac_dev->ops->free_pkt(gmac_dev->priv, lwip_buf->buf);
    memp_free_pool(lwip_buf->gmacdev->memp_rx_pool, lwip_buf);
    //SYS_ARCH_UNPROTECT(old_level);
}

static rt_err_t dw_gmac_tx(rt_device_t dev, struct pbuf *p)
{
    gmac_handle_t *gmac = (gmac_handle_t *)dev;
    struct pbuf *q;
    void *dist;
    int copy_offset = 0, ret;

    if (!gmac->phy_dev || !gmac->phy_dev->link_status)
	return -RT_ERROR;

    ret = gmac->ops->check_descriptor(gmac->priv, &dist);
    if (ret < 0) {
	return RT_ERROR;
    }

    LOG_DBG("gmac tx packet len %d total len %d\n", p->len, p->tot_len);

    for(q = p;q != RT_NULL;q=q->next)
    {
        rt_memcpy(dist + copy_offset,q->payload,q->len);
        copy_offset += q->len;

        if(copy_offset > HAL_ETHERNET_MTU)
        {
            rt_kprintf("send data exceed max len copy_offset %d\n",copy_offset);
            return -RT_ERROR;
        }
    }

    return gmac->ops->send(gmac->priv, copy_offset);

}

static struct pbuf *dw_gmac_rx(rt_device_t dev)
{
    gmac_handle_t *gmac = (gmac_handle_t *)dev;
    struct pbuf *pbuf = RT_NULL;
    struct gmac_lwip_pbuf *lwip_buf;
    int rx_index, pkt_len;
    void *packetp;

    if ((pkt_len = gmac->ops->recv(gmac->priv, &packetp)) > 0) {
	LOG_DBG("gmac rx packet len %d\n", pkt_len);
        lwip_buf = (void *)memp_malloc_pool(gmac->memp_rx_pool);
	RT_ASSERT(lwip_buf);
        {
	    lwip_buf->p.custom_free_function = dw_gmac_pbuf_free;
	    lwip_buf->buf	= packetp;
	    lwip_buf->gmacdev = gmac;

	    pbuf = pbuf_alloced_custom(PBUF_RAW,
				   pkt_len,
				   PBUF_REF,
				   &lwip_buf->p,
				   packetp,
				   HAL_ETHERNET_MTU);
	    if (pbuf == RT_NULL)
	        rt_kprintf("%s: failed to alloted %08x\n", gmac->name, pbuf);
         }
    }

    return pbuf;
}

static void get_gmac_addr_from_sharemem(gmac_handle_t *gmac, int index)
{
    void *base = get_rpmsg_sharemem_base();
    const char *addr = gmac->gmac_config.enetaddr;

    if (index == 0)
	rt_memcpy(gmac->gmac_config.enetaddr, base, 6);
    else if (index == 1)
	rt_memcpy(gmac->gmac_config.enetaddr, base + 8, 6);
}

int rt_hw_gmac_init(void)
{
    int i;
    rt_err_t ret = RT_EOK;
    rt_thread_t link_detect;

    for (i = (GMAC_START + 1); i < GMAC_CNT; i++)
    {
        gmac_handle_t *gmac = &dw_gmac[i];

#if defined(BSP_USING_EXT_GMAC)
	if (i == GMAC_EXT_IDX) {
		ret = rtl_gmac_ops_init(gmac);
		if (ret < 0)
			break;
	} else
#endif
		eqos_gmac_ops_init(gmac);
        /* Register member functions */
        gmac->eth.parent.type       = RT_Device_Class_NetIf;
        gmac->eth.parent.init       = dw_gmac_init;
        gmac->eth.parent.open       = dw_gmac_open;
        gmac->eth.parent.close      = dw_gmac_close;
        gmac->eth.parent.read       = dw_gmac_read;
        gmac->eth.parent.write      = dw_gmac_write;
        gmac->eth.parent.control    = dw_gmac_control;
        gmac->eth.parent.user_data  = gmac;
        gmac->eth.eth_rx            = dw_gmac_rx;
        gmac->eth.eth_tx            = dw_gmac_tx;

        /* Initial zero_copy rx pool */
        memp_init_pool(gmac->memp_rx_pool);
	eth_device_linkchange(&gmac->eth, RT_FALSE);

#if defined(BSP_USING_EXT_GMAC)
	if (i != GMAC_EXT_IDX)
#endif
		get_gmac_addr_from_sharemem(gmac, gmac->id);

        /* Register eth device */
        ret = eth_device_init(&gmac->eth, gmac->name);

        RT_ASSERT(ret == RT_EOK);
    }

    link_detect = rt_thread_create("link_detect",
			phy_link_detect,
			NULL,
			4096,
			13,
			20);

    if (link_detect != RT_NULL)
	    rt_thread_startup(link_detect);

    return 0;
}
#endif /* HAL_GMAC_ENABLED */

