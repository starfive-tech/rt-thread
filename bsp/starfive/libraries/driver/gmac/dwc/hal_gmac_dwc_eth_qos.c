
#include <rtthread.h>

#include "board.h"
#include "hal_gmac.h"
#include "hal_gmac_dwc_eth_qos.h"
#include "motorcomm.h"
#include <riscv_io.h>
#include <io.h>

/* The `const' in roundup() prevents gcc-3.3 from calling __divdi3 */
#define roundup(x, y) (                    \
{                            \
    const typeof(y) __y = y;            \
    (((x) + (__y - 1)) / __y) * __y;        \
}                            \
)
#define rounddown(x, y) (                \
{                            \
    typeof(x) __x = (x);                \
    __x - (__x % (y));                \
}                            \
)

static int wait_for_bit(void *reg, const uint32_t mask, const int set, const unsigned int timeout_ms)
{
    uint32_t val;
    unsigned long start = sys_cur_time_ms();

    while (1)
    {
        val = sys_readl(reg);
        val = set ? val : ~val;

        if ((val & mask) == mask)
        {
            return 0;
        }

        if (start + timeout_ms < sys_cur_time_ms())
        {
            break;
        }

        sys_udelay(1);
    }

    return -1;
}

static uint8_t csr_clk_range_get(gmac_handle_t *gmac)
{
    uint32_t clk_m = 0;

    clk_m = sys_gmac_get_csr_clk(gmac->id);
    clk_m = clk_m/1000000;

    /*frequency range 1.0 MHz-2.5 MHz*/
    switch(clk_m)
    {
        case 0 ... 7:
            return EQOS_MAC_MDIO_ADDRESS_CR_DIV_4;
        case 8 ... 11:
            return EQOS_MAC_MDIO_ADDRESS_CR_DIV_6;
        case 12 ... 15:
            return EQOS_MAC_MDIO_ADDRESS_CR_DIV_8;
        case 16 ... 19:
            return EQOS_MAC_MDIO_ADDRESS_CR_DIV_10;
        case 20 ... 34:
            return EQOS_MAC_MDIO_ADDRESS_CR_20_35M;//div 16
        case 35 ... 59:
            return EQOS_MAC_MDIO_ADDRESS_CR_35_60M;//div 26
        case 60 ... 99:
            return EQOS_MAC_MDIO_ADDRESS_CR_60_100M;//div 42
        case 100 ... 149:
            return EQOS_MAC_MDIO_ADDRESS_CR_100_150M;//div 62
        case 150 ... 249:
            return EQOS_MAC_MDIO_ADDRESS_CR_150_250M;//div 102
        case 250 ... 299:
            return EQOS_MAC_MDIO_ADDRESS_CR_250_300M;//div 124
        case 300 ... 499:
            return EQOS_MAC_MDIO_ADDRESS_CR_300_500M;//div 204
        case 500 ... 800:
            return EQOS_MAC_MDIO_ADDRESS_CR_500_800M;//div 324
        default:
            return EQOS_MAC_MDIO_ADDRESS_CR_500_800M;//div 324
            // return 0;
    }
}

static int eqos_set_gmii_speed_1000(eqos_eth_dev_t *eqos_dev)
{
    clrbits(&eqos_dev->mac_regs->configuration, EQOS_MAC_CONFIGURATION_PS | EQOS_MAC_CONFIGURATION_FES);

    return 0;
}

static int eqos_set_mii_speed_100(eqos_eth_dev_t *eqos_dev)
{
    setbits(&eqos_dev->mac_regs->configuration, EQOS_MAC_CONFIGURATION_PS | EQOS_MAC_CONFIGURATION_FES);

    return 0;
}

static int eqos_set_mii_speed_10(eqos_eth_dev_t *eqos_dev)
{
    clrsetbits(&eqos_dev->mac_regs->configuration, EQOS_MAC_CONFIGURATION_FES, EQOS_MAC_CONFIGURATION_PS);

    return 0;
}

int eqos_set_speed_duplex(void *priv)
{
    eqos_eth_dev_t *eqos_dev = (void *)priv;
    int speed_mode, duplex;

    speed_mode = eqos_dev->handle->phy_dev->speed_mode;
    duplex = eqos_dev->handle->phy_dev->duplex;
    switch (speed_mode)
    {
    case GMAC_PHY_SPEED_10M:
            eqos_set_mii_speed_10(eqos_dev);
        break;
    case GMAC_PHY_SPEED_100M:
            eqos_set_mii_speed_100(eqos_dev);
        break;
    case GMAC_PHY_SPEED_1000M:
            eqos_set_gmii_speed_1000(eqos_dev);
        break;
    default:
        break;
    }

    if(duplex==GMAC_PHY_FULL_DUPLEX)
    {
        sys_modl(&eqos_dev->mac_regs->configuration, EQOS_MAC_CONFIGURATION_DM, EQOS_MAC_CONFIGURATION_DM);
    }
    else
    {
        sys_modl(&eqos_dev->mac_regs->configuration, EQOS_MAC_CONFIGURATION_DM, 0);
        /* WAR: Flush TX queue when switching to half-duplex */
        sys_modl(&eqos_dev->mtl_regs->txq0_operation_mode, EQOS_MTL_TXQ0_OPERATION_MODE_FTQ,
		EQOS_MTL_TXQ0_OPERATION_MODE_FTQ);
    }

    return 0;
}

static int eqos_gmac_isr(void *priv)
{
    eqos_eth_dev_t *eqos_dev = priv;
    struct eqos_int *x = &eqos_dev->int_summary;
    unsigned int intr_status, intr_en;
    int ret = 0;

    intr_status = sys_readl(&eqos_dev->dma_regs->ch0_status);
    intr_en = sys_readl(&eqos_dev->dma_regs->ch0_interrupt_enable);
    //LOG_DBG("int status %x int en %x\n", intr_status, intr_en);

	/* ABNORMAL interrupts */
    if (intr_status & DMA_CHAN_STATUS_AIS) {
	if ((intr_status & DMA_CHAN_STATUS_RBU))
	    x->rx_buf_unav_irq++;
	if ((intr_status & DMA_CHAN_STATUS_RPS))
	    x->rx_process_stopped_irq++;
	if ((intr_status & DMA_CHAN_STATUS_RWT))
	    x->rx_watchdog_irq++;
	if ((intr_status & DMA_CHAN_STATUS_ETI))
	    x->tx_early_irq++;
	if ((intr_status & DMA_CHAN_STATUS_TPS)) {
	    x->tx_process_stopped_irq++;
	    ret = INT_TX_HARD_ERROR;
	}
	if ((intr_status & DMA_CHAN_STATUS_FBE)) {
	    x->fatal_bus_error_irq++;
	    ret = INT_TX_HARD_ERROR;
	}
    }

    if (ret < 0) {
	sys_writel(intr_status & intr_en, &eqos_dev->dma_regs->ch0_status);
	return ret;
    }
    /* TX/RX NORMAL interrupts */
    if ((intr_status & DMA_CHAN_STATUS_NIS))
	x->normal_irq_n++;
    if ((intr_status & DMA_CHAN_STATUS_RI)) {
	x->rx_normal_irq_n++;
	ret |= INT_RX;
    }
    if ((intr_status & DMA_CHAN_STATUS_TI)) {
	x->tx_normal_irq_n++;
	ret |= INT_TX;
    }
    if ((intr_status & DMA_CHAN_STATUS_TBU)) {
	//x->txbuffer_not_aval++;
	ret |= INT_TX;
    } if ((intr_status & DMA_CHAN_STATUS_ERI))
	x->rx_early_irq++;

    sys_writel(intr_status & intr_en, &eqos_dev->dma_regs->ch0_status);
    return ret;
}

//static int eqos_mdio_read(eqos_eth_dev_t *eqos_dev, int addr, int devad, int reg)

static int eqos_mdio_read(void *bus, rt_uint32_t addr, rt_uint32_t reg, void *data, rt_uint32_t size)
{
    uint32_t mdio_val;
    int ret;
    uint8_t csr_clk_range = 0;
    eqos_eth_dev_t *eqos_dev = (void *)bus;

    ret = wait_for_bit(&eqos_dev->mac_regs->mdio_address, EQOS_MAC_MDIO_ADDRESS_GB, 0, 1000000);
    if (ret)
    {
	hal_printf("read time out\n");
        return ret;
    }

    csr_clk_range = csr_clk_range_get(eqos_dev->handle);
    mdio_val = sys_readl(&eqos_dev->mac_regs->mdio_address);
    mdio_val &= EQOS_MAC_MDIO_ADDRESS_SKAP |
        EQOS_MAC_MDIO_ADDRESS_C45E;
    mdio_val |= (addr << EQOS_MAC_MDIO_ADDRESS_PA_SHIFT) |
        (reg << EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT) |
        (csr_clk_range <<
         EQOS_MAC_MDIO_ADDRESS_CR_SHIFT) |
        (EQOS_MAC_MDIO_ADDRESS_GOC_READ <<
         EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT) |
        EQOS_MAC_MDIO_ADDRESS_GB;
    sys_writel(mdio_val, &eqos_dev->mac_regs->mdio_address);
    
    sys_udelay(10);

    ret = wait_for_bit(&eqos_dev->mac_regs->mdio_address, EQOS_MAC_MDIO_ADDRESS_GB, 0, 1000000);
    if (ret) {
	hal_printf("read time out 1\n");
        return ret;
    }

    mdio_val = sys_readl(&eqos_dev->mac_regs->mdio_data);
    mdio_val &= EQOS_MAC_MDIO_DATA_GD_MASK;
    *(uint32_t *)data = mdio_val;

    return 0;
}

static int eqos_mdio_write(void *bus, rt_uint32_t addr, rt_uint32_t reg, void *data, rt_uint32_t size)
{
    uint32_t mdio_val;
    int ret;
    uint8_t csr_clk_range = 0;
    eqos_eth_dev_t *eqos_dev = (void *)bus;

    ret = wait_for_bit(&eqos_dev->mac_regs->mdio_address, EQOS_MAC_MDIO_ADDRESS_GB, 0, 1000000);
    if (ret) {
        return ret;
    }

    sys_writel(*(rt_uint16_t *)data, &eqos_dev->mac_regs->mdio_data);

    csr_clk_range = csr_clk_range_get(eqos_dev->handle);
    mdio_val = sys_readl(&eqos_dev->mac_regs->mdio_address);
    mdio_val &= EQOS_MAC_MDIO_ADDRESS_SKAP |
        EQOS_MAC_MDIO_ADDRESS_C45E;
    mdio_val |= (addr << EQOS_MAC_MDIO_ADDRESS_PA_SHIFT) |
        (reg << EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT) |
        (csr_clk_range <<
         EQOS_MAC_MDIO_ADDRESS_CR_SHIFT) |
        (EQOS_MAC_MDIO_ADDRESS_GOC_WRITE <<
         EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT) |
        EQOS_MAC_MDIO_ADDRESS_GB;
    sys_writel(mdio_val, &eqos_dev->mac_regs->mdio_address);

    sys_udelay(10);

    ret = wait_for_bit(&eqos_dev->mac_regs->mdio_address, EQOS_MAC_MDIO_ADDRESS_GB, 0, 1000000);
    if (ret) {
        return ret;
    }

    return 0;
}

static int eqos_check_descriptor(void *priv, void **buffer)
{
    eqos_eth_dev_t *eqos_dev = (void *)priv;
    eqos_desc_t *tx_desc;

    tx_desc = &(eqos_dev->tx_descs[eqos_dev->tx_desc_idx]);

    if (tx_desc->des3 & EQOS_DESC3_OWN) {
	hal_printf("current tx discriptor not avail %d", eqos_dev->tx_desc_idx);
	return -1;
    }

    *buffer = eqos_dev->tx_dma_buf[eqos_dev->tx_desc_idx];
    return 0;
}

static int eqos_send(void *priv, int length)
{
    eqos_eth_dev_t *eqos_dev = (void *)priv;
    eqos_desc_t *tx_desc;
    int i;

    sys_gmac_flush_dcache_range(rounddown((unsigned long)eqos_dev->tx_dma_buf, ARCH_DMA_MINALIGN),
        roundup((unsigned long)eqos_dev->tx_dma_buf + length, ARCH_DMA_MINALIGN));

    tx_desc = &(eqos_dev->tx_descs[eqos_dev->tx_desc_idx]);
    tx_desc->des0 = (unsigned long)eqos_dev->tx_dma_buf[eqos_dev->tx_desc_idx];
    tx_desc->des1 = 0;
    tx_desc->des2 = EQOS_DESC2_IOC | length;
    /*
     * Make sure that if HW sees the _OWN write below, it will see all the
     * writes to the rest of the descriptor too.
     */
    mb();
    tx_desc->des3 = EQOS_DESC3_OWN | EQOS_DESC3_FD | EQOS_DESC3_LD | length;

    sys_gmac_flush_dcache_range(rounddown((unsigned long)tx_desc, ARCH_DMA_MINALIGN),
        roundup((unsigned long)tx_desc + EQOS_DESCRIPTOR_SIZE, ARCH_DMA_MINALIGN));

    sys_writel((unsigned long)(&(eqos_dev->tx_descs[eqos_dev->tx_desc_idx])),
	&eqos_dev->dma_regs->ch0_txdesc_tail_pointer);

    eqos_dev->tx_desc_idx++;
    eqos_dev->tx_desc_idx &= (EQOS_DESCRIPTORS_TX - 1);

    return 0;
}

static int eqos_recv(void *priv, void **packetp)
{
    eqos_eth_dev_t *eqos_dev = priv;
    eqos_desc_t *rx_desc;
    int length;

    rx_desc = &(eqos_dev->rx_descs[eqos_dev->rxbusy]);
    sys_gmac_invalidate_cache_range(rounddown((unsigned long)rx_desc, ARCH_DMA_MINALIGN),
        roundup((unsigned long)rx_desc + EQOS_DESCRIPTOR_SIZE, ARCH_DMA_MINALIGN));
    if (rx_desc->des3 & EQOS_DESC3_OWN) {
        //hal_printf("RX packet not available %d\n", eqos_dev->rxbusy);
        return -1;
    }
    length = rx_desc->des3 & 0x7fff;
    if (!length) {
	hal_printf("RX packet len 0 %d\n", eqos_dev->rxbusy);
	eqos_dev->rxbusy++;
	eqos_dev->rxbusy &= (EQOS_DESCRIPTORS_RX - 1);
	eqos_dev->rxnext++;
	eqos_dev->rxnext &= (EQOS_DESCRIPTORS_RX - 1);
	return -1;
    }
    rx_desc->des0 = 0;
    mb();
    rx_desc->des3 = 0;

    *packetp = eqos_dev->rx_dma_buf[eqos_dev->rxbusy];

    eqos_dev->rxbusy++;
    eqos_dev->rxbusy &= (EQOS_DESCRIPTORS_RX - 1);
    eqos_dev->busyrxdesc--;
    //printf("%s: *packetp=%p, length=%d\n", __func__, *packetp, length);
    // printf("CRC Err:%d  LT:%d\n", (rx_desc->des3&BIT(24) != 0), (rx_desc->des3>>16)&0x7);
#if 0
    sys_gmac_invalidate_cache_range(rounddown((unsigned long)*packetp, ARCH_DMA_MINALIGN),
        roundup((unsigned long)*packetp + length, ARCH_DMA_MINALIGN));
#endif
    return length;
}

static int eqos_free_pkt(void *priv, void *packet)
{
    eqos_eth_dev_t *eqos_dev = (void *)priv;
    void *packet_expected;
    eqos_desc_t *rx_desc;
    char rx_next;

    LOG_DBG("%s rx busy %d rx next %d busydesc %d\n", __func__,
	eqos_dev->rxbusy, eqos_dev->rxnext, eqos_dev->busyrxdesc);

    rx_next = eqos_dev->rxnext;
    packet_expected = eqos_dev->rx_dma_buf[eqos_dev->rxnext];
    if (packet != packet_expected) {
        hal_printf("Unexpected packet (expected %p)\n",
              packet_expected);
        return -1;
    }

    rx_desc = &(eqos_dev->rx_descs[eqos_dev->rxnext]);
    rx_desc->des0 = 0;
    mb();

    rx_desc->des0 = (uint32_t)(unsigned long)packet;
    rx_desc->des1 = 0;
    rx_desc->des2 = 0;
    /*
     * Make sure that if HW sees the _OWN write below, it will see all the
     * writes to the rest of the descriptor too.
     */
    mb();
    rx_desc->des3 = EQOS_DESC3_OWN | EQOS_DESC3_BUF1V | EQOS_DESC3_IOC;

    sys_gmac_flush_dcache_range(rounddown((unsigned long)rx_desc, ARCH_DMA_MINALIGN),
        roundup((unsigned long)rx_desc + EQOS_DESCRIPTOR_SIZE, ARCH_DMA_MINALIGN));
    sys_writel((unsigned long)rx_desc, &eqos_dev->dma_regs->ch0_rxdesc_tail_pointer);

    eqos_dev->rxnext++;
    eqos_dev->rxnext &= (EQOS_DESCRIPTORS_RX - 1);
    eqos_dev->busyrxdesc++;

    return rx_next;
}

static int eqos_write_hwaddr(eqos_eth_dev_t *eqos_dev,const uint8_t *mac_id)
{
    uint32_t val;

    /*
     * This function may be called before start() or after stop(). At that
     * time, on at least some configurations of the EQoS HW, all clocks to
     * the EQoS HW block will be stopped, and a reset signal applied. If
     * any register access is attempted in this state, bus timeouts or CPU
     * hangs may occur. This check prevents that.
     *
     * A simple solution to this problem would be to not implement
     * write_hwaddr(), since start() always writes the MAC address into HW
     * anyway. However, it is desirable to implement write_hwaddr() to
     * support the case of SW that runs subsequent to U-Boot which expects
     * the MAC address to already be programmed into the EQoS registers,
     * which must happen irrespective of whether the U-Boot user (or
     * scripts) actually made use of the EQoS device, and hence
     * irrespective of whether start() was ever called.
     *
     * Note that this requirement by subsequent SW is not valid for
     * Tegra186, and is likely not valid for any non-PCI instantiation of
     * the EQoS HW block. This function is implemented solely as
     * future-proofing with the expectation the driver will eventually be
     * ported to some system where the expectation above is true.
     */

    /* Update the MAC address */
    val = (mac_id[5] << 8) |
        (mac_id[4]) | (1 << 31);
    sys_writel(val, &eqos_dev->mac_regs->address0_high);
    val = (mac_id[3] << 24) |
        (mac_id[2] << 16) |
        (mac_id[1] << 8) |
        (mac_id[0]);
    sys_writel(val, &eqos_dev->mac_regs->address0_low);

    return 0;
}

static int eqos_resources_malloc(eqos_eth_dev_t *eqos_dev)
{
    int ret;
    int i;

    eqos_dev->descs_noalign = hal_malloc(EQOS_DESCRIPTORS_SIZE  + EQOS_DESCRIPTOR_ALIGN);//memalign(EQOS_DESCRIPTOR_ALIGN, EQOS_DESCRIPTORS_SIZE);
    if (!eqos_dev->descs_noalign) {
        hal_printf("%s: eqos_alloc_descs() failed\n", __func__);
        ret = -1;
        goto err;
    }
    eqos_dev->descs = (void *)ALIGN((unsigned long)eqos_dev->descs_noalign, EQOS_DESCRIPTOR_ALIGN);
    eqos_dev->tx_descs = (eqos_desc_t *)eqos_dev->descs;
    eqos_dev->rx_descs = (eqos_dev->tx_descs + EQOS_DESCRIPTORS_TX);

    for (i = 0; i < EQOS_DESCRIPTORS_TX; i++) {
	eqos_dev->tx_dma_buf_noalign[i] = hal_malloc(EQOS_MAX_PACKET_SIZE + EQOS_BUFFER_ALIGN);//memalign(EQOS_BUFFER_ALIGN, EQOS_MAX_PACKET_SIZE);
    	if (!eqos_dev->tx_dma_buf_noalign) {
	    hal_printf("%s: memalign(tx_dma_buf) failed\n", __func__);
            ret = -1;
            goto err_free_descs;
        }
	eqos_dev->tx_dma_buf[i] = (void *)ALIGN((unsigned long)eqos_dev->tx_dma_buf_noalign[i], EQOS_BUFFER_ALIGN);
    }

    for (i = 0; i < EQOS_DESCRIPTORS_RX; i++) {
	eqos_dev->rx_dma_buf_noalign[i] = hal_malloc(EQOS_RX_BUFFER_SIZE + EQOS_BUFFER_ALIGN);//memalign(EQOS_BUFFER_ALIGN, EQOS_MAX_PACKET_SIZE);
	if (!eqos_dev->rx_dma_buf_noalign[i]) {
            hal_printf("%s: memalign(rx_dma_buf) failed\n", __func__);
            ret = -1;
            goto err_free_tx_dma_buf;
        }
	eqos_dev->rx_dma_buf[i] = (void *)ALIGN((unsigned long)eqos_dev->rx_dma_buf_noalign[i], EQOS_BUFFER_ALIGN);
    }

    //sys_gmac_invalidate_cache_range(rounddown((unsigned long)eqos_dev->rx_dma_buf, ARCH_DMA_MINALIGN), roundup((unsigned long)eqos_dev->rx_dma_buf + EQOS_RX_BUFFER_SIZE, ARCH_DMA_MINALIGN));

    return 0;

err_free_tx_dma_buf:
    for (i = 0; i < EQOS_DESCRIPTORS_TX; i++) {
	hal_free(eqos_dev->tx_dma_buf_noalign[i]);
    }
err_free_descs:
    hal_free(eqos_dev->descs_noalign);
err:

    return ret;
}

static int eqos_resources_free(eqos_eth_dev_t *eqos_dev)
{
    int i;

    if(eqos_dev->descs != NULL)
    {
        hal_free(eqos_dev->descs_noalign);
        eqos_dev->descs_noalign = NULL;
        eqos_dev->descs = NULL;
        eqos_dev->tx_descs = NULL;
        eqos_dev->rx_descs = NULL;
    }
    for (i = 0; i < EQOS_DESCRIPTORS_TX; i++)
	if (eqos_dev->tx_dma_buf[i])
	{
            hal_free(eqos_dev->tx_dma_buf_noalign[i]);
            eqos_dev->tx_dma_buf_noalign[i] = NULL;
            eqos_dev->tx_dma_buf[i] = NULL;
	}

    for (i = 0; i < EQOS_DESCRIPTORS_RX; i++)
	if (eqos_dev->rx_dma_buf[i])
        {
	    hal_free(eqos_dev->rx_dma_buf_noalign[i]);
	    eqos_dev->rx_dma_buf_noalign[i] = NULL;
	    eqos_dev->rx_dma_buf[i] = NULL;
	}

    return 0;
}

static void eqos_eth_trans_ctrl(eqos_eth_dev_t *eqos_dev, int en)
{
    int i;

    if(en)
    {
        setbits(&eqos_dev->dma_regs->ch0_tx_control,
             EQOS_DMA_CH0_TX_CONTROL_ST);
        setbits(&eqos_dev->dma_regs->ch0_rx_control,
             EQOS_DMA_CH0_RX_CONTROL_SR);
        setbits(&eqos_dev->mac_regs->configuration,
            EQOS_MAC_CONFIGURATION_TE | EQOS_MAC_CONFIGURATION_RE);
    }
    else
    {
        /* Disable TX DMA */
        clrbits(&eqos_dev->dma_regs->ch0_tx_control,
             EQOS_DMA_CH0_TX_CONTROL_ST);

        /* Wait for TX all packets to drain out of MTL */
        for (i = 0; i < 1000000; i++) {
            rt_uint32_t val = sys_readl(&eqos_dev->mtl_regs->txq0_debug);
            rt_uint32_t trcsts = (val >> EQOS_MTL_TXQ0_DEBUG_TRCSTS_SHIFT) &
                EQOS_MTL_TXQ0_DEBUG_TRCSTS_MASK;
            rt_uint32_t txqsts = val & EQOS_MTL_TXQ0_DEBUG_TXQSTS;
            if ((trcsts != 1) && (!txqsts))
                break;
        }

        /* Turn off MAC TX and RX */
        clrbits(&eqos_dev->mac_regs->configuration,
                EQOS_MAC_CONFIGURATION_TE | EQOS_MAC_CONFIGURATION_RE);

        /* Wait for all RX packets to drain out of MTL */
        for (i = 0; i < 1000000; i++) {
            rt_uint32_t val = sys_readl(&eqos_dev->mtl_regs->rxq0_debug);
            rt_uint32_t prxq = (val >> EQOS_MTL_RXQ0_DEBUG_PRXQ_SHIFT) &
                EQOS_MTL_RXQ0_DEBUG_PRXQ_MASK;
            rt_uint32_t rxqsts = (val >> EQOS_MTL_RXQ0_DEBUG_RXQSTS_SHIFT) &
                EQOS_MTL_RXQ0_DEBUG_RXQSTS_MASK;
            if ((!prxq) && (!rxqsts))
                break;
        }

        /* Turn off RX DMA */
        clrbits(&eqos_dev->dma_regs->ch0_rx_control,
                EQOS_DMA_CH0_RX_CONTROL_SR);
    }
}

static int eqos_eth_init(eqos_eth_dev_t *eqos_dev)
{
    int ret, i;
    unsigned long rate;
    unsigned int val, tx_fifo_sz, rx_fifo_sz, tqs, rqs, pbl;
    unsigned long last_rx_desc;

    ret = eqos_resources_malloc(eqos_dev);
    if(ret)
    {
        hal_printf("eqos_resources_malloc failed\n");
        return -1;
    }

    ret = wait_for_bit(&eqos_dev->dma_regs->mode, EQOS_DMA_MODE_SWR, 0, 10000);
    if (ret) {
        hal_printf("EQOS_DMA_MODE_SWR stuck\n");
        goto err;
    }
    //set speed and duplex
    eqos_set_speed_duplex(eqos_dev);

    /* Configure MTL */
    //FIXME:uboot code,don't why use it, so comment the code
    // sys_writel(0x60, &eqos_dev->mtl_regs->txq0_quantum_weight - 0x100);
    /* Enable Store and Forward mode for TX */
    /* Program Tx operating mode */
    setbits(&eqos_dev->mtl_regs->txq0_operation_mode,
             EQOS_MTL_TXQ0_OPERATION_MODE_TSF |
             (EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_ENABLED <<
              EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT));

    /* Transmit Queue weight */
    sys_writel(0x10, &eqos_dev->mtl_regs->txq0_quantum_weight);

    /* Enable Store and Forward mode for RX, since no jumbo frame */
    setbits(&eqos_dev->mtl_regs->rxq0_operation_mode,
        EQOS_MTL_RXQ0_OPERATION_MODE_RSF |
        EQOS_MTL_RXQ0_OPERATION_MODE_FEP |
        EQOS_MTL_RXQ0_OPERATION_MODE_FUP);

    /* Transmit/Receive queue fifo size; use all RAM for 1 queue */
    val = sys_readl(&eqos_dev->mac_regs->hw_feature1);
    tx_fifo_sz = (val >> EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT) &
        EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_MASK;
    rx_fifo_sz = (val >> EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT) &
        EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_MASK;

    /*
     * r/tx_fifo_sz is encoded as log2(n / 128). Undo that by shifting.
     * r/tqs is encoded as (n / 256) - 1.
     */
    tqs = (128 << tx_fifo_sz) / 256 - 1;
    rqs = (128 << rx_fifo_sz) / 256 - 1;

    clrsetbits(&eqos_dev->mtl_regs->txq0_operation_mode,
            EQOS_MTL_TXQ0_OPERATION_MODE_TQS_MASK <<
            EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT,
            tqs << EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
    clrsetbits(&eqos_dev->mtl_regs->rxq0_operation_mode,
            EQOS_MTL_RXQ0_OPERATION_MODE_RQS_MASK <<
            EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT,
            rqs << EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT);

    /* Flow control used only if each channel gets 4KB or more FIFO */
    if (rqs >= ((4096 / 256) - 1)) {
        rt_uint32_t rfd, rfa;

        setbits(&eqos_dev->mtl_regs->rxq0_operation_mode,
                 EQOS_MTL_RXQ0_OPERATION_MODE_EHFC);

        /*
         * Set Thr eshold for Activating Flow Contol space for min 2
         * frames ie, (1500 * 1) = 1500 bytes.
         *
         * Set Threshold for Deactivating Flow Contol for space of
         * min 1 frame (frame size 1500bytes) in receive fifo
         */
        if (rqs == ((4096 / 256) - 1)) {
            /*
             * This violates the above formula because of FIFO size
             * limit therefore overflow may occur inspite of this.
             */
            rfd = 0x3;  /* Full-3K */
            rfa = 0x1;  /* Full-1.5K */
        } else if (rqs == ((8192 / 256) - 1)) {
            rfd = 0x6;  /* Full-4K */
            rfa = 0xa;  /* Full-6K */
        } else if (rqs == ((16384 / 256) - 1)) {
            rfd = 0x6;  /* Full-4K */
            rfa = 0x12; /* Full-10K */
        } else {
            rfd = 0x6;  /* Full-4K */
            rfa = 0x1E; /* Full-16K */
        }

        clrsetbits(&eqos_dev->mtl_regs->rxq0_operation_mode,
                (EQOS_MTL_RXQ0_OPERATION_MODE_RFD_MASK <<
                 EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT) |
                (EQOS_MTL_RXQ0_OPERATION_MODE_RFA_MASK <<
                 EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT),
                (rfd <<
                 EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT) |
                (rfa <<
                 EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT));
    }

    /* Configure MAC */

    clrsetbits(&eqos_dev->mac_regs->rxq_ctrl0,
            EQOS_MAC_RXQ_CTRL0_RXQ0EN_MASK <<
            EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT,
            EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB <<
            EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT);

    /* Multicast and Broadcast Queue Enable */
    setbits(&eqos_dev->mac_regs->unused_0a4,
             BIT(20));
    /* enable promise mode */
    setbits(&eqos_dev->mac_regs->unused_004[1],
             BIT(0));

    /* Set TX flow control parameters */
    /* Set Pause Time */
    setbits(&eqos_dev->mac_regs->q0_tx_flow_ctrl,
             0xffff << EQOS_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT);
    /* Assign priority for TX flow control */
    clrbits(&eqos_dev->mac_regs->txq_prty_map0,
             EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_MASK <<
             EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_SHIFT);
    /* Assign priority for RX flow control */
    clrbits(&eqos_dev->mac_regs->rxq_ctrl2,
             EQOS_MAC_RXQ_CTRL2_PSRQ0_MASK <<
             EQOS_MAC_RXQ_CTRL2_PSRQ0_SHIFT);
    /* Enable flow control */
    setbits(&eqos_dev->mac_regs->q0_tx_flow_ctrl,
             EQOS_MAC_Q0_TX_FLOW_CTRL_TFE);
    setbits(&eqos_dev->mac_regs->rx_flow_ctrl,
             EQOS_MAC_RX_FLOW_CTRL_RFE);

    // clrsetbits(&eqos_dev->mac_regs->configuration,
    //         EQOS_MAC_CONFIGURATION_GPSLCE |
    //         EQOS_MAC_CONFIGURATION_WD |
    //         EQOS_MAC_CONFIGURATION_JD |
    //         EQOS_MAC_CONFIGURATION_JE,
    //         EQOS_MAC_CONFIGURATION_CST |
    //         EQOS_MAC_CONFIGURATION_ACS);
    clrbits(&eqos_dev->mac_regs->configuration,
            EQOS_MAC_CONFIGURATION_GPSLCE |
            EQOS_MAC_CONFIGURATION_WD |
            EQOS_MAC_CONFIGURATION_JD |
            EQOS_MAC_CONFIGURATION_JE);

    eqos_write_hwaddr(eqos_dev, eqos_dev->handle->gmac_config.enetaddr);

    /* Configure DMA */

    /* Enable OSP mode */
    setbits(&eqos_dev->dma_regs->ch0_tx_control,
             EQOS_DMA_CH0_TX_CONTROL_OSP);

    /* RX buffer size. Must be a multiple of bus width */
    clrsetbits(&eqos_dev->dma_regs->ch0_rx_control,
            EQOS_DMA_CH0_RX_CONTROL_RBSZ_MASK <<
            EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT,
            EQOS_MAX_PACKET_SIZE <<
            EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT);

    setbits(&eqos_dev->dma_regs->ch0_control,
             EQOS_DMA_CH0_CONTROL_PBLX8);

    /*
     * Burst length must be < 1/2 FIFO size.
     * FIFO size in tqs is encoded as (n / 256) - 1.
     * Each burst is n * 8 (PBLX8) * 16 (AXI width) == 128 bytes.
     * Half of n * 256 is n * 128, so pbl == tqs, modulo the -1.
     */
    pbl = tqs + 1;
    if (pbl > 32)
        pbl = 32;
    clrsetbits(&eqos_dev->dma_regs->ch0_tx_control,
            EQOS_DMA_CH0_TX_CONTROL_TXPBL_MASK <<
            EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT,
            pbl << EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT);

    clrsetbits(&eqos_dev->dma_regs->ch0_rx_control,
            EQOS_DMA_CH0_RX_CONTROL_RXPBL_MASK <<
            EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT,
            8 << EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT);

    /* DMA performance configuration */
    val = (2 << EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT) |
        EQOS_DMA_SYSBUS_MODE_EAME | EQOS_DMA_SYSBUS_MODE_BLEN16 |
        EQOS_DMA_SYSBUS_MODE_BLEN8 | EQOS_DMA_SYSBUS_MODE_BLEN4;
    sys_writel(val, &eqos_dev->dma_regs->sysbus_mode);

    /* Set up descriptors */

    for (i = 0; i < EQOS_DESCRIPTORS_RX; i++) {
        struct eqos_desc *rx_desc = &(eqos_dev->rx_descs[i]);
	memset(rx_desc, 0, EQOS_DESCRIPTORS_SIZE);
        rx_desc->des0 = (rt_uint32_t)(unsigned long)(eqos_dev->rx_dma_buf[i]);
        rx_desc->des3 |= EQOS_DESC3_OWN | EQOS_DESC3_BUF1V | EQOS_DESC3_IOC;

        mb();
	eqos_dev->busyrxdesc++;
        sys_gmac_flush_dcache_range(rounddown((unsigned long)rx_desc, ARCH_DMA_MINALIGN),
		roundup((unsigned long)rx_desc + EQOS_DESCRIPTOR_SIZE, ARCH_DMA_MINALIGN));
    }
    sys_gmac_flush_dcache_range(rounddown((unsigned long)eqos_dev->descs, ARCH_DMA_MINALIGN), roundup((unsigned long)eqos_dev->descs + EQOS_DESCRIPTORS_SIZE, ARCH_DMA_MINALIGN));

    sys_writel(0, &eqos_dev->dma_regs->ch0_txdesc_list_haddress);
    sys_writel((unsigned long)eqos_dev->tx_descs, &eqos_dev->dma_regs->ch0_txdesc_list_address);
    sys_writel(EQOS_DESCRIPTORS_TX - 1,
           &eqos_dev->dma_regs->ch0_txdesc_ring_length);

    sys_writel(0, &eqos_dev->dma_regs->ch0_rxdesc_list_haddress);
    sys_writel((unsigned long)eqos_dev->rx_descs, &eqos_dev->dma_regs->ch0_rxdesc_list_address);
    sys_writel(EQOS_DESCRIPTORS_RX - 1,
           &eqos_dev->dma_regs->ch0_rxdesc_ring_length);

    /* Enable everything */
    setbits(&eqos_dev->dma_regs->ch0_tx_control,
             EQOS_DMA_CH0_TX_CONTROL_ST);
    setbits(&eqos_dev->dma_regs->ch0_rx_control,
             EQOS_DMA_CH0_RX_CONTROL_SR);
    setbits(&eqos_dev->mac_regs->configuration,
            EQOS_MAC_CONFIGURATION_TE | EQOS_MAC_CONFIGURATION_RE);

    /* TX tail pointer not written until we need to TX a packet */
    /*
     * Point RX tail pointer at last descriptor. Ideally, we'd point at the
     * first descriptor, implying all descriptors were available. However,
     * that's not distinguishable from none of the descriptors being
     * available.
     */
    last_rx_desc = (unsigned long)&(eqos_dev->rx_descs[(EQOS_DESCRIPTORS_RX - 1)]);
    sys_writel(last_rx_desc, &eqos_dev->dma_regs->ch0_rxdesc_tail_pointer);

    return 0;

err:
    eqos_resources_free(eqos_dev);
    return ret;
}

/**********public interface**********/
static gmac_handle_t* eqos_open(gmac_handle_t *gmac)
{
    eqos_eth_dev_t *eqos_dev;
    int ret;

    /*
     * Since the priv structure contains the descriptors which need a strict
     * buswidth alignment, memalign is used to allocate memory
     */
    eqos_dev = (eqos_eth_dev_t *)hal_malloc(sizeof(eqos_eth_dev_t));//(struct dw_eth_dev *) memalign(ARCH_DMA_MINALIGN,
    if (!eqos_dev) {
 	hal_free(gmac);
        hal_printf("malloc mem failed!\n");
        return NULL;
    }
#if 0
    phy_dev = (void *)hal_malloc(sizeof(struct rt_phy_device));
    if (!phy_dev) {
        return NULL;
    }
    ret = rt_hw_phy_register(phy_dev, "yt8531_phy");
    if (ret) {
    	hal_free(gmac);
        return NULL;
    }
    phy_rt_dev = &phy_dev->parent;
#endif
    //rt_memset(gmac, 0, sizeof(gmac_handle_t));
    memset(eqos_dev, 0, sizeof(eqos_eth_dev_t));

    gmac->priv = eqos_dev;
    gmac_plat_init(gmac);
    eqos_dev->handle = gmac;
    eqos_dev->mac_regs = (void *)(gmac->base + EQOS_MAC_REGS_BASE);
    eqos_dev->mtl_regs = (void *)(gmac->base + EQOS_MTL_REGS_BASE);
    eqos_dev->dma_regs = (void *)(gmac->base + EQOS_DMA_REGS_BASE);
    eqos_dev->tegra186_regs = (void *)(gmac->base + EQOS_TEGRA186_REGS_BASE);

    ret = genric_gmac_phy_init(gmac);
    if (ret)
	return NULL;

    if(eqos_eth_init(eqos_dev))
    {
        gmac_close(gmac);
        return NULL;
    }

    /* dma interrupt enable */
    sys_writel(DMA_CHAN_INTR_NORMAL | DMA_CHAN_INTR_ABNORMAL,
    	&eqos_dev->dma_regs->ch0_interrupt_enable);

    return gmac;

}

int gmac_close(gmac_handle_t *gmac)
{
    //gmac_eth_int_register(gmac, NULL, NULL, 0);
    //sys_gmac_deinit(gmac->id);

    eqos_resources_free(gmac->priv);
    hal_free(gmac->priv);
    hal_free(gmac->phy_dev);
    //hal_free(gmac);

    return 0;
}

int gmac_mdio_write(gmac_handle_t *gmac, int reg, unsigned int data)
{
    return eqos_mdio_write(gmac->priv, gmac->gmac_config.phy_addr, reg, &data, 2);
}

int gmac_mdio_read(gmac_handle_t *gmac, int reg, void *data)
{
    return eqos_mdio_read(gmac->priv, gmac->gmac_config.phy_addr, reg, data, 2);
}

int gmac_set_macaddr(gmac_handle_t *gmac, const uint8_t *mac_id)
{
    if(mac_id == NULL)
    {
        return -1;
    }

    memcpy(gmac->gmac_config.enetaddr, mac_id, sizeof(gmac->gmac_config.enetaddr));
    eqos_eth_trans_ctrl(gmac->priv, 0);
    eqos_write_hwaddr(gmac->priv, mac_id);
    eqos_eth_trans_ctrl(gmac->priv, 1);
    return 0;
}

static struct gmac_ops eqos_ops = {
    .open = eqos_open,
    .recv = eqos_recv,
    .send = eqos_send,
    .check_descriptor = eqos_check_descriptor,
    .free_pkt = eqos_free_pkt,
    .gmac_isr = eqos_gmac_isr,
    .set_speed_and_duplex = eqos_set_speed_duplex,
};

void eqos_gmac_ops_init(gmac_handle_t *handle)
{
    handle->ops = &eqos_ops;
}

