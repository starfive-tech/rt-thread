/**
 ******************************************************************************
 * @copyright Copyright (c) 2020 StarFive Technology Co.,Ltd.
 * @file gmac_dwc_eth_qos.h
 * @author StarFive FW Team
 * @brief
 ******************************************************************************
 */
#ifndef __HAL_GMAC_DWC_ETH_QOS_H_
#define __HAL_GMAC_DWC_ETH_QOS_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*******start add for compiler********/
#ifndef ARCH_DMA_MINALIGN
#define ARCH_DMA_MINALIGN   64
#endif


/* Core registers */

#define EQOS_MAC_REGS_BASE 0x000
typedef struct gmac_mac_regs {
    uint32_t configuration;             /* 0x000 */
    uint32_t unused_004[(0x070 - 0x004) / 4];   /* 0x004 */
    uint32_t q0_tx_flow_ctrl;           /* 0x070 */
    uint32_t unused_070[(0x090 - 0x074) / 4];   /* 0x074 */
    uint32_t rx_flow_ctrl;              /* 0x090 */
    uint32_t unused_094;                /* 0x094 */
    uint32_t txq_prty_map0;             /* 0x098 */
    uint32_t unused_09c;                /* 0x09c */
    uint32_t rxq_ctrl0;             /* 0x0a0 */
    uint32_t unused_0a4;                /* 0x0a4 */
    uint32_t rxq_ctrl2;             /* 0x0a8 */
    uint32_t unused_0ac[(0x0c0 - 0x0ac) / 4];   /* 0x0ac */
    uint32_t pmt_ctrl_status;            /* 0x0c0 */
    uint32_t unused_0c4[(0x0dc - 0x0c4) / 4];   /* 0x0c4 */
    uint32_t us_tic_counter;            /* 0x0dc */
    uint32_t unused_0e0[(0x11c - 0x0e0) / 4];   /* 0x0e0 */
    uint32_t hw_feature0;               /* 0x11c */
    uint32_t hw_feature1;               /* 0x120 */
    uint32_t hw_feature2;               /* 0x124 */
    uint32_t unused_128[(0x200 - 0x128) / 4];   /* 0x128 */
    uint32_t mdio_address;              /* 0x200 */
    uint32_t mdio_data;             /* 0x204 */
    uint32_t unused_208[(0x300 - 0x208) / 4];   /* 0x208 */
    uint32_t address0_high;             /* 0x300 */
    uint32_t address0_low;              /* 0x304 */
}gmac_mac_regs_t;


#define EQOS_MAC_CONFIGURATION_GPSLCE           BIT(23)
#define EQOS_MAC_CONFIGURATION_CST          BIT(21)
#define EQOS_MAC_CONFIGURATION_ACS          BIT(20)
#define EQOS_MAC_CONFIGURATION_WD           BIT(19)
#define EQOS_MAC_CONFIGURATION_JD           BIT(17)
#define EQOS_MAC_CONFIGURATION_JE           BIT(16)
#define EQOS_MAC_CONFIGURATION_PS           BIT(15)
#define EQOS_MAC_CONFIGURATION_FES          BIT(14)
#define EQOS_MAC_CONFIGURATION_DM           BIT(13)
#define EQOS_MAC_CONFIGURATION_LM            BIT(12)
#define EQOS_MAC_CONFIGURATION_TE           BIT(1)
#define EQOS_MAC_CONFIGURATION_RE           BIT(0)

#define EQOS_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT       16
#define EQOS_MAC_Q0_TX_FLOW_CTRL_PT_MASK        0xffff
#define EQOS_MAC_Q0_TX_FLOW_CTRL_TFE            BIT(1)

#define EQOS_MAC_RX_FLOW_CTRL_RFE           BIT(0)

#define EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_SHIFT      0
#define EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_MASK       0xff

#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT         0
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_MASK          3
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_NOT_ENABLED       0
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB       2
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_AV        1

#define EQOS_MAC_RXQ_CTRL2_PSRQ0_SHIFT          0
#define EQOS_MAC_RXQ_CTRL2_PSRQ0_MASK           0xff

#define EQOS_MAC_PMT_CTRL_STATUS_PWRDWN         BIT(0)
#define EQOS_MAC_PMT_CTRL_STATUS_MGKPKTEN       BIT(1)
#define EQOS_MAC_PMT_CTRL_STATUS_RWKPKTEN       BIT(2)

#define EQOS_MAC_HW_FEATURE0_MMCSEL_SHIFT        8
#define EQOS_MAC_HW_FEATURE0_HDSEL_SHIFT        2
#define EQOS_MAC_HW_FEATURE0_GMIISEL_SHIFT        1
#define EQOS_MAC_HW_FEATURE0_MIISEL_SHIFT        0
#define EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT       6
#define EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_MASK        0x1f
#define EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT       0
#define EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_MASK        0x1f

#define EQOS_MAC_HW_FEATURE3_ASP_SHIFT            28
#define EQOS_MAC_HW_FEATURE3_ASP_MASK            0x3
#define EQOS_MAC_MDIO_ADDRESS_PA_SHIFT          21
#define EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT         16

/* MDIO address register CR definitions */
#define EQOS_MAC_MDIO_ADDRESS_CR_SHIFT          8
#define EQOS_MAC_MDIO_ADDRESS_CR_60_100M        (0)
#define EQOS_MAC_MDIO_ADDRESS_CR_100_150M       (0x1)
#define EQOS_MAC_MDIO_ADDRESS_CR_20_35M         (0x2)
#define EQOS_MAC_MDIO_ADDRESS_CR_35_60M         (0x3)
#define EQOS_MAC_MDIO_ADDRESS_CR_150_250M       (0x4)
#define EQOS_MAC_MDIO_ADDRESS_CR_250_300M       (0x5)
#define EQOS_MAC_MDIO_ADDRESS_CR_300_500M       (0x6)
#define EQOS_MAC_MDIO_ADDRESS_CR_500_800M       (0x7)
#define EQOS_MAC_MDIO_ADDRESS_CR_DIV_4          (0x8)
#define EQOS_MAC_MDIO_ADDRESS_CR_DIV_6          (0x9)
#define EQOS_MAC_MDIO_ADDRESS_CR_DIV_8          (0xA)
#define EQOS_MAC_MDIO_ADDRESS_CR_DIV_10         (0xB)
#define EQOS_MAC_MDIO_ADDRESS_CR_DIV_12         (0xC)
#define EQOS_MAC_MDIO_ADDRESS_CR_DIV_14         (0xD)
#define EQOS_MAC_MDIO_ADDRESS_CR_DIV_16         (0xE)
#define EQOS_MAC_MDIO_ADDRESS_CR_DIV_18         (0xF)



#define EQOS_MAC_MDIO_ADDRESS_SKAP          BIT(4)
#define EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT         2
#define EQOS_MAC_MDIO_ADDRESS_GOC_READ          3
#define EQOS_MAC_MDIO_ADDRESS_GOC_WRITE         1
#define EQOS_MAC_MDIO_ADDRESS_C45E          BIT(1)
#define EQOS_MAC_MDIO_ADDRESS_GB            BIT(0)

#define EQOS_MAC_MDIO_DATA_GD_MASK          0xffff

#define EQOS_MTL_REGS_BASE 0xd00
typedef struct gmac_mtl_regs {
    uint32_t txq0_operation_mode;           /* 0xd00 */
    uint32_t unused_d04;                /* 0xd04 */
    uint32_t txq0_debug;                /* 0xd08 */
    uint32_t unused_d0c[(0xd18 - 0xd0c) / 4];   /* 0xd0c */
    uint32_t txq0_quantum_weight;           /* 0xd18 */
    uint32_t unused_d1c[(0xd30 - 0xd1c) / 4];   /* 0xd1c */
    uint32_t rxq0_operation_mode;           /* 0xd30 */
    uint32_t unused_d34;                /* 0xd34 */
    uint32_t rxq0_debug;                /* 0xd38 */
}gmac_mtl_regs_t;

#define EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT      16
#define EQOS_MTL_TXQ0_OPERATION_MODE_TQS_MASK       0x1ff
#define EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT    2
#define EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_MASK     3
#define EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_ENABLED  2
#define EQOS_MTL_TXQ0_OPERATION_MODE_TSF        BIT(1)
#define EQOS_MTL_TXQ0_OPERATION_MODE_FTQ        BIT(0)

#define EQOS_MTL_TXQ0_DEBUG_TXQSTS          BIT(4)
#define EQOS_MTL_TXQ0_DEBUG_TRCSTS_SHIFT        1
#define EQOS_MTL_TXQ0_DEBUG_TRCSTS_MASK         3

#define EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT      20
#define EQOS_MTL_RXQ0_OPERATION_MODE_RQS_MASK       0x3ff
#define EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT      14
#define EQOS_MTL_RXQ0_OPERATION_MODE_RFD_MASK       0x3f
#define EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT      8
#define EQOS_MTL_RXQ0_OPERATION_MODE_RFA_MASK       0x3f
#define EQOS_MTL_RXQ0_OPERATION_MODE_EHFC       BIT(7)
#define EQOS_MTL_RXQ0_OPERATION_MODE_RSF        BIT(5)
#define EQOS_MTL_RXQ0_OPERATION_MODE_FEP        BIT(4)
#define EQOS_MTL_RXQ0_OPERATION_MODE_FUP        BIT(3)

#define EQOS_MTL_RXQ0_DEBUG_PRXQ_SHIFT          16
#define EQOS_MTL_RXQ0_DEBUG_PRXQ_MASK           0x7fff
#define EQOS_MTL_RXQ0_DEBUG_RXQSTS_SHIFT        4
#define EQOS_MTL_RXQ0_DEBUG_RXQSTS_MASK         3

#define EQOS_DMA_REGS_BASE 0x1000
typedef struct gmac_dma_regs {
    uint32_t mode;                  /* 0x1000 */
    uint32_t sysbus_mode;               /* 0x1004 */
    uint32_t unused_1008[(0x1100 - 0x1008) / 4];    /* 0x1008 */
    uint32_t ch0_control;               /* 0x1100 */
    uint32_t ch0_tx_control;            /* 0x1104 */
    uint32_t ch0_rx_control;            /* 0x1108 */
    uint32_t unused_110c;               /* 0x110c */
    uint32_t ch0_txdesc_list_haddress;      /* 0x1110 */
    uint32_t ch0_txdesc_list_address;       /* 0x1114 */
    uint32_t ch0_rxdesc_list_haddress;      /* 0x1118 */
    uint32_t ch0_rxdesc_list_address;       /* 0x111c */
    uint32_t ch0_txdesc_tail_pointer;       /* 0x1120 */
    uint32_t unused_1124;               /* 0x1124 */
    uint32_t ch0_rxdesc_tail_pointer;       /* 0x1128 */
    uint32_t ch0_txdesc_ring_length;        /* 0x112c */
    uint32_t ch0_rxdesc_ring_length;        /* 0x1130 */
    uint32_t ch0_interrupt_enable;        /* 0x1134 */
    uint32_t unused_1138[(0x1160 - 0x1138) / 4];    /* 0x1138 */
    uint32_t ch0_status;                /* 0x1160 */
}gmac_dma_regs_t;

/* Interrupt enable bits per channel */
#define DMA_CHAN_INTR_ENA_NIE		BIT(15)
#define DMA_CHAN_INTR_ENA_AIE		BIT(14)
#define DMA_CHAN_INTR_ENA_CDE		BIT(13)
#define DMA_CHAN_INTR_ENA_FBE		BIT(12)
#define DMA_CHAN_INTR_ENA_ERE		BIT(11)
#define DMA_CHAN_INTR_ENA_ETE		BIT(10)
#define DMA_CHAN_INTR_ENA_RWE		BIT(9)
#define DMA_CHAN_INTR_ENA_RSE		BIT(8)
#define DMA_CHAN_INTR_ENA_RBUE		BIT(7)
#define DMA_CHAN_INTR_ENA_RIE		BIT(6)
#define DMA_CHAN_INTR_ENA_TBUE		BIT(2)
#define DMA_CHAN_INTR_ENA_TSE		BIT(1)
#define DMA_CHAN_INTR_ENA_TIE		BIT(0)

#define DMA_CHAN_INTR_NORMAL		(DMA_CHAN_INTR_ENA_NIE | \
					 DMA_CHAN_INTR_ENA_RIE | \
					 DMA_CHAN_INTR_ENA_TIE)

#define DMA_CHAN_INTR_ABNORMAL		(DMA_CHAN_INTR_ENA_AIE | \
					 DMA_CHAN_INTR_ENA_FBE)

/* Interrupt status per channel */
#define DMA_CHAN_STATUS_REB		GENMASK(21, 19)
#define DMA_CHAN_STATUS_REB_SHIFT	19
#define DMA_CHAN_STATUS_TEB		GENMASK(18, 16)
#define DMA_CHAN_STATUS_TEB_SHIFT	16
#define DMA_CHAN_STATUS_NIS		BIT(15)
#define DMA_CHAN_STATUS_AIS		BIT(14)
#define DMA_CHAN_STATUS_CDE		BIT(13)
#define DMA_CHAN_STATUS_FBE		BIT(12)
#define DMA_CHAN_STATUS_ERI		BIT(11)
#define DMA_CHAN_STATUS_ETI		BIT(10)
#define DMA_CHAN_STATUS_RWT		BIT(9)
#define DMA_CHAN_STATUS_RPS		BIT(8)
#define DMA_CHAN_STATUS_RBU		BIT(7)
#define DMA_CHAN_STATUS_RI		BIT(6)
#define DMA_CHAN_STATUS_TBU		BIT(2)
#define DMA_CHAN_STATUS_TPS		BIT(1)
#define DMA_CHAN_STATUS_TI		BIT(0)

#define DMA_CHAN_STATUS_MSK_COMMON	(DMA_CHAN_STATUS_NIS | \
					 DMA_CHAN_STATUS_AIS | \
					 DMA_CHAN_STATUS_CDE | \
					 DMA_CHAN_STATUS_FBE)

#define DMA_CHAN_STATUS_MSK_RX		(DMA_CHAN_STATUS_REB | \
					 DMA_CHAN_STATUS_ERI | \
					 DMA_CHAN_STATUS_RWT | \
					 DMA_CHAN_STATUS_RPS | \
					 DMA_CHAN_STATUS_RBU | \
					 DMA_CHAN_STATUS_RI | \
					 DMA_CHAN_STATUS_MSK_COMMON)

#define DMA_CHAN_STATUS_MSK_TX		(DMA_CHAN_STATUS_ETI | \
					 DMA_CHAN_STATUS_TBU | \
					 DMA_CHAN_STATUS_TPS | \
					 DMA_CHAN_STATUS_TI | \
					 DMA_CHAN_STATUS_MSK_COMMON)

#define EQOS_DMA_MODE_SWR               BIT(0)

#define EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT       16
#define EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_MASK        0xf
#define EQOS_DMA_SYSBUS_MODE_EAME           BIT(11)
#define EQOS_DMA_SYSBUS_MODE_BLEN16         BIT(3)
#define EQOS_DMA_SYSBUS_MODE_BLEN8          BIT(2)
#define EQOS_DMA_SYSBUS_MODE_BLEN4          BIT(1)

#define EQOS_DMA_CH0_CONTROL_PBLX8          BIT(16)

#define EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT     16
#define EQOS_DMA_CH0_TX_CONTROL_TXPBL_MASK      0x3f
#define EQOS_DMA_CH0_TX_CONTROL_OSP         BIT(4)
#define EQOS_DMA_CH0_TX_CONTROL_ST          BIT(0)

#define EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT     16
#define EQOS_DMA_CH0_RX_CONTROL_RXPBL_MASK      0x3f
#define EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT      1
#define EQOS_DMA_CH0_RX_CONTROL_RBSZ_MASK       0x3fff
#define EQOS_DMA_CH0_RX_CONTROL_SR          BIT(0)

/* These registers are Tegra186-specific */
#define EQOS_TEGRA186_REGS_BASE 0x8800
typedef struct gmac_tegra186_regs {
    uint32_t sdmemcomppadctrl;          /* 0x8800 */
    uint32_t auto_cal_config;           /* 0x8804 */
    uint32_t unused_8808;               /* 0x8808 */
    uint32_t auto_cal_status;           /* 0x880c */
}gmac_tegra186_regs_t;

#define EQOS_SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD BIT(31)

#define EQOS_AUTO_CAL_CONFIG_START          BIT(31)
#define EQOS_AUTO_CAL_CONFIG_ENABLE         BIT(29)

#define EQOS_AUTO_CAL_STATUS_ACTIVE         BIT(31)

/* Descriptors */

#define EQOS_DESCRIPTOR_WORDS   4
#define EQOS_DESCRIPTOR_SIZE    (EQOS_DESCRIPTOR_WORDS * 4)
/* We assume ARCH_DMA_MINALIGN >= 16; 16 is the EQOS HW minimum */
#define EQOS_DESCRIPTOR_ALIGN   ARCH_DMA_MINALIGN
#define EQOS_DESCRIPTORS_TX 256
#define EQOS_DESCRIPTORS_RX HAL_EQOS_DESC_NUM
#define EQOS_DESCRIPTORS_NUM    (EQOS_DESCRIPTORS_TX + EQOS_DESCRIPTORS_RX)
#define EQOS_DESCRIPTORS_SIZE    ALIGN(EQOS_DESCRIPTORS_NUM * \
                      EQOS_DESCRIPTOR_SIZE, ARCH_DMA_MINALIGN)
#define EQOS_BUFFER_ALIGN    ARCH_DMA_MINALIGN
#define EQOS_MAX_PACKET_SIZE    ALIGN(1568, ARCH_DMA_MINALIGN)
#define EQOS_RX_BUFFER_SIZE (EQOS_MAX_PACKET_SIZE)

/*
 * Warn if the cache-line size is larger than the descriptor size. In such
 * cases the driver will likely fail because the CPU needs to flush the cache
 * when requeuing RX buffers, therefore descriptors written by the hardware
 * may be discarded. Architectures with full IO coherence, such as x86, do not
 * experience this issue, and hence are excluded from this condition.
 *
 * This can be fixed by defining CONFIG_SYS_NONCACHED_MEMORY which will cause
 * the driver to allocate descriptors from a pool of non-cached memory.
 */
#if EQOS_DESCRIPTOR_SIZE < ARCH_DMA_MINALIGN
#if !defined(CONFIG_SYS_NONCACHED_MEMORY) && \
    !defined(CONFIG_SYS_DCACHE_OFF) && !defined(CONFIG_X86)
#warning Cache line size is larger than descriptor size
#endif
#endif

typedef struct eqos_desc {
    uint32_t des0;
    uint32_t des1;
    uint32_t des2;
    uint32_t des3;
}eqos_desc_t;

#define EQOS_DESC2_IOC      BIT(31)
#define EQOS_DESC3_OWN      BIT(31)
#define EQOS_DESC3_IOC      BIT(30)
#define EQOS_DESC3_FD       BIT(29)
#define EQOS_DESC3_LD       BIT(28)
#define EQOS_DESC3_BUF1V    BIT(24)

struct eqos_int {
    int rx_buf_unav_irq;
    int rx_process_stopped_irq;
    int rx_watchdog_irq;
    int tx_early_irq;
    int tx_process_stopped_irq;
    int fatal_bus_error_irq;
    int tx_hard_error;
    int normal_irq_n;
    int rx_normal_irq_n;
    int tx_normal_irq_n;
    int txbuffer_not_aval;
    int rx_early_irq;
};

typedef struct gmac_eth_dev {
    gmac_mac_regs_t *mac_regs;
    gmac_mtl_regs_t *mtl_regs;
    gmac_dma_regs_t *dma_regs;
    gmac_tegra186_regs_t *tegra186_regs;
    //struct rt_device *phydev;

    void *descs_noalign;
    void *descs;
    eqos_desc_t *tx_descs;
    eqos_desc_t *rx_descs;
    int tx_desc_idx;
    //int rx_desc_idx;
    char rxbusy;
    char rxnext;
    unsigned short busyrxdesc;

    void *tx_dma_buf[EQOS_DESCRIPTORS_TX];
    void *rx_dma_buf[EQOS_DESCRIPTORS_RX];
    void *tx_dma_buf_noalign[EQOS_DESCRIPTORS_TX];
    void *rx_dma_buf_noalign[EQOS_DESCRIPTORS_RX];

    struct eqos_int int_summary;
    gmac_handle_t *handle;
}eqos_eth_dev_t;

/*********add by adapter**********/

#define clrbits(addr, clear) \
    sys_writel(sys_readl(addr) & ~(clear), addr)

#define setbits(addr, set) \
    sys_writel(sys_readl(addr) | (set), addr)

#define clrsetbits(addr, clear, set) \
    sys_writel((sys_readl(addr) & ~(clear)) | (set), addr)

#define  DEFAULT_INT_TX_EN_VAL       0x8001 //tx interrupt
#define  DEFAULT_INT_RX_EN_VAL       0x8040 //Rx interrupt
#define  DEFAULT_INT_ERR_EN_VAL      0x7200 //error interrupt

#define  STATUS_TI                  BIT(0)
#define  STATUS_RI                  BIT(6)
#define  STATUS_FBI                 BIT(13)

int gmac_mdio_write(gmac_handle_t *gmac, int reg, unsigned int data);
int gmac_mdio_read(gmac_handle_t *gmac, int reg, unsigned int *data);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __HAL_GMAC_DWC_ETH_QOS_H_ */

