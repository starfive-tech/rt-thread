/**
 ******************************************************************************
 * @copyright Copyright (c) 2020 StarFive Technology Co.,Ltd.
 *
 * @file gmac_phy_dev.h
 * @author StarFive FW Team
 * @brief
 ******************************************************************************
 */
#ifndef __HAL_GMAC_DEV_H_
#define __HAL_GMAC_DEV_H_

//#include "bsp_hal_gmac.h"
#include "hal_gmac.h"

/* Generic MII registers. */
#define MII_BMCR        0x00    /* Basic mode control register */
#define MII_BMSR        0x01    /* Basic mode status register  */
#define MII_PHYSID1     0x02    /* PHYS ID 1               */
#define MII_PHYSID2     0x03    /* PHYS ID 2               */
#define MII_ADVERTISE       0x04    /* Advertisement control reg   */
#define MII_LPA         0x05    /* Link partner ability reg    */
#define MII_EXPANSION       0x06    /* Expansion register          */
#define MII_CTRL1000        0x09    /* 1000BASE-T control          */
#define MII_STAT1000        0x0a    /* 1000BASE-T status           */
#define MII_ESTATUS     0x0f    /* Extended Status */
#define MII_DCOUNTER        0x12    /* Disconnect counter          */
#define MII_FCSCOUNTER      0x13    /* False carrier counter       */
#define MII_NWAYTEST        0x14    /* N-way auto-neg test reg     */
#define MII_RERRCOUNTER     0x15    /* Receive error counter       */
#define MII_SREVISION       0x16    /* Silicon revision        */
#define MII_RESV1       0x17    /* Reserved...             */
#define MII_LBRERROR        0x18    /* Lpback, rx, bypass error    */
#define MII_PHYADDR     0x19    /* PHY address             */
#define MII_RESV2       0x1a    /* Reserved...             */
#define MII_TPISTATUS       0x1b    /* TPI status for 10mbps       */
#define MII_NCONFIG     0x1c    /* Network interface config    */

/* Basic mode control register. */
#define BMCR_RESV       0x003f  /* Unused...               */
#define BMCR_SPEED1000      0x0040  /* MSB of Speed (1000)         */
#define BMCR_CTST       0x0080  /* Collision test          */
#define BMCR_FULLDPLX       0x0100  /* Full duplex             */
#define BMCR_ANRESTART      0x0200  /* Auto negotiation restart    */
#define BMCR_ISOLATE        0x0400  /* Disconnect DP83840 from MII */
#define BMCR_PDOWN      0x0800  /* Powerdown the DP83840       */
#define BMCR_ANENABLE       0x1000  /* Enable auto negotiation     */
#define BMCR_SPEED100       0x2000  /* Select 100Mbps          */
#define BMCR_LOOPBACK       0x4000  /* TXD loopback bits           */
#define BMCR_RESET      0x8000  /* Reset the DP83840           */

/* Basic mode status register. */
#define BMSR_ERCAP      0x0001  /* Ext-reg capability          */
#define BMSR_JCD        0x0002  /* Jabber detected         */
#define BMSR_LSTATUS        0x0004  /* Link status             */
#define BMSR_ANEGCAPABLE    0x0008  /* Able to do auto-negotiation */
#define BMSR_RFAULT     0x0010  /* Remote fault detected       */
#define BMSR_ANEGCOMPLETE   0x0020  /* Auto-negotiation complete   */
#define BMSR_RESV       0x00c0  /* Unused...               */
#define BMSR_ESTATEN        0x0100  /* Extended Status in R15 */
#define BMSR_100HALF2       0x0200  /* Can do 100BASE-T2 HDX */
#define BMSR_100FULL2       0x0400  /* Can do 100BASE-T2 FDX */
#define BMSR_10HALF     0x0800  /* Can do 10mbps, half-duplex  */
#define BMSR_10FULL     0x1000  /* Can do 10mbps, full-duplex  */
#define BMSR_100HALF        0x2000  /* Can do 100mbps, half-duplex */
#define BMSR_100FULL        0x4000  /* Can do 100mbps, full-duplex */
#define BMSR_100BASE4       0x8000  /* Can do 100mbps, 4k packets  */

/* Advertisement control register. */
#define ADVERTISE_SLCT      0x001f  /* Selector bits           */
#define ADVERTISE_CSMA      0x0001  /* Only selector supported     */
#define ADVERTISE_10HALF    0x0020  /* Try for 10mbps half-duplex  */
#define ADVERTISE_1000XFULL 0x0020  /* Try for 1000BASE-X full-duplex */
#define ADVERTISE_10FULL    0x0040  /* Try for 10mbps full-duplex  */
#define ADVERTISE_1000XHALF 0x0040  /* Try for 1000BASE-X half-duplex */
#define ADVERTISE_100HALF   0x0080  /* Try for 100mbps half-duplex */
#define ADVERTISE_1000XPAUSE    0x0080  /* Try for 1000BASE-X pause    */
#define ADVERTISE_100FULL   0x0100  /* Try for 100mbps full-duplex */
#define ADVERTISE_1000XPSE_ASYM 0x0100  /* Try for 1000BASE-X asym pause */
#define ADVERTISE_100BASE4  0x0200  /* Try for 100mbps 4k packets  */
#define ADVERTISE_PAUSE_CAP 0x0400  /* Try for pause           */
#define ADVERTISE_PAUSE_ASYM    0x0800  /* Try for asymetric pause     */
#define ADVERTISE_RESV      0x1000  /* Unused...               */
#define ADVERTISE_RFAULT    0x2000  /* Say we can detect faults    */
#define ADVERTISE_LPACK     0x4000  /* Ack link partners response  */
#define ADVERTISE_NPAGE     0x8000  /* Next page bit           */

#define ADVERTISE_FULL (ADVERTISE_100FULL | ADVERTISE_10FULL | \
            ADVERTISE_CSMA)
#define ADVERTISE_ALL (ADVERTISE_10HALF | ADVERTISE_10FULL | \
               ADVERTISE_100HALF | ADVERTISE_100FULL)

/* Link partner ability register. */
#define LPA_SLCT        0x001f  /* Same as advertise selector  */
#define LPA_10HALF      0x0020  /* Can do 10mbps half-duplex   */
#define LPA_1000XFULL       0x0020  /* Can do 1000BASE-X full-duplex */
#define LPA_10FULL      0x0040  /* Can do 10mbps full-duplex   */
#define LPA_1000XHALF       0x0040  /* Can do 1000BASE-X half-duplex */
#define LPA_100HALF     0x0080  /* Can do 100mbps half-duplex  */
#define LPA_1000XPAUSE      0x0080  /* Can do 1000BASE-X pause     */
#define LPA_100FULL     0x0100  /* Can do 100mbps full-duplex  */
#define LPA_1000XPAUSE_ASYM 0x0100  /* Can do 1000BASE-X pause asym*/
#define LPA_100BASE4        0x0200  /* Can do 100mbps 4k packets   */
#define LPA_PAUSE_CAP       0x0400  /* Can pause               */
#define LPA_PAUSE_ASYM      0x0800  /* Can pause asymetrically     */
#define LPA_RESV        0x1000  /* Unused...               */
#define LPA_RFAULT      0x2000  /* Link partner faulted        */
#define LPA_LPACK       0x4000  /* Link partner acked us       */
#define LPA_NPAGE       0x8000  /* Next page bit           */

#define LPA_DUPLEX      (LPA_10FULL | LPA_100FULL)
#define LPA_100         (LPA_100FULL | LPA_100HALF | LPA_100BASE4)

/* Expansion register for auto-negotiation. */
#define EXPANSION_NWAY      0x0001  /* Can do N-way auto-nego      */
#define EXPANSION_LCWP      0x0002  /* Got new RX page code word   */
#define EXPANSION_ENABLENPAGE   0x0004  /* This enables npage words    */
#define EXPANSION_NPCAPABLE 0x0008  /* Link partner supports npage */
#define EXPANSION_MFAULTS   0x0010  /* Multiple faults detected    */
#define EXPANSION_RESV      0xffe0  /* Unused...               */

#define ESTATUS_1000_XFULL  0x8000  /* Can do 1000BX Full */
#define ESTATUS_1000_XHALF  0x4000  /* Can do 1000BX Half */
#define ESTATUS_1000_TFULL  0x2000  /* Can do 1000BT Full */
#define ESTATUS_1000_THALF  0x1000  /* Can do 1000BT Half */

/* N-way test register. */
#define NWAYTEST_RESV1      0x00ff  /* Unused...               */
#define NWAYTEST_LOOPBACK   0x0100  /* Enable loopback for N-way   */
#define NWAYTEST_RESV2      0xfe00  /* Unused...               */

/* 1000BASE-T Control register */
#define ADVERTISE_1000FULL  0x0200  /* Advertise 1000BASE-T full duplex */
#define ADVERTISE_1000HALF  0x0100  /* Advertise 1000BASE-T half duplex */

/* 1000BASE-T Status register */
#define LPA_1000LOCALRXOK   0x2000  /* Link partner local receiver status */
#define LPA_1000REMRXOK     0x1000  /* Link partner remote receiver status */
#define LPA_1000FULL        0x0800  /* Link partner 1000BASE-T full duplex */
#define LPA_1000HALF        0x0400  /* Link partner 1000BASE-T half duplex */

/* Flow control flags */
#define FLOW_CTRL_TX        0x01
#define FLOW_CTRL_RX        0x02



/*********pubilc***********/
/* Indicates what features are supported by the interface. */
#define SUPPORTED_10baseT_Half          (1 << 0)
#define SUPPORTED_10baseT_Full          (1 << 1)
#define SUPPORTED_100baseT_Half         (1 << 2)
#define SUPPORTED_100baseT_Full         (1 << 3)
#define SUPPORTED_1000baseT_Half        (1 << 4)
#define SUPPORTED_1000baseT_Full        (1 << 5)
#define SUPPORTED_Autoneg               (1 << 6)
#define SUPPORTED_TP                    (1 << 7)
#define SUPPORTED_AUI                   (1 << 8)
#define SUPPORTED_MII                   (1 << 9)
#define SUPPORTED_FIBRE                 (1 << 10)
#define SUPPORTED_BNC                   (1 << 11)
#define SUPPORTED_10000baseT_Full       (1 << 12)
#define SUPPORTED_Pause                 (1 << 13)
#define SUPPORTED_Asym_Pause            (1 << 14)
#define SUPPORTED_2500baseX_Full        (1 << 15)
#define SUPPORTED_Backplane             (1 << 16)
#define SUPPORTED_1000baseKX_Full       (1 << 17)
#define SUPPORTED_10000baseKX4_Full     (1 << 18)
#define SUPPORTED_10000baseKR_Full      (1 << 19)
#define SUPPORTED_10000baseR_FEC        (1 << 20)
#define SUPPORTED_1000baseX_Half        (1 << 21)
#define SUPPORTED_1000baseX_Full        (1 << 22)

/* Indicates what features are advertised by the interface. */
#define ADVERTISED_10baseT_Half         (1 << 0)
#define ADVERTISED_10baseT_Full         (1 << 1)
#define ADVERTISED_100baseT_Half        (1 << 2)
#define ADVERTISED_100baseT_Full        (1 << 3)
#define ADVERTISED_1000baseT_Half       (1 << 4)
#define ADVERTISED_1000baseT_Full       (1 << 5)
#define ADVERTISED_Autoneg              (1 << 6)
#define ADVERTISED_TP                   (1 << 7)
#define ADVERTISED_AUI                  (1 << 8)
#define ADVERTISED_MII                  (1 << 9)
#define ADVERTISED_FIBRE                (1 << 10)
#define ADVERTISED_BNC                  (1 << 11)
#define ADVERTISED_10000baseT_Full      (1 << 12)
#define ADVERTISED_Pause                (1 << 13)
#define ADVERTISED_Asym_Pause           (1 << 14)
#define ADVERTISED_2500baseX_Full       (1 << 15)
#define ADVERTISED_Backplane            (1 << 16)
#define ADVERTISED_1000baseKX_Full      (1 << 17)
#define ADVERTISED_10000baseKX4_Full    (1 << 18)
#define ADVERTISED_10000baseKR_Full     (1 << 19)
#define ADVERTISED_10000baseR_FEC       (1 << 20)
#define ADVERTISED_1000baseX_Half       (1 << 21)
#define ADVERTISED_1000baseX_Full       (1 << 22)

#define PHY_DEFAULT_FEATURES    (SUPPORTED_Autoneg | \
                 SUPPORTED_TP | \
                 SUPPORTED_MII)

#define PHY_10BT_FEATURES    (SUPPORTED_10baseT_Half | \
                 SUPPORTED_10baseT_Full)

#define PHY_100BT_FEATURES    (SUPPORTED_100baseT_Half | \
                 SUPPORTED_100baseT_Full)

#define PHY_1000BT_FEATURES    (SUPPORTED_1000baseT_Half | \
                 SUPPORTED_1000baseT_Full)

#define PHY_BASIC_FEATURES    (PHY_10BT_FEATURES | \
                 PHY_100BT_FEATURES | \
                 PHY_DEFAULT_FEATURES)

#define PHY_GBIT_FEATURES    (PHY_BASIC_FEATURES | \
                 PHY_1000BT_FEATURES)


int gmac_dev_setup(struct gmac_dev *dev, gmac_handle_t *hal);
int gmac_dev_genphy_config_aneg(struct gmac_dev *dev);
int gmac_dev_genphy_restart_aneg(struct gmac_dev *dev);
int gmac_dev_genphy_reset(struct gmac_dev *dev);
int gmac_dev_genphy_process_aneg_result(struct gmac_dev *dev, int result);
int genphy_update_link(struct gmac_dev *dev);
int register_gmac_phy_driver(struct gmac_dev *dev, unsigned int value);
/**
 * gmac_phy_interface_is_rgmii - Convenience function for testing if a PHY interface
 * is RGMII (all variants)
 * @interface: the phy interface type
 */
#if 0
static inline int gmac_phy_interface_is_rgmii(gmac_phy_interface_t interface)
{
    return (interface >= GMAC_PHY_INTERFACE_MODE_RGMII && interface <= GMAC_PHY_INTERFACE_MODE_RGMII_TXID);
}

/**
 * gmac_phy_interface_is_sgmii - Convenience function for testing if a PHY interface
 * is SGMII (all variants)
 * @interface: the phy interface type
 */
static inline int gmac_phy_interface_is_sgmii(gmac_phy_interface_t interface)
{
    return (interface >= GMAC_PHY_INTERFACE_MODE_SGMII && interface <= GMAC_PHY_INTERFACE_MODE_QSGMII);
}
#endif
#endif /* __HAL_GMAC_DEV_H_ */

