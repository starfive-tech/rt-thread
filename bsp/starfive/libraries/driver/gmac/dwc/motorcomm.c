
#include <rtthread.h>
#include "board.h"
#include "hal_gmac.h"
#include "hal_gmac_dwc_eth_qos.h"
#include "hal_gmac_dev.h"
#include "motorcomm.h"

//#define MOTORCOMM_PHY_ID_MASK    0x00000fff
//#define MOTORCOMM_MPHY_ID_MASK    0x0000ffff

#define PHY_ID_YT8010        0x00000309
#define PHY_ID_YT8510        0x00000109
#define PHY_ID_YT8511        0x0000010a
#define PHY_ID_YT8512        0x00000118
#define PHY_ID_YT8512B        0x00000128
#define PHY_ID_YT8521        0x0000011a
#define PHY_ID_YT8531S       0x4f51e91a
#define PHY_ID_YT8531        0x4f51e91b
#define MOTORCOMM_8531_PHY_ID_MASK    0xffffffff

//#define PHY_ID_YT8614        0x0000e899
#define PHY_ID_YT8618        0x0000e889

#define REG_PHY_SPEC_STATUS        0x11
#define REG_DEBUG_ADDR_OFFSET        0x1e
#define REG_DEBUG_DATA            0x1f

#define YT8512_EXTREG_AFE_PLL        0x50
#define YT8512_EXTREG_EXTEND_COMBO    0x4000
#define YT8512_EXTREG_LED0        0x40c0
#define YT8512_EXTREG_LED1        0x40c3

#define YT8512_EXTREG_SLEEP_CONTROL1    0x2027

#define YT_SOFTWARE_RESET        0x8000

#define YT8512_CONFIG_PLL_REFCLK_SEL_EN    0x0040
#define YT8512_CONTROL1_RMII_EN        0x0001
#define YT8512_LED0_ACT_BLK_IND        0x1000
#define YT8512_LED0_DIS_LED_AN_TRY    0x0001
#define YT8512_LED0_BT_BLK_EN        0x0002
#define YT8512_LED0_HT_BLK_EN        0x0004
#define YT8512_LED0_COL_BLK_EN        0x0008
#define YT8512_LED0_BT_ON_EN        0x0010
#define YT8512_LED1_BT_ON_EN        0x0010
#define YT8512_LED1_TXACT_BLK_EN    0x0100
#define YT8512_LED1_RXACT_BLK_EN    0x0200
#define YT8512_SPEED_MODE        0xc000
#define YT8512_DUPLEX            0x2000

#define YT8512_SPEED_MODE_BIT        14
#define YT8512_DUPLEX_BIT        13
#define YT8512_EN_SLEEP_SW_BIT        15

#define YT8521_EXTREG_SLEEP_CONTROL1    0x27
#define YT8521_EN_SLEEP_SW_BIT        15

#define YT8521_SPEED_MODE        0xc000
#define YT8521_DUPLEX            0x2000
#define YT8521_SPEED_MODE_BIT        14
#define YT8521_DUPLEX_BIT        13
#define YT8521_LINK_STATUS_BIT        10

#define YT8521_EXTREG_SMI_SDS_PHY                       0xa000
#define YT8521_EXTREG_CHIP_CONFIG                       0xa001
#define YT8521_RXC_DLY_EN_BIT                           8
#define YT8521_EXTREG_RGMII_CONFIG1                     0xa003
#define YT8521_RX_DELAY_SEL_SHIFT                       10
#define YT8521_RX_DELAY_SEL_MASK                        (0xFUL << YT8521_RX_DELAY_SEL_SHIFT)
#define YT8521_TX_DELAY_SEL_FE_SHIFT                    4
#define YT8521_TX_DELAY_SEL_FE_MASK                     (0xFUL << YT8521_TX_DELAY_SEL_FE_SHIFT)
#define YT8521_TX_DELAY_SEL_SHIFT                       0
#define YT8521_TX_DELAY_SEL_MASK                        (0xFUL << YT8521_TX_DELAY_SEL_SHIFT)

/* based on yt8521 wol config register */
#define YTPHY_UTP_INTR_REG             0x12
/* WOL Event Interrupt Enable */
#define YTPHY_WOL_INTR            BIT(6)

/* Magic Packet MAC address registers */
#define YTPHY_MAGIC_PACKET_MAC_ADDR2                 0xa007
#define YTPHY_MAGIC_PACKET_MAC_ADDR1                 0xa008
#define YTPHY_MAGIC_PACKET_MAC_ADDR0                 0xa009

#define YTPHY_WOL_CFG_REG        0xa00a
#define YTPHY_WOL_CFG_TYPE        BIT(0)    /* WOL TYPE */
#define YTPHY_WOL_CFG_EN        BIT(3)    /* WOL Enable */
#define YTPHY_WOL_CFG_INTR_SEL    BIT(6)    /* WOL Event Interrupt Enable */
#define YTPHY_WOL_CFG_WIDTH1    BIT(1)    /* WOL Pulse Width */
#define YTPHY_WOL_CFG_WIDTH2    BIT(2)

#define YTPHY_REG_SPACE_UTP             0
#define YTPHY_REG_SPACE_FIBER           2

#define YTPHY_EXTREG_CHIP_CONFIG	0xa001
#define YTPHY_EXTREG_RGMII_CONFIG1	0xa003
#define YTPHY_PAD_DRIVES_STRENGTH_CFG	0xa010
#define YTPHY_DUPLEX		0x2000
#define YTPHY_DUPLEX_BIT	13
#define YTPHY_SPEED_MODE	0xc000
#define YTPHY_SPEED_MODE_BIT	14
//#define YTPHY_RGMII_SW_DR_MASK	GENMASK(5, 4)
//#define YTPHY_RGMII_RXC_DR_MASK	GENMASK(15, 13)

/* Phy gmii clock gating Register */
#define YT8521_CLOCK_GATING_REG		0xC
#define YT8521_CGR_RX_CLK_EN		BIT(12)

#define SPEED_10        10
#define SPEED_100       100
#define SPEED_1000      1000
#define SPEED_2500      2500
#define SPEED_10000     10000

/* Produces a mask of set bits covering a range of a uint value */
static inline unsigned int bitfield_mask(unsigned int shift, unsigned int width)
{
        return ((1 << width) - 1) << shift;
}

/* Extract the value of a bitfield found within a given register value */
static inline unsigned int bitfield_extract(unsigned int reg_val, unsigned int shift, unsigned int width)
{
        return (reg_val & bitfield_mask(shift, width)) >> shift;
}

/*
 * Replace the value of a bitfield found within a given register value
 * Returns the newly modified uint value with the replaced field.
 */
static inline unsigned int bitfield_replace(unsigned int reg_val, unsigned int shift, unsigned int width,
                                    unsigned int bitfield_val)
{
        unsigned int mask = bitfield_mask(shift, width);

        return (reg_val & ~mask) | ((bitfield_val << shift) & mask);
}


enum ytphy_wol_type_e
{
    YTPHY_WOL_TYPE_LEVEL,
    YTPHY_WOL_TYPE_PULSE,
    YTPHY_WOL_TYPE_MAX
};
typedef enum ytphy_wol_type_e ytphy_wol_type_t;

enum ytphy_wol_width_e
{
    YTPHY_WOL_WIDTH_84MS,
    YTPHY_WOL_WIDTH_168MS,
    YTPHY_WOL_WIDTH_336MS,
    YTPHY_WOL_WIDTH_672MS,
    YTPHY_WOL_WIDTH_MAX
};
typedef enum ytphy_wol_width_e ytphy_wol_width_t;

struct ytphy_wol_cfg_s
{
    int enable;
    int type;
    int width;
};
typedef struct ytphy_wol_cfg_s ytphy_wol_cfg_t;

typedef struct yt8512_priv
{
    yt8512_refclk_sel_t refclk_sel;
    yt8512_mod_type_t mod_type;
} yt8512_priv_t;

typedef struct yt8521_priv
{
    yt8521_delay_cfg_t *delay_cfg;
} yt8521_priv_t;


struct ytphy_reg_field {
	const char	*name;
	const unsigned char	size;	/* Size of the bitfield, in bits */
	const unsigned char	off;	/* Offset from bit 0 */
	const unsigned char	dflt;	/* Default value */
};

static const struct ytphy_reg_field ytphy_dr_grp[] = {
	{ "rgmii_sw_dr", 2, 4, 0x3},
	{ "rgmii_sw_dr_2", 1, 12, 0x0},
	{ "rgmii_sw_dr_rxc", 3, 13, 0x3}
};

static const struct ytphy_reg_field ytphy_rxtxd_grp[] = {
	{ "rx_delay_sel", 4, 10, 0x0 },
	{ "tx_delay_sel_fe", 4, 4, 0xf },
	{ "tx_delay_sel", 4, 0, 0x1 }
};

static const struct ytphy_reg_field ytphy_txinver_grp[] = {
	{ "tx_inverted_1000", 1, 14, 0x0 },
	{ "tx_inverted_100", 1, 14, 0x0 },
	{ "tx_inverted_10", 1, 14, 0x0 }
};

static const struct ytphy_reg_field ytphy_rxden_grp[] = {
	{ "rxc_dly_en", 1, 8, 0x1 }
};

static int ytphy_read_ext(struct gmac_dev *dev, uint32_t regnum)
{
    int ret;
    int val;

    ret = gmac_mdio_write(dev->hal, REG_DEBUG_ADDR_OFFSET, regnum);
    if (ret < 0)
        return ret;

    ret = gmac_mdio_read(dev->hal, REG_DEBUG_DATA, &val);

    if (ret < 0)
	return ret;

    return val;
}

static int ytphy_write_ext(struct gmac_dev *dev, uint32_t regnum, uint16_t val)
{
    int ret;

    ret = gmac_mdio_write(dev->hal, REG_DEBUG_ADDR_OFFSET, regnum);
    if (ret < 0)
        return ret;

    ret = gmac_mdio_write(dev->hal, REG_DEBUG_DATA, val);

    return ret;
}

static int ytphy_of_config(struct gmac_dev *dev)
{
    gmac_handle_t *handle;
    unsigned int val;
    int i;
    char cfg;

    handle = dev->hal;
    cfg = handle->phy_config.rxc_dly_en;
    val = ytphy_read_ext(dev, YTPHY_EXTREG_CHIP_CONFIG);

    LOG_DBG("ext chip val 0 %x\n", val);
    cfg = (cfg > ((1 << ytphy_rxden_grp[0].size) - 1)) ?
		((1 << ytphy_rxden_grp[0].size) - 1) : cfg;
    val = bitfield_replace(val, ytphy_rxden_grp[0].off,
		    ytphy_rxden_grp[0].size, cfg);
    LOG_DBG("ext chip val 1 %x\n", val);

    ytphy_write_ext(dev, YTPHY_EXTREG_CHIP_CONFIG, val);

    val = ytphy_read_ext(dev, YTPHY_PAD_DRIVES_STRENGTH_CFG);

    LOG_DBG("drv strength val 0 %x\n", val);
    for (i = 0; i < sizeof(ytphy_dr_grp)/sizeof(ytphy_dr_grp[0]); i++) {

	 if (i == 0)
             cfg = handle->phy_config.rgmii_sw_dr;
	 else if (i == 1)
	     cfg = handle->phy_config.rgmii_sw_dr_2;
	 else
	     cfg = handle->phy_config.rgmii_sw_dr_rxc;

         /*check the cfg overflow or not*/
         cfg = (cfg > ((1 << ytphy_dr_grp[i].size) - 1)) ?
			((1 << ytphy_dr_grp[i].size) - 1) : cfg;

	 val = bitfield_replace(val, ytphy_dr_grp[i].off,
				ytphy_dr_grp[i].size, cfg);
    }
    LOG_DBG("drv strength val 1 %x\n", val);
    ytphy_write_ext(dev, YTPHY_PAD_DRIVES_STRENGTH_CFG, val);

    val = ytphy_read_ext(dev, YTPHY_EXTREG_RGMII_CONFIG1);
    LOG_DBG("rgmii strength val %x\n", val);

    for (i = 0; i < sizeof(ytphy_rxtxd_grp)/sizeof(ytphy_dr_grp[0]); i++) {
	if (i == 0)
            cfg = handle->phy_config.rx_delay_sel;
	else if (i == 1)
            cfg = handle->phy_config.tx_delay_sel_fe;
        else
            cfg = handle->phy_config.tx_delay_sel;

            /*check the cfg overflow or not*/
	    cfg = (cfg > ((1 << ytphy_rxtxd_grp[i].size) - 1)) ?
			((1 << ytphy_rxtxd_grp[i].size) - 1) : cfg;

	    val = bitfield_replace(val, ytphy_rxtxd_grp[i].off,
				ytphy_rxtxd_grp[i].size, cfg);
    }

    LOG_DBG("rgmii 1 strength val %x\n", val);
    return ytphy_write_ext(dev, YTPHY_EXTREG_RGMII_CONFIG1, val);
}

static int ytphy_parse_status(struct gmac_dev *dev)
{
    int val, ret;
    int speed, speed_mode, duplex;

    ret = gmac_mdio_read(dev->hal, REG_PHY_SPEC_STATUS, &val);
    if (ret < 0)
        return val;

    duplex = (val & YTPHY_DUPLEX) >> YTPHY_DUPLEX_BIT;
    speed_mode = (val & YTPHY_SPEED_MODE) >> YTPHY_SPEED_MODE_BIT;
    switch (speed_mode) {
    case 2:
        speed = SPEED_1000;
	break;
    case 1:
	speed = SPEED_100;
	break;
    default:
	speed = SPEED_10;
	break;
    }

    if ((dev->speed_mode != speed_mode) ||
	(dev->duplex != duplex))
	dev->mode_changed = 1;

    dev->speed_mode = speed_mode;
    dev->speed = speed;
    dev->duplex = duplex;

    hal_printf("speed %d duplex %d\n", speed, duplex);

    return 0;
}

static int ytphy_of_inverted(struct gmac_dev *dev)
{
    int val, old_val;
    char inver_10, inver_100, inver_1000;

    val = ytphy_read_ext(dev, YTPHY_EXTREG_RGMII_CONFIG1);
    inver_10 = dev->hal->phy_config.tx_inverted_10;
    inver_100 = dev->hal->phy_config.tx_inverted_100;
    inver_1000 = dev->hal->phy_config.tx_inverted_1000;

    LOG_DBG("inverted val 0 %x\n", val);
    switch (dev->speed) {
    case SPEED_1000:
	val = bitfield_replace(val, ytphy_txinver_grp[0].off,
				ytphy_txinver_grp[0].size, inver_1000);
	break;
    case SPEED_100:
	val = bitfield_replace(val, ytphy_txinver_grp[1].off,
				ytphy_txinver_grp[1].size, inver_100);
	break;
    case SPEED_10:
	val = bitfield_replace(val, ytphy_txinver_grp[2].off,
				ytphy_txinver_grp[2].size, inver_10);
	break;
    default:
	hal_printf("UNKOWN SPEED\n");
	break;
    }

    if (val == old_val)
	return 0;

    LOG_DBG("inverted val 1 %x\n", val);
    return ytphy_write_ext(dev, YTPHY_EXTREG_RGMII_CONFIG1, val);
}

static int yt8531_disable_llp(gmac_handle_t *handle)
{
    unsigned int val;
    int ret;

    /* disable auto sleep */
    ret = gmac_mdio_read(handle, YT8521_EXTREG_SLEEP_CONTROL1, &val);
    if (ret < 0)
	return val;

    val &= (~BIT(YT8521_EN_SLEEP_SW_BIT));
    ret = gmac_mdio_write(handle, YT8521_EXTREG_SLEEP_CONTROL1, val);
    if (ret < 0)
	return ret;

     /* enable RXC clock when no wire plug */
    ret = gmac_mdio_read(handle, YT8521_CLOCK_GATING_REG, &val);
    if (val < 0)
	return val;

    val &= ~YT8521_CGR_RX_CLK_EN;
    ret = gmac_mdio_write(handle, YT8521_CLOCK_GATING_REG, val);

    if (ret < 0)
	return ret;

    return 0;
}

//The configurations for 8531 and 8521 are the same
static int yt8531_dev_init(struct gmac_dev *dev)
{
    gmac_handle_t *handle = dev->hal;
    int ret;

    ret = gmac_dev_genphy_config_aneg(dev);
    if (ret < 0)
	return ret;

    ret = gmac_dev_genphy_process_aneg_result(dev, ret);
    if (ret < 0)
	return ret;

    if (handle->phy_config.disable_llp)
	yt8531_disable_llp(handle);

    ret = ytphy_of_config(dev);
    if (ret < 0)
        return ret;

    ret = genphy_update_link(dev);
    if (ret < 0)
 	return ret;

    ret = ytphy_parse_status(dev);
    if (ret < 0)
	return ret;

    ret = ytphy_of_inverted(dev);
    if (ret < 0)
 	return ret;

    return 0;

}

int yt8531_dev_deinit(struct gmac_dev *dev)
{
    if (dev) {
        hal_free(dev);
    }
    return 0;
}

static struct gmac_phy_dev phy_dev[] = {
    {
	.id   = PHY_ID_YT8531,
	.mask = MOTORCOMM_8531_PHY_ID_MASK,
	.name  = "yt8531",
	.ops = {
	    .init = yt8531_dev_init,
	.deinit = yt8531_dev_deinit,
	},
    }
};

int register_gmac_phy_driver(struct gmac_dev *dev, unsigned int value)
{
    int i;

    for (i = 0; i < sizeof(phy_dev) / sizeof(phy_dev[0]); i++) {
	if ((phy_dev[i].id & phy_dev[i].mask) == value) {
	    dev->name = phy_dev->name;
	    dev->ops = &phy_dev->ops;
	    return 0;
	}
    }

    return -1;
}
