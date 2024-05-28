/**
 ******************************************************************************
 * @copyright Copyright (c) 2020 StarFive Technology Co.,Ltd.
 * @file hal_gmac.h
 * @author StarFive FW Team
 * @brief
 ******************************************************************************
 */
#ifndef __HAL_GMAC_H_
#define __HAL_GMAC_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#include <netif/ethernetif.h>

typedef enum gmac_phy_speed{
    GMAC_PHY_SPEED_10M      = 0,
    GMAC_PHY_SPEED_100M     = 1,
    GMAC_PHY_SPEED_1000M    = 2,
} gmac_phy_speed_t;

typedef enum gmac_phy_duplex {
    GMAC_PHY_HALF_DUPLEX     = 0,
    GMAC_PHY_FULL_DUPLEX     = 1,
} gmac_phy_duplex_t;


struct phy_dts_config {
    char rgmii_sw_dr_2;
    char rgmii_sw_dr;
    char rgmii_sw_dr_rxc;
    char tx_delay_sel_fe;
    char tx_delay_sel;
    char rxc_dly_en;
    char rx_delay_sel;
    char tx_inverted_10;
    char tx_inverted_100;
    char tx_inverted_1000;
    char disable_llp;
};

struct gmac_handle;
struct gmac_phy_ops;

struct gmac_dev {
    char *name;
    struct gmac_phy_ops *ops;
    struct gmac_handle *hal;
    rt_uint32_t advertising;
    rt_uint32_t supported;
    rt_uint16_t link_status;
    int speed_mode;
    int speed;
    int duplex;
    char mode_changed:1;
    char disable_llp:1;
};

struct gmac_phy_ops {
    int (*init) (struct gmac_dev *);
    int (*deinit) (struct gmac_dev *);
};

struct gmac_phy_dev {
    unsigned int id;
    unsigned int mask;
    struct gmac_phy_ops ops;
    char *name;
};

struct gmac_config {
    int speed_mode;
    int speed;
    int phy_addr;
    int irq;
    unsigned char enetaddr[6];
    unsigned char duplex;
};

typedef struct gmac_handle {
    struct eth_device eth;
    struct gmac_config gmac_config;
    struct gmac_dev *phy_dev;
    struct phy_dts_config phy_config;
    const struct memp_desc *memp_rx_pool;
    void *base;
    void *priv;
    int id;
    char name[16];
} gmac_handle_t;

gmac_handle_t * gmac_open(gmac_handle_t *);
void gmac_plat_init(gmac_handle_t *gmac);
void gmac_set_board_config(gmac_handle_t *gmac);
int gmac_close(gmac_handle_t *gmac);
void phy_link_detect(void *param);
int gmac_phy_init(gmac_handle_t *handle);
int genric_gmac_phy_init(gmac_handle_t * handle);
void gmac_link_change(gmac_handle_t *dev,int up);

#if defined(GMAC_DEBUG)
    #define DW_GMAC_TRACE         rt_kprintf
    #define LOG_DBG rt_kprintf
#else
    #define DW_GMAC_TRACE(...)
    #define LOG_DBG
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __HAL_GMAC_H_ */
