/*
 * include/linux/motorcomm_phy.h
 *
 * Motorcomm PHY IDs
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef _MOTORCOMM_H
#define _MOTORCOMM_H


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "hal_gmac_dev.h"

typedef enum {
    YT8512_MODE_DEFAULT = 0,
    YT8512_MODE_MII,
    YT8512_MODE_REMII,
    YT8512_MODE_RMII2,
    YT8512_MODE_RMII1,
} yt8512_mod_type_t;

typedef enum {
    YT8512_REFCLK_SEL_25M = 0,
    YT8512_REFCLK_SEL_50M,
} yt8512_refclk_sel_t;

typedef struct {
    uint8_t rx_2ns_delay_en;//0:disalbe 2ns delay;1:enable 2ns delay
    uint8_t rx_delay_sel; //RGMII RX_CLK delay train configuration,about 150ps per step
    uint8_t tx_10_100M_delay_sel;    //RGMII TX_CLK delay train configuration when speed is 100Mbps or 10Mbps, it's 150ps per step typically
    uint8_t tx_1000M_delay_sel;    //RGMII TX_CLK delay train configuration when speed is 1000Mbps, it's 150ps per step typically
} yt8521_delay_cfg_t;

typedef yt8521_delay_cfg_t yt8531_delay_cfg_t;
/*
*p0:gmac id
*p1:yt8512_refclk_sel_t
*p2:yt8512_mod_type_t
*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MOTORCOMM_H */

