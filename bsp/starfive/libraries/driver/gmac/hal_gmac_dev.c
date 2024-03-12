/**
 ******************************************************************************
 * @copyright Copyright (c) 2020 StarFive Technology Co.,Ltd.
 * @file hal_gmac_dev.c
 * @author StarFive FW Team
 * @brief
 ******************************************************************************
 */
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include "board.h"
#include "hal_gmac.h"

#include "hal_gmac_dev.h"
#include "hal_gmac_dwc_eth_qos.h"

int gmac_dev_genphy_config_aneg(struct gmac_dev *dev)
{
    rt_uint32_t advertise, supported;
    int oldadv, adv, bmsr;
    int err, changed = 0;

#if 0
    switch (speed)
    {
    case GMAC_PHY_SPEED_10M:
            supported = (duplex == GMAC_PHY_HALF_DUPLEX) ? SUPPORTED_10baseT_Half : SUPPORTED_10baseT_Full;
        break;
    case GMAC_PHY_SPEED_100M:
            supported = (duplex == GMAC_PHY_HALF_DUPLEX) ? SUPPORTED_100baseT_Half : SUPPORTED_100baseT_Full;
        break;
    case GMAC_PHY_SPEED_1000M:
            supported = (duplex == GMAC_PHY_HALF_DUPLEX) ? SUPPORTED_1000baseT_Half : SUPPORTED_1000baseT_Full;
        break;
    default:
        break;
    }
#endif
    advertise = dev->advertising;
    /* Only allow advertising what this PHY supports */
    advertise = advertise & dev->supported;

    /* Setup standard advertisement */
    err = gmac_mdio_read(dev->hal, MII_ADVERTISE, &adv);

    if (err < 0)
    {
        return err;
    }
    oldadv = adv;

    adv &= ~(ADVERTISE_ALL | ADVERTISE_100BASE4 | ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);
    if (advertise & ADVERTISED_10baseT_Half)
    {
        adv |= ADVERTISE_10HALF;
    }
    if (advertise & ADVERTISED_10baseT_Full)
    {
        adv |= ADVERTISE_10FULL;
    }
    if (advertise & ADVERTISED_100baseT_Half)
    {
        adv |= ADVERTISE_100HALF;
    }
    if (advertise & ADVERTISED_100baseT_Full)
    {
        adv |= ADVERTISE_100FULL;
    }
    if (advertise & ADVERTISED_Pause)
    {
        adv |= ADVERTISE_PAUSE_CAP;
    }
    if (advertise & ADVERTISED_Asym_Pause)
    {
        adv |= ADVERTISE_PAUSE_ASYM;
    }
    if (advertise & ADVERTISED_1000baseX_Half)
    {
        adv |= ADVERTISE_1000XHALF;
    }
    if (advertise & ADVERTISED_1000baseX_Full)
    {
        adv |= ADVERTISE_1000XFULL;
    }

    if (adv != oldadv)
    {
        err = gmac_mdio_write(dev->hal, MII_ADVERTISE, adv);

        if (err < 0)
        {
            return err;
        }
        changed = 1;
    }

    err = gmac_mdio_read(dev->hal, MII_BMSR, &bmsr);
    if (err < 0)
    {
        return err;
    }

    /* Per 802.3-2008, Section 22.2.4.2.16 Extended status all
     * 1000Mbits/sec capable PHYs shall have the BMSR_ESTATEN bit set to a
     * logical 1.
     */
    if (!(bmsr & BMSR_ESTATEN))
    {
        return changed;
    }

    /* Configure gigabit if it's supported */
    err = gmac_mdio_read(dev->hal, MII_CTRL1000, &adv);

    if (err < 0)
    {
        return err;
    }
    oldadv = adv;

    adv &= ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF);

    if (supported
        & (SUPPORTED_1000baseT_Half | SUPPORTED_1000baseT_Full))
    {
        if (advertise & SUPPORTED_1000baseT_Half)
        {
            adv |= ADVERTISE_1000HALF;
        }
        if (advertise & SUPPORTED_1000baseT_Full)
        {
            adv |= ADVERTISE_1000FULL;
        }
    }

    if (adv != oldadv)
    {
        changed = 1;
    }

    err = gmac_mdio_write(dev->hal, MII_CTRL1000, adv);
    if (err < 0)
    {
        return err;
    }

    return changed;
}

int gmac_dev_genphy_restart_aneg(struct gmac_dev *dev)
{
    int reg, err;

    err = gmac_mdio_read(dev->hal, MII_BMCR, &reg);
    if (err < 0)
	return err;
    reg |= (BMCR_ANENABLE | BMCR_ANRESTART);
    /* Don't isolate the PHY if we're negotiating */
    reg &= ~(BMCR_ISOLATE);
    err = gmac_mdio_write(dev->hal, MII_BMCR, reg);
    if (err < 0)
	return err;

    return err;
}

int gmac_dev_genphy_process_aneg_result(struct gmac_dev *dev, int result)
{
    int ctl, err;

    if (result == 0) {
	/*
	 * Advertisment hasn't changed, but maybe aneg was never on to
	 * begin with?	Or maybe phy was isolated?
	 */
	err = gmac_mdio_read(dev->hal, MII_BMCR, &ctl);

	if (err < 0)
		return err;

	if (!(ctl & BMCR_ANENABLE) || (ctl & BMCR_ISOLATE))
		result = 1; /* do restart aneg */
    }

    /*
     * Only restart aneg if we are advertising something different
     * than we were before.
     */
    if (result > 0)
	err =  gmac_dev_genphy_restart_aneg(dev);

    return err;
}

int genphy_update_link(struct gmac_dev *dev)
{
   unsigned int mii_reg;
   int err;

   /*
    * Wait if the link is up, and autonegotiation is in progress
    * (ie - we're capable and it's not done)
    */
    err = gmac_mdio_read(dev->hal, MII_BMSR, &mii_reg);
    if (err < 0)
	return err;
   /*
    * If we already saw the link up, and it hasn't gone down, then
    * we don't need to wait for autoneg again
    */
    if (dev->link_status && mii_reg & BMSR_LSTATUS)
	return 0;

    //(phydev->autoneg == AUTONEG_ENABLE) &&

    if (!(mii_reg & BMSR_ANEGCOMPLETE)) {
        hal_printf("Waiting for PHY auto negotiation to complete");
        while(1) {
            err = gmac_mdio_read(dev->hal, MII_BMSR, &mii_reg);
            if (err < 0)
                return err;
	    if (mii_reg & BMSR_ANEGCOMPLETE)
		break;
            hal_printf(".");
            sys_mdelay(500);
        }
        hal_printf("\n");
   }

   return 0;
}	  

int gmac_dev_genphy_reset(struct gmac_dev *dev)
{
    int reg, ret;
    int timeout = 500;

    if (gmac_mdio_write(dev->hal, MII_BMCR, BMCR_RESET) < 0)
    {
        hal_printf("PHY reset failed\n");
        return -1;
    }

    /*some phy need wait some time*/
    sys_mdelay(100);
    /*
     * Poll the control register for the reset bit to go to 0 (it is
     * auto-clearing).  This should happen within 0.5 seconds per the
     * IEEE spec.
     */
    ret = gmac_mdio_read(dev->hal, MII_BMCR, &reg);
    if (ret < 0)
	return ret;
    while ((reg & BMCR_RESET) && timeout--)
    {
        ret = gmac_mdio_read(dev->hal, MII_BMCR, &reg);

        if (ret < 0)
        {
            hal_printf("PHY status read failed\n");
            return -1;
        }
        sys_udelay(1000);
    }

    if (reg & BMCR_RESET)
    {
        hal_printf("PHY reset timed out\n");
        return -1;
    }

    return 0;
}

void phy_link_detect(void *param)
{
    gmac_handle_t *handle = param;
    rt_uint16_t bmsr = 0;
    rt_uint16_t link_status = 0;
    rt_uint16_t link_status_old = 0;
    rt_uint16_t phy_val;
    int ret = -1;

    while(1)
    {
        //hal_printf("link detect\n");
        ret = gmac_mdio_read(handle, MII_BMSR, &bmsr);
        link_status = bmsr & BMSR_LSTATUS;
        if(link_status_old != link_status)
        {
            if(link_status)
            {
                ret = gmac_phy_init(handle);
                if(ret == 0)
                {
                    gmac_link_change(handle, 1);
                }
            }
            else
            {
                if(link_status_old != link_status)
                {
                    gmac_link_change(handle, 0);
                }
            }

        }
        link_status_old = link_status;
	sys_tick_sleep(RT_TICK_PER_SECOND);
    }

}

int gmac_phy_init(gmac_handle_t * handle)
{
    struct gmac_dev *dev = handle->phy_dev;

    return dev->ops->init(dev);
}

int genric_gmac_phy_init(gmac_handle_t * handle)
{
    struct gmac_dev *dev;
    rt_uint32_t value;
    rt_uint16_t temp_val;
    int ret, i;

    dev = (struct gmac_dev *)hal_malloc(sizeof(struct gmac_dev));
    if (!dev)
	return -RT_ERROR;

    rt_memset(dev, 0, sizeof(struct gmac_dev));
    dev->advertising = PHY_GBIT_FEATURES;
    dev->supported = PHY_GBIT_FEATURES;

    //dev->name = "yt8531";
    dev->hal = handle;

    for (i = 0; i < 0x1f; i++) {
	handle->gmac_config.phy_addr = i;
	ret = gmac_mdio_read(handle, MII_PHYSID1, &temp_val);
	if (ret < 0)
		continue;
	value = temp_val << 16;
	ret = gmac_mdio_read(handle, MII_PHYSID2, &temp_val);
	if (ret < 0)
		continue;
	value |= temp_val;

	if ((value & 0x1fffffff) == 0x1fffffff) {
	    sys_tick_sleep(RT_TICK_PER_SECOND/100);
	    continue;
	}
        break;
    }

    if (handle->gmac_config.phy_addr == 0x1f) {
	hal_printf("No PHY device!\n");
	return -1;
    }

    ret = register_gmac_phy_driver(dev, value);
    if (ret)
	return ret;

    handle->phy_dev = dev;

    return 0;
}
