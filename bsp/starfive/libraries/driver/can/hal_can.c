
//#if defined(BSP_USING_CAN)

#include <rtthread.h>
#include <rtdevice.h>

//#include <drivers/can.h>
#include "hal_can.h"

#if 0
#define LOG_TAG    "drv.canfd"
#undef  DBG_ENABLE
#define DBG_SECTION_NAME   LOG_TAG
#define DBG_LEVEL       LOG_LVL_ERROR
#define DBG_COLOR
#include <rtdbg.h>
#endif

/* Private Define ---------------------------------------------------------------*/
#define IS_CAN_STDID(STDID)   ((STDID) <= 0x7FFU)
#define IS_CAN_EXTID(EXTID)   ((EXTID) <= 0x1FFFFFFFU)
#define IS_CAN_DLC(DLC)       ((DLC) <= 8U)

/* Default config for serial_configure structure */
#define IPMS_CANFD_CONFIG_DEFAULT                  \
{                                              \
    CAN1MBaud,           /* 1M bits/s       */ \
    RT_CANMSG_BOX_SZ,    /* message box max size */ \
    RT_CANSND_BOX_NUM,   /* message box number   */ \
    RT_CAN_MODE_NORMAL,  /* Normal mode     */ \
    0,                   /* privmode        */ \
    0,                   /* reserved        */ \
    100,                 /* Timeout Tick    */ \
}

enum
{
    CANFD_START = -1,
#if defined(BSP_USING_CAN0)
    CAN0_IDX,
#endif
#if defined(BSP_USING_CAN1)
    CAN1_IDX,
#endif
    CANFD_CNT
};

/* Private functions ------------------------------------------------------------*/
static rt_err_t canfd_configure(struct rt_can_device *can, struct can_configure *cfg);
static rt_err_t canfd_control(struct rt_can_device *can, int cmd, void *arg);
static rt_ssize_t canfd_sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t boxno);
static rt_ssize_t canfd_recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t boxno);

static struct hal_canfd cans_inst[] =
{
#if defined(BSP_USING_CAN0)
    {
        .cfg = {.name = "can0"},
	.index = 0,
    },
#endif
#if defined(BSP_USING_CAN1)
    {
        .cfg = {.name = "can1"},
	.index = 1,
    },
#endif
}; /* struct nu_can */

/* Public functions ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/
static const struct rt_can_ops canfd_ops =
{
    .configure = canfd_configure,
    .control = canfd_control,
    .sendmsg = canfd_sendmsg,
    .recvmsg = canfd_recvmsg,
};

static const struct can_configure ipms_canfd_default_config = IPMS_CANFD_CONFIG_DEFAULT;

/* Interrupt Handle Function  ----------------------------------------------------*/

static void canfd_isr(int vector, void *param)
{
    /* Get base address of canfd register */
    struct hal_canfd *can = (struct hal_canfd *)param;
    rt_uint32_t status;

    status = can->can_ops->can_isr(can->priv);

    if (status & CAN_TX_DONE)
    {
        if (can->int_flag & RT_DEVICE_FLAG_INT_TX)
        {
            rt_hw_can_isr(&can->dev, RT_CAN_EVENT_TX_DONE);
        }
    }

    if (status & CAN_RX_IND)
    {
        if (can->int_flag & RT_DEVICE_FLAG_INT_RX)
        {
            rt_hw_can_isr(&can->dev, RT_CAN_EVENT_RX_IND);
        }
    }

    if (status & CAN_RX_OVERFLOW)
    {
        rt_hw_can_isr(&can->dev, RT_CAN_EVENT_RXOF_IND);
    }

    if (status & CAN_ERROR)
    {
        rt_hw_can_isr(&can->dev, RT_CAN_EVENT_TX_FAIL);
    }
}

static void ipms_canfd_ie(struct hal_canfd *ipms)
{
}

static rt_err_t canfd_configure(struct rt_can_device *dev, struct can_configure *cfg)
{
    struct hal_canfd *can  = (struct hal_canfd *)dev;
    int ctrlmode = 0;
    int ret;

    RT_ASSERT(can);
    RT_ASSERT(cfg);
    switch (cfg->mode)
    {
    case RT_CAN_MODE_NORMAL:    // Normal
        break;
    case RT_CAN_MODE_LOOPBACK:  // Test - Internal loopback
        ctrlmode |=  CAN_CTRLMODE_LOOPBACK;
        break;

    default:
        rt_kprintf("Unsupported Operating mode\n");
        return RT_ERROR;
    }

    ret = can->can_ops->canfd_init(can, ctrlmode);

    if (ret)
	return -RT_ERROR;

    return RT_EOK;

   // return -(RT_ERROR);
}

static rt_err_t canfd_control(struct rt_can_device *dev, int cmd, void *arg)
{
    unsigned long argval = (unsigned long)arg;
    struct hal_canfd *can = (struct hal_canfd *)dev;

    RT_ASSERT(can);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_SET_INT:
	can->int_flag |= argval;
	break;
    case RT_DEVICE_CTRL_CLR_INT:
	can->int_flag &= ~argval;
	break;
    case RT_CAN_CMD_SET_MODE:
        if ((argval == RT_CAN_MODE_NORMAL) ||
                //(argval == RT_CAN_MODE_LISTEN) ||
                (argval == RT_CAN_MODE_LOOPBACK))
                //(argval == RT_CAN_MODE_LOOPBACKANLISTEN)
        {
            if (argval != dev->config.mode)
            {
                dev->config.mode = argval;
                return canfd_configure(dev, &dev->config);
            }
        }
        else
        {
            return -(RT_ERROR);
        }
        break;

    case RT_CAN_CMD_SET_BAUD:
    {
        if ((argval == CAN1MBaud) ||
                (argval == CAN800kBaud) ||
                (argval == CAN500kBaud) ||
                (argval == CAN250kBaud) ||
                (argval == CAN125kBaud) ||
                (argval == CAN100kBaud) ||
                (argval == CAN50kBaud) ||
                (argval == CAN20kBaud) ||
                (argval == CAN10kBaud))
        {
            if (argval != dev->config.baud_rate)
            {
                dev->config.baud_rate = argval;
		can->baudrate = argval;
                //return ipms_canfd_configure(can, &can->config);
            }
        }
        else
        {
            return -(RT_ERROR);
        }
    }
    break;
    case RT_CAN_CMD_GET_STATUS:
    {
 #if 0
        rt_uint32_t u32ErrCounter = psNuCANFD->base->ECR;
        rt_uint32_t u32ProtocolStatus = psNuCANFD->base->PSR;

        RT_ASSERT(arg);

        /*Receive Error Counter, return value is with Receive Error Passive.*/
        can->status.rcverrcnt = ((u32ErrCounter & CANFD_ECR_REC_Msk) >> CANFD_ECR_REC_Pos);

        /*Transmit Error Counter*/
        can->status.snderrcnt = ((u32ErrCounter & CANFD_ECR_TEC_Msk) >> CANFD_ECR_TEC_Pos);

        /*Last Error Type*/
        can->status.lasterrtype = ((u32ProtocolStatus & CANFD_PSR_LEC_Msk) >> CANFD_PSR_LEC_Pos);

        /*Status error code*/
        can->status.errcode = (u32ProtocolStatus & CANFD_PSR_EW_Msk) ? 1 :
                              (u32ProtocolStatus & CANFD_PSR_EP_Msk) ? 2 :
                              (u32ProtocolStatus & CANFD_PSR_BO_Msk) ? 3 :
                              0;

        rt_memcpy(arg, &can->status, sizeof(struct rt_can_status));
#endif
    }
    break;

    default:
        return -(RT_EINVAL);

    }

    return RT_EOK;
}

static rt_ssize_t canfd_sendmsg(struct rt_can_device *dev, const void *buf, rt_uint32_t boxno)
{
    struct hal_canfd *can = (struct hal_canfd *)dev;
    struct rt_can_msg *pmsg;
    int ret;

    RT_ASSERT(can);
    RT_ASSERT(buf);

    pmsg = (struct rt_can_msg *) buf;
    //if (pmsg->len == sizeof(struct rt_can_msg))
	//ret = canfd_driver_start_xmit(buf, ipms->priv, 1);
    //else
    ret = can->can_ops->start_xmit(buf, can->priv, 0);
	//ret = canfd_driver_start_xmit(buf, can->priv, 0);
    if (ret <  0)
	return (rt_ssize_t)-RT_ERROR;

    return (rt_ssize_t)RT_EOK;
}

static rt_ssize_t canfd_recvmsg(struct rt_can_device *dev, void *buf, rt_uint32_t boxno)
{
    struct hal_canfd *can = (struct hal_canfd *)dev;
    int ret;
    //struct rt_can_msg *pmsg;

    RT_ASSERT(can);
    RT_ASSERT(buf);

    //pmsg = (struct rt_can_msg *) buf;
    ret = can->can_ops->rx_poll(can->priv, buf);

    if (ret <  0)
	return (rt_ssize_t)-RT_ERROR;

    return (rt_ssize_t)RT_EOK;
}

/**
 * Hardware CAN Initialization
 */
int rt_hw_canfd_init(void)
{
    int i;
    rt_err_t ret = RT_EOK;

    for (i = (CANFD_START + 1); i < CANFD_CNT; i++)
    {
        can_plat_init(&cans_inst[i]);
        cans_inst[i].dev.config = ipms_canfd_default_config;

#ifdef RT_CAN_USING_HDR
        cans_inst[i].dev.config.maxhdr = RT_CANMSG_BOX_SZ;
#endif
	ipms_ops_init(&cans_inst[i]);
        /* Register can device */
        ret = rt_hw_can_register(&cans_inst[i].dev, cans_inst[i].cfg.name, &canfd_ops, NULL);
        RT_ASSERT(ret == RT_EOK);

        /* Register ISR. */
        rt_hw_interrupt_install(cans_inst[i].irq, canfd_isr, (void *)&cans_inst[i], cans_inst[i].cfg.name);

        /* Unmask interrupt. */
        rt_hw_interrupt_umask(cans_inst[i].irq);
    }

    return (int)ret;
}
//#endif
