// SPDX-License-Identifier: GPL-2.0
/*
 * StarFive Controller Area Network Host Controller Driver
 *
 * Copyright (c) 2022 StarFive Technology Co., Ltd.
 */
#include <rtthread.h>
#include <rtdevice.h>

#include "hal_can.h"
#include "board.h"
#include <riscv_io.h>
#include <io.h>
#include "ipms_canfd.h"

#define DRIVER_NAME "ipms_canfd"

/* CAN registers set */
enum canfd_device_reg {
	CANFD_RUBF_OFFSET           =   0x00,	/* Receive Buffer Registers 0x00-0x4f */
	CANFD_RUBF_ID_OFFSET        =   0x00,
	CANFD_RBUF_CTL_OFFSET       =   0x04,
	CANFD_RBUF_DATA_OFFSET      =   0x08,
	CANFD_TBUF_OFFSET           =   0x50,	/* Transmit Buffer Registers 0x50-0x97 */
	CANFD_TBUF_ID_OFFSET        =   0x50,
	CANFD_TBUF_CTL_OFFSET       =   0x54,
	CANFD_TBUF_DATA_OFFSET      =   0x58,
	CANFD_TTS_OFFSET            =   0x98,	/* Transmission Time Stamp 0x98-0x9f */
	CANFD_CFG_STAT_OFFSET       =   0xa0,
	CANFD_TCMD_OFFSET           =   0xa1,
	CANFD_TCTRL_OFFSET          =   0xa2,
	CANFD_RCTRL_OFFSET          =   0xa3,
	CANFD_RTIE_OFFSET           =   0xa4,
	CANFD_RTIF_OFFSET           =   0xa5,
	CANFD_ERRINT_OFFSET         =   0xa6,
	CANFD_LIMIT_OFFSET          =   0xa7,
	CANFD_S_SEG_1_OFFSET        =   0xa8,
	CANFD_S_SEG_2_OFFSET        =   0xa9,
	CANFD_S_SJW_OFFSET          =   0xaa,
	CANFD_S_PRESC_OFFSET        =   0xab,
	CANFD_F_SEG_1_OFFSET        =   0xac,
	CANFD_F_SEG_2_OFFSET        =   0xad,
	CANFD_F_SJW_OFFSET          =   0xae,
	CANFD_F_PRESC_OFFSET        =   0xaf,
	CANFD_EALCAP_OFFSET         =   0xb0,
	CANFD_RECNT_OFFSET          =   0xb2,
	CANFD_TECNT_OFFSET          =   0xb3,
};

enum canfd_reg_bitchange {
	CAN_FD_SET_RST_MASK         =   0x80,	/* Set Reset Bit */
	CAN_FD_OFF_RST_MASK         =   0x7f,	/* Reset Off Bit */
	CAN_FD_SET_FULLCAN_MASK     =   0x10,	/* set TTTBM as 1->full TTCAN mode */
	CAN_FD_OFF_FULLCAN_MASK     =   0xef,	/* set TTTBM as 0->separate PTB and STB mode */
	CAN_FD_SET_FIFO_MASK        =   0x20,	/* set TSMODE as 1->FIFO mode */
	CAN_FD_OFF_FIFO_MASK        =   0xdf,	/* set TSMODE as 0->Priority mode */
	CAN_FD_SET_TSONE_MASK       =   0x04,
	CAN_FD_OFF_TSONE_MASK       =   0xfb,
	CAN_FD_SET_TSALL_MASK       =   0x02,
	CAN_FD_OFF_TSALL_MASK       =   0xfd,
	CAN_FD_LBMEMOD_MASK         =   0x40,	/* set loop back mode, external */
	CAN_FD_LBMIMOD_MASK         =   0x20,	/* set loopback internal mode */
	CAN_FD_SET_BUSOFF_MASK      =   0x01,
	CAN_FD_OFF_BUSOFF_MASK      =   0xfe,
	CAN_FD_SET_TTSEN_MASK       =   0x80,	/* set ttsen, tts update enable */
	CAN_FD_SET_BRS_MASK         =   0x10,	/* can fd Bit Rate Switch mask */
	CAN_FD_OFF_BRS_MASK         =   0xef,
	CAN_FD_SET_EDL_MASK         =   0x20,	/* Extended Data Length */
	CAN_FD_OFF_EDL_MASK         =   0xdf,
	CAN_FD_SET_DLC_MASK         =   0x0f,
	CAN_FD_SET_TENEXT_MASK      =   0x40,
	CAN_FD_SET_IDE_MASK         =   0x80,
	CAN_FD_OFF_IDE_MASK         =   0x7f,
	CAN_FD_SET_RTR_MASK         =   0x40,
	CAN_FD_OFF_RTR_MASK         =   0xbf,
	CAN_FD_INTR_ALL_MASK        =   0xff,	/* all interrupts enable mask */
	CAN_FD_SET_RIE_MASK         =   0x80,
	CAN_FD_OFF_RIE_MASK         =   0x7f,
	CAN_FD_SET_RFIE_MASK        =   0x20,
	CAN_FD_OFF_RFIE_MASK        =   0xdf,
	CAN_FD_SET_RAFIE_MASK       =   0x10,
	CAN_FD_OFF_RAFIE_MASK       =   0xef,
	CAN_FD_SET_EIE_MASK         =   0x02,
	CAN_FD_OFF_EIE_MASK         =   0xfd,
	CAN_FD_TASCTIVE_MASK        =   0x02,
	CAN_FD_RASCTIVE_MASK        =   0x04,
	CAN_FD_SET_TBSEL_MASK       =   0x80,	/* message writen in STB */
	CAN_FD_OFF_TBSEL_MASK       =   0x7f,	/* message writen in PTB */
	CAN_FD_SET_STBY_MASK        =   0x20,
	CAN_FD_OFF_STBY_MASK        =   0xdf,
	CAN_FD_SET_TPE_MASK         =   0x10,	/* Transmit primary enable */
	CAN_FD_SET_TPA_MASK         =   0x08,
	CAN_FD_SET_SACK_MASK        =   0x80,
	CAN_FD_SET_RREL_MASK        =   0x10,
	CAN_FD_RSTAT_NOT_EMPTY_MASK =   0x03,
	CAN_FD_SET_RIF_MASK         =   0x80,
	CAN_FD_OFF_RIF_MASK         =   0x7f,
	CAN_FD_SET_RAFIF_MASK       =   0x10,
	CAN_FD_SET_RFIF_MASK        =   0x20,
	CAN_FD_SET_TPIF_MASK        =   0x08,	/* Transmission Primary Interrupt Flag */
	CAN_FD_SET_TSIF_MASK        =   0x04,
	CAN_FD_SET_EIF_MASK         =   0x02,
	CAN_FD_SET_AIF_MASK         =   0x01,
	CAN_FD_SET_EWARN_MASK       =   0x80,
	CAN_FD_SET_EPASS_MASK       =   0x40,
	CAN_FD_SET_EPIE_MASK        =   0x20,
	CAN_FD_SET_EPIF_MASK        =   0x10,
	CAN_FD_SET_ALIE_MASK        =   0x08,
	CAN_FD_SET_ALIF_MASK        =   0x04,
	CAN_FD_SET_BEIE_MASK        =   0x02,
	CAN_FD_SET_BEIF_MASK        =   0x01,
	CAN_FD_OFF_EPIE_MASK        =   0xdf,
	CAN_FD_OFF_BEIE_MASK        =   0xfd,
	CAN_FD_SET_AFWL_MASK        =   0x40,
	CAN_FD_SET_EWL_MASK         =   0x0b,
	CAN_FD_SET_KOER_MASK        =   0xe0,
	CAN_FD_SET_BIT_ERROR_MASK   =   0x20,
	CAN_FD_SET_FORM_ERROR_MASK  =   0x40,
	CAN_FD_SET_STUFF_ERROR_MASK =   0x60,
	CAN_FD_SET_ACK_ERROR_MASK   =   0x80,
	CAN_FD_SET_CRC_ERROR_MASK   =   0xa0,
	CAN_FD_SET_OTH_ERROR_MASK   =   0xc0,
};

/* seg1,seg2,sjw,prescaler all have 8 bits */
#define BITS_OF_BITTIMING_REG		8

/* in can_bittiming strucure every field has 32 bits---->unsigned int */
#define FBITS_IN_BITTIMING_STR		32
#define SEG_1_SHIFT			0
#define SEG_2_SHIFT			8
#define SJW_SHIFT			16
#define PRESC_SHIFT			24

/* TTSEN bit used for 32 bit register read or write */
#define TTSEN_8_32_SHIFT		24
#define RTR_32_8_SHIFT			24

/* transmit mode */
#define XMIT_FULL			0
#define XMIT_SEP_FIFO			1
#define XMIT_SEP_PRIO			2
#define XMIT_PTB_MODE			3


/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error message frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

#define CANFD_BRS 0x01 /* bit rate switch (second bitrate for payload data) */
#define CANFD_ESI 0x02 /* error state indicator of the transmitting node */
#define CANFD_FDF 0x04 /* mark CAN FD for dual use of struct canfd_frame */

enum  IPMS_CAN_TYPE {
	IPMS_CAN_TYPY_CAN 	= 0,
	IPMS_CAN_TYPE_CANFD,
};

#define CAN_MAX_DLC 8
#define CANFD_MAX_DLC 15

#define min_t(type, x, y) ({			\
	type __min1 = (x);			\
	type __min2 = (y);			\
	__min1 < __min2 ? __min1: __min2; })

#define max_t(type, x, y) ({			\
	type __max1 = (x);			\
	type __max2 = (y);			\
	__max1 > __max2 ? __max1: __max2; })

#define get_can_dlc(i)		(min_t(uint8_t, (i), CAN_MAX_DLC))
#define get_canfd_dlc(i)	(min_t(uint8_t, (i), CANFD_MAX_DLC))

enum can_state {
	CAN_STATE_ERROR_ACTIVE = 0,	/* RX/TX error count < 96 */
	CAN_STATE_ERROR_WARNING,	/* RX/TX error count < 128 */
	CAN_STATE_ERROR_PASSIVE,	/* RX/TX error count < 256 */
	CAN_STATE_BUS_OFF,		/* RX/TX error count >= 256 */
	CAN_STATE_STOPPED,		/* Device is stopped */
	CAN_STATE_SLEEPING,		/* Device is sleeping */
	CAN_STATE_MAX
};

struct can_bittiming {
	uint32_t act_bitrate;		/* Bit-rate in bits/second */
	uint32_t sample_point;	/* Sample point in one-tenth of a percent */
	uint32_t tq;		/* Time quanta (TQ) in nanoseconds */
	uint32_t prop_seg;		/* Propagation segment in TQs */
	uint32_t phase_seg1;	/* Phase buffer segment 1 in TQs */
	uint32_t phase_seg2;	/* Phase buffer segment 2 in TQs */
	uint32_t sjw;		/* Synchronisation jump width in TQs */
	uint32_t brp;		/* Bit-rate prescaler */
};

struct can_bittiming_const {
	char name[16];		/* Name of the CAN controller hardware */
	uint32_t tseg1_min;	/* Time segment 1 = prop_seg + phase_seg1 */
	uint32_t tseg1_max;
	uint32_t tseg2_min;	/* Time segment 2 = phase_seg2 */
	uint32_t tseg2_max;
	uint32_t sjw_max;		/* Synchronisation jump width */
	uint32_t brp_min;		/* Bit-rate prescaler */
	uint32_t brp_max;
	uint32_t brp_inc;
};

#define CAN_MAX_DLEN 8
#define CANFD_MAX_DLEN 64
struct can_priv {
	struct can_bittiming_const *bittiming_const;
	struct can_bittiming_const *canfd_data_bittiming_const;
	struct can_bittiming bittiming;
	struct can_bittiming data_bittiming;
	uint32_t ctrlmode_supported;	/* options that can be modified by netlink */
	uint32_t ctrlmode;
	enum can_state state;
};

struct can_frame {
	uint32_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	union {
		/* CAN frame payload length in byte (0 .. CAN_MAX_DLEN)
		 * was previously named can_dlc so we need to carry that
		 * name for legacy support
		 */
		unsigned char len;
		unsigned char can_dlc; /* deprecated */
	} __attribute__((packed)); /* disable padding added in some ABIs */
	unsigned char __pad; /* padding */
	unsigned char __res0; /* reserved / padding */
	unsigned char len8_dlc; /* optional DLC for 8 byte payload length (9 .. 15) */
	unsigned char data[CAN_MAX_DLEN] __attribute__((aligned(8)));
};

struct canfd_frame {
	uint32_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	unsigned char    len;     /* frame payload length in byte */
	unsigned char    flags;   /* additional flags for CAN FD */
	unsigned char    __res0;  /* reserved / padding */
	unsigned char    __res1;  /* reserved / padding */
	unsigned char    data[CANFD_MAX_DLEN] __attribute__((aligned(8)));
};

struct ipms_canfd_priv {
	struct can_priv can;
	void *reg_base;
	uint32_t (*read_reg)(const struct ipms_canfd_priv *priv, enum canfd_device_reg reg);
	void (*write_reg)(const struct ipms_canfd_priv *priv, enum canfd_device_reg reg, uint32_t val);
	uint32_t tx_mode;
	struct canfd_cfg *can_cfg;
};

struct ipms_baud_rate
{
    uint32_t baud_rate;
    struct can_bittiming bittiming;
};

static struct ipms_baud_rate bt_table[] =
{                 /*actrate sample_cnt tc p_seg p_seg1 p_seg2 sjw brp */
    {CAN1MBaud,   {990000,     750,   126,  2,   3,     2,     1,   5}},
    {CAN800kBaud, {792000,     800,   126,  3,   4,     2,     1,   5}},
    {CAN500kBaud, {495000,     875,   126,  6,   7,     2,     1,   5}},
    {CAN250kBaud, {247500,     875,   251,  6,   7,     2,     1,  10}},
    {CAN125kBaud, {124528,     666,  1338,  1,   2,     2,     1,  53}},
    {CAN100kBaud, {100000,     833,   833,  4,   5,     2,     1,  33}},
    {CAN50kBaud,  {50000,      833,  1666,  4,   5,     2,     1,  66 }},
    {CAN20kBaud,  {20000,      866,  3333,  6,   6,     2,     1,  132}},
    {CAN10kBaud,  {10000,      866,  6666,  6,   6,     2,     1,  264}},
};

static struct ipms_baud_rate dbt_table[] =
{                 /*actrate sample_cnt tc p_seg p_seg1 p_seg2 sjw brp */
    {CAN1MBaud,   {990000,     750,   126,  2,   3,     2,     1,   5}},
    {CAN800kBaud, {792000,     800,   126,  3,   4,     2,     1,   5}},
    {CAN500kBaud, {495000,     875,   126,  6,   7,     2,     1,   5}},
    {CAN250kBaud, {247500,     875,   251,  6,   7,     2,     1,  10}},
    {CAN125kBaud, {125316,     500,  1994,  0,   1,     2,     1,  79}},
    {CAN100kBaud, {100000,     833,   833,  4,   5,     2,     1,  33}},
    {CAN50kBaud,  {50000,      833,  1666,  4,   5,     2,     1,  66 }},
    {CAN20kBaud,  {20000,      866,  3333,  6,   6,     2,     1,  132}},
    {CAN10kBaud,  {10000,      866,  6666,  6,   6,     2,     1,  264}},
};

static struct can_bittiming_const canfd_bittiming_const = {
	.name = DRIVER_NAME,
	.tseg1_min = 2,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 512,
	.brp_inc = 1,
};

static struct can_bittiming_const canfd_data_bittiming_const = {
	.name = DRIVER_NAME,
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 8,
	.brp_min = 1,
	.brp_max = 512,
	.brp_inc = 1,
};

static const unsigned char len2dlc[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8,      /* 0 - 8 */
        9, 9, 9, 9,                     /* 9 - 12 */
        10, 10, 10, 10,                 /* 13 - 16 */
        11, 11, 11, 11,                 /* 17 - 20 */
        12, 12, 12, 12,                 /* 21 - 24 */
        13, 13, 13, 13, 13, 13, 13, 13, /* 25 - 32 */
        14, 14, 14, 14, 14, 14, 14, 14, /* 33 - 40 */
        14, 14, 14, 14, 14, 14, 14, 14, /* 41 - 48 */
        15, 15, 15, 15, 15, 15, 15, 15, /* 49 - 56 */
        15, 15, 15, 15, 15, 15, 15, 15  /* 57 - 64 */
};

/* map the sanitized data length to an appropriate data length code */
static unsigned char can_fd_len2dlc(unsigned char len)
{
        /* check for length mapping table size at build time */
       // BUILD_BUG_ON(ARRAY_SIZE(len2dlc) != CANFD_MAX_DLEN + 1);

        if (len > CANFD_MAX_DLEN)
                return CANFD_MAX_DLC;

        return len2dlc[len];
}

static void canfd_write_reg_le(const struct ipms_canfd_priv *priv,
				enum canfd_device_reg reg, unsigned int val)
{
	sys_writel(val, priv->reg_base + reg);
}

static unsigned int canfd_read_reg_le(const struct ipms_canfd_priv *priv,
				enum canfd_device_reg reg)
{
	return sys_readl(priv->reg_base + reg);
}

#define ALIGN_DOWN(addr, align) ((addr) + (align) & ~(align))

static inline unsigned char can_ioread8(const void  *addr)
{
	void  *addr_down;
	union val {
		unsigned char val_8[4];
		unsigned int val_32;
	} val;
	unsigned char offset = 0;

	addr_down = (void  *)ALIGN_DOWN((unsigned long)addr, 4);
	offset = addr - addr_down;
	val.val_32 = sys_readl(addr_down);
	return val.val_8[offset];
}

static inline void can_iowrite8(unsigned char value, void  *addr)
{
	void  *addr_down;
	union val {
		unsigned char val_8[4];
		unsigned int val_32;
	} val;
	unsigned char offset = 0;

	addr_down = (void *)ALIGN_DOWN((unsigned long)addr, 4);
	offset = addr - addr_down;
	val.val_32 = sys_readl(addr_down);
	val.val_8[offset] = value;
	sys_writel(val.val_32, addr_down);
}

static void canfd_reigister_set_bit(const struct ipms_canfd_priv *priv,
					enum canfd_device_reg reg,
					enum canfd_reg_bitchange set_mask)
{
	void  *addr_down;
	union val {
		unsigned char val_8[4];
		unsigned int val_32;
	} val;
	unsigned char offset = 0;

	addr_down = (void *)ALIGN_DOWN((unsigned long)(priv->reg_base + reg), 4);
	offset = (priv->reg_base + reg) - addr_down;
	val.val_32 = sys_readl(addr_down);
	val.val_8[offset] |= set_mask;
	sys_writel(val.val_32, addr_down);
}

static void canfd_reigister_off_bit(const struct ipms_canfd_priv *priv,
					enum canfd_device_reg reg,
					enum canfd_reg_bitchange set_mask)
{
	void  *addr_down;
	union val {
		unsigned char val_8[4];
		unsigned int val_32;
	} val;
	unsigned char offset = 0;

	addr_down = (void *)ALIGN_DOWN((unsigned long)(priv->reg_base + reg), 4);
	offset = (priv->reg_base + reg) - addr_down;
	val.val_32 = sys_readl(addr_down);
	val.val_8[offset] &= set_mask;
	sys_writel(val.val_32, addr_down);
}

static int canfd_device_driver_bittime_configuration(struct ipms_canfd_priv *priv)
{
	struct can_bittiming *bt = &priv->can.bittiming;
	struct can_bittiming *dbt = &priv->can.data_bittiming;
	unsigned int reset_test, bittiming_temp, dat_bittiming;

	reset_test = can_ioread8(priv->reg_base + CANFD_CFG_STAT_OFFSET);

	if (!(reset_test & CAN_FD_SET_RST_MASK)) {
		hal_printf("%s Not in reset mode, cannot set bit timing\n", priv->can_cfg->name);
		return -1;
	}

	bittiming_temp = ((bt->phase_seg1 + bt->prop_seg + 1 - 2) << SEG_1_SHIFT) |
			 ((bt->phase_seg2 - 1) << SEG_2_SHIFT) |
			 ((bt->sjw - 1) << SJW_SHIFT) |
			 ((bt->brp - 1) << PRESC_SHIFT);

	/* Check the bittime parameter */
	if ((((int)(bt->phase_seg1 + bt->prop_seg + 1) - 2) < 0) ||
		(((int)(bt->phase_seg2) - 1) < 0) ||
		(((int)(bt->sjw) - 1) < 0) ||
		(((int)(bt->brp) - 1) < 0)) {
		hal_printf("%s bittime fail\n", priv->can_cfg->name);
		return -1;
	}
	priv->write_reg(priv, CANFD_S_SEG_1_OFFSET, bittiming_temp);

	if (priv->can_cfg->is_canfd == IPMS_CAN_TYPE_CANFD) {
		dat_bittiming = ((dbt->phase_seg1 + dbt->prop_seg + 1 - 2) << SEG_1_SHIFT) |
				((dbt->phase_seg2 - 1) << SEG_2_SHIFT) |
				((dbt->sjw - 1) << SJW_SHIFT) |
				((dbt->brp - 1) << PRESC_SHIFT);

		if ((((int)(dbt->phase_seg1 + dbt->prop_seg + 1) - 2) < 0) ||
			(((int)(dbt->phase_seg2) - 1) < 0) ||
			(((int)(dbt->sjw) - 1) < 0) ||
			(((int)(dbt->brp) - 1) < 0)) {
			hal_printf("%s canfd bittime fail\n", priv->can_cfg->name);
			return -1;
		}

		priv->write_reg(priv, CANFD_F_SEG_1_OFFSET, dat_bittiming);
	}

	canfd_reigister_off_bit(priv, CANFD_CFG_STAT_OFFSET, CAN_FD_OFF_RST_MASK);

	hal_printf("%s Slow bit rate: %08x\n", priv->can_cfg->name, priv->read_reg(priv, CANFD_S_SEG_1_OFFSET));
	hal_printf("%s Fast bit rate: %08x\n", priv->can_cfg->name, priv->read_reg(priv, CANFD_F_SEG_1_OFFSET));

	return 0;
}

#if 0
int canfd_get_freebuffer(struct ipms_canfd_priv *priv)
{
	/* Get next transmit buffer */
	canfd_reigister_set_bit(priv, CANFD_TCTRL_OFFSET, CAN_FD_SET_TENEXT_MASK);

	if (can_ioread8(priv->reg_base + CANFD_TCTRL_OFFSET) & CAN_FD_SET_TENEXT_MASK)
		return -1;

	return 0;
}
#endif

static void canfd_tx_interrupt(struct ipms_canfd_priv *priv, unsigned char isr)
{
	/* wait till transmission of the PTB or STB finished */
	while (isr & (CAN_FD_SET_TPIF_MASK | CAN_FD_SET_TSIF_MASK)) {
		if (isr & CAN_FD_SET_TPIF_MASK)
			canfd_reigister_set_bit(priv, CANFD_RTIF_OFFSET, CAN_FD_SET_TPIF_MASK);

		if (isr & CAN_FD_SET_TSIF_MASK)
			canfd_reigister_set_bit(priv, CANFD_RTIF_OFFSET, CAN_FD_SET_TSIF_MASK);

		isr = can_ioread8(priv->reg_base + CANFD_RTIF_OFFSET);
	}
	//netif_wake_queue(ndev);
}

static int can_rx(struct ipms_canfd_priv *priv, void *data)
{
	struct can_frame *cf = (void *)data;
	//struct net_device_stats *stats = &ndev->stats;
	unsigned int can_id;
	unsigned char  dlc, control, rx_status;

	rx_status = can_ioread8(priv->reg_base + CANFD_RCTRL_OFFSET);

	if (!(rx_status & CAN_FD_RSTAT_NOT_EMPTY_MASK))
		return -1;
	control = can_ioread8(priv->reg_base + CANFD_RBUF_CTL_OFFSET);
	can_id = priv->read_reg(priv, CANFD_RUBF_ID_OFFSET);
	dlc = can_ioread8(priv->reg_base + CANFD_RBUF_CTL_OFFSET) & CAN_FD_SET_DLC_MASK;

#if 0
	skb = alloc_can_skb(ndev, (struct can_frame **)&cf);
	if (!skb) {
		stats->rx_dropped++;
		return 0;
	}
#endif
	cf->can_dlc = get_can_dlc(dlc);

	/* change the CANFD id into socketcan id format */
	if (control & CAN_FD_SET_IDE_MASK) {
		cf->can_id = can_id;
		cf->can_id |= CAN_EFF_FLAG;
	} else {
		cf->can_id = can_id;
		cf->can_id &= (~CAN_EFF_FLAG);
	}

	if (control & CAN_FD_SET_RTR_MASK)
		cf->can_id |= CAN_RTR_FLAG;

	if (!(control & CAN_FD_SET_RTR_MASK)) {
		*((unsigned int *)(cf->data + 0)) = priv->read_reg(priv, CANFD_RBUF_DATA_OFFSET);
		*((unsigned int *)(cf->data + 4)) = priv->read_reg(priv, CANFD_RBUF_DATA_OFFSET + 4);
	}

	canfd_reigister_set_bit(priv, CANFD_RCTRL_OFFSET, CAN_FD_SET_RREL_MASK);
	//stats->rx_bytes += can_fd_dlc2len(cf->can_dlc);
	//stats->rx_packets++;
	//netif_receive_skb(skb);

	return 0;
}

static int canfd_rx(struct ipms_canfd_priv *priv, struct canfd_frame *cf)
{
	unsigned int can_id;
	unsigned char  dlc, control, rx_status;
	int i;

	rx_status = can_ioread8(priv->reg_base + CANFD_RCTRL_OFFSET);

	if (!(rx_status & CAN_FD_RSTAT_NOT_EMPTY_MASK))
		return -1;
	control = can_ioread8(priv->reg_base + CANFD_RBUF_CTL_OFFSET);
	can_id = priv->read_reg(priv, CANFD_RUBF_ID_OFFSET);
	dlc = can_ioread8(priv->reg_base + CANFD_RBUF_CTL_OFFSET) & CAN_FD_SET_DLC_MASK;

#if 0
	if (control & CAN_FD_SET_EDL_MASK)
		/* allocate sk_buffer for canfd frame */
		skb = alloc_canfd_skb(ndev, &cf);
	else
		/* allocate sk_buffer for can frame */
		skb = alloc_can_skb(ndev, (struct can_frame **)&cf);

	if (!skb) {
		stats->rx_dropped++;
		return 0;
	}
#endif
	/* change the CANFD or CAN2.0 data into socketcan data format */
	if (control & CAN_FD_SET_EDL_MASK)
		cf->len = get_canfd_dlc(dlc);
	else
		cf->len = get_can_dlc(dlc);

	/* change the CANFD id into socketcan id format */
	if (control & CAN_FD_SET_EDL_MASK) {
		cf->can_id = can_id;
		if (control & CAN_FD_SET_IDE_MASK)
			cf->can_id |= CAN_EFF_FLAG;
		else
			cf->can_id &= (~CAN_EFF_FLAG);
	} else {
		cf->can_id = can_id;
		if (control & CAN_FD_SET_IDE_MASK)
			cf->can_id |= CAN_EFF_FLAG;
		else
			cf->can_id &= (~CAN_EFF_FLAG);

		if (control & CAN_FD_SET_RTR_MASK)
			cf->can_id |= CAN_RTR_FLAG;
	}

	/* CANFD frames handed over to SKB */
	if (control & CAN_FD_SET_EDL_MASK) {
		for (i = 0; i < cf->len; i += 4)
			*((unsigned int *)(cf->data + i)) = priv->read_reg(priv, CANFD_RBUF_DATA_OFFSET + i);
	} else {
		/* skb reads the received datas, if the RTR bit not set */
		if (!(control & CAN_FD_SET_RTR_MASK)) {
			*((unsigned int *)(cf->data + 0)) = priv->read_reg(priv, CANFD_RBUF_DATA_OFFSET);
			*((unsigned int *)(cf->data + 4)) = priv->read_reg(priv, CANFD_RBUF_DATA_OFFSET + 4);
		}
	}

	canfd_reigister_set_bit(priv, CANFD_RCTRL_OFFSET, CAN_FD_SET_RREL_MASK);

#if 0
	stats->rx_bytes += cf->len;
	stats->rx_packets++;
	netif_receive_skb(skb);
#endif
	return 1;
}

static int canfd_rx_poll(void *can_priv, void *data)
{
	struct ipms_canfd_priv *priv = (void *)can_priv;
	struct canfd_frame *cf = (void *)data;
	int ret = 0;
	unsigned char rx_status = 0, control = 0;

	control = can_ioread8(priv->reg_base + CANFD_RBUF_CTL_OFFSET);
	rx_status = can_ioread8(priv->reg_base + CANFD_RCTRL_OFFSET);

	/* clear receive interrupt and deal with all the received frames */

	if ((rx_status & CAN_FD_RSTAT_NOT_EMPTY_MASK)) {
		if (control & CAN_FD_SET_EDL_MASK)
			ret = canfd_rx(priv, cf);
		else
			ret = can_rx(priv, data);
			//(work_done += canfd_rx(priv, cf)) : (work_done += can_rx(priv, cf));
	} else
		ret = -1;

	//napi_complete(napi);
	canfd_reigister_set_bit(priv, CANFD_RTIE_OFFSET, CAN_FD_SET_RIE_MASK);
	return ret;
}

static void canfd_rxfull_interrupt(struct ipms_canfd_priv *priv, unsigned char isr)
{
	if (isr & CAN_FD_SET_RAFIF_MASK)
		canfd_reigister_set_bit(priv, CANFD_RTIF_OFFSET, CAN_FD_SET_RAFIF_MASK);

	if (isr & (CAN_FD_SET_RAFIF_MASK | CAN_FD_SET_RFIF_MASK))
		canfd_reigister_set_bit(priv, CANFD_RTIF_OFFSET,
					(CAN_FD_SET_RAFIF_MASK | CAN_FD_SET_RFIF_MASK));
}

static int set_canfd_xmit_mode(struct ipms_canfd_priv *priv)
{
	switch (priv->tx_mode) {
	case XMIT_FULL:
		canfd_reigister_set_bit(priv, CANFD_TCTRL_OFFSET, CAN_FD_SET_FULLCAN_MASK);
		break;
	case XMIT_SEP_FIFO:
		canfd_reigister_off_bit(priv, CANFD_TCTRL_OFFSET, CAN_FD_OFF_FULLCAN_MASK);
		canfd_reigister_set_bit(priv, CANFD_TCTRL_OFFSET, CAN_FD_SET_FIFO_MASK);
		canfd_reigister_off_bit(priv, CANFD_TCMD_OFFSET, CAN_FD_SET_TBSEL_MASK);
		break;
	case XMIT_SEP_PRIO:
		canfd_reigister_off_bit(priv, CANFD_TCTRL_OFFSET, CAN_FD_OFF_FULLCAN_MASK);
		canfd_reigister_off_bit(priv, CANFD_TCTRL_OFFSET, CAN_FD_OFF_FIFO_MASK);
		canfd_reigister_off_bit(priv, CANFD_TCMD_OFFSET, CAN_FD_SET_TBSEL_MASK);
		break;
	case XMIT_PTB_MODE:
		canfd_reigister_off_bit(priv, CANFD_TCMD_OFFSET, CAN_FD_OFF_TBSEL_MASK);
		break;
	default:
		break;
	}
	return 0;
}

static int canfd_driver_start_xmit(const void *data, void *ipms_priv, int canfd)
{
	struct ipms_canfd_priv *priv = (void *)ipms_priv;
	struct canfd_frame *cf = (void *)data;
	uint32_t ttsen, id, ctl, addr_off;
	int i;

	//if (can_dropped_invalid_skb(ndev, skb))
		//return NETDEV_TX_OK;

	switch (priv->tx_mode) {
	case XMIT_PTB_MODE:
		set_canfd_xmit_mode(priv);
		canfd_reigister_off_bit(priv, CANFD_TCMD_OFFSET, CAN_FD_OFF_STBY_MASK);

		if (cf->can_id & CAN_EFF_FLAG) {
			id = (cf->can_id & CAN_EFF_MASK);
			ttsen = 0 << TTSEN_8_32_SHIFT;
			id |= ttsen;
		} else {
			id = (cf->can_id & CAN_SFF_MASK);
			ttsen = 0 << TTSEN_8_32_SHIFT;
			id |= ttsen;
		}

		ctl = can_fd_len2dlc(cf->len);

		/* transmit can fd frame */
		if (priv->can_cfg->is_canfd == IPMS_CAN_TYPE_CANFD) {
			if (canfd) {
				if (cf->can_id & CAN_EFF_FLAG)
					ctl |= CAN_FD_SET_IDE_MASK;
				else
					ctl &= CAN_FD_OFF_IDE_MASK;

				if (cf->flags & CANFD_BRS)
					ctl |= CAN_FD_SET_BRS_MASK;

				ctl |= CAN_FD_SET_EDL_MASK;

				addr_off = CANFD_TBUF_DATA_OFFSET;

				for (i = 0; i < cf->len; i += 4) {
					priv->write_reg(priv, addr_off,
							*((unsigned int *)(cf->data + i)));
					addr_off += 4;
				}
			} else {
				ctl &= CAN_FD_OFF_EDL_MASK;
				ctl &= CAN_FD_OFF_BRS_MASK;

				if (cf->can_id & CAN_EFF_FLAG)
					ctl |= CAN_FD_SET_IDE_MASK;
				else
					ctl &= CAN_FD_OFF_IDE_MASK;

				if (cf->can_id & CAN_RTR_FLAG) {
					ctl |= CAN_FD_SET_RTR_MASK;
					priv->write_reg(priv,
						CANFD_TBUF_ID_OFFSET, id);
					priv->write_reg(priv,
						CANFD_TBUF_CTL_OFFSET, ctl);
				} else {
					ctl &= CAN_FD_OFF_RTR_MASK;
					addr_off = CANFD_TBUF_DATA_OFFSET;
					priv->write_reg(priv, addr_off,
							*((unsigned int *)(cf->data + 0)));
					priv->write_reg(priv, addr_off + 4,
							*((unsigned int *)(cf->data + 4)));
				}
			}
			priv->write_reg(priv, CANFD_TBUF_ID_OFFSET, id);
			priv->write_reg(priv, CANFD_TBUF_CTL_OFFSET, ctl);
			addr_off = CANFD_TBUF_DATA_OFFSET;
		} else {
			ctl &= CAN_FD_OFF_EDL_MASK;
			ctl &= CAN_FD_OFF_BRS_MASK;

			if (cf->can_id & CAN_EFF_FLAG)
				ctl |= CAN_FD_SET_IDE_MASK;
			else
				ctl &= CAN_FD_OFF_IDE_MASK;

			if (cf->can_id & CAN_RTR_FLAG) {
				ctl |= CAN_FD_SET_RTR_MASK;
				priv->write_reg(priv, CANFD_TBUF_ID_OFFSET, id);
				priv->write_reg(priv, CANFD_TBUF_CTL_OFFSET, ctl);
			} else {
				ctl &= CAN_FD_OFF_RTR_MASK;
				priv->write_reg(priv, CANFD_TBUF_ID_OFFSET, id);
				priv->write_reg(priv, CANFD_TBUF_CTL_OFFSET, ctl);
				addr_off = CANFD_TBUF_DATA_OFFSET;
				priv->write_reg(priv, addr_off,
						*((unsigned int *)(cf->data + 0)));
				priv->write_reg(priv, addr_off + 4,
						*((unsigned int *)(cf->data + 4)));
			}
		}
		canfd_reigister_set_bit(priv, CANFD_TCMD_OFFSET, CAN_FD_SET_TPE_MASK);
		//stats->tx_bytes += cf->len;
		break;
	default:
		break;
	}

	return 0;
}

static int set_reset_mode(struct ipms_canfd_priv *priv)
{
	unsigned char ret;

	ret = can_ioread8(priv->reg_base + CANFD_CFG_STAT_OFFSET);
	ret |= CAN_FD_SET_RST_MASK;
	can_iowrite8(ret, priv->reg_base + CANFD_CFG_STAT_OFFSET);

	return 0;
}

#if 0
static void canfd_driver_stop(struct net_device *ndev)
{
	struct ipms_canfd_priv *priv = netdev_priv(ndev);
	int ret;

	ret = set_reset_mode(ndev);
	if (ret)
		netdev_err(ndev, "Mode Resetting Failed!\n");

	priv->can.state = CAN_STATE_STOPPED;
}

static int canfd_driver_close(struct net_device *ndev)
{
	struct ipms_canfd_priv *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);
	napi_disable(&priv->napi);
	canfd_driver_stop(ndev);

	free_irq(ndev->irq, ndev);
	close_candev(ndev);

	pm_runtime_put(priv->dev);

	return 0;
}
#endif

static enum can_state get_of_chip_status(struct ipms_canfd_priv *priv)
{
	unsigned char can_stat, eir;

	can_stat = can_ioread8(priv->reg_base + CANFD_CFG_STAT_OFFSET);
	eir = can_ioread8(priv->reg_base + CANFD_ERRINT_OFFSET);

	if (can_stat & CAN_FD_SET_BUSOFF_MASK)
		return CAN_STATE_BUS_OFF;

	if ((eir & CAN_FD_SET_EPASS_MASK) && ~(can_stat & CAN_FD_SET_BUSOFF_MASK))
		return CAN_STATE_ERROR_PASSIVE;

	if (eir & CAN_FD_SET_EWARN_MASK && ~(eir & CAN_FD_SET_EPASS_MASK))
		return CAN_STATE_ERROR_WARNING;

	if (~(eir & CAN_FD_SET_EPASS_MASK))
		return CAN_STATE_ERROR_ACTIVE;

	return CAN_STATE_ERROR_ACTIVE;
}

static void canfd_error_interrupt(struct ipms_canfd_priv *priv, unsigned char isr, unsigned char eir)
{
	//struct net_device_stats *stats = &ndev->stats;
	//struct can_frame *cf;
	unsigned char koer, recnt = 0, tecnt = 0, can_stat = 0;

	koer = can_ioread8(priv->reg_base + CANFD_EALCAP_OFFSET) & CAN_FD_SET_KOER_MASK;
	recnt = can_ioread8(priv->reg_base + CANFD_RECNT_OFFSET);
	tecnt = can_ioread8(priv->reg_base + CANFD_TECNT_OFFSET);

	/*Read can status*/
	can_stat = can_ioread8(priv->reg_base + CANFD_CFG_STAT_OFFSET);

	/* Bus off --->active error mode */
	if ((isr & CAN_FD_SET_EIF_MASK) && priv->can.state == CAN_STATE_BUS_OFF)
		priv->can.state = get_of_chip_status(priv);

	/* State selection */
	if (can_stat & CAN_FD_SET_BUSOFF_MASK) {
		priv->can.state = get_of_chip_status(priv);
		canfd_reigister_set_bit(priv, CANFD_CFG_STAT_OFFSET, CAN_FD_SET_BUSOFF_MASK);
#if 0
		priv->can.can_stats.bus_off++;
		can_bus_off(ndev);
		if (skb)
			cf->can_id |= CAN_ERR_BUSOFF;
#endif
	} else if ((eir & CAN_FD_SET_EPASS_MASK) && ~(can_stat & CAN_FD_SET_BUSOFF_MASK)) {
		priv->can.state = get_of_chip_status(priv);
#if 0
		priv->can.can_stats.error_passive++;
		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] |= (recnt > 127) ? CAN_ERR_CRTL_RX_PASSIVE : 0;
			cf->data[1] |= (tecnt > 127) ? CAN_ERR_CRTL_TX_PASSIVE : 0;
			cf->data[6] = tecnt;
			cf->data[7] = recnt;
		}
#endif
	} else if (eir & CAN_FD_SET_EWARN_MASK && ~(eir & CAN_FD_SET_EPASS_MASK)) {
		priv->can.state = get_of_chip_status(priv);
#if 0
		priv->can.can_stats.error_warning++;
		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] |= (recnt > 95) ? CAN_ERR_CRTL_RX_WARNING : 0;
			cf->data[1] |= (tecnt > 95) ? CAN_ERR_CRTL_TX_WARNING : 0;
			cf->data[6] = tecnt;
			cf->data[7] = recnt;
		}
#endif
	}

	/* Check for in protocol defined error interrupt */
	if (eir & CAN_FD_SET_BEIF_MASK) {
#if 0
		if (skb)
			cf->can_id |= CAN_ERR_BUSERROR | CAN_ERR_PROT;
#endif
		/* bit error interrupt */
		if (koer == CAN_FD_SET_BIT_ERROR_MASK) {
#if 0
			stats->tx_errors++;
			if (skb) {
				cf->can_id |= CAN_ERR_PROT;
				cf->data[2] = CAN_ERR_PROT_BIT;
			}
#endif
		}
		/* format error interrupt */
		if (koer == CAN_FD_SET_FORM_ERROR_MASK) {
#if 0
			stats->rx_errors++;
			if (skb) {
				cf->can_id |= CAN_ERR_PROT;
				cf->data[2] = CAN_ERR_PROT_FORM;
			}
#endif
		}
		/* stuffing error interrupt */
		if (koer == CAN_FD_SET_STUFF_ERROR_MASK) {
#if 0
			stats->rx_errors++;
			if (skb) {
				cf->can_id |= CAN_ERR_PROT;
				cf->data[3] = CAN_ERR_PROT_STUFF;
			}
#endif
		}
		/* ack error interrupt */
		if (koer == CAN_FD_SET_ACK_ERROR_MASK) {
#if 0
			stats->tx_errors++;
			if (skb) {
				cf->can_id |= CAN_ERR_PROT;
				cf->data[2] = CAN_ERR_PROT_LOC_ACK;
			}
#endif
		}
		/* crc error interrupt */
		if (koer == CAN_FD_SET_CRC_ERROR_MASK) {
#if 0
			stats->rx_errors++;
			if (skb) {
				cf->can_id |= CAN_ERR_PROT;
				cf->data[2] = CAN_ERR_PROT_LOC_CRC_SEQ;
			}
#endif
		}
		//priv->can.can_stats.bus_error++;
	}
#if 0
	if (skb) {
		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;
		netif_rx(skb);
	}
#endif
	//CAN_DBG("Recnt is 0x%02x", can_ioread8(priv->reg_base + CANFD_RECNT_OFFSET));
	//CAN_DBG("Tecnt is 0x%02x", can_ioread8(priv->reg_base + CANFD_TECNT_OFFSET));
}

static unsigned int canfd_interrupt(void *ipms_priv)
{
	unsigned char isr, eir;
	unsigned char isr_handled = 0, eir_handled = 0;
	struct ipms_canfd_priv *priv = (void *)ipms_priv;
	unsigned int ret = 0;

	/* read the value of interrupt status register */
	isr = can_ioread8(priv->reg_base + CANFD_RTIF_OFFSET);

	/* read the value of error interrupt register */
	eir = can_ioread8(priv->reg_base + CANFD_ERRINT_OFFSET);

	/* Check for Tx interrupt and Processing it */
	if (isr & (CAN_FD_SET_TPIF_MASK | CAN_FD_SET_TSIF_MASK)) {
		canfd_tx_interrupt(priv, isr);
		isr_handled |= (CAN_FD_SET_TPIF_MASK | CAN_FD_SET_TSIF_MASK);
		ret |= CAN_TX_DONE;
	}
	if (isr & (CAN_FD_SET_RAFIF_MASK | CAN_FD_SET_RFIF_MASK)) {
		canfd_rxfull_interrupt(priv, isr);
		isr_handled |= (CAN_FD_SET_RAFIF_MASK | CAN_FD_SET_RFIF_MASK);
		ret |= CAN_RX_OVERFLOW;
	}
	/* Check Rx interrupt and Processing the receive interrupt routine */
	if (isr & CAN_FD_SET_RIF_MASK) {
		canfd_reigister_off_bit(priv, CANFD_RTIE_OFFSET, CAN_FD_OFF_RIE_MASK);
		canfd_reigister_set_bit(priv, CANFD_RTIF_OFFSET, CAN_FD_SET_RIF_MASK);
		//rx irq
		//napi_schedule(&priv->napi);
		isr_handled |= CAN_FD_SET_RIF_MASK;
		ret |= CAN_RX_IND;
	}
	if ((isr & CAN_FD_SET_EIF_MASK) | (eir & (CAN_FD_SET_EPIF_MASK | CAN_FD_SET_BEIF_MASK))) {
		/* reset EPIF and BEIF. Reset EIF */
		canfd_reigister_set_bit(priv, CANFD_ERRINT_OFFSET,
					eir & (CAN_FD_SET_EPIF_MASK | CAN_FD_SET_BEIF_MASK));
		canfd_reigister_set_bit(priv, CANFD_RTIF_OFFSET,
					isr & CAN_FD_SET_EIF_MASK);

		canfd_error_interrupt(priv, isr, eir);

		isr_handled |= CAN_FD_SET_EIF_MASK;
		eir_handled |= (CAN_FD_SET_EPIF_MASK | CAN_FD_SET_BEIF_MASK);
		ret |= CAN_ERROR;
	}

	if ((isr_handled == 0) && (eir_handled == 0))
		hal_printf("Unhandled interrupt!\n");

	return ret;
}

static int canfd_chip_start(struct ipms_canfd_priv *priv)
{
	int err;
	unsigned char ret;

	set_reset_mode(priv);

	err = canfd_device_driver_bittime_configuration(priv);
	if (err) {
		hal_printf("%s Bittime Setting Failed!\n", priv->can_cfg->name);
		return err;
	}

	/* Set Almost Full Warning Limit */
	canfd_reigister_set_bit(priv, CANFD_LIMIT_OFFSET, CAN_FD_SET_AFWL_MASK);

	/* Programmable Error Warning Limit = (EWL+1)*8. Set EWL=11->Error Warning=96 */
	canfd_reigister_set_bit(priv, CANFD_LIMIT_OFFSET, CAN_FD_SET_EWL_MASK);

	/* Interrupts enable */
	/* todo set by control */
	can_iowrite8(CAN_FD_INTR_ALL_MASK, priv->reg_base + CANFD_RTIE_OFFSET);

	/* Error Interrupts enable(Error Passive and Bus Error) */
	canfd_reigister_set_bit(priv, CANFD_ERRINT_OFFSET, CAN_FD_SET_EPIE_MASK);

	ret = can_ioread8(priv->reg_base + CANFD_CFG_STAT_OFFSET);

	/* Check whether it is loopback mode or normal mode */
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		ret |= CAN_FD_LBMIMOD_MASK;
	} else {
		ret &= ~CAN_FD_LBMEMOD_MASK;
		ret &= ~CAN_FD_LBMIMOD_MASK;
	}

	can_iowrite8(ret, priv->reg_base + CANFD_CFG_STAT_OFFSET);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	return 0;
}

#if 0
static int  canfd_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	int ret;

	switch (mode) {
	case CAN_MODE_START:
		ret = canfd_chip_start(ndev);
		if (ret) {
			netdev_err(ndev, "Could Not Start CAN device !!\n");
			return ret;
		}
		netif_wake_queue(ndev);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}
#endif

static int ipms_canfd_init(struct hal_canfd *ipms, int ctrl_mode)
{
	struct ipms_canfd_priv *priv;
	int ret;
	int i;

	if (!ipms->priv) {
		priv = hal_malloc(sizeof(*priv));
		if (!priv)
			return -1;

		priv->can.bittiming_const = &canfd_bittiming_const;
		priv->can.canfd_data_bittiming_const = &canfd_data_bittiming_const;
		//priv->can.do_set_mode = canfd_do_set_mode;
		/* in user space the execution mode can be chosen */
		if (ipms->cfg.is_canfd)
			priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_FD;
		else
			priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK;

		priv->reg_base = ipms->base;
		priv->write_reg = canfd_write_reg_le;
		priv->read_reg = canfd_read_reg_le;

		priv->can_cfg = &ipms->cfg;
		ipms->priv = (void *)priv;
	}
	priv->tx_mode = XMIT_PTB_MODE;
	priv->can.ctrlmode = ctrl_mode;

	for (i = 0; i < sizeof(bt_table) /sizeof(bt_table[0]); i++) {
	     if (ipms->baudrate == bt_table[i].baud_rate) {
		memcpy(&priv->can.bittiming, &bt_table[i].bittiming, sizeof(struct can_bittiming));
		memcpy(&priv->can.data_bittiming, &dbt_table[i].bittiming, sizeof(struct can_bittiming));
	     }
	}
	//set_reset_mode(priv);

	//ret = request_irq(ndev->irq, canfd_interrupt, IRQF_SHARED, ndev->name, ndev);

	ret = canfd_chip_start(priv);
	if (ret)
		hal_free(priv);

	return 0;
}


struct hal_can_ops ipms_ops = {
	.start_xmit = canfd_driver_start_xmit,
	.rx_poll = canfd_rx_poll,
	.can_isr = canfd_interrupt,
	.canfd_init = ipms_canfd_init, 
};

void ipms_ops_init(struct hal_canfd *can)
{
	can->can_ops = &ipms_ops;
}

