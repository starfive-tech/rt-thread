
#include <rtthread.h>
#include <riscv_io.h>
#include <io.h>

#include "board.h"
#include "hal_gmac.h"

#include "../../pcie/pci.h"

//#define DEBUG_RTL8169
//#define DEBUG_RTL8169_TX
//#define DEBUG_RTL8169_RX

#define printf hal_printf

#define drv_version "v1.5"
#define drv_date "01-17-2004"

static unsigned long ioaddr;

#define u8	uint8_t
#define u16	uint16_t
#define u32	uint32_t

#define le32_to_cpu(a)	(a)
#define cpu_to_le32(a)	(a)

/* Condensed operations for readability. */
//#define currticks()	get_timer(0)

/* media options */
#define MAX_UNITS 8
static int media[MAX_UNITS] = { -1, -1, -1, -1, -1, -1, -1, -1 };

/* MAC address length*/
#define MAC_ADDR_LEN	6

#define ETH_ZLEN	60

/* max supported gigabit ethernet frame size -- must be at least (dev->mtu+14+4).*/
#define MAX_ETH_FRAME_SIZE	1536

#define TX_FIFO_THRESH 256	/* In bytes */

#define RX_FIFO_THRESH	7	/* 7 means NO threshold, Rx buffer level before first PCI xfer.	 */
#define RX_DMA_BURST	6	/* Maximum PCI burst, '6' is 1024 */
#define TX_DMA_BURST	6	/* Maximum PCI burst, '6' is 1024 */
#define EarlyTxThld	0x3F	/* 0x3F means NO early transmit */
#define RxPacketMaxSize 0x0800	/* Maximum size supported is 16K-1 */
#define InterFrameGap	0x03	/* 3 means InterFrameGap = the shortest one */

#define NUM_TX_DESC	256	/* Number of Tx descriptor registers */
#define NUM_RX_DESC     256
#define RX_BUF_SIZE	1536	/* Rx Buffer size */
#define RX_BUF_LEN	8192

#define DESCRIPTOR_WORDS   4
#define DESCRIPTOR_SIZE    (DESCRIPTOR_WORDS * 4)
#define RTL8169_DESC_SIZE DESCRIPTOR_SIZE
/* We assume ARCH_DMA_MINALIGN >= 16; 16 is the EQOS HW minimum */
#define DESCRIPTOR_ALIGN   64
#define DESCRIPTORS_NUM    (NUM_TX_DESC + NUM_RX_DESC)
#define DESCRIPTORS_SIZE    ALIGN(DESCRIPTORS_NUM * \
                      DESCRIPTOR_SIZE, DESCRIPTOR_ALIGN)
#define BUFFER_ALIGN    DESCRIPTOR_ALIGN
#define RTL8169_ALIGN	256
#define MAX_PACKET_SIZE    ALIGN(RX_BUF_SIZE, DESCRIPTOR_ALIGN)
//#define RX_BUFFER_SIZE (MAX_PACKET_SIZE)

#define RTL_MIN_IO_SIZE 0x80
#define TX_TIMEOUT  (6*1000)

/* write/read MMIO register. Notice: {read,write}[wl] do the necessary swapping */
#define RTL_W8(reg, val8)	sys_writeb((val8), ioaddr + (reg))
#define RTL_W16(reg, val16)	sys_writew((val16), ioaddr + (reg))
#define RTL_W32(reg, val32)	sys_writel((val32), ioaddr + (reg))
#define RTL_R8(reg)		sys_readb(ioaddr + (reg))
#define RTL_R16(reg)		sys_readw(ioaddr + (reg))
#define RTL_R32(reg)		sys_readl(ioaddr + (reg))

#define bus_to_phys(a)	(a)
#define phys_to_bus(a)	(a)

enum RTL8169_registers {
	MAC0 = 0,		/* Ethernet hardware address. */
	MAR0 = 8,		/* Multicast filter. */
	TxDescStartAddrLow = 0x20,
	TxDescStartAddrHigh = 0x24,
	TxHDescStartAddrLow = 0x28,
	TxHDescStartAddrHigh = 0x2c,
	FLASH = 0x30,
	ERSR = 0x36,
	ChipCmd = 0x37,
	TxPoll = 0x38,
	IntrMask = 0x3C,
	IntrStatus = 0x3E,
	TxConfig = 0x40,
	RxConfig = 0x44,
	RxMissed = 0x4C,
	Cfg9346 = 0x50,
	Config0 = 0x51,
	Config1 = 0x52,
	Config2 = 0x53,
	Config3 = 0x54,
	Config4 = 0x55,
	Config5 = 0x56,
	MultiIntr = 0x5C,
	PHYAR = 0x60,
	TBICSR = 0x64,
	TBI_ANAR = 0x68,
	TBI_LPAR = 0x6A,
	PHYstatus = 0x6C,
	RxMaxSize = 0xDA,
	CPlusCmd = 0xE0,
	RxDescStartAddrLow = 0xE4,
	RxDescStartAddrHigh = 0xE8,
	EarlyTxThres = 0xEC,
	FuncEvent = 0xF0,
	FuncEventMask = 0xF4,
	FuncPresetState = 0xF8,
	FuncForceEvent = 0xFC,
};

enum RTL8169_register_content {
	/*InterruptStatusBits */
	SYSErr = 0x8000,
	PCSTimeout = 0x4000,
	SWInt = 0x0100,
	TxDescUnavail = 0x80,
	RxFIFOOver = 0x40,
	//RxUnderrun = 0x20,
	LinkChg		= 0x0020,
	RxOverflow = 0x10,
	TxErr = 0x08,
	TxOK = 0x04,
	RxErr = 0x02,
	RxOK = 0x01,

	/*RxStatusDesc */
	RxRES = 0x00200000,
	RxCRC = 0x00080000,
	RxRUNT = 0x00100000,
	RxRWT = 0x00400000,

	/*ChipCmdBits */
	CmdReset = 0x10,
	CmdRxEnb = 0x08,
	CmdTxEnb = 0x04,
	RxBufEmpty = 0x01,

	/*Cfg9346Bits */
	Cfg9346_Lock = 0x00,
	Cfg9346_Unlock = 0xC0,

	/*rx_mode_bits */
	AcceptErr = 0x20,
	AcceptRunt = 0x10,
	AcceptBroadcast = 0x08,
	AcceptMulticast = 0x04,
	AcceptMyPhys = 0x02,
	AcceptAllPhys = 0x01,

	/*RxConfigBits */
	RxCfgFIFOShift = 13,
	RxCfgDMAShift = 8,

	/*TxConfigBits */
	TxInterFrameGapShift = 24,
	TxDMAShift = 8,		/* DMA burst value (0-7) is shift this many bits */

	/*rtl8169_PHYstatus */
	TBI_Enable = 0x80,
	TxFlowCtrl = 0x40,
	RxFlowCtrl = 0x20,
	_1000bpsF = 0x10,
	_100bps = 0x08,
	_10bps = 0x04,
	LinkStatus = 0x02,
	FullDup = 0x01,

	/*GIGABIT_PHY_registers */
	PHY_CTRL_REG = 0,
	PHY_STAT_REG = 1,
	PHY_AUTO_NEGO_REG = 4,
	PHY_1000_CTRL_REG = 9,

	/*GIGABIT_PHY_REG_BIT */
	PHY_Restart_Auto_Nego = 0x0200,
	PHY_Enable_Auto_Nego = 0x1000,

	/* PHY_STAT_REG = 1; */
	PHY_Auto_Nego_Comp = 0x0020,

	/* PHY_AUTO_NEGO_REG = 4; */
	PHY_Cap_10_Half = 0x0020,
	PHY_Cap_10_Full = 0x0040,
	PHY_Cap_100_Half = 0x0080,
	PHY_Cap_100_Full = 0x0100,

	/* PHY_1000_CTRL_REG = 9; */
	PHY_Cap_1000_Full = 0x0200,

	PHY_Cap_Null = 0x0,

	/*_MediaType*/
	_10_Half = 0x01,
	_10_Full = 0x02,
	_100_Half = 0x04,
	_100_Full = 0x08,
	_1000_Full = 0x10,

	/*_TBICSRBit*/
	TBILinkOK = 0x02000000,

	/* FuncEvent/Misc */
	RxDv_Gated_En = 0x80000,
};

static struct {
	const char *name;
	u8 version;		/* depend on RTL8169 docs */
	u32 RxConfigMask;	/* should clear the bits supported by this chip */
} rtl_chip_info[] = {
	{"RTL-8169", 0x00, 0xff7e1880,},
	{"RTL-8169", 0x04, 0xff7e1880,},
	{"RTL-8169", 0x00, 0xff7e1880,},
	{"RTL-8169s/8110s",	0x02, 0xff7e1880,},
	{"RTL-8169s/8110s",	0x04, 0xff7e1880,},
	{"RTL-8169sb/8110sb",	0x10, 0xff7e1880,},
	{"RTL-8169sc/8110sc",	0x18, 0xff7e1880,},
	{"RTL-8168b/8111sb",	0x30, 0xff7e1880,},
	{"RTL-8168b/8111sb",	0x38, 0xff7e1880,},
	{"RTL-8168c/8111c",	0x3c, 0xff7e1880,},
	{"RTL-8168d/8111d",	0x28, 0xff7e1880,},
	{"RTL-8168evl/8111evl",	0x2e, 0xff7e1880,},
	{"RTL-8168/8111g",	0x4c, 0xff7e1880,},
	{"RTL-8101e",		0x34, 0xff7e1880,},
	{"RTL-8100e",		0x32, 0xff7e1880,},
	{"RTL-8168h/8111h",	0x54, 0xff7e1880,},
};

enum _DescStatusBit {
	OWNbit = 0x80000000,
	EORbit = 0x40000000,
	FSbit = 0x20000000,
	LSbit = 0x10000000,
};

struct TxDesc {
	u32 status;
	u32 vlan_tag;
	u32 buf_addr;
	u32 buf_Haddr;
};

struct RxDesc {
	u32 status;
	u32 vlan_tag;
	u32 buf_addr;
	u32 buf_Haddr;
};

//static unsigned char rxdata[RX_BUF_LEN];

struct rtl8169_private {
	struct udevice *udev;
	unsigned long iobase;
	void *mmio_addr;	/* memory map physical address */
	int chipset;
	//unsigned long cur_rx;	/* Index into the Rx descriptor buffer of next Rx pkt. */
	unsigned short busyrxdesc;
	unsigned char rxbusy;
	unsigned char rxnext;
	unsigned int cur_tx;	/* Index into the Tx descriptor buffer of next Rx pkt. */
	struct TxDesc *TxDescArray;	/* Index of 256-alignment Tx Descriptor buffer */
	struct RxDesc *RxDescArray;	/* Index of 256-alignment Rx Descriptor buffer */
	unsigned char *RxBufferRing[NUM_RX_DESC];	/* Index of Rx Buffer array */
	unsigned char *Tx_skbuff[NUM_TX_DESC];
	unsigned char *rx_dma_buf_noalign[NUM_RX_DESC];	/* Index of Rx Buffer array */
	unsigned char *tx_dma_buf_noalign[NUM_TX_DESC];
	unsigned char *descs_noalign;
        unsigned char *descs;
	unsigned short msi_irq;
	unsigned short irq_mask;
};

static const unsigned int rtl8169_rx_config =
    (RX_FIFO_THRESH << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift);

void mdio_write(int RegAddr, int value)
{
	int i;

	RTL_W32(PHYAR, 0x80000000 | (RegAddr & 0xFF) << 16 | value);
	sys_udelay(1000);

	for (i = 2000; i > 0; i--) {
		/* Check if the RTL8169 has completed writing to the specified MII register */
		if (!(RTL_R32(PHYAR) & 0x80000000)) {
			break;
		} else {
			sys_udelay(100);
		}
	}
}

int mdio_read(int RegAddr)
{
	int i, value = -1;

	RTL_W32(PHYAR, 0x0 | (RegAddr & 0xFF) << 16);
	sys_udelay(1000);

	for (i = 2000; i > 0; i--) {
		/* Check if the RTL8169 has completed retrieving data from the specified MII register */
		if (RTL_R32(PHYAR) & 0x80000000) {
			value = (int) (RTL_R32(PHYAR) & 0xFFFF);
			break;
		} else {
			sys_udelay(100);
		}
	}
	return value;
}

static int rtl8169_init_board(struct rtl8169_private *tpc, const char *name)
{
	int i;
	u32 tmp;

#ifdef DEBUG_RTL8169
	printf ("%s\n", __FUNCTION__);
#endif
	//ioaddr = tpc->iobase;

	/* Soft reset the chip. */
	RTL_W8(ChipCmd, CmdReset);

	/* Check that the chip has finished the reset. */
	for (i = 1000; i > 0; i--)
		if ((RTL_R8(ChipCmd) & CmdReset) == 0)
			break;
		else
			sys_udelay(10);

	/* identify chip attached to board */
	tmp = RTL_R32(TxConfig);
	tmp = ((tmp & 0x7c000000) + ((tmp & 0x00800000) << 2)) >> 24;

	for (i = sizeof(rtl_chip_info) / sizeof(rtl_chip_info[0]) - 1; i >= 0; i--){
		if (tmp == rtl_chip_info[i].version) {
			tpc->chipset = i;
			goto match;
		}
	}

	/* if unknown chip, assume array element #0, original RTL-8169 in this case */
	printf("PCI device %s: unknown chip version, assuming RTL-8169\n",
	       name);
	printf("PCI device: TxConfig = 0x%lX\n", (unsigned long) RTL_R32(TxConfig));
	tpc->chipset = 0;

match:
	return 0;
}

/*
 * Cache maintenance functions. These are simple wrappers around the more
 * general purpose flush_cache() and invalidate_dcache_range() functions.
 */

static inline void rtl_inval_rx_desc(struct RxDesc *desc)
{
#if 0
	unsigned long start = (unsigned long)desc & ~(ARCH_DMA_MINALIGN - 1);
	unsigned long end = ALIGN(start + sizeof(*desc), ARCH_DMA_MINALIGN);

	invalidate_dcache_range(start, end);
#endif
}

static inline void rtl_flush_rx_desc(struct RxDesc *desc)
{
#if 0
	flush_cache((unsigned long)desc, sizeof(*desc));
#endif
}

static inline void rtl_inval_tx_desc(struct TxDesc *desc)
{
#if 0
	unsigned long start = (unsigned long)desc & ~(ARCH_DMA_MINALIGN - 1);
	unsigned long end = ALIGN(start + sizeof(*desc), ARCH_DMA_MINALIGN);

	invalidate_dcache_range(start, end);
#endif
}

static inline void rtl_flush_tx_desc(struct TxDesc *desc)
{
#if 0
	flush_cache((unsigned long)desc, sizeof(*desc));
#endif
}

static inline void rtl_inval_buffer(void *buf, size_t size)
{
#if 0
	unsigned long start = (unsigned long)buf & ~(ARCH_DMA_MINALIGN - 1);
	unsigned long end = ALIGN(start + size, ARCH_DMA_MINALIGN);

	invalidate_dcache_range(start, end);
#endif
}

static inline void  rtl_flush_buffer(void *buf, size_t size)
{
#if 0
	flush_cache((unsigned long)buf, size);
#endif
}

static int rtl8169_free_pkt(void *priv, void *packet)
{
	struct rtl8169_private *tpc = (void *)priv;
	int cur_rx;
	void *packet_expected;

	cur_rx = tpc->rxnext;
	packet_expected = tpc->RxBufferRing[cur_rx];

	if (packet != packet_expected) {
	    hal_printf("Unexpected packet (expected %p)\n",
		  packet_expected);
	    return -1;
	}

	tpc->RxDescArray[cur_rx].buf_addr =
		(u32)(unsigned long)tpc->RxBufferRing[cur_rx];

	if (cur_rx == NUM_RX_DESC - 1)
		tpc->RxDescArray[cur_rx].status =
			cpu_to_le32((OWNbit | EORbit) + RX_BUF_SIZE);
	else
		tpc->RxDescArray[cur_rx].status =
			cpu_to_le32(OWNbit + RX_BUF_SIZE);


	tpc->rxnext++;
	tpc->rxnext &= (NUM_RX_DESC - 1);
	tpc->busyrxdesc++;

	return cur_rx;
}

static int rtl8169_eth_recv(void *priv, void **packetp)
{
	/* return true if there's an ethernet packet ready to read */
	/* nic->packet should contain data on return */
	/* nic->packetlen should contain length of data */
	struct rtl8169_private *tpc = (void *)priv;
	int cur_rx;
	int length = 0;

#ifdef DEBUG_RTL8169_RX
	printf ("%s\n", __FUNCTION__);
#endif

	cur_rx = tpc->rxbusy;

	//rtl_inval_rx_desc(&tpc->RxDescArray[cur_rx]);

	if ((le32_to_cpu(tpc->RxDescArray[cur_rx].status) & OWNbit) == 0) {
		if (!(le32_to_cpu(tpc->RxDescArray[cur_rx].status) & RxRES)) {
			length = (int) (le32_to_cpu(tpc->RxDescArray[cur_rx].
						status) & 0x00001FFF) - 4;

			//printf ("recv packet len %d\n", length);
			if (!length) {
				hal_printf("RX packet len 0 %d\n", cur_rx);
				tpc->rxnext++;
				return -1;
			} else {
				//rtl_inval_buffer(tpc->RxBufferRing[cur_rx], length);
				tpc->RxDescArray[cur_rx].buf_addr = 0;

				//printf("rtl8169: recv common @0x%x 0x%x\n", (unsigned long)tpc->RxBufferRing[cur_rx],
							//tpc->RxDescArray[cur_rx].buf_addr);
				//rtl_flush_rx_desc(&tpc->RxDescArray[cur_rx]);
				*packetp = tpc->RxBufferRing[cur_rx];
				tpc->busyrxdesc--;
			}
		} else {
			hal_printf("Error Rx");
			tpc->rxnext++;
			length = -1;
		}
		//cur_rx = (cur_rx + 1) % NUM_RX_DESC;
		tpc->rxbusy++;
		tpc->rxbusy &= (NUM_RX_DESC - 1);
		return length;
	} else {
		unsigned short sts = RTL_R8(IntrStatus);
		//printf ("sts %x\n", sts);
		RTL_W8(IntrStatus, sts & ~(TxErr | RxErr | SYSErr));
		sys_udelay(100);	/* wait */
	}

	return (0);		/* initially as this is called to flush the input */
}

static int rtl8169_check_descriptor(void *priv, void **buffer)
{
    struct rtl8169_private *tpc = (void *)priv;
    int entry = tpc->cur_tx;
    struct TxDesc *tx_desc;

    tx_desc = &(tpc->TxDescArray[entry]);

    if (tx_desc->status & OWNbit) {
	hal_printf("current tx discriptor not avail %d", entry);
	return -1;
    }

    *buffer = tpc->Tx_skbuff[entry];
    return 0;
}

static int rtl8169_eth_send(void *priv, int len)
{
	/* send the packet to destination */
	struct rtl8169_private *tpc = (void *)priv;
	u32 to;
	u8 *ptxb;
	int ret, entry;

	entry = tpc->cur_tx;
	/* point to the current txb incase multiple tx_rings are used */
	ptxb = tpc->Tx_skbuff[entry];

#ifdef DEBUG_RTL8169_TX
	u32 *debug = (void *)ptxb;

	printf ("%s\n", __FUNCTION__);
	printf("sending %d bytes\n", len);
	printf("tx debug %08x %08x %08x %08x\n", debug[0], debug[1], debug[2], debug[3]);
#endif

	while (len < ETH_ZLEN)
		ptxb[len++] = '\0';

	//rtl_flush_buffer(ptxb, ALIGN(len, RTL8169_ALIGN));

	tpc->TxDescArray[entry].buf_Haddr = 0;

	tpc->TxDescArray[entry].buf_addr = (u32)(unsigned long)ptxb;
	if (entry != (NUM_TX_DESC - 1)) {
		tpc->TxDescArray[entry].status =
			cpu_to_le32(OWNbit | FSbit | LSbit |
				((len < ETH_ZLEN) ? ETH_ZLEN : len));
	} else {
		tpc->TxDescArray[entry].status =
			cpu_to_le32(OWNbit | EORbit | FSbit | LSbit |
					((len < ETH_ZLEN) ? ETH_ZLEN : len));
	}

	mb();
	//rtl_flush_tx_desc(&tpc->TxDescArray[entry]);
	RTL_W8(TxPoll, 0x40);	/* set polling bit */

	tpc->cur_tx++;
	tpc->cur_tx &= (NUM_RX_DESC - 1);

	return ret;
}

static void rtl8169_set_rx_mode(struct rtl8169_private *tpc)
{
	u32 mc_filter[2];	/* Multicast hash filter */
	int rx_mode;
	u32 tmp = 0;

#ifdef DEBUG_RTL8169
	printf ("%s\n", __FUNCTION__);
#endif

	/* IFF_ALLMULTI */
	/* Too many to filter perfectly -- accept all multicasts. */
	rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
	mc_filter[1] = mc_filter[0] = 0xffffffff;

	tmp = rtl8169_rx_config | rx_mode | (RTL_R32(RxConfig) &
				   rtl_chip_info[tpc->chipset].RxConfigMask);

	RTL_W32(RxConfig, tmp);
	RTL_W32(MAR0 + 0, mc_filter[0]);
	RTL_W32(MAR0 + 4, mc_filter[1]);
}

static void rtl8169_hw_start(struct rtl8169_private *tpc)
{
	u32 i;

#ifdef DEBUG_RTL8169
	printf ("%s\n", __FUNCTION__);
#endif

#if 0
	/* Soft reset the chip. */
	RTL_W8(ChipCmd, CmdReset);

	/* Check that the chip has finished the reset. */
	for (i = 1000; i > 0; i--) {
		if ((RTL_R8(ChipCmd) & CmdReset) == 0)
			break;
		else
			udelay(10);
	}
#endif

	RTL_W8(Cfg9346, Cfg9346_Unlock);

	/* RTL-8169sb/8110sb or previous version */
	if (tpc->chipset <= 5)
		RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);

	RTL_W8(EarlyTxThres, EarlyTxThld);

	/* For gigabit rtl8169 */
	RTL_W16(RxMaxSize, RxPacketMaxSize);

	/* Set Rx Config register */
	i = rtl8169_rx_config | (RTL_R32(RxConfig) &
				 rtl_chip_info[tpc->chipset].RxConfigMask);
	RTL_W32(RxConfig, i);

	/* Set DMA burst size and Interframe Gap Time */
	RTL_W32(TxConfig, (TX_DMA_BURST << TxDMAShift) |
				(InterFrameGap << TxInterFrameGapShift));

	RTL_W32(TxDescStartAddrLow, (u32)(unsigned long)tpc->TxDescArray);
	RTL_W32(TxDescStartAddrHigh, 0);

	RTL_W32(RxDescStartAddrLow, (u32)(unsigned long)tpc->RxDescArray);
	RTL_W32(RxDescStartAddrHigh, 0);

	/* RTL-8169sc/8110sc or later version */
	if (tpc->chipset > 5)
		RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);

	RTL_W8(Cfg9346, Cfg9346_Lock);
	sys_udelay(10);

	RTL_W32(RxMissed, 0);

	rtl8169_set_rx_mode(tpc);

	/* no early-rx interrupts */
	RTL_W16(MultiIntr, RTL_R16(MultiIntr) & 0xF000);

}

static void rtl8169_init_ring(struct rtl8169_private *tpc)
{
	int i;

#ifdef DEBUG_RTL8169
	printf ("%s\n", __FUNCTION__);
#endif

	tpc->rxnext = 0;
	tpc->rxbusy = 0;
	memset(tpc->TxDescArray, 0x0, NUM_TX_DESC * sizeof(struct TxDesc));
	memset(tpc->RxDescArray, 0x0, NUM_RX_DESC * sizeof(struct RxDesc));

	for (i = 0; i < NUM_RX_DESC; i++) {
		if (i == (NUM_RX_DESC - 1))
			tpc->RxDescArray[i].status =
				cpu_to_le32((OWNbit | EORbit) + RX_BUF_SIZE);
		else
			tpc->RxDescArray[i].status =
				cpu_to_le32(OWNbit + RX_BUF_SIZE);
		tpc->busyrxdesc++;
		tpc->RxDescArray[i].buf_addr = cpu_to_le32((u32)(unsigned long)tpc->RxBufferRing[i]);
		//rtl_flush_rx_desc(&tpc->RxDescArray[i]);
	}
}

static void rtl8169_set_irq_mask(struct rtl8169_private *tpc)
{
	tpc->irq_mask = RxOK | RxErr | TxOK | TxErr | LinkChg | RxOverflow;
	RTL_W16(IntrMask, tpc->irq_mask);
#if 0
	if (tp->mac_version <= RTL_GIGA_MAC_VER_06)
		tp->irq_mask |= SYSErr | RxOverflow | RxFIFOOver;
	else if (tp->mac_version == RTL_GIGA_MAC_VER_11)
		/* special workaround needed */
		tp->irq_mask |= RxFIFOOver;
	else
		tp->irq_mask |= RxOverflow;
#endif
}

static void rtl8169_eth_start(struct rtl8169_private *tpc)
{
	int i, ret;
#ifdef DEBUG_RTL8169
	printf ("%s\n", __FUNCTION__);
#endif

	//ioaddr = dev_iobase;

	rtl8169_init_ring(tpc);
	rtl8169_hw_start(tpc);

	ret = pcie_alloc_irq_vectors(tpc->udev, 1, 1, tpc->msi_irq, PCI_IRQ_MSIX);
	if (ret < 0)
		hal_printf("enable msi irq fail\n");

	pci_set_master(tpc->udev, 1);
	rtl8169_set_irq_mask(tpc);

#if 0
	/* Construct a perfect filter frame with the mac address as first match
	 * and broadcast for all others */
	for (i = 0; i < 192; i++)
		txb[i] = 0xFF;

	txb[0] = enetaddr[0];
	txb[1] = enetaddr[1];
	txb[2] = enetaddr[2];
	txb[3] = enetaddr[3];
	txb[4] = enetaddr[4];
	txb[5] = enetaddr[5];
#endif
}

static void rtl_halt_common(struct rtl8169_private *tpc)
{
	int i;

#ifdef DEBUG_RTL8169
	printf ("%s\n", __FUNCTION__);
#endif

	//ioaddr = tpc->iobase;

	/* Stop the chip's Tx and Rx DMA processes. */
	RTL_W8(ChipCmd, 0x00);

	/* Disable interrupts by clearing the interrupt mask. */
	RTL_W16(IntrMask, 0x0000);

	RTL_W32(RxMissed, 0);

	for (i = 0; i < NUM_RX_DESC; i++) {
		tpc->RxBufferRing[i] = NULL;
	}
}

static int rtl8169_write_hwaddr(unsigned char *enetaddr)
{
	unsigned int i;

	RTL_W8(Cfg9346, Cfg9346_Unlock);

	for (i = 0; i < MAC_ADDR_LEN; i++)
		RTL_W8(MAC0 + i, enetaddr[i]);

	RTL_W8(Cfg9346, Cfg9346_Lock);

	return 0;
}

/**************************************************************************
INIT - Look for an adapter, this routine's visible to the outside
***************************************************************************/
static int rtl8169_resources_malloc(struct rtl8169_private *dev)
{
    int ret, i;

    dev->descs_noalign = hal_malloc(DESCRIPTORS_SIZE  + RTL8169_ALIGN);//memalign(EQOS_DESCRIPTOR_ALIGN, EQOS_DESCRIPTORS_SIZE);
    if (!dev->descs_noalign) {
        hal_printf("%s: eqos_alloc_descs() failed\n", __func__);
        ret = -1;
        goto err;
    }
    dev->descs = (void *)ALIGN((unsigned long)dev->descs_noalign, RTL8169_ALIGN);
    dev->TxDescArray = (struct TxDesc *)dev->descs;
    dev->RxDescArray = (struct RxDesc *)(dev->TxDescArray + NUM_TX_DESC);

    for (i = 0; i < NUM_TX_DESC; i++) {
	dev->tx_dma_buf_noalign[i] = hal_malloc(MAX_PACKET_SIZE + RTL8169_ALIGN);//memalign(EQOS_BUFFER_ALIGN, EQOS_MAX_PACKET_SIZE);
	if (!dev->tx_dma_buf_noalign) {
	    hal_printf("%s: memalign(tx_dma_buf) failed\n", __func__);
            ret = -1;
            goto err_free_descs;
        }
	dev->Tx_skbuff[i] = (void *)ALIGN((unsigned long)dev->tx_dma_buf_noalign[i], RTL8169_ALIGN);
    }

    for (i = 0; i < NUM_RX_DESC; i++) {
	dev->rx_dma_buf_noalign[i] = hal_malloc(MAX_PACKET_SIZE + RTL8169_ALIGN);//memalign(EQOS_BUFFER_ALIGN, EQOS_MAX_PACKET_SIZE);
	if (!dev->rx_dma_buf_noalign[i]) {
            hal_printf("%s: memalign(rx_dma_buf) failed\n", __func__);
            ret = -1;
            goto err_free_tx_dma_buf;
        }
	dev->RxBufferRing[i] = (void *)ALIGN((unsigned long)dev->rx_dma_buf_noalign[i], RTL8169_ALIGN);
    }

    //sys_gmac_invalidate_cache_range(rounddown((unsigned long)eqos_dev->rx_dma_buf, ARCH_DMA_MINALIGN), roundup((unsigned long)eqos_dev->rx_dma_buf + EQOS_RX_BUFFER_SIZE, ARCH_DMA_MINALIGN));

    return 0;

err_free_tx_dma_buf:
    for (i = 0; i < NUM_TX_DESC; i++) {
	hal_free(dev->tx_dma_buf_noalign[i]);
    }
err_free_descs:
    hal_free(dev->descs_noalign);
err:

    return ret;
}

static int rtl8169_resources_free(struct rtl8169_private *dev)
{
    int i;

    if(dev->descs != NULL)
    {
        hal_free(dev->descs_noalign);
        dev->descs_noalign = NULL;
        dev->descs = NULL;
        dev->TxDescArray = NULL;
        dev->RxDescArray = NULL;
    }
    for (i = 0; i < NUM_TX_DESC; i++)
	if (dev->Tx_skbuff[i])
	{
            hal_free(dev->tx_dma_buf_noalign[i]);
            dev->tx_dma_buf_noalign[i] = NULL;
            dev->Tx_skbuff[i] = NULL;
	}

    for (i = 0; i < NUM_RX_DESC; i++)
	if (dev->RxBufferRing[i])
        {
	    hal_free(dev->rx_dma_buf_noalign[i]);
	    dev->rx_dma_buf_noalign[i] = NULL;
	    dev->RxBufferRing[i] = NULL;
	}

    return 0;
}


#define board_found 1
#define valid_link 0
static int rtl_init(unsigned long dev_ioaddr,  struct rtl8169_private *tpc, const char *name,
		    unsigned char *enetaddr)
{
	static int board_idx = -1;
	int i, rc;
	int option = -1, Cap10_100 = 0, Cap1000 = 0;
	int ret;

#ifdef DEBUG_RTL8169
	printf ("%s\n", __FUNCTION__);
#endif

	ret = rtl8169_resources_malloc(tpc);
	if (ret < 0)
		return ret;

	ioaddr = dev_ioaddr;

	board_idx++;

	rc = rtl8169_init_board(tpc, name);
	if (rc)
		return rc;

#ifdef DEBUG_RTL8169
	printf("chipset = %d\n", tpc->chipset);
#endif

#ifdef DEBUG_RTL8169
	/* Print out some hardware info */
	printf("%s: at ioaddr 0x%lx\n", name, ioaddr);
#endif
	return 0;
}

void rtl8169_check_link(void *param)
{
	gmac_handle_t *handle = param;
	unsigned short link_status = 0;
	unsigned short link_status_old = handle->phy_dev->link_register & LinkStatus;
	u8 phy_status;

	phy_status = RTL_R8(PHYstatus);
	link_status = phy_status & LinkStatus;
	if (link_status_old != link_status) {
		if(link_status)
			gmac_link_change(handle, 1);
		else
			gmac_link_change(handle, 0);
	    handle->phy_dev->link_register = link_status;
	}
}

struct gmac_phy_ops rtl8169_phy_ops  = {
    .check_link_status = rtl8169_check_link,
};

static int rtl8169_phy_init(gmac_handle_t *gmac)
{
	u8 phy_status = RTL_R8(PHYstatus);
	struct gmac_dev *dev = gmac->phy_dev;

	dev->hal = gmac;

	dev->ops = &rtl8169_phy_ops;
	if (!(phy_status & TBI_Enable)) {
		/* always enable? */
	} else {
		sys_udelay(100);
#ifdef DEBUG_RTL8169
		printf
		    ("%s: 1000Mbps Full-duplex operation, TBI Link %s!\n",
		     name,
		     (RTL_R32(TBICSR) & TBILinkOK) ? "OK" : "Failed");
#endif
		if (phy_status & LinkStatus) {
			dev->link_status = 1;
		}
		dev->pcie_netcard = 1;
		dev->link_register = phy_status;
	}
}

static gmac_handle_t* rtl8169_eth_open(gmac_handle_t *gmac)
{
    struct udevice *udev = gmac->priv;
    struct rtl8169_private *priv;
    struct gmac_dev *phy_dev;
    int ret, msi_irq;
    u32 iobase;

    /*
     * Since the priv structure contains the descriptors which need a strict
     * buswidth alignment, memalign is used to allocate memory
     */
    msi_irq = alloc_msi_irq(udev, 1);
    if (msi_irq < 0) {
	hal_printf("all msi irq failed!\n");
	return NULL;
    }
    phy_dev = (struct gmac_dev *)hal_malloc(sizeof(struct gmac_dev));
    if (!phy_dev)
	return NULL;

    memset(phy_dev, 0, sizeof(struct gmac_dev));

    priv = (struct rtl8169_private *)hal_malloc(sizeof(struct rtl8169_private));
    if (!priv) {
	hal_free(phy_dev);
	hal_free(gmac);
        hal_printf("malloc mem failed!\n");
        return NULL;
    }

    memset(priv, 0, sizeof(struct rtl8169_private));

    priv->udev = udev;
    gmac->priv = priv;
    gmac->phy_dev = phy_dev;

    priv->iobase = gmac->pcie_iobase;
    priv->mmio_addr = (void *)gmac->pcie_iobase;
    priv->msi_irq = msi_irq;
    printf("rtl8169: REALTEK RTL8169 @0x%x 0x%x\n", priv->iobase, priv->mmio_addr);

    ret = rtl_init(priv->iobase, priv, gmac->name, gmac->gmac_config.enetaddr);
    if (ret < 0) {
	    hal_printf("failed to initialize card: %d\n", ret);
	    return NULL;
    }

    ret = rtl8169_phy_init(gmac);

    u32 val = RTL_R32(FuncEvent);
    printf("%s: FuncEvent/Misc (0xF0) = 0x%08X\n", __func__, val);
    val &= ~RxDv_Gated_En;
    RTL_W32(FuncEvent, val);

    register_msi_irq(gmac, gmac->msi_handler, udev, msi_irq);

    rtl8169_eth_start(priv);

    return gmac;
}

#if 0
static void rtl8169_pcierr_interrupt(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct pci_dev *pdev = tp->pci_dev;
	int pci_status_errs;
	u16 pci_cmd;

	pci_read_config_word(pdev, PCI_COMMAND, &pci_cmd);

	pci_status_errs = pci_status_get_and_clear_errors(pdev);

	if (net_ratelimit())
		netdev_err(dev, "PCI error (cmd = 0x%04x, status_errs = 0x%04x)\n",
			   pci_cmd, pci_status_errs);
}
#endif

static int rtl8169_gmac_isr(void *priv)
{
	struct rtl8169_private *tp = priv;
	u32 status = RTL_R16(IntrStatus);
	int ret = 0;

	if (status & SYSErr) {
		hal_printf("pcie gmac sys error\n");
		//rtl8169_pcierr_interrupt(tp);
		return INT_TX_HARD_ERROR;
	}

	if ((status & 0xffff) == 0xffff || !(status & tp->irq_mask))
		return 0;

	if (status & TxOK)
		ret |= INT_TX;

	if (status & RxOK)
		ret |= INT_RX;

	if (status & LinkChg) {
		hal_printf("link status %x\n", RTL_R8(PHYstatus));
		ret |= LINK_STATUS;
	}

	if (status & (RxOverflow | TxErr | RxErr)) {
		ret = INT_TX_HARD_ERROR;
		hal_printf("status %x\n", status);
	}

	RTL_W16(IntrStatus, status);

	return ret;
}

static struct gmac_ops rtl8169_ops = {
    .open = rtl8169_eth_open,
    .recv = rtl8169_eth_recv,
    .send = rtl8169_eth_send,
    .check_descriptor = rtl8169_check_descriptor,
    .free_pkt = rtl8169_free_pkt,
    .gmac_isr = rtl8169_gmac_isr,
};


static struct pci_device_id supported[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8167) },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8168) },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8169) },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8161) },
	{}
};

int rtl_gmac_ops_init(gmac_handle_t *handle)
{
    struct udevice *dev;
    int i, region;
    unsigned int iobase;

    if (!handle->pcie_iobase) {
	for (i = 0; i < sizeof(supported)/sizeof(supported[0]); i++) {
	    dev = get_match_pci_device(&supported[i]);
	    if (dev)
		break;
	}
    }

    if (!dev)
	return -1;

    switch (supported[i].device) {
    case 0x8168:
    case 0x8161:
	    region = 2;
	    break;
    default:
	    region = 1;
	    break;
    }
    dm_pci_read_config32(dev, PCI_BASE_ADDRESS_0 + region * 4, &iobase);

    iobase &= ~0xf;

    printf("iobase %x\n", iobase);

    handle->pcie_iobase = iobase;
    handle->priv = dev;

    ioaddr = iobase;

    /* Get MAC address.  FIXME: read EEPROM */
    for (i = 0; i < MAC_ADDR_LEN; i++)
	handle->gmac_config.enetaddr[i] = RTL_R8(MAC0 + i);

    hal_printf("MAC Address");
    for (i = 0; i < MAC_ADDR_LEN; i++)
	    hal_printf(":%02x", handle->gmac_config.enetaddr[i]);
    hal_printf("\n");

    handle->ops = &rtl8169_ops;

    return 0;
}

