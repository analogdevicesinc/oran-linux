// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/if_ether.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/spinlock.h>
#include <linux/bitfield.h>
#include <net/rtnetlink.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/jhash.h>
#include <linux/jiffies.h>
#include "adrv906x-ndma.h"

#define NDMA_TX_STAT_AND_CTRL                      0x000
#define   NDMA_DATAPATH_EN                         BIT(0)
#define NDMA_TX_EVENT_EN                           0x004
#define NDMA_TX_EVENT_STAT                         0x008
#define   NDMA_TX_STATUS_FIFO_FULL_EVENT           BIT(4)
#define   NDMA_TX_FRAME_SIZE_ERR_EVENT             BIT(3)
#define   NDMA_TX_STATUS_WRITE_COMPLETE_EVENT      BIT(2)
#define   NDMA_TX_WORKUNIT_COMPLETE_EVENT          BIT(1)
#define   NDMA_TX_WU_HEADER_ERR_EVENT              BIT(0)
#define NDMA_TX_TIMEOUT_VALUE                      0x00c
#define NDMA_TX_FRAME_SIZE                         0x010
#define   NDMA_TX_MIN_FRAME_SIZE                   GENMASK(15, 0)
#define   NDMA_TX_MAX_FRAME_SIZE                   GENMASK(31, 16)

#define NDMA_TX_ERROR_EVENTS    (NDMA_TX_FRAME_SIZE_ERR_EVENT | \
				 NDMA_TX_WU_HEADER_ERR_EVENT)
#define NDMA_TX_STATUS_EVENTS   (NDMA_TX_STATUS_FIFO_FULL_EVENT | \
				 NDMA_TX_STATUS_WRITE_COMPLETE_EVENT | \
				 NDMA_TX_WORKUNIT_COMPLETE_EVENT)

#define NDMA_RX_STAT_AND_CTRL                      0x000
#define   NDMA_RX_DATAPATH_EN                      BIT(0)
#define   NDMA_LOOPBACK_EN                         BIT(20)
#define NDMA_RX_EVENT_EN                           0x004
#define NDMA_RX_EVENT_STAT                         0x008
#define   NDMA_RX_FRAME_SIZE_ERR_EVENT             BIT(4)
#define   NDMA_RX_ERR_EVENT                        BIT(3)
#define   NDMA_RX_STATUS_WR_EVENT                  BIT(2)
#define   NDMA_RX_WORKUNIT_COMPLETE_EVENT          BIT(1)
#define   NDMA_RX_FRAME_DROPPED_ERR_EVENT          BIT(0)
#define NDMA_RX_FRAME_DROPPED_COUNT_MPLANE         0x00c
#define NDMA_RX_FRAME_DROPPED_COUNT_SPLANE         0x010
#define NDMA_RX_FRAME_SIZE                         0x014
#define   NDMA_RX_MIN_FRAME_SIZE                   GENMASK(15, 0)
#define   NDMA_RX_MAX_FRAME_SIZE                   GENMASK(31, 16)
#define NDMA_RX_SYNC_FIFO_MPLANE_THRESHOLD         0x018
#define NDMA_RX_IPV4_FRAME_FIELD_VALUE0            0x100
#define NDMA_RX_IPV4_FRAME_FIELD_VALUE1            0x104
#define NDMA_RX_IPV4_FRAME_FIELD_OFFSET0           0x108
#define NDMA_RX_IPV4_FRAME_FIELD_OFFSET1           0x10c
#define NDMA_RX_IPV4_FRAME_FIELD_MASK0             0x110
#define NDMA_RX_IPV4_FRAME_FIELD_MASK1             0x114
#define NDMA_RX_IPV6_FRAME_FIELD_VALUE0            0x200
#define NDMA_RX_IPV6_FRAME_FIELD_VALUE1            0x204
#define NDMA_RX_IPV6_FRAME_FIELD_OFFSET0           0x208
#define NDMA_RX_IPV6_FRAME_FIELD_OFFSET1           0x20c
#define NDMA_RX_IPV6_FRAME_FIELD_MASK0             0x210
#define NDMA_RX_IPV6_FRAME_FIELD_MASK1             0x214
#define NDMA_RX_ETH_FRAME_FIELD_VALUE              0x300
#define NDMA_RX_ETH_FRAME_FIELD_OFFSET             0x304
#define NDMA_RX_ETH_FRAME_FIELD_MASK               0x308
#define NDMA_RX_SPLANE_FILTER_PTP_MSG_VALUE        0x400
#define NDMA_RX_SPLANE_FILTER_VLAN_TAG_VALUE       0x404
#define NDMA_RX_SPLANE_FILTER_VLAN_TAG_OFFSET      0x408
#define NDMA_RX_SPLANE_FILTER_VLAN_TAG_MASK        0x40c
#define NDMA_RX_SPLANE_FILTER_VLAN_FRAME_OFFSET    0x410
#define NDMA_RX_SPLANE_FILTER_EN                   0x414
#define NDMA_RX_GEN_FILTER_REG_STRIDE              0x100
#define NDMA_RX_GEN_FILTER0_REG_OFFSET             0x500
#define NDMA_RX_GEN_FILTER1_REG_OFFSET             0x600
#define NDMA_RX_GEN_FILTER2_REG_OFFSET             0x700
#define NDMA_RX_GEN_FILTER3_REG_OFFSET             0x800
#define NDMA_RX_GEN_FILTER4_REG_OFFSET             0x900

#define NDMA_RX_CYCLE0_LOWER_VALUE_REG_OFFSET      0x00
#define NDMA_RX_CYCLE0_LOWER_MASK_REG_OFFSET       0x60

#define NDMA_RX_ERROR_EVENTS    (NDMA_RX_FRAME_SIZE_ERR_EVENT | \
				 NDMA_RX_ERR_EVENT | \
				 NDMA_RX_FRAME_DROPPED_ERR_EVENT)
#define NDMA_RX_STATUS_EVENTS   (NDMA_RX_STATUS_WR_EVENT | \
				 NDMA_RX_WORKUNIT_COMPLETE_EVENT)
#define NDMA_RX_FRAME_SIZE_DISABLED_MAX            0xffff
#define NDMA_RX_FRAME_SIZE_DISABLED_MIN            0

/* PTP Filter Configuration Values
 * These filters match PTP (IEEE 1588) packets in various encapsulations
 */

/* RX IPv4 PTP Filter - Matches PTP over IPv4/UDP packets
 * Value0: [31:24] Protocol (0x11=UDP)
 *         [23:16] IPv4 version (0x45)
 *         [15:0]  EtherType (0x0800=IPv4)
 * Value1: [31:16] UDP dst port 1 (0x140=320), [15:0] UDP dst port 0 (0x13f=319)
 * Offsets0: [31:24] UDP dst port, [23:16] Protocol, [15:8] Version, [7:0] EtherType
 */
#define NDMA_RX_IPV4_PTP_PROTO_VER_TYPE            0x11450800  /* Protocol, Ver/IHL, EType */
#define NDMA_RX_IPV4_PTP_UDP_PORTS                 0x0140013f  /* PTP ports: 320, 319 */
#define NDMA_RX_IPV4_PTP_FIELD_OFFSETS0            0x24170e0c  /* Field offsets */
#define NDMA_RX_IPV4_PTP_MSG_TYPE_OFFSET           0x0000002a  /* PTP msg type offset */
#define NDMA_RX_IPV4_PTP_FIELD_MASKS0              0x00000000  /* No masking */
#define NDMA_RX_IPV4_PTP_MSG_TYPE_MASK             0x000f0000  /* PTP msg type mask */

/* RX IPv6 PTP Filter - Matches PTP over IPv6/UDP packets
 * Value0: [31:24] Protocol (0x11=UDP)
 *         [23:16] IPv6 version (0x06)
 *         [15:0]  EtherType (0x86dd=IPv6)
 * Value1: [31:16] UDP dst port 1 (0x140=320), [15:0] UDP dst port 0 (0x13f=319)
 * Offsets0: [31:24] UDP dst port, [23:16] Protocol, [15:8] Version, [7:0] EtherType
 */
#define NDMA_RX_IPV6_PTP_PROTO_VER_TYPE            0x110686dd  /* Protocol, Ver, EType */
#define NDMA_RX_IPV6_PTP_UDP_PORTS                 0x0140013f  /* PTP ports: 320, 319 */
#define NDMA_RX_IPV6_PTP_FIELD_OFFSETS0            0x38140e0c  /* Field offsets */
#define NDMA_RX_IPV6_PTP_MSG_TYPE_OFFSET           0x0000003e  /* PTP msg type offset */
#define NDMA_RX_IPV6_PTP_FIELD_MASKS0              0x00000000  /* No masking */
#define NDMA_RX_IPV6_PTP_MSG_TYPE_MASK             0x000f0000  /* PTP msg type mask */

/* RX Ethernet PTP Filter - Matches PTP over raw Ethernet (L2)
 * Value: [15:0] EtherType (0x88f7=PTPv2 over Ethernet)
 * Offsets: [15:8] PTP msg type offset (0x0e=14), [7:0] EtherType offset (0x0c=12)
 * Mask: [23:16] PTP msg type mask (0x0f), [15:0] EtherType mask (0x0000=no masking)
 */
#define NDMA_RX_ETH_PTP_ETHERTYPE                  0x000088f7  /* EtherType for PTPv2 */
#define NDMA_RX_ETH_PTP_TYPE_MSG_OFFSETS           0x00000e0c  /* Field offsets */
#define NDMA_RX_ETH_PTP_MSG_TYPE_MASK              0x000f0000  /* PTP msg type mask */

/* RX S-Plane Filter - Matches specific PTP message types and VLAN tags
 * PTP msg types: Sync(0), Delay_Req(1), Follow_Up(8), Delay_Resp(9),
 *                Announce(B), Signaling(C)
 */
#define NDMA_RX_SPLANE_PTP_MSG_TYPES               0x00cb9810  /* PTP msg types */
#define NDMA_RX_SPLANE_VLAN_TAGS                   0x88a88100  /* Q-in-Q, C-Tag */
#define NDMA_RX_SPLANE_VLAN_TAG_OFFSET             0x0000000c  /* VLAN tag offset */
#define NDMA_RX_SPLANE_VLAN_TAG_MASK               0x00000000  /* No VLAN masking */
#define NDMA_RX_SPLANE_VLAN_FRAME_OFFSET           0x00000084  /* Frame offset */

/* RX eCPRI Filter - Matches eCPRI One-Way Delay Measurement messages */
#define NDMA_RX_ECPRI_FILTER_PATTERN_12            0x0500feae  /* Bytes 12-15 */
#define NDMA_RX_ECPRI_FILTER_MASK_12               0x00ff0000  /* Mask byte 14 */
#define NDMA_RX_ECPRI_FILTER_PATTERN_16            0x00000000  /* Bytes 16-19 */
#define NDMA_RX_ECPRI_FILTER_MASK_16               0x01ffffff  /* Mask bytes 16-18 */
#define NDMA_RX_ECPRI_FILTER_MASK_ANY              0xffffffff  /* Accept any byte value */
#define NDMA_RX_ECPRI_FILTER_SIZE                  96          /* Filter size */

#define NDMA_INTR_CTRL_TX                          0x00
#define   NDMA_INTR_CTRL_TX_DMA_ERR_EN             BIT(4)
#define   NDMA_INTR_CTRL_TX_DMA_DMADONE_EN         BIT(3)
#define   NDMA_INTR_CTRL_TX_DMA_DONE_EN            BIT(2)
#define   NDMA_INTR_CTRL_TX_ERR_EN                 BIT(0)
#define NDMA_INTR_CTRL_STATUS                      0x10
#define   NDMA_INTR_CTRL_TX_STATUS_DMA_ERR_EN      BIT(4)
#define   NDMA_INTR_CTRL_TX_STATUS_DMA_DMADONE_EN  BIT(3)
#define   NDMA_INTR_CTRL_TX_STATUS_DMA_DONE_EN     BIT(2)
#define   NDMA_INTR_CTRL_TX_STATUS_EN              BIT(1)
#define   NDMA_INTR_CTRL_RX_STATUS_EN              BIT(0)
#define NDMA_INTR_CTRL_RX                          0x20
#define   NDMA_INTR_CTRL_RX_DMA_ERR_EN             BIT(4)
#define   NDMA_INTR_CTRL_RX_DMA_DMADONE_EN         BIT(3)
#define   NDMA_INTR_CTRL_RX_DMA_DONE_EN            BIT(2)
#define   NDMA_INTR_CTRL_RX_ERR_EN                 BIT(0)

#define NDMA_RESET                                 0x00
#define   NDMA_RX0_RST                             BIT(0)
#define   NDMA_RX1_RST                             BIT(1)
#define   NDMA_TX0_RST                             BIT(2)
#define   NDMA_TX1_RST                             BIT(3)
#define   NDMA_TX0_PTP_MODE                        BIT(4)
#define   NDMA_TX1_PTP_MODE                        BIT(5)

#define DMA_NEXT_DESC           0x00
#define DMA_ADDRSTART           0x04

#define DMA_CFG                 0x08
#define   DMA2D                 BIT(26)                 /* DMA Mode (2D/1D*) */
#define   DESCIDCPY             BIT(25)                 /* Descriptor ID Copy Control */
#define   DMA_INT_MSK           GENMASK(21, 20)         /* Generate Interrupt Bits Mask */
#define   DI_EN_X               0x00100000              /* Data Interrupt Enable in X count */
#define   DI_EN_Y               0x00200000              /* Data Interrupt Enable in Y count */
#define   DI_EN_P               0x00300000              /* Data Interrupt Enable in Peripheral */
#define   DI_EN                 DI_EN_X                 /* Data Interrupt Enable */
#define   NDSIZE                GENMASK(18, 16)         /* Next Descriptor */
#define   NDSIZE_0              0x00000000              /* Next Descriptor Size 1 */
#define   NDSIZE_1              0x00010000              /* Next Descriptor Size 2 */
#define   NDSIZE_2              0x00020000              /* Next Descriptor Size 3 */
#define   NDSIZE_3              0x00030000              /* Next Descriptor Size 4 */
#define   NDSIZE_4              0x00040000              /* Next Descriptor Size 5 */
#define   NDSIZE_5              0x00050000              /* Next Descriptor Size 6 */
#define   NDSIZE_6              0x00060000              /* Next Descriptor Size 7 */
#define   NDSIZE_OFFSET         16                      /* Next Descriptor Size Offset */
#define   DMAFLOW               GENMASK(14, 12)         /* Flow Control */
#define   DMAFLOW_STOP          0x00000000              /* Stop Mode */
#define   DMAFLOW_AUTO          0x00001000              /* Autobuffer Mode */
#define   DMAFLOW_LIST          0x00004000              /* Descriptor List Mode */
#define   DMAFLOW_LARGE         DMAFLOW_LIST
#define   DMAFLOW_ARRAY         0x00005000              /* Descriptor Array Mode */
#define   DMAFLOW_LIST_DEMAND   0x00006000              /* Descriptor Demand List Mode */
#define   DMAFLOW_ARRAY_DEMAND  0x00007000              /* Descriptor Demand Array Mode */
#define   WDSIZE_MSK            GENMASK(10, 8)          /* Memory Transfer Word Size Mask */
#define   WDSIZE_8              0x00000000              /* Memory Transfer Word Size 8 bits */
#define   WDSIZE_16             0x00000100              /* Memory Transfer Word Size 16 bits */
#define   WDSIZE_32             0x00000200              /* Memory Transfer Word Size 32 bits */
#define   WDSIZE_64             0x00000300              /* Memory Transfer Word Size 64 bits */
#define   WDSIZE_128            0x00000400              /* Memory Transfer Word Size 128 bits */
#define   WDSIZE_256            0x00000500              /* Memory Transfer Word Size 256 bits */
#define   PSIZE_MSK             GENMASK(6, 4)           /* Peripheral Transfer Word Size Mask */
#define   PSIZE_8               0x00000000              /* Peripheral Transfer Word Size 8 bits */
#define   PSIZE_16              0x00000010              /* Peripheral Transfer Word Size 16 bits */
#define   PSIZE_32              0x00000020              /* Peripheral Transfer Word Size 32 bits */
#define   PSIZE_64              0x00000030              /* Peripheral Transfer Word Size 64 bits */
#define   DMASYNC               BIT(2)                  /* DMA Buffer Clear SYNC */
#define   WNR                   BIT(1)                  /* Channel Direction (W/R*) */
#define   DMAEN                 BIT(0)                  /* DMA Channel Enable */
#define DMA_XCNT                0x0c
#define DMA_XMOD                0x10
#define    XMODE_8              0x01
#define    XMODE_16             0x02
#define    XMODE_32             0x04
#define    XMODE_64             0x08
#define    XMODE_128            0x10
#define    XMODE_256            0x20
#define DMA_YCNT                0x14
#define DMA_YMOD                0x18
#define DSCPTR_CUR              0x24
#define DSCPTR_PRV              0x28
#define DMA_ADDR_CUR            0x2c
#define DMA_STAT                0x30
#define   DMA_RUN_MASK          GENMASK(10, 8)          /* DMA Run Bits Mask */
#define   DMA_RUN_IDLE          0x00000000              /* DMA Run IDLE */
#define   DMA_RUN_DFETCH        0x00000100              /* DMA Run Fetch */
#define   DMA_RUN               0x00000200              /* DMA Run Trans */
#define   DMA_RUN_WAIT_TRIG     0x00000300              /* DMA Run WAIT TRIG */
#define   DMA_RUN_WAIT_ACK      0x00000400              /* DMA Run WAIT ACK */
#define   DMA_PIRQ              BIT(2)                  /* DMA Peripheral Error Interrupt Status */
#define   DMA_ERR               BIT(1)                  /* DMA Error Interrupt Status */
#define   DMA_DONE              BIT(0)                  /* DMA Completion Interrupt Status */
#define DMA_XCNT_CUR            0x34
#define DMA_YCNT_CUR            0x38
#define DMA_BWLCNT              0x40
#define DMA_BWLCNT_CUR          0x44
#define DMA_BWMCNT              0x48
#define DMA_BWMCNT_CUR          0x4c
#define   FULL_BANDWIDTH        0x0000
#define   SUSPEND_TRANSFER      0xffff

#define DMA_DESC_FETCH          0x100                   /* DMA is fetching descriptors */
#define DMA_DATA_XFER           0x200                   /* DMA is in data transfer state */
#define DMA_IDLE_MASK           0x700
#define DMA_IDLE(x)             (((x) & DMA_IDLE_MASK) == 0)
#define DMA_FETCHING_DESC(x)    ((x) & DMA_DESC_FETCH)
#define DMA_XFER_DATA(x)        ((x) & DMA_DATA_XFER)

#define NDMA_TX_PACKET_LIST_TIMEOUT_MS              500
#define NDMA_TX_PTP_MAX_SEQNUM                      8
#define NDMA_TX_PTP_MIN_SEQNUM                      1
#define NDMA_TX_MAX_SEQNUM                          255
#define NDMA_TX_MIN_SEQNUM                          (NDMA_TX_PTP_MAX_SEQNUM + 1)
#define NDMA_TX_TSTAMP_TIMEOUT_CNT_THRESHOLD        1

static DEFINE_SPINLOCK(ndma_reset_lock);

enum adrv906x_ndma_rx_filter_status {
	NDMA_RX_FILTER_OFF,
	NDMA_RX_FILTER_ON
};

enum adrv906x_ndma_error {
	NDMA_NO_ERROR,
	NDMA_TX_FRAME_SIZE_ERROR,
	NDMA_TX_DATA_HEADER_ERROR,
	NDMA_TX_STATUS_HEADER_ERROR,
	NDMA_TX_TSTAMP_TIMEOUT_ERROR,
	NDMA_TX_SEQNUM_MISMATCH_ERROR,
	NDMA_TX_DMA_TRANS_ERROR,
	NDMA_TX_UNKNOWN_ERROR,
	NDMA_RX_FRAME_SIZE_ERROR,
	NDMA_RX_FRAME_DROPPED_ERROR,
	NDMA_RX_ERROR,
	NDMA_RX_SEQNUM_MISMATCH_ERROR,
	NDMA_RX_DMA_TRANS_ERROR,
	NDMA_RX_UNKNOWN_ERROR,
};

enum adrv906x_ndma_irqs {
	NDMA_TX_DATA_DMA_ERR_IRQ	= BIT(0),
	NDMA_TX_DATA_DMA_DMADONE_IRQ	= BIT(1),
	NDMA_TX_DATA_DMA_DONE_IRQ	= BIT(2),
	NDMA_TX_STATUS_DMA_ERR_IRQ	= BIT(3),
	NDMA_TX_STATUS_DMA_DMADONE_IRQ	= BIT(4),
	NDMA_TX_STATUS_DMA_DONE_IRQ	= BIT(5),
	NDMA_TX_STATUS_IRQ		= BIT(6),
	NDMA_TX_ERR_IRQ			= BIT(7),
	NDMA_RX_ERR_IRQ			= BIT(8),
	NDMA_RX_STATUS_IRQ		= BIT(9),
	NDMA_RX_DMA_ERR_IRQ		= BIT(10),
	NDMA_RX_DMA_DMADONE_IRQ		= BIT(11),
	NDMA_RX_DMA_DONE_IRQ		= BIT(12),
};

enum adrv906x_ndma_rx_filter_id {
	NDMA_RX_IPV4_FILTER = 0,
	NDMA_RX_IPV6_FILTER,
	NDMA_RX_ETH_FILTER,
	NDMA_RX_GENERIC_FILTER_0,
	NDMA_RX_GENERIC_FILTER_1,
	NDMA_RX_GENERIC_FILTER_2,
	NDMA_RX_GENERIC_FILTER_3,
	NDMA_RX_GENERIC_FILTER_4,
	NDMA_RX_FILTER_CNT,
};

static void get_ts_from_status(u8 *status, struct timespec64 *ts)
{
	ts->tv_nsec = get_unaligned_le32(&status[4]);
	ts->tv_sec = get_unaligned_le32(&status[8]) |
		     ((u64)get_unaligned_le16(&status[12]) << 32);
}

static bool is_timestamp_all_zero(u8 *status)
{
	return (get_unaligned_le32(&status[2]) |
		get_unaligned_le32(&status[6]) |
		get_unaligned_le32(&status[10])) == 0;
}

static void adrv906x_ndma_enable_irqs(struct adrv906x_ndma_dev *ndma_dev,
				      enum adrv906x_ndma_irqs irqs)
{
	u32 val;

	val = 0;
	if (irqs & NDMA_TX_DATA_DMA_ERR_IRQ)
		val |= NDMA_INTR_CTRL_TX_DMA_ERR_EN;
	if (irqs & NDMA_TX_DATA_DMA_DMADONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_DMA_DMADONE_EN;
	if (irqs & NDMA_TX_DATA_DMA_DONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_DMA_DONE_EN;
	if (irqs & NDMA_TX_ERR_IRQ)
		val |= NDMA_INTR_CTRL_TX_ERR_EN;
	if (val) {
		val |= ioread32(ndma_dev->intr_ctrl + NDMA_INTR_CTRL_TX);
		iowrite32(val, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_TX);
	}

	val = 0;
	if (irqs & NDMA_TX_STATUS_DMA_ERR_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_DMA_ERR_EN;
	if (irqs & NDMA_TX_STATUS_DMA_DMADONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_DMA_DMADONE_EN;
	if (irqs & NDMA_TX_STATUS_DMA_DONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_DMA_DONE_EN;
	if (irqs & NDMA_TX_STATUS_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_EN;
	if (irqs & NDMA_RX_STATUS_IRQ)
		val |= NDMA_INTR_CTRL_RX_STATUS_EN;
	if (val) {
		val |= ioread32(ndma_dev->intr_ctrl + NDMA_INTR_CTRL_STATUS);
		iowrite32(val, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_STATUS);
	}

	val = 0;
	if (irqs & NDMA_RX_DMA_ERR_IRQ)
		val |= NDMA_INTR_CTRL_RX_DMA_ERR_EN;
	if (irqs & NDMA_RX_DMA_DMADONE_IRQ)
		val |= NDMA_INTR_CTRL_RX_DMA_DMADONE_EN;
	if (irqs & NDMA_RX_DMA_DONE_IRQ)
		val |= NDMA_INTR_CTRL_RX_DMA_DONE_EN;
	if (irqs & NDMA_RX_ERR_IRQ)
		val |= NDMA_INTR_CTRL_RX_ERR_EN;

	if (val) {
		val |= ioread32(ndma_dev->intr_ctrl + NDMA_INTR_CTRL_RX);
		iowrite32(val, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_RX);
	}
}

static void adrv906x_ndma_disable_irqs(struct adrv906x_ndma_dev *ndma_dev,
				       enum adrv906x_ndma_irqs irqs)
{
	u32 val;

	val = 0;
	if (irqs & NDMA_TX_DATA_DMA_ERR_IRQ)
		val |= NDMA_INTR_CTRL_TX_DMA_ERR_EN;
	if (irqs & NDMA_TX_DATA_DMA_DMADONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_DMA_DMADONE_EN;
	if (irqs & NDMA_TX_DATA_DMA_DONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_DMA_DONE_EN;
	if (irqs & NDMA_TX_ERR_IRQ)
		val |= NDMA_INTR_CTRL_TX_ERR_EN;
	if (val) {
		val = ioread32(ndma_dev->intr_ctrl + NDMA_INTR_CTRL_TX) & ~val;
		iowrite32(val, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_TX);
	}

	val = 0;
	if (irqs & NDMA_TX_STATUS_DMA_ERR_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_DMA_ERR_EN;
	if (irqs & NDMA_TX_STATUS_DMA_DMADONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_DMA_DMADONE_EN;
	if (irqs & NDMA_TX_STATUS_DMA_DONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_DMA_DONE_EN;
	if (irqs & NDMA_TX_STATUS_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_EN;
	if (irqs & NDMA_RX_STATUS_IRQ)
		val |= NDMA_INTR_CTRL_RX_STATUS_EN;
	if (val) {
		val = ioread32(ndma_dev->intr_ctrl + NDMA_INTR_CTRL_STATUS) & ~val;
		iowrite32(val, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_STATUS);
	}

	val = 0;
	if (irqs & NDMA_RX_DMA_ERR_IRQ)
		val |= NDMA_INTR_CTRL_RX_DMA_ERR_EN;
	if (irqs & NDMA_RX_DMA_DMADONE_IRQ)
		val |= NDMA_INTR_CTRL_RX_DMA_DMADONE_EN;
	if (irqs & NDMA_RX_DMA_DONE_IRQ)
		val |= NDMA_INTR_CTRL_RX_DMA_DONE_EN;
	if (irqs & NDMA_RX_ERR_IRQ)
		val |= NDMA_INTR_CTRL_RX_ERR_EN;
	if (val) {
		val = ioread32(ndma_dev->intr_ctrl + NDMA_INTR_CTRL_RX) & ~val;
		iowrite32(val, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_RX);
	}
}

static void adrv906x_ndma_disable_all_irqs(struct adrv906x_ndma_dev *ndma_dev)
{
	iowrite32(0, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_TX);
	iowrite32(0, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_STATUS);
	iowrite32(0, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_RX);
}

static void adrv906x_dma_rx_reset(struct adrv906x_ndma_chan *ndma_ch)
{
	iowrite32(0, ndma_ch->rx_dma_base + DMA_CFG);
	iowrite32(DMA_ERR | DMA_PIRQ | DMA_DONE, ndma_ch->rx_dma_base + DMA_STAT);
}

static void adrv906x_dma_tx_reset(struct adrv906x_ndma_chan *ndma_ch)
{
	if (ndma_ch->tx_dma_base) {
		iowrite32(0, ndma_ch->tx_dma_base + DMA_CFG);
		iowrite32(DMA_ERR | DMA_PIRQ | DMA_DONE, ndma_ch->tx_dma_base + DMA_STAT);
	}
}

static void adrv906x_dma_rx_start(struct adrv906x_ndma_chan *ndma_ch)
{
	dma_addr_t desc_addr;

	desc_addr = ndma_ch->rx_ring_dma + sizeof(struct dma_desc) * ndma_ch->rx_tail;

	iowrite32(0, ndma_ch->rx_dma_base + DMA_CFG);
	iowrite32(0, ndma_ch->rx_dma_base + DMA_STAT);
	iowrite32(desc_addr, ndma_ch->rx_dma_base + DMA_NEXT_DESC);
	iowrite32(ndma_ch->rx_ring[ndma_ch->rx_tail].cfg, ndma_ch->rx_dma_base + DMA_CFG);
}

static void adrv906x_dma_tx_start(struct adrv906x_ndma_chan *ndma_ch)
{
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	dma_addr_t desc_addr;

	desc_addr = ndma_ch->tx_ring_dma + sizeof(struct dma_desc) * ndma_ch->tx_tail;
	iowrite32(0, ndma_ch->tx_dma_base + DMA_CFG);
	iowrite32(desc_addr, ndma_ch->tx_dma_base + DMA_NEXT_DESC);
	iowrite32(ndma_ch->tx_ring[ndma_ch->tx_tail].cfg | DMAFLOW_LIST,
		  ndma_ch->tx_dma_base + DMA_CFG);

	queue_delayed_work(ndma_dev->wq, &ndma_ch->tx_frames_timeout_work,
			   msecs_to_jiffies(NDMA_TX_PACKET_LIST_TIMEOUT_MS));
}

static irqreturn_t adrv906x_dma_rx_done_irq_handler(int irq, void *ctx)
{
	struct adrv906x_ndma_chan *ndma_ch = ctx;
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	unsigned long flags;

	if (napi_schedule_prep(&ndma_ch->napi)) {
		spin_lock_irqsave(&ndma_ch->lock, flags);
		if (ndma_ch->chan_type == NDMA_RX_CHANNEL)
			adrv906x_ndma_disable_irqs(ndma_dev, NDMA_RX_DMA_DONE_IRQ);
		else
			adrv906x_ndma_disable_irqs(ndma_dev, NDMA_TX_STATUS_DMA_DONE_IRQ);
		spin_unlock_irqrestore(&ndma_ch->lock, flags);
		__napi_schedule(&ndma_ch->napi);
	}

	return IRQ_HANDLED;
}

static irqreturn_t adrv906x_dma_error_irq_handler(int irq, void *ctx)
{
	struct adrv906x_ndma_chan *ndma_ch = ctx;

	if (ndma_ch->rx_dma_error_irq == irq)
		adrv906x_dma_rx_reset(ndma_ch);
	else
		adrv906x_dma_tx_reset(ndma_ch);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t adrv906x_dma_error_irq_handler_thread(int irq, void *ctx)
{
	struct adrv906x_ndma_chan *ndma_ch = ctx;
	unsigned long flags;

	/* DMA error handling: increment statistics for monitoring */
	spin_lock_irqsave(&ndma_ch->lock, flags);
	if (ndma_ch->chan_type == NDMA_RX_CHANNEL) {
		ndma_ch->stats.rx.dma_errors++;
	} else {
		if (ndma_ch->rx_dma_error_irq == irq)
			ndma_ch->stats.tx.status_dma_errors++;
		else
			ndma_ch->stats.tx.data_dma_errors++;
	}
	spin_unlock_irqrestore(&ndma_ch->lock, flags);

	return IRQ_HANDLED;
}

static void adrv906x_ndma_chan_enable(struct adrv906x_ndma_chan *ndma_ch)
{
	u32 val, offset;

	offset = (ndma_ch->chan_type == NDMA_RX_CHANNEL) ?
		 NDMA_RX_STAT_AND_CTRL : NDMA_TX_STAT_AND_CTRL;

	val = ioread32(ndma_ch->ctrl_base + offset);
	val |= NDMA_DATAPATH_EN;
	iowrite32(val, ndma_ch->ctrl_base + offset);
}

static bool adrv906x_ndma_chan_enabled(struct adrv906x_ndma_chan *ndma_ch)
{
	u32 val, offset;

	offset = (ndma_ch->chan_type == NDMA_RX_CHANNEL) ?
		 NDMA_RX_STAT_AND_CTRL : NDMA_TX_STAT_AND_CTRL;
	val = ioread32(ndma_ch->ctrl_base + offset);
	return val & NDMA_DATAPATH_EN;
}

static void adrv906x_ndma_chan_disable(struct adrv906x_ndma_chan *ndma_ch)
{
	u32 val, offset;

	offset = (ndma_ch->chan_type == NDMA_RX_CHANNEL) ?
		 NDMA_RX_STAT_AND_CTRL : NDMA_TX_STAT_AND_CTRL;

	val = ioread32(ndma_ch->ctrl_base + offset);
	val &= ~NDMA_DATAPATH_EN;
	iowrite32(val, ndma_ch->ctrl_base + offset);

	/* Reset DMAs just to ensure there will be no active DMA transfer during NDMA reset */
	adrv906x_dma_rx_reset(ndma_ch);
	adrv906x_dma_tx_reset(ndma_ch);
}

static void adrv906x_ndma_set_frame_size(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	u32 val;

	val = FIELD_PREP(NDMA_RX_MIN_FRAME_SIZE, NDMA_RX_FRAME_SIZE_DISABLED_MIN)
	      | FIELD_PREP(NDMA_RX_MAX_FRAME_SIZE, NDMA_RX_FRAME_SIZE_DISABLED_MAX);
	iowrite32(val, rx_chan->ctrl_base + NDMA_RX_FRAME_SIZE);

	val = FIELD_PREP(NDMA_TX_MIN_FRAME_SIZE, NDMA_TX_MIN_FRAME_SIZE_VALUE)
	      | FIELD_PREP(NDMA_TX_MAX_FRAME_SIZE, NDMA_MAX_FRAME_SIZE_VALUE);
	iowrite32(val, tx_chan->ctrl_base + NDMA_TX_FRAME_SIZE);
}

static void adrv906x_ndma_set_tx_timeout_value(struct adrv906x_ndma_dev *ndma_dev, u32 val)
{
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;

	iowrite32(val, tx_chan->ctrl_base + NDMA_TX_TIMEOUT_VALUE);
}

static void adrv906x_ndma_set_ptp_mode(struct adrv906x_ndma_dev *ndma_dev, u32 mode)
{
	struct adrv906x_ndma_reset *reset = &ndma_dev->reset;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&ndma_reset_lock, flags);
	val = ioread32(reset->reg);
	if (ndma_dev->dev_num == 0) {
		val &= ~NDMA_TX0_PTP_MODE;
		val |= FIELD_PREP(NDMA_TX0_PTP_MODE, mode);
	} else {
		val &= ~NDMA_TX1_PTP_MODE;
		val |= FIELD_PREP(NDMA_TX1_PTP_MODE, mode);
	}
	iowrite32(val, reset->reg);
	spin_unlock_irqrestore(&ndma_reset_lock, flags);
}

static void adrv906x_ndma_config_rx_ipv4_filter(struct adrv906x_ndma_chan *ndma_ch)
{
	iowrite32(NDMA_RX_IPV4_PTP_PROTO_VER_TYPE,
		  ndma_ch->ctrl_base + NDMA_RX_IPV4_FRAME_FIELD_VALUE0);
	iowrite32(NDMA_RX_IPV4_PTP_UDP_PORTS,
		  ndma_ch->ctrl_base + NDMA_RX_IPV4_FRAME_FIELD_VALUE1);
	iowrite32(NDMA_RX_IPV4_PTP_FIELD_OFFSETS0,
		  ndma_ch->ctrl_base + NDMA_RX_IPV4_FRAME_FIELD_OFFSET0);
	iowrite32(NDMA_RX_IPV4_PTP_MSG_TYPE_OFFSET,
		  ndma_ch->ctrl_base + NDMA_RX_IPV4_FRAME_FIELD_OFFSET1);
	iowrite32(NDMA_RX_IPV4_PTP_FIELD_MASKS0,
		  ndma_ch->ctrl_base + NDMA_RX_IPV4_FRAME_FIELD_MASK0);
	iowrite32(NDMA_RX_IPV4_PTP_MSG_TYPE_MASK,
		  ndma_ch->ctrl_base + NDMA_RX_IPV4_FRAME_FIELD_MASK1);
}

static void adrv906x_ndma_config_rx_ipv6_filter(struct adrv906x_ndma_chan *ndma_ch)
{
	iowrite32(NDMA_RX_IPV6_PTP_PROTO_VER_TYPE,
		  ndma_ch->ctrl_base + NDMA_RX_IPV6_FRAME_FIELD_VALUE0);
	iowrite32(NDMA_RX_IPV6_PTP_UDP_PORTS,
		  ndma_ch->ctrl_base + NDMA_RX_IPV6_FRAME_FIELD_VALUE1);
	iowrite32(NDMA_RX_IPV6_PTP_FIELD_OFFSETS0,
		  ndma_ch->ctrl_base + NDMA_RX_IPV6_FRAME_FIELD_OFFSET0);
	iowrite32(NDMA_RX_IPV6_PTP_MSG_TYPE_OFFSET,
		  ndma_ch->ctrl_base + NDMA_RX_IPV6_FRAME_FIELD_OFFSET1);
	iowrite32(NDMA_RX_IPV6_PTP_FIELD_MASKS0,
		  ndma_ch->ctrl_base + NDMA_RX_IPV6_FRAME_FIELD_MASK0);
	iowrite32(NDMA_RX_IPV6_PTP_MSG_TYPE_MASK,
		  ndma_ch->ctrl_base + NDMA_RX_IPV6_FRAME_FIELD_MASK1);
}

static void adrv906x_ndma_config_rx_eth_filter(struct adrv906x_ndma_chan *ndma_ch)
{
	iowrite32(NDMA_RX_ETH_PTP_ETHERTYPE,
		  ndma_ch->ctrl_base + NDMA_RX_ETH_FRAME_FIELD_VALUE);
	iowrite32(NDMA_RX_ETH_PTP_TYPE_MSG_OFFSETS,
		  ndma_ch->ctrl_base + NDMA_RX_ETH_FRAME_FIELD_OFFSET);
	iowrite32(NDMA_RX_ETH_PTP_MSG_TYPE_MASK,
		  ndma_ch->ctrl_base + NDMA_RX_ETH_FRAME_FIELD_MASK);
}

static void adrv906x_ndma_config_rx_splane_filter(struct adrv906x_ndma_chan *ndma_ch)
{
	iowrite32(NDMA_RX_SPLANE_PTP_MSG_TYPES,
		  ndma_ch->ctrl_base + NDMA_RX_SPLANE_FILTER_PTP_MSG_VALUE);
	iowrite32(NDMA_RX_SPLANE_VLAN_TAGS,
		  ndma_ch->ctrl_base + NDMA_RX_SPLANE_FILTER_VLAN_TAG_VALUE);
	iowrite32(NDMA_RX_SPLANE_VLAN_TAG_OFFSET,
		  ndma_ch->ctrl_base + NDMA_RX_SPLANE_FILTER_VLAN_TAG_OFFSET);
	iowrite32(NDMA_RX_SPLANE_VLAN_TAG_MASK,
		  ndma_ch->ctrl_base + NDMA_RX_SPLANE_FILTER_VLAN_TAG_MASK);
	iowrite32(NDMA_RX_SPLANE_VLAN_FRAME_OFFSET,
		  ndma_ch->ctrl_base + NDMA_RX_SPLANE_FILTER_VLAN_FRAME_OFFSET);
}

static void adrv906x_ndma_config_rx_ecpri_filter(struct adrv906x_ndma_chan *ndma_ch)
{
	u32 val, mask, nbytes;

	/* eCPRI "One-Way delay measurement" message to match:
	 *   byte 12: 0xae    
	 *   byte 13: 0xfe    
	 *   byte 15: 0x05    
	 *   byte 19: 0x00 or 0x01
	 */
	for (nbytes = 0; nbytes < NDMA_RX_ECPRI_FILTER_SIZE; nbytes += 4) {
		if (nbytes == 12) {
			val = NDMA_RX_ECPRI_FILTER_PATTERN_12;
			mask = NDMA_RX_ECPRI_FILTER_MASK_12;
		} else if (nbytes == 16) {
			val = NDMA_RX_ECPRI_FILTER_PATTERN_16;
			mask = NDMA_RX_ECPRI_FILTER_MASK_16;
		} else {
			val = 0;
			mask = NDMA_RX_ECPRI_FILTER_MASK_ANY;
		}

		iowrite32(val, ndma_ch->ctrl_base + NDMA_RX_GEN_FILTER0_REG_OFFSET +
			  NDMA_RX_CYCLE0_LOWER_VALUE_REG_OFFSET + nbytes);
		iowrite32(mask, ndma_ch->ctrl_base + NDMA_RX_GEN_FILTER0_REG_OFFSET +
			  NDMA_RX_CYCLE0_LOWER_MASK_REG_OFFSET + nbytes);
	}
}

static void adrv906x_ndma_enable_rx_filter(struct adrv906x_ndma_dev *ndma_dev,
					   u32 filter_en_mask,
					   enum adrv906x_ndma_rx_filter_status status)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	u32 val;

	val = ioread32(rx_chan->ctrl_base + NDMA_RX_SPLANE_FILTER_EN);

	if (status == NDMA_RX_FILTER_ON)
		val |= filter_en_mask;
	else
		val &= ~filter_en_mask;

	iowrite32(val, rx_chan->ctrl_base + NDMA_RX_SPLANE_FILTER_EN);
}

static void adrv906x_ndma_config_rx_filter(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	u32 en_mask;

	adrv906x_ndma_config_rx_ipv4_filter(rx_chan);
	adrv906x_ndma_config_rx_ipv6_filter(rx_chan);
	adrv906x_ndma_config_rx_eth_filter(rx_chan);
	adrv906x_ndma_config_rx_ecpri_filter(rx_chan);
	adrv906x_ndma_config_rx_splane_filter(rx_chan);

	en_mask = BIT(NDMA_RX_IPV4_FILTER)
		  | BIT(NDMA_RX_IPV6_FILTER)
		  | BIT(NDMA_RX_ETH_FILTER)
		  | BIT(NDMA_RX_GENERIC_FILTER_0);

	adrv906x_ndma_enable_rx_filter(ndma_dev, en_mask, NDMA_RX_FILTER_ON);
}

static int adrv906x_ndma_refill_rx(struct adrv906x_ndma_chan *ndma_ch, int budget)
{
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	struct device *dev = ndma_dev->dev;
	int end_desc_idx, done = 0;
	dma_addr_t addr, offset;
	struct sk_buff *skb;

	/* Get the index of the end descriptor in the list */
	end_desc_idx = (ndma_ch->rx_head + NDMA_RX_RING_SIZE - 1) % NDMA_RX_RING_SIZE;

	while (ndma_ch->rx_free < NDMA_RX_RING_SIZE) {
		skb = napi_alloc_skb(&ndma_ch->napi, NDMA_RX_WU_BUF_SIZE);
		if (!skb)
			break;

		/* Adjust the buffer alignment to 32 bytes */
		offset = (32 - ((dma_addr_t)skb->data & 0x1f)) & 0x1f;
		skb_reserve(skb, offset);
		/* Mark an empty buffer by setting the first byte of the WU header to 0 */
		skb->data[0] = 0;

		addr = dma_map_single(dev, skb->data, NDMA_RX_WU_BUF_SIZE, DMA_FROM_DEVICE);

		if (unlikely(dma_mapping_error(dev, addr))) {
			napi_consume_skb(skb, budget);
			break;
		}

		if (unlikely(addr & 0x1f)) {
			dma_unmap_single(dev, addr, NDMA_RX_WU_BUF_SIZE, DMA_FROM_DEVICE);
			napi_consume_skb(skb, budget);
			break;
		}

		ndma_ch->rx_buffs[ndma_ch->rx_head] = skb;
		ndma_ch->rx_ring[ndma_ch->rx_head].start = addr;
		ndma_ch->rx_ring[ndma_ch->rx_head].cfg |= DMAEN | DMAFLOW_LIST;
		ndma_ch->rx_head = (ndma_ch->rx_head + 1) % NDMA_RX_RING_SIZE;
		ndma_ch->rx_free++;
		done++;
	}

	if (done) {
		/* Clear previously set end of the descriptor list */
		ndma_ch->rx_ring[end_desc_idx].cfg |= DMAFLOW_LIST;
		/* Set new end of the descriptor list */
		ndma_ch->rx_ring[(end_desc_idx + done) % NDMA_RX_RING_SIZE].cfg &= ~DMAFLOW_LIST;
	}

	return done;
}

static int adrv906x_ndma_init_irqs(struct device_node *node, struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	struct device *dev = ndma_dev->dev;
	int irq, ret;

	/* Init for TX */
	irq = of_irq_get_byname(node, "tx_data_dma_done");
	if (irq <= 0) {
		dev_err(dev, "failed to get tx_data_dma_done interrupt");
		return irq ? irq : -ENOENT;
	}
	tx_chan->tx_dma_done_irq = irq;

	irq = of_irq_get_byname(node, "tx_data_dma_error");
	if (irq <= 0) {
		dev_err(dev, "failed to get tx_data_dma_error interrupt");
		return irq ? irq : -ENOENT;
	}
	tx_chan->tx_dma_error_irq = irq;

	ret = devm_request_threaded_irq(dev, tx_chan->tx_dma_error_irq,
					adrv906x_dma_error_irq_handler,
					adrv906x_dma_error_irq_handler_thread,
					IRQF_ONESHOT, dev_name(dev), tx_chan);
	if (ret) {
		dev_err(dev, "failed to request tx_dma_error_irq interrupt");
		return ret;
	}

	irq = of_irq_get_byname(node, "tx_status_dma_done");
	if (irq <= 0) {
		dev_err(dev, "failed to get tx_status_dma_done interrupt");
		return irq ? irq : -ENOENT;
	}
	tx_chan->rx_dma_done_irq = irq;

	irq = of_irq_get_byname(node, "tx_status_dma_error");
	if (irq <= 0) {
		dev_err(dev, "failed to get tx_status_dma_error interrupt");
		return irq ? irq : -ENOENT;
	}
	tx_chan->rx_dma_error_irq = irq;

	ret = devm_request_irq(dev, tx_chan->rx_dma_done_irq,
			       adrv906x_dma_rx_done_irq_handler, 0, dev_name(dev), tx_chan);
	if (ret) {
		dev_err(dev, "failed to request tx_status_dma_done interrupt");
		return ret;
	}

	ret = devm_request_threaded_irq(dev, tx_chan->rx_dma_error_irq,
					adrv906x_dma_error_irq_handler,
					adrv906x_dma_error_irq_handler_thread,
					IRQF_ONESHOT, dev_name(dev), tx_chan);
	if (ret) {
		dev_err(dev, "failed to request tx_status_dma_error interrupt");
		return ret;
	}

	/* Init for RX */
	irq = of_irq_get_byname(node, "rx_dma_done");
	if (irq <= 0) {
		dev_err(dev, "failed to get rx_dma_done interrupt");
		return irq ? irq : -ENOENT;
	}
	rx_chan->rx_dma_done_irq = irq;

	irq = of_irq_get_byname(node, "rx_dma_error");
	if (irq <= 0) {
		dev_err(dev, "failed to get rx_dma_error interrupt");
		return irq ? irq : -ENOENT;
	}
	rx_chan->rx_dma_error_irq = irq;

	ret = devm_request_irq(dev, rx_chan->rx_dma_done_irq,
			       adrv906x_dma_rx_done_irq_handler, 0, dev_name(dev), rx_chan);
	if (ret) {
		dev_err(dev, "failed to request rx_dma_done interrupt");
		return ret;
	}

	ret = devm_request_threaded_irq(dev, rx_chan->rx_dma_error_irq,
					adrv906x_dma_error_irq_handler,
					adrv906x_dma_error_irq_handler_thread,
					IRQF_ONESHOT, dev_name(dev), rx_chan);
	if (ret) {
		dev_err(dev, "failed to request rx_dma_error interrupt");
		return ret;
	}

	return 0;
}

static int adrv906x_ndma_get_reset_ctrl(struct adrv906x_ndma_dev *ndma_dev,
					struct device_node *ndma_np, bool switch_enabled)
{
	struct adrv906x_ndma_reset *reset = &ndma_dev->reset;
	struct device *dev = ndma_dev->dev;
	struct device_node *reset_np;
	u32 reg, len;
	int ret;

	reset_np = of_parse_phandle(ndma_np, "reset-ctrl", 0);
	if (!reset_np) {
		dev_err(dev, "missing reset-ctrl property");
		return -ENODEV;
	}
	ret = of_property_read_u32_index(reset_np, "reg", 0, &reg);
	if (ret) {
		dev_err(dev, "missing reg property of reset-ctrl node");
		return -EINVAL;
	}
	ret = of_property_read_u32_index(reset_np, "reg", 1, &len);
	if (ret) {
		dev_err(dev, "missing reg length of reset-ctrl node");
		return -EINVAL;
	}
	reset->reg = devm_ioremap(dev->parent, reg, len);
	if (!reset->reg) {
		dev_err(dev, "ioremap ndma-rst failed!");
		return -EINVAL;
	}

	if (switch_enabled) {
		reset->rx_chan_reset_bit = NDMA_RX0_RST | NDMA_RX1_RST;
		reset->tx_chan_reset_bit = NDMA_TX0_RST | NDMA_TX1_RST;
	} else {
		if (ndma_dev->dev_num == 0) {
			reset->rx_chan_reset_bit = NDMA_RX0_RST;
			reset->tx_chan_reset_bit = NDMA_TX0_RST;
		} else {
			reset->rx_chan_reset_bit = NDMA_RX1_RST;
			reset->tx_chan_reset_bit = NDMA_TX1_RST;
		}
	}

	return 0;
}

static int adrv906x_ndma_get_intr_ctrl(struct adrv906x_ndma_dev *ndma_dev,
				       struct device_node *ndma_np)
{
	struct device *dev = ndma_dev->dev;
	struct device_node *intr_ctrl;
	u32 reg, len;
	int ret;

	intr_ctrl = of_parse_phandle(ndma_np, "interrupt-ctrl", 0);
	if (!intr_ctrl) {
		dev_err(dev, "missing interrupt-ctrl property");
		return -ENODEV;
	}
	ret = of_property_read_u32_index(intr_ctrl, "reg", 0, &reg);
	if (ret) {
		dev_err(dev, "missing interrupt-ctrl node reg addr");
		return -EINVAL;
	}
	ret = of_property_read_u32_index(intr_ctrl, "reg", 1, &len);
	if (ret) {
		dev_err(dev, "missing interrupt-ctrl node reg length");
		return -EINVAL;
	}
	ndma_dev->intr_ctrl = devm_ioremap(dev->parent, reg, len);
	return 0;
}

void adrv906x_ndma_update_frame_drop_stats(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	union adrv906x_ndma_chan_stats *stats = &rx_chan->stats;
	u32 count;

	spin_lock(&ndma_dev->lock);

	if (!ndma_dev->enabled) {
		spin_unlock(&ndma_dev->lock);
		return;
	}

	count = ioread32(rx_chan->ctrl_base + NDMA_RX_FRAME_DROPPED_COUNT_SPLANE);
	if (count < (u32)stats->rx.frame_dropped_splane_errors)
		stats->rx.frame_dropped_splane_errors += BIT_ULL(32);
	stats->rx.frame_dropped_splane_errors &= GENMASK_ULL(63, 32);
	stats->rx.frame_dropped_splane_errors |= count;

	count = ioread32(rx_chan->ctrl_base + NDMA_RX_FRAME_DROPPED_COUNT_MPLANE);
	if (count < (u32)stats->rx.frame_dropped_mplane_errors)
		stats->rx.frame_dropped_mplane_errors += BIT_ULL(32);
	stats->rx.frame_dropped_mplane_errors &= GENMASK_ULL(63, 32);
	stats->rx.frame_dropped_mplane_errors |= count;

	stats->rx.frame_dropped_errors = stats->rx.frame_dropped_splane_errors
					 + stats->rx.frame_dropped_mplane_errors;

	spin_unlock(&ndma_dev->lock);
}

static void adrv906x_dma_tx_prep_desc_list(struct adrv906x_ndma_chan *ndma_ch)
{
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	struct dma_desc *tx_ring = ndma_ch->tx_ring;
	u32 num_of_desc, descs_processed = 0;
	u32 desc_indx = ndma_ch->tx_tail;
	struct sk_buff *skb;

	num_of_desc = ndma_dev->loopback_en ?
		      2 * ndma_ch->tx_frames_waiting : ndma_ch->tx_frames_waiting;
	while (num_of_desc) {
		tx_ring[desc_indx].cfg |= DMAFLOW_LIST;
		skb = ndma_ch->tx_buffs[desc_indx];
		desc_indx = (desc_indx + 1) % NDMA_TX_RING_SIZE;
		num_of_desc--;
		descs_processed++;

		/* Defer transmission of the next packet until the status
		 * for the PTP packet is received
		 */
		if (skb && FIELD_GET(NDMA_TX_HDR_SOF_FR_PTP, skb->data[0]))
			break;
	}

	ndma_ch->tx_frames_pending = ndma_dev->loopback_en ? descs_processed / 2 : descs_processed;
	ndma_ch->tx_frames_waiting -= ndma_ch->tx_frames_pending;
	desc_indx = (desc_indx) ? desc_indx - 1 : NDMA_TX_RING_SIZE - 1;
	tx_ring[desc_indx].cfg &= ~DMAFLOW_LIST;
}

static void adrv906x_ndma_reset_tx(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_reset *reset = &ndma_dev->reset;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&ndma_reset_lock, flags);
	val = ioread32(reset->reg);
	iowrite32(val & ~reset->tx_chan_reset_bit, reset->reg);
	iowrite32(val | reset->tx_chan_reset_bit, reset->reg);
	spin_unlock_irqrestore(&ndma_reset_lock, flags);
}

static void __maybe_unused adrv906x_ndma_reset_rx(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_reset *reset = &ndma_dev->reset;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&ndma_reset_lock, flags);
	val = ioread32(reset->reg);
	iowrite32(val & ~reset->rx_chan_reset_bit, reset->reg);
	iowrite32(val | reset->rx_chan_reset_bit, reset->reg);
	spin_unlock_irqrestore(&ndma_reset_lock, flags);
}

static void adrv906x_ndma_enable_events(struct adrv906x_ndma_chan *ndma_ch, unsigned int events)
{
	u32 val, offset;

	offset = (ndma_ch->chan_type == NDMA_RX_CHANNEL) ? NDMA_RX_EVENT_EN : NDMA_TX_EVENT_EN;

	val = ioread32(ndma_ch->ctrl_base + offset);
	val |= events;
	iowrite32(val, ndma_ch->ctrl_base + offset);
}

static void adrv906x_ndma_disable_all_event(struct adrv906x_ndma_chan *ndma_ch)
{
	u32 offset;

	offset = (ndma_ch->chan_type == NDMA_RX_CHANNEL) ? NDMA_RX_EVENT_EN : NDMA_TX_EVENT_EN;

	iowrite32(0, ndma_ch->ctrl_base + offset);
}

static void adrv906x_ndma_rx_free_data_wu_list(struct list_head *data_wu_list, int budget)
{
	struct sk_buff *skb, *next;

	list_for_each_entry_safe(skb, next, data_wu_list, list) {
		skb_list_del_init(skb);
		napi_consume_skb(skb, budget);
	}
}

static void adrv906x_ndma_rx_flood_evt_handler(struct work_struct *work)
{
	struct adrv906x_ndma_chan *ndma_ch =
		container_of(work, struct adrv906x_ndma_chan, rx_flood_mitigate_work);
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	struct adrv906x_ndma_flood_evt *evt;
	unsigned long flags;

	while (true) {
		spin_lock_irqsave(&ndma_ch->lock, flags);
		if (list_empty(&ndma_ch->rx_flood_evt_list)) {
			spin_unlock_irqrestore(&ndma_ch->lock, flags);
			break;
		}
		evt = list_first_entry(&ndma_ch->rx_flood_evt_list,
				       struct adrv906x_ndma_flood_evt, node);
		list_del(&evt->node);
		spin_unlock_irqrestore(&ndma_ch->lock, flags);

		if (ndma_dev->flood_cb_fn)
			ndma_dev->flood_cb_fn(ndma_dev->ndev, evt->mac, evt->port_id);

		kfree(evt);
	}
}

/* Invoked when the NDMA transmit channel enters freeze state - possibly after
 * a link toggle. This handler performs the following recovery steps:
 *
 * 1. Release all buffers owned by the driver that contain packets pending transmission.
 * 2. Execute four consecutive NDMA TX resets.
 * 3. Restore the NDMA TX configuration to its initial state.
 */
static void adrv906x_ndma_tx_recovery_handler(struct work_struct *work)
{
	struct adrv906x_ndma_chan *ndma_ch =
		container_of(to_delayed_work(work), struct adrv906x_ndma_chan,
			     tx_frames_timeout_work);
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	struct device *dev = ndma_dev->dev;
	unsigned long flags;
	struct sk_buff *skb;
	dma_addr_t addr;
	u32 size;
	u8 port;
	int i;

	spin_lock_irqsave(&ndma_ch->lock, flags);
	ndma_ch->stats.tx.recovery_count++;

	adrv906x_ndma_disable_all_event(ndma_ch);
	adrv906x_ndma_chan_disable(ndma_ch);
	adrv906x_ndma_disable_irqs(ndma_dev,
				   NDMA_TX_DATA_DMA_ERR_IRQ |
				   NDMA_TX_STATUS_DMA_ERR_IRQ |
				   NDMA_TX_STATUS_DMA_DONE_IRQ);

	for (i = 0; i < 4; i++)
		adrv906x_ndma_reset_tx(ndma_dev);

	while (ndma_ch->tx_frames_pending) {
		skb = ndma_ch->tx_buffs[ndma_ch->tx_tail];
		if (!skb) {
			ndma_ch->tx_tail = (ndma_ch->tx_tail + 1) % NDMA_TX_RING_SIZE;
			continue;
		}

		port = FIELD_GET(NDMA_TX_HDR_SOF_PORT_ID, skb->data[0]);
		addr = ndma_ch->tx_ring[ndma_ch->tx_tail].start;
		size = ndma_ch->tx_ring[ndma_ch->tx_tail].xcnt *
		       ndma_ch->tx_ring[ndma_ch->tx_tail].xmod;
		dma_unmap_single(dev, addr, size, DMA_TO_DEVICE);

		ndma_ch->tx_buffs[ndma_ch->tx_tail] = NULL;
		ndma_ch->tx_tail = (ndma_ch->tx_tail + 1) % NDMA_TX_RING_SIZE;

		if (FIELD_GET(NDMA_TX_HDR_SOF_FR_PTP, skb->data[0])) {
			if (ndma_ch->ptp_exp_seq_num < NDMA_TX_PTP_MAX_SEQNUM)
				ndma_ch->ptp_exp_seq_num++;
			else
				ndma_ch->ptp_exp_seq_num = NDMA_TX_PTP_MIN_SEQNUM;
		} else {
			if (ndma_ch->exp_seq_num < NDMA_TX_MAX_SEQNUM)
				ndma_ch->exp_seq_num++;
			else
				ndma_ch->exp_seq_num = NDMA_TX_MIN_SEQNUM;
		}

		ndma_ch->status_cb_fn(skb, port, NULL, ndma_ch->cb_param);
		ndma_ch->tx_frames_pending--;
	}

	adrv906x_ndma_enable_events(ndma_ch, NDMA_TX_ERROR_EVENTS);
	adrv906x_ndma_set_frame_size(ndma_dev);
	adrv906x_ndma_set_tx_timeout_value(ndma_dev, NDMA_TX_TS_DELAY);
	adrv906x_dma_rx_reset(ndma_ch);
	adrv906x_dma_tx_reset(ndma_ch);
	adrv906x_dma_rx_start(ndma_ch);
	adrv906x_ndma_enable_irqs(ndma_dev,
				  NDMA_TX_DATA_DMA_ERR_IRQ |
				  NDMA_TX_STATUS_DMA_ERR_IRQ |
				  NDMA_TX_STATUS_DMA_DONE_IRQ);

	adrv906x_ndma_chan_enable(ndma_ch);

	if (ndma_ch->tx_frames_waiting) {
		adrv906x_dma_tx_prep_desc_list(ndma_ch);
		adrv906x_dma_tx_start(ndma_ch);
	}

	spin_unlock_irqrestore(&ndma_ch->lock, flags);
}

static int adrv906x_ndma_device_init(struct adrv906x_ndma_dev *ndma_dev, struct device_node *np)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	struct device *dev = ndma_dev->dev;
	char wq_name[32];
	u32 reg, len;
	int ret;

	/* Config TX  */
	ret = of_property_read_u32_index(np, "reg", 0, &reg);
	if (ret)
		return ret;
	ret = of_property_read_u32_index(np, "reg", 1, &len);
	if (ret)
		return ret;
	tx_chan->ctrl_base = devm_ioremap(dev->parent, reg, len);
	ret = of_property_read_u32_index(np, "reg", 2, &reg);
	if (ret)
		return ret;
	ret = of_property_read_u32_index(np, "reg", 3, &len);
	if (ret)
		return ret;
	tx_chan->tx_dma_base = devm_ioremap(dev->parent, reg, len);
	ret = of_property_read_u32_index(np, "reg", 4, &reg);
	if (ret)
		return ret;
	ret = of_property_read_u32_index(np, "reg", 5, &len);
	if (ret)
		return ret;
	tx_chan->rx_dma_base = devm_ioremap(dev->parent, reg, len);
	tx_chan->chan_type = NDMA_TX_CHANNEL;
	tx_chan->parent = ndma_dev;

	/* Config RX  */
	ret = of_property_read_u32_index(np, "reg", 6, &reg);
	if (ret)
		return ret;
	ret = of_property_read_u32_index(np, "reg", 7, &len);
	if (ret)
		return ret;
	rx_chan->ctrl_base = devm_ioremap(dev->parent, reg, len);
	ret = of_property_read_u32_index(np, "reg", 8, &reg);
	if (ret)
		return ret;
	ret = of_property_read_u32_index(np, "reg", 9, &len);
	if (ret)
		return ret;
	rx_chan->rx_dma_base = devm_ioremap(dev->parent, reg, len);
	rx_chan->chan_type = NDMA_RX_CHANNEL;
	rx_chan->parent = ndma_dev;

	ret = adrv906x_ndma_init_irqs(np, ndma_dev);
	if (ret)
		return ret;

	spin_lock_init(&ndma_dev->lock);
	spin_lock_init(&tx_chan->lock);
	spin_lock_init(&rx_chan->lock);

	snprintf(wq_name, sizeof(wq_name), "adrv906x_ndma_wq_%s", dev_name(dev));
	ndma_dev->wq = alloc_workqueue(wq_name, WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!ndma_dev->wq)
		return -ENOMEM;

	INIT_WORK(&rx_chan->rx_flood_mitigate_work, adrv906x_ndma_rx_flood_evt_handler);
	INIT_DELAYED_WORK(&tx_chan->tx_frames_timeout_work, adrv906x_ndma_tx_recovery_handler);

	return ret;
}

static void adrv906x_ndma_add_tx_header(struct adrv906x_ndma_dev *ndma_dev, struct sk_buff *skb,
					u8 port, bool hw_tstamp_req, bool dsa_en)
{
	struct adrv906x_ndma_chan *ndma_ch = &ndma_dev->tx_chan;
	u32 frame_len;
	u8 *hdr;

	frame_len = ndma_dev->loopback_en ? skb->len + NDMA_TX_HDR_LOOPBACK_SIZE  : skb->len;
	hdr = skb_push(skb, NDMA_TX_HDR_SOF_SIZE);

	hdr[0] = FIELD_PREP(NDMA_HDR_TYPE_MASK, NDMA_TX_HDR_TYPE_SOF)
		 | FIELD_PREP(NDMA_TX_HDR_SOF_FR_PTP, hw_tstamp_req)
		 | FIELD_PREP(NDMA_TX_HDR_SOF_PORT_ID, port)
		 | FIELD_PREP(NDMA_TX_HDR_SOF_DSA_EN, dsa_en);
	hdr[1] = hw_tstamp_req ? ndma_ch->ptp_seq_num : ndma_ch->seq_num;
	hdr[2] = FIELD_PREP(NDMA_TX_HDR_SOF_FRAME_LEN_MASK, frame_len);
	hdr[3] = FIELD_PREP(NDMA_TX_HDR_SOF_FRAME_LEN_MASK, frame_len >> 8);
	hdr[4] = 0;
	hdr[5] = 0;
	hdr[6] = 0;
	hdr[7] = 0;
}

static void adrv906x_ndma_reset(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	unsigned long flags;

	adrv906x_ndma_reset_tx(ndma_dev);

	spin_lock_irqsave(&tx_chan->lock, flags);
	tx_chan->exp_seq_num = NDMA_TX_MIN_SEQNUM;
	tx_chan->seq_num = NDMA_TX_MIN_SEQNUM;
	tx_chan->ptp_exp_seq_num = NDMA_TX_PTP_MIN_SEQNUM;
	tx_chan->ptp_seq_num = NDMA_TX_PTP_MIN_SEQNUM;

	spin_unlock_irqrestore(&tx_chan->lock, flags);

	spin_lock_irqsave(&rx_chan->lock, flags);
	rx_chan->exp_seq_num = 0;
	spin_unlock_irqrestore(&rx_chan->lock, flags);
}

static int adrv906x_ndma_alloc_rings(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	struct device *dev = ndma_dev->dev;
	void *tx_status_buffs;
	dma_addr_t addr;
	int i;

	/* Allocate and initialize DMA descriptor ring for RX data & status */
	rx_chan->rx_ring = dmam_alloc_coherent(dev,
					       sizeof(struct dma_desc) * NDMA_RX_RING_SIZE,
					       &rx_chan->rx_ring_dma, GFP_KERNEL);
	if (!rx_chan->rx_ring)
		return -ENOMEM;

	for (i = 0; i < NDMA_RX_RING_SIZE; i++) {
		rx_chan->rx_ring[i].cfg = (DESCIDCPY | DI_EN_X | NDSIZE_4 | DMASYNC |
					   WDSIZE_256 | PSIZE_64 | WNR | DMAEN);
		rx_chan->rx_ring[i].cfg |=
			(i == NDMA_RX_RING_SIZE - 1) ? DMAFLOW_STOP : DMAFLOW_LIST;
		rx_chan->rx_ring[i].xcnt = NDMA_RX_WU_BUF_SIZE / XMODE_256;
		rx_chan->rx_ring[i].xmod = XMODE_256;
		rx_chan->rx_ring[i].next = rx_chan->rx_ring_dma +
					   sizeof(struct dma_desc) * ((i + 1) % NDMA_RX_RING_SIZE);
	}

	/* Allocate and initialize DMA descriptor ring for TX data */
	tx_chan->tx_ring = dmam_alloc_coherent(dev,
					       sizeof(struct dma_desc) * NDMA_TX_RING_SIZE,
					       &tx_chan->tx_ring_dma, GFP_KERNEL);
	if (!tx_chan->tx_ring)
		return -ENOMEM;

	for (i = 0; i < NDMA_TX_RING_SIZE; i++) {
		tx_chan->tx_ring[i].cfg = (DESCIDCPY | NDSIZE_4 | DMASYNC | PSIZE_64 | DMAEN);
		tx_chan->tx_ring[i].next = tx_chan->tx_ring_dma +
					   sizeof(struct dma_desc) * ((i + 1) % NDMA_TX_RING_SIZE);
	}

	/* Allocate and initialize DMA descriptor ring and buffers for TX status */
	tx_chan->rx_ring = dmam_alloc_coherent(dev,
					       sizeof(struct dma_desc) * NDMA_TX_RING_SIZE,
					       &tx_chan->rx_ring_dma, GFP_KERNEL);
	if (!tx_chan->rx_ring)
		return -ENOMEM;

	tx_status_buffs = dmam_alloc_coherent(dev, NDMA_TX_HDR_STATUS_SIZE * NDMA_TX_RING_SIZE,
					      &addr, GFP_KERNEL);
	if (!tx_status_buffs)
		return -ENOMEM;

	for (i = 0; i < NDMA_TX_RING_SIZE; i++) {
		tx_chan->rx_buffs[i] = tx_status_buffs + i * NDMA_TX_HDR_STATUS_SIZE;

		tx_chan->rx_ring[i].cfg = (DESCIDCPY | DI_EN_X | NDSIZE_4 | DMASYNC |
					   WDSIZE_64 | PSIZE_32 | WNR | DMAEN | DMAFLOW_LIST);
		tx_chan->rx_ring[i].xcnt = NDMA_TX_HDR_STATUS_SIZE / XMODE_64;
		tx_chan->rx_ring[i].xmod = XMODE_64;
		tx_chan->rx_ring[i].next = tx_chan->rx_ring_dma +
					   sizeof(struct dma_desc) * ((i + 1) % NDMA_TX_RING_SIZE);
		tx_chan->rx_ring[i].start = addr + i * NDMA_TX_HDR_STATUS_SIZE;
	}

	return 0;
}

void adrv906x_ndma_config_loopback(struct adrv906x_ndma_dev *ndma_dev, bool enable)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&tx_chan->lock, flags);

	if (ndma_dev->loopback_en == enable) {
		spin_unlock_irqrestore(&tx_chan->lock, flags);
		return;
	}

	ndma_dev->loopback_en = enable;

	val = ioread32(rx_chan->ctrl_base + NDMA_RX_STAT_AND_CTRL);
	if (enable) {
		val |= NDMA_LOOPBACK_EN;
		tx_chan->tx_loopback_wu[0] = NDMA_TX_HDR_TYPE_LOOPBACK;
		tx_chan->tx_loopback_addr = dma_map_single(ndma_dev->dev, tx_chan->tx_loopback_wu,
							   NDMA_TX_HDR_LOOPBACK_SIZE,
							   DMA_TO_DEVICE);
		tx_chan->tx_loopback_desc.cfg = (DESCIDCPY | DI_EN_X | NDSIZE_4 |
						 WDSIZE_8 | PSIZE_32 | DMAEN);
		tx_chan->tx_loopback_desc.xcnt = NDMA_TX_HDR_LOOPBACK_SIZE / XMODE_8;
		tx_chan->tx_loopback_desc.xmod = XMODE_8;
		tx_chan->tx_loopback_desc.start = tx_chan->tx_loopback_addr;
	} else {
		val &= ~NDMA_LOOPBACK_EN;

		dma_unmap_single(ndma_dev->dev, tx_chan->tx_loopback_addr,
				 NDMA_TX_HDR_LOOPBACK_SIZE, DMA_TO_DEVICE);
	}
	iowrite32(val, rx_chan->ctrl_base + NDMA_RX_STAT_AND_CTRL);

	spin_unlock_irqrestore(&tx_chan->lock, flags);
}

static u32 adrv906x_ndma_mac_hash(const u8 *mac)
{
	return jhash(mac, ETH_ALEN, 0);
}

static int adrv906x_ndma_add_mac(struct adrv906x_ndma_dev *ndma_dev, const u8 *mac)
{
	struct adrv906x_ndma_mac_entry *entry;

	if (!is_valid_ether_addr(mac))
		return -EADDRNOTAVAIL;

	entry = kmalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	memcpy(entry->mac, mac, ETH_ALEN);

	spin_lock(&ndma_dev->lock);
	if (ndma_dev->mac_count < NDMA_MAX_MACS) {
		hash_add_rcu(ndma_dev->mac_table, &entry->hnode, adrv906x_ndma_mac_hash(mac));
		ndma_dev->mac_count++;
	} else {
		kfree(entry);
		spin_unlock(&ndma_dev->lock);
		return -ENOMEM;
	}
	spin_unlock(&ndma_dev->lock);

	return 0;
}

static int adrv906x_ndma_remove_mac(struct adrv906x_ndma_dev *ndma_dev, const u8 *mac)
{
	struct adrv906x_ndma_mac_entry *entry;
	u32 key = adrv906x_ndma_mac_hash(mac);

	hash_for_each_possible_rcu(ndma_dev->mac_table, entry, hnode, key) {
		if (ether_addr_equal(entry->mac, mac)) {
			spin_lock(&ndma_dev->lock);
			hash_del_rcu(&entry->hnode);
			kfree_rcu(entry, rcu);
			ndma_dev->mac_count--;
			spin_unlock(&ndma_dev->lock);
			return 0;
		}
	}

	return -ENOENT;
}

static bool adrv906x_ndma_mac_exists_in_table(struct adrv906x_ndma_dev *ndma_dev, const u8 *mac)
{
	struct adrv906x_ndma_mac_entry *entry;
	u32 key = adrv906x_ndma_mac_hash(mac);
	bool found = false;

	rcu_read_lock();
	hash_for_each_possible_rcu(ndma_dev->mac_table, entry, hnode, key) {
		if (ether_addr_equal(entry->mac, mac)) {
			found = true;
			break;
		}
	}
	rcu_read_unlock();
	return found;
}

static void adrv906x_ndma_clear_mac_table(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_mac_entry *entry;
	struct hlist_node *tmp;
	int bkt;

	hash_for_each_safe(ndma_dev->mac_table, bkt, tmp, entry, hnode) {
		hash_del(&entry->hnode);
		kfree_rcu(entry, rcu);
	}
}

void adrv906x_ndma_open(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	struct net_device *ndev = (struct net_device *)tx_chan->cb_param;
	unsigned long flags0, flags1;

	spin_lock_irqsave(&ndma_dev->lock, flags0);
	if (!ndma_dev->enabled) {
		adrv906x_ndma_disable_all_irqs(ndma_dev);
		adrv906x_ndma_enable_events(rx_chan, NDMA_RX_ERROR_EVENTS);
		adrv906x_ndma_enable_events(tx_chan, NDMA_TX_ERROR_EVENTS);

		adrv906x_ndma_config_rx_filter(ndma_dev);
		adrv906x_ndma_set_frame_size(ndma_dev);
		adrv906x_ndma_set_tx_timeout_value(ndma_dev, NDMA_TX_TS_DELAY);

		spin_lock_irqsave(&tx_chan->lock, flags1);
		adrv906x_dma_rx_reset(tx_chan);
		adrv906x_dma_tx_reset(tx_chan);
		adrv906x_ndma_chan_enable(tx_chan);
		spin_unlock_irqrestore(&tx_chan->lock, flags1);

		spin_lock_irqsave(&rx_chan->lock, flags1);
		adrv906x_dma_rx_reset(rx_chan);
		adrv906x_dma_tx_reset(rx_chan);
		adrv906x_ndma_chan_enable(rx_chan);
		spin_unlock_irqrestore(&rx_chan->lock, flags1);

		tx_chan->rx_head = 0;
		tx_chan->rx_tail = 0;
		tx_chan->rx_free = 0;
		tx_chan->tx_head = 0;
		tx_chan->tx_tail = 0;
		tx_chan->tx_frames_waiting = 0;
		tx_chan->tx_frames_pending = 0;
		tx_chan->exp_seq_num = NDMA_TX_MIN_SEQNUM;
		tx_chan->seq_num = NDMA_TX_MIN_SEQNUM;
		tx_chan->ptp_exp_seq_num = NDMA_TX_PTP_MIN_SEQNUM;
		tx_chan->ptp_seq_num = NDMA_TX_PTP_MIN_SEQNUM;
		adrv906x_dma_rx_start(tx_chan);
		napi_enable(&tx_chan->napi);

		rx_chan->exp_seq_num = 0;
		rx_chan->rx_head = 0;
		rx_chan->rx_tail = 0;
		rx_chan->rx_free = 0;
		adrv906x_ndma_refill_rx(rx_chan, NDMA_RX_RING_SIZE);
		adrv906x_dma_rx_start(rx_chan);
		napi_enable(&rx_chan->napi);

		adrv906x_ndma_set_ptp_mode(ndma_dev, NDMA_PTP_MODE_1);
		adrv906x_ndma_enable_irqs(ndma_dev,
					  NDMA_RX_DMA_ERR_IRQ |
					  NDMA_RX_DMA_DONE_IRQ |
					  NDMA_TX_DATA_DMA_ERR_IRQ |
					  NDMA_TX_STATUS_DMA_ERR_IRQ |
					  NDMA_TX_STATUS_DMA_DONE_IRQ);

		ndma_dev->ndev = ndev;
		ndma_dev->enabled = true;
		kref_init(&ndma_dev->refcount);
	} else {
		kref_get(&ndma_dev->refcount);
	}

	spin_unlock_irqrestore(&ndma_dev->lock, flags0);

	/* Add the net device MAC address to the MAC filter list */
	adrv906x_ndma_add_mac(ndma_dev, ndev->dev_addr);
}

static void adrv906x_ndma_stop(struct kref *ref)
{
	struct adrv906x_ndma_dev *ndma_dev = container_of(ref, struct adrv906x_ndma_dev, refcount);
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	struct device *dev = ndma_dev->dev;
	u32 size, num_frames;
	unsigned long flags;
	struct sk_buff *skb;
	dma_addr_t addr;
	u8 port;

	spin_lock_irqsave(&ndma_dev->lock, flags);
	adrv906x_ndma_disable_all_irqs(ndma_dev);
	spin_unlock_irqrestore(&ndma_dev->lock, flags);

	/* Disable ndma RX channel */
	napi_disable(&rx_chan->napi);
	spin_lock_irqsave(&rx_chan->lock, flags);
	adrv906x_ndma_disable_all_event(rx_chan);
	adrv906x_ndma_chan_disable(rx_chan);

	adrv906x_ndma_rx_free_data_wu_list(&rx_chan->rx_data_wu_list, 0);
	while (rx_chan->rx_free) {
		skb = (struct sk_buff *)rx_chan->rx_buffs[rx_chan->rx_tail];
		addr = rx_chan->rx_ring[rx_chan->rx_tail].start;

		dma_unmap_single(dev, addr, NDMA_RX_WU_BUF_SIZE, DMA_FROM_DEVICE);
		dev_kfree_skb(skb);

		rx_chan->rx_tail = (rx_chan->rx_tail + 1) % NDMA_RX_RING_SIZE;
		rx_chan->rx_free--;
	}
	spin_unlock_irqrestore(&rx_chan->lock, flags);

	/* Disable ndma TX channel */
	napi_disable(&tx_chan->napi);
	spin_lock_irqsave(&tx_chan->lock, flags);
	adrv906x_ndma_disable_all_event(tx_chan);
	adrv906x_ndma_chan_disable(tx_chan);

	num_frames = tx_chan->tx_frames_waiting + tx_chan->tx_frames_pending;
	while (num_frames--) {
		skb = tx_chan->tx_buffs[tx_chan->tx_tail];
		if (!skb) {
			tx_chan->tx_tail = (tx_chan->tx_tail + 1) % NDMA_TX_RING_SIZE;
			continue;
		}

		port = FIELD_GET(NDMA_TX_HDR_SOF_PORT_ID, skb->data[0]);
		addr = tx_chan->tx_ring[tx_chan->tx_tail].start;
		size = tx_chan->tx_ring[tx_chan->tx_tail].xcnt *
		       tx_chan->tx_ring[tx_chan->tx_tail].xmod;
		dma_unmap_single(dev, addr, size, DMA_TO_DEVICE);

		tx_chan->tx_buffs[tx_chan->tx_tail] = NULL;
		tx_chan->tx_tail = (tx_chan->tx_tail + 1) % NDMA_TX_RING_SIZE;
		tx_chan->status_cb_fn(skb, port, NULL, tx_chan->cb_param);
	}
	tx_chan->tx_frames_pending = 0;
	tx_chan->tx_frames_waiting = 0;
	spin_unlock_irqrestore(&tx_chan->lock, flags);

	adrv906x_ndma_reset(ndma_dev);
	ndma_dev->enabled = false;
}

void adrv906x_ndma_close(struct adrv906x_ndma_dev *ndma_dev, struct net_device *ndev)
{
	kref_put(&ndma_dev->refcount, adrv906x_ndma_stop);

	adrv906x_ndma_remove_mac(ndma_dev, ndev->dev_addr);
}

static int adrv906x_ndma_parse_rx_status_header(struct adrv906x_ndma_chan *ndma_ch,
						u8 *status_hdr, struct timespec64 *ts, u32 *port_id,
						u32 *frame_size)
{
	union adrv906x_ndma_chan_stats *stats = &ndma_ch->stats;
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	struct device *dev __maybe_unused = ndma_dev->dev;
	int ret = NDMA_NO_ERROR;
	u32 error;

	get_ts_from_status(status_hdr, ts);
	*port_id = FIELD_GET(NDMA_RX_HDR_STATUS_PORT_ID, status_hdr[0]);
	*frame_size = get_unaligned_le16(&status_hdr[NDMA_RX_FRAME_LEN_LSB]);

	if (NDMA_RX_HDR_STATUS_FR_ERR & status_hdr[0]) {
		error = ioread32(ndma_ch->ctrl_base + NDMA_RX_EVENT_STAT) & NDMA_RX_ERROR_EVENTS;

		/* Get error type from IRQ status register and update statistics.
		 * Note: More than one error bit in IRQ status register can be set,
		 * so to avoid losing them, we clear only one error bit at a time.
		 */
		if (NDMA_RX_FRAME_SIZE_ERR_EVENT & error) {
			dev_dbg(dev, "frame size error");
			stats->rx.frame_size_errors++;
			if (NDMA_RX_HDR_STATUS_FR_DROP_ERR & status_hdr[0])
				dev_dbg(dev, "partial frame dropped error");
			iowrite32(NDMA_RX_FRAME_SIZE_ERR_EVENT,
				  ndma_ch->ctrl_base + NDMA_RX_EVENT_STAT);
			ret = NDMA_RX_FRAME_SIZE_ERR_EVENT;
		} else if (NDMA_RX_ERR_EVENT & error) {
			dev_dbg(dev, "mac error(s) signaled by tuser[0]");
			stats->rx.frame_errors++;
			iowrite32(NDMA_RX_ERR_EVENT, ndma_ch->ctrl_base + NDMA_RX_EVENT_STAT);
			ret = NDMA_RX_ERR_EVENT;
		} else if (NDMA_RX_FRAME_DROPPED_ERR_EVENT & error) {
			dev_dbg(dev, "frame dropped error");
			iowrite32(NDMA_RX_FRAME_DROPPED_ERR_EVENT,
				  ndma_ch->ctrl_base + NDMA_RX_EVENT_STAT);
			ret = NDMA_RX_FRAME_DROPPED_ERR_EVENT;
		} else {
			dev_dbg(dev, "status wu has error flag set but no interrupt generated");
			stats->rx.unknown_errors++;
			ret = NDMA_RX_UNKNOWN_ERROR;
		}
	} else {
		/* If no error, check and update sequence number */
		if (status_hdr[1] != ndma_ch->exp_seq_num) {
			dev_dbg(dev, "frame seq number mismatch, exp:0x%x recv:0x%x",
				ndma_ch->exp_seq_num, status_hdr[1]);
			stats->rx.seqnumb_mismatch_errors++;
			ndma_ch->exp_seq_num = status_hdr[1];
			ret = NDMA_RX_SEQNUM_MISMATCH_ERROR;
		}
		ndma_ch->exp_seq_num++;
	}

	return ret;
}

static int adrv906x_ndma_rx_validate_data_wu_list(struct list_head *data_wu_list,
						  int frame_size)
{
	int n_elem_exp, n_elem = 0;
	struct list_head *pos;

	n_elem_exp = DIV_ROUND_UP(frame_size, NDMA_RX_PKT_BUF_SIZE);

	list_for_each(pos, data_wu_list) {
		n_elem++;
	}

	return (n_elem_exp != n_elem) ? -EINVAL : 0;
}

static struct sk_buff *adrv906x_ndma_rx_build_linear_pkt_buf(struct list_head *data_wu_list,
							     u32 frame_size)
{
	struct sk_buff *frag, *skb = NULL;
	struct list_head *pos;
	u32 length;

	if (frame_size > NDMA_MAX_FRAME_SIZE_VALUE || frame_size < NDMA_RX_MIN_FRAME_SIZE_VALUE)
		goto out;

	if (adrv906x_ndma_rx_validate_data_wu_list(data_wu_list, frame_size))
		goto out;

	/* If the data work unit list contains only one entry, we avoid allocating a new buffer.
	 * Instead, we hold a reference to the existing buffer to prevent it from being freed
	 * by adrv906x_ndma_rx_free_data_wu_list().
	 */
	if (list_is_singular(data_wu_list)) {
		skb = list_first_entry(data_wu_list, struct sk_buff, list);
		skb_put(skb, NDMA_RX_HDR_DATA_SIZE + frame_size);
		skb_pull(skb, NDMA_RX_HDR_DATA_SIZE);
		skb_get(skb);
	} else {
		skb = alloc_skb(frame_size, GFP_ATOMIC);
		if (!skb)
			goto out;
		list_for_each(pos, data_wu_list) {
			frag = list_entry(pos, struct sk_buff, list);
			length = (frame_size > NDMA_RX_PKT_BUF_SIZE) ?
				 NDMA_RX_PKT_BUF_SIZE : frame_size;
			frame_size -= length;
			skb_put(frag, NDMA_RX_HDR_DATA_SIZE + length);
			skb_pull(frag, NDMA_RX_HDR_DATA_SIZE);
			skb_copy_from_linear_data(frag, skb_tail_pointer(skb), length);
			skb_put(skb, length);
		}
	}
out:
	return skb;
}

static bool adrv906x_ndma_mac_filter_match(struct adrv906x_ndma_dev *ndma_dev,
					   int port_id)
{
	struct adrv906x_ndma_chan *ndma_ch = &ndma_dev->rx_chan;
	struct sk_buff *skb;
	struct ethhdr *eth;
	u64 mac;

	if (!ndma_dev->flood_mitigate_en)
		return true;

	skb = list_first_entry(&ndma_ch->rx_data_wu_list, struct sk_buff, list);
	eth = (struct ethhdr *)(skb->data + NDMA_RX_HDR_DATA_SIZE);

	if (!adrv906x_ndma_mac_exists_in_table(ndma_dev, eth->h_dest) &&
	    !is_multicast_ether_addr(eth->h_dest)) {
		mac = ether_addr_to_u64(eth->h_dest);

		if (ndma_dev->flood_cb_fn) {
			struct adrv906x_ndma_flood_evt *evt;

			evt = kmalloc(sizeof(*evt), GFP_ATOMIC);
			if (evt) {
				evt->mac = mac;
				evt->port_id = port_id;

				list_add_tail(&evt->node, &ndma_ch->rx_flood_evt_list);
				queue_work(ndma_dev->wq, &ndma_ch->rx_flood_mitigate_work);
			}
		}
		ndma_ch->stats.rx.flooded_frame_count++;
		return false;
	}
	return true;
}

static void adrv906x_ndma_process_rx_work_unit(struct adrv906x_ndma_chan *rx_chan,
					       struct sk_buff *skb, int budget)
{
	struct adrv906x_ndma_dev *ndma_dev = rx_chan->parent;
	union adrv906x_ndma_chan_stats *stats = &rx_chan->stats;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	struct device *dev __maybe_unused = ndma_dev->dev;
	u32 port_id = 0, frame_size = 0, hdr_type;
	struct timespec64 ts = { 0, 0 };
	struct sk_buff *pktbuf;
	unsigned long flags;
	int ret;

	hdr_type = FIELD_GET(NDMA_HDR_TYPE_MASK, skb->data[0]);

	switch (hdr_type) {
	case NDMA_RX_HDR_TYPE_STATUS:
		if (list_empty(&rx_chan->rx_data_wu_list)) {
			dev_dbg(dev, "status received without preceding data work units");
		} else {
			ret = adrv906x_ndma_parse_rx_status_header(rx_chan, skb->data, &ts,
								   &port_id, &frame_size);

			if (ret != NDMA_NO_ERROR && ret != NDMA_RX_SEQNUM_MISMATCH_ERROR &&
			    unlikely(!ndma_dev->loopback_en))
				goto consume;

			if (!adrv906x_ndma_mac_filter_match(ndma_dev, port_id))
				goto consume;

			pktbuf = adrv906x_ndma_rx_build_linear_pkt_buf(&rx_chan->rx_data_wu_list,
								       frame_size);
			if (!pktbuf)
				goto consume;

			rx_chan->status_cb_fn(pktbuf, port_id, &ts, rx_chan->cb_param);

			spin_lock_irqsave(&tx_chan->lock, flags);
			if (port_id == 0)
				tx_chan->tx_block_timestamp_req_port0 = false;
			else
				tx_chan->tx_block_timestamp_req_port1 = false;
			spin_unlock_irqrestore(&tx_chan->lock, flags);
		}
consume:
		adrv906x_ndma_rx_free_data_wu_list(&rx_chan->rx_data_wu_list, budget);
		napi_consume_skb(skb, budget); /* Free skb with status WU */
		break;
	case NDMA_RX_HDR_TYPE_DATA:
		if (FIELD_GET(NDMA_RX_HDR_TYPE_DATA_SOF, skb->data[0])) {
			if (!list_empty(&rx_chan->rx_data_wu_list)) {
				dev_dbg(dev, "no status received for previous data work units");
				adrv906x_ndma_rx_free_data_wu_list(&rx_chan->rx_data_wu_list,
								   budget);
			}
			list_add_tail(&skb->list, &rx_chan->rx_data_wu_list);
		} else {
			if (list_empty(&rx_chan->rx_data_wu_list)) {
				dev_dbg(dev, "start of frame not detected");
				napi_consume_skb(skb, budget);
			} else {
				list_add_tail(&skb->list, &rx_chan->rx_data_wu_list);
			}
		}
		break;
	default:
		dev_dbg(dev, "incorrect wu header detected");
		adrv906x_ndma_rx_free_data_wu_list(&rx_chan->rx_data_wu_list, budget);
		napi_consume_skb(skb, budget);
		stats->rx.wu_header_errors++;
	}

	stats->rx.done_work_units++;
}

static int adrv906x_ndma_parse_tx_status_header(struct adrv906x_ndma_chan *ndma_ch,
						u8 *status_hdr, struct timespec64 *ts)
{
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	union adrv906x_ndma_chan_stats *stats = &ndma_ch->stats;
	struct device *dev __maybe_unused = ndma_dev->dev;
	int ret = NDMA_NO_ERROR;
	u32 val;

	if (FIELD_GET(NDMA_HDR_TYPE_MASK, status_hdr[0]) != NDMA_TX_HDR_TYPE_STATUS) {
		dev_dbg(dev, "incorrect format of wu status header: 0x%x", status_hdr[0]);
		stats->tx.wu_status_header_errors++;
		ret = NDMA_TX_STATUS_HEADER_ERROR;
	} else {
		get_ts_from_status(status_hdr, ts);

		if (NDMA_TX_HDR_STATUS_FR_ERR & status_hdr[0]) {
			if (adrv906x_ndma_chan_enabled(ndma_ch) &&
			    is_timestamp_all_zero(status_hdr)) {
				dev_dbg(dev, "hw timestamp timeout error");
				stats->tx.tstamp_timeout_errors++;
				ret = NDMA_TX_TSTAMP_TIMEOUT_ERROR;
			} else {
				/* Get error type from IRQ status register and update statistics.
				 * Note: More than one error bit in IRQ status register can be set,
				 * so to avoid losing them, we clear only one error bit at a time.
				 */
				val = ioread32(ndma_ch->ctrl_base + NDMA_TX_EVENT_STAT) &
					NDMA_TX_ERROR_EVENTS;

				if (NDMA_TX_FRAME_SIZE_ERR_EVENT & val) {
					dev_dbg(dev, "frame size error");
					stats->tx.frame_size_errors++;
					iowrite32(NDMA_TX_FRAME_SIZE_ERR_EVENT, ndma_ch->ctrl_base +
						  NDMA_TX_EVENT_STAT);
					ret = NDMA_TX_FRAME_SIZE_ERROR;
				} else if (NDMA_TX_WU_HEADER_ERR_EVENT & val) {
					dev_dbg(dev, "incorrect format of wu data header");
					stats->tx.wu_data_header_errors++;
					iowrite32(NDMA_TX_WU_HEADER_ERR_EVENT, ndma_ch->ctrl_base +
						  NDMA_TX_EVENT_STAT);
					ret = NDMA_TX_DATA_HEADER_ERROR;
				} else {
					dev_dbg(dev, "unknown error: status register does not indicate an error");
					stats->tx.unknown_errors++;
					ret = NDMA_RX_UNKNOWN_ERROR;
				}
			}
		}

		if (ret == NDMA_TX_TSTAMP_TIMEOUT_ERROR || !is_timestamp_all_zero(status_hdr)) {
			/* If a timestamp timeout occurs or the timestamp is non-zero,
			 * this indicates a PTP packet, so we need to verify its sequence number.
			 */
			if (status_hdr[1] != ndma_ch->ptp_exp_seq_num) {
				dev_dbg(dev, "tx ptp pkt seq number mismatch, exp:0x%x recv:0x%x",
					ndma_ch->ptp_exp_seq_num, status_hdr[1]);
				stats->tx.seqnumb_mismatch_errors++;
				ndma_ch->ptp_exp_seq_num = status_hdr[1];
				if (ret != NDMA_TX_TSTAMP_TIMEOUT_ERROR)
					ret = NDMA_TX_SEQNUM_MISMATCH_ERROR;
			}

			if (ndma_ch->ptp_exp_seq_num < NDMA_TX_PTP_MAX_SEQNUM)
				ndma_ch->ptp_exp_seq_num++;
			else
				ndma_ch->ptp_exp_seq_num = NDMA_TX_PTP_MIN_SEQNUM;
		} else {
			/* Validate sequence number for non-PTP packets */
			if (status_hdr[1] != ndma_ch->exp_seq_num) {
				dev_dbg(dev, "tx pkt seq number mismatch, exp:0x%x recv:0x%x",
					ndma_ch->exp_seq_num, status_hdr[1]);
				stats->tx.seqnumb_mismatch_errors++;
				ndma_ch->exp_seq_num = status_hdr[1];
				ret = NDMA_TX_SEQNUM_MISMATCH_ERROR;
			}

			if (ndma_ch->exp_seq_num < NDMA_TX_MAX_SEQNUM)
				ndma_ch->exp_seq_num++;
			else
				ndma_ch->exp_seq_num = NDMA_TX_MIN_SEQNUM;
		}
	}

	stats->tx.pending_work_units = ndma_ch->tx_frames_pending;
	stats->tx.done_work_units++;

	return ret;
}

static int adrv906x_ndma_process_tx_status(struct adrv906x_ndma_chan *ndma_ch, u8 *status)
{
	union adrv906x_ndma_chan_stats *stats = &ndma_ch->stats;
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	struct dma_desc *tx_ring = ndma_ch->tx_ring;
	struct device *dev = ndma_dev->dev;
	struct timespec64 ts = { 0, 0 };
	bool invalid_ts = false;
	struct sk_buff *skb;
	dma_addr_t addr;
	u32 size;
	u8 port;
	int ret;

	ret = adrv906x_ndma_parse_tx_status_header(ndma_ch, status, &ts);
	/* Re-enable ndma channel after error */
	if (ret)
		adrv906x_ndma_chan_enable(ndma_ch);

	skb = ndma_ch->tx_buffs[ndma_ch->tx_tail];
	port = FIELD_GET(NDMA_TX_HDR_SOF_PORT_ID, skb->data[0]);
	addr = tx_ring[ndma_ch->tx_tail].start;
	size = tx_ring[ndma_ch->tx_tail].xcnt * tx_ring[ndma_ch->tx_tail].xmod;
	dma_unmap_single(dev, addr, size, DMA_TO_DEVICE);

	/* After two timestamp timeouts in a row, block new timestamp requests.
	 * This usually happens after a link drop, and the hardware is sensitive
	 * to this condition. Keep blocking until RX traffic is detected.
	 */
	if (ret == NDMA_TX_TSTAMP_TIMEOUT_ERROR) {
		if (port == 0) {
			if (++ndma_ch->tx_timestamp_timeout_cnt_port0 >
			    NDMA_TX_TSTAMP_TIMEOUT_CNT_THRESHOLD)
				ndma_ch->tx_block_timestamp_req_port0 = true;
		} else if (port == 1) {
			if (++ndma_ch->tx_timestamp_timeout_cnt_port1 >
			    NDMA_TX_TSTAMP_TIMEOUT_CNT_THRESHOLD)
				ndma_ch->tx_block_timestamp_req_port1 = true;
		}

		invalid_ts = true;
	} else if (ts.tv_sec != 0 || ts.tv_nsec != 0) {
		/* The first timestamp after a series of timeouts is not valid */
		if (port == 0 && ndma_ch->tx_timestamp_timeout_cnt_port0 > 0) {
			ndma_ch->tx_timestamp_timeout_cnt_port0 = 0;
			invalid_ts = true;
		} else if (port == 1 && ndma_ch->tx_timestamp_timeout_cnt_port1 > 0) {
			ndma_ch->tx_timestamp_timeout_cnt_port1 = 0;
			invalid_ts = true;
		}
	}

	ndma_ch->tx_buffs[ndma_ch->tx_tail] = NULL;
	if (ndma_dev->loopback_en)
		ndma_ch->tx_tail = (ndma_ch->tx_tail + 2) % NDMA_TX_RING_SIZE;
	else
		ndma_ch->tx_tail = (ndma_ch->tx_tail + 1) % NDMA_TX_RING_SIZE;

	ndma_ch->status_cb_fn(skb, port, invalid_ts ? NULL : &ts, ndma_ch->cb_param);

	if (--ndma_ch->tx_frames_pending == 0) {
		cancel_delayed_work(&ndma_ch->tx_frames_timeout_work);

		if (ndma_ch->tx_frames_waiting) {
			adrv906x_dma_tx_prep_desc_list(ndma_ch);
			adrv906x_dma_tx_start(ndma_ch);
		}
	}

	stats->tx.pending_work_units = ndma_ch->tx_frames_pending;

	return ret;
}

int adrv906x_ndma_start_xmit(struct adrv906x_ndma_dev *ndma_dev, struct sk_buff *skb,
			     u8 port, bool hw_tstamp_req, bool dsa_en)
{
	u32 tx_frames_max_num, needed_tailroom, size, wdsize, xmod;
	struct adrv906x_ndma_chan *ndma_ch = &ndma_dev->tx_chan;
	union adrv906x_ndma_chan_stats *stats = &ndma_ch->stats;
	struct device *dev = ndma_dev->dev;
	unsigned long flags;
	dma_addr_t addr;
	int ret = 0;

	spin_lock_irqsave(&ndma_ch->lock, flags);
	tx_frames_max_num = ndma_dev->loopback_en ? NDMA_TX_RING_SIZE / 2 : NDMA_TX_RING_SIZE;
	if (ndma_ch->tx_frames_waiting + ndma_ch->tx_frames_pending >= tx_frames_max_num) {
		ret = -EBUSY;
		goto out;
	}

	if (skb->len < NDMA_TX_MIN_FRAME_SIZE_VALUE) {
		needed_tailroom = NDMA_TX_MIN_FRAME_SIZE_VALUE - skb->len;

		if (unlikely(skb_tailroom(skb) < needed_tailroom)) {
			ret = pskb_expand_head(skb, 0,
					       needed_tailroom - skb_tailroom(skb),
					       GFP_ATOMIC);
			if (ret) {
				dev_kfree_skb(skb); /* Drop the packet if expansion fails */
				ret = 0;
				goto out;
			}
		}

		skb_put(skb, needed_tailroom);
	}

	if ((ndma_ch->tx_block_timestamp_req_port0 && port == 0) ||
	    (ndma_ch->tx_block_timestamp_req_port1 && port == 1))
		hw_tstamp_req = false;

	adrv906x_ndma_add_tx_header(ndma_dev, skb, port, hw_tstamp_req, dsa_en);

	size = ALIGN(skb->len, 8);
	addr = dma_map_single(dev, skb->data, size, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, addr))) {
		ret = -EINVAL;
		goto out;
	}

	if (addr % 8 == 0) {
		wdsize = WDSIZE_64;
		xmod = XMODE_64;
	} else if (addr % 4 == 0) {
		wdsize = WDSIZE_32;
		xmod = XMODE_32;
	} else if (addr % 2 == 0) {
		wdsize = WDSIZE_16;
		xmod = XMODE_16;
	} else {
		wdsize = WDSIZE_8;
		xmod = XMODE_8;
	}

	ndma_ch->tx_buffs[ndma_ch->tx_head] = skb;
	ndma_ch->tx_ring[ndma_ch->tx_head].start = addr;
	ndma_ch->tx_ring[ndma_ch->tx_head].xmod = xmod;
	ndma_ch->tx_ring[ndma_ch->tx_head].xcnt = size / xmod;
	ndma_ch->tx_ring[ndma_ch->tx_head].cfg &= ~WDSIZE_MSK;
	ndma_ch->tx_ring[ndma_ch->tx_head].cfg |= wdsize;
	ndma_ch->tx_head = (ndma_ch->tx_head + 1) % NDMA_TX_RING_SIZE;
	if (ndma_dev->loopback_en) {
		ndma_ch->tx_buffs[ndma_ch->tx_head] = NULL;
		ndma_ch->tx_ring[ndma_ch->tx_head] = ndma_ch->tx_loopback_desc;
		ndma_ch->tx_head = (ndma_ch->tx_head + 1) % NDMA_TX_RING_SIZE;
	}
	ndma_ch->tx_frames_waiting++;

	if (hw_tstamp_req)
		ndma_ch->ptp_seq_num = (ndma_ch->ptp_seq_num < NDMA_TX_PTP_MAX_SEQNUM) ?
				       ndma_ch->ptp_seq_num + 1 : NDMA_TX_PTP_MIN_SEQNUM;
	else
		ndma_ch->seq_num = (ndma_ch->seq_num < NDMA_TX_MAX_SEQNUM) ?
				   ndma_ch->seq_num + 1 : NDMA_TX_MIN_SEQNUM;

	if (!ndma_ch->tx_frames_pending) {
		adrv906x_dma_tx_prep_desc_list(ndma_ch);
		adrv906x_dma_tx_start(ndma_ch);
	}

	stats->tx.pending_work_units = ndma_ch->tx_frames_pending;
out:
	spin_unlock_irqrestore(&ndma_ch->lock, flags);
	return ret;
}

static int adrv906x_ndma_tx_status_poll(struct napi_struct *napi, int budget)
{
	struct adrv906x_ndma_chan *ndma_ch = container_of(napi, struct adrv906x_ndma_chan, napi);
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	dma_addr_t addr, addr_cur, state;
	unsigned long flags;
	int count = 0;
	u8 *buff;

	spin_lock_irqsave(&ndma_ch->lock, flags);
	while (count < budget) {
		buff = (u8 *)ndma_ch->rx_buffs[ndma_ch->rx_tail];
		addr = ndma_ch->rx_ring[ndma_ch->rx_tail].start;

		if (buff[0] == 0)
			break; /* WU not copied */

		/* Clear DMA IRQ status */
		iowrite32(DMA_DONE, ndma_ch->rx_dma_base + DMA_STAT);

		addr_cur = ioread32(ndma_ch->rx_dma_base + DMA_ADDR_CUR);
		state = ioread32(ndma_ch->rx_dma_base + DMA_STAT);
		if (addr_cur >= addr &&
		    addr_cur < addr + NDMA_TX_HDR_STATUS_SIZE &&
		    (DMA_RUN_MASK & state) != DMA_RUN_IDLE)
			break; /* WU copy in progress */

		ndma_ch->rx_tail = (ndma_ch->rx_tail + 1) % NDMA_TX_RING_SIZE;
		adrv906x_ndma_process_tx_status(ndma_ch, buff);

		buff[0] = 0;
		count++;
	}
	spin_unlock_irqrestore(&ndma_ch->lock, flags);

	if (count < budget) {
		/* We processed all status available. Tell NAPI it can
		 * stop polling then re-enable rx interrupts.
		 */
		napi_complete_done(napi, count);

		spin_lock_irqsave(&ndma_ch->lock, flags);
		adrv906x_ndma_enable_irqs(ndma_dev, NDMA_TX_STATUS_DMA_DONE_IRQ);
		spin_unlock_irqrestore(&ndma_ch->lock, flags);
	}

	return count;
}

static int adrv906x_ndma_rx_data_and_status_poll(struct napi_struct *napi, int budget)
{
	struct adrv906x_ndma_chan *ndma_ch = container_of(napi, struct adrv906x_ndma_chan, napi);
	union adrv906x_ndma_chan_stats *stats = &ndma_ch->stats;
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	dma_addr_t buf_addr, cur_addr, next_desc_addr;
	int count = 0, cur_desc_idx, next_desc_idx;
	struct device *dev = ndma_dev->dev;
	unsigned long flags;
	struct sk_buff *skb;
	u32 state;

	spin_lock_irqsave(&ndma_ch->lock, flags);

	/* Clear DMA IRQ status */
	iowrite32(DMA_DONE, ndma_ch->rx_dma_base + DMA_STAT);

	while (count < budget) {
		if (!ndma_ch->rx_buffs[ndma_ch->rx_tail])
			break;

		skb = (struct sk_buff *)ndma_ch->rx_buffs[ndma_ch->rx_tail];
		buf_addr = ndma_ch->rx_ring[ndma_ch->rx_tail].start;

		dma_sync_single_for_cpu(dev, buf_addr, NDMA_RX_HDR_DATA_SIZE, DMA_FROM_DEVICE);

		if (skb->data[0] == 0)
			break; /* WU not copied */

		cur_addr = ioread32(ndma_ch->rx_dma_base + DMA_ADDR_CUR);
		state = ioread32(ndma_ch->rx_dma_base + DMA_STAT);
		if (cur_addr >= buf_addr &&
		    cur_addr < buf_addr + NDMA_RX_WU_BUF_SIZE &&
		    (DMA_RUN_MASK & state) != DMA_RUN_IDLE)
			break; /* WU copy in progress */

		ndma_ch->rx_buffs[ndma_ch->rx_tail] = NULL;
		dma_unmap_single(dev, buf_addr, NDMA_RX_WU_BUF_SIZE, DMA_FROM_DEVICE);
		adrv906x_ndma_process_rx_work_unit(ndma_ch, skb, budget);

		ndma_ch->rx_tail = (ndma_ch->rx_tail + 1) % NDMA_RX_RING_SIZE;
		ndma_ch->rx_free--;
		count++;
	}

	/* If there are no free buffers, the DMA will be idle. In this case, we need to
	 * allocate and apply a new descriptor list. Otherwise, we stop the DMA transfer,
	 * verify if the end of the descriptor list hasn't been reached, and if it hasn't,
	 * extend the current list with the new descriptor before resuming the DMA transfer.
	 */
	if (ndma_ch->rx_free == 0) {
		adrv906x_ndma_refill_rx(ndma_ch, budget);
		adrv906x_dma_rx_start(ndma_ch);
	} else if (count) {
		/* Suspend the next DMA transfer - note: this doesn't stop fetching new
		 * descriptors after finishing the current transfer, so we need to check the
		 * status of both the current and next descriptors.
		 */
		iowrite32(SUSPEND_TRANSFER, ndma_ch->rx_dma_base + DMA_BWLCNT);

		/* Get the index of the current and next descriptors */
		next_desc_addr = ioread32(ndma_ch->rx_dma_base + DMA_NEXT_DESC);
		next_desc_idx = div64_u64(next_desc_addr - ndma_ch->rx_ring_dma,
					  sizeof(struct dma_desc));
		cur_desc_idx = (next_desc_idx + NDMA_RX_RING_SIZE - 1) % NDMA_RX_RING_SIZE;

		/* If neither current nor next descriptors are the end of the list,
		 * append new descriptors to the end.
		 */
		if ((ndma_ch->rx_ring[cur_desc_idx].cfg & DMAFLOW_LIST) &&
		    (ndma_ch->rx_ring[next_desc_idx].cfg & DMAFLOW_LIST))
			adrv906x_ndma_refill_rx(ndma_ch, budget);

		/* Resume DMA transfer */
		iowrite32(FULL_BANDWIDTH, ndma_ch->rx_dma_base + DMA_BWLCNT);
	}

	spin_unlock_irqrestore(&ndma_ch->lock, flags);

	stats->rx.pending_work_units = ndma_ch->rx_free;

	if (count < budget) {
		/* We processed all packets available. Tell NAPI it can
		 * stop polling then re-enable rx interrupts.
		 */
		napi_complete_done(napi, count);

		spin_lock_irqsave(&ndma_ch->lock, flags);
		adrv906x_ndma_enable_irqs(ndma_dev, NDMA_RX_DMA_DONE_IRQ);
		spin_unlock_irqrestore(&ndma_ch->lock, flags);
	}

	return count;
}

static ssize_t adrv906x_ndma_flood_mitigate_show(struct device *dev,
						 struct device_attribute *attr, char *buf)
{
	struct adrv906x_ndma_dev *ndma =
		container_of(attr, struct adrv906x_ndma_dev, attr_flood_mitigate);

	return sprintf(buf, "%d\n", ndma->flood_mitigate_en);
}

static ssize_t adrv906x_ndma_flood_mitigate_store(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf, size_t count)
{
	struct adrv906x_ndma_dev *ndma =
		container_of(attr, struct adrv906x_ndma_dev, attr_flood_mitigate);
	int val, ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return -EINVAL;

	ndma->flood_mitigate_en = !!val;
	return count;
}

static ssize_t adrv906x_ndma_mac_add_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct adrv906x_ndma_dev *ndma =
		container_of(attr, struct adrv906x_ndma_dev, attr_mac_add);
	u8 mac[ETH_ALEN];
	int ret;

	if (!mac_pton(buf, mac))
		return -EINVAL;

	if (adrv906x_ndma_mac_exists_in_table(ndma, mac))
		return -EEXIST;

	ret = adrv906x_ndma_add_mac(ndma, mac);

	return ret ? ret : count;
}

static ssize_t adrv906x_ndma_mac_remove_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct adrv906x_ndma_dev *ndma =
		container_of(attr, struct adrv906x_ndma_dev, attr_mac_remove);
	u8 mac[ETH_ALEN];
	int ret;

	if (!mac_pton(buf, mac))
		return -EINVAL;

	ret = adrv906x_ndma_remove_mac(ndma, mac);

	return ret ? ret : count;
}

static ssize_t adrv906x_ndma_mac_list_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct adrv906x_ndma_dev *ndma =
		container_of(attr, struct adrv906x_ndma_dev, attr_mac_list);
	struct adrv906x_ndma_mac_entry *entry;
	int bkt, len = 0;

	rcu_read_lock();
	hash_for_each_rcu(ndma->mac_table, bkt, entry, hnode) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%pM\n", entry->mac);
	}
	rcu_read_unlock();

	return len;
}

int adrv906x_ndma_probe(struct platform_device *pdev, struct net_device *ndev,
			struct device_node *ndma_np, struct adrv906x_ndma_dev *ndma_dev,
			ndma_flood_callback flood_cb_fn, ndma_pkt_callback tx_cb_fn,
			ndma_pkt_callback rx_cb_fn, void *cb_param)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	bool is_switch_enabled = !!flood_cb_fn;
	struct platform_device *ndma_pdev;
	struct device *dev;
	int ret;

	ndma_pdev = of_platform_device_create(ndma_np, NULL, &pdev->dev);
	if (!ndma_pdev) {
		dev_err(&pdev->dev, "failed to create ndma platform device");
		return -ENODEV;
	}
	dev = &ndma_pdev->dev;
	ndma_dev->dev = dev;
	ndma_dev->flood_cb_fn = flood_cb_fn;
	tx_chan->status_cb_fn = tx_cb_fn;
	tx_chan->cb_param = cb_param;
	rx_chan->status_cb_fn = rx_cb_fn;
	rx_chan->cb_param = cb_param;

	hash_init(ndma_dev->mac_table);
	if (of_property_read_u32(ndma_np, "id", &ndma_dev->dev_num)) {
		dev_err(dev, "failed to retrieve ndma device id from device tree");
		return -EINVAL;
	}

	ret = adrv906x_ndma_device_init(ndma_dev, ndma_np);
	if (ret)
		return ret;
	ret = adrv906x_ndma_get_reset_ctrl(ndma_dev, ndma_np, is_switch_enabled);
	if (ret)
		return ret;
	ret = adrv906x_ndma_get_intr_ctrl(ndma_dev, ndma_np);
	if (ret)
		return ret;
	ret = adrv906x_ndma_alloc_rings(ndma_dev);
	if (ret)
		return ret;

	INIT_LIST_HEAD(&rx_chan->rx_data_wu_list);
	INIT_LIST_HEAD(&rx_chan->rx_flood_evt_list);

	dev_set_threaded(ndev, true);
	netif_napi_add_weight(ndev, &rx_chan->napi,
			      adrv906x_ndma_rx_data_and_status_poll, NDMA_RX_NAPI_POLL_WEIGHT);
	netif_napi_add_weight(ndev, &tx_chan->napi,
			      adrv906x_ndma_tx_status_poll, NDMA_TX_NAPI_POLL_WEIGHT);

	ndma_dev->attr_flood_mitigate.attr.name = "flood_mitigate";
	ndma_dev->attr_flood_mitigate.attr.mode = 0664;
	ndma_dev->attr_flood_mitigate.show = adrv906x_ndma_flood_mitigate_show;
	ndma_dev->attr_flood_mitigate.store = adrv906x_ndma_flood_mitigate_store;
	ndma_dev->attr_mac_add.attr.name = "mac_add";
	ndma_dev->attr_mac_add.attr.mode = 0200;
	ndma_dev->attr_mac_add.show = NULL;
	ndma_dev->attr_mac_add.store = adrv906x_ndma_mac_add_store;
	ndma_dev->attr_mac_remove.attr.name = "mac_remove";
	ndma_dev->attr_mac_remove.attr.mode = 0200;
	ndma_dev->attr_mac_remove.show = NULL;
	ndma_dev->attr_mac_remove.store = adrv906x_ndma_mac_remove_store;
	ndma_dev->attr_mac_list.attr.name = "mac_list";
	ndma_dev->attr_mac_list.attr.mode = 0444;
	ndma_dev->attr_mac_list.show = adrv906x_ndma_mac_list_show;
	ndma_dev->attr_mac_list.store = NULL;
	ndma_dev->attrs[0] = &ndma_dev->attr_flood_mitigate.attr;
	ndma_dev->attrs[1] = &ndma_dev->attr_mac_add.attr;
	ndma_dev->attrs[2] = &ndma_dev->attr_mac_remove.attr;
	ndma_dev->attrs[3] = &ndma_dev->attr_mac_list.attr;
	ndma_dev->attrs[4] = NULL;
	ndma_dev->attr_group.attrs = ndma_dev->attrs;

	ret = sysfs_create_group(&ndma_dev->dev->kobj, &ndma_dev->attr_group);
	if (ret) {
		dev_err(ndma_dev->dev, "failed to create sysfs group\n");
		return ret;
	}

	return 0;
}

void adrv906x_ndma_remove(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;

	adrv906x_ndma_clear_mac_table(ndma_dev);
	sysfs_remove_group(&ndma_dev->dev->kobj, &ndma_dev->attr_group);
	cancel_delayed_work_sync(&tx_chan->tx_frames_timeout_work);
	cancel_work_sync(&rx_chan->rx_flood_mitigate_work);
	if (ndma_dev->wq)
		destroy_workqueue(ndma_dev->wq);

	adrv906x_ndma_chan_disable(tx_chan);
	adrv906x_ndma_chan_disable(rx_chan);
	of_platform_device_destroy(ndma_dev->dev, NULL);
}
