// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/ethtool.h>
#include <linux/bitrev.h>
#include <linux/completion.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <net/pkt_cls.h>
#include <net/pkt_sched.h>
#include <net/tcp.h>
#include <net/udp.h>
#include "adrv906x-net.h"
#include "adrv906x-cmn.h"
#include "adrv906x-mac.h"
#include "adrv906x-phy.h"
#include "adrv906x-ndma.h"
#include "adrv906x-ethtool.h"

/* Global PHC index exported by the PTP driver module.
 * This is used to report the PHC clock index via ethtool's get_ts_info.
 * The variable is set by the adrv906x_ptp module when it registers the
 * PTP hardware clock. This cross-module dependency exists because the
 * PTP and Ethernet drivers are separate loadable modules that need to
 * coordinate on the PHC index for userspace timestamping applications.
 */
#if IS_BUILTIN(CONFIG_PTP_1588_CLOCK_ADRV906X)
extern int adrv906x_phc_index;
#endif
extern const struct ethtool_ops adrv906x_ethtool_ops;

static const char adrv906x_gstrings_stats_names[][ETH_GSTRING_LEN] = {
	"mac_rx_drop_events",
	"mac_rx_octets",
	"mac_rx_pkts",
	"mac_rx_broadcast_pkts",
	"mac_rx_multicast_pkts",
	"mac_rx_unicast_pkts",
	"mac_rx_undersize_pkts",
	"mac_rx_oversize_pkts",
	"mac_rx_pkts_64_octets",
	"mac_rx_pkts_65to127_octets",
	"mac_rx_pkts_128to255_octets",
	"mac_rx_pkts_256to511_octets",
	"mac_rx_pkts_512to1023_octets",
	"mac_rx_pkts_1024to1518_octets",
	"mac_rx_pkts_1519tox_octets",
	"mac_rx_overflow",
	"mac_rx_crc_error",
	"mac_rx_mc_drop",
	"mac_rx_fragments",
	"mac_rx_jabbers",
	"mac_rx_mac_framing_error",
	"mac_rx_rs_framing_error",
	"mac_tx_drop_events",
	"mac_tx_octets",
	"mac_tx_pkts",
	"mac_tx_broadcast_pkts",
	"mac_tx_multicast_pkts",
	"mac_tx_unicast_pkts",
	"mac_tx_undersize_pkts",
	"mac_tx_oversize_pkts",
	"mac_tx_pkts_64_octets",
	"mac_tx_pkts_65to127_octets",
	"mac_tx_pkts_128to255_octets",
	"mac_tx_pkts_256to511_octets",
	"mac_tx_pkts_512to1023_octets",
	"mac_tx_pkts_1024to1518_octets",
	"mac_tx_pkts_1519tox_octets",
	"mac_tx_underflow",
	"mac_tx_padded",
	"intf_recovery_resets",
	"ndma_rx_frame_error",
	"ndma_rx_frame_size_error",
	"ndma_rx_frame_dropped_error",
	"ndma_rx_frame_dropped_s_plane",
	"ndma_rx_frame_dropped_m_plane",
	"ndma_rx_seqnumb_mismatch_error",
	"ndma_rx_wu_header_error",
	"ndma_rx_unknown_error",
	"ndma_rx_pending_work_unit",
	"ndma_rx_done_work_unit",
	"ndma_rx_dma_error",
	"ndma_rx_flooded_frame_count",
	"ndma_tx_frame_size_error",
	"ndma_tx_wu_data_header_error",
	"ndma_tx_wu_status_header_error",
	"ndma_tx_tstamp_timeout_error",
	"ndma_tx_seqnumb_mismatch_error",
	"ndma_tx_unknown_error",
	"ndma_tx_pending_work_unit",
	"ndma_tx_done_work_unit",
	"ndma_tx_data_dma_error",
	"ndma_tx_status_dma_error",
	"ndma_tx_recovery_count",
	"switch_port_reset_count",
	"switch_port0_err_irq",
	"switch_port0_pkt_fltr_rx",
	"switch_port0_bytes_fltr_rx",
	"switch_port0_pkt_buf_ovfl",
	"switch_port0_bytes_buf_ovfl",
	"switch_port0_pkt_err",
	"switch_port0_bytes_err",
	"switch_port0_drop_pkt_tx",
	"switch_port0_0_pkt_voq_nqn",
	"switch_port0_1_pkt_voq_nqn",
	"switch_port0_0_bytes_voq_nqn",
	"switch_port0_1_bytes_voq_nqn",
	"switch_port0_0_pkt_voq_dqn",
	"switch_port0_1_pkt_voq_dqn",
	"switch_port0_0_bytes_voq_dqn",
	"switch_port0_1_bytes_voq_dqn",
	"switch_port0_0_pkt_voq_dropn",
	"switch_port0_1_pkt_voq_dropn",
	"switch_port0_0_bytes_voq_dropn",
	"switch_port0_1_bytes_voq_dropn",
	"switch_port0_ucast_pkt_rx",
	"switch_port0_ucast_bytes_rx",
	"switch_port0_ucast_pkt_tx",
	"switch_port0_ucast_bytes_tx",
	"switch_port0_mcast_pkt_rx",
	"switch_port0_mcast_bytes_rx",
	"switch_port0_mcast_pkt_tx",
	"switch_port0_mcast_bytes_tx",
	"switch_port0_bcast_pkt_rx",
	"switch_port0_bcast_bytes_rx",
	"switch_port0_bcast_pkt_tx",
	"switch_port0_bcast_bytes_tx",
	"switch_port0_crd_buffer_drop",
	"switch_port1_err_irq",
	"switch_port1_pkt_fltr_rx",
	"switch_port1_bytes_fltr_rx",
	"switch_port1_pkt_buf_ovfl",
	"switch_port1_bytes_buf_ovfl",
	"switch_port1_pkt_err",
	"switch_port1_bytes_err",
	"switch_port1_drop_pkt_tx",
	"switch_port1_0_pkt_voq_nqn",
	"switch_port1_1_pkt_voq_nqn",
	"switch_port1_0_bytes_voq_nqn",
	"switch_port1_1_bytes_voq_nqn",
	"switch_port1_0_pkt_voq_dqn",
	"switch_port1_1_pkt_voq_dqn",
	"switch_port1_0_bytes_voq_dqn",
	"switch_port1_1_bytes_voq_dqn",
	"switch_port1_0_pkt_voq_dropn",
	"switch_port1_1_pkt_voq_dropn",
	"switch_port1_0_bytes_voq_dropn",
	"switch_port1_1_bytes_voq_dropn",
	"switch_port1_ucast_pkt_rx",
	"switch_port1_ucast_bytes_rx",
	"switch_port1_ucast_pkt_tx",
	"switch_port1_ucast_bytes_tx",
	"switch_port1_mcast_pkt_rx",
	"switch_port1_mcast_bytes_rx",
	"switch_port1_mcast_pkt_tx",
	"switch_port1_mcast_bytes_tx",
	"switch_port1_bcast_pkt_rx",
	"switch_port1_bcast_bytes_rx",
	"switch_port1_bcast_pkt_tx",
	"switch_port1_bcast_bytes_tx",
	"switch_port1_crd_buffer_drop",
	"switch_port2_pkt_fltr_rx",
	"switch_port2_bytes_fltr_rx",
	"switch_port2_pkt_buf_ovfl",
	"switch_port2_bytes_buf_ovfl",
	"switch_port2_pkt_err",
	"switch_port2_bytes_err",
	"switch_port2_drop_pkt_tx",
	"switch_port2_0_pkt_voq_nqn",
	"switch_port2_1_pkt_voq_nqn",
	"switch_port2_0_bytes_voq_nqn",
	"switch_port2_1_bytes_voq_nqn",
	"switch_port2_0_pkt_voq_dqn",
	"switch_port2_1_pkt_voq_dqn",
	"switch_port2_0_bytes_voq_dqn",
	"switch_port2_1_bytes_voq_dqn",
	"switch_port2_0_pkt_voq_dropn",
	"switch_port2_1_pkt_voq_dropn",
	"switch_port2_0_bytes_voq_dropn",
	"switch_port2_1_bytes_voq_dropn",
	"switch_port2_ucast_pkt_rx",
	"switch_port2_ucast_bytes_rx",
	"switch_port2_ucast_pkt_tx",
	"switch_port2_ucast_bytes_tx",
	"switch_port2_mcast_pkt_rx",
	"switch_port2_mcast_bytes_rx",
	"switch_port2_mcast_pkt_tx",
	"switch_port2_mcast_bytes_tx",
	"switch_port2_bcast_pkt_rx",
	"switch_port2_bcast_bytes_rx",
	"switch_port2_bcast_pkt_tx",
	"switch_port2_bcast_bytes_tx",
	"switch_port2_crd_buffer_drop",
	"pcs_link_drop_cnt",
};

static const char adrv906x_gstrings_selftest_names[][ETH_GSTRING_LEN] = {
	"NDMA loopback:    ",
	"Near-end loopback:",
};

#define ADRV906X_NUM_STATS ARRAY_SIZE(adrv906x_gstrings_stats_names)
#define ADRV906X_NUM_SELFTEST ARRAY_SIZE(adrv906x_gstrings_selftest_names)

struct adrv906x_test {
	char name[ETH_GSTRING_LEN];
	int (*fn)(struct net_device *ndev);
	unsigned char etest_flag;
};

struct payload_hdr {
	__be32 version;
	__be64 magic;
	u8 id;
} __packed;

struct adrv906x_loopback_test_attrs {
	const unsigned char *src;
	const unsigned char *dst;
	u32 ip_src;
	u32 ip_dst;
	u32 exp_hash;
	int timeout;
	u8 id;
};

struct adrv906x_test_priv {
	struct adrv906x_loopback_test_attrs *attrs;
	struct packet_type pt;
	struct completion completion;
	bool ok;
};

#define ADRV906X_TEST_PKT_SIZE (sizeof(struct ethhdr) + sizeof(struct iphdr) + \
				sizeof(struct udphdr) + sizeof(struct payload_hdr))
#define ADRV906X_TEST_PKT_TOTAL_SIZE 1024
#define ADRV906X_TEST_PKT_MAGIC 0xdeadcafecafedeadULL
#define ADRV906X_LB_TIMEOUT       msecs_to_jiffies(200)

static u8 adrv906x_packet_next_id;

static int adrv906x_ethtool_set_link_ksettings(struct net_device *ndev,
					       const struct ethtool_link_ksettings *cmd)
{
	struct phy_device *phydev = ndev->phydev;
	u8 autoneg = cmd->base.autoneg;
	u8 duplex = cmd->base.duplex;
	u32 speed = cmd->base.speed;

	if (!phydev)
		return -ENODEV;

	/* This PHY only supports:
	 * - Speeds: 10G or 25G
	 * - Autoneg: Disabled
	 * - Duplex: Full
	 */
	if (autoneg != AUTONEG_DISABLE)
		return -EINVAL;

	if (speed != SPEED_10000 && speed != SPEED_25000)
		return -EINVAL;

	if (duplex != DUPLEX_FULL)
		return -EINVAL;

	/* Configure the PHY directly for fixed speed operation */
	mutex_lock(&phydev->lock);
	phydev->autoneg = AUTONEG_DISABLE;
	phydev->speed = speed;
	phydev->duplex = DUPLEX_FULL;

	/* Update interface mode based on speed */
	if (speed == SPEED_25000)
		phydev->interface = PHY_INTERFACE_MODE_25GBASER;
	else if (speed == SPEED_10000)
		phydev->interface = PHY_INTERFACE_MODE_10GBASER;

	/* If PHY is running, trigger reconfiguration */
	if (phy_is_started(phydev)) {
		phydev->state = PHY_UP;
		phy_trigger_machine(phydev);
	}
	mutex_unlock(&phydev->lock);

	return 0;
}

static int adrv906x_ethtool_get_ts_info(struct net_device *ndev,
					struct kernel_ethtool_ts_info *info)
{
	info->so_timestamping =
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE |
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;
	info->tx_types =
		(1 << HWTSTAMP_TX_OFF) |
		(1 << HWTSTAMP_TX_ON);
	info->rx_filters =
		(1 << HWTSTAMP_FILTER_PTP_V2_L4_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L4_SYNC) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L2_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L2_SYNC) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ) |
		(1 << HWTSTAMP_FILTER_PTP_V2_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_SYNC) |
		(1 << HWTSTAMP_FILTER_PTP_V2_DELAY_REQ) |
		(1 << HWTSTAMP_FILTER_ALL);

#if IS_BUILTIN(CONFIG_PTP_1588_CLOCK_ADRV906X)
	info->phc_index = adrv906x_phc_index;
#else
	info->phc_index = -1;
#endif
	return 0;
}

static int adrv906x_ethtool_get_sset_count(struct net_device *ndev, int sset)
{
	if (sset == ETH_SS_STATS)
		return ADRV906X_NUM_STATS;

	if (sset == ETH_SS_TEST)
		return ADRV906X_NUM_SELFTEST;

	return -EOPNOTSUPP;
}

static void adrv906x_ethtool_get_strings(struct net_device *ndev, u32 sset, u8 *buf)
{
	if (sset == ETH_SS_STATS)
		memcpy(buf, &adrv906x_gstrings_stats_names,
		       sizeof(adrv906x_gstrings_stats_names));

	if (sset == ETH_SS_TEST)
		memcpy(buf, &adrv906x_gstrings_selftest_names,
		       sizeof(adrv906x_gstrings_selftest_names));
}

static void adrv906x_ethtool_get_stats(struct net_device *ndev, struct ethtool_stats *stats,
				       u64 *data)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	struct adrv906x_eth_switch *es = &eth_if->ethswitch;
	union adrv906x_ndma_chan_stats *ndma_rx_stats = &adrv906x_dev->ndma_dev->rx_chan.stats;
	union adrv906x_ndma_chan_stats *ndma_tx_stats = &adrv906x_dev->ndma_dev->tx_chan.stats;
	struct adrv906x_mac_rx_stats *mac_rx_stats = &adrv906x_dev->mac.hw_stats_rx;
	struct adrv906x_mac_tx_stats *mac_tx_stats = &adrv906x_dev->mac.hw_stats_tx;
	int i, base_idx;

	/* Refresh all stats from hardware before reading */
	adrv906x_mac_update_hw_stats(&adrv906x_dev->mac, false);
	adrv906x_switch_update_hw_stats(es);
	adrv906x_ndma_update_frame_drop_stats(adrv906x_dev->ndma_dev);
	adrv906x_cmn_pcs_link_drop_cnt_read(eth_if);

	/* Read MAC stats under lock to prevent race with delayed work */
	mutex_lock(&adrv906x_dev->mac.stats_lock);
	data[0] = mac_rx_stats->general_stats.drop_events;
	data[1] = mac_rx_stats->general_stats.octets;
	data[2] = mac_rx_stats->general_stats.pkts;
	data[3] = mac_rx_stats->general_stats.broadcast_pkts;
	data[4] = mac_rx_stats->general_stats.multicast_pkts;
	data[5] = mac_rx_stats->general_stats.unicast_pkts;
	data[6] = mac_rx_stats->general_stats.undersize_pkts;
	data[7] = mac_rx_stats->general_stats.oversize_pkts;
	data[8] = mac_rx_stats->general_stats.pkts_64_octets;
	data[9] = mac_rx_stats->general_stats.pkts_65to127_octets;
	data[10] = mac_rx_stats->general_stats.pkts_128to255_octets;
	data[11] = mac_rx_stats->general_stats.pkts_256to511_octets;
	data[12] = mac_rx_stats->general_stats.pkts_512to1023_octets;
	data[13] = mac_rx_stats->general_stats.pkts_1024to1518_octets;
	data[14] = mac_rx_stats->general_stats.pkts_1519tox_octets;
	data[15] = mac_rx_stats->overflow;
	data[16] = mac_rx_stats->crc_errors;
	data[17] = mac_rx_stats->mc_drop;
	data[18] = mac_rx_stats->fragments;
	data[19] = mac_rx_stats->jabbers;
	data[20] = mac_rx_stats->mac_framing_error;
	data[21] = mac_rx_stats->rs_framing_error;
	data[22] = mac_tx_stats->general_stats.drop_events;
	data[23] = mac_tx_stats->general_stats.octets;
	data[24] = mac_tx_stats->general_stats.pkts;
	data[25] = mac_tx_stats->general_stats.broadcast_pkts;
	data[26] = mac_tx_stats->general_stats.multicast_pkts;
	data[27] = mac_tx_stats->general_stats.unicast_pkts;
	data[28] = mac_tx_stats->general_stats.undersize_pkts;
	data[29] = mac_tx_stats->general_stats.oversize_pkts;
	data[30] = mac_tx_stats->general_stats.pkts_64_octets;
	data[31] = mac_tx_stats->general_stats.pkts_65to127_octets;
	data[32] = mac_tx_stats->general_stats.pkts_128to255_octets;
	data[33] = mac_tx_stats->general_stats.pkts_256to511_octets;
	data[34] = mac_tx_stats->general_stats.pkts_512to1023_octets;
	data[35] = mac_tx_stats->general_stats.pkts_1024to1518_octets;
	data[36] = mac_tx_stats->general_stats.pkts_1519tox_octets;
	data[37] = mac_tx_stats->underflow;
	data[38] = mac_tx_stats->padded;
	mutex_unlock(&adrv906x_dev->mac.stats_lock);

	data[39] = adrv906x_dev->intf_recovery_resets;

	/* Read NDMA stats under lock to prevent race with delayed work */
	spin_lock(&adrv906x_dev->ndma_dev->lock);
	data[40] = ndma_rx_stats->rx.frame_errors;
	data[41] = ndma_rx_stats->rx.frame_size_errors;
	data[42] = ndma_rx_stats->rx.frame_dropped_errors;
	data[43] = ndma_rx_stats->rx.frame_dropped_splane_errors;
	data[44] = ndma_rx_stats->rx.frame_dropped_mplane_errors;
	data[45] = ndma_rx_stats->rx.seqnumb_mismatch_errors;
	data[46] = ndma_rx_stats->rx.wu_header_errors;
	data[47] = ndma_rx_stats->rx.unknown_errors;
	data[48] = ndma_rx_stats->rx.pending_work_units;
	data[49] = ndma_rx_stats->rx.done_work_units;
	data[50] = ndma_rx_stats->rx.dma_errors;
	data[51] = ndma_rx_stats->rx.flooded_frame_count;
	data[52] = ndma_tx_stats->tx.frame_size_errors;
	data[53] = ndma_tx_stats->tx.wu_data_header_errors;
	data[54] = ndma_tx_stats->tx.wu_status_header_errors;
	data[55] = ndma_tx_stats->tx.tstamp_timeout_errors;
	data[56] = ndma_tx_stats->tx.seqnumb_mismatch_errors;
	data[57] = ndma_tx_stats->tx.unknown_errors;
	data[58] = ndma_tx_stats->tx.pending_work_units;
	data[59] = ndma_tx_stats->tx.done_work_units;
	data[60] = ndma_tx_stats->tx.data_dma_errors;
	data[61] = ndma_tx_stats->tx.status_dma_errors;
	data[62] = ndma_tx_stats->tx.recovery_count;
	spin_unlock(&adrv906x_dev->ndma_dev->lock);

	data[63] = atomic64_read(&es->port_reset_count);

	/* Read switch stats under lock to prevent race with delayed work */
	mutex_lock(&es->lock);
	for (i = 0; i < SWITCH_MAX_PORT_NUM; i++) {
		base_idx = 64 + i * (SWITCH_PORT_STATS_NUM + 1);

		/* Error IRQ stats for ports 0 and 1 only (no IRQ for CPU port 2) */
		if (i < SWITCH_MAX_PORT_NUM - 1) {
			data[base_idx] = atomic64_read(&es->err_irq_count[i]);
			base_idx++;
		}

		data[base_idx + 0] = es->port_stats[i].pkt_fltr_rx;
		data[base_idx + 1] = es->port_stats[i].bytes_fltr_rx;
		data[base_idx + 2] = es->port_stats[i].pkt_buf_ovfl;
		data[base_idx + 3] = es->port_stats[i].bytes_buf_ovfl;
		data[base_idx + 4] = es->port_stats[i].pkt_err;
		data[base_idx + 5] = es->port_stats[i].bytes_err;
		data[base_idx + 6] = es->port_stats[i].drop_pkt_tx;
		data[base_idx + 7] = es->port_stats[i].ipv0_pkt_voq_nqn;
		data[base_idx + 8] = es->port_stats[i].ipv1_pkt_voq_nqn;
		data[base_idx + 9] = es->port_stats[i].ipv0_bytes_voq_nqn;
		data[base_idx + 10] = es->port_stats[i].ipv1_bytes_voq_nqn;
		data[base_idx + 11] = es->port_stats[i].ipv0_pkt_voq_dqn;
		data[base_idx + 12] = es->port_stats[i].ipv1_pkt_voq_dqn;
		data[base_idx + 13] = es->port_stats[i].ipv0_bytes_voq_dqn;
		data[base_idx + 14] = es->port_stats[i].ipv1_bytes_voq_dqn;
		data[base_idx + 15] = es->port_stats[i].ipv0_pkt_voq_dropn;
		data[base_idx + 16] = es->port_stats[i].ipv1_pkt_voq_dropn;
		data[base_idx + 17] = es->port_stats[i].ipv0_bytes_voq_dropn;
		data[base_idx + 18] = es->port_stats[i].ipv1_bytes_voq_dropn;
		data[base_idx + 19] = es->port_stats[i].ucast_pkt_rx;
		data[base_idx + 20] = es->port_stats[i].ucast_bytes_rx;
		data[base_idx + 21] = es->port_stats[i].ucast_pkt_tx;
		data[base_idx + 22] = es->port_stats[i].ucast_bytes_tx;
		data[base_idx + 23] = es->port_stats[i].mcast_pkt_rx;
		data[base_idx + 24] = es->port_stats[i].mcast_bytes_rx;
		data[base_idx + 25] = es->port_stats[i].mcast_pkt_tx;
		data[base_idx + 26] = es->port_stats[i].mcast_bytes_tx;
		data[base_idx + 27] = es->port_stats[i].bcast_pkt_rx;
		data[base_idx + 28] = es->port_stats[i].bcast_bytes_rx;
		data[base_idx + 29] = es->port_stats[i].bcast_pkt_tx;
		data[base_idx + 30] = es->port_stats[i].bcast_bytes_tx;
		data[base_idx + 31] = es->port_stats[i].crd_buffer_drop;
	}
	mutex_unlock(&es->lock);

	mutex_lock(&eth_if->mtx);
	data[ADRV906X_NUM_STATS - 1] = adrv906x_dev->pcs_link_drop_cnt;
	mutex_unlock(&eth_if->mtx);
}

static const struct ethtool_rmon_hist_range adrv906x_ethtool_rmon_ranges[] = {
	{ 64,	64			  },
	{ 65,	127			  },
	{ 128,	255			  },
	{ 256,	511			  },
	{ 512,	1023			  },
	{ 1024, 1518			  },
	{ 1519, NDMA_MAX_FRAME_SIZE_VALUE },
	{},
};

static void adrv906x_ethtool_get_rmon_stats(struct net_device *ndev,
					    struct ethtool_rmon_stats *stats,
					    const struct ethtool_rmon_hist_range **ranges)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_mac_rx_stats *mac_rx_stats = &adrv906x_dev->mac.hw_stats_rx;
	struct adrv906x_mac_tx_stats *mac_tx_stats = &adrv906x_dev->mac.hw_stats_tx;

	*ranges = adrv906x_ethtool_rmon_ranges;

	/* Refresh stats from hardware before reading */
	adrv906x_mac_update_hw_stats(&adrv906x_dev->mac, false);

	/* Read MAC stats under lock to prevent race with delayed work */
	mutex_lock(&adrv906x_dev->mac.stats_lock);
	stats->undersize_pkts = mac_rx_stats->general_stats.undersize_pkts;
	stats->oversize_pkts = mac_rx_stats->general_stats.oversize_pkts;
	stats->fragments = mac_rx_stats->fragments;
	stats->jabbers = mac_rx_stats->jabbers;
	stats->hist[0] = mac_rx_stats->general_stats.pkts_64_octets;
	stats->hist[1] = mac_rx_stats->general_stats.pkts_65to127_octets;
	stats->hist[2] = mac_rx_stats->general_stats.pkts_128to255_octets;
	stats->hist[3] = mac_rx_stats->general_stats.pkts_256to511_octets;
	stats->hist[4] = mac_rx_stats->general_stats.pkts_512to1023_octets;
	stats->hist[5] = mac_rx_stats->general_stats.pkts_1024to1518_octets;
	stats->hist[6] = mac_rx_stats->general_stats.pkts_1519tox_octets;
	stats->hist_tx[0] = mac_tx_stats->general_stats.pkts_64_octets;
	stats->hist_tx[1] = mac_tx_stats->general_stats.pkts_65to127_octets;
	stats->hist_tx[2] = mac_tx_stats->general_stats.pkts_128to255_octets;
	stats->hist_tx[3] = mac_tx_stats->general_stats.pkts_256to511_octets;
	stats->hist_tx[4] = mac_tx_stats->general_stats.pkts_512to1023_octets;
	stats->hist_tx[5] = mac_tx_stats->general_stats.pkts_1024to1518_octets;
	stats->hist_tx[6] = mac_tx_stats->general_stats.pkts_1519tox_octets;
	mutex_unlock(&adrv906x_dev->mac.stats_lock);
}

static int adrv906x_ethtool_get_fecparam(struct net_device *ndev,
					 struct ethtool_fecparam *fecparam)
{
	struct phy_device *phydev = ndev->phydev;

	mutex_lock(&phydev->lock);
	if (phydev->speed == SPEED_25000) {
		fecparam->fec = ETHTOOL_FEC_OFF | ETHTOOL_FEC_RS;
		if (phydev->dev_flags & ADRV906X_PHY_FLAGS_PCS_RS_FEC_EN)
			fecparam->active_fec = ETHTOOL_FEC_RS;
		else
			fecparam->active_fec = ETHTOOL_FEC_OFF;
	} else {
		fecparam->active_fec = ETHTOOL_FEC_OFF;
		fecparam->fec = ETHTOOL_FEC_OFF;
	}
	mutex_unlock(&phydev->lock);

	return 0;
}

static int adrv906x_ethtool_set_fecparam(struct net_device *ndev,
					 struct ethtool_fecparam *fecparam)
{
	struct phy_device *phydev = ndev->phydev;
	bool fec_cur_en, fec_new_en;

	if (!(fecparam->fec & ETHTOOL_FEC_OFF) && !(fecparam->fec & ETHTOOL_FEC_RS))
		return -EOPNOTSUPP;

	mutex_lock(&phydev->lock);
	fec_cur_en = !!(phydev->dev_flags & ADRV906X_PHY_FLAGS_PCS_RS_FEC_EN);
	fec_new_en = !!(fecparam->fec & ETHTOOL_FEC_RS);

	if (fec_cur_en != fec_new_en) {
		phydev->dev_flags ^= ADRV906X_PHY_FLAGS_PCS_RS_FEC_EN;

		if (phy_is_started(phydev)) {
			phydev->state = PHY_UP;
			phy_start_machine(phydev);
		}
	}
	mutex_unlock(&phydev->lock);

	return 0;
}

static struct sk_buff *adrv906x_test_get_udp_skb(struct net_device *ndev,
						 struct adrv906x_loopback_test_attrs *attr)
{
	struct sk_buff *skb = NULL;
	struct udphdr *uhdr = NULL;
	struct payload_hdr *phdr;
	struct ethhdr *ehdr;
	struct iphdr *ihdr;
	int iplen, size, payload_size;
	u8 *payload_data;
	int i;

	/* Calculate payload to pad packet to 1024 bytes */
	payload_size = ADRV906X_TEST_PKT_TOTAL_SIZE - ADRV906X_TEST_PKT_SIZE;
	size = ADRV906X_TEST_PKT_TOTAL_SIZE + NDMA_TX_HDR_SOF_SIZE;

	skb = netdev_alloc_skb(ndev, size);
	if (!skb)
		return NULL;

	prefetchw(skb->data);

	skb_reserve(skb, NDMA_TX_HDR_SOF_SIZE);
	skb_reset_mac_header(skb);
	ehdr = (struct ethhdr *)skb_put(skb, ETH_HLEN);

	skb_set_network_header(skb, skb->len);
	ihdr = skb_put(skb, sizeof(*ihdr));

	skb_set_transport_header(skb, skb->len);
	uhdr = skb_put(skb, sizeof(*uhdr));

	eth_zero_addr(ehdr->h_dest);
	if (attr->src)
		ether_addr_copy(ehdr->h_source, attr->src);
	if (attr->dst)
		ether_addr_copy(ehdr->h_dest, attr->dst);

	ehdr->h_proto = htons(ETH_P_IP);

	uhdr->len = htons(sizeof(*phdr) + sizeof(*uhdr) + payload_size);
	uhdr->check = 0;

	ihdr->ihl = 5;
	ihdr->ttl = 32;
	ihdr->version = 4;
	ihdr->protocol = IPPROTO_UDP;
	iplen = sizeof(*ihdr) + sizeof(*phdr) + payload_size;
	iplen += sizeof(*uhdr);

	ihdr->tot_len = htons(iplen);
	ihdr->frag_off = 0;
	ihdr->saddr = htonl(attr->ip_src);
	ihdr->daddr = htonl(attr->ip_dst);
	ihdr->tos = 0;
	ihdr->id = 0;
	ip_send_check(ihdr);

	phdr = skb_put(skb, sizeof(*phdr));
	phdr->version = 0;
	phdr->magic = cpu_to_be64(ADRV906X_TEST_PKT_MAGIC);
	attr->id = adrv906x_packet_next_id;
	phdr->id = adrv906x_packet_next_id++;

	/* Add padding data */
	payload_data = skb_put(skb, payload_size);
	for (i = 0; i < payload_size; i++)
		payload_data[i] = i & 0xff;

	skb->csum = 0;
	skb->ip_summed = CHECKSUM_PARTIAL;
	udp4_hwcsum(skb, ihdr->saddr, ihdr->daddr);

	skb->protocol = htons(ETH_P_IP);
	skb->pkt_type = PACKET_HOST;
	skb->dev = ndev;

	return skb;
}

static int adrv906x_test_loopback_validate(struct sk_buff *skb, struct net_device *ndev,
					   struct packet_type *pt, struct net_device *orig_ndev)
{
	struct adrv906x_test_priv *tpriv = pt->af_packet_priv;
	const unsigned char *src = tpriv->attrs->src;
	const unsigned char *dst = tpriv->attrs->dst;

	struct payload_hdr *phdr;
	struct ethhdr *ehdr;
	struct udphdr *uhdr;
	struct iphdr *ihdr;

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (!skb)
		goto out;

	if (skb_linearize(skb))
		goto out;
	if (skb_headlen(skb) < (ADRV906X_TEST_PKT_SIZE - ETH_HLEN))
		goto out;

	ehdr = (struct ethhdr *)skb_mac_header(skb);

	if (dst)
		if (!ether_addr_equal_unaligned(ehdr->h_dest, dst))
			goto out;

	if (src)
		if (!ether_addr_equal_unaligned(ehdr->h_source, src))
			goto out;

	ihdr = ip_hdr(skb);

	if (ihdr->protocol != IPPROTO_UDP)
		goto out;

	uhdr = (struct udphdr *)((u8 *)ihdr + 4 * ihdr->ihl);

	phdr = (struct payload_hdr *)((u8 *)uhdr + sizeof(*uhdr));

	if (phdr->magic != cpu_to_be64(ADRV906X_TEST_PKT_MAGIC))
		goto out;
	if (tpriv->attrs->exp_hash && !skb->hash)
		goto out;
	if (tpriv->attrs->id != phdr->id)
		goto out;

	tpriv->ok = true;
	complete(&tpriv->completion);
out:
	kfree_skb(skb);
	return 0;
}

static int adrv906x_test_loopback_run(struct net_device *ndev,
				      struct adrv906x_loopback_test_attrs *attr)
{
	struct adrv906x_test_priv *tpriv;
	struct sk_buff *skb = NULL;
	int ret;

	netdev_printk(KERN_DEBUG, ndev, "adrv906x_test_loopback_run");
	tpriv = kzalloc(sizeof(*tpriv), GFP_KERNEL);
	if (!tpriv)
		return -ENOMEM;

	tpriv->ok = false;
	init_completion(&tpriv->completion);

	tpriv->pt.type = htons(ETH_P_IP);
	tpriv->pt.func = adrv906x_test_loopback_validate;
	tpriv->pt.dev = ndev;
	tpriv->pt.af_packet_priv = tpriv;
	tpriv->attrs = attr;

	dev_add_pack(&tpriv->pt);
	skb = adrv906x_test_get_udp_skb(ndev, attr);
	if (!skb) {
		ret = -ENOMEM;
		goto cleanup;
	}

	ret = __netdev_start_xmit(ndev->netdev_ops, skb, ndev, false);
	if (ret)
		goto cleanup;

	wait_for_completion_timeout(&tpriv->completion, attr->timeout);

	ret = tpriv->ok ? 0 : -ETIMEDOUT;

cleanup:
	dev_remove_pack(&tpriv->pt);
	kfree(tpriv);

	return ret;
}

static int adrv906x_phy_loopback_config(struct net_device *ndev, bool enable)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	struct adrv906x_eth_switch *es = &eth_if->ethswitch;
	struct adrv906x_mac *mac = &adrv906x_dev->mac;
	struct phy_device *phydev = ndev->phydev;
	int other_port = 1 - adrv906x_dev->port;

	if (enable) {
		phy_loopback(phydev, true);
		msleep(100);
		adrv906x_mac_set_path(mac, true);
		if (es->enabled) {
			/* Disable the other port to isolate loopback traffic */
			adrv906x_switch_port_enable(es, other_port, false);
			adrv906x_switch_port_enable(es, adrv906x_dev->port, true);
		}
		adrv906x_ndma_open(adrv906x_dev->ndma_dev);
	} else {
		adrv906x_ndma_close(adrv906x_dev->ndma_dev, ndev);
		if (es->enabled) {
			adrv906x_switch_port_enable(es, other_port, false);
			adrv906x_switch_port_enable(es, adrv906x_dev->port, false);
		}
		adrv906x_mac_set_path(mac, false);
		phy_loopback(phydev, false);
	}

	return 0;
}

static int adrv906x_ndma_loopback_config(struct net_device *ndev, bool enable)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_ndma_dev *ndma_dev = adrv906x_dev->ndma_dev;
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	struct adrv906x_eth_switch *es = &eth_if->ethswitch;
	struct adrv906x_mac *mac = &adrv906x_dev->mac;
	struct phy_device *phydev = ndev->phydev;

	if (enable) {
		adrv906x_ndma_config_loopback(ndma_dev, enable);

		/* When NDMA loopback is enabled, also enable PHY loopback to prevent
		 * communication with the SerDes application. Additionally, block all NDMA
		 * egress traffic by disabling the switch CPU port or the MAC data path,
		 * depending on the current configuration.
		 */
		if (es->enabled)
			adrv906x_switch_port_enable(es, SWITCH_CPU_PORT, !enable);
		else
			adrv906x_mac_set_path(mac, !enable);

		phy_loopback(phydev, enable);
		adrv906x_ndma_open(adrv906x_dev->ndma_dev);
	} else {
		adrv906x_ndma_config_loopback(ndma_dev, enable);
		adrv906x_ndma_close(adrv906x_dev->ndma_dev, ndev);

		if (es->enabled)
			adrv906x_switch_port_enable(es, SWITCH_CPU_PORT, !enable);
		else
			adrv906x_mac_set_path(mac, !enable);

		phy_loopback(phydev, enable);
	}

	return 0;
}

static int adrv906x_loopback_test_common(struct net_device *ndev,
					 int (*config_loopback)(struct net_device *, bool))
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	struct adrv906x_eth_switch *es = &eth_if->ethswitch;
	struct adrv906x_loopback_test_attrs attr = { };
	int ret;

	/* Enable loopback mode */
	config_loopback(ndev, true);
	ndev->netdev_ops->ndo_open(ndev);
	if (eth_if->ethswitch.enabled)
		adrv906x_switch_port_reset(es);

	msleep(100);

	/* Run the loopback test */
	attr.dst = ndev->dev_addr;
	attr.timeout = ADRV906X_LB_TIMEOUT;
	ret = adrv906x_test_loopback_run(ndev, &attr);

	/* Disable loopback mode */
	if (eth_if->ethswitch.enabled)
		adrv906x_switch_port_reset(es);
	ndev->netdev_ops->ndo_stop(ndev);
	config_loopback(ndev, false);

	msleep(100);

	return ret;
}

static int adrv906x_phy_loopback_test(struct net_device *ndev)
{
	int ret;

	ret = adrv906x_loopback_test_common(ndev, adrv906x_phy_loopback_config);
	netdev_printk(KERN_DEBUG, ndev, "near-end loopback test result: %d", ret);

	return ret;
}

static int adrv906x_ndma_loopback_test(struct net_device *ndev)
{
	int ret;

	ret = adrv906x_loopback_test_common(ndev, adrv906x_ndma_loopback_config);
	netdev_printk(KERN_DEBUG, ndev, "ndma loopback test result: %d", ret);

	return ret;
}

struct adrv906x_test adrv906x_ethtool_selftests[] = {
	{
		.name = "NDMA loopback",
		.fn = adrv906x_ndma_loopback_test,
		.etest_flag = ETH_TEST_FL_OFFLINE,
	},
	{
		.name = "Near-end loopback",
		.fn = adrv906x_phy_loopback_test,
		.etest_flag = ETH_TEST_FL_OFFLINE,
	},
};

static void adrv906x_ethtool_selftest_run(struct net_device *ndev, struct ethtool_test *etest,
					  u64 *buf)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_ndma_dev *ndma_dev = adrv906x_dev->ndma_dev;
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	bool dev_was_running[MAX_NETDEV_NUM] = {false};
	bool check_all_interfaces = false;
	struct net_device *temp_ndev;
	unsigned char etest_flags = etest->flags;
	int port = adrv906x_dev->port;
	int i, ret;

	/* Check if multiple interfaces share the NDMA device */
	check_all_interfaces = eth_if->ethswitch.enabled && kref_read(&ndma_dev->refcount) > 1;

	/* Stop all running interfaces before tests */
	if (check_all_interfaces) {
		for (i = 0; i < MAX_NETDEV_NUM; i++) {
			temp_ndev = eth_if->adrv906x_dev[i]->ndev;
			dev_was_running[i] = netif_running(temp_ndev);
			if (dev_was_running[i])
				ndev->netdev_ops->ndo_stop(temp_ndev);
		}
	} else {
		dev_was_running[port] = netif_running(ndev);
		if (dev_was_running[port])
			ndev->netdev_ops->ndo_stop(ndev);
	}

	msleep(1500);

	/* Run all tests */
	for (i = 0; i < ARRAY_SIZE(adrv906x_ethtool_selftests); i++) {
		ret = 1;
		if (etest_flags == adrv906x_ethtool_selftests[i].etest_flag) {
			ret = adrv906x_ethtool_selftests[i].fn(ndev);
			if (ret)
				etest->flags |= ETH_TEST_FL_FAILED;
		}
		buf[i] = ret;
	}

	/* Restore previously running interfaces */
	if (check_all_interfaces) {
		for (i = 0; i < MAX_NETDEV_NUM; i++) {
			if (dev_was_running[i]) {
				temp_ndev = eth_if->adrv906x_dev[i]->ndev;
				ndev->netdev_ops->ndo_open(temp_ndev);
			}
		}
	} else {
		if (dev_was_running[port])
			ndev->netdev_ops->ndo_open(ndev);
	}

	msleep(1500);
}

const struct ethtool_ops adrv906x_ethtool_ops = {
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= adrv906x_ethtool_set_link_ksettings,
	.get_fec_stats		= adrv906x_phy_get_fec_stats,
	.get_fecparam		= adrv906x_ethtool_get_fecparam,
	.set_fecparam		= adrv906x_ethtool_set_fecparam,
	.get_ts_info		= adrv906x_ethtool_get_ts_info,
	.get_sset_count		= adrv906x_ethtool_get_sset_count,
	.get_strings		= adrv906x_ethtool_get_strings,
	.get_ethtool_stats	= adrv906x_ethtool_get_stats,
	.get_rmon_stats		= adrv906x_ethtool_get_rmon_stats,
	.self_test		= adrv906x_ethtool_selftest_run,
};

MODULE_LICENSE("GPL");
