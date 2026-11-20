/****************************************************************************
 *
 * Copyright (c) 2014 - 2022 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/
#include <linux/types.h>
#include "debug.h"
#include "dev.h"
#include "sap.h"
#include "sap_ma.h"
#include "hip.h"
#include "ba.h"
#include "mgt.h"
#include "utils.h"
#include "nl80211_vendor.h"
#include "hip4_sampler.h"
#include "traffic_monitor.h"
#include "tdls_manager.h"

#ifdef CONFIG_SCSC_WLAN_ANDROID
#include "scsc_wifilogger_rings.h"
#endif

#ifdef CONFIG_SCSC_WLAN_TX_API
#include "tx_api.h"
#endif

#ifdef CONFIG_SCSC_WLAN_LOAD_BALANCE_MANAGER
#include "load_manager.h"
#endif

#ifdef CONFIG_SCSC_WLAN_LPC
#include "local_packet_capture.h"
#endif

#ifdef CONFIG_SCSC_WLAN_WONDERTAP
#include <wonder/wondertap.h>
#include <net/ieee80211_radiotap.h>
#endif

#define SUPPORTED_OLD_VERSION   0

/* Number of bytes of the last delayed RX packet to capture for debug. The
 * backing buffer is MAX_LAST_DELAYD_PKT_SIZE; larger values are clamped.
 */
static uint dbg_last_delayd_pkt_capture_len = MAX_LAST_DELAYD_PKT_SIZE;
module_param(dbg_last_delayd_pkt_capture_len, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dbg_last_delayd_pkt_capture_len,
		 "Bytes of the last delayed RX packet to capture (max/default 86)");

static bool wondertap_amsdu_deaggregate;
module_param(wondertap_amsdu_deaggregate, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(wondertap_amsdu_deaggregate, "deaggregate wondertap A-MSDU into per-subframe skbs, 1: deaggregate, 0: pass the whole A-MSDU up (default)");

/* Only consulted when the A-MSDU is passed up whole (deaggregate=0, or a frame
 * that cannot be deaggregated). Selects how the whole frame is delivered:
 * frag_list intact (default) vs. copied into one linear jumbo skb.
 */
static bool wondertap_amsdu_linearize;
module_param(wondertap_amsdu_linearize, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(wondertap_amsdu_linearize, "when passing a whole A-MSDU up, copy it into one linear jumbo skb; 0: keep the RX frag_list intact (default), 1: linearize into a jumbo skb");

/* Scratch buffer size for building an ethernet header view used by RX wakeup
 * stats: ethhdr(14) + ipv6hdr(40) + icmp6hdr(8) rounded up.
 */
#define SLSI_WAKE_STATS_HDR_LEN 64

static int sap_ma_version_supported(u16 version);
static int sap_ma_rx_handler(struct slsi_dev *sdev, struct sk_buff *skb);
#ifdef CONFIG_SCSC_WLAN_TX_API
static int sap_ma_txdone(struct slsi_dev *sdev, u32 colour, bool more);
#else
static int sap_ma_txdone(struct slsi_dev *sdev,  u8 vif, u8 peer_index, u8 ac);
#endif
static int sap_ma_notifier(struct slsi_dev *sdev, unsigned long event);

static DEFINE_SPINLOCK(sap_ma_lock);

void slsi_sap_ma_lock(void)
{
	spin_lock_bh(&sap_ma_lock);
}

void slsi_sap_ma_unlock(void)
{
	spin_unlock_bh(&sap_ma_lock);
}

static struct sap_api sap_ma = {
	.sap_class = SAP_MA,
	.sap_version_supported = sap_ma_version_supported,
	.sap_handler = sap_ma_rx_handler,
	.sap_versions = { FAPI_DATA_SAP_VERSION, SUPPORTED_OLD_VERSION },
	.sap_txdone = sap_ma_txdone,
	.sap_notifier = sap_ma_notifier,
};

static int sap_ma_notifier(struct slsi_dev *sdev, unsigned long event)
{
	uint vif;

	SLSI_INFO_NODEV("Notifier event received: %lu\n", event);
	if (event >= SCSC_MAX_NOTIFIER)
		return -EIO;

	switch (event) {
	case SCSC_WIFI_STOP:
		SLSI_INFO_NODEV("Stop netdev queues\n");
		rcu_read_lock();
		for (vif = SLSI_NET_INDEX_WLAN;
		     vif <= SLSI_NET_INDEX_P2PX_SWLAN; vif++) {
			struct net_device *ndev =
				slsi_get_netdev_rcu(sdev, vif);
			if (ndev && !netif_queue_stopped(ndev))
				netif_tx_stop_all_queues(ndev);
		}
		rcu_read_unlock();
		break;

	case SCSC_WIFI_FAILURE_RESET:
		SLSI_DBG1_NODEV(SLSI_NETDEV, "Netdevs queues will not be restarted - recovery will take care of it\n");
		break;

	case SCSC_WIFI_SUSPEND:
		break;

	case SCSC_WIFI_RESUME:
		break;
	case SCSC_WIFI_CHIP_READY:
		break;
	case SCSC_WIFI_SUBSYSTEM_RESET:
		break;
	default:
		SLSI_INFO_NODEV("Unknown event code %lu\n", event);
		break;
	}

	return 0;
}

static int sap_ma_version_supported(u16 version)
{
	unsigned int major = SAP_MAJOR(version);
	unsigned int minor = SAP_MINOR(version);
	u8           i = 0;

	SLSI_INFO_NODEV("Reported version: %d.%d\n", major, minor);

	for (i = 0; i < SAP_MAX_VER; i++)
		if (SAP_MAJOR(sap_ma.sap_versions[i]) == major)
			return 0;

	SLSI_ERR_NODEV("Version %d.%d Not supported\n", major, minor);

	return -EINVAL;
}

static bool slsi_rx_check_mc_addr_regd(struct slsi_dev *sdev, struct net_device *dev, struct ethhdr *ehdr)
{
	int i;
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	slsi_spinlock_lock(&ndev_vif->sta.regd_mc_addr_lock);

	for (i = 0 ; i < ndev_vif->sta.regd_mc_addr_count; i++)
		if (memcmp(ehdr->h_dest, ndev_vif->sta.regd_mc_addr[i], ETH_ALEN) == 0) {
			SLSI_INFO(sdev, "Wakeup by regd mc addr " MACSTR "\n", MAC2STR(ndev_vif->sta.regd_mc_addr[i]));
			slsi_spinlock_unlock(&ndev_vif->sta.regd_mc_addr_lock);
			return true;
		}

	SLSI_ERR(sdev, "Wakeup by unregistered mc address\n");
	SLSI_INFO(sdev, "Received packet source : " MACSTR ", dest : " MACSTR "\n", MAC2STR(ehdr->h_source), MAC2STR(ehdr->h_dest));
	SLSI_INFO(sdev, "Regd mc addr : \n");
	for (i = 0 ; i < ndev_vif->sta.regd_mc_addr_count; i++)
		SLSI_INFO(sdev, "    " MACSTR "\n", MAC2STR(ndev_vif->sta.regd_mc_addr[i]));

	slsi_spinlock_unlock(&ndev_vif->sta.regd_mc_addr_lock);
	return false;
}

int slsi_rx_amsdu_deaggregate_gro(struct net_device *dev, struct sk_buff *skb, struct sk_buff_head *msdu_list)
{
	struct slsi_skb_cb *skb_cb = slsi_skb_cb_get(skb);
	unsigned int data_len = 0;
	unsigned int subframe_len = 0;
	unsigned char padding = 0;
	struct sk_buff *subframe = NULL;
	const unsigned char mac_0[ETH_ALEN] = { 0 };
	struct ethhdr *mh = NULL;
	bool discard;

	SLSI_NET_DBG4(dev, SLSI_RX, "A-MSDU received (len:%d)\n", skb->len);
	while (skb->len) {
		/* MSDU format.
		 * MSDU is 4 byte aligned execpt for the last one, i.e., the last frame does not have padding.
		 * Dst: Destination MAC
		 * Src: Source MAC
		 * Len: Data length
		 * SNAP->ETH_TYPE: EtherType
		 * +-----+-----+--------+-----------------------------+----------------------+------+-----+
		 * | Dst | Src |   Len  |             LLC             |         SNAP         | Data | Pad |
		 * | (6) | (6) |   (2)  | DSAP(1) | SSAP(1) | Ctrl(1) | OUI(3) | ETH_TYPE(2) |      |     |
		 * +-----+-----+--------+-----------------------------+----------------------+------+-----+
		 *             ^msdu_len^
		 */
		if (unlikely((ETH_ALEN * 2 + 2) > skb->len)) {
			SLSI_NET_ERR(dev, "invalid SKB length %d < %d\n", skb->len, (ETH_ALEN * 2 + 2));
			__skb_queue_purge(msdu_list);
			kfree_skb(skb);
			return -EINVAL;
		}

		data_len = (skb->data[ETH_ALEN * 2] << 8) | skb->data[(ETH_ALEN * 2) + 1];
		if (unlikely(data_len <= LLC_SNAP_HDR_LEN)) {
			SLSI_NET_ERR(dev, "invalid data length %d < %d\n", data_len, LLC_SNAP_HDR_LEN);
			__skb_queue_purge(msdu_list);
			kfree_skb(skb);
			return -EINVAL;
		}
		data_len -= LLC_SNAP_HDR_LEN;
		/* check if the length of sub-frame is valid.
		 * <---------------------(ETH_ALEN * 2 + 2 + 8 + data_len)---------------------->
		 * +-----+-----+-----+-----------------------------+----------------------+------+-----+
		 * | Dst | Src | Len |             LLC             |         SNAP         | Data | Pad |
		 * | (6) | (6) | (2) | DSAP(1) | SSAP(1) | Ctrl(1) | OUI(3) | ETH_TYPE(2) |      |     |
		 * +-----+-----+-----+-----------------------------+----------------------+------+-----+
		 */
		if (unlikely(ETH_ALEN * 2 + 2 + LLC_SNAP_HDR_LEN + data_len > skb->len)) {
			SLSI_NET_ERR(dev, "invalid MSDU data length %d, SKB length = %d\n", data_len, skb->len);
			__skb_queue_purge(msdu_list);
			kfree_skb(skb);
			return -EINVAL;
		}
		/* We convert MSDU to Ethernet II format.
		 * <--------subframe_len--------->
		 * <---(struct ethhdr)--->
		 * +-----+-----+----------+------+
		 * | Dst | Src | ETH_TYPE | Data |
		 * | (6) | (6) |   (2)    |      |
		 * +-----+-----+----------+------+
		 */
		subframe_len = sizeof(struct ethhdr) + data_len;

		discard = (skb_cb->discard & 0x1 ? true : false);
		skb_cb->discard = skb_cb->discard >> 1;
		/* handle last subframe */
		if (skb->len == subframe_len + LLC_SNAP_HDR_LEN) {
			if (discard) {
				kfree_skb(skb);
				return 0;
			}

			SLSI_ETHER_COPY(&skb->data[14], &skb->data[6]);
			SLSI_ETHER_COPY(&skb->data[8], &skb->data[0]);

			skb_pull(skb, LLC_SNAP_HDR_LEN);

			skb_set_mac_header(skb, 0);
			mh = eth_hdr(skb);
			if (SLSI_ETHER_EQUAL(mh->h_dest, mac_0)) {
				SLSI_NET_DBG3(dev, SLSI_RX, "msdu subframe filtered out: MAC destination address %pM\n", mh->h_dest);
				kfree_skb(skb);
			} else {
				__skb_queue_tail(msdu_list, skb);
			}

			return 0;
		}

		padding = (4 - (subframe_len % 4)) & 0x3;
		if (discard) {
			skb_pull(skb, LLC_SNAP_HDR_LEN + subframe_len + padding);
			continue;
		}
		/* Allocate subframe skbuff */
		subframe = alloc_skb(subframe_len, GFP_ATOMIC);
		if (unlikely(!subframe)) {
			SLSI_NET_ERR(dev, "failed to alloc the SKB for A-MSDU subframe\n");
			__skb_queue_purge(msdu_list);
			kfree_skb(skb);
			return -ENOMEM;
		}
		/* Copy Destination and Source MAC
		 * +-----+-----+-----+-----------------------------+----------------------+------+-----+
		 * | Dst | Src | Len |             LLC             |         SNAP         | Data | Pad |
		 * | (6) | (6) | (2) | DSAP(1) | SSAP(1) | Ctrl(1) | OUI(3) | ETH_TYPE(2) |      |     |
		 * +-----+-----+-----+-----------------------------+----------------------+------+-----+
		 * ^
		 * skb->data
		 * +-----+-----+
		 * | Dst | Src |
		 * | (6) | (6) |
		 * +-----+-----+
		 * ^
		 * skb_put(subframe, ETH_ALEN * 2)
		 */
		skb_put_data(subframe, skb->data, ETH_ALEN * 2);
		skb_pull(skb, ETH_ALEN * 2);
		/* Skip MSDU length field, LLC header and SNAP OUI field
		 * +-----+-----+-----+-----------------------------+----------------------+------+-----+
		 * | Dst | Src | Len |             LLC             |         SNAP         | Data | Pad |
		 * | (6) | (6) | (2) | DSAP(1) | SSAP(1) | Ctrl(1) | OUI(3) | ETH_TYPE(2) |      |     |
		 * +-----+-----+-----+-----------------------------+----------------------+------+-----+
		 *                                                          ^
		 *             <-------------------(8)--------------------->skb->data
		 */
		skb_pull(skb, 8);
		/* Copy ETH_TYPE.
		 * +-----+-----+-----+-----------------------------+----------------------+------+-----+
		 * | Dst | Src | Len |             LLC             |         SNAP         | Data | Pad |
		 * | (6) | (6) | (2) | DSAP(1) | SSAP(1) | Ctrl(1) | OUI(3) | ETH_TYPE(2) |      |     |
		 * +-----+-----+-----+-----------------------------+----------------------+------+-----+
		 *                                                          ^
		 *                                                          skb->data
		 * +-----+-----+----------+
		 * | Dst | Src | ETH_TYPE |
		 * | (6) | (6) |   (2)    |
		 * +-----+-----+----------+
		 *             ^
		 *             skb_put(subframe, 2)
		 */
		skb_put_data(subframe, skb->data, 2);
		skb_pull(skb, 2);
		/* Copy data.
		 * +-----+-----+-----+-----------------------------+----------------------+------+-----+
		 * | Dst | Src | Len |             LLC             |         SNAP         | Data | Pad |
		 * | (6) | (6) | (2) | DSAP(1) | SSAP(1) | Ctrl(1) | OUI(3) | ETH_TYPE(2) |      |     |
		 * +-----+-----+-----+-----------------------------+----------------------+------+-----+
		 *                                                                        ^
		 *                                                                        skb->data
		 * +-----+-----+----------+------+
		 * | Dst | Src | ETH_TYPE | Data |
		 * | (6) | (6) |   (2)    |      |
		 * +-----+-----+----------+------+
		 *                        ^
		 *                        skb_put(subframe, data_len)
		 */
		skb_put_data(subframe, skb->data, data_len);
		skb_pull(skb, data_len);

		/* Find padding size and skip the padding.
		 * +----------+-----+-------
		 * | (n)-MSDU | Pad | (n+1)-MSDU
		 * |          |     |
		 * +----------+-----+-------
		 *                  ^
		 *                  skb->data
		 */
		if (unlikely(!skb_pull(skb, padding))) {
			kfree_skb(subframe);
			SLSI_NET_ERR(dev, "Invalid A-MSDU Padding\n");
			__skb_queue_purge(msdu_list);
			kfree_skb(skb);
			return -EINVAL;
		}
		/* Before preparing the skb, filter out if the Destination Address of the Ethernet frame
		 * or A-MSDU subframe is set to an invalid value, i.e. all zeros
		 */
		skb_set_mac_header(subframe, 0);
		mh = eth_hdr(subframe);
		if (unlikely(SLSI_ETHER_EQUAL(mh->h_dest, mac_0))) {
			SLSI_NET_DBG3(dev, SLSI_RX, "msdu subframe filtered out: MAC destination address %pM\n", mh->h_dest);
			kfree_skb(subframe);
			continue;
		}
		__skb_queue_tail(msdu_list, subframe);
	}
	return 0;
}

static inline unsigned int skb_len(struct sk_buff *skb)
{
	return (unsigned int)(skb_tail_pointer(skb) - skb->data);
}

/**
 * skb_put_data_frag - Copy data into msdu subframe from given skb
 * @subframe: MSDU frame to be written
 * @skb_pointer: Address of source skb
 * @write_len: Length of the data to write
 *
 * This function copies a specified fragment from the source sk_buff into MSDU subframe
 * which should passed to Upper layer. It handles both linear and non-linear (fragmented)
 * skb data. When skb's length is not enough to handle write_len, skb is replaced with
 * the next fragment (skb->next). Using sk_buf **skb_pointer, the skb replacement is valid
 * even after the function returned.
 *
 * Return: 0 on success
 *         -EINVAL on failure (when write_len is still left, but there is no more frag skb following)
 */
static int skb_put_data_frag(struct sk_buff *subframe, struct sk_buff **skb_pointer, unsigned int write_len)
{
	struct sk_buff *skb = *skb_pointer;

	if (skb_len(skb) >= write_len) {
		skb_put_data(subframe, skb->data, write_len);
		skb_pull(skb, write_len);
	} else {
		write_len -= skb_len(skb);
		skb_put_data(subframe, skb->data, skb_len(skb));

		if (!skb->next)
			return -EINVAL;
		*skb_pointer = skb->next;

		skb = *skb_pointer;
		skb_put_data(subframe, skb->data, write_len);
		skb_pull(skb, write_len);
	}
	return 0;
}

#ifdef CONFIG_SCSC_WLAN_WONDERTAP
/* Radiotap fields: TSFT(8) + Flags(1) + pad(1) + Channel(4) + Signal(1) + Noise(1) + Antenna(1) */
#define RADIOTAP_WONDERTAP_EXTRA_LEN (8 + 1 + 1 + 4 + 1 + 1 + 1)
#define WONDER_DUMMY_FCS_LEN 4
static int slsi_rx_wondertap_add_radiotap_header(struct net_device *dev, struct sk_buff **skb_pointer, s16 rssi, u32 timestamp, u8 *rtap_buf);
#endif

int slsi_rx_amsdu_deaggregate(struct net_device *dev, struct sk_buff *skb, struct sk_buff_head *msdu_list)
{
	struct slsi_skb_cb *skb_cb = slsi_skb_cb_get(skb);
	unsigned int data_len = 0;
	unsigned int subframe_len = 0;
	unsigned char padding = 0;
	struct sk_buff *subframe = NULL;
	const unsigned char mac_0[ETH_ALEN] = { 0 };
	struct ethhdr *mh = NULL;
	bool discard;
	bool is_last = false;

#ifdef CONFIG_SCSC_WLAN_WONDERTAP
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	unsigned char eth_buf[ETH_ALEN * 2 + 2] = { 0 };
	struct sk_buff *skb_free_all = skb;
	/* 802.11 header can be up to 36 bytes (4-addr + QoS + HT Control) */
	struct fapi_signal *fapi_hdr = (struct fapi_signal *)(skb->data - skb_cb->sig_length);
	s16 w_rssi = (s16)le16_to_cpu(fapi_hdr->u.ma_unitdata_ind.rssi);
	u32 w_timestamp = le32_to_cpu(fapi_hdr->u.ma_unitdata_ind.timestamp);
	int rtap_len = sizeof(struct ieee80211_radiotap_header) + RADIOTAP_WONDERTAP_EXTRA_LEN;
	u8 hdr_buf[sizeof(struct ieee80211_hdr) + IEEE80211_QOS_CTL_LEN + IEEE80211_HT_CTL_LEN];
	u8 rtap_buf[sizeof(struct ieee80211_radiotap_header) + RADIOTAP_WONDERTAP_EXTRA_LEN];
	int hdrlen = 0;
	int buf_len = 0;

	if (ndev_vif->ifnum == SLSI_NET_INDEX_RELAY) {
		struct ieee80211_hdr *hdr;

		/* Prepend [radiotap] */
		if (w_rssi > 0)
			w_rssi -= 1;
		else if (w_rssi < 0)
			w_rssi += 1;
		slsi_rx_wondertap_add_radiotap_header(dev, &skb, w_rssi, w_timestamp, rtap_buf);

		hdr = (struct ieee80211_hdr *)skb->data;
		hdrlen = ieee80211_hdrlen(hdr->frame_control);
		buf_len = rtap_len + hdrlen;
		memcpy(hdr_buf, hdr, hdrlen);
		/* A-MSDU is a single MPDU carrying one sequence number. When we
		 * deaggregate it into N subframes, the same 802.11 header (hence
		 * the same SN) is copied onto every subframe below. If the source
		 * MPDU was a retransmission (Retry=1), the far-end 802.11 duplicate
		 * filter keys on (Addr2, TID, SN, Retry) and discards every subframe
		 * after the first as a duplicate, punching a hole in the stream that
		 * shows up as TCP dup-ACKs / retransmissions. Clear Retry on the
		 * shared header copy so the subframes are not treated as duplicates.
		 */
		((struct ieee80211_hdr *)hdr_buf)->frame_control &= ~cpu_to_le16(IEEE80211_FCTL_RETRY);
		skb_pull(skb, hdrlen);
	}
#endif

	SLSI_NET_DBG4(dev, SLSI_RX, "A-MSDU received (len:%d)\n", skb->len);
	while (skb->len) {
		/* MSDU format.
		 * MSDU is 4 byte aligned execpt for the last one, i.e., the last frame does not have padding.
		 * Dst: Destination MAC
		 * Src: Source MAC
		 * Len: Data length
		 * SNAP->ETH_TYPE: EtherType
		 * +-----+-----+--------+-----------------------------+----------------------+------+-----+
		 * | Dst | Src |   Len  |             LLC             |         SNAP         | Data | Pad |
		 * | (6) | (6) |   (2)  | DSAP(1) | SSAP(1) | Ctrl(1) | OUI(3) | ETH_TYPE(2) |      |     |
		 * +-----+-----+--------+-----------------------------+----------------------+------+-----+
		 *             ^msdu_len^
		 */
		if (unlikely((ETH_ALEN * 2 + 2) > skb->len)) {
			SLSI_NET_ERR(dev, "invalid SKB length %d < %d\n", skb->len, (ETH_ALEN * 2 + 2));
			__skb_queue_purge(msdu_list);
			kfree_skb(skb);
			return -EINVAL;
		}

		data_len = (skb->data[ETH_ALEN * 2] << 8) | skb->data[(ETH_ALEN * 2) + 1];
		if (unlikely(data_len <= LLC_SNAP_HDR_LEN)) {
			SLSI_NET_ERR(dev, "invalid data length %d < %d\n", data_len, LLC_SNAP_HDR_LEN);
			__skb_queue_purge(msdu_list);
			kfree_skb(skb);
			return -EINVAL;
		}
		data_len -= LLC_SNAP_HDR_LEN;
		/* check if the length of sub-frame is valid.
		 * <---------------------(ETH_ALEN * 2 + 2 + 8 + data_len)---------------------->
		 * +-----+-----+-----+-----------------------------+----------------------+------+-----+
		 * | Dst | Src | Len |             LLC             |         SNAP         | Data | Pad |
		 * | (6) | (6) | (2) | DSAP(1) | SSAP(1) | Ctrl(1) | OUI(3) | ETH_TYPE(2) |      |     |
		 * +-----+-----+-----+-----------------------------+----------------------+------+-----+
		 */
		if (unlikely(ETH_ALEN * 2 + 2 + LLC_SNAP_HDR_LEN + data_len > skb->len)) {
			SLSI_NET_ERR(dev, "invalid MSDU data length %d, SKB length = %d\n", data_len, skb->len);
			__skb_queue_purge(msdu_list);
			kfree_skb(skb);
			return -EINVAL;
		}

		/* We convert MSDU to Ethernet II format.
		 * <--------subframe_len--------->
		 * <---(struct ethhdr)--->
		 * +-----+-----+----------+------+
		 * | Dst | Src | ETH_TYPE | Data |
		 * | (6) | (6) |   (2)    |      |
		 * +-----+-----+----------+------+
		 */
		subframe_len = sizeof(struct ethhdr) + data_len;
		discard = (skb_cb->discard & 0x1 ? true : false);
		skb_cb->discard = skb_cb->discard >> 1;

#ifdef CONFIG_SCSC_WLAN_WONDERTAP
		if (ndev_vif->ifnum == SLSI_NET_INDEX_RELAY) {
			/* handle last subframe */
			if (skb->len == subframe_len + LLC_SNAP_HDR_LEN) {
				if (discard) {
					if (skb_has_frag_list(skb_free_all))
						skb_free_all->next = NULL;
					kfree_skb(skb_free_all);
					return 0;
				}

				padding = 0;
				is_last = true;
			} else {
				padding = (4 - (subframe_len % 4)) & 0x3;
				if (discard) {
					skb_pull(skb, LLC_SNAP_HDR_LEN + subframe_len + padding);
					continue;
				}
			}

			subframe = alloc_skb(buf_len + subframe_len + LLC_SNAP_HDR_LEN + WONDER_DUMMY_FCS_LEN,
					     GFP_ATOMIC);
			if (unlikely(!subframe)) {
				SLSI_NET_ERR(dev, "[Wonder] failed to alloc the SKB for A-MSDU subframe\n");
				__skb_queue_purge(msdu_list);
				if (skb_has_frag_list(skb_free_all))
					skb_free_all->next = NULL;
				kfree_skb(skb_free_all);
				return -ENOMEM;
			}
			/* Consume the 14 byte subframe header [Dst|Src|Len] so the payload
			 * copy below starts at the LLC/SNAP, and keep the real Dst/Src MACs
			 * in eth_buf for the wondertap subframe below.
			 */
			memcpy(eth_buf, skb->data, ETH_ALEN * 2 + 2);
			skb_pull(skb, ETH_ALEN * 2 + 2);
			/* For wondertap: build [radiotap][802.11 hdr][LLC/SNAP + Data]
			 * Keep the full LLC/SNAP header as part of the 802.11 frame body.
			 */
			skb_put_data(subframe, rtap_buf, rtap_len);
			skb_put_data(subframe, hdr_buf, hdrlen);
			skb_put_data(subframe, eth_buf, ETH_ALEN * 2);
			/* Copy MSDU length */
			{
				__be16 msdu_len = htons(data_len + LLC_SNAP_HDR_LEN);
				skb_put_data(subframe, &msdu_len, 2);
			}
			/* Copy LLC/SNAP (8 bytes) + Data */
			if (unlikely(skb_put_data_frag(subframe, &skb, LLC_SNAP_HDR_LEN + data_len))) {
				kfree_skb(subframe);
				SLSI_NET_ERR(dev, "[Wonder] Invalid A-MSDU DATA\n");
				__skb_queue_purge(msdu_list);
				if (skb_has_frag_list(skb_free_all))
					skb_free_all->next = NULL;
				kfree_skb(skb_free_all);
				return -EINVAL;
			}
			skb_set_mac_header(subframe, 0);
			/* Add dummy FCS */
			memset(skb_put(subframe, WONDER_DUMMY_FCS_LEN), 0, WONDER_DUMMY_FCS_LEN);
			slsi_skb_cb_init(subframe);
			memcpy(subframe->cb, skb_cb, sizeof(struct slsi_skb_cb));
			__skb_queue_tail(msdu_list, subframe);

			if (is_last) {
				/* The last subframe drained the source skb; free it. */
				if (skb_has_frag_list(skb_free_all))
					skb_free_all->next = NULL;
				kfree_skb(skb_free_all);
				return 0;
			}

			/* Advance skb->data to the next subframe: the 14 byte header and
			 * the LLC/SNAP + Data have already been consumed above, so only
			 * the 4-byte alignment padding is left.
			 */
			if (padding && unlikely(!skb_pull(skb, padding))) {
				SLSI_NET_ERR(dev, "[Wonder] Invalid A-MSDU Padding\n");
				__skb_queue_purge(msdu_list);
				if (skb_has_frag_list(skb_free_all))
					skb_free_all->next = NULL;
				kfree_skb(skb_free_all);
				return -EINVAL;
			}
			continue;
		}
#endif

		/* handle last subframe */
		if (skb->len == subframe_len + LLC_SNAP_HDR_LEN) {
			if (discard) {
				kfree_skb(skb);
				return 0;
			}

			subframe = skb;
			padding = 0;
			is_last = true;
		} else {
			padding = (4 - (subframe_len % 4)) & 0x3;
			if (discard) {
				skb_pull(skb, LLC_SNAP_HDR_LEN + subframe_len + padding);
				continue;
			}

			subframe = skb_clone(skb, GFP_ATOMIC);
			if (unlikely(!subframe)) {
				SLSI_NET_ERR(dev, "failed to clone SKB for A-MSDU subframe\n");
				__skb_queue_purge(msdu_list);
				kfree_skb(skb);
				return -ENOMEM;
			}
		}

		/* set tail pointer of cloned skb
		 * +-----+-----+-----+-----+-------------+-----+------+-----+------+-----+
		 * | Dst | Src | Len | LLC | SNAP | Data | Pad |  subframe  |  subframe  |
		 * | (6) | (6) | (2) | (3) |  (5) |      |     |            |            |
		 * +-----+-----+-----+-----+-------------+-----+------+-----+------+-----+
		 *                                       ^
		 *                                       skb_trim(subframe, subframe_len + LLC_SNAP_HDR_LEN)
		 */
		skb_trim(subframe, subframe_len + LLC_SNAP_HDR_LEN);

		/* Cut LLC/OUI and Set Destination and Source MAC, set subframe->dat and skb->data
		 * +-----+-----+-----+-----+-------------+-----+
		 * | Dst | Src | Len | LLC | SNAP | Data | Pad |
		 * | (6) | (6) | (2) | (3) |  (5) |      |     |
		 * +-----+-----+-----+-----+-------------+-----+
		 *         +-----+-----+----------+------+-----+
		 *         | Dst | Src | ETH_TYPE | Data | Pad |
		 *         | (6) | (6) |   (2)    |      |     |
		 *         +-----+-----+----------+------+-----+
		 *         ^
		 *         skb_pull(subframe, LLC_SNAP_HDR_LEN);
		 *         skb_pull(skb, LLC_SNAP_HDR_LEN);
		 */
		SLSI_ETHER_COPY(&subframe->data[14], &subframe->data[6]);
		SLSI_ETHER_COPY(&subframe->data[8], &subframe->data[0]);
		skb_pull(subframe, LLC_SNAP_HDR_LEN);

		/* Before preparing the skb, filter out if the Destination Address of the Ethernet frame
		 * or A-MSDU subframe is set to an invalid value, i.e. all zeros
		 */
		skb_set_mac_header(subframe, 0);
		mh = eth_hdr(subframe);
		if (unlikely(SLSI_ETHER_EQUAL(mh->h_dest, mac_0))) {
			SLSI_NET_DBG3(dev, SLSI_RX, "msdu subframe filtered out: MAC destination address %pM\n", mh->h_dest);
			skb_pull(skb, LLC_SNAP_HDR_LEN + subframe_len + padding);
			kfree_skb(subframe);
			/* The last subframe is processed in place (subframe == skb), so
			 * the source skb is already freed here; do not fall through to
			 * the cb init/queueing below with a freed skb.
			 */
			if (is_last)
				return 0;
			continue;
		}
		slsi_skb_cb_init(subframe);
		memcpy(subframe->cb, skb_cb, sizeof(struct slsi_skb_cb));
		__skb_queue_tail(msdu_list, subframe);


		if (is_last)
			return 0;

		/* Set skb->data to next subframe
		 * +-----+-----+-----+-----+-------------+-----+------+-----+------+-----+
		 * | Dst | Src | Len | LLC | SNAP | Data | Pad |  subframe  |  subframe  |
		 * | (6) | (6) | (2) | (3) |  (5) |      |     |            |            |
		 * +-----+-----+-----+-----+-------------+-----+------+-----+------+-----+
		 *                                             ^
		 *                                             skb_pull(skb, subframe_len)
		 */
		if (unlikely(!skb_pull(skb, LLC_SNAP_HDR_LEN + subframe_len + padding))) {
			SLSI_NET_ERR(dev, "Invalid A-MSDU Padding\n");
			__skb_queue_purge(msdu_list);
			kfree_skb(skb);
			return -EINVAL;
		}
	}
	return 0;
}

static bool is_unicast_mac_with_multicst_ip(struct sk_buff *skb)
{
	struct ethhdr *ehdr = (struct ethhdr *)(skb->data);

	if (is_unicast_ether_addr(ehdr->h_dest)) {
		if (be16_to_cpu(ehdr->h_proto) == ETH_P_IP) {
			struct iphdr *iph = (struct iphdr *)(skb->data + sizeof(struct ethhdr));
			u8 *dst = (u8 *)&iph->daddr;

			if ((dst[0] & 0xf0) == 0xe0) /* iphdr->daddr starts with 1110 */
				return true;
		} else if (be16_to_cpu(ehdr->h_proto) == ETH_P_IPV6) {
			struct ipv6hdr *iph = (struct ipv6hdr *)(skb->data + sizeof(struct ethhdr));

			if (iph->daddr.s6_addr[0] == 0xff) /* iphdr->daddr starts with 0xff */
				return true;
		}
	}

	return false;
}
static void slsi_rx_check_opt_out_packet(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct ethhdr *ehdr = (struct ethhdr *)(skb->data);

	ndev_vif->is_opt_out_packet = false;

	if (is_broadcast_ether_addr(ehdr->h_dest)) {
		ndev_vif->is_opt_out_packet = true;
		SLSI_NET_ERR(dev, "Wakeup by Broadcast packet\n");
	} else if (is_multicast_ether_addr(ehdr->h_dest)) {
		ndev_vif->is_opt_out_packet = !slsi_rx_check_mc_addr_regd(sdev, dev, ehdr);
	} else if (be16_to_cpu(ehdr->h_proto) == ETH_P_ARP) {
		u8 *frame;
		u16 arp_opcode;

		frame = skb->data + sizeof(struct ethhdr);
		arp_opcode = frame[SLSI_ARP_OPCODE_OFFSET] << 8 | frame[SLSI_ARP_OPCODE_OFFSET + 1];

		/* not enhanced : only request opcode opt out
		 * enhanced : if (enhanced_pkt_filter_enabled) - ALL pkt opt out
		 *                else - only request opcode opt out
		 */
		if (arp_opcode == SLSI_ARP_REQUEST_OPCODE)
			ndev_vif->is_opt_out_packet = true;
#ifdef CONFIG_SCSC_WLAN_ENHANCED_PKT_FILTER
		else
			ndev_vif->is_opt_out_packet = sdev->enhanced_pkt_filter_enabled;
#endif
		SLSI_NET_ERR(dev, "Wakeup by ARP packet, arp_opcode : %d\n", arp_opcode);
#ifdef CONFIG_SCSC_WLAN_ENHANCED_PKT_FILTER
	} else if (sdev->enhanced_pkt_filter_enabled) {
		if (is_unicast_ether_addr(ehdr->h_dest)) {
			/* IPv4 & TCP / IPv6 & TCP : opt in */
			if (be16_to_cpu(ehdr->h_proto) == ETH_P_IP) {
				struct iphdr *ip = (struct iphdr *)(skb->data + sizeof(struct ethhdr));

				ndev_vif->is_opt_out_packet = !(ip->protocol == IPPROTO_TCP);
				SLSI_NET_INFO(dev, "Wakeup by IPV4 , ip->protocol = %d\n", ip->protocol);
			} else if (be16_to_cpu(ehdr->h_proto) == ETH_P_IPV6) {
				struct ipv6hdr *ip = (struct ipv6hdr *)(skb->data + sizeof(struct ethhdr));

				ndev_vif->is_opt_out_packet = !(ip->nexthdr == IPPROTO_TCP);
				SLSI_NET_INFO(dev, "Wakeup by IPV6 , ip->nexthdr = %d\n", ip->nexthdr);
			} else {
				ndev_vif->is_opt_out_packet = true;
				SLSI_NET_ERR(dev, "Wakeup by h_proto : 0x%.4X\n", be16_to_cpu(ehdr->h_proto));
			}
		} else {
			SLSI_NET_ERR(dev, "Wakeup by Not Unicast - the packet must be checking in Multicast condition\n");
		}
#endif
	} else if (is_unicast_mac_with_multicst_ip(skb)) {
		ndev_vif->is_opt_out_packet = true;
		SLSI_NET_ERR(dev, "Wakeup by unicast mac with multicast ip packet\n");
	} else {
		SLSI_NET_INFO(dev, "Wakeup by Opt in packet\n");
	}
}

static struct sk_buff *slsi_parse_and_pull_ma_unitdata_ind_signal(struct slsi_dev *sdev, struct sk_buff *skb)
{
	struct slsi_skb_cb *skb_cb;
	bool is_ciphered, is_wakeup;
	u8 discard;
#ifdef CONFIG_SCSC_WLAN_LPC
	u32 lpc_tag;

	lpc_tag = fapi_get_u16(skb, u.ma_unitdata_ind.reserved1);
#endif

	/* 0.save info in fapi signal and remove fapi signal
	 * KeyRSC	B3	RX: is The received frame ciphered? The first 48bit contains the KeyRSC
	 * Discard	B8-15	RX only: When set to 1, which a-msdu subframe to host shall discared. (B8 first subframe B15 8th subframe)
	 */
	is_ciphered = (fapi_get_u16(skb, u.ma_unitdata_ind.configuration_option) & FAPI_OPTION_KEYRSC) >> 3;
	discard = (fapi_get_u16(skb, u.ma_unitdata_ind.configuration_option) & FAPI_OPTION_DISCARD) >> 8;
	is_wakeup = ((struct slsi_skb_cb *)skb->cb)->wakeup;
#ifdef CONFIG_SCSC_SMAPPER
	/* Get if the skb is in the SMAPPER entry */
	if (slsi_skb_cb_get(skb)->smapper_linked) {
		skb = slsi_hip_get_skb_from_smapper(sdev, skb);
		if (!skb) {
			SLSI_WARN(sdev, "SKB from SMAPPER is NULL\n");
			return NULL;
		}
	} else {
		skb_pull(skb, fapi_get_siglen(skb));
	}
#else
	skb_pull(skb, fapi_get_siglen(skb));
#endif
	skb_cb = (struct slsi_skb_cb *)skb->cb;
	/* skb now holds the original 802.11 frame (FAPI signal stripped). It stays
	 * in 802.11 form through BA reordering and is converted to ethernet only at
	 * the single delivery point (slsi_rx_data_deliver_skb).
	 */
	skb_cb->is_80211_frame = true;
	skb_cb->is_ciphered = is_ciphered;
	skb_cb->discard = discard;
	skb_cb->wakeup = is_wakeup;

#ifdef CONFIG_SCSC_WLAN_LPC
	if (lpc_tag && slsi_lpc_is_lpc_enabled() && !slsi_lpc_send_ampdu_rx_later(lpc_tag, skb))
		slsi_lpc_add_packet_data(lpc_tag, skb, SLSI_LPC_DATA_TYPE_RX);
#endif
	return skb;
}

#ifdef CONFIG_SCSC_WLAN_WONDERTAP
static struct sk_buff *slsi_rx_wondertap_linearize_skb(struct sk_buff *skb)
{
	struct sk_buff *new_skb;
	struct sk_buff *frag_skb;
	unsigned int total_len;

	if (!skb_has_frag_list(skb))
		return skb;

	/* Calculate total length across all fragments */
	total_len = skb->len;

	/* Allocate a new linear skb large enough for all data */
	new_skb = alloc_skb(total_len + WONDER_DUMMY_FCS_LEN, GFP_ATOMIC);
	if (!new_skb) {
		kfree_skb(skb);
		return NULL;
	}

	/* Copy head skb data */
	if (skb->len)
		fapi_append_data(new_skb, skb->data, skb_headlen(skb));

	/* Walk the frag_list and copy each fragment's data */
	skb_walk_frags(skb, frag_skb) {
		if (frag_skb->len)
			fapi_append_data(new_skb, frag_skb->data, frag_skb->len);
	}

	memcpy(new_skb->cb, skb->cb, sizeof(struct slsi_skb_cb));
	consume_skb(skb);
	return new_skb;
}

static struct sk_buff *slsi_rx_wondertap_append_fcs(struct sk_buff *skb)
{
	/* TEST PATCH: append 4-byte dummy FCS (0x00000000) as required by Wonder driver */
	if (skb_tailroom(skb) < WONDER_DUMMY_FCS_LEN) {
		struct sk_buff *skb2 = skb_copy_expand(skb, skb_headroom(skb), WONDER_DUMMY_FCS_LEN, GFP_ATOMIC);

		if (!skb2) {
			SLSI_ERR_NODEV("wondertap: failed to expand tailroom for FCS\n");
			kfree_skb(skb);
			return NULL;
		}
		consume_skb(skb);
		skb = skb2;
	}
	memset(skb_put(skb, WONDER_DUMMY_FCS_LEN), 0, WONDER_DUMMY_FCS_LEN);
	return skb;
}

/* Append the dummy FCS to a frame that is kept in its native frag_list form
 * (wondertap_amsdu_linearize=0). The FCS must land at the very end of the frame,
 * i.e. on the last fragment of the frag_list, not on the head skb (which would
 * put it in the middle of the payload). If skb has no frag_list this is just the
 * plain head-skb append.
 */
static struct sk_buff *slsi_rx_wondertap_append_fcs_fraglist(struct sk_buff *skb)
{
	struct sk_buff *tail;
	struct sk_buff *prev = NULL;
	struct sk_buff *f;

	if (!skb_has_frag_list(skb))
		return slsi_rx_wondertap_append_fcs(skb);

	/* Walk to the last fragment, remembering its predecessor so we can
	 * relink the chain if the tail has to be reallocated below.
	 */
	for (f = skb_shinfo(skb)->frag_list; f->next; f = f->next)
		prev = f;
	tail = f;

	/* The tail must be writable and have room for the FCS. A shared (cloned)
	 * fragment or one without tailroom is replaced by a private, expanded
	 * copy that is spliced back into the frag_list in place of the original.
	 */
	if (skb_cloned(tail) || skb_tailroom(tail) < WONDER_DUMMY_FCS_LEN) {
		struct sk_buff *tail2 = skb_copy_expand(tail, skb_headroom(tail),
							WONDER_DUMMY_FCS_LEN, GFP_ATOMIC);

		if (!tail2) {
			SLSI_ERR_NODEV("wondertap: failed to expand tail frag for FCS\n");
			kfree_skb(skb);
			return NULL;
		}
		tail2->next = NULL;
		if (prev)
			prev->next = tail2;
		else
			skb_shinfo(skb)->frag_list = tail2;
		kfree_skb(tail);
		tail = tail2;
	}

	memset(skb_put(tail, WONDER_DUMMY_FCS_LEN), 0, WONDER_DUMMY_FCS_LEN);
	/* skb_put() only grew the tail fragment; propagate the added bytes to
	 * the parent's aggregate length accounting.
	 */
	skb->len += WONDER_DUMMY_FCS_LEN;
	skb->data_len += WONDER_DUMMY_FCS_LEN;
	return skb;
}
#endif

/* Resolve the destination address for an 802.11 frame. For EHT/MLD, when the
 * frame targets one of our link addresses, translate it to our MLD (netdev)
 * address. Depends on netdev_vif, hence kept here rather than in utils_80211.h.
 */
static const u8 *slsi_80211_resolve_da(struct net_device *dev, struct ieee80211_hdr *hdr)
{
#ifdef CONFIG_SCSC_WLAN_EHT
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	int link_id;

	for (link_id = 0; link_id < MAX_NUM_MLD_LINKS; link_id++) {
		if (!(ndev_vif->sta.valid_links & BIT(link_id)))
			continue;

		if (SLSI_ETHER_EQUAL(ieee80211_get_DA(hdr), ndev_vif->sta.links[link_id].addr))
			break;
	}
	return (link_id >= MAX_NUM_MLD_LINKS) ? ieee80211_get_DA(hdr) : dev->dev_addr;
#else
	return ieee80211_get_DA(hdr);
#endif
}

/* Convert the original 802.11 data frame held by skb into an 802.3/ethernet
 * frame in place. The caller must read any 802.11 fields it needs (tid, A-MSDU,
 * sn, ...) via the utils_80211.h helpers before calling this, as the 802.11
 * header is gone afterwards.
 */
int slsi_get_msdu_frame(struct net_device *dev, struct sk_buff *skb)
{
#ifdef CONFIG_SCSC_WLAN_EHT
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	int link_id;
#endif
	struct slsi_skb_cb   *skb_cb = (struct slsi_skb_cb *)skb->cb;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	int offset;
	const u8 *da;

	if (!ieee80211_is_data(hdr->frame_control)) {
		SLSI_NET_DBG3(dev, SLSI_RX, "Ignore this packet. It is not data\n");
		return -EINVAL;
	}

	/* Offset to the 802.11 payload, including the KeyRSC for ciphered frames.
	 * The PN inside that KeyRSC was already consumed from the 802.11 frame by
	 * the BA/replay path before conversion.
	 */
	offset = slsi_80211_payload_offset(hdr->frame_control, skb_cb->is_ciphered);

	if (slsi_80211_is_amsdu(hdr)) {
		/* A-MSDU payload: just pull the 802.11 header */
		skb_pull(skb, offset);
		SLSI_NET_DBG4(dev, SLSI_RX, "Packet is A-MSDU offset:%d\n", offset);
	} else {
		/* MSDU: remove llc and add eth_hdr */
		SLSI_ETHER_COPY(&skb->data[offset], ieee80211_get_SA(hdr));
		da = slsi_80211_resolve_da(dev, hdr);
		/**
		 * skb->data[offset - ETH_ALEN] and ieee80211_get_DA(hdr)
		 * can have memory overlapping which leads to data corruption.
		 * Especially when addr3 is used for DA.
		 * We use memmove which considers memory overlapping case.
		 * https://man7.org/linux/man-pages/man3/memmove.3.html
		 */
		memmove(&skb->data[offset - ETH_ALEN], da, ETH_ALEN);
		skb_pull(skb, offset - ETH_ALEN);
		SLSI_NET_DBG4(dev, SLSI_RX, "Packet is MSDU offset:%d\n", offset);
	}
	skb_cb->is_80211_frame = false;
	return 0;
}

#define SLSI_AP_GUEST_VLAN 2

/* Build the ethernet-format representation of an 802.11 data skb into dst
 * without mutating skb. Returns the number of bytes written = min(eth_len,
 * dst_cap). The eth frame is [DA(6)][SA(6)][SNAP ethertype + payload], matching
 * the byte layout produced by slsi_get_msdu_frame().
 */
static size_t slsi_80211_to_eth_buf(struct net_device *dev, struct sk_buff *skb,
				    u8 *dst, size_t dst_cap)
{
	struct slsi_skb_cb   *skb_cb = (struct slsi_skb_cb *)skb->cb;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	int offset = slsi_80211_payload_offset(hdr->frame_control, skb_cb->is_ciphered);
	/* DA+SA overwrite the 802.11 header, KeyRSC and the first 6 LLC/SNAP bytes;
	 * the SNAP ethertype and data follow.
	 */
	size_t llc_skip = (size_t)offset + ETH_ALEN;
	u8 eth_hdr_buf[ETH_ALEN * 2];
	size_t eth_len, n, hdr_copy;

	if (skb->len <= llc_skip)
		return 0;

	SLSI_ETHER_COPY(eth_hdr_buf, slsi_80211_resolve_da(dev, hdr));
	SLSI_ETHER_COPY(eth_hdr_buf + ETH_ALEN, ieee80211_get_SA(hdr));

	eth_len = (ETH_ALEN * 2) + (skb->len - llc_skip);
	n = min(eth_len, dst_cap);

	hdr_copy = min(n, sizeof(eth_hdr_buf));
	memcpy(dst, eth_hdr_buf, hdr_copy);
	if (n > sizeof(eth_hdr_buf))
		memcpy(dst + sizeof(eth_hdr_buf), skb->data + llc_skip, n - sizeof(eth_hdr_buf));

	return n;
}

/* Capture (a debug copy of) the last delayed RX packet in ethernet format.
 * is_80211_frame selects whether skb still holds the original 802.11 frame
 * (RX data path) or an already-converted ethernet frame with the FAPI signal
 * still attached (mlme received-frame path).
 */
void slsi_capture_last_delayd_pkt(struct slsi_dev *sdev, struct net_device *dev,
				  struct sk_buff *skb, bool is_80211_frame)
{
	size_t cap = min_t(size_t, READ_ONCE(dbg_last_delayd_pkt_capture_len),
			   (size_t)MAX_LAST_DELAYD_PKT_SIZE);
	size_t n;

	if (is_80211_frame) {
		n = slsi_80211_to_eth_buf(dev, skb, sdev->last_delayd_pkt.pkt, cap);
	} else {
		size_t len = skb->len - fapi_get_siglen(skb);

		n = min(len, cap);
		if (n)
			memcpy(sdev->last_delayd_pkt.pkt, fapi_get_data(skb), n);
	}
	sdev->last_delayd_pkt.pkt_size = n;
}

static inline bool slsi_is_guest_network(struct slsi_dev *sdev, struct net_device *dev,
					 u8 *src_mac, struct net_device **vlan_dev)
{
	struct slsi_peer *peer = NULL;
	int vlan_id;

	peer = slsi_get_peer_from_mac(sdev, dev, src_mac);
	if (peer && peer->group_key_index) {
		vlan_id = SLSLI_VLAN_GRP_KEY_2_VLANID(peer->group_key_index);
		if (vlan_id < 0)
			return false;
		*vlan_dev = rcu_dereference(peer->netdev_vlan);
		if (vlan_id == SLSI_AP_GUEST_VLAN)
			return true;
	}
	return false;
}

static inline bool slsi_is_isolate_ap_or_guest_vlan(struct slsi_dev *sdev, struct net_device *dev,
						    u8 *src_mac, struct net_device **vlan_dev)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	if (ndev_vif->ap.ap_isolate)
		return true;

	if (sdev->num_ap_vlan)
		return slsi_is_guest_network(sdev, dev, src_mac, vlan_dev);

	return false;
}

void slsi_rx_data_deliver_skb(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb, bool ctx_napi)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff_head msdu_list;
	struct slsi_skb_cb *skb_cb = (struct slsi_skb_cb *)skb->cb;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	struct slsi_peer *peer = NULL;
	struct ethhdr *eth_hdr;
	bool is_amsdu;
	u16 tid;
	bool amsdu_packet_drop = false;
	u8 trafic_q;
	struct netdev_vif *to_ndev_vif;
	struct net_device *to_dev;

	/* Single conversion point: the RX path delivers the original 802.11 frame
	 * here (directly, or out of the BA reorder queue). Read the 802.11 fields
	 * we need, then convert it to ethernet exactly once.
	 */
	if (WLBT_WARN_ON(!skb_cb->is_80211_frame)) {
		kfree_skb(skb);
		return;
	}

	is_amsdu = slsi_80211_is_amsdu(hdr);
	tid = slsi_80211_get_tid(hdr);
	trafic_q = slsi_frame_priority_to_ac_queue(tid);

#ifdef CONFIG_SCSC_WLAN_WONDERTAP
	/* Wondertap RELAY A-MSDU: deaggregate with radiotap+802.11 prefix per
	 * subframe. Read rssi/timestamp from the FAPI signal still in headroom.
	 */
	if (ndev_vif->ifnum == SLSI_NET_INDEX_RELAY) {
		skb_cb = (struct slsi_skb_cb *)skb->cb;
		skb_cb->is_80211_frame = false;

		if (!slsi_80211_is_amsdu(hdr) || !wondertap_amsdu_deaggregate) {
			/* Pass the whole frame up (a non-A-MSDU, or an A-MSDU when
			 * deaggregation is disabled). radiotap is prepended, then either
			 * the frag_list is kept intact (default) or the frame is copied
			 * into one linear jumbo skb (wondertap_amsdu_linearize).
			 */
			struct fapi_signal *fapi_hdr = (struct fapi_signal *)(skb->data - skb_cb->sig_length);
			s16 w_rssi = (s16)le16_to_cpu(fapi_hdr->u.ma_unitdata_ind.rssi);
			u32 w_timestamp = le32_to_cpu(fapi_hdr->u.ma_unitdata_ind.timestamp);

			if (w_rssi > 0)
				w_rssi -= 1;
			else if (w_rssi < 0)
				w_rssi += 1;

			slsi_rx_wondertap_add_radiotap_header(dev, &skb, w_rssi, w_timestamp, NULL);

			if (wondertap_amsdu_linearize) {
				skb = slsi_rx_wondertap_linearize_skb(skb);
				if (!skb) {
					SLSI_NET_WARN(dev, "Packet dropped (A-MSDU linearize failed)\n");
					dev->stats.rx_dropped++;
					return;
				}
				skb = slsi_rx_wondertap_append_fcs(skb);
			} else {
				/* Keep the RX frag_list intact: radiotap sits on the head
				 * skb (skb_push preserves frag_list) and the FCS goes on the
				 * last fragment so it ends up at the frame tail.
				 */
				skb = slsi_rx_wondertap_append_fcs_fraglist(skb);
			}
			if (!skb) {
				SLSI_NET_WARN(dev, "Packet dropped (Appending FCS failed)\n");
				dev->stats.rx_dropped++;
				return;
			}

			skb->dev = dev;
			skb->ip_summed = CHECKSUM_NONE;
			skb->protocol = htons(ETH_P_802_2);
			skb->pkt_type = PACKET_OTHERHOST;
			skb_reset_mac_header(skb);

			dev->stats.rx_packets++;
			dev->stats.rx_bytes += skb->len;
			ndev_vif->rx_packets[trafic_q]++;
			slsi_traffic_mon_event_rx(sdev, dev, skb);
			netif_receive_skb(skb);
		} else {
			__skb_queue_head_init(&msdu_list);
			if (slsi_rx_amsdu_deaggregate(dev, skb, &msdu_list)) {
				SLSI_NET_WARN(dev, "Packet dropped (A-MSDU wondertap deaggregation error)\n");
				dev->stats.rx_dropped++;
				return;
			}
			/* Deliver subframes to wondertap netdev */
			while (!skb_queue_empty(&msdu_list)) {
				struct sk_buff *rx_skb = __skb_dequeue(&msdu_list);

				rx_skb->dev = dev;
				rx_skb->ip_summed = CHECKSUM_NONE;
				rx_skb->protocol = htons(ETH_P_802_2);
				rx_skb->pkt_type = PACKET_OTHERHOST;
				skb_reset_mac_header(rx_skb);

				dev->stats.rx_packets++;
				dev->stats.rx_bytes += rx_skb->len;
				ndev_vif->rx_packets[trafic_q]++;
				slsi_traffic_mon_event_rx(sdev, dev, rx_skb);
				netif_receive_skb(rx_skb);
			}
		}
		slsi_wake_lock_timeout(&sdev->wlan_wl_ma, msecs_to_jiffies(SLSI_RX_WAKELOCK_TIME));
		return;
	}
#endif

	if (slsi_get_msdu_frame(dev, skb)) {
		SLSI_NET_WARN(dev, "Packet dropped (invalid 802.11 data frame)\n");
		kfree_skb(skb);
		return;
	}
	__skb_queue_head_init(&msdu_list);

	eth_hdr = (struct ethhdr *)skb->data;
	peer = slsi_get_peer_from_mac(sdev, dev, eth_hdr->h_source);
	if (!peer) {
		SLSI_NET_WARN(dev, "Packet dropped (no peer records)\n");
		kfree_skb(skb);
		return;
	}

	/* A-MSDU deaggregation */
	if (is_amsdu) {
#ifdef CONFIG_SCSC_WLAN_RX_NAPI_GRO
		read_lock_bh(&ndev_vif->gro_lock);
		if (ndev_vif->gro_enabled == SLSI_GRO_ENABLED)
			amsdu_packet_drop = slsi_rx_amsdu_deaggregate_gro(dev, skb, &msdu_list);
		else
			amsdu_packet_drop = slsi_rx_amsdu_deaggregate(dev, skb, &msdu_list);
#else
		amsdu_packet_drop = slsi_rx_amsdu_deaggregate(dev, skb, &msdu_list);
#endif
		if (amsdu_packet_drop) {
			SLSI_NET_WARN(dev, "Packet dropped (A-MSDU error)\n");
			ndev_vif->stats.rx_dropped++;
			if (peer)
				peer->sinfo.rx_dropped_misc++;
#ifdef CONFIG_SCSC_WLAN_RX_NAPI_GRO
			read_unlock_bh(&ndev_vif->gro_lock);
			return;
		}
		read_unlock_bh(&ndev_vif->gro_lock);
#else
			return;
		}
#endif
	} else {
		if (skb_cb->discard) {
			SLSI_NET_WARN(dev, "Packet dropped (discard flag set)\n");
			/* is a duplicate frame from control plane, that should be
			 * discarded.
			 * not needed to update drop stats here
			 */
			consume_skb(skb);
			return;
		}
		__skb_queue_tail(&msdu_list, skb);
	}
	/* WARNING: skb may be NULL here and should not be used after this */
	while (!skb_queue_empty(&msdu_list)) {
		struct sk_buff *rx_skb;
		struct net_device *vlan_dev = NULL;
		struct netdev_vif *vlan_ndev_vif = NULL;
#if defined(CONFIG_SCSC_WLAN_RX_NAPI) && defined(CONFIG_SCSC_WLAN_RX_NAPI_GRO)
		u16 drop_proto;
#endif
		rx_skb = __skb_dequeue(&msdu_list);

		/* In STA mode, if Wakeup by Opt out packet, it looks it was
		 * NOT install packet filter when enter Suspend mode. So it
		 * should be install Packet filter and LCD state again.
		 */
		if (ndev_vif->vif_type == FAPI_VIFTYPE_STATION) {
			struct ethhdr *ehdr = (struct ethhdr *)(rx_skb->data);

			if (unlikely(slsi_skb_cb_get(rx_skb)->wakeup)) {
				slsi_rx_check_opt_out_packet(sdev, dev, rx_skb);
				slsi_wake_lock(&sdev->wlan_wl);
				schedule_work(&ndev_vif->update_pkt_filter_work);
			}

			/* In STA mode, the AP relays back our multicast
			 * traffic. Receiving these frames and passing it up
			 * confuses some protocols and applications, notably
			 * IPv6 Duplicate Address Detection.
			 *
			 * So these frames are dropped instead of passing it
			 * further. No need to update the drop statistics as
			 * these frames are locally generated and should not be
			 * accounted in reception.
			 */
			if (is_multicast_ether_addr(ehdr->h_dest) && !compare_ether_addr(ehdr->h_source, dev->dev_addr)) {
				SLSI_NET_DBG2(dev, SLSI_RX, "drop locally generated multicast frame relayed back by AP\n");
				consume_skb(rx_skb);
				continue;
			}
		}

		/* Intra BSS */
		if (ndev_vif->vif_type == FAPI_VIFTYPE_AP && ndev_vif->peer_sta_records) {
			struct slsi_peer *peer = NULL;
			struct ethhdr *ehdr = (struct ethhdr *)(rx_skb->data);
			struct net_device *vlan_dev = NULL;

			if (is_multicast_ether_addr(ehdr->h_dest)) {
				/* When ap_isolate is enabled, no need to forward multicast packet to peers in BSS.
				 * But, it should be passed up to the kernel.
				 */
				if (!slsi_is_isolate_ap_or_guest_vlan(sdev, dev, ehdr->h_source, &vlan_dev)) {
					/* For the case of uing NAPI, we need to use GFP_ATOMIC */
					struct sk_buff *rebroadcast_skb = skb_copy(rx_skb, GFP_ATOMIC);

					if (!rebroadcast_skb) {
						SLSI_WARN(sdev, "Intra BSS: failed to alloc new SKB for broadcast\n");
					} else {
						SLSI_DBG3(sdev, SLSI_RX, "Intra BSS: multicast %pM, proto=0x%.4X headroom:%d\n",
							  ehdr->h_dest, be16_to_cpu(ehdr->h_proto), skb_headroom(rebroadcast_skb));
						rebroadcast_skb->dev = vlan_dev ? vlan_dev : dev;
						rebroadcast_skb->protocol = cpu_to_be16(ETH_P_802_3);
						skb_reset_network_header(rebroadcast_skb);
						skb_reset_mac_header(rebroadcast_skb);
						dev_queue_xmit(rebroadcast_skb);
					}
				} else {
					SLSI_DBG3(sdev, SLSI_RX, "Intra BSS: Don't forward mcast. %pM, proto=0x%.4X\n",
						  ehdr->h_dest, be16_to_cpu(ehdr->h_proto));
				}
			} else {
				peer = slsi_get_peer_from_mac(sdev, dev, ehdr->h_dest);
				if (peer && peer->authorized) {
					if (slsi_is_isolate_ap_or_guest_vlan(sdev, dev, ehdr->h_source, &vlan_dev)) {
						/* When ap_isolate is enabled, no need to forward unicast to peer.
						 * It just need to be dropped. No need to pass up to kernel.
						 */
						SLSI_DBG3(sdev, SLSI_RX, "Intra BSS: Drop unicast. %pM, proto=0x%.4X\n",
							  ehdr->h_dest, be16_to_cpu(ehdr->h_proto));
						consume_skb(rx_skb);
						continue;
					}
					SLSI_DBG3(sdev, SLSI_RX, "Intra BSS: unicast %pM, proto=0x%.4X, headroom:%d\n",
						  ehdr->h_dest, be16_to_cpu(ehdr->h_proto), skb_headroom(rx_skb));

					rx_skb->dev = vlan_dev ? vlan_dev : dev;
					rx_skb->protocol = cpu_to_be16(ETH_P_802_3);
					skb_reset_network_header(rx_skb);
					skb_reset_mac_header(rx_skb);
					dev_queue_xmit(rx_skb);
					continue;
				}
			}
		}

#ifdef CONFIG_SCSC_WIFI_NAN_ENABLE
		/* NAN: multicast receive */
		if (ndev_vif->ifnum >= SLSI_NAN_DATA_IFINDEX_START) {
			struct net_device *other_dev;
			struct netdev_vif *other_ndev_vif;
			struct ethhdr *ehdr = (struct ethhdr *)(rx_skb->data);
			u8 i, j;

			if (is_multicast_ether_addr(ehdr->h_dest)) {
				/* In case of NAN, the multicast packet is received on a NDL VIF.
				 * NDL VIF is between two peers.
				 * But two peers can have multiple NDP connections.
				 * Multiple NDP connections with same peer needs multiple netdevices.
				 *
				 * So, loop through each net device and find if there is a match in
				 * source address, and if so, duplicate the multicast packet, and send
				 * them up on each netdev that matches.
				 */
				SLSI_NET_DBG2(dev, SLSI_RX, "NAN: multicast (source address: %pM)\n", ehdr->h_source);
				for ((i = ndev_vif->ifnum + 1); i <= CONFIG_SCSC_WLAN_MAX_INTERFACES; i++) {
					other_dev = (struct net_device *) sdev->netdev[i];
					if (!other_dev)
						continue;
					other_ndev_vif = (struct netdev_vif *)netdev_priv(other_dev);
					if (!other_ndev_vif)
						continue;
					for (j = 0; j < SLSI_PEER_INDEX_MAX; j++) {
						if (other_ndev_vif->peer_sta_record[j] &&
							other_ndev_vif->peer_sta_record[j]->valid &&
						    ether_addr_equal(other_ndev_vif->peer_sta_record[j]->address, ehdr->h_source)) {
							/* For the case of uing NAPI, we need to use GFP_ATOMIC */
							struct sk_buff *duplicate_skb = skb_copy(rx_skb, GFP_ATOMIC);
							SLSI_NET_DBG2(other_dev, SLSI_RX, "NAN: source address match %pM\n", other_ndev_vif->peer_sta_record[j]->address);
							if (!duplicate_skb) {
								SLSI_NET_WARN(other_dev, "NAN: multicast: failed to alloc new SKB\n");
								continue;
							}

							other_ndev_vif->peer_sta_record[j]->sinfo.rx_bytes += duplicate_skb->len;
							other_ndev_vif->stats.rx_packets++;
							other_ndev_vif->stats.rx_bytes += duplicate_skb->len;
							other_ndev_vif->rx_packets[trafic_q]++;

							duplicate_skb->dev = other_dev;
							duplicate_skb->ip_summed = CHECKSUM_NONE;
							duplicate_skb->protocol = eth_type_trans(duplicate_skb, other_dev);

							SLSI_NET_DBG4(other_dev, SLSI_RX, "pass %u bytes to local stack\n", duplicate_skb->len);
#ifdef CONFIG_SCSC_WLAN_RX_NAPI
							netif_receive_skb(duplicate_skb);
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
							netif_rx(duplicate_skb);
#else
							netif_rx_ni(duplicate_skb);
#endif
#endif
						}
					}
				}
			}
		}
#endif
		/* SoftAP VLAN traffic */
		if (ndev_vif->vif_type == FAPI_VIFTYPE_AP && sdev->num_ap_vlan) {
			struct ethhdr *ehdr = (struct ethhdr *)(rx_skb->data);

			/* When VLAN is enabled, send packet to corresponding VLAN interface */
			slsi_is_guest_network(sdev, dev, ehdr->h_source, &vlan_dev);
			if (!vlan_dev) {
				SLSI_DBG3(sdev, SLSI_RX,
					  "No VLAN devices forund. Drop unicast. %pM, proto=0x%.4X\n",
					  ehdr->h_dest, be16_to_cpu(ehdr->h_proto));
				consume_skb(rx_skb);
				continue;
			}
			vlan_ndev_vif = (struct netdev_vif *)netdev_priv(vlan_dev);
			if (!vlan_ndev_vif) {
				vlan_dev = NULL;
				consume_skb(rx_skb);
				continue;
			}
		}

		if (peer) {
			peer->sinfo.rx_bytes += rx_skb->len;
		}

		if (vlan_dev && vlan_ndev_vif) {
			to_dev = vlan_dev;
			to_ndev_vif = vlan_ndev_vif;
		} else {
			to_dev = dev;
			to_ndev_vif = ndev_vif;
		}

		to_ndev_vif->stats.rx_packets++;
		to_ndev_vif->stats.rx_bytes += rx_skb->len;
		to_ndev_vif->rx_packets[trafic_q]++;

#ifdef CONFIG_SCSC_WLAN_STA_ENHANCED_ARP_DETECT
	if (!to_ndev_vif->enhanced_arp_stats.is_duplicate_addr_detected) {
		u8 *frame = rx_skb->data + 12; /* frame points to packet type */
		u16 packet_type = frame[0] << 8 | frame[1];

		if (packet_type == ETH_P_ARP) {
			frame = frame + 2; /* ARP packet */
			/*match source IP address in ARP with the DUT Ip address*/
			if ((frame[SLSI_ARP_SRC_IP_ADDR_OFFSET] == (to_ndev_vif->ipaddress & 255)) &&
			    (frame[SLSI_ARP_SRC_IP_ADDR_OFFSET + 1] == ((to_ndev_vif->ipaddress >>  8U) & 255)) &&
			    (frame[SLSI_ARP_SRC_IP_ADDR_OFFSET + 2] == ((to_ndev_vif->ipaddress >> 16U) & 255)) &&
			    (frame[SLSI_ARP_SRC_IP_ADDR_OFFSET + 3] == ((to_ndev_vif->ipaddress >> 24U) & 255)) &&
			    !SLSI_IS_GRATUITOUS_ARP(frame) &&
			    !SLSI_ETHER_EQUAL(to_dev->dev_addr, frame + 8)) /*if src MAC = DUT MAC */
				to_ndev_vif->enhanced_arp_stats.is_duplicate_addr_detected = 1;
		}
	}

	if (to_ndev_vif->enhanced_arp_detect_enabled && (to_ndev_vif->vif_type == FAPI_VIFTYPE_STATION)) {
		u8 *frame = rx_skb->data + 12; /* frame points to packet type */
		u16 packet_type = frame[0] << 8 | frame[1];
		u16 arp_opcode;

		if (packet_type == ETH_P_ARP) {
			frame = frame + 2; /* ARP packet */
			arp_opcode = frame[SLSI_ARP_OPCODE_OFFSET] << 8 | frame[SLSI_ARP_OPCODE_OFFSET + 1];
			/* check if sender ip = gateway ip and it is an ARP response */
			if ((arp_opcode == SLSI_ARP_REPLY_OPCODE) &&
			    !SLSI_IS_GRATUITOUS_ARP(frame) &&
			    !memcmp(&frame[SLSI_ARP_SRC_IP_ADDR_OFFSET], &to_ndev_vif->target_ip_addr, 4)) {
				to_ndev_vif->enhanced_arp_stats.arp_rsp_count_to_netdev++;
				to_ndev_vif->enhanced_arp_stats.arp_rsp_rx_count_by_upper_mac++;
			}
		}
	}
#endif
		slsi_traffic_mon_event_rx(sdev, to_dev, rx_skb);

		SLSI_NET_DBG4(to_dev, SLSI_RX, "pass %u bytes to local stack\n", rx_skb->len);
		rx_skb->dev = to_dev;
		rx_skb->ip_summed = CHECKSUM_NONE;
		rx_skb->protocol = eth_type_trans(rx_skb, to_dev);
		SCSC_HIP4_SAMPLER_TCP_DECODE(sdev, to_dev, rx_skb->data, true);
		slsi_skb_cb_init(rx_skb);
#ifdef CONFIG_SCSC_WLAN_RX_NAPI
#ifdef CONFIG_SCSC_WLAN_RX_NAPI_GRO
		read_lock_bh(&to_ndev_vif->gro_lock);
		if (to_ndev_vif->gro_enabled == SLSI_GRO_ENABLED) {
			drop_proto = rx_skb->protocol;
			if (ctx_napi) {
#ifndef CONFIG_SCSC_WLAN_LOAD_BALANCE_MANAGER
				napi_gro_receive(&sdev->hip.hip_priv->napi, rx_skb);
#else
				struct napi_struct *napi;
#ifdef CONFIG_SCSC_WLAN_NAPI_PER_NETDEV
				napi = &ndev_vif->rx_data_napi;
#else
				int cpu = smp_processor_id();

				napi = &sdev->hip.hip_priv->bh_dat->bh_priv.napi.cpu_info[cpu].napi_instance;
#endif
				napi_gro_receive(napi, rx_skb);
#endif
			} else {
				if (netif_receive_skb(rx_skb) == NET_RX_DROP)
					SLSI_NET_WARN(to_dev, "Packet is dropped. Protocol=%hx\n", ntohs(drop_proto));
			}
		} else {
			netif_receive_skb(rx_skb);
		}
		read_unlock_bh(&to_ndev_vif->gro_lock);
#else /* #ifdef CONFIG_SCSC_WLAN_RX_NAPI_GRO */
		netif_receive_skb(rx_skb);
#endif
#else /* #ifdef CONFIG_SCSC_WLAN_RX_NAPI */
#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
		netif_rx(rx_skb);
#else
		netif_rx_ni(rx_skb);
#endif
#endif
		slsi_wake_lock_timeout(&sdev->wlan_wl_ma, msecs_to_jiffies(SLSI_RX_WAKELOCK_TIME));
	}
}

#ifdef CONFIG_SCSC_WLAN_WONDERTAP
static int slsi_rx_wondertap_add_radiotap_header(struct net_device *dev, struct sk_buff **skb_pointer, s16 rssi, u32 timestamp, u8 *rtap_buf)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	int rtap_len = sizeof(struct ieee80211_radiotap_header) + RADIOTAP_WONDERTAP_EXTRA_LEN;
	struct ieee80211_radiotap_header *rtap;
	struct sk_buff *skb = *skb_pointer;
	u8 *rtap_start;
	u8 *pos;
	u16 channel_freq = 0;
	u16 channel_flags = 0;

	/* Get channel frequency from VIF */
	if (ndev_vif->chan)
		channel_freq = ndev_vif->chan->center_freq;

	/* Set channel flags: 5GHz OFDM or 0 */
	if (channel_freq > 4000)
		channel_flags = IEEE80211_CHAN_OFDM | IEEE80211_CHAN_5GHZ;

	if (!rtap_buf && skb_headroom(skb) < rtap_len) {
		struct sk_buff *skb2 = NULL;

		skb2 = skb_realloc_headroom(skb, rtap_len);
		if (!skb2)
			return -EINVAL;
		consume_skb(skb);
		skb = skb2;
		*skb_pointer = skb;
	}

	if (!rtap_buf) {
		rtap_start = skb_push(skb, rtap_len);
		memset(rtap_start, 0, rtap_len);
		rtap = (void *)rtap_start;
	} else
		rtap = (void *)rtap_buf;

	rtap->it_version = 0;
	rtap->it_pad = 0;
	rtap->it_len = cpu_to_le16(rtap_len);

	/* Present flags: TSFT, Flags, Channel, Signal, Noise, Antenna */
	rtap->it_present = cpu_to_le32(
		(1 << IEEE80211_RADIOTAP_TSFT) |
		(1 << IEEE80211_RADIOTAP_FLAGS) |
		(1 << IEEE80211_RADIOTAP_CHANNEL) |
		(1 << IEEE80211_RADIOTAP_DBM_ANTSIGNAL) |
		(1 << IEEE80211_RADIOTAP_DBM_ANTNOISE) |
		(1 << IEEE80211_RADIOTAP_ANTENNA)
	);

	pos = (u8 *)rtap + sizeof(struct ieee80211_radiotap_header);

	/* TSFT (8 bytes, naturally 8-byte aligned after 8-byte radiotap header) */
	put_unaligned_le64((u64)timestamp, pos);
	pos += 8;

	/* Flags (1 byte) */
	*pos = 0x10;
	pos += 1;

	/* Padding for 2-byte alignment before Channel */
	pos += 1;

	/* Channel frequency (2 bytes) */
	put_unaligned_le16(channel_freq, pos);
	pos += 2;

	/* Channel flags (2 bytes) */
	put_unaligned_le16(channel_flags, pos);
	pos += 2;

	/* Antenna Signal / DBM_ANTSIGNAL (1 byte) */
	*pos = (s8)rssi;
	pos += 1;

	/* Antenna Noise / DBM_ANTNOISE (1 byte) - set to 0 */
	*pos = 0;
	pos += 1;

	/* Antenna (1 byte) - set to 0 */
	*pos = 0;

	return 0;
}
#endif

#ifdef CONFIG_SCSC_WLAN_NAPI_PER_NETDEV
void slsi_rx_data_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
#else
static void slsi_rx_data_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
#endif
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_skb_cb *skb_cb;
	struct slsi_peer *peer = NULL;
	struct ieee80211_hdr *hdr;
	u16 tid;
	u16 sn;
	bool is_tdls;
	bool delayed = (fapi_get_u16(skb, u.ma_unitdata_ind.configuration_option) & FAPI_OPTION_DELAYED);
	bool last_pkt = (fapi_get_u16(skb, u.ma_unitdata_ind.configuration_option) & FAPI_OPTION_LAST);
#ifdef CONFIG_SCSC_WLAN_WONDERTAP
	s16 rssi = fapi_get_s16(skb, u.ma_unitdata_ind.rssi);
	u32 timestamp = fapi_get_u32(skb, u.ma_unitdata_ind.timestamp);
	int radiotap_headroom = sizeof(struct ieee80211_radiotap_header) + RADIOTAP_WONDERTAP_EXTRA_LEN;
	bool is_wonder_data_frame = false;
#endif

	skb = slsi_parse_and_pull_ma_unitdata_ind_signal(sdev, skb);
	if (!skb)
		return;

	/* pass the data up if the VIF type is Monitor */
	if (ndev_vif->vif_type == FAPI_VIFTYPE_MONITOR) {
		skb_reset_mac_header(skb);
		skb->dev = dev;
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		skb->pkt_type = PACKET_OTHERHOST;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
		netif_rx(skb);
#else
		netif_rx_ni(skb);
#endif
		return;
	}

#ifdef CONFIG_SCSC_WLAN_WONDERTAP
	if (ndev_vif->vif_type == FAPI_VIFTYPE_RELAY) {
		u8 hdr11_buf[sizeof(struct ieee80211_hdr) + IEEE80211_QOS_CTL_LEN];
		struct ieee80211_hdr *hdr11 = (struct ieee80211_hdr *)hdr11_buf;
		size_t hdr11_copy_len = min_t(size_t, skb->len, sizeof(hdr11_buf));
		const u8 *da;

		/* Deep copy the 802.11 header into a local buffer so subsequent skb
		 * realloc paths (skb_copy_expand below, skb_realloc_headroom inside
		 * slsi_rx_wondertap_add_radiotap_header) cannot leave us with a
		 * dangling pointer into the original skb.
		 */
		memcpy(hdr11_buf, skb->data, hdr11_copy_len);
		da = ieee80211_get_DA(hdr11);
		is_wonder_data_frame = ieee80211_is_data(hdr11->frame_control);

		if (!is_multicast_ether_addr(da) && compare_ether_addr(da, dev->dev_addr)) {
			SLSI_NET_DBG4(dev, SLSI_RX, "drop packet which is unicast, but not targetting this device.\n");
			kfree_skb(skb);
			return;
		}

		peer = slsi_get_peer_from_mac(sdev, dev, ieee80211_get_SA(hdr11));

		/* A-MSDU data frames on RELAY: let them fall through to
		 * slsi_rx_data_deliver_skb() which handles deaggregation with
		 * radiotap info read from the FAPI header still in skb headroom.
		 */
		if (is_wonder_data_frame && peer && !is_multicast_ether_addr(da))
			goto wonder_amsdu_via_deliver;

		if (skb_headroom(skb) < radiotap_headroom + sizeof(struct msdu_hdr)) {
			struct sk_buff *skb2 = NULL;
			skb2 = skb_copy_expand(skb, radiotap_headroom + sizeof(struct msdu_hdr), skb_tailroom(skb), GFP_ATOMIC);
			if (!skb2) {
				SLSI_NET_ERR(dev, "skb_copy_expand didn't expand for needed headroom\n");
				dev_kfree_skb(skb);
				return;
			}
			consume_skb(skb);
			skb = skb2;
		}
		if (rssi > 0)
			rssi = rssi - 1;
		else if (rssi < 0)
			rssi = rssi + 1;

		slsi_rx_wondertap_add_radiotap_header(dev, &skb, rssi, timestamp, NULL);

		if (is_wonder_data_frame && wondertap_amsdu_linearize) {
			skb = slsi_rx_wondertap_linearize_skb(skb);
			if (!skb) {
				SLSI_NET_ERR(dev, "wondertap: failed to linearize skb\n");
				return;
			}
			skb = slsi_rx_wondertap_append_fcs(skb);
		} else {
			/* Default for a passed-up data frame: keep the native frag_list
			 * intact and put the FCS on the last fragment (see deliver path).
			 * Non-data frames have no frag_list, so this is a plain append.
			 */
			skb = slsi_rx_wondertap_append_fcs_fraglist(skb);
		}
		if (!skb) {
			SLSI_NET_WARN(dev, "Packet dropped (Appending FCS failed)\n");
			dev->stats.rx_dropped++;
			return;
		}

		slsi_skb_cb_init(skb);
		skb->ip_summed = CHECKSUM_NONE;
		skb->dev = dev;
		skb->protocol = htons(ETH_P_802_2);
		skb->pkt_type = PACKET_OTHERHOST;
		skb_reset_mac_header(skb);

		netif_receive_skb(skb);
		slsi_wake_lock_timeout(&sdev->wlan_wl_ma, msecs_to_jiffies(SLSI_RX_WAKELOCK_TIME));
		return;
	}
wonder_amsdu_via_deliver:
#endif

	/* From here the skb still holds the original 802.11 frame. BA reordering,
	 * replay detection and routing are all performed on this 802.11 frame; the
	 * conversion to ethernet happens once in slsi_rx_data_deliver_skb().
	 */
	hdr = (struct ieee80211_hdr *)skb->data;
	if (!ieee80211_is_data(hdr->frame_control)) {
		WLBT_WARN_ON(1);
		kfree_skb(skb);
		return;
	}
	skb_cb = (struct slsi_skb_cb *)skb->cb;

	tid = slsi_80211_get_tid(hdr);
	sn = slsi_80211_get_sn(hdr);
	is_tdls = slsi_80211_is_tdls(hdr);

	/* Populate wake reason stats here. The stats only need the ethernet/IP
	 * headers, so build a small ethernet-format view without converting skb.
	 */
	if (unlikely(skb_cb->wakeup)) {
		u8 wake_buf[SLSI_WAKE_STATS_HDR_LEN];
		size_t wake_len = slsi_80211_to_eth_buf(dev, skb, wake_buf, sizeof(wake_buf));

		schedule_work(&sdev->wakeup_time_work);
		skb->mark = SLSI_WAKEUP_PKT_MARK;
		slsi_rx_update_wake_stats(sdev, (struct ethhdr *)wake_buf, wake_len, skb);		
	}

	if (delayed && last_pkt) {
		slsi_capture_last_delayd_pkt(sdev, dev, skb, true);
	}

	SLSI_NET_DBG4(dev, SLSI_RX,
		      "ma_unitdata_ind(ifnum:%d, dest:" MACSTR ", src:" MACSTR ", tid:%d, seq:%d, cipher:%d, discard:0x%X, is_tdls:%d delayed:%d last:%d\n",
		      ndev_vif->ifnum,
		      MAC2STR(ieee80211_get_DA(hdr)),
		      MAC2STR(ieee80211_get_SA(hdr)),
		      tid,
		      sn,
		      skb_cb->is_ciphered,
		      skb_cb->discard,
		      is_tdls, delayed, last_pkt);

	peer = slsi_get_peer_from_mac(sdev, dev, ieee80211_get_SA(hdr));
	if (!peer) {
		SLSI_NET_WARN(dev, "Packet dropped (no peer records)\n");
		/* Race in Data plane (Shows up in fw test mode) */
		kfree_skb(skb);
		return;
	}

	/* discard data frames if received before key negotiations are completed */
	if (ndev_vif->vif_type == FAPI_VIFTYPE_AP && peer->connected_state != SLSI_STA_CONN_STATE_CONNECTED) {
		SLSI_NET_WARN(dev, "Packet dropped (peer connection not complete (state:%u))\n", peer->connected_state);
		kfree_skb(skb);
		return;
	}

	/* skip BA reorder if the destination address is Multicast.
	 * Routing decisions use the outer 802.11 addresses. For A-MSDU the inner
	 * subframe addresses may differ, but multicast frames are not BA/A-MSDU
	 * aggregated so this matches the pre-refactor behaviour.
	 */
	if (ndev_vif->vif_type == FAPI_VIFTYPE_STATION && is_multicast_ether_addr(ieee80211_get_DA(hdr))) {
		/* Skip BA reorder and pass the frames Up */
		SLSI_NET_DBG2(dev, SLSI_RX, "Multicast/Broadcast packet received in STA mode(seq: %d) skip BA\n", sn);
		slsi_rx_data_deliver_skb(sdev, dev, skb, false);
		return;
	}

	/* When TDLS connection has just been closed a few last frame may still arrive from the closed connection.
	 * This frames must not be injected in to the block session with the AP as the sequence numbers are different
	 * that will confuse the BA process. Therefore we have to skip BA for those frames.
	 */
	if (ndev_vif->vif_type == FAPI_VIFTYPE_STATION && peer->aid < SLSI_TDLS_PEER_INDEX_MIN && is_tdls) {
		if (printk_ratelimit())
			SLSI_NET_WARN(dev, "Packet received from TDLS but no TDLS exists (seq: 0x%x) Skip BA\n", sn);

		/* Skip BA reorder and pass the frames Up */
		slsi_rx_data_deliver_skb(sdev, dev, skb, false);
		return;
	}

	/* TDLS is enabled for the PEER but still packet is received through the AP. Process this packet with the AP PEER */
	if (ndev_vif->vif_type == FAPI_VIFTYPE_STATION && peer->aid >= SLSI_TDLS_PEER_INDEX_MIN && !is_tdls) {
		SLSI_NET_DBG2(dev, SLSI_TDLS, "Packet received from TDLS peer through the AP(seq: 0x%x)\n", sn);
		peer = slsi_get_peer_from_qs(sdev, dev, SLSI_STA_PEER_QUEUESET);
		if (!peer) {
			SLSI_NET_WARN(dev, "Packet dropped (AP peer not found)\n");
			kfree_skb(skb);
			return;
		}
	}

	/* If frame belongs to a negotiated BA, BA will consume the frame */
	if (slsi_ba_check(peer, tid))
		if (!slsi_ba_process_frame(dev, peer, skb, sn, tid))
			return;

	/* store the SN even when BA agreement is not setup yet */
	slsi_ba_store_sn(dev, peer, tid, sn);

	/* store the PN even when BA agreement is not setup yet */
	{
		u8 pn[SLSI_RX_PN_LEN] = {0};

		if (skb_cb->is_ciphered)
			slsi_80211_get_pn(hdr, pn);
		slsi_ba_replay_store_pn(dev, peer, tid, sn, skb_cb->is_ciphered, pn);
	}

	/* Pass to next receive process */
	slsi_rx_data_deliver_skb(sdev, dev, skb, false);
}

#ifdef CONFIG_SCSC_WLAN_RX_NAPI
static int slsi_rx_napi_process(struct slsi_dev *sdev, struct sk_buff *skb)
{
	struct net_device *dev;
	struct netdev_vif *ndev_vif;
	u16 vif;

	vif = fapi_get_vif(skb);

	rcu_read_lock();
#ifdef CONFIG_SCSC_WIFI_NAN_ENABLE
	if ((vif >= SLSI_NAN_DATA_IFINDEX_START) && fapi_get_sigid(skb) == MA_UNITDATA_IND) {
		dev = slsi_nan_get_netdev_rcu(sdev, skb);
	} else {
		dev = slsi_get_netdev_rcu(sdev, vif);
	}
#else
	dev = slsi_get_netdev_rcu(sdev, vif);
#endif
	if (!dev) {
		SLSI_ERR(sdev, "netdev(%d) No longer exists\n", vif);
		rcu_read_unlock();
		return -EINVAL;
	}
	rcu_read_unlock();

	ndev_vif = netdev_priv(dev);
	if (!ndev_vif->activated) {
		SLSI_NET_INFO(dev, "Packet dropped (vif is not activated)\n");
		return -EINVAL;
	}

#ifdef CONFIG_SCSC_WLAN_NAPI_PER_NETDEV
	skb_queue_tail(&ndev_vif->rx_data_q, skb);
	napi_schedule(&ndev_vif->rx_data_napi);
#else
	slsi_debug_frame(sdev, dev, skb, "RX");
	switch (fapi_get_u16(skb, id)) {
	case MA_UNITDATA_IND:
		slsi_rx_data_ind(sdev, dev, skb);

		/* SKBs in a BA session are not passed yet */
		slsi_spinlock_lock(&ndev_vif->ba_lock);
		if (atomic_read(&ndev_vif->ba_flush)) {
			atomic_set(&ndev_vif->ba_flush, 0);
			slsi_ba_process_complete(dev, true);
		}
		slsi_spinlock_unlock(&ndev_vif->ba_lock);
		break;
	default:
		SLSI_DBG1(sdev, SLSI_RX, "Unexpected Data: 0x%.4x\n", fapi_get_sigid(skb));
		kfree_skb(skb);
		break;
	}
#endif
	return 0;
}
#else
void slsi_rx_netdev_data_work(struct work_struct *work)
{
	struct slsi_skb_work *w = container_of(work, struct slsi_skb_work, work);
	struct slsi_dev *sdev = w->sdev;
	struct net_device *dev = w->dev;
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff *skb;

	if (WLBT_WARN_ON(!dev))
		return;

	slsi_wake_lock(&sdev->wlan_wl);

	while (1) {
		SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
		if (!ndev_vif->activated) {
			skb_queue_purge(&w->queue);
			SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
			break;
		}

		slsi_spinlock_lock(&ndev_vif->ba_lock);
		if (atomic_read(&ndev_vif->ba_flush)) {
			atomic_set(&ndev_vif->ba_flush, 0);
			slsi_ba_process_complete(dev, false);
		}
		slsi_spinlock_unlock(&ndev_vif->ba_lock);

		skb = slsi_skb_work_dequeue(w);
		if (!skb) {
			SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
			break;
		}

		slsi_debug_frame(sdev, dev, skb, "RX");
		switch (fapi_get_u16(skb, id)) {
		case MA_UNITDATA_IND:
#ifdef CONFIG_SCSC_SMAPPER
			if (slsi_skb_cb_get(skb)->smapper_linked) {
				u8 *frame = (u8 *)slsi_hip_get_skb_data_from_smapper(sdev, skb);

				if (frame)
					SCSC_HIP4_SAMPLER_TCP_DECODE(sdev, dev, frame, false);
			} else {
				SCSC_HIP4_SAMPLER_TCP_DECODE(sdev, dev, skb->data + fapi_get_siglen(skb), false);
			}
#else
			SCSC_HIP4_SAMPLER_TCP_DECODE(sdev, dev, skb->data + fapi_get_siglen(skb), false);
#endif
			slsi_rx_data_ind(sdev, dev, skb);
			break;
		default:
			SLSI_DBG1(sdev, SLSI_RX, "Unexpected Data: 0x%.4x\n", fapi_get_sigid(skb));
			kfree_skb(skb);
			break;
		}
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	}
	slsi_wake_unlock(&sdev->wlan_wl);
	/* reprocess hip_wq for skipped data BH */
	slsi_hip_reprocess_skipped_data_bh(sdev);

}

static int slsi_rx_queue_data(struct slsi_dev *sdev, struct sk_buff *skb)
{
	struct net_device *dev;
	struct netdev_vif *ndev_vif;
	int vif;

	vif = fapi_get_vif(skb);

	rcu_read_lock();
#ifdef CONFIG_SCSC_WIFI_NAN_ENABLE
	if (vif >= SLSI_NAN_DATA_IFINDEX_START && fapi_get_sigid(skb) == MA_UNITDATA_IND) {
		dev = slsi_nan_get_netdev_rcu(sdev, skb);
	} else {
		dev = slsi_get_netdev_rcu(sdev, vif);
	}
#else
	dev = slsi_get_netdev_rcu(sdev, vif);
#endif

	if (!dev) {
		SLSI_ERR(sdev, "netdev(%d) No longer exists\n", vif);
		rcu_read_unlock();
		goto err;
	}
	ndev_vif = netdev_priv(dev);

	slsi_skb_work_enqueue(&ndev_vif->rx_data, skb);
	rcu_read_unlock();
	return 0;
err:
	return -EINVAL;
}
#endif

static int sap_ma_rx_handler(struct slsi_dev *sdev, struct sk_buff *skb)
{
	int ret = -EINVAL;
#ifdef CONFIG_SCSC_SMAPPER
	u16 sig_len;
	u32 err;
#endif

	switch (fapi_get_sigid(skb)) {
	case MA_UNITDATA_IND:
#ifdef CONFIG_SCSC_SMAPPER
		/* Check SMAPPER to nullify entry*/
		if (slsi_skb_cb_get(skb)->smapper_linked) {
			sig_len = fapi_get_siglen(skb);
			skb_pull(skb, sig_len);
			err = slsi_hip_consume_smapper_entry(sdev, skb);
			skb_push(skb, sig_len);
			if (err)
				return err;
		}
#endif
#ifdef CONFIG_SCSC_WLAN_RX_NAPI
		spin_lock_bh(&sap_ma_lock);
		ret = slsi_rx_napi_process(sdev, skb);
		spin_unlock_bh(&sap_ma_lock);
#else
		ret = slsi_rx_queue_data(sdev, skb);
#endif
		return ret;
	default:
		break;
	}
	SLSI_ERR_NODEV("Shouldn't be getting here!\n");
	return ret;
}

/* Adjust the scod value and flow control appropriately. */
#ifdef CONFIG_SCSC_WLAN_TX_API
static int sap_ma_txdone(struct slsi_dev *sdev, u32 colour, bool more)
{
	return slsi_tx_done(sdev, colour, more);
}
#else
static int sap_ma_txdone(struct slsi_dev *sdev,  u8 vif, u8 peer_index, u8 ac)
{
	struct net_device *dev;
	struct slsi_peer *peer;

	rcu_read_lock();
	dev = slsi_get_netdev_rcu(sdev, vif);

	if (!dev) {
		SLSI_ERR(sdev, "netdev(%d) No longer exists\n", vif);
		rcu_read_unlock();
		return -EINVAL;
	}
	rcu_read_unlock();

	if (peer_index <= SLSI_PEER_INDEX_MAX) {
		/* peer_index = 0 for Multicast queues */
		if (peer_index == 0) {
			struct netdev_vif *ndev_vif = netdev_priv(dev);

			return scsc_wifi_fcq_receive_data(dev, &ndev_vif->ap.group_data_qs, ac, sdev, vif, peer_index);
		}
		peer = slsi_get_peer_from_qs(sdev, dev, MAP_AID_TO_QS(peer_index));
		if (peer)
			return scsc_wifi_fcq_receive_data(dev, &peer->data_qs, ac, sdev, vif, peer_index);

		SLSI_DBG1(sdev, SLSI_RX, "peer record NOT found for vif=%d peer_index=%d\n", vif, peer_index);
		/* We need to handle this case as special. Peer disappeared bug hip4
		 * is sending back the colours to free.
		 */
		return scsc_wifi_fcq_receive_data_no_peer(dev, ac, sdev, vif, peer_index);
	}
	SLSI_ERR(sdev, "illegal peer_index vif=%d peer_index=%d\n", vif, peer_index);
	return -EINVAL;
}
#endif

int sap_ma_init(void)
{
	SLSI_INFO_NODEV("Registering SAP\n");
	slsi_hip_sap_register(&sap_ma);
	return 0;
}

int sap_ma_deinit(void)
{
	SLSI_INFO_NODEV("Unregistering SAP\n");
	slsi_hip_sap_unregister(&sap_ma);
	return 0;
}
