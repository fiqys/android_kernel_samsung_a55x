/****************************************************************************
 *
 * Copyright (c) 2012 - 2026 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#ifndef _WONDERTAP_H_
#define _WONDERTAP_H_
#include "dev.h"
#include <wonder/wondertap.h>
#include <linux/ieee80211.h>

/*
 * Named encodings of the wonder/ public API version, matching the layout
 * documented for CONFIG_SCSC_WLAN_WONDERTAP (MMmmpp).
 *
 * Use these in `#if CONFIG_SCSC_WLAN_WONDERTAP >= SLSI_WONDER_API_*` to gate
 * code that depends on a particular wonder ops contract. The same config
 * also doubles as the master enable: `#if CONFIG_SCSC_WLAN_WONDERTAP` is the
 * canonical enable check (the macro is defined as 0 when disabled, so an
 * `#if`-style test correctly evaluates false).
 */
#define SLSI_WONDER_API_V1_5     10500
#define SLSI_WONDER_API_V1_5_1   10501
#define SLSI_WONDER_API_V1_6_1   10601
#define SLSI_WONDER_API_V1_6_4   10604

/*
 * Fall-back for out-of-tree builds where Kconfig didn't define the macro.
 * 0 means disabled, matching `int` Kconfig default-when-absent semantics.
 */
#ifndef CONFIG_SCSC_WLAN_WONDERTAP
#define CONFIG_SCSC_WLAN_WONDERTAP 0
#endif

#if CONFIG_SCSC_WLAN_WONDERTAP >= SLSI_WONDER_API_V1_6_4
struct slsi_wondertap_station_entry {
	struct list_head node;
	/* Station info exactly as reported by wonder. info.aid is wonder's aid,
	 * which lives in its own namespace and is metadata only.
	 */
	struct wondertap_station_info info;
	/* Internal aid handed to slsi_peer_add(), in
	 * [SLSI_PEER_INDEX_MIN, SLSI_PEER_INDEX_MAX]. Allocated by
	 * slsi_wondertap_alloc_peer_aid() and stable for the life of the peer.
	 */
	u16 peer_aid;
};

struct mcs_array {
	u32 legacy_rates;
	u16 ht_mcs[WONDERTAP_HT_NSS_MAX];
	u16 vht_mcs[WONDERTAP_VHT_NSS_MAX];
	u16 he_mcs[WONDERTAP_HE_NSS_MAX];
	u16 eht_mcs[WONDERTAP_EHT_NSS_MAX];
};
#endif

struct vendor_hardware_ctx {
	struct wondertap_set_freq_params freq;
	struct wondertap_fixed_tx_rate_params tx_rate;
	struct wondertap_tx_rate_mask_params rate_mask;
	bool initialized;
	u8 mac_addr[ETH_ALEN];
	u8 bssid[ETH_ALEN];
#if CONFIG_SCSC_WLAN_WONDERTAP >= SLSI_WONDER_API_V1_6_4
	struct wondertap_station_info sta_info;
	enum wondertap_station_action action;
	struct mcs_array mcs_array;
	struct list_head station_list;
	u16 flow_id;
	int station_info_len;
	u8 ra_enabled;
	u8 amsdu_enabled;
	u8 ampdu_enabled;
#endif
};
int slsi_wondertap_register(struct slsi_dev *sdev);
void slsi_wondertap_unregister(struct slsi_dev *sdev);

/* Platform driver registration hooks for module init/exit. */
int slsi_wondertap_pdrv_register(void);
void slsi_wondertap_pdrv_unregister(void);

/**
 * @brief Extract source and destination addresses from 802.11 header
 * @param skb The socket buffer containing 802.11 frame
 * @param da Pointer to store destination address
 * @param sa Pointer to store source address
 * @return 0 on success, -EINVAL on error
 */
static inline int slsi_tx_wondertap_addrs_from_80211(struct sk_buff *skb, u8 *da, u8 *sa)
{
	struct ieee80211_hdr *hdr;
	u8 *dst;
	u8 *src;

	if (!skb || skb->len < sizeof(struct ieee80211_hdr))
		return -EINVAL;

	hdr = (struct ieee80211_hdr *)(skb->head + skb->mac_header);

	dst = ieee80211_get_DA(hdr);
	src = ieee80211_get_SA(hdr);

	memcpy(da, dst, ETH_ALEN);
	memcpy(sa, src, ETH_ALEN);

	return 0;
}

/**
 * @brief Create a pseudo-Ethernet header for WonderTap TX API integration
 * @param skb The socket buffer containing raw 802.11 frame
 * @return 0 on success, negative error code on failure
 *
 * For WonderTap, we need to create a temporary Ethernet header so the TX API
 * can process the packet. The addresses are extracted from the 802.11 header.
 */
static inline int slsi_tx_wondertap_push_eth_header(struct sk_buff *skb)
{
	u8 da[ETH_ALEN], sa[ETH_ALEN];
	struct ethhdr *ethhdr;
	int ret;

	if (!skb)
		return -EINVAL;

	/* Extract addresses from 802.11 header */
	ret = slsi_tx_wondertap_addrs_from_80211(skb, da, sa);
	if (ret < 0)
		return ret;

	/* keep tx_d */
	skb_push(skb, sizeof(struct wonder_txd));
	/* Push Ethernet header */
	ethhdr = (struct ethhdr *)skb_push(skb, sizeof(struct ethhdr));

	/* Set Ethernet addresses */
	memcpy(ethhdr->h_dest, da, ETH_ALEN);
	memcpy(ethhdr->h_source, sa, ETH_ALEN);
	ethhdr->h_proto = htons(ETH_P_802_2); /* Use 802.2 for raw frames */

	return 0;
}

/* Worst-case buffer size for slsi_wondertap_sta_capa_to_ies(): all four
 * capability elements present (2-byte element header for HT/VHT, 3 bytes
 * for the extension elements). For HE, 12 covers the largest Supported
 * HE-MCS And NSS Set and 1 the minimal PPE Thresholds field.
 */
#define SLSI_WONDERTAP_STA_CAPA_IES_MAX_LEN \
	(2 + sizeof(struct ieee80211_ht_cap) + \
	 2 + sizeof(struct ieee80211_vht_cap) + \
	 3 + sizeof(struct ieee80211_he_cap_elem) + 12 + 1 + \
	 3 + sizeof(struct ieee80211_he_6ghz_capa))

/**
 * @brief Append one 802.11 information element (TLV)
 * @param pos Write position in the output buffer
 * @param end End of the output buffer (first byte past it)
 * @param eid Element ID
 * @param data Element payload
 * @param len Element payload length
 * @return New write position, or NULL if the element does not fit
 */
static inline u8 *slsi_wondertap_put_ie(u8 *pos, const u8 *end, u8 eid, const void *data, u8 len)
{
	if (pos + 2 + len > end)
		return NULL;

	*pos++ = eid;
	*pos++ = len;
	memcpy(pos, data, len);

	return pos + len;
}

/**
 * @brief Append one 802.11 extension information element (TLV)
 * @param pos Write position in the output buffer
 * @param end End of the output buffer (first byte past it)
 * @param ext_eid Element ID Extension (payload is prefixed with it)
 * @param data Element payload (without the Element ID Extension byte)
 * @param len Element payload length (without the Element ID Extension byte),
 *            at most 254 so the Length byte (len + 1) cannot wrap
 * @return New write position, or NULL if the element does not fit or len
 *         exceeds 254
 */
static inline u8 *slsi_wondertap_put_ext_ie(u8 *pos, const u8 *end, u8 ext_eid, const void *data, u8 len)
{
	/* The Length byte covers the Element ID Extension too: a 255-byte
	 * payload would wrap it to 0 and produce a malformed element.
	 */
	if (len > 254)
		return NULL;

	if (pos + 3 + len > end)
		return NULL;

	*pos++ = WLAN_EID_EXTENSION;
	*pos++ = len + 1;
	*pos++ = ext_eid;
	memcpy(pos, data, len);

	return pos + len;
}

/**
 * @brief Pack station PHY capabilities into standard 802.11 information elements
 * @param info Station information from the WonderTap core
 * @param buf Output buffer for the packed elements
 * @param buf_len Size of the output buffer
 * @return Total number of bytes written on success (0 if no capability is
 *         set in capability_mask), -EINVAL on invalid arguments, -E2BIG if
 *         the buffer is too small
 *
 * Packs the capabilities flagged in info->capability_mask as standard
 * 802.11 TLV information elements, in element ID order:
 *   - HT Capabilities (EID 45, struct ieee80211_ht_cap)
 *   - VHT Capabilities (EID 191, struct ieee80211_vht_cap)
 *   - HE Capabilities (EID 255 / EID ext 35, struct ieee80211_he_cap_elem)
 *   - HE 6GHz Band Capabilities (EID 255 / EID ext 59, struct ieee80211_he_6ghz_capa)
 *
 * A buffer of SLSI_WONDERTAP_STA_CAPA_IES_MAX_LEN bytes is always large
 * enough.
 *
 * Note on HE: wondertap_station_info only carries the fixed portion of the
 * HE Capabilities element (struct ieee80211_he_cap_elem). To keep the
 * element spec-conformant, the mandatory Supported HE-MCS And NSS Set
 * (sized from the channel width bits in phy_cap_info) and, when flagged
 * present in phy_cap_info, a minimal PPE Thresholds field are zero-filled.
 * Beware that an all-zero HE-MCS map semantically advertises "MCS 0-7
 * supported" for every spatial stream (not-supported would be 0b11 per
 * stream); replace the zero-fill with real data once the wonder core
 * provides it. he_capa_len is clamped to the fixed portion; a short
 * he_capa_len is also zero-padded up to the fixed portion size.
 */
static inline int slsi_wondertap_sta_capa_to_ies(const struct wondertap_station_info *info,
						 u8 *buf, size_t buf_len)
{
	u8 *pos = buf;
	const u8 *end = buf + buf_len;

	if (!info || !buf)
		return -EINVAL;

	if (info->capability_mask & BIT(WONDERTAP_STATION_CAP_HT)) {
		pos = slsi_wondertap_put_ie(pos, end, WLAN_EID_HT_CAPABILITY,
					    &info->ht_capa, sizeof(info->ht_capa));
		if (!pos)
			return -E2BIG;
	}

	if (info->capability_mask & BIT(WONDERTAP_STATION_CAP_VHT)) {
		pos = slsi_wondertap_put_ie(pos, end, WLAN_EID_VHT_CAPABILITY,
					    &info->vht_capa, sizeof(info->vht_capa));
		if (!pos)
			return -E2BIG;
	}

	if (info->capability_mask & BIT(WONDERTAP_STATION_CAP_HE)) {
		u8 fixed_len = info->he_capa_len;
		u8 mcs_nss_len = ieee80211_he_mcs_nss_size(&info->he_capa);
		u8 ppe_len = (info->he_capa.phy_cap_info[6] &
			      IEEE80211_HE_PHY_CAP6_PPE_THRESHOLD_PRESENT) ? 1 : 0;
		u8 elem_len = sizeof(info->he_capa) + mcs_nss_len + ppe_len;

		if (!fixed_len || fixed_len > sizeof(info->he_capa))
			fixed_len = sizeof(info->he_capa);

		/* The element payload is not contiguous in the source (the
		 * fixed part comes from he_capa, the rest is zero-filled), so
		 * slsi_wondertap_put_ext_ie() cannot be used here.
		 */
		if (pos + 3 + elem_len > end)
			return -E2BIG;

		*pos++ = WLAN_EID_EXTENSION;
		*pos++ = elem_len + 1;
		*pos++ = WLAN_EID_EXT_HE_CAPABILITY;
		memcpy(pos, &info->he_capa, fixed_len);
		memset(pos + fixed_len, 0, elem_len - fixed_len);
		pos += elem_len;
	}

	if (info->capability_mask & BIT(WONDERTAP_STATION_CAP_HE_6G)) {
		pos = slsi_wondertap_put_ext_ie(pos, end, WLAN_EID_EXT_HE_6GHZ_CAPA,
						&info->he_6ghz_capa, sizeof(info->he_6ghz_capa));
		if (!pos)
			return -E2BIG;
	}

	return pos - buf;
}
#endif /* _WONDERTAP_H_ */
