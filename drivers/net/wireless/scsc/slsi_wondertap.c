/****************************************************************************
 *
 * Copyright (c) 2012 - 2026 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/component.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/bitops.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include <linux/if_arp.h>
#include <scsc/scsc_mx.h>
#include "slsi_wondertap.h"
#include "debug.h"
#include "mlme.h"
#include "dev.h"
#include "mgt.h"
#include "tx_api.h"
#include "netif.h"
#include "txbp.h"
#include "hip.h"

/* ============================================================================
 * 2. Implementation of wondertap_ops (Callbacks)
 * ============================================================================
 * These functions are called by the WonderTap Core Driver.
 */
/**
 * vendor_wondertap_init - Initializes the vendor driver instance.
 * @handle:  Double pointer to be populated with the vendor context.
 * @params:  Initialization parameters from the core.
 *
 * Return: 0 on success, negative error code on failure.
 */
static struct slsi_dev               *sdev;
static struct net_device             *wondertap_dev;
static struct platform_device        *slsi_wondertap_pdev;
static atomic_t                       slsi_wondertap_component_added = ATOMIC_INIT(0);
/*
 * slsi_wondertap_op_mutex serializes all wondertap_ops invocations against
 * teardown (slsi_wondertap_unregister / component unbind). All ops must take
 * this mutex before touching `sdev` / `wondertap_dev` / the active ctx, and
 * teardown takes it after component_del to drain any in-flight callers
 * before clearing the static state.
 *
 * Lock order (do not deviate):
 *      component_mutex            (held by component_add/component_del across
 *        -> rtnl_lock              our .bind/.unbind callbacks)
 *          -> wondertap->lock     (wonder side)
 *            -> slsi_wondertap_op_mutex
 *              -> sdev->netdev_add_remove_mutex
 *                -> sdev->start_stop_mutex
 *                  -> slsi_start_mutex
 *                    -> ndev_vif->vif_mutex -> ndev_vif->peer_lock
 *
 * Two consequences, both load-bearing:
 *  - Nothing holding slsi_start_mutex may take component_mutex or
 *    slsi_wondertap_op_mutex. That is why cm_if.c calls
 *    slsi_wondertap_register()/_unregister() outside slsi_start_mutex, and
 *    why slsi_wondertap_get_capabilities() (reached from wonder_master_bind
 *    inside component_add) takes no lock at all.
 *  - Because the unbind takes rtnl, slsi_wondertap_unregister() may only be
 *    called from a context that is allowed to block on it. It is NOT called
 *    from wlan_stop_on_failure_v2(), which runs under the scsc core's
 *    srvman->service_list_mutex; slsi_wlan_service_remove() does it instead.
 *  - Nothing holding slsi_wondertap_op_mutex may take rtnl_lock.
 *
 * ->init/->deinit always arrive with rtnl held (mac80211 .start/.stop via
 * ndo_open/ndo_stop and ieee80211_*register_hw); ->init proves it by calling
 * dev_change_flags(). Some ops arrive without rtnl (wonder's sta-update
 * workqueue item, its debugfs files, and nl80211 vendor commands on kernels
 * where NL80211_CMD_VENDOR does not carry NL80211_FLAG_NEED_RTNL) - that is
 * fine, it is only a shorter prefix of the same order. What is never allowed
 * is taking rtnl underneath op_mutex.
 *
 * slsi_wondertap_active_ctx tracks the live ctx allocated by ->init(). It
 * lets the unbind callback free the ctx if WLAN service goes down before
 * userspace called ->deinit(), and it lets ops detect stale handles after
 * a previous teardown freed the ctx.
 */
static DEFINE_MUTEX(slsi_wondertap_op_mutex);
static struct vendor_hardware_ctx    *slsi_wondertap_active_ctx;
#if CONFIG_SCSC_WLAN_WONDERTAP >= SLSI_WONDER_API_V1_6_4
static struct slsi_wondertap_station_entry *
slsi_wondertap_station_find(struct vendor_hardware_ctx *ctx, const u8 *mac)
{
	struct slsi_wondertap_station_entry *entry;

	list_for_each_entry(entry, &ctx->station_list, node)
		if (ether_addr_equal(entry->info.mac, mac))
			return entry;
	return NULL;
}

/*
 * Allocate the internal aid that we pass to slsi_peer_add().
 *
 * Wonder's aid and the driver's aid are two separate namespaces:
 *
 *   wonder aid  - an upper layer identifier. It is an 802.11 AID (1..2007) and
 *                 typically increases monotonically as stations come and go, so
 *                 it must never be used as an array index.
 *   internal aid - selects the queueset, since the driver derives it as aid-1
 *                 (see netif.h MAP_AID_TO_QS) to index peer_sta_record[], which
 *                 only has SLSI_PEER_INDEX_MAX slots.
 *
 * So wonder's aid is carried as metadata in station_entry->info.aid and the
 * internal aid is allocated here as the first free queueset. That decoupling is
 * safe because nothing in the firmware interface depends on the aid:
 * MLME_RELAY_SET_PEER_INFO_REQ identifies the peer by MAC address, and TX
 * addresses it through the flow_id that the CFM hands back (see peer->flow_id
 * use in tx.c), so the driver is free to pick any free slot.
 *
 * Caller must hold ndev_vif->peer_lock so the peer_sta_record[] scan is stable.
 * Returns 0 only when every peer record is in use.
 */
static u16 slsi_wondertap_alloc_peer_aid(struct net_device *dev)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	u16 qs;

	for (qs = 0; qs < SLSI_PEER_INDEX_MAX; qs++) {
		struct slsi_peer *p = ndev_vif->peer_sta_record[qs];

		if (!p || !p->valid)
			return MAP_QS_TO_AID(qs);
	}
	return 0;
}
#endif

static int slsi_wondertap_interface_add(void)
{
	struct netdev_vif *ndev_vif;
	int ret = 0;
	bool is_cfg80211 = false;
	/* sdev->netdev[] is __rcu and slsi_get_netdev_locked() asserts
	 * netdev_add_remove_mutex, so both the existence test and the cached
	 * pointer must be taken under it.
	 */
	SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);
	wondertap_dev = slsi_get_netdev_locked(sdev, SLSI_NET_INDEX_RELAY);
	if (wondertap_dev) {
		/* Re-attach to the surviving netdev. Without this, a stale
		 * wondertap_dev == NULL makes every subsequent init fail with
		 * -ENODEV while the interface is still present.
		 */
		SLSI_ERR(sdev, "[Wonder]Interface already exists : %s\n", wondertap_dev->name);
		SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
		return ret;
	}
	ret = slsi_netif_add_locked(sdev, "wondertap0", SLSI_NET_INDEX_RELAY);
		if (ret) {
			SLSI_ERR(sdev, "[Wonder] fail net_if_add wondertap0, if_idx:%d\n", SLSI_NET_INDEX_RELAY);
			SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
			return ret;
	}
	wondertap_dev = slsi_get_netdev_locked(sdev, SLSI_NET_INDEX_RELAY);
	if (!wondertap_dev) {
		SLSI_ERR(sdev, "[Wonder]slsi_get_netdev_locked fail");
		SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
		return -ENODEV;
	}
	SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
	/* We can use slsi_netif_register_locked instead of slsi_netif_register_rtlnl_locked
	 * To NOT hold `sdev->netdev_add_remove_mutex` during the registration call.
	 * This requires verifying that no race condition is introduced (the netdev was just created and should be safe to register).
	 * Holding sdev->netdev_add_remove_mutex results in deadlock.
	 */
	ret = slsi_netif_register_locked(sdev, wondertap_dev, is_cfg80211);
	if (ret) {
		SLSI_ERR(sdev, "[Wonder]wondertap0 interface registration fail:%d\n", ret);
		return ret;
	} else {
		SLSI_ERR(sdev, "[Wonder] wondertap0 interface registration success, name : %s\n", wondertap_dev->name);
		ndev_vif = netdev_priv(wondertap_dev);
		SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
		ndev_vif->vif_type = FAPI_VIFTYPE_RELAY;
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	}
	return ret;
}
/**
 * slsi_wondertap_interface_remove - drop the wondertap0 netdev.
 *
 * MUST be called with rtnl_lock held: slsi_netif_remove_rtlnl_locked() ->
 * slsi_netif_remove_locked() calls unregister_netdevice()/free_netdev(),
 * both of which require the rtnl lock. All callers reach here either from a
 * mac80211 .start/.stop callback (rtnl taken by the netdev core) or from
 * slsi_wondertap_component_unbind(), which takes rtnl itself.
 *
 * rtnl MUST be taken *before* slsi_wondertap_op_mutex, never the other way
 * round: the wondertap_ops entry points are all invoked with rtnl already
 * held, so acquiring rtnl underneath op_mutex would invert the order.
 */
static void slsi_wondertap_interface_remove(void)
{
	bool is_cfg80211 = false;

	if (!wondertap_dev)
		return;
	if (!sdev) {
		/* No sdev to unregister against; just drop our reference so a
		 * later init() does not dereference a stale pointer.
		 */
		SLSI_WARN_NODEV("[Wonder] interface_remove with no sdev, leaking netdev\n");
		wondertap_dev = NULL;
		return;
	}
	WLBT_WARN_ON(!rtnl_is_locked());
	SLSI_INFO(sdev, "wonder_interface_remove: detach %s\n", wondertap_dev->name);
	slsi_netif_remove_rtlnl_locked(sdev, wondertap_dev, is_cfg80211);
	wondertap_dev = NULL;
}
static bool slsi_wondertap_nan_activated(void)
{
	struct net_device *dev = slsi_get_netdev(sdev, SLSI_NET_INDEX_NAN);
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	if (ndev_vif->activated && ndev_vif->vif_type == FAPI_VIFTYPE_NAN)
		return true;
	return false;
}
static bool slsi_wondertap_sta_connected(void)
{
	struct net_device *dev = slsi_get_netdev(sdev, SLSI_NET_INDEX_WLAN);
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	if (ndev_vif->activated && ndev_vif->vif_type == FAPI_VIFTYPE_STATION &&
	    (ndev_vif->sta.vif_status == SLSI_VIF_STATUS_CONNECTING ||
	     ndev_vif->sta.vif_status == SLSI_VIF_STATUS_CONNECTED)) {
		/* Scenario A (DBS, e.g. Redwood): STA coexists with Wonder, allow */
		/* Scenario B (non-DBS, e.g. Rose/Santa/Terra): any STA blocks Wonder */
		if (sdev->dualband_concurrency) {
			SLSI_INFO(sdev, "[Wonder]STA connected, DBS supported -> Wonder init allowed\n");
			return false;
		}
		SLSI_ERR(sdev, "[Wonder]STA connected. Cannot start Relay\n");
		return true;
	}
	return false;
}

static bool slsi_wondertap_softap_activated(void)
{
	struct net_device *dev;
	struct netdev_vif *ndev_vif;
	dev = slsi_get_netdev(sdev, SLSI_NET_INDEX_AP);
	if (dev) {
		ndev_vif = netdev_priv(dev);
		if (ndev_vif->activated)
			return true;
	}
	dev = slsi_get_netdev(sdev, SLSI_NET_INDEX_AP2);
	if (dev) {
		ndev_vif = netdev_priv(dev);
		if (ndev_vif->activated)
			return true;
	}
	dev = slsi_get_netdev(sdev, SLSI_NET_INDEX_AP_VLAN + sdev->num_ap_vlan);
	if (dev) {
		ndev_vif = netdev_priv(dev);
		if (ndev_vif->activated)
			return true;
	}
	dev = slsi_get_netdev(sdev, SLSI_NET_INDEX_P2PX_SWLAN);
	if (dev) {
		ndev_vif = netdev_priv(dev);
		if (ndev_vif->activated)
			return true;
	}
	return false;
}
/**
 * slsi_wondertap_station_cache_purge - free the cached wondertap_station_info
 * entries owned by @ctx. Without this every deinit (and every unbind after a
 * FW panic) leaks one entry per station wonder ever reported.
 */
static void slsi_wondertap_station_cache_purge(struct vendor_hardware_ctx *ctx)
{
#if CONFIG_SCSC_WLAN_WONDERTAP >= SLSI_WONDER_API_V1_6_4
	struct slsi_wondertap_station_entry *entry, *tmp;

	if (!ctx || !ctx->station_list.next)
		return;
	list_for_each_entry_safe(entry, tmp, &ctx->station_list, node) {
		list_del(&entry->node);
		kfree(entry);
	}
#else
	SLSI_UNUSED_PARAMETER(ctx);
#endif
}

/**
 * slsi_wondertap_deinit_ctx - Internal deinit helper.
 *
 * Caller MUST hold rtnl_lock and slsi_wondertap_op_mutex, in that order.
 * Shared by slsi_wondertap_deinit() (normal path), the re-init teardown in
 * slsi_wondertap_init(), and slsi_wondertap_component_unbind().
 */
static void slsi_wondertap_deinit_ctx(struct vendor_hardware_ctx *ctx)
{
	struct netdev_vif *ndev_vif;

	if (!sdev || !wondertap_dev)
		goto free_ctx;

	ndev_vif = netdev_priv(wondertap_dev);
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (ndev_vif->activated) {
		/* Same convention as slsi_stop_net_dev(): during recovery the
		 * chip is gone, so a MLME request would only burn the cfm
		 * timeout (seconds) while rtnl_lock and op_mutex are held, and
		 * may trip missing_cfm_ind_panic into a nested panic.
		 */
		bool hw_available = !sdev->recovery_status && !sdev->mlme_blocked &&
				    sdev->wlan_service_on;

		if (hw_available && slsi_mlme_del_vif(sdev, wondertap_dev) != 0)
			SLSI_ERR(sdev, "[Wonder]slsi_mlme_del_vif failed\n");
		slsi_vif_deactivated(sdev, wondertap_dev);
	}
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

free_ctx:
	slsi_wondertap_station_cache_purge(ctx);
	slsi_wondertap_interface_remove();
	slsi_wondertap_active_ctx = NULL;
	kfree(ctx);
}

static int slsi_wondertap_init(void **handle, const struct wondertap_init_params *params)
{
	struct vendor_hardware_ctx *ctx = NULL;
	struct netdev_vif *ndev_vif = NULL;
	u8 device_address[ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	char country_code[3];
	int ret = 0;
	mutex_lock(&slsi_wondertap_op_mutex);
	/* Clear the caller's handle up front so that every failure path below
	 * leaves it NULL rather than the previous (already freed) session's
	 * ctx. It is re-published only once init has fully succeeded.
	 */
	*handle = NULL;
	/*
	 * Prevention: if init is called again without a matching deinit (e.g.,
	 * userspace re-initializes after interface deletion), the previous ctx
	 * and network interface are still live. Tear them down first so we
	 * start from a clean slate.
	 */
	if (slsi_wondertap_active_ctx) {
		SLSI_WARN_NODEV("[Wonder] re-init without deinit: tearing down previous context\n");
		slsi_wondertap_deinit_ctx(slsi_wondertap_active_ctx);
	}
	if (!sdev) {
		SLSI_ERR_NODEV("[Wonder] sdev is NULL\n");
		ret = -ENODEV;
		goto err_unlock;
	}
	/* Everything that can reject the request without touching wondertap0 is
	 * checked BEFORE the interface is created. wonder retries ->init() six
	 * times with a 400 ms gap while holding rtnl, so creating and
	 * unregistering a netdev per attempt would hold rtnl for seconds and
	 * churn synchronize_rcu() for nothing.
	 */
	if (!params) {
		SLSI_ERR(sdev, "[Wonder] init error: NULL params\n");
		ret = -EINVAL;
		goto err_unlock;
	}
	if (!sdev->wlan_service_on) {
		SLSI_ERR(sdev, "[Wonder]WLAN service is not on. Cannot start Relay\n");
		ret = -EBUSY;
		goto err_unlock;
	}
	if (sdev->mlme_blocked) {
		SLSI_ERR(sdev, "[Wonder]MLME is blocked. Cannot start Relay\n");
		ret = -EBUSY;
		goto err_unlock;
	}
	if (sdev->p2p_state == P2P_SCANNING || sdev->p2p_state == P2P_LISTENING
		|| sdev->p2p_state == P2P_GROUP_FORMED_CLI || sdev->p2p_state == P2P_GROUP_FORMED_GO) {
		SLSI_ERR(sdev, "[Wonder]P2P is running. Cannot start Relay\n");
		ret = -EBUSY;
		goto err_unlock;
	}
	if (slsi_wondertap_nan_activated()) {
		SLSI_ERR(sdev, "[Wonder]NAN is running. Cannot start Relay\n");
		ret = -EBUSY;
		goto err_unlock;
	}
	if (slsi_wondertap_sta_connected()) {
		ret = -EBUSY;
		goto err_unlock;
	}
	if (slsi_wondertap_softap_activated()) {
		SLSI_ERR(sdev, "[Wonder]MHS is activated. Cannot start Relay\n");
		ret = -EBUSY;
		goto err_unlock;
	}
	/* Allocate the vendor context before the interface so that an -ENOMEM
	 * here also costs nothing.
	 */
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto err_unlock;
	}
#if CONFIG_SCSC_WLAN_WONDERTAP >= SLSI_WONDER_API_V1_6_4
	/* Must be valid before any error path can reach the cache purge. */
	INIT_LIST_HEAD(&ctx->station_list);
#endif
	ret = slsi_wondertap_interface_add();
	if (ret) {
		SLSI_ERR(sdev, "[Wonder] wonder_interface_add failed: %d\n", ret);
		/* Do NOT clear the global sdev here: it would make every later
		 * wondertap op return -ENODEV until the WLAN service is probed
		 * again, so wonder could never retry a failed init.
		 */
		goto err_free_ctx;
	}
	if (!wondertap_dev) {
		SLSI_ERR(sdev, "[Wonder]failed to dev. Cannot start Relay\n");
		ret = -ENODEV;
		goto err_free_ctx;
	}
	ndev_vif = netdev_priv(wondertap_dev);
	if (!ndev_vif) {
		SLSI_ERR(sdev, "[Wonder] ndev_vif is NULL\n");
		ret = -ENODEV;
		goto err_free_ctx;
	}
	/* Store initial parameters */
	ctx->freq = params->channel;
	ctx->tx_rate = params->tx_rate;
	if((ctx->tx_rate.preamble == WONDERTAP_RATE_PREAMBLE_HT || ctx->tx_rate.preamble == WONDERTAP_RATE_PREAMBLE_VHT) && (ctx->tx_rate.gi > 1)) {
		SLSI_ERR(sdev, "[Wonder]A GI that does not match the parameters was delivered. preamble :%d, gi:%d\n", ctx->tx_rate.preamble, ctx->tx_rate.gi);
		ctx->tx_rate.gi = WONDERTAP_RATE_GI_DEFAULT;
		SLSI_INFO(sdev, "[Wonder]Changed the tx_rate.gi:%d\n", ctx->tx_rate.gi);
	}
	if((ctx->tx_rate.preamble == WONDERTAP_RATE_PREAMBLE_HE || ctx->tx_rate.preamble == WONDERTAP_RATE_PREAMBLE_EHT) && (ctx->tx_rate.gi < 2))	{
		SLSI_ERR(sdev, "[Wonder]A GI that does not match the parameters was delivered. preamble :%d, gi:%d\n", ctx->tx_rate.preamble, ctx->tx_rate.gi);
		if(ctx->tx_rate.gi == WONDERTAP_RATE_GI_DEFAULT) {
			ctx->tx_rate.gi = WONDERTAP_RATE_GI_0_8_US;
			SLSI_INFO(sdev, "[Wonder]Changed the tx_rate.gi:%d\n", ctx->tx_rate.gi);
		} else {
			SLSI_ERR(sdev, "[Wonder] Invalid tx_rate.gi : %d\n", ctx->tx_rate.gi);
			ret = -EINVAL;
			goto err_free_ctx;
		}
	}
	SLSI_INFO(sdev, "[Wonder]ctx->tx_rate.preamble:%d ctx->tx_rate.bw:%d, ctx->tx_rate.gi:%d, ctx->tx_rate.nss:%d, ctx->tx_rate.mcs:%d\n",
		ctx->tx_rate.preamble, ctx->tx_rate.bw, ctx->tx_rate.gi, ctx->tx_rate.nss, ctx->tx_rate.mcs);
	memcpy(ctx->mac_addr, params->mac_addr, ETH_ALEN);
	memcpy(ctx->bssid, params->bssid, ETH_ALEN);
#if CONFIG_SCSC_WLAN_WONDERTAP >= SLSI_WONDER_API_V1_6_4
	ctx->amsdu_enabled = params->amsdu_enable;
	ctx->ampdu_enabled = params->ampdu_enable;
	ctx->ra_enabled = params->rate_adaptation_enable;
	ctx->rate_mask = params->tx_rate_mask;
	SLSI_INFO(sdev, "[Wonder]ctx->amsdu_enabled:%d, ctx->ampdu_enabled:%d, ctx->ra_enabled:%d,"
		"ctx->rate_mask._max_preamble:%d, ctx->rate_mask.max_bw:%d, ctx->rate_mask.max_nss:%d, ctx->rate_mask.max_mcs:%d\n",
		ctx->amsdu_enabled, ctx->ampdu_enabled, ctx->ra_enabled,
		ctx->rate_mask.max_preamble, ctx->rate_mask.max_bw, ctx->rate_mask.max_nss, ctx->rate_mask.max_mcs);
#endif
	ctx->initialized = true;
	/*	Bring the interface up if it is not already up	*/
	wondertap_dev->type = ARPHRD_IEEE80211_RADIOTAP;
	eth_hw_addr_set(wondertap_dev, params->mac_addr);
	ret = dev_change_flags(wondertap_dev, wondertap_dev->flags | IFF_UP, NULL);
	if (ret) {
		SLSI_ERR(sdev, "[Wonder]Failed to bring up dev(%s): %d\n", wondertap_dev->name, ret);
		goto err_free_ctx;
	}
	/* ADD COUNTRY CODE */
	memcpy(country_code, params->country_code, 2);
	country_code[2] = '\0';
	ret = slsi_mlme_set_country(sdev, country_code);
	if (ret) {
		SLSI_ERR(sdev, "[Wonder]failed to set country. Cannot start Relay\n");
		goto err_free_ctx;
	}
	SLSI_INFO(sdev, "[Wonder]BSSID:%pM, MAC: %pM, Country: %s\n", ctx->bssid, ctx->mac_addr, params->country_code);
	SLSI_INFO(sdev, "[Wonder]Retry Limits - Mgmt:%d, Data:%d\n", params->mgmt_retry_limit, params->data_retry_limit);
	/* MLME ADD VIF */
	if (ndev_vif->activated) {
		SLSI_INFO(sdev, "[Wonder]Already Enabled. Req Rejected\n");
		ret = -EBUSY;
		goto err_free_ctx;
	}
	ndev_vif->vif_type = FAPI_VIFTYPE_RELAY;
	ret = slsi_mlme_add_vif(sdev, wondertap_dev, ctx->mac_addr, device_address);
	if (ret) {
			SLSI_ERR(sdev, "[Wonder]failed to add relay vif. Cannot start Relay\n");
			goto err_free_ctx;
	}
	if (slsi_vif_activated(sdev, wondertap_dev) != 0)
		SLSI_ERR(sdev, "slsi_vif_activated failed\n");
	if (!sdev->wlan_service_on || !ndev_vif->tx_netdev_data) {
		SLSI_ERR(sdev, "[Wonder]cannot init tx_netdev_data: service_on=%d, tx_priv=%p\n",
				 sdev->wlan_service_on, ndev_vif->tx_netdev_data);
		ret = -EBUSY;
		goto err_del_vif;
	}

	/* MLME RELAY START */
	ret = slsi_mlme_relay_start_request(sdev, wondertap_dev, ctx);
	if (ret) {
		SLSI_ERR(sdev, "[Wonder]failed to relay start\n");
		goto err_del_vif;
	}
	SLSI_INFO(sdev, "[Wonder]Relay vif and start success\n");
	/* Publish the handle only now. Everything above can still fail and
	 * kfree(ctx), and wonder does not clear wondertap->vendor_handle on a
	 * failed ->init(), so an early assignment would leave it dangling.
	 */
	*handle = ctx;
	slsi_wondertap_active_ctx = ctx; /* track for leak prevention on unbind */
	mutex_unlock(&slsi_wondertap_op_mutex);
	return 0;
err_del_vif:
	if (ndev_vif) {
		SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
		if (ndev_vif->activated) {
			bool hw_available = !sdev->recovery_status && !sdev->mlme_blocked &&
				    sdev->wlan_service_on;

			if (hw_available && slsi_mlme_del_vif(sdev, wondertap_dev) != 0)
				SLSI_ERR(sdev, "[Wonder]slsi_mlme_del_vif failed\n");
			/* Use the common helper: clearing ->activated by hand leaks
			 * the peer records and the tx_netdev_data of the vif.
			 */
			slsi_vif_deactivated(sdev, wondertap_dev);
		}
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	}
err_free_ctx:
	kfree(ctx);
	/* Every failure from here on is after slsi_wondertap_interface_add(),
	 * so wondertap0 must be dropped again: no ctx was published, hence
	 * component_unbind cannot clean it up either. Checks that can reject
	 * the request without an interface all run earlier and jump straight
	 * to err_unlock.
	 */
	slsi_wondertap_interface_remove();
err_unlock:
	mutex_unlock(&slsi_wondertap_op_mutex);
	return ret;
}
/**
 * vendor_wondertap_deinit - Deinitializes the vendor driver instance.
 * @handle: The opaque driver instance handle.
 *
 * Note: This operation signature does not include extra params in the ops struct.
 */
static void slsi_wondertap_deinit(void *handle, const struct wondertap_deinit_params *params)
{
	struct vendor_hardware_ctx *ctx = handle;

	SLSI_UNUSED_PARAMETER(params);

	if (!ctx) {
		SLSI_ERR_NODEV("[Wonder]deinit called with NULL handle\n");
		return;
	}
	mutex_lock(&slsi_wondertap_op_mutex);
	if (ctx != slsi_wondertap_active_ctx) {
		/*
		 * Stale handle: ctx was already freed (e.g., by unbind cleanup
		 * after WLAN service teardown). Don't double-free or deref.
		 */
		SLSI_WARN_NODEV("[Wonder]deinit called with stale handle, ignoring\n");
		mutex_unlock(&slsi_wondertap_op_mutex);
		return;
	}
	if (sdev)
		SLSI_INFO(sdev, "[Wonder]deinit called\n");
	slsi_wondertap_deinit_ctx(ctx);
	mutex_unlock(&slsi_wondertap_op_mutex);
}
/**
 * vendor_wondertap_set_freq - Sets the operating channel and bandwidth.
 * @handle: The driver instance handle.
 * @params: Pointer to the channel configuration.
 */
static int slsi_wondertap_set_freq(void *handle, const struct wondertap_set_freq_params *params)
{
	struct vendor_hardware_ctx *ctx = handle;
	int ret;
	if (!ctx)
		return -EINVAL;
	mutex_lock(&slsi_wondertap_op_mutex);
	if (!sdev || ctx != slsi_wondertap_active_ctx || !ctx->initialized) {
		mutex_unlock(&slsi_wondertap_op_mutex);
		return -ENODEV;
	}
	SLSI_INFO(sdev, "[Wonder]Set Frequency - Center Freq: %d MHz, BW: %d\n", params->freq, params->bandwidth);
	ret = slsi_mlme_relay_set_channel_request(sdev, wondertap_dev, params);
	if (ret)
		SLSI_ERR(sdev, "[Wonder]Failed to set frequency\n");
	else
		SLSI_INFO(sdev, "[Wonder]Set frequency success\n");
	if (!ret)
		ctx->freq = *params;
	mutex_unlock(&slsi_wondertap_op_mutex);
	return ret;
}
/**
 * vendor_wondertap_set_filter - Configures a packet filter.
 * @handle:       The driver instance handle.
 * @filter_type:   Type of filter to configure.
 * @params:       Void pointer to filter-specific params.
 */
static int slsi_wondertap_set_filter(void *handle, enum wondertap_filter_type filter_type, const void *params)
{
	struct vendor_hardware_ctx *ctx = handle;
	int ret = 0;
	if (!ctx)
		return -EINVAL;
	mutex_lock(&slsi_wondertap_op_mutex);
	if (!sdev || ctx != slsi_wondertap_active_ctx || !ctx->initialized) {
		mutex_unlock(&slsi_wondertap_op_mutex);
		return -ENODEV;
	}
	if (filter_type == WONDERTAP_FILTER_TYPE_FRAME) {
		const struct wondertap_frame_filter_params *fp = params;
		SLSI_INFO(sdev, "[Wonder]Set Frame Filter - Enabled:%d, Type:0x%04x, Subtype:0x%04x\n",
				fp->enabled, fp->frame_type, fp->frame_subtype);
		if (fp->enabled)
			/* Enable hardware filter logic */
			SLSI_INFO(sdev, "[Wonder]Hardware filter configured for Type:0x%04x\n", fp->frame_type);
	} else {
		SLSI_ERR(sdev, "[Wonder]Unsupported filter type %d\n", filter_type);
		mutex_unlock(&slsi_wondertap_op_mutex);
		ret = -EINVAL;
	}
	mutex_unlock(&slsi_wondertap_op_mutex);
	return ret;
}
/**
 * vendor_wondertap_set_fixed_tx_rate - Configures a fixed transmission rate.
 * @handle: The driver instance handle.
 * @params: Pointer to the rate configuration.
 */
static int slsi_wondertap_set_fixed_tx_rate(void *handle, const struct wondertap_fixed_tx_rate_params *params)
{
	struct vendor_hardware_ctx *ctx = handle;
	int ret;
	if (!ctx)
		return -EINVAL;
	mutex_lock(&slsi_wondertap_op_mutex);
	if (!sdev || ctx != slsi_wondertap_active_ctx || !ctx->initialized) {
		mutex_unlock(&slsi_wondertap_op_mutex);
		return -ENODEV;
	}
	SLSI_INFO(sdev, "[Wonder]Set Fixed TX Rate - Preamble:%d, MCS:%d, NSS:%d, BW:%d, GI:%d\n",
			params->preamble, params->mcs, params->nss, params->bw, params->gi);
	ctx->tx_rate = *params;
	if((ctx->tx_rate.preamble == WONDERTAP_RATE_PREAMBLE_HT || ctx->tx_rate.preamble == WONDERTAP_RATE_PREAMBLE_VHT) && (ctx->tx_rate.gi > 1)) {
		SLSI_ERR(sdev, "[Wonder]A GI that does not match the parameters was delivered. preamble :%d, gi:%d\n", ctx->tx_rate.preamble, ctx->tx_rate.gi);
		ctx->tx_rate.gi = WONDERTAP_RATE_GI_DEFAULT;
		SLSI_INFO(sdev, "[Wonder]Changed the tx_rate.gi:%d\n", ctx->tx_rate.gi);
	}
	if((ctx->tx_rate.preamble == WONDERTAP_RATE_PREAMBLE_HE || ctx->tx_rate.preamble == WONDERTAP_RATE_PREAMBLE_EHT) && (ctx->tx_rate.gi < 2))	{
		SLSI_ERR(sdev, "[Wonder]A GI that does not match the parameters was delivered. preamble :%d, gi:%d\n", ctx->tx_rate.preamble, ctx->tx_rate.gi);
		if(ctx->tx_rate.gi == WONDERTAP_RATE_GI_DEFAULT) {
			ctx->tx_rate.gi = WONDERTAP_RATE_GI_0_8_US;
			SLSI_INFO(sdev, "[Wonder]Changed the tx_rate.gi:%d\n", ctx->tx_rate.gi);
		} else {
			SLSI_ERR(sdev, "[Wonder] Invalid tx_rate.gi : %d\n", ctx->tx_rate.gi);
			mutex_unlock(&slsi_wondertap_op_mutex);
			return -EINVAL;
		}
	}

	ret = slsi_mlme_relay_fixed_tx_rate_request(sdev, wondertap_dev, ctx);
	if (ret)
		SLSI_ERR(sdev, "[Wonder]failed to set fixed tx rate \n");
	else
		SLSI_INFO(sdev, "[Wonder]Set fixed tx rate success\n");
	mutex_unlock(&slsi_wondertap_op_mutex);
	return ret;
}
/**
 * vendor_wondertap_set_tx_rate_mask - Configures permitted TX rates.
 * @handle: The driver instance handle.
 * @params: Pointer to the rate mask configuration.
 */
static int slsi_wondertap_set_tx_rate_mask(void *handle, const struct wondertap_tx_rate_mask_params *params)
{
	struct vendor_hardware_ctx *ctx = handle;
	int ret;
	if (!ctx)
		return -EINVAL;
	mutex_lock(&slsi_wondertap_op_mutex);
	if (!sdev || ctx != slsi_wondertap_active_ctx || !ctx->initialized) {
		mutex_unlock(&slsi_wondertap_op_mutex);
		return -ENODEV;
	}
	ctx->rate_mask = *params;
	if((ctx->tx_rate.preamble == WONDERTAP_RATE_PREAMBLE_HT || ctx->tx_rate.preamble == WONDERTAP_RATE_PREAMBLE_VHT) && (ctx->tx_rate.gi > 1)) {
		SLSI_ERR(sdev, "[Wonder]A GI that does not match the parameters was delivered. preamble :%d, gi:%d\n", ctx->tx_rate.preamble, ctx->tx_rate.gi);
		ctx->tx_rate.gi = WONDERTAP_RATE_GI_DEFAULT;
		SLSI_INFO(sdev, "[Wonder]Changed the tx_rate.gi:%d\n", ctx->tx_rate.gi);
	}
	if((ctx->tx_rate.preamble == WONDERTAP_RATE_PREAMBLE_HE || ctx->tx_rate.preamble == WONDERTAP_RATE_PREAMBLE_EHT) && (ctx->tx_rate.gi < 2))	{
		SLSI_ERR(sdev, "[Wonder]A GI that does not match the parameters was delivered. preamble :%d, gi:%d\n", ctx->tx_rate.preamble, ctx->tx_rate.gi);
		if(ctx->tx_rate.gi == WONDERTAP_RATE_GI_DEFAULT) {
			ctx->tx_rate.gi = WONDERTAP_RATE_GI_0_8_US;
			SLSI_INFO(sdev, "[Wonder]Changed the tx_rate.gi:%d\n", ctx->tx_rate.gi);
		} else {
			SLSI_ERR(sdev, "[Wonder] Invalid tx_rate.gi : %d\n", ctx->tx_rate.gi);
			mutex_unlock(&slsi_wondertap_op_mutex);
			return -EINVAL;
		}
	}
#if CONFIG_SCSC_WLAN_WONDERTAP >= SLSI_WONDER_API_V1_6_4
	if (ctx->ra_enabled)
		ret = slsi_mlme_relay_tx_rate_mask_request(sdev, wondertap_dev, ctx);
	else
		ret = slsi_mlme_relay_fixed_tx_rate_request(sdev, wondertap_dev, ctx);
#else
	ret = slsi_mlme_relay_fixed_tx_rate_request(sdev, wondertap_dev, ctx);
#endif
	if (ret)
		SLSI_ERR(sdev, "[Wonder]Failed to set tx rate mask\n");
	else
		SLSI_INFO(sdev, "[Wonder]Set tx rate mask success\n");
	mutex_unlock(&slsi_wondertap_op_mutex);
	return ret;
}
/**
 * vendor_wondertap_get_capability - Retrieves capability bits.
 * @handle:       The driver instance handle.
 * @capabilities: Pointer to the struct to be filled.
 */
static int slsi_wondertap_get_capabilities(void *handle, struct wondertap_capability *features)
{
	SLSI_UNUSED_PARAMETER(handle);
	/* Deliberately lock-free: this only fills the caller's buffer with
	 * compile-time constants. It is reached from wonder_master_bind(),
	 * i.e. from inside component_add() which slsi_wondertap_register()
	 * calls; taking op_mutex there would add a
	 * slsi_start_mutex -> op_mutex edge that inverts the op path
	 * (rtnl -> op_mutex -> ... -> slsi_start_mutex).
	 */
	/* Populate the structure with our cached capabilities */
	features->bits.sta_coexist = 0;
	features->bits.sap_coexist = 0;
	features->bits.p2p_coexist = 0;
	features->bits.nan_coexist = 0;
	features->bits.ranging_coexist = 0;
	features->bits.dynamic_freq = 1;
	features->bits.dynamic_fixed_tx_rate = 1;
	features->bits.custom_mgmt_retry_limit = 0;
	features->bits.custom_data_retry_limit = 0;
	features->bits.frame_type_filter = 1;
#if CONFIG_SCSC_WLAN_WONDERTAP >= SLSI_WONDER_API_V1_6_4
	features->bits.amsdu_aggregation = 1;
	features->bits.ampdu_aggregation = 1;
	features->bits.channel_hopping = 0;
	features->bits.rate_adaptation = 1;
#else
	features->bits.amsdu_aggregation = 0;
	features->bits.ampdu_aggregation = 0;
	features->bits.rate_adaptation = 0;
#endif
	SLSI_INFO_NODEV("[Wonder]Get Capability report\n");
	return 0;
}
#if CONFIG_SCSC_WLAN_WONDERTAP >= SLSI_WONDER_API_V1_6_4
static int slsi_wondertap_channel_schedule_request(void *handle, const struct channel_schedule_request *request)
{
	struct vendor_hardware_ctx *ctx = handle;
	int ret = 0;

	if (!ctx || !request)
		return -ENODEV;

	mutex_lock(&slsi_wondertap_op_mutex);
	if (!sdev || ctx != slsi_wondertap_active_ctx || !ctx->initialized) {
		mutex_unlock(&slsi_wondertap_op_mutex);
		return -ENODEV;
	}

	SLSI_INFO(sdev,
		  "[Wonder] channel_schedule_request: len=%u,next_idx=%u, dwell_tu=%u, target_tsf=%u\n",
		  request->channel_list_len,
		  request->next_channel_index,
		  request->dwell_time_tu,
		  request->target_switch_time_tsf);

	SLSI_INFO(sdev,
		  "[Wonder] channel_list: freq=%u, wondertap_rate_bw=%d, wondertap_role=%d\n",
		  request->channel_list->freq,
		  request->channel_list->bandwidth,
		  request->channel_list->role);
	/*
	 * TODO(wonder-v1.6.4): translate request->channel_list[] into the
	 * MLME channel-hopping primitive and dispatch via slsi_mlme_*. Until
	 * then we report unsupported so the core can fall back gracefully.
	 */
	mutex_unlock(&slsi_wondertap_op_mutex);
	return ret;
}
static int slsi_wondertap_get_mac_tsf(void *handle, u32 *mac_tsf)
{
	struct vendor_hardware_ctx *ctx = handle;
	int ret = 0;

	if (!ctx || !mac_tsf)
		return -ENODEV;

	mutex_lock(&slsi_wondertap_op_mutex);
	if (!sdev || ctx != slsi_wondertap_active_ctx || !ctx->initialized) {
		mutex_unlock(&slsi_wondertap_op_mutex);
		return -ENODEV;
	}

	SLSI_INFO(sdev, "[Wonder]mac_tsf:%u\n", *mac_tsf);
	mutex_unlock(&slsi_wondertap_op_mutex);
	return ret;
}

static int slsi_wondertap_get_channel_status_report(void *handle, struct wondertap_channel_status_report *report)
{
	struct vendor_hardware_ctx *ctx = handle;
	int ret = 0;

	if (!ctx || !report)
		return -ENODEV;

	mutex_lock(&slsi_wondertap_op_mutex);
	if (!sdev || ctx != slsi_wondertap_active_ctx || !ctx->initialized) {
		mutex_unlock(&slsi_wondertap_op_mutex);
		return -ENODEV;
	}

	SLSI_INFO(sdev,"[Wonder] get_channel_status_report: current_channel_hopping_request_tsf=%u, current_channel_index=%u, channel_status_len=%u\n",
			  report->current_channel_hopping_request_tsf,
			  report->current_channel_index,
			  report->channel_status_len);

	mutex_unlock(&slsi_wondertap_op_mutex);
	return ret;
}

static int slsi_wondertap_set_station_info(void *handle, const enum wondertap_station_action action,
			struct wondertap_station_info *info)
{
	struct vendor_hardware_ctx *ctx = handle;
	struct netdev_vif *ndev_vif;
	struct slsi_peer *peer;
	struct slsi_wondertap_station_entry *entry;
	u16 peer_aid;
	int ret = 0;

	if (!ctx)
		return -EINVAL;
	if (action >= WONDERTAP_STATION_MAX)
		return -EINVAL;
	mutex_lock(&slsi_wondertap_op_mutex);

	if (!sdev || ctx != slsi_wondertap_active_ctx || !ctx->initialized) {
		mutex_unlock(&slsi_wondertap_op_mutex);
		return -ENODEV;
	}
	if (!ctx->amsdu_enabled || !ctx->ampdu_enabled || !ctx->ra_enabled) {
		SLSI_ERR(sdev, "[Wonder]Set station info is not supported: ctx->amsdu_enabled:%d, ctx->ampdu_enabled:%d, ctx->ra_enabled:%d\n",
			ctx->amsdu_enabled, ctx->ampdu_enabled, ctx->ra_enabled);
		mutex_unlock(&slsi_wondertap_op_mutex);
		return -ENODEV;
	}

	ndev_vif = netdev_priv(wondertap_dev);

	SLSI_INFO(sdev,
		  "[Wonder] set_station_info: action=%u mac=%pM aid=%u cap_mask=0x%08x\n",
		  action, info->mac, info->aid, info->capability_mask);

	switch (action) {
	case WONDERTAP_STATION_STATE_NEW:
		if(slsi_wondertap_station_find(ctx, info->mac)) {
			SLSI_WARN(sdev, "[Wonder] NEW: %pM already cached\n", info->mac);
			ret = -EEXIST;
			break;
		}

		entry = kzalloc(sizeof(*entry), GFP_KERNEL);
		if(!entry) {
			SLSI_WARN(sdev, "[Wonder] fail to alloc entry\n");
			ret = -ENOMEM;
			break;
		}

		SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
		slsi_spinlock_lock(&ndev_vif->peer_lock);
		/* Wonder's aid plays no part in slot selection: the internal aid is
		 * allocated from the free queuesets, so the only failure here is a
		 * genuinely full peer record table. entry->info keeps wonder's aid
		 * for QUERY/UPDATE. See slsi_wondertap_alloc_peer_aid.
		 */
		peer_aid = slsi_wondertap_alloc_peer_aid(wondertap_dev);
		if (!peer_aid) {
			slsi_spinlock_unlock(&ndev_vif->peer_lock);
			SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
			SLSI_ERR(sdev, "[Wonder] NEW: all %u peer records in use, cannot add %pM\n",
				 SLSI_PEER_INDEX_MAX, info->mac);
			kfree(entry);
			ret = -ENOSPC;
			break;
		}
		peer = slsi_peer_add(sdev, wondertap_dev, info->mac, peer_aid);
		if (peer)
			peer->connected_state = SLSI_STA_CONN_STATE_CONNECTED;
		slsi_spinlock_unlock(&ndev_vif->peer_lock);
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		if (!peer) {
			SLSI_ERR(sdev, "[Wonder] slsi_peer_add(%pM, wonder_aid=%u peer_aid=%u) failed\n",
				 info->mac, info->aid, peer_aid);
			kfree(entry);
			ret = -ENOMEM;
			break;
		}

		entry->info = *info;
		entry->peer_aid = peer_aid;
		SLSI_INFO(sdev, "[Wonder] NEW: %pM wonder_aid=%u -> peer_aid=%u (qs=%u)\n",
			  info->mac, info->aid, peer_aid, MAP_AID_TO_QS(peer_aid));
		list_add_tail(&entry->node, &ctx->station_list);
		ctx->sta_info = *info;
		ctx->action = action;
		ctx->station_info_len = 0;
		if (ctx->sta_info.capability_mask & BIT(WONDERTAP_STATION_CAP_HT))
			ctx->station_info_len += sizeof(ctx->sta_info.ht_capa);
		if (ctx->sta_info.capability_mask & BIT(WONDERTAP_STATION_CAP_VHT))
			ctx->station_info_len += sizeof(ctx->sta_info.vht_capa);
		if (ctx->sta_info.capability_mask & BIT(WONDERTAP_STATION_CAP_HE))
			ctx->station_info_len += sizeof(ctx->sta_info.he_capa);
		if (ctx->sta_info.capability_mask & BIT(WONDERTAP_STATION_CAP_HE_6G))
			ctx->station_info_len += sizeof(ctx->sta_info.he_6ghz_capa);

		ret = slsi_mlme_relay_set_peer_info_request(sdev, wondertap_dev, ctx);
		if (ret) {
			SLSI_WARN(sdev, "[Wonder] mlme_relay_set_peer fail:%d\n", ret);
			/* FW refused the peer: undo the driver side too. Leaving the
			 * slsi_peer behind keeps its queueset occupied forever, so a
			 * retry for the same MAC/aid would be rejected by
			 * slsi_peer_add() with "already exists".
			 */
			SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
			slsi_spinlock_lock(&ndev_vif->peer_lock);
			peer = slsi_get_peer_from_mac(sdev, wondertap_dev, info->mac);
			if (peer)
				slsi_peer_remove(sdev, wondertap_dev, peer);
			slsi_spinlock_unlock(&ndev_vif->peer_lock);
			SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
			list_del(&entry->node);
			kfree(entry);
		} else {
			SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
			slsi_spinlock_lock(&ndev_vif->peer_lock);
			peer = slsi_get_peer_from_mac(sdev, wondertap_dev, info->mac);
			if (peer)
				peer->flow_id = ctx->flow_id;
			slsi_spinlock_unlock(&ndev_vif->peer_lock);
			SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		}
		break;

	case WONDERTAP_STATION_STATE_UPDATE:
		entry = slsi_wondertap_station_find(ctx, info->mac);
		if(!entry) {
			SLSI_WARN(sdev, "[Wonder] UPDATE: %pM not cached\n", info->mac);
			ret = -ENOENT;
			break;
		}

		/* Wonder's aid is decoupled from the internal aid, so an aid change
		 * needs no peer remove/re-add: the peer keeps its queueset and there
		 * is no window where TX has no peer to enqueue against. Only the
		 * cached metadata has to be refreshed.
		 */
		if (entry->info.aid != info->aid)
			SLSI_INFO(sdev, "[Wonder] UPDATE: %pM wonder_aid %u -> %u (peer_aid=%u unchanged)\n",
				  info->mac, entry->info.aid, info->aid, entry->peer_aid);

		entry->info = *info;
		ctx->sta_info = *info;
		ctx->action = action;
		ctx->station_info_len = 0;
		if (ctx->sta_info.capability_mask & BIT(WONDERTAP_STATION_CAP_HT))
			ctx->station_info_len += sizeof(ctx->sta_info.ht_capa);
		if (ctx->sta_info.capability_mask & BIT(WONDERTAP_STATION_CAP_VHT))
			ctx->station_info_len += sizeof(ctx->sta_info.vht_capa);
		if (ctx->sta_info.capability_mask & BIT(WONDERTAP_STATION_CAP_HE))
			ctx->station_info_len += sizeof(ctx->sta_info.he_capa);
		if (ctx->sta_info.capability_mask & BIT(WONDERTAP_STATION_CAP_HE_6G))
			ctx->station_info_len += sizeof(ctx->sta_info.he_6ghz_capa);

		ret = slsi_mlme_relay_set_peer_info_request(sdev, wondertap_dev, ctx);
		/* Re-look-up under the peer locks: the request slept, so the
		 * pointer captured before it may since have been recycled by a
		 * concurrent ndo_stop -> slsi_vif_cleanup.
		 */
		SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
		slsi_spinlock_lock(&ndev_vif->peer_lock);
		peer = slsi_get_peer_from_mac(sdev, wondertap_dev, info->mac);
		if (peer)
			peer->flow_id = ctx->flow_id;
		slsi_spinlock_unlock(&ndev_vif->peer_lock);
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		break;

	case WONDERTAP_STATION_STATE_DEL:
		entry = slsi_wondertap_station_find(ctx, info->mac);
		if(!entry) {
			SLSI_WARN(sdev, "[Wonder] DEL: %pM not cached\n", info->mac);
			ret = -ENOENT;
			break;
		}

		SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
		slsi_spinlock_lock(&ndev_vif->peer_lock);
		peer = slsi_get_peer_from_mac(sdev, wondertap_dev, info->mac);

		if (peer)
			ret = slsi_peer_remove(sdev, wondertap_dev, peer);
		else
			SLSI_WARN(sdev, "[Wonder] DEL: cached but no peer for %pM\n", info->mac);

		slsi_spinlock_unlock(&ndev_vif->peer_lock);
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

		ctx->sta_info = *info;
		ctx->action = action;
		ret = slsi_mlme_relay_set_peer_info_request(sdev, wondertap_dev, ctx);

		list_del(&entry->node);
		kfree(entry);
		break;

	case WONDERTAP_STATION_STATE_QUERY:
		entry = slsi_wondertap_station_find(ctx, info->mac);
		if (!entry) {
			ret = -ENOENT;
			break;
		}
		*info = entry->info;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&slsi_wondertap_op_mutex);
	return ret;
}
#endif
/* ============================================================================
 * 3. Ops Table Registration
 * ============================================================================
 * This structure is exposed via component mechanism and used by the Core.
 */
static const struct wondertap_ops vendor_ops = {
	.init = slsi_wondertap_init,
	.deinit = slsi_wondertap_deinit,
	.set_freq = slsi_wondertap_set_freq,
	.set_filter = slsi_wondertap_set_filter,
	.set_fixed_tx_rate = slsi_wondertap_set_fixed_tx_rate,
	.set_tx_rate_mask = slsi_wondertap_set_tx_rate_mask,
	.get_capabilities = slsi_wondertap_get_capabilities,
#if CONFIG_SCSC_WLAN_WONDERTAP >= SLSI_WONDER_API_V1_6_4
	.channel_schedule_request = slsi_wondertap_channel_schedule_request,
	.get_mac_tsf = slsi_wondertap_get_mac_tsf,
	.get_channel_status_report = slsi_wondertap_get_channel_status_report,
	.set_station_info = slsi_wondertap_set_station_info,
#endif
};
/* ============================================================================
 * 4. Component Framework Binding
 * ============================================================================
 */
static int slsi_wondertap_component_bind(struct device *dev, struct device *master, void *data)
{
	SLSI_INFO(sdev, "[Wonder]Successfully bound to master '%s'\n", dev_name(master));
	return 0;
}
static void slsi_wondertap_component_unbind(struct device *dev, struct device *master, void *data)
{
	/*
	 * Runs synchronously inside component_del()/wonder_master_unbind().
	 * Take the op mutex to (a) drain any in-flight wondertap_ops calls
	 * still holding it on another CPU, and (b) safely free a ctx that
	 * userspace never deinit'd before WLAN service teardown.
	 *
	 * rtnl must be taken here, and it must be taken BEFORE op_mutex:
	 * slsi_wondertap_deinit_ctx() unregisters wondertap0, and every other
	 * entry into op_mutex (mac80211 .start/.stop, nl80211 vendor commands)
	 * already holds rtnl. Acquiring rtnl underneath op_mutex would be an
	 * AB-BA inversion against those paths.
	 *
	 * component_del() is never called with rtnl held (slsi_wondertap_
	 * unregister() from cm_if.c, slsi_wondertap_pdev_remove() from module
	 * exit, wonder's component_master_del()), so taking it here is safe.
	 */
	rtnl_lock();
	mutex_lock(&slsi_wondertap_op_mutex);
	if (slsi_wondertap_active_ctx) {
		SLSI_WARN_NODEV("[Wonder] freeing leaked vendor_handle on unbind\n");
		slsi_wondertap_deinit_ctx(slsi_wondertap_active_ctx);
	}
	mutex_unlock(&slsi_wondertap_op_mutex);
	/* op_mutex is dropped before rtnl_unlock() on purpose.
	 *
	 * rtnl_unlock() runs netdev_run_todo(), which does __rtnl_unlock()
	 * FIRST and only then waits in netdev_wait_allrefs() for the last
	 * reference on wondertap0 - one of which is wonder's own
	 * dev_get_by_name(). (netdev_wait_allrefs() takes rtnl_lock() itself
	 * every second to rebroadcast NETDEV_UNREGISTER, so it cannot be
	 * entered with rtnl held.) The wait therefore does not block wonder
	 * from taking rtnl, and it terminates: unregister_netdevice() ->
	 * dev_close_many() emits NETDEV_DOWN synchronously above, wonder's
	 * netdev notifier schedules its pdev_down_work, and that runs
	 * ieee80211_unregister_hw() -> its .stop -> dev_put().
	 *
	 * What we must not do is hold op_mutex across that wait: wonder can
	 * re-enter our ops during it, because it keeps a private copy of the
	 * ops table in wondertap->wonder_ops that wonder_master_unbind() does
	 * not clear.
	 */
	rtnl_unlock();
	SLSI_INFO_NODEV("[Wonder]Unbound from master\n");
}
static const struct component_ops vendor_component_ops = {
	.bind = slsi_wondertap_component_bind,
	.unbind = slsi_wondertap_component_unbind,
};
/* ============================================================================
 * 5. Platform Driver Implementation
 * ============================================================================
 *
 * The platform_driver below is bound to the DTSI node with
 * compatible = "samsung,scsc-wondertap-driver". On probe it allocates the
 * wondertap_priv (carrying the version and ops table) and publishes it via
 * platform_set_drvdata(); the wonder driver retrieves it through
 * of_find_device_by_node() + platform_get_drvdata().
 *
 * The component_add() that signals the wonder-side master to bind happens
 * later, inside slsi_wondertap_register(sdev), which is called when the WLAN
 * service comes up. component_del() in slsi_wondertap_unregister() tears
 * the binding down when the service goes away.
 */
static int slsi_wondertap_pdev_probe(struct platform_device *pdev)
{
	struct wondertap_priv *priv;
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->ver = WONDER_VERSION_1_6_4;
	priv->wonder_ops = &vendor_ops;
	platform_set_drvdata(pdev, priv);
	slsi_wondertap_pdev = pdev;
	dev_info(&pdev->dev, "WonderTap provider probed (ver %d)\n", priv->ver);
	return 0;
}
static void slsi_wondertap_pdev_remove(struct platform_device *pdev)
{
	/*
	 * Safety net: if WLAN service teardown didn't run before module
	 * unload, the wonder master may still hold our vendor_ops pointer.
	 * Force component_del here so wonder unbinds before our .text vanishes.
	 */
	if (atomic_cmpxchg(&slsi_wondertap_component_added, 1, 0) == 1)
		component_del(&pdev->dev, &vendor_component_ops);
	platform_set_drvdata(pdev, NULL);
	slsi_wondertap_pdev = NULL;
}
/*
 * platform_driver::remove() ABI shim
 * ----------------------------------
 * Starting with kernel 6.11, the return type of the platform_driver
 * .remove callback changed:
 *   int  (*remove)(struct platform_device *)   // pre-6.11
 *     ↓
 *   void (*remove)(struct platform_device *)   // 6.11+
 * (commit 0edb555a65d1).
 *
 * This driver implements the core function (slsi_wondertap_pdev_remove)
 * using the new (void) signature, but uses the following shim structure
 * to remain build-compatible with older kernels:
 *
 *   - 6.11+: Macros are inactive. The .remove = slsi_wondertap_pdev_remove
 *     in the platform_driver struct below directly binds to the void function.
 *   - Pre-6.11: An int-returning wrapper (_wrapper) is additionally defined,
 *     and token pasting redirects the .remove registration to the wrapper.
 *     The wrapper calls the core function and returns 0, satisfying the old signature.
 *
 * Important macro placement: These macros must appear after the core function
 * definition (slsi_wondertap_pdev_remove) but before the .remove field
 * initialization (slsi_wondertap_pdrv). Otherwise the core function itself
 * gets replaced by the wrapper, causing self-recursion.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int slsi_wondertap_pdev_remove_wrapper(struct platform_device *pdev)
{
	slsi_wondertap_pdev_remove(pdev);
	return 0;
}
#define slsi_wondertap_pdev_remove slsi_wondertap_pdev_remove_wrapper
#endif
static const struct of_device_id slsi_wondertap_of_match[] = {
	{ .compatible = "samsung,scsc-wondertap-driver" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, slsi_wondertap_of_match);
static struct platform_driver slsi_wondertap_pdrv = {
	.probe = slsi_wondertap_pdev_probe,
	.remove = slsi_wondertap_pdev_remove,
	.driver = {
		.name = "scsc-wondertap",
		.of_match_table = slsi_wondertap_of_match,
	},
};
int slsi_wondertap_pdrv_register(void)
{
	return platform_driver_register(&slsi_wondertap_pdrv);
}
void slsi_wondertap_pdrv_unregister(void)
{
	platform_driver_unregister(&slsi_wondertap_pdrv);
}
int slsi_wondertap_register(struct slsi_dev *sdev_param)
{
	struct wondertap_priv *priv;
	int ret;
	if (!sdev_param)
		return -EINVAL;
	if (!slsi_wondertap_pdev) {
		SLSI_ERR(sdev_param,
			 "[Wonder] platform device not bound; check DT for 'samsung,scsc-wondertap-driver' node\n");
		return -ENODEV;
	}
	mutex_lock(&slsi_wondertap_op_mutex);
	sdev = sdev_param;
	mutex_unlock(&slsi_wondertap_op_mutex);
	priv = platform_get_drvdata(slsi_wondertap_pdev);
	if (!priv) {
		SLSI_ERR(sdev, "[Wonder] wondertap_priv missing on pdev\n");
		ret = -ENODEV;
		goto err;
	}
	sdev->wondertap_priv = priv;
	ret = component_add(&slsi_wondertap_pdev->dev, &vendor_component_ops);
	if (ret) {
		SLSI_ERR(sdev, "Failed to add wondertap component: %d\n", ret);
		sdev->wondertap_priv = NULL;
		goto err;
	}
	atomic_set(&slsi_wondertap_component_added, 1);
	SLSI_INFO(sdev, "WonderTap provider registered (ver %d, of_node=%pOF)\n",
		  priv->ver, slsi_wondertap_pdev->dev.of_node);
	return 0;
err:
	mutex_lock(&slsi_wondertap_op_mutex);
	sdev = NULL;
	wondertap_dev = NULL;
	mutex_unlock(&slsi_wondertap_op_mutex);
	return ret;
}
void slsi_wondertap_unregister(struct slsi_dev *sdev_param)
{
	struct platform_device *pdev;

	if (!sdev_param)
		return;
	/*
	 * component_del() synchronously triggers wonder_master_unbind, which
	 * (a) clears wondertap_ops on the wonder side so no NEW dispatches
	 *     can reach our vendor_ops, and
	 * (b) calls our slsi_wondertap_component_unbind, which takes
	 *     slsi_wondertap_op_mutex -- this drains any in-flight ops
	 *     before component_del() returns.
	 */
	pdev = slsi_wondertap_pdev;
	if (pdev && atomic_cmpxchg(&slsi_wondertap_component_added, 1, 0) == 1)
		component_del(&pdev->dev, &vendor_component_ops);
	sdev_param->wondertap_priv = NULL;
	/*
	 * Re-take the mutex to publish sdev=NULL atomically w.r.t. ops.
	 * After component_del returns, no new ops can start (ops table is
	 * NULL on wonder side), so this is mostly a memory-ordering fence,
	 * but it costs nothing and keeps the invariant explicit.
	 */
	mutex_lock(&slsi_wondertap_op_mutex);
	sdev = NULL;
	/* Drop the netdev pointer with sdev. slsi_dev_detach() ->
	 * slsi_netif_remove_all() frees wondertap0 without going through us, so
	 * keeping it here would leave a dangling pointer for the next init().
	 */
	wondertap_dev = NULL;
	mutex_unlock(&slsi_wondertap_op_mutex);
	SLSI_INFO_NODEV("WonderTap provider unregistered\n");
}

