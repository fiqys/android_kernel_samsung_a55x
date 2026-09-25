// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kprobes.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/sched.h>
#include "core.h"
#include "gadget.h"
#include "dwc3-exynos.h"
#include "exynos_usb_tpmon.h"

#include <linux/platform_device.h>
#include "../host/xhci.h"
#include "../../../sound/usb/exynos_usb_audio.h"

#if IS_ENABLED(CONFIG_SOC_S5E8845)
extern struct dwc3_exynos *g_dwc3_exynos;
#endif

#if IS_ENABLED(CONFIG_SND_EXYNOS_USB_AUDIO_MODULE)
extern struct hcd_hw_info *g_hwinfo;
#endif

struct kprobe_data {
	void *x0;
	void *x1;
	void *x2;
};

static int entry_dwc3_gadget_ep_queue(struct kretprobe_instance *ri,
				   struct pt_regs *regs)
{
	// pr_info("%s+++\n", __func__);

#if IS_ENABLED(CONFIG_USB_EXYNOS_TPMON_MODULE)
	// mainline code - func. format
	// gadget.c:1995:static int dwc3_gadget_ep_queue(struct usb_ep *ep,	=> regs[0]
	// 				struct usb_request *request,		=> regs[1]
	//				gfp_t gfp_flags)	 		=> regs[2]
	struct usb_request *request = (struct usb_request *)regs->regs[1];
	struct dwc3_request *req = to_dwc3_request(request);
	int *dummy_data = NULL;


	// pr_info("[TP] check TP for u1/u2 onoff at %s function (w/ kretprobe)\n", __func__);

	// func. format: void usb_tpmon_check_tp(void *data, struct dwc3_request *req)
	usb_tpmon_check_tp(dummy_data, req);
#endif

	return 0;
}

static int exit_dwc3_gadget_ep_queue(struct kretprobe_instance *ri,
				   struct pt_regs *regs)
{
	// pr_info("%s---\n", __func__);

	return 0;
}

static int entry_dwc3_gadget_pullup(struct kretprobe_instance *ri,
				   struct pt_regs *regs)
{
	struct usb_gadget *g = (struct usb_gadget *)regs->regs[0];
	struct dwc3	*dwc = gadget_to_dwc(g);
	int is_on = (int)regs->regs[1];

	pr_info("%s on=%d, pullups_connected=%d\n", __func__, is_on, dwc->pullups_connected);

#if IS_ENABLED(CONFIG_SOC_S5E8845)
	/*
	 * Avoid unnecessary DWC3 core wake-ups If the Exynos configuration
	 * is not completed.
	 */
	if (is_on && g_dwc3_exynos && !g_dwc3_exynos->vbus_state) {
		pr_info("%s exynos setup is not done: vbus_state =%d!!\n",
				__func__, g_dwc3_exynos->vbus_state);

			regs->regs[1] = 0;
			g_dwc3_exynos->force_pullup = true;
	}
#endif	/* CONFIG_SOC_S5E8845 */

	return 0;
}

static int exit_dwc3_gadget_pullup(struct kretprobe_instance *ri,
				   struct pt_regs *regs)
{
	int ret = (int)regs->regs[0];

	pr_info("%s--- ret = %d\n", __func__, ret);

	return 0;
}

#if IS_ENABLED(CONFIG_SOC_S5E8845)
static int entry_usb_gadget_connect_locked(struct kretprobe_instance *ri,
					struct pt_regs *regs)
{
	g_dwc3_exynos->pullup_state = 1;
	pr_info("%s enter !!\n", __func__);

	return 0;
}

static int exit_usb_gadget_connect_locked(struct kretprobe_instance *ri,
					struct pt_regs *regs)
{
	g_dwc3_exynos->pullup_state = 0;
	pr_info("%s exit !!\n", __func__);

	return 0;
}
#endif

#define ENTRY_EXIT(name) {\
	.handler = exit_##name,\
	.entry_handler = entry_##name,\
	.data_size = sizeof(struct kprobe_data),\
	.maxactive = 8,\
	.kp.symbol_name = #name,\
}

#define ENTRY(name) {\
	.entry_handler = entry_##name,\
	.data_size = sizeof(struct kprobe_data),\
	.maxactive = 8,\
	.kp.symbol_name = #name,\
}

static struct kretprobe dwc3_kret_probes[] = {
	ENTRY_EXIT(dwc3_gadget_ep_queue),
	ENTRY_EXIT(dwc3_gadget_pullup),
#if IS_ENABLED(CONFIG_SOC_S5E8845)
	ENTRY_EXIT(usb_gadget_connect_locked),
#endif
};

int dwc3_kretprobe_init(void)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(dwc3_kret_probes); i++) {
		ret = register_kretprobe(&dwc3_kret_probes[i]);
		if (ret < 0) {
			pr_err("register_kretprobe failed, returned %d\n", ret);
			return ret;
		}
	}

	return 0;
}

void dwc3_kretprobe_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dwc3_kret_probes); i++)
		unregister_kretprobe(&dwc3_kret_probes[i]);
}

MODULE_SOFTDEP("pre:dwc3-exynos-usb");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 EXYNOS Glue Layer function handler");
