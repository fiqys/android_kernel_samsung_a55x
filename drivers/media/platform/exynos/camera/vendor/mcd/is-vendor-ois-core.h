/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung Exynos SoC series Pablo driver
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef IS_VENDOR_OIS_CORE_H
#define IS_VENDOR_OIS_CORE_H
#if defined(CONFIG_CAMERA_USE_INTERNAL_MCU)
#include "is-vendor-ois-internal-mcu.h"
#endif

#define	GYRO_CAL_VALUE_FROM_EFS	"/efs/FactoryApp/camera_ois_gyro_cal"
#define IS_OIS_SDCARD_PATH		"/data/vendor/camera/"
#define	MAX_GYRO_EFS_DATA_LENGTH	30

#define MCU_OIS_GYRO_DIRECTION_MAX	8

enum is_ois_power_mode {
	OIS_POWER_MODE_NONE = 0,
	OIS_POWER_MODE_SINGLE_WIDE,
	OIS_POWER_MODE_SINGLE_TELE,
	OIS_POWER_MODE_SINGLE_TELE2,
	OIS_POWER_MODE_DUAL,
	OIS_POWER_MODE_TRIPLE,
};

enum ois_mcu_state {
	OM_HW_NONE,
	OM_HW_FW_LOADED,
	OM_HW_RUN,
	OM_HW_SUSPENDED,
	OM_HW_END
};

enum ois_mcu_base_reg_index {
	OM_REG_CORE = 0,
	OM_REG_PERI1 = 1,
	OM_REG_PERI2 = 2,
	OM_REG_PERI_SETTING = 3,
	OM_REG_SFR = 4,
	OM_REG_MAX
};

struct ois_mcu_dev {
#if defined(CONFIG_CAMERA_USE_INTERNAL_MCU)
	struct platform_device	*pdev;
	struct device		*dev;
	struct clk		*clk;
	struct clk		*spi_clk;
	struct mutex 	power_mutex;
	int			irq;
	resource_size_t		regs_start[OM_REG_MAX];
	resource_size_t		regs_end[OM_REG_MAX];
	atomic_t 		shared_rsc_count;
	int			current_rsc_count;
	bool			dev_ctrl_state;
	struct work_struct	mcu_power_on_work;
#endif
	void __iomem		*regs[OM_REG_MAX];
	unsigned long		state;
	int			current_power_mode;
	u16			current_error_reg;
	bool			need_reset_mcu;
	bool			need_af_delay;
	bool			is_mcu_active;

	bool			ois_wide_init;
	bool			ois_tele_init;
	bool			ois_tele2_init;
	bool			ois_hw_check;
	bool			ois_fadeupdown;
	int			ois_gyro_direction[MCU_OIS_GYRO_DIRECTION_MAX];
#if defined(CONFIG_CAMERA_USE_EXTERNAL_MCU) || defined(CONFIG_CAMERA_USE_AOIS)
	u32			aperture_delay_list[2];
	u32			aperture_delay_list_len;
#endif
};

#ifdef CONFIG_CAMERA_USE_AOIS
#define is_vendor_ois_set_aois_fac_mode_on(void) { cam_ois_set_aois_fac_mode_on(); }
#define is_vendor_ois_set_aois_fac_mode_off(void) { cam_ois_set_aois_fac_mode_off(); }
#else
#define is_vendor_ois_set_aois_fac_mode_on(void) { }
#define is_vendor_ois_set_aois_fac_mode_off(void) { }
#endif

/*
 * log
 */
#define err_mcu(fmt, args...) \
	pr_err("[@][OIS_MCU]%s:%d:" fmt "\n", __func__, __LINE__, ##args)

#define warn_mcu(fmt, args...) \
	pr_warn("[@][OIS_MCU]%s:%d:" fmt "\n", __func__, __LINE__, ##args)

#define info_mcu(fmt, args...) \
	pr_info("[@][OIS_MCU]" fmt, ##args)

#define dbg_mcu(fmt, args...) \
	pr_debug("[@][OIS_MCU]" fmt, ##args)

#define MCU_ERR_PRINT(fmt, args...) \
	pr_err("[@][OIS_MCU]%s:%d:" fmt "\n", __func__, __LINE__, ##args)

#define MCU_GET_ERR_PRINT(idx) \
	MCU_ERR_PRINT("%s: get fail (%s:%X)", __func__, ois_mcu_regs[idx].reg_name, ois_mcu_regs[idx].sfr_offset)
#define MCU_SET_ERR_PRINT(idx) \
	MCU_ERR_PRINT("%s: set fail (%s:%X)", __func__, ois_mcu_regs[idx].reg_name, ois_mcu_regs[idx].sfr_offset)
#endif
