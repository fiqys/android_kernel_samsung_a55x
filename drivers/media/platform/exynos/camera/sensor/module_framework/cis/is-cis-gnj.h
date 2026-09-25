/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2025 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef IS_CIS_GNJ_H
#define IS_CIS_GNJ_H

#include "is-cis.h"

/* Related EEPROM CAL */
#define CIS_CALIBRATION 1
#if IS_ENABLED(CIS_CALIBRATION)
#define GNJ_XTC_ADDR				(0x21D0)
#define GNJ_BURST_WRITE

enum gnj_endian {
	GNJ_LITTLE_ENDIAN = 0,
	GNJ_BIG_ENDIAN = 1,
};
#define GNJ_ENDIAN(a, b, endian)  ((endian == GNJ_BIG_ENDIAN) ? ((a << 8)|(b)) : ((a)|(b << 8)))
#endif

#define GNJ_REMOSAIC_ZOOM_RATIO_X_2	20	/* x2.0 = 20 */

enum sensor_mode_enum {
	SENSOR_GNJ_MODE_4080x3060_60FPS_R12,
	SENSOR_GNJ_MODE_4080x2296_60FPS_R12,
	SENSOR_GNJ_MODE_2040x1532_30FPS_R10,
	SENSOR_GNJ_MODE_2040x1532_120FPS_R10,
	SENSOR_GNJ_MODE_2040x1148_243FPS_R10,
	SENSOR_GNJ_MODE_REMOSAIC_8160x6120_30FPS_R10,
	SENSOR_GNJ_MODE_REMOSAIC_8160x4592_30FPS_R10,
	SENSOR_GNJ_MODE_4080x3060_30FPS_IDCG_R12,
	SENSOR_GNJ_MODE_4080x2296_30FPS_IDCG_R12,
	SENSOR_GNJ_MODE_4080x3060_30FPS_REMOSAIC_CROP_R12,
	SENSOR_GNJ_MODE_4080x2296_30FPS_REMOSAIC_CROP_R12,
	SENSOR_GNJ_MODE_4080x3060_30FPS_LN4_R12,
	SENSOR_GNJ_MODE_4080x2296_30FPS_LN4_R12,
	SENSOR_GNJ_MODE_MAX
};

#define MODE_GROUP_NONE (-1)
enum sensor_gnj_mode_group_enum {
	SENSOR_GNJ_MODE_NORMAL,
	SENSOR_GNJ_MODE_RMS_CROP,
	SENSOR_GNJ_MODE_LN4,
	SENSOR_GNJ_MODE_IDCG,
	SENSOR_GNJ_MODE_MODE_GROUP_MAX
};
u32 sensor_gnj_mode_groups[SENSOR_GNJ_MODE_MODE_GROUP_MAX];

struct sensor_gnj_private_data {
	const struct sensor_regs global;
	const struct sensor_regs xtc_prefix;
	const struct sensor_regs load_sram;
};

const u32 sensor_gnj_rms_binning_ratio[SENSOR_GNJ_MODE_MAX] = {
	[SENSOR_GNJ_MODE_4080x3060_30FPS_REMOSAIC_CROP_R12] = 1000,
	[SENSOR_GNJ_MODE_4080x2296_30FPS_REMOSAIC_CROP_R12] = 1000,
};

static const struct sensor_reg_addr sensor_gnj_reg_addr = {
	.fll = 0x0340,
	.fll_shifter = 0x0702,
	.cit = 0x0202,
	.cit_shifter = 0x0704,
	.again = 0x0204,
	.dgain = 0x020E,
	.dgain_secondary = 0x0230,
	.group_param_hold = 0x0104,
};

#endif
