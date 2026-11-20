/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2026 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef IS_CIS_IMX355_H
#define IS_CIS_IMX355_H

#include "is-cis.h"

#define SENSOR_IMX355_MAX_CIT_LSHIFT_VALUE                     (0x7)

enum sensor_imx355_mode_enum {
	SENSOR_IMX355_MODE_3264x2448_30FPS = 0,
	SENSOR_IMX355_MODE_3264x1836_30FPS,
	SENSOR_IMX355_MODE_800x600_120FPS,
	SENSOR_IMX355_MODE_1632x1224_30FPS,
	// SENSOR_IMX355_MODE_3472x2388_30FPS,
	SENSOR_IMX355_MODE_MAX,
};

struct sensor_imx355_private_data {
	const struct sensor_regs global;
};

#define MODE_GROUP_NONE (-1)
enum sensor_imx355_mode_group_enum {
	SENSOR_IMX355_MODE_NORMAL,
	SENSOR_IMX355_MODE_LN4,
	SENSOR_IMX355_MODE_DSG,
	SENSOR_IMX355_MODE_MODE_GROUP_MAX
};
u32 sensor_imx355_mode_groups[SENSOR_IMX355_MODE_MODE_GROUP_MAX];

static const struct sensor_reg_addr sensor_imx355_reg_addr = {
	.fll = 0x0340,
	.fll_shifter = 0,
	.cit = 0x0202,
	.cit_shifter = 0, /* not supported */
	.again = 0x0204,
	.dgain = 0x020E,
	.group_param_hold = 0x0104,
};
#endif
