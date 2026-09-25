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

#ifndef IS_CIS_GC12A2_H
#define IS_CIS_GC12A2_H

#include "is-cis.h"

enum sensor_gc12a2_mode_enum {
	SENSOR_GC12A2_4000x3000_30FPS_R10 = 0,
	SENSOR_GC12A2_4000x3000_30FPS_R12,
	SENSOR_GC12A2_4000x2252_30FPS_R10,
	SENSOR_GC12A2_4000x2252_30FPS_R12,
	SENSOR_GC12A2_3472x2388_30FPS_R10,
	SENSOR_GC12A2_3184x2388_30FPS_R10,
	SENSOR_GC12A2_2000x1128_30FPS_R10,
	SENSOR_GC12A2_1960x1472_60FPS_R10,
	SENSOR_GC12A2_2000x1496_59FPS_R10,
	SENSOR_GC12A2_MODE_MAX,
};

struct sensor_gc12a2_private_data {
	const struct sensor_regs global;
};

static const struct sensor_reg_addr sensor_gc12a2_reg_addr = {
	.fll = 0x0340,
	.cit = 0x0202,
	.again = 0x0205,
	.dgain = 0x0276,
};
#endif
