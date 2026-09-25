/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2024 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef IS_CIS_GC02M1_H
#define IS_CIS_GC02M1_H

#include "is-cis.h"

#define MULTIPLE_OF_4(val) ((val >> 2) << 2)
#define USE_OTP_AWB_CAL_DATA            (0)
#define MAX_GAIN_INDEX    15
#define CODE_GAIN_INDEX    0
#define PERMILE_GAIN_INDEX 1
#define POLL_TIME_MS (1)
#define STREAM_OFF_POLL_TIME_MS (500)
#define STREAM_ON_POLL_TIME_MS (500)

enum sensor_gc02m1_mode_enum {
	SENSOR_GC02M1_1600x1200_30FPS = 0,
	SENSOR_GC02M1_1600x900_30FPS,
	SENSOR_GC02M1_800x600_60FPS,
	SENSOR_GC02M1_MODE_MAX,
};

struct sensor_gc02m1_private_data {
	const struct sensor_regs global;
	const struct sensor_regs fsync_master;
	const struct sensor_regs fsync_slave;
};

static const struct sensor_reg_addr sensor_gc02m1_reg_addr = {
	.fll = 0x41,
	.fll_shifter = 0, /* not supported */
	.cit_shifter = 0, /* not supported */
	.again = 0xb6,
	.dgain = 0xb1,
};

const u32 sensor_gc02m1_analog_gain[][MAX_GAIN_INDEX + 1] = {
	{0, 1000},
	{1, 1500},
	{2, 1987},
	{3, 2459},
	{4, 3090},
	{5, 3541},
	{6, 4050},
	{7, 4485},
	{8, 4975},
	{9, 5563},
	{10, 6123},
	{11, 6556},
	{12, 7041},
	{13, 7505},
	{14, 8021},
	{15, 10095},
};

const u32 sensor_gc02m1_otp_read[] = {
	0xfc, 0x01, 0x01,
	0xf4, 0x41, 0x01,
	0xf5, 0xc0, 0x01,
	0xf6, 0x44, 0x01,
	0xf8, 0x38, 0x01,
	0xf9, 0x82, 0x01,
	0xfa, 0x00, 0x01,
	0xfd, 0x80, 0x01,
	0xfc, 0x81, 0x01,
	0xfe, 0x03, 0x01,
	0x01, 0x0b, 0x01,
	0xf7, 0x01, 0x01,
	0xfc, 0x80, 0x01,
	0xfc, 0x80, 0x01,
	0xfc, 0x80, 0x01,
	0xfc, 0x8e, 0x01,
	0xfe, 0x00, 0x01,
	0x87, 0x09, 0x01,
	0xee, 0x72, 0x01,
	0xfe, 0x01, 0x01,
	0x8c, 0x90, 0x01,
	0xF3, 0x30, 0x01,
	0xFE, 0x02, 0x01,
};

#endif //IS_CIS_GC02M1_H
