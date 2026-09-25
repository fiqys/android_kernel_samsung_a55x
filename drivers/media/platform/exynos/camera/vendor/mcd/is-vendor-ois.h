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

#ifndef IS_VENDOR_OIS_H
#define IS_VENDOR_OIS_H
#include <linux/i2c.h>
#include "is-device-sensor-peri.h"

void is_vendor_ois_parsing_raw_data(uint8_t *buf, long efs_size, long *raw_data_x, long *raw_data_y, long *raw_data_z);
int is_vendor_ois_init(struct v4l2_subdev *subdev);
int is_vendor_ois_init_factory(struct v4l2_subdev *subdev);
#if defined(CAMERA_3RD_OIS)
void is_vendor_ois_init_rear2(struct is_core *core);
#endif /* CAMERA_3RD_OIS */
int is_vendor_ois_deinit(struct v4l2_subdev *subdev);
int is_vendor_ois_set_mode(struct v4l2_subdev *subdev, int mode);
int is_vendor_ois_shift_compensation(struct v4l2_subdev *subdev, int position, int resolution);
int is_vendor_ois_self_test(struct is_core *core);
bool is_vendor_ois_auto_test_all(struct is_core *core,
					int threshold, bool *x_result, bool *y_result, int *sin_x, int *sin_y,
					bool *x_result_2nd, bool *y_result_2nd, int *sin_x_2nd, int *sin_y_2nd,
					bool *x_result_3rd, bool *y_result_3rd, int *sin_x_3rd, int *sin_y_3rd);
#if defined(CAMERA_2ND_OIS)
bool is_vendor_ois_auto_test_rear2(struct is_core *core,
					int threshold, bool *x_result, bool *y_result, int *sin_x, int *sin_y,
					bool *x_result_2nd, bool *y_result_2nd, int *sin_x_2nd, int *sin_y_2nd);
int is_vendor_ois_set_power_mode(struct v4l2_subdev *subdev, int forceMode);
#endif /* CAMERA_2ND_OIS */
void is_vendor_ois_enable(struct is_core *core);
int is_vendor_ois_disable(struct v4l2_subdev *subdev);
void is_vendor_ois_get_hall_position(struct is_core *core, u16 *targetPos, u16 *hallPos);
bool is_vendor_ois_offset_test(struct is_core *core, long *raw_data_x, long *raw_data_y, long *raw_data_z);
void is_vendor_ois_get_offset_data(struct is_core *core, long *raw_data_x, long *raw_data_y, long *raw_data_z);
void is_vendor_ois_gyro_sleep(struct is_core *core);
void is_vendor_ois_exif_data(struct is_core *core);
u8 is_vendor_ois_read_status(struct is_core *core);
u8 is_vendor_ois_read_cal_checksum(struct is_core *core);
int is_vendor_ois_set_coef(struct v4l2_subdev *subdev, u8 coef);
void is_vendor_ois_set_center_shift(struct v4l2_subdev *subdev, int16_t *shiftValue);
int is_vendor_ois_set_centering(struct v4l2_subdev *subdev);
u8 is_vendor_ois_read_mode(struct v4l2_subdev *subdev);
bool is_vendor_ois_gyro_cal(struct is_core *core, long *x_value, long *y_value, long *z_value);
bool is_vendor_ois_read_gyro_noise(struct is_core *core, long *x_value, long *y_value);
#ifdef USE_OIS_HALL_DATA_FOR_VDIS
int is_vendor_ois_get_hall_data(struct v4l2_subdev *subdev, struct is_ois_hall_data *halldata);
#endif
void is_vendor_ois_check_valid(struct v4l2_subdev *subdev, u8 *value);
bool is_vendor_ois_get_active(struct v4l2_subdev *subdev);
#endif
