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

#ifndef IS_VENDOR_OIS_INTERNAL_MCU_H
#define IS_VENDOR_OIS_INTERNAL_MCU_H

#include <media/v4l2-subdev.h>
#include "is-core.h"
#include "is-interface-sensor.h"

#define	IS_MCU_FW_NAME		"is_mcu_fw.bin"
#define	IS_MCU_PATH		"/system/vendor/firmware/"

#define	MCU_AF_MODE_STANDBY			0x40
#ifdef USE_TELE2_OIS_AF_COMMON_INTERFACE
#define	MCU_AF_MODE_ACTIVE			0x00
#define	MCU_ACT_DEFAULT_FIRST_POSITION		2048
#endif
#if defined(USE_TELE_OIS_AF_COMMON_INTERFACE) || defined(USE_TELE2_OIS_AF_COMMON_INTERFACE)
#define	MCU_ACT_POS_SIZE_BIT		ACTUATOR_POS_SIZE_12BIT
#define	MCU_ACT_POS_MAX_SIZE		((1 << MCU_ACT_POS_SIZE_BIT) - 1)
#define	MCU_ACT_POS_DIRECTION		ACTUATOR_RANGE_INF_TO_MAC
#endif
#ifndef OIS_DUAL_CAL_DEFAULT_VALUE_TELE
#define	MCU_HALL_SHIFT_ADDR_X_M2	0x02AA
#define	MCU_HALL_SHIFT_ADDR_Y_M2	0x02AC
#endif
#if IS_ENABLED(CONFIG_CAMERA_HW_BIG_DATA)
#define	MCU_I2C_ERR_VAL				0x7E00
#define	MCU_REAR_OIS_ERR_REG		0x0600
#define	MCU_REAR_2ND_OIS_ERR_REG	0x1800
#define	MCU_REAR_3RD_OIS_ERR_REG	0x6000
#endif
#define	MCU_BYPASS_MODE_WRITE_ID	0x48
#define	MCU_BYPASS_MODE_READ_ID	0x49
#define	MCU_HW_VERSION_OFFSET		0x7C
#define	MCU_BIN_VERSION_OFFSET		0xF8
#define	MCU_SHARED_SRC_ON_COUNT		1
#define	MCU_SHARED_SRC_OFF_COUNT		0

enum ois_mcu_uw_mode {
	OIS_USE_UW_NONE = 0,
	OIS_USE_UW_ONLY,
	OIS_USE_UW_WIDE,
};

enum is_efs_state {
	IS_EFS_STATE_READ,
};

struct mcu_efs_info {
	unsigned long	efs_state;
	s16 ois_hall_shift_x;
	s16 ois_hall_shift_y;
};

/*
 * APIs
 */
int is_vendor_ois_power_ctrl(struct ois_mcu_dev *mcu, int on);
int is_vendor_ois_load_binary(struct ois_mcu_dev *mcu);
int is_vendor_ois_core_ctrl(struct ois_mcu_dev *mcu, int on);
int is_vendor_ois_dump(struct ois_mcu_dev *mcu, int type);
void is_vendor_ois_device_ctrl(struct ois_mcu_dev *mcu, u8 value);
int is_vendor_ois_set_dev_ctrl(struct v4l2_subdev *subdev, int forceMode);
#if IS_ENABLED(CONFIG_CAMERA_HW_BIG_DATA)
void is_vendor_ois_get_hw_param(struct cam_hw_param *hw_param, u16 i2c_error_reg);
#endif

struct platform_driver *get_internal_ois_platform_driver(void);
#endif
