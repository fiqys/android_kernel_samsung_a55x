// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos SoC series Pablo driver
 *
 * Exynos Pablo image subsystem functions
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/vmalloc.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <exynos-is-sensor.h>
#include "is-device-sensor-peri.h"
#include "is-vendor-ois.h"
#include "is-vendor-ois-core.h"
#include "is-vendor-ois-reg.h"
#include "is-device-ois_common.h"
#ifdef CONFIG_AF_HOST_CONTROL
#include "is-device-af.h"
#endif
#include "is-vendor-private.h"
#include "is-sec-define.h"

#if defined(CONFIG_CAMERA_USE_EXTERNAL_MCU)
#include "is-vendor-ois-external-mcu.h"
#include <linux/pinctrl/pinctrl.h>
#include "is-device-ischain.h"
#include "is-dt.h"
#include "is-core.h"
#include "is-interface.h"
#elif defined(CONFIG_CAMERA_USE_AOIS)
#include "is-vendor-ois-advanced.h"
#include "is-interface-aois.h"
#include <linux/pinctrl/pinctrl.h>
#include "is-device-ischain.h"
#include "is-dt.h"
#include "is-core.h"
#include "is-interface.h"
#else
#include "is-vendor-ois-internal-mcu.h"
#include <linux/clk.h>
#include <linux/file.h>
#include <soc/samsung/exynos-pmu-if.h>
#include "pablo-hw-api-common.h"
#include "is-hw-api-ois-mcu.h"
#endif
#include "is-ixc-config.h"

struct ois_error_info {
	u8 mask;
	const char description[15];
};

#if defined(CONFIG_CAMERA_USE_EXTERNAL_MCU) || defined(CONFIG_CAMERA_USE_AOIS)
u8 is_mcu_get_reg_u8(__always_unused void __iomem *base, int cmd)
{
	u8 ret = 0;
#if defined(CONFIG_CAMERA_USE_EXTERNAL_MCU)
	ret = is_ois_external_mcu_read_u8(cmd, &ret);
#else
	ret = is_ois_advanced_read_u8(cmd, &ret);
#endif
	return ret;
}
void is_mcu_set_reg_u8(__always_unused void __iomem *base, int cmd, u8 val)
{
#if defined(CONFIG_CAMERA_USE_EXTERNAL_MCU)
	is_ois_external_mcu_write_u8(cmd, val);
#else
	is_ois_advanced_write_u8(cmd, val);
#endif
}
#endif

#if defined(CONFIG_CAMERA_USE_EXTERNAL_MCU) || defined(CONFIG_CAMERA_USE_AOIS)
#define OIS_LOCK(lock) { IXC_MUTEX_LOCK(lock); }
#define OIS_UNLOCK(lock) { IXC_MUTEX_UNLOCK(lock); }
#else
#define OIS_LOCK(lock) {}
#define OIS_UNLOCK(lock) {}
#endif

#define	MIN_AF_POSITION	1
#define	MCU_AF_INIT_POSITION		0x7F

static u64 timestampboot;
#if !defined(OIS_DUAL_CAL_DEFAULT_VALUE_TELE) && defined(CAMERA_2ND_OIS)
static struct mcu_efs_info efs_info;
#endif

#ifdef USE_OIS_DEBUGGING_LOG
void ois_mcu_debug_log(struct ois_mcu_dev *mcu)
{
	u8 status, err_status, checksum;

	status = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
	err_status = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_ERROR_STATUS);
	checksum = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CHECKSUM);
	info_mcu("Err status (%02X,%02X,%02X)\n", status, err_status, checksum);
}
#endif

#if defined(USE_TELE_OIS_AF_COMMON_INTERFACE) || defined(USE_TELE2_OIS_AF_COMMON_INTERFACE)
int is_vendor_ois_af_move_lens(struct is_core *core)
{
	struct ois_mcu_dev *mcu = NULL;
	struct is_ois *ois = NULL;
	struct is_mcu *is_mcu = NULL;

	mcu = core->mcu;

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err_mcu("%s, is_mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}
	ois = is_mcu->ois;

	info_mcu("%s : E\n", __func__);
	OIS_LOCK(ois->ixc_lock);
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CTRL_AF, MCU_AF_MODE_ACTIVE);
#if defined(USE_TELE_OIS_AF_COMMON_INTERFACE)
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_POS1_REAR2_AF, 0x80);
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_POS2_REAR2_AF, 0x00);
#elif defined(USE_TELE2_OIS_AF_COMMON_INTERFACE)
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_POS1_REAR3_AF, 0x80);
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_POS2_REAR3_AF, 0x00);
#endif
	OIS_UNLOCK(ois->ixc_lock);

	info_mcu("%s : X\n", __func__);

	return 0;
}
#endif

void is_vendor_ois_parsing_raw_data(uint8_t *buf, long efs_size, long *raw_data_x, long *raw_data_y, long *raw_data_z)
{
	int ret;
	int i = 0, j = 0;
	char efs_data_pre[MAX_GYRO_EFS_DATA_LENGTH + 1];
	char efs_data_post[MAX_GYRO_EFS_DATA_LENGTH + 1];
	bool detect_point = false;
	int sign = 1;
	long raw_pre = 0, raw_post = 0;

	memset(efs_data_pre, 0x0, sizeof(efs_data_pre));
	memset(efs_data_post, 0x0, sizeof(efs_data_post));
	i = 0;
	j = 0;
	while ((*(buf + i)) != ',') {
		if (((char)*(buf + i)) == '-' ) {
			sign = -1;
			i++;
		}

		if (((char)*(buf + i)) == '.') {
			detect_point = true;
			i++;
			j = 0;
		}

		if (detect_point) {
			memcpy(efs_data_post + j, buf + i, 1);
			j++;
		} else {
			memcpy(efs_data_pre + j, buf + i, 1);
			j++;
		}

		if (++i > MAX_GYRO_EFS_DATA_LENGTH) {
			err_mcu("wrong EFS data");
			break;
		}
	}
	i++;
	ret = kstrtol(efs_data_pre, 10, &raw_pre);
	ret = kstrtol(efs_data_post, 10, &raw_post);
	*raw_data_x = sign * (raw_pre * 1000 + raw_post);

	detect_point = false;
	j = 0;
	raw_pre = 0;
	raw_post = 0;
	sign = 1;
	memset(efs_data_pre, 0x0, sizeof(efs_data_pre));
	memset(efs_data_post, 0x0, sizeof(efs_data_post));
	while ((*(buf + i)) != ',') {
		if (((char)*(buf + i)) == '-' ) {
			sign = -1;
			i++;
		}

		if (((char)*(buf + i)) == '.') {
			detect_point = true;
			i++;
			j = 0;
		}

		if (detect_point) {
			memcpy(efs_data_post + j, buf + i, 1);
			j++;
		} else {
			memcpy(efs_data_pre + j, buf + i, 1);
			j++;
		}

		if (++i > MAX_GYRO_EFS_DATA_LENGTH) {
			err_mcu("wrong EFS data");
			break;
		}
	}
	ret = kstrtol(efs_data_pre, 10, &raw_pre);
	ret = kstrtol(efs_data_post, 10, &raw_post);
	*raw_data_y = sign * (raw_pre * 1000 + raw_post);

	detect_point = false;
	j = 0;
	raw_pre = 0;
	raw_post = 0;
	sign = 1;
	memset(efs_data_pre, 0x0, sizeof(efs_data_pre));
	memset(efs_data_post, 0x0, sizeof(efs_data_post));
	while (i < efs_size) {
		if (((char)*(buf + i)) == '-' ) {
			sign = -1;
			i++;
		}

		if (((char)*(buf + i)) == '.') {
			detect_point = true;
			i++;
			j = 0;
		}

		if (detect_point) {
			memcpy(efs_data_post + j, buf + i, 1);
			j++;
		} else {
			memcpy(efs_data_pre + j, buf + i, 1);
			j++;
		}

		if (i++ > MAX_GYRO_EFS_DATA_LENGTH) {
			err_mcu("wrong EFS data");
			break;
		}
	}
	ret = kstrtol(efs_data_pre, 10, &raw_pre);
	ret = kstrtol(efs_data_post, 10, &raw_post);
	*raw_data_z = sign * (raw_pre * 1000 + raw_post);

	info_mcu("%s : X raw_x = %ld, raw_y = %ld, raw_z = %ld\n", __func__, *raw_data_x, *raw_data_y, *raw_data_z);
}

long is_vendor_ois_get_efs_data(struct ois_mcu_dev *mcu, long *raw_data_x, long *raw_data_y, long *raw_data_z)
{
	long efs_size = 0;
	struct is_core *core = NULL;
	struct is_vendor_private *vendor_priv;

	core = is_get_is_core();
	vendor_priv = core->vendor.private_data;

	info_mcu("%s : E\n", __func__);

	efs_size = vendor_priv->gyro_efs_size;

	if (efs_size == 0) {
		err_mcu("efs read failed");
		goto p_err;
	}

	is_vendor_ois_parsing_raw_data(vendor_priv->gyro_efs_data, efs_size, raw_data_x, raw_data_y, raw_data_z);

p_err:
	return efs_size;
}

int is_vendor_ois_set_ggfadeupdown(struct v4l2_subdev *subdev, int up, int down)
{
	int ret = 0;
	struct is_ois *ois = NULL;
	struct is_mcu *is_mcu = NULL;
	struct ois_mcu_dev *mcu = NULL;
	u8 status = 0;
	int retries = 100;
	u8 data[2];

	WARN_ON(!subdev);

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err_mcu("mcu is NULL");
		ret = -EINVAL;
		return ret;
	}

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err_mcu("%s, is_mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	ois = is_mcu->ois;

	dbg_ois("%s up:%d down:%d\n", __func__, up, down);

	OIS_LOCK(ois->ixc_lock);

	/* Wide af position value */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_AF, MCU_AF_INIT_POSITION);

#if defined(CAMERA_2ND_OIS)
	/* Tele af position value */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_AF, MCU_AF_INIT_POSITION);
#endif
#if defined(CAMERA_3RD_OIS)
	/* Tele2 af position value */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR3_AF, MCU_AF_INIT_POSITION);
#endif

	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CACTRL_WRITE, 0x01);

	/* set fadeup */
	data[0] = up & 0xFF;
	data[1] = (up >> 8) & 0xFF;
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_FADE_UP1, data[0]);
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_FADE_UP2, data[1]);

	/* set fadedown */
	data[0] = down & 0xFF;
	data[1] = (down >> 8) & 0xFF;
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_FADE_DOWN1, data[0]);
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_FADE_DOWN2, data[1]);

	/* wait idle status
	 * 100msec delay is needed between "ois_power_on" and "ois_mode_s6".
	 */
	do {
		status = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
		if (status == 0x01 || status == 0x13)
			break;
		if (--retries < 0) {
			err_mcu("%s : read register fail!. status: 0x%x\n", __func__, status);
			ret = -1;
			break;
		}
		usleep_range(1000, 1100);
	} while (status != 0x01);

	OIS_UNLOCK(ois->ixc_lock);

	dbg_ois("%s retryCount = %d , status = 0x%x\n", __func__, 100 - retries, status);

	return ret;
}

void is_vendor_ois_reset_mcu(struct ois_mcu_dev *mcu)
{
#ifdef USE_TELE2_OIS_AF_COMMON_INTERFACE
	/* write AF CTRL standby */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CTRL_AF, MCU_AF_MODE_STANDBY);
	usleep_range(10000, 11000);
#endif
	/* clear ois err reg */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CHECKSUM, 0x0);
#ifdef USE_TELE2_OIS_AF_COMMON_INTERFACE
	/* write AF CTRL active */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CTRL_AF, MCU_AF_MODE_ACTIVE);
#endif
}

static void is_vendor_ois_check_errors(u8 error_reg, u8 checksum_reg)
{
	char error_str[150];
	int i;
	int len = 0;
	const struct ois_error_info ois_error_status_info[] = {
		{ 0x01, "GX " },
		{ 0x02, "GY " },
		{ 0x04, "GZ " },
		{ 0x08, "GCOMM " },
	};
	const struct ois_error_info ois_checksum_info[] = {
		{ 0x03, "W " },
		{ 0x0C, "T " },
		{ 0x30, "T2 " },
	};

	if (error_reg == 0x00 && checksum_reg == 0x00) {
		info_mcu("%s No errors detected", __func__);
		return;
	}

	memset(error_str, 0, sizeof(error_str));

	if (error_reg != 0x00) {
		len += sprintf(error_str + len, "Gyro Error (0x%02X): ", error_reg);
		for (i = 0; i < ARRAY_SIZE(ois_error_status_info); i++)
			if (error_reg & ois_error_status_info[i].mask)
				len += sprintf(error_str + len, "%s", ois_error_status_info[i].description);
	}

	if (checksum_reg != 0x00) {
		if (error_reg != 0x00) {
			len--;
			len += sprintf(error_str + len, ", ");
		}
		len += sprintf(error_str + len, "OIS Checksum Error (0x%02X): ", checksum_reg);
		for (i = 0; i < ARRAY_SIZE(ois_checksum_info); i++)
			if (checksum_reg & ois_checksum_info[i].mask)
				len += sprintf(error_str + len, "%s", ois_checksum_info[i].description);
	}

	err_mcu("%s", error_str);
}

int is_vendor_ois_init(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u8 val = 0;
	u8 error_reg[2] = {0, };
	u8 gyro_orientation = 0;
	u8 wx_pole = 0;
	u8 wy_pole = 0;
#if defined(CAMERA_2ND_OIS)
	u8 tx_pole = 0;
	u8 ty_pole = 0;
#endif
#if defined(CAMERA_3RD_OIS)
	u8 t2x_pole = 0;
	u8 t2y_pole = 0;
#endif
	int retries = 600;
	int i = 0;
	int scale_factor = OIS_GYRO_SCALE_FACTOR;
	long gyro_data_x = 0, gyro_data_y = 0, gyro_data_z = 0, gyro_data_size = 0;
	u8 gyro_x = 0, gyro_x2 = 0;
	u8 gyro_y = 0, gyro_y2 = 0;
	u8 gyro_z = 0, gyro_z2 = 0;
#if defined(CAMERA_2ND_OIS)
	int tele_cmd_xcoef = 0;
	int tele_cmd_ycoef = 0;
#ifndef OIS_DUAL_CAL_DEFAULT_VALUE_TELE
	struct is_vendor_private *vendor_priv;
	u8 tele_xcoef[2];
	u8 tele_ycoef[2];
	long efs_size = 0;
#ifndef OIS_DUAL_CAL_DEFAULT_EEPROM_VALUE_TELE
	int rom_id = 0;
	char *cal_buf;
	struct is_vendor_rom *rom_info = NULL;
	u8 eeprom_xcoef[2];
	u8 eeprom_ycoef[2];
#endif
#endif
#endif /* CAMERA_2ND_OIS */
	struct is_mcu *is_mcu = NULL;
	struct ois_mcu_dev *mcu = NULL;
	struct is_ois *ois = NULL;
	struct is_module_enum *module = NULL;
	struct is_device_sensor_peri *sensor_peri = NULL;
	struct is_ois_info *ois_pinfo = NULL;
	struct is_core *core = NULL;
#if IS_ENABLED(CONFIG_CAMERA_HW_BIG_DATA)
	struct cam_hw_param *hw_param = NULL;
	u16 i2c_error_reg = 0;
#endif

	WARN_ON(!subdev);

	info_mcu("%s E\n", __func__);

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err_mcu("mcu is NULL");
		ret = -EINVAL;
		return ret;
	}

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err_mcu("is_mcu is NULL");
		ret = -EINVAL;
		return ret;
	}

	sensor_peri = is_mcu->sensor_peri;
	if (!sensor_peri) {
		err_mcu("sensor_peri is NULL");
		ret = -EINVAL;
		return ret;
	}

	module = sensor_peri->module;
	if (!module) {
		err_mcu("module is NULL");
		ret = -EINVAL;
		return ret;
	}

	core = is_get_is_core();
	if (!core) {
		err_mcu("core is null");
		ret = -EINVAL;
		return ret;
	}

	is_ois_get_phone_version(&ois_pinfo);

	ois = is_mcu->ois;
	ois->pre_ois_mode = OPTICAL_STABILIZATION_MODE_OFF;
	ois->ois_mode = sensor_peri->mcu->ois->ois_mode;
	ois->coef = 0;
	ois->pre_coef = 255;
	ois->fadeupdown = false;
	ois->initial_centering_mode = false;
	ois->af_pos_wide = 0;
#if defined(CAMERA_2ND_OIS)
	ois->af_pos_tele = 0;
#endif
	ois->ois_power_mode = -1;
	ois_pinfo->reset_check = false;

	if (mcu->ois_hw_check) {
		if (module->position == SENSOR_POSITION_REAR)
			mcu->ois_wide_init = true;
#if defined(CAMERA_2ND_OIS)
		else if (module->position == SENSOR_POSITION_REAR2)
			mcu->ois_tele_init = true;
#endif
#if defined(CAMERA_3RD_OIS)
		else if (module->position == SENSOR_POSITION_REAR4)
			mcu->ois_tele2_init = true;
#endif

		info_mcu("%s %d sensor(%d) mcu is already initialized.\n", __func__, __LINE__, module->position);
		ois->ois_shift_available = true;
	}
	OIS_LOCK(ois->ixc_lock);

	if (mcu->ois_hw_check && mcu->current_error_reg && (mcu->current_power_mode >= OIS_POWER_MODE_DUAL)) {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
		if (val == 0x01) { //idle status check, 0x01 == idle.
#if IS_ENABLED(CONFIG_CAMERA_HW_BIG_DATA)
			i2c_error_reg = mcu->current_error_reg;
			if (i2c_error_reg & MCU_I2C_ERR_VAL)
				is_vendor_ois_get_hw_param(hw_param, i2c_error_reg);
#endif
#ifdef CONFIG_CAMERA_USE_INTERNAL_MCU
			is_vendor_ois_reset_mcu(mcu);
			is_vendor_ois_set_dev_ctrl(subdev, 1);
#endif
			info_mcu("%s Process dev ctrl again in case of error reg detected.", __func__);
			error_reg[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_ERROR_STATUS);
			error_reg[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CHECKSUM);
			mcu->current_error_reg = (error_reg[1] << 8) | error_reg[0];
#if IS_ENABLED(CONFIG_CAMERA_HW_BIG_DATA)
			if (hw_param && (hw_param->i2c_ois_err_cnt > 0) && (mcu->current_error_reg == 0))
				hw_param->i2c_ois_err_cnt--;
#endif
			is_vendor_ois_check_errors(error_reg[0], error_reg[1]);
		} else {
			info_mcu("%s Do not process dev ctrl again. Mcu is not idle.", __func__);
		}
	}

	if (!mcu->ois_hw_check && test_bit(OM_HW_RUN, &mcu->state)) {
		do {
			val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
			usleep_range(500, 510);
			if (--retries < 0) {
				err_mcu("Read status failed!!!!, data = 0x%04x", val);
				break;
			}
		} while (val != 0x01);

		error_reg[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_ERROR_STATUS);
		error_reg[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CHECKSUM);
		mcu->current_error_reg = (error_reg[1] << 8) | error_reg[0];
		is_vendor_ois_check_errors(error_reg[0], error_reg[1]);

#if IS_ENABLED(CONFIG_CAMERA_HW_BIG_DATA)
#if defined(CAMERA_3RD_OIS)
		if (mcu->current_power_mode >= OIS_POWER_MODE_TRIPLE)
#elif defined(CAMERA_2ND_OIS)
		if (mcu->current_power_mode >= OIS_POWER_MODE_DUAL)
#endif
		{
			i2c_error_reg = (error_reg[1] << 8) | error_reg[0];
			if (i2c_error_reg & MCU_I2C_ERR_VAL) {
				is_vendor_ois_get_hw_param(hw_param, i2c_error_reg);
				if (hw_param) {
					hw_param->i2c_ois_err_cnt++;
					if (i2c_error_reg & MCU_REAR_3RD_OIS_ERR_REG)
						hw_param->i2c_af_err_cnt++;
				}
			}
		}
#endif

		/* MCU err reg recovery code */
		if (core->mcu->need_reset_mcu && error_reg[1]) {
#ifdef CONFIG_CAMERA_USE_INTERNAL_MCU
			is_vendor_ois_reset_mcu(mcu);
			info("[%s] clear ois reset flag.", __func__);
#endif
			core->mcu->need_reset_mcu = false;
		}

		if (val == 0x01) {
			/* loading gyro data */
			gyro_data_size = is_vendor_ois_get_efs_data(mcu, &gyro_data_x, &gyro_data_y, &gyro_data_z);
			info_mcu("Read Gyro offset data :  0x%04lx, 0x%04lx, 0x%04lx", gyro_data_x, gyro_data_y, gyro_data_z);
			gyro_data_x = gyro_data_x * scale_factor;
			gyro_data_y = gyro_data_y * scale_factor;
			gyro_data_z = gyro_data_z * scale_factor;
			gyro_data_x = gyro_data_x / 1000;
			gyro_data_y = gyro_data_y / 1000;
			gyro_data_z = gyro_data_z / 1000;
			if (gyro_data_size > 0) {
				gyro_x = gyro_data_x & 0xFF;
				gyro_x2 = (gyro_data_x >> 8) & 0xFF;
				gyro_y = gyro_data_y & 0xFF;
				gyro_y2 = (gyro_data_y >> 8) & 0xFF;
				gyro_z = gyro_data_z & 0xFF;
				gyro_z2 = (gyro_data_z >> 8) & 0xFF;
				is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_X1, gyro_x);
				is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_X2, gyro_x2);
				is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_Y1, gyro_y);
				is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_Y2, gyro_y2);
				is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_Z1, gyro_z);
				is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_Z2, gyro_z2);
				info_mcu("Write Gyro offset data :  0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x", gyro_x, gyro_x2, gyro_y, gyro_y2, gyro_z, gyro_z2);
			}
			/* write wide xgg ygg xcoef ycoef */
			if (ois_pinfo->wide_romdata.cal_mark[0] == 0xBB) {
				for (i = 0; i < 4; i++) {
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_XGG1 + i, ois_pinfo->wide_romdata.xgg[i]);
				}
				for (i = 0; i < 4; i++) {
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_YGG1 + i, ois_pinfo->wide_romdata.ygg[i]);
				}
				for (i = 0; i < 2; i++) {
#ifdef OIS_DUAL_CAL_DEFAULT_VALUE_WIDE
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_XCOEF_M1_1 + i, OIS_DUAL_CAL_DEFAULT_VALUE_WIDE);
#else
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_XCOEF_M1_1 + i, ois_pinfo->wide_romdata.xcoef[i]);
#endif
				}
				for (i = 0; i < 2; i++) {
#ifdef OIS_DUAL_CAL_DEFAULT_VALUE_WIDE
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_YCOEF_M1_1 + i, OIS_DUAL_CAL_DEFAULT_VALUE_WIDE);
#else
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_YCOEF_M1_1 + i, ois_pinfo->wide_romdata.ycoef[i]);
#endif
				}
			} else {
				info_mcu("%s Does not loading wide xgg/ygg data from eeprom.", __func__);
			}

#if defined(CAMERA_2ND_OIS)
			/* write tele xgg ygg xcoef ycoef */
			if (ois_pinfo->tele_tilt_romdata.cal_mark[0] == 0xBB) {
#ifdef OIS_DUAL_CAL_USE_REAR3_DATA
				tele_cmd_xcoef = OIS_CMD_XCOEF_M3_1;
				tele_cmd_ycoef = OIS_CMD_YCOEF_M3_1;
#else
				tele_cmd_xcoef = OIS_CMD_XCOEF_M2_1;
				tele_cmd_ycoef = OIS_CMD_YCOEF_M2_1;
#endif

#ifndef OIS_DUAL_CAL_DEFAULT_VALUE_TELE
				vendor_priv = core->vendor.private_data;
				efs_size = vendor_priv->tilt_cal_tele2_efs_size;
				if (efs_size) {
					efs_info.ois_hall_shift_x = *((s16 *)&vendor_priv->tilt_cal_tele2_efs_data[MCU_HALL_SHIFT_ADDR_X_M2]);
					efs_info.ois_hall_shift_y = *((s16 *)&vendor_priv->tilt_cal_tele2_efs_data[MCU_HALL_SHIFT_ADDR_Y_M2]);
					set_bit(IS_EFS_STATE_READ, &efs_info.efs_state);
				} else {
					clear_bit(IS_EFS_STATE_READ, &efs_info.efs_state);
				}
#endif
				for (i = 0; i < 4; i++) {
#if defined(CAMERA_2ND_OIS)
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_XGG1 + i, ois_pinfo->tele_romdata.xgg[i]);
#endif
#if defined(CAMERA_3RD_OIS)
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR3_XGG1 + i, ois_pinfo->tele2_romdata.xgg[i]);
#endif
				}
				for (i = 0; i < 4; i++) {
#if defined(CAMERA_2ND_OIS)
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_YGG1 + i, ois_pinfo->tele_romdata.ygg[i]);
#endif
#if defined(CAMERA_3RD_OIS)
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR3_YGG1 + i, ois_pinfo->tele2_romdata.ygg[i]);
#endif
				}
#ifdef OIS_DUAL_CAL_DEFAULT_VALUE_TELE
				is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_xcoef, OIS_DUAL_CAL_DEFAULT_VALUE_TELE);
				is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_xcoef + 1, OIS_DUAL_CAL_DEFAULT_VALUE_TELE);
				is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_ycoef, OIS_DUAL_CAL_DEFAULT_VALUE_TELE);
				is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_ycoef + 1, OIS_DUAL_CAL_DEFAULT_VALUE_TELE);

				info_mcu("%s tele use default coef value", __func__);
#else
				if (!test_bit(IS_EFS_STATE_READ, &efs_info.efs_state)) {
#ifdef OIS_DUAL_CAL_DEFAULT_EEPROM_VALUE_TELE
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_xcoef, OIS_DUAL_CAL_DEFAULT_EEPROM_VALUE_TELE);
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_xcoef + 1, OIS_DUAL_CAL_DEFAULT_EEPROM_VALUE_TELE);
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_ycoef, OIS_DUAL_CAL_DEFAULT_EEPROM_VALUE_TELE);
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_ycoef + 1, OIS_DUAL_CAL_DEFAULT_EEPROM_VALUE_TELE);

					info_mcu("%s tele use default eeprom coef value", __func__);
#else
					rom_id = is_vendor_get_rom_id_from_position(SENSOR_POSITION_REAR2);
					is_sec_get_rom_info(&rom_info, rom_id);
					cal_buf = rom_info->buf;

					eeprom_xcoef[0] = *((u8 *)&cal_buf[rom_info->rom_dualcal_slave1_oisshift_x_addr]);
					eeprom_xcoef[1] = *((u8 *)&cal_buf[rom_info->rom_dualcal_slave1_oisshift_x_addr + 1]);
					eeprom_ycoef[0] = *((u8 *)&cal_buf[rom_info->rom_dualcal_slave1_oisshift_y_addr]);
					eeprom_ycoef[1] = *((u8 *)&cal_buf[rom_info->rom_dualcal_slave1_oisshift_y_addr + 1]);

					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_xcoef, eeprom_xcoef[0]);
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_xcoef + 1, eeprom_xcoef[1]);
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_ycoef, eeprom_ycoef[0]);
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_ycoef + 1, eeprom_ycoef[1]);

					info_mcu("%s tele eeprom xcoef = %d/%d, ycoef = %d/%d", __func__, eeprom_xcoef[0], eeprom_xcoef[1],
						eeprom_ycoef[0], eeprom_ycoef[1]);
#endif
				} else {
#ifdef USE_OIS_SHIFT_FOR_12BIT
					efs_info.ois_hall_shift_x >>= 2;
					efs_info.ois_hall_shift_y >>= 2;
#endif
					tele_xcoef[0] = efs_info.ois_hall_shift_x & 0xFF;
					tele_xcoef[1] = (efs_info.ois_hall_shift_x >> 8) & 0xFF;
					tele_ycoef[0] = efs_info.ois_hall_shift_y & 0xFF;
					tele_ycoef[1] = (efs_info.ois_hall_shift_y >> 8) & 0xFF;

					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_xcoef, tele_xcoef[0]);
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_xcoef + 1, tele_xcoef[1]);
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_ycoef, tele_ycoef[0]);
					is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], tele_cmd_ycoef + 1, tele_ycoef[1]);

					info_mcu("%s tele efs xcoef = %d, ycoef = %d", __func__, efs_info.ois_hall_shift_x, efs_info.ois_hall_shift_y);
				}
#endif
			} else {
				info_mcu("%s Does not loading tele xgg/ygg data from eeprom.", __func__);
			}
#endif /* CAMERA_2ND_OIS */

			/* enable dual cal */
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_ENABLE_DUALCAL, 0x01);

			wx_pole = mcu->ois_gyro_direction[0];
			wy_pole = mcu->ois_gyro_direction[1];
			gyro_orientation = mcu->ois_gyro_direction[2];
#if defined(CAMERA_2ND_OIS)
			tx_pole = mcu->ois_gyro_direction[3];
			ty_pole = mcu->ois_gyro_direction[4];
#endif
#if defined(CAMERA_3RD_OIS)
			t2x_pole = mcu->ois_gyro_direction[5];
			t2y_pole = mcu->ois_gyro_direction[6];
#endif

#if defined(CAMERA_3RD_OIS)
			info_mcu("%s gyro direction list  %d,%d,%d,%d,%d,%d,%d\n", __func__, wx_pole, wy_pole, gyro_orientation,
				tx_pole, ty_pole, t2x_pole, t2y_pole);
#elif defined(CAMERA_2ND_OIS)
			info_mcu("%s gyro direction list  %d,%d,%d,%d,%d\n", __func__, wx_pole, wy_pole, gyro_orientation,
				tx_pole, ty_pole);
#else
			info_mcu("%s gyro direction list  %d,%d,%d\n", __func__, wx_pole, wy_pole, gyro_orientation);
#endif

			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_POLA_X, wx_pole);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_POLA_Y, wy_pole);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_ORIENT, gyro_orientation);
#if defined(CAMERA_2ND_OIS)
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_POLA_X_M2, tx_pole);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_POLA_Y_M2, ty_pole);
#endif
#if defined(CAMERA_3RD_OIS)
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_POLA_X_M3, t2x_pole);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_POLA_Y_M3, t2y_pole);
#endif
			info_mcu("%s gyro init data applied.\n", __func__);

			mcu->ois_hw_check = true;

			if (module->position == SENSOR_POSITION_REAR)
				mcu->ois_wide_init = true;
#if defined(CAMERA_2ND_OIS)
			else if (module->position == SENSOR_POSITION_REAR2)
				mcu->ois_tele_init = true;
#endif
#if defined(CAMERA_3RD_OIS)
			else if (module->position == SENSOR_POSITION_REAR4) {
				mcu->ois_tele2_init = true;
				mcu->need_af_delay = true;
			}
#endif
		}
	}
	OIS_UNLOCK(ois->ixc_lock);

	info_mcu("%s sensor(%d) X\n", __func__, module->position);

	return ret;
}

int is_vendor_ois_init_factory(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u8 val = 0;
	int retries = 600;
	u8 gyro_orientation = 0;
	struct is_mcu *is_mcu = NULL;
	struct ois_mcu_dev *mcu = NULL;
	struct is_ois *ois = NULL;
	struct is_ois_info *ois_pinfo = NULL;

	WARN_ON(!subdev);

	info_mcu("%s E\n", __func__);

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err_mcu("mcu is NULL");
		ret = -EINVAL;
		return ret;
	}

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err_mcu("is_mcu is NULL");
		ret = -EINVAL;
		return ret;
	}

	is_ois_get_phone_version(&ois_pinfo);

	ois = is_mcu->ois;
	ois->ois_mode = OPTICAL_STABILIZATION_MODE_OFF;
	ois->pre_ois_mode = OPTICAL_STABILIZATION_MODE_OFF;
	ois->coef = 0;
	ois->pre_coef = 255;
	ois->fadeupdown = false;
	ois->initial_centering_mode = false;
	ois->af_pos_wide = 0;
#if defined(CAMERA_2ND_OIS)
	ois->af_pos_tele = 0;
#endif
	ois->ois_power_mode = -1;
	ois_pinfo->reset_check = false;

	if (test_bit(OM_HW_RUN, &mcu->state)) {
		do {
			val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
			usleep_range(500, 510);
			if (--retries < 0) {
				err_mcu("Read status failed!!!!, data = 0x%04x", val);
				break;
			}
		} while (val != 0x01);
	}

	/* OIS SEL (wide : 1 , tele : 2, w/t : 3, tele2 : 4, triple : 7) */
#if defined(CAMERA_3RD_OIS)
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x07);
#elif defined(CAMERA_2ND_OIS)
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x03);
#else
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x01);
#endif

	gyro_orientation = mcu->ois_gyro_direction[2];
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_ORIENT, gyro_orientation);

	info_mcu("%s sensor(%d) X\n", __func__, ois->device);
	return ret;
}

#if defined(CAMERA_3RD_OIS)
void is_vendor_ois_init_rear2(struct is_core *core)
{
	u8 val = 0;
	int retries = 600;
	struct ois_mcu_dev *mcu = NULL;

	mcu = core->mcu;

	info_mcu("%s : E\n", __func__);

	/* check ois status */
	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
		usleep_range(500, 510);
		if (--retries < 0) {
			err_mcu("Read status failed!!!!, data = 0x%04x", val);
			break;
		}
	} while (val != 0x01);

	/* set power mode (wide : 1 , tele : 2, w/t : 3, tele2 : 4, triple : 7) */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x04);

	info_mcu("%s : X\n", __func__);

	return;
}
#endif /* CAMERA_3RD_OIS */

int is_vendor_ois_deinit(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct ois_mcu_dev *mcu = NULL;
	struct is_mcu *is_mcu = NULL;
	struct is_device_sensor_peri *sensor_peri = NULL;
	struct is_module_enum *module = NULL;
	struct is_core *core = NULL;
	int retries = 50;
	u8 val = 0;

	WARN_ON(!subdev);

	info_mcu("%s E\n", __func__);

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err_mcu("mcu subdev is NULL");
		ret = -EINVAL;
		return ret;
	}

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err_mcu("is_mcu is NULL");
		ret = -EINVAL;
		return ret;
	}

	sensor_peri = is_mcu->sensor_peri;
	if (!sensor_peri) {
		err_mcu("sensor_peri is NULL");
		ret = -EINVAL;
		return ret;
	}

	module = sensor_peri->module;
	if (!module) {
		err_mcu("module is NULL");
		ret = -EINVAL;
		return ret;
	}

	core = is_get_is_core();
	if (!core) {
		err_mcu("core is null");
		ret = -EINVAL;
		return ret;
	}

	if (module->position  == SENSOR_POSITION_REAR)
		mcu->ois_wide_init = false;
#if defined(CAMERA_2ND_OIS)
	else if  (module->position  == SENSOR_POSITION_REAR2)
		mcu->ois_tele_init = false;
#endif
#if defined(CAMERA_3RD_OIS)
	else if  (module->position  == SENSOR_POSITION_REAR4)
		mcu->ois_tele2_init = false;
#endif
	if (mcu->ois_hw_check) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x00);
		usleep_range(2000, 2100);
		do {
			val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
			usleep_range(1000, 1100);
			if (--retries < 0) {
				err_mcu("Read status failed!!!!, data = 0x%04x", val);
				break;
			}
		} while (val != 0x01);

		mcu->ois_fadeupdown = false;
		mcu->ois_hw_check = false;
		mcu->need_af_delay = false;
		mcu->is_mcu_active = false;
		info_mcu("%s ois stop. sensor = (%d)X\n", __func__, module->position);
	}

	if (core->mcu->need_reset_mcu) {
		core->mcu->need_reset_mcu = false;
		info("[%s] clear ois reset flag.", __func__);
	}

#if defined(CONFIG_CAMERA_USE_EXTERNAL_MCU) || defined(CONFIG_CAMERA_USE_AOIS)
	clear_bit(OM_HW_RUN, &mcu->state);
#endif

	info_mcu("%s sensor = (%d)X\n", __func__, module->position);

	return ret;
}

int is_vendor_ois_set_mode(struct v4l2_subdev *subdev, int mode)
{
	int ret = 0;
	struct is_ois *ois = NULL;
	struct is_mcu *is_mcu = NULL;
	struct ois_mcu_dev *mcu = NULL;

	WARN_ON(!subdev);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if(!is_mcu) {
		err_mcu("%s, is_mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if(!mcu) {
		err_mcu("%s, mcu subdev is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

#ifndef CONFIG_SEC_FACTORY
	if (!mcu->ois_wide_init
#if defined(CAMERA_2ND_OIS)
		&& !mcu->ois_tele_init
#endif
#if defined(CAMERA_3RD_OIS)
		&& !mcu->ois_tele2_init
#endif
	)
		return 0;
#endif

	ois = is_mcu->ois;

	if (ois->fadeupdown == false) {
		if (mcu->ois_fadeupdown == false) {
			mcu->ois_fadeupdown = true;
			is_vendor_ois_set_ggfadeupdown(subdev, 1000, 1000);
		}
		ois->fadeupdown = true;
	}

	if (mode == ois->pre_ois_mode) {
		return ret;
	}

	ois->pre_ois_mode = mode;
	info_mcu("%s: ois_mode value(%d)\n", __func__, mode);

	OIS_LOCK(ois->ixc_lock);
	switch(mode) {
		case OPTICAL_STABILIZATION_MODE_STILL:
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE, 0x00);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x01);
			break;
		case OPTICAL_STABILIZATION_MODE_VIDEO:
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE, 0x01);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x01);
			break;
		case OPTICAL_STABILIZATION_MODE_CENTERING:
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE, 0x05);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x01);
			break;
		case OPTICAL_STABILIZATION_MODE_HOLD:
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE, 0x06);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x01);
			break;
		case OPTICAL_STABILIZATION_MODE_STILL_ZOOM:
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE, 0x13);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x01);
			break;
		case OPTICAL_STABILIZATION_MODE_VDIS:
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE, 0x14);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x01);
			break;
		case OPTICAL_STABILIZATION_MODE_VDIS_ASR:
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE, 0x15);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x01);
			break;
		case OPTICAL_STABILIZATION_MODE_SINE_X:
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_SINE_1, 0x01);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_SINE_2, 0x01);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_SINE_3, 0x2D);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE, 0x03);
			msleep(20);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x01);
			break;
		case OPTICAL_STABILIZATION_MODE_SINE_Y:
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_SINE_1, 0x02);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_SINE_2, 0x01);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_SINE_3, 0x2D);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE, 0x03);
			msleep(20);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x01);
			break;
		default:
			dbg_ois("%s: ois_mode value(%d)\n", __func__, mode);
			break;
	}
	OIS_UNLOCK(ois->ixc_lock);

#ifdef USE_OIS_STABILIZATION_DELAY
	if (!mcu->is_mcu_active) {
		usleep_range(USE_OIS_STABILIZATION_DELAY, USE_OIS_STABILIZATION_DELAY + 10);
		mcu->is_mcu_active = true;
		info_mcu("%s : Stabilization delay applied\n", __func__);
	}
#endif

	return ret;
}

int is_vendor_ois_shift_compensation(struct v4l2_subdev *subdev, int position, int resolution)
{
	int ret = 0;
	struct is_ois *ois;
	struct ois_mcu_dev *mcu = NULL;
	struct is_mcu *is_mcu = NULL;
	struct is_module_enum *module = NULL;
	struct is_device_sensor_peri *sensor_peri = NULL;
	int position_changed;

	WARN_ON(!subdev);

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if(!mcu) {
		err_mcu("%s, mcu subdev is NULL", __func__);
		ret = -EINVAL;
		goto p_err;
	}

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if(!is_mcu) {
		err_mcu("%s, is_mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	sensor_peri = is_mcu->sensor_peri;
	if (!sensor_peri) {
		err_mcu("%s, sensor_peri is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	module = sensor_peri->module;
	if (!module) {
		err_mcu("%s, module is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	ois = is_mcu->ois;

	position_changed = position >> 4;

	if (position_changed < MIN_AF_POSITION)
		position_changed = MIN_AF_POSITION;

	OIS_LOCK(ois->ixc_lock);

	if (module->position == SENSOR_POSITION_REAR && ois->af_pos_wide != position_changed) {
		/* Wide af position value */
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_AF, (u8)position_changed);
		ois->af_pos_wide = position_changed;
	}
#if !defined(USE_TELE_OIS_AF_COMMON_INTERFACE) && defined(CAMERA_2ND_OIS)
	else if (module->position == SENSOR_POSITION_REAR2 && ois->af_pos_tele != position_changed) {
		/* Tele af position value */
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_AF, (u8)position_changed);
		ois->af_pos_tele = position_changed;
	}
#elif !defined(USE_TELE2_OIS_AF_COMMON_INTERFACE) && defined(CAMERA_3RD_OIS)
	else if (module->position == SENSOR_POSITION_REAR4 && ois->af_pos_tele2 != position_changed) {
		/* Tele af position value */
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR3_AF, (u8)position_changed);
		ois->af_pos_tele2 = position_changed;
	}
	OIS_UNLOCK(ois->ixc_lock);
#endif

p_err:
	return ret;
}

int is_vendor_ois_self_test(struct is_core *core)
{
	u8 val = 0;
	u8 reg_val = 0, x = 0, y = 0, z = 0;
	u16 x_gyro_log = 0, y_gyro_log = 0, z_gyro_log = 0;
	int retries = 30;
	struct ois_mcu_dev *mcu = NULL;

	info_mcu("%s : E\n", __func__);

	mcu = core->mcu;

	is_vendor_ois_set_aois_fac_mode_on();
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_CAL, 0x08);

	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_CAL);
		msleep(50);
		if (--retries < 0) {
			err_mcu("Read register failed!!!!, data = 0x%04x\n", val);
#ifdef USE_OIS_DEBUGGING_LOG
			ois_mcu_debug_log(mcu);
#endif
			break;
		}
	} while (val);

	val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_ERROR_STATUS);

	/* Gyro selfTest result */
	reg_val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_VAL_X);
	x = reg_val;
	reg_val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_LOG_X);
	x_gyro_log = (reg_val << 8) | x;

	reg_val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_VAL_Y);
	y = reg_val;
	reg_val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_LOG_Y);
	y_gyro_log = (reg_val << 8) | y;

	reg_val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_VAL_Z);
	z = reg_val;
	reg_val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_LOG_Z);
	z_gyro_log = (reg_val << 8) | z;

	info_mcu("%s(GSTLOG0=%d, GSTLOG1=%d, GSTLOG2=%d)\n", __func__, x_gyro_log, y_gyro_log, z_gyro_log);

	info_mcu("%s(%d) : X\n", __func__, val);
	is_vendor_ois_set_aois_fac_mode_off();
	return (int)val;
}



bool is_vendor_ois_sine_wavecheck_all(struct is_core *core,
					int threshold, int *sinx, int *siny, int *result,
					int *sinx_2nd, int *siny_2nd, int *sinx_3rd, int *siny_3rd)
{
	u8 buf[2] = {0, }, val = 0;
	int retries = 10;
	int sinx_count = 0, siny_count = 0;
#if defined(CAMERA_2ND_OIS)
	int sinx_count_2nd = 0, siny_count_2nd = 0;
#endif
#if defined(CAMERA_3RD_OIS)
	int sinx_count_3rd = 0, siny_count_3rd = 0;
#endif
	u8 u8_sinx_count[2] = {0, }, u8_siny_count[2] = {0, };
	u8 u8_sinx[2] = {0, }, u8_siny[2] = {0, };
	struct ois_mcu_dev *mcu = NULL;

	mcu = core->mcu;

	is_vendor_ois_set_aois_fac_mode_on();

	/* OIS SEL (wide: 1, tele: 2, w/t: 3, tele2: 4, all: 7) */
#if defined(CAMERA_3RD_OIS)
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x07);
#elif defined(CAMERA_2ND_OIS)
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x03);
#else
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x01);
#endif

	/* Error threshold level */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_THRESH_ERR_LEV, (u8)threshold);
#if defined(CAMERA_2ND_OIS)
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_THRESH_ERR_LEV_M2, (u8)threshold);
#endif
#if defined(CAMERA_3RD_OIS)
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_THRESH_ERR_LEV_M3, (u8)threshold);
#endif

	/* count value for error judgement level */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_ERR_VAL_CNT, 0x00);

	/* frequency level for measurement */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_FREQ_LEV, 0x05);

	/* amplitude level for measurement */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_AMPLI_LEV, 0x2A);

	/* dummy pulse setting */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_DUM_PULSE, 0x03);

	/* vyvle level for measurement */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_VYVLE_LEV, 0x02);

	/* start sine wave check operation */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START_WAVE_CHECK, 0x01);

	retries = 22;
	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START_WAVE_CHECK);
		msleep(100);
		if (--retries < 0) {
			err_mcu("sine wave operation fail, val = 0x%02x.\n", val);
			break;
		}
	} while (val);

	buf[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MCERR_W);
	buf[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MCERR_W2);

	*result = (buf[1] << 8) | buf[0];

	u8_sinx_count[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINX_COUNT1);
	u8_sinx_count[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINX_COUNT2);
	sinx_count = (u8_sinx_count[1] << 8) | u8_sinx_count[0];
	if (sinx_count > 0x7FFF) {
		sinx_count = -((sinx_count ^ 0xFFFF) + 1);
	}
	u8_siny_count[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINY_COUNT1);
	u8_siny_count[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINY_COUNT2);
	siny_count = (u8_siny_count[1] << 8) | u8_siny_count[0];
	if (siny_count > 0x7FFF) {
		siny_count = -((siny_count ^ 0xFFFF) + 1);
	}
	u8_sinx[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINX_DIFF1);
	u8_sinx[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINX_DIFF2);
	*sinx = (u8_sinx[1] << 8) | u8_sinx[0];
	if (*sinx > 0x7FFF) {
		*sinx = -((*sinx ^ 0xFFFF) + 1);
	}
	u8_siny[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINY_DIFF1);
	u8_siny[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINY_DIFF2);
	*siny = (u8_siny[1] << 8) | u8_siny[0];
	if (*siny > 0x7FFF) {
		*siny = -((*siny ^ 0xFFFF) + 1);
	}

	info_mcu("%s threshold = %d, sinx = %d, siny = %d, sinx_count = %d, syny_count = %d, MCERR result = 0x%04x\n",
		__func__, threshold, *sinx, *siny, sinx_count, siny_count, *result);

#if defined(CAMERA_2ND_OIS)
	u8_sinx_count[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINX_COUNT1);
	u8_sinx_count[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINX_COUNT2);
	sinx_count_2nd = (u8_sinx_count[1] << 8) | u8_sinx_count[0];
	if (sinx_count_2nd > 0x7FFF) {
		sinx_count_2nd = -((sinx_count_2nd ^ 0xFFFF) + 1);
	}
	u8_siny_count[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINY_COUNT1);
	u8_siny_count[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINY_COUNT2);
	siny_count_2nd = (u8_siny_count[1] << 8) | u8_siny_count[0];
	if (siny_count_2nd > 0x7FFF) {
		siny_count_2nd = -((siny_count_2nd ^ 0xFFFF) + 1);
	}
	u8_sinx[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINX_DIFF1);
	u8_sinx[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINX_DIFF2);
	*sinx_2nd = (u8_sinx[1] << 8) | u8_sinx[0];
	if (*sinx_2nd > 0x7FFF) {
		*sinx_2nd = -((*sinx_2nd ^ 0xFFFF) + 1);
	}
	u8_siny[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINY_DIFF1);
	u8_siny[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINY_DIFF2);
	*siny_2nd = (u8_siny[1] << 8) | u8_siny[0];
	if (*siny_2nd > 0x7FFF) {
		*siny_2nd = -((*siny_2nd ^ 0xFFFF) + 1);
	}

	info_mcu("%s threshold = %d, sinx_2nd = %d, siny_2nd = %d, sinx_count_2nd = %d, syny_count_2nd = %d\n",
		__func__, threshold, *sinx_2nd, *siny_2nd, sinx_count_2nd, siny_count_2nd);
#endif

#if defined(CAMERA_3RD_OIS)
	u8_sinx_count = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR3_SINX_COUNT1);
	u8_sinx_count = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR3_SINX_COUNT2);
	sinx_count_3rd = (u8_sinx_count[1] << 8) | u8_sinx_count[0];
	if (sinx_count_3rd > 0x7FFF) {
		sinx_count_3rd = -((sinx_count_3rd ^ 0xFFFF) + 1);
	}
	u8_siny_count = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR3_SINY_COUNT1);
	u8_siny_count = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR3_SINY_COUNT2);
	siny_count_3rd = (u8_siny_count[1] << 8) | u8_siny_count[0];
	if (siny_count_3rd > 0x7FFF) {
		siny_count_3rd = -((siny_count_3rd ^ 0xFFFF) + 1);
	}
	u8_sinx[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR3_SINX_DIFF1);
	u8_sinx[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR3_SINX_DIFF2);
	*sinx_3rd = (u8_sinx[1] << 8) | u8_sinx[0];
	if (*sinx_3rd > 0x7FFF) {
		*sinx_3rd = -((*sinx_3rd ^ 0xFFFF) + 1);
	}
	u8_siny[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR3_SINY_DIFF1);
	u8_siny[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR3_SINY_DIFF2);
	*siny_3rd = (u8_siny[1] << 8) | u8_siny[0];
	if (*siny_3rd > 0x7FFF) {
		*siny_3rd = -((*siny_3rd ^ 0xFFFF) + 1);
	}

	info_mcu("%s threshold = %d, sinx_3rd = %d, siny_3rd = %d, sinx_count_3rd = %d, syny_count_3rd = %d\n",
		__func__, threshold, *sinx_3rd, *siny_3rd, sinx_count_3rd, siny_count_3rd);
#endif

	is_vendor_ois_set_aois_fac_mode_off();

	if (*result == 0x0) {
		return true;
	} else {
		warn_mcu("sine wave operation is failed, result = 0x%04x", *result);
		return false;
	}
}

bool is_vendor_ois_auto_test_all(struct is_core *core,
					int threshold, bool *x_result, bool *y_result, int *sin_x, int *sin_y,
					bool *x_result_2nd, bool *y_result_2nd, int *sin_x_2nd, int *sin_y_2nd,
					bool *x_result_3rd, bool *y_result_3rd, int *sin_x_3rd, int *sin_y_3rd)
{
	int result = 0;
	bool value = false;

	info_mcu("%s : E\n", __func__);

#ifdef CONFIG_AF_HOST_CONTROL
#if defined(CAMERA_2ND_OIS)
#ifdef USE_TELE_OIS_AF_COMMON_INTERFACE
	is_vendor_ois_af_move_lens(core);
#else
	is_af_move_lens(core, SENSOR_POSITION_REAR2);
#endif
	msleep(100);
#endif
#if defined(CAMERA_3RD_OIS)
#ifdef USE_TELE2_OIS_AF_COMMON_INTERFACE
	is_vendor_ois_af_move_lens(core);
#else
	is_af_move_lens(core, SENSOR_POSITION_REAR4);
#endif
	msleep(100);
#endif
	is_af_move_lens(core, SENSOR_POSITION_REAR);
	msleep(100);
#endif /* CONFIG_AF_HOST_CONTROL */

	value = is_vendor_ois_sine_wavecheck_all(core, threshold, sin_x, sin_y, &result,
				sin_x_2nd, sin_y_2nd, sin_x_3rd, sin_y_3rd);

	if (*sin_x == -1 && *sin_y == -1) {
		err_mcu("OIS device is not prepared");
		*x_result = false;
		*y_result = false;

		return false;
	}
#if defined(CAMERA_2ND_OIS)
	if (*sin_x_2nd == -1 && *sin_y_2nd == -1) {
		err_mcu("OIS 2 device is not prepared");
		*x_result_2nd = false;
		*y_result_2nd = false;

		return false;
	}
#endif
#if defined(CAMERA_3RD_OIS)
	if (*sin_x_3rd == -1 && *sin_y_3rd == -1) {
		err_mcu("OIS 3 device is not prepared");
		*x_result_3rd = false;
		*y_result_3rd = false;

		return false;
	}
#endif

	if (value == true) {
		*x_result = true;
		*y_result = true;
#if defined(CAMERA_2ND_OIS)
		*x_result_2nd = true;
		*y_result_2nd = true;
#endif
#if defined(CAMERA_3RD_OIS)
		*x_result_3rd = true;
		*y_result_3rd = true;
#endif
		return true;
	} else {
		err_mcu("OIS autotest is failed. result = 0x%04x", result);
		if ((result & 0x03) == 0x00) {
			*x_result = true;
			*y_result = true;
		} else if ((result & 0x03) == 0x01) {
			*x_result = false;
			*y_result = true;
		} else if ((result & 0x03) == 0x02) {
			*x_result = true;
			*y_result = false;
		} else {
			*x_result = false;
			*y_result = false;
		}
#if defined(CAMERA_2ND_OIS)
		if ((result & 0x30) == 0x00) {
			*x_result_2nd = true;
			*y_result_2nd = true;
		} else if ((result & 0x30) == 0x10) {
			*x_result_2nd = false;
			*y_result_2nd = true;
		} else if ((result & 0x30) == 0x20) {
			*x_result_2nd = true;
			*y_result_2nd = false;
		} else {
			*x_result_2nd = false;
			*y_result_2nd = false;
		}
#endif
#if defined(CAMERA_3RD_OIS)
		if ((result & 0x300) == 0x00) {
			*x_result_3rd = true;
			*y_result_3rd = true;
		} else if ((result & 0x300) == 0x100) {
			*x_result_3rd = false;
			*y_result_3rd = true;
		} else if ((result & 0x300) == 0x200) {
			*x_result_3rd = true;
			*y_result_3rd = false;
		} else {
			*x_result_3rd = false;
			*y_result_3rd = false;
		}
#endif
		return false;
	}
}

#if defined(CAMERA_2ND_OIS)
bool is_vendor_ois_sine_wavecheck_rear2(struct is_core *core,
					int threshold, int *sinx, int *siny, int *result,
					int *sinx_2nd, int *siny_2nd)
{
	u8 buf = 0, val = 0;
	int retries = 10;
	int sinx_count = 0, siny_count = 0;
	int sinx_count_2nd = 0, siny_count_2nd = 0;
	u8 u8_sinx_count[2] = {0, }, u8_siny_count[2] = {0, };
	u8 u8_sinx[2] = {0, }, u8_siny[2] = {0, };
	struct ois_mcu_dev *mcu = NULL;

	mcu = core->mcu;

	/* OIS SEL (wide: 1, tele: 2, w/t: 3, tele2: 4, all: 7) */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x03);
	/* error threshold level */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_THRESH_ERR_LEV, (u8)threshold);
	/* error threshold level */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_THRESH_ERR_LEV_M2, (u8)threshold);
	/* count value for error judgement level */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_ERR_VAL_CNT, 0x00);
	/* frequency level for measurement */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_FREQ_LEV, 0x05);
	/* amplitude level for measurement */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_AMPLI_LEV, 0x2A);
	/* dummy pulse setting */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_DUM_PULSE, 0x03);
	/* vyvle level for measurement */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_VYVLE_LEV, 0x02);
	/* start sine wave check operation */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START_WAVE_CHECK, 0x01);

	retries = 22;
	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START_WAVE_CHECK);
		msleep(100);
		if (--retries < 0) {
			err_mcu("sine wave operation fail.\n");
			break;
		}
	} while (val);

	buf = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MCERR_W);

	*result = (int)buf;

	u8_sinx_count[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINX_COUNT1);
	u8_sinx_count[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINX_COUNT2);
	sinx_count = (u8_sinx_count[1] << 8) | u8_sinx_count[0];
	if (sinx_count > 0x7FFF) {
		sinx_count = -((sinx_count ^ 0xFFFF) + 1);
	}
	u8_siny_count[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINY_COUNT1);
	u8_siny_count[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINY_COUNT2);
	siny_count = (u8_siny_count[1] << 8) | u8_siny_count[0];
	if (siny_count > 0x7FFF) {
		siny_count = -((siny_count ^ 0xFFFF) + 1);
	}
	u8_sinx[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINX_DIFF1);
	u8_sinx[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINX_DIFF2);
	*sinx = (u8_sinx[1] << 8) | u8_sinx[0];
	if (*sinx > 0x7FFF) {
		*sinx = -((*sinx ^ 0xFFFF) + 1);
	}
	u8_siny[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINY_DIFF1);
	u8_siny[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR_SINY_DIFF2);
	*siny = (u8_siny[1] << 8) | u8_siny[0];
	if (*siny > 0x7FFF) {
		*siny = -((*siny ^ 0xFFFF) + 1);
	}

	u8_sinx_count[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINX_COUNT1);
	u8_sinx_count[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINX_COUNT2);
	sinx_count_2nd = (u8_sinx_count[1] << 8) | u8_sinx_count[0];
	if (sinx_count_2nd > 0x7FFF) {
		sinx_count_2nd = -((sinx_count_2nd ^ 0xFFFF) + 1);
	}
	u8_siny_count[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINY_COUNT1);
	u8_siny_count[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINY_COUNT2);
	siny_count_2nd = (u8_siny_count[1] << 8) | u8_siny_count[0];
	if (siny_count_2nd > 0x7FFF) {
		siny_count_2nd = -((siny_count_2nd ^ 0xFFFF) + 1);
	}
	u8_sinx[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINX_DIFF1);
	u8_sinx[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINX_DIFF2);
	*sinx_2nd = (u8_sinx[1] << 8) | u8_sinx[0];
	if (*sinx_2nd > 0x7FFF) {
		*sinx_2nd = -((*sinx_2nd ^ 0xFFFF) + 1);
	}
	u8_siny[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINY_DIFF1);
	u8_siny[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_REAR2_SINY_DIFF2);
	*siny_2nd = (u8_siny[1] << 8) | u8_siny[0];
	if (*siny_2nd > 0x7FFF) {
		*siny_2nd = -((*siny_2nd ^ 0xFFFF) + 1);
	}

	info_mcu("threshold = %d, sinx = %d, siny = %d, sinx_count = %d, syny_count = %d\n",
		threshold, *sinx, *siny, sinx_count, siny_count);

	info_mcu("threshold = %d, sinx_2nd = %d, siny_2nd = %d, sinx_count_2nd = %d, syny_count_2nd = %d\n",
		threshold, *sinx_2nd, *siny_2nd, sinx_count_2nd, siny_count_2nd);

	if (buf == 0x0) {
		return true;
	} else {
		return false;
	}
}

bool is_vendor_ois_auto_test_rear2(struct is_core *core,
					int threshold, bool *x_result, bool *y_result, int *sin_x, int *sin_y,
					bool *x_result_2nd, bool *y_result_2nd, int *sin_x_2nd, int *sin_y_2nd)
{
	int result = 0;
	bool value = false;

#ifdef CONFIG_AF_HOST_CONTROL
#if defined(CAMERA_2ND_OIS)
#ifdef USE_TELE_OIS_AF_COMMON_INTERFACE
	is_vendor_ois_af_move_lens(core);
#else
	is_af_move_lens(core, SENSOR_POSITION_REAR2);
#endif
	msleep(100);
#endif
#if defined(CAMERA_3RD_OIS)
#ifdef USE_TELE2_OIS_AF_COMMON_INTERFACE
	is_vendor_ois_af_move_lens(core);
#else
	is_af_move_lens(core, SENSOR_POSITION_REAR4);
#endif
	msleep(100);
#endif
	is_af_move_lens(core, SENSOR_POSITION_REAR);
	msleep(100);
#endif

	value = is_vendor_ois_sine_wavecheck_rear2(core, threshold, sin_x, sin_y, &result,
				sin_x_2nd, sin_y_2nd);

	if (*sin_x == -1 && *sin_y == -1) {
		err_mcu("OIS device is not prepared");
		*x_result = false;
		*y_result = false;

		return false;
	}

	if (*sin_x_2nd == -1 && *sin_y_2nd == -1) {
		err_mcu("OIS 2 device is not prepared");
		*x_result_2nd = false;
		*y_result_2nd = false;

		return false;
	}

	if (value == true) {
		*x_result = true;
		*y_result = true;
		*x_result_2nd = true;
		*y_result_2nd = true;

		return true;
	} else {
		err_mcu("OIS autotest_2nd is failed result (0x0051) = 0x%x\n", result);
		if ((result & 0x03) == 0x00) {
			*x_result = true;
			*y_result = true;
		} else if ((result & 0x03) == 0x01) {
			*x_result = false;
			*y_result = true;
		} else if ((result & 0x03) == 0x02) {
			*x_result = true;
			*y_result = false;
		} else {
			*x_result = false;
			*y_result = false;
		}

		if ((result & 0x30) == 0x00) {
			*x_result_2nd = true;
			*y_result_2nd = true;
		} else if ((result & 0x30) == 0x10) {
			*x_result_2nd = false;
			*y_result_2nd = true;
		} else if ((result & 0x30) == 0x20) {
			*x_result_2nd = true;
			*y_result_2nd = false;
		} else {
			*x_result_2nd = false;
			*y_result_2nd = false;
		}

		return false;
	}
}

int is_vendor_ois_set_power_mode(struct v4l2_subdev *subdev, int forceMode)
{
	struct is_ois *ois = NULL;
	struct ois_mcu_dev *mcu = NULL;
	struct is_mcu *is_mcu = NULL;
	bool camera_running;
	bool camera_running2;
#if defined(CAMERA_3RD_OIS)
	bool camera_running4;
#endif

	mcu = (struct ois_mcu_dev*)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err_mcu("%s, mcu subdev is NULL", __func__);
		return -EINVAL;
	}

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err_mcu("%s, is_mcu is NULL", __func__);
		return -EINVAL;
	}

	ois = is_mcu->ois;
	if (!ois) {
		err_mcu("%s, ois subdev is NULL", __func__);
		return -EINVAL;
	}

	info_mcu("%s : E\n", __func__);

	camera_running = is_vendor_check_camera_running(SENSOR_POSITION_REAR);
	camera_running2 = is_vendor_check_camera_running(SENSOR_POSITION_REAR2);
#if defined(CAMERA_3RD_OIS)
	camera_running4 = is_vendor_check_camera_running(SENSOR_POSITION_REAR4);
#endif

	OIS_LOCK(ois->ixc_lock);

	/* OIS SEL (wide : 1 , tele : 2, w/t : 3, tele2 : 4, triple : 7) */
#if defined(CAMERA_3RD_OIS)
	if (forceMode == OIS_USE_UW_ONLY) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x00);
		ois->ois_power_mode = OIS_POWER_MODE_NONE;
	} else if (camera_running && !camera_running2 && !camera_running4) { //TEMP_OLYMPUS ==> need to be changed based on camera scenario
#ifdef USE_TELE2_OIS_AF_COMMON_INTERFACE
		is_vendor_ois_set_sleep_mode_folded_zoom();
#endif
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x01);
		ois->ois_power_mode = OIS_POWER_MODE_SINGLE_WIDE;
		usleep_range(5000, 5010);
	} else if (!camera_running && camera_running2 && !camera_running4) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x02);
		ois->ois_power_mode = OIS_POWER_MODE_SINGLE_TELE;
	} else if (camera_running && camera_running2 && !camera_running4) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x03);
		ois->ois_power_mode = OIS_POWER_MODE_DUAL;
	} else if (!camera_running && !camera_running2 && camera_running4) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x04);
		ois->ois_power_mode = OIS_POWER_MODE_SINGLE_TELE2;
	} else {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x07);
		ois->ois_power_mode = OIS_POWER_MODE_TRIPLE;
	}
#else
	if (forceMode == OIS_USE_UW_ONLY) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x00);
		ois->ois_power_mode = OIS_POWER_MODE_NONE;
	} else if (forceMode == OIS_USE_UW_WIDE) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x01);
		ois->ois_power_mode = OIS_POWER_MODE_SINGLE_WIDE;
	} else if (camera_running && !camera_running2) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x01);
		ois->ois_power_mode = OIS_POWER_MODE_SINGLE_WIDE;
	} else if (!camera_running && camera_running2) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x02);
		ois->ois_power_mode = OIS_POWER_MODE_SINGLE_TELE;
	} else {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_OIS_SEL, 0x03);
		ois->ois_power_mode = OIS_POWER_MODE_DUAL;
	}
#endif
	OIS_UNLOCK(ois->ixc_lock);

	mcu->current_power_mode = ois->ois_power_mode;
	info_mcu("%s ois power setting is %d X\n", __func__, ois->ois_power_mode);

	return 0;
}
#endif /* CAMERA_2ND_OIS */

void is_vendor_ois_enable(struct is_core *core)
{
	struct ois_mcu_dev *mcu = NULL;

	mcu = core->mcu;

	info_mcu("%s : E\n", __func__);

	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE, 0x00);

	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x01);

	info_mcu("%s : X\n", __func__);
}

int is_vendor_ois_disable(struct v4l2_subdev *subdev)
{
	struct ois_mcu_dev *mcu = NULL;

	info_mcu("%s : E\n", __func__);

	mcu = (struct ois_mcu_dev*)v4l2_get_subdevdata(subdev);
	if(!mcu) {
		err_mcu("%s, mcu subdev is NULL", __func__);
		return -EINVAL;
	}

	if (mcu->ois_hw_check) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x00);
		usleep_range(2000, 2100);

		mcu->ois_fadeupdown = false;
		mcu->ois_hw_check = false;

		/* off all ois */
		mcu->ois_wide_init = false;
#if defined(CAMERA_2ND_OIS)
		mcu->ois_tele_init = false;
#endif
#if defined(CAMERA_3RD_OIS)
		mcu->ois_tele2_init = false;
#endif
		info_mcu("%s ois stop.X\n", __func__);
	}

	info_mcu("%s : X\n", __func__);

	return 0;
}

void is_vendor_ois_get_hall_position(struct is_core *core, u16 *targetPos, u16 *hallPos)
{
	struct ois_mcu_dev *mcu = NULL;
	u8 pos_temp[2] = {0, };
	u16 pos = 0;

	mcu = core->mcu;

	info_mcu("%s : E\n", __func__);

	is_vendor_ois_set_aois_fac_mode_on();

	/* set centering mode */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE, 0x05);

	/* enable position data read */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_FWINFO_CTRL, 0x01);

	msleep(150);

	pos_temp[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_TARGET_POS_REAR_X);
	pos_temp[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_TARGET_POS_REAR_X2);
	pos = (pos_temp[1] << 8) | pos_temp[0];
	targetPos[0] = pos;

	pos_temp[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_TARGET_POS_REAR_Y);
	pos_temp[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_TARGET_POS_REAR_Y2);
	pos = (pos_temp[1] << 8) | pos_temp[0];
	targetPos[1] = pos;

	pos_temp[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_POS_REAR_X);
	pos_temp[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_POS_REAR_X2);
	pos = (pos_temp[1] << 8) | pos_temp[0];
	hallPos[0] = pos;

	pos_temp[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_POS_REAR_Y);
	pos_temp[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_POS_REAR_Y2);
	pos = (pos_temp[1] << 8) | pos_temp[0];
	hallPos[1] = pos;

	info_mcu("%s : wide pos = 0x%04x, 0x%04x, 0x%04x, 0x%04x\n", __func__, targetPos[0], targetPos[1], hallPos[0], hallPos[1]);

#if defined(CAMERA_2ND_OIS)
	pos_temp[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_TARGET_POS_REAR2_X);
	pos_temp[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_TARGET_POS_REAR2_X2);
	pos = (pos_temp[1] << 8) | pos_temp[0];
	targetPos[2] = pos;

	pos_temp[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_TARGET_POS_REAR2_Y);
	pos_temp[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_TARGET_POS_REAR2_Y2);
	pos = (pos_temp[1] << 8) | pos_temp[0];
	targetPos[3] = pos;

	pos_temp[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_POS_REAR2_X);
	pos_temp[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_POS_REAR2_X2);
	pos = (pos_temp[1] << 8) | pos_temp[0];
	hallPos[2] = pos;

	pos_temp[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_POS_REAR2_Y);
	pos_temp[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_POS_REAR2_Y2);
	pos = (pos_temp[1] << 8) | pos_temp[0];
	hallPos[3] = pos;

	info_mcu("%s : tele pos = 0x%04x, 0x%04x, 0x%04x, 0x%04x\n", __func__, targetPos[2], targetPos[3], hallPos[2], hallPos[3]);
#endif
#if defined(CAMERA_3RD_OIS)
	pos_temp[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_TARGET_POS_REAR3_X);
	pos_temp[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_TARGET_POS_REAR3_X2);
	pos = (pos_temp[1] << 8) | pos_temp[0];
	targetPos[4] = pos;

	pos_temp[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_TARGET_POS_REAR3_Y);
	pos_temp[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_TARGET_POS_REAR3_Y2);
	pos = (pos_temp[1] << 8) | pos_temp[0];
	targetPos[5] = pos;

	pos_temp[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_POS_REAR3_X);
	pos_temp[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_POS_REAR3_X2);
	pos = (pos_temp[1] << 8) | pos_temp[0];
	hallPos[4] = pos;

	pos_temp[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_POS_REAR3_Y);
	pos_temp[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_POS_REAR3_Y2);
	pos = (pos_temp[1] << 8) | pos_temp[0];
	hallPos[5] = pos;

	info_mcu("%s : tele2 pos = 0x%04x, 0x%04x, 0x%04x, 0x%04x\n", __func__, targetPos[4], targetPos[5], hallPos[4], hallPos[5]);
#endif

	/* disable position data read */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_FWINFO_CTRL, 0x00);

	is_vendor_ois_set_aois_fac_mode_off();

	info_mcu("%s : X\n", __func__);
}

bool is_vendor_ois_offset_test(struct is_core *core, long *raw_data_x, long *raw_data_y, long *raw_data_z)
{
	int i = 0;
	u8 val = 0, x = 0, y = 0, z = 0;
	int x_sum = 0, y_sum = 0, z_sum = 0, sum = 0;
	int retries = 0, avg_count = 30;
	bool result = false;
	int scale_factor = OIS_GYRO_SCALE_FACTOR;
	struct ois_mcu_dev *mcu = NULL;

	mcu = core->mcu;

	info_mcu("%s : E\n", __func__);
	is_vendor_ois_set_aois_fac_mode_on();

	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_CAL, 0x01);

	retries = avg_count;
	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_CAL);
		msleep(50);
		if (--retries < 0) {
			err_mcu("Read register failed!!!!, data = 0x%04x\n", val);
#ifdef USE_OIS_DEBUGGING_LOG
			ois_mcu_debug_log(mcu);
#endif
			break;
		}
	} while (val);

	/* Gyro result check */
	val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_ERROR_STATUS);

	if ((val & 0x63) == 0x0) {
		info_mcu("[%s] Gyro result check success. Result is OK. gyro value = 0x%02x", __func__, val);
		result = true;
	} else {
		info_mcu("[%s] Gyro result check fail. Result is NG. gyro value = 0x%02x", __func__, val);
		result = false;
	}

	sum = 0;
	retries = avg_count;
	for (i = 0; i < retries; retries--) {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_X1);
		x = val;
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_X2);
		x_sum = (val << 8) | x;
		if (x_sum > 0x7FFF) {
			x_sum = -((x_sum ^ 0xFFFF) + 1);
		}
		sum += x_sum;
	}
	sum = sum * 10 / avg_count;
	*raw_data_x = sum * 1000 / scale_factor / 10;

	sum = 0;
	retries = avg_count;
	for (i = 0; i < retries; retries--) {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_Y1);
		y = val;
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_Y2);
		y_sum = (val << 8) | y;
		if (y_sum > 0x7FFF) {
			y_sum = -((y_sum ^ 0xFFFF) + 1);
		}
		sum += y_sum;
	}
	sum = sum * 10 / avg_count;
	*raw_data_y = sum * 1000 / scale_factor / 10;

	sum = 0;
	retries = avg_count;
	for (i = 0; i < retries; retries--) {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_Z1);
		z = val;
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_Z2);
		z_sum = (val << 8) | z;
		if (z_sum > 0x7FFF) {
			z_sum = -((z_sum ^ 0xFFFF) + 1);
		}
		sum += z_sum;
	}
	sum = sum * 10 / avg_count;
	*raw_data_z = sum * 1000 / scale_factor / 10;

	is_vendor_ois_set_aois_fac_mode_off();
	//is_mcu_fw_version(core); // TEMP_2020
	info_mcu("%s : X raw_x = %ld, raw_y = %ld, raw_z = %ld\n", __func__, *raw_data_x, *raw_data_y, *raw_data_z);

	return result;
}

void is_vendor_ois_get_offset_data(struct is_core *core, long *raw_data_x, long *raw_data_y, long *raw_data_z)
{
	u8 val = 0;
	int retries = 0, avg_count = 40;
	struct ois_mcu_dev *mcu = NULL;

	mcu = core->mcu;

	info_mcu("%s : E\n", __func__);

	/* check ois status */
	retries = avg_count;
	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
		msleep(50);
		if (--retries < 0) {
			err_mcu("%s Read status failed!!!!, data = 0x%04x", __func__, val);
			break;
		}
	} while (val != 0x01);

	is_vendor_ois_get_efs_data(mcu, raw_data_x, raw_data_y, raw_data_z);

	return;
}

void is_vendor_ois_gyro_sleep(struct is_core *core)
{
	u8 val = 0;
	int retries = 20;
	struct ois_mcu_dev *mcu = NULL;

	mcu = core->mcu;

	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x00);

	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);

		if (val == 0x01 || val == 0x13)
			break;

		usleep_range(1000, 1100);
	} while (--retries > 0);

	if (retries <= 0) {
		err_mcu("Read register failed!!!!, data = 0x%04x\n", val);
	}

	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_SLEEP, 0x03);
	usleep_range(1000, 1100);

	return;
}

void is_vendor_ois_exif_data(struct is_core *core)
{
	u8 error_reg[2], status_reg;
	u16 error_sum;
	struct ois_mcu_dev *mcu = NULL;
	 struct is_ois_exif *ois_exif_data = NULL;

	mcu = core->mcu;

	is_ois_get_exif_data(&ois_exif_data);

	error_reg[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_ERROR_STATUS);
	error_reg[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CHECKSUM);

	error_sum = (error_reg[1] << 8) | error_reg[0];

	status_reg = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);

	ois_exif_data->error_data = error_sum;
	ois_exif_data->status_data = status_reg;

	return;
}

u8 is_vendor_ois_read_status(struct is_core *core)
{
	u8 status = 0;
	struct ois_mcu_dev *mcu = NULL;

	mcu = core->mcu;

	status = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_READ_STATUS);

	return status;
}

u8 is_vendor_ois_read_cal_checksum(struct is_core *core)
{
	u8 status = 0;
	struct ois_mcu_dev *mcu = NULL;

	mcu = core->mcu;

	status = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CHECKSUM);

	return status;
}

int is_vendor_ois_set_coef(struct v4l2_subdev *subdev, u8 coef)
{
	int ret = 0;
	struct is_ois *ois = NULL;
	struct ois_mcu_dev *mcu = NULL;
	struct is_mcu *is_mcu = NULL;

	WARN_ON(!subdev);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err_mcu("%s, is_mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err_mcu("%s, mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	if (!mcu->ois_wide_init
#if defined(CAMERA_2ND_OIS)
		&& !mcu->ois_tele_init
#endif
#if defined(CAMERA_3RD_OIS)
		&& !mcu->ois_tele2_init
#endif
	)
		return 0;

	ois = is_mcu->ois;

	if (ois->pre_coef == coef)
		return ret;

	dbg_ois("%s %d\n", __func__, coef);

	OIS_LOCK(ois->ixc_lock);
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_SET_COEF, coef);
	OIS_UNLOCK(ois->ixc_lock);

	ois->pre_coef = coef;

	return ret;
}

void is_vendor_ois_set_center_shift(struct v4l2_subdev *subdev, int16_t *shiftValue)
{
	int i = 0;
	int j = 0;
	u8 data[2];
	struct is_ois *ois = NULL;
	struct ois_mcu_dev *mcu = NULL;
	struct is_mcu *is_mcu = NULL;

	WARN_ON(!subdev);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err_mcu("%s, is_mcu is NULL", __func__);
		return;
	}

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err_mcu("%s, mcu is NULL", __func__);
		return;
	}

	ois = is_mcu->ois;

	info_mcu("%s wide x = %hd, wide y = %hd, tele x = %hd, tele y = %hd, tele2 x = %hd, tele2 y = %hd",
		__func__, shiftValue[0], shiftValue[1], shiftValue[2], shiftValue[3], shiftValue[4], shiftValue[5]);

	OIS_LOCK(ois->ixc_lock);
	for (i = 0; i < 6; i++) {
		if (shiftValue[i]) {
			data[0] = shiftValue[i] & 0xFF;
			data[1] = (shiftValue[i] >> 8) & 0xFF;
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_XCOEF_M1_1 + j++, data[0]);
			is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_XCOEF_M1_1 + j++, data[1]);
		} else {
			j += 2;
		}
	}
	OIS_UNLOCK(ois->ixc_lock);
}

int is_vendor_ois_set_centering(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct is_ois *ois = NULL;
	struct ois_mcu_dev *mcu = NULL;
	struct is_mcu *is_mcu = NULL;

	WARN_ON(!subdev);

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if(!mcu) {
		err_mcu("%s, mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if(!is_mcu) {
		err_mcu("%s, is_mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	ois = is_mcu->ois;

	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE, 0x05);

	ois->pre_ois_mode = OPTICAL_STABILIZATION_MODE_CENTERING;

	return ret;
}

u8 is_vendor_ois_read_mode(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u8 mode = OPTICAL_STABILIZATION_MODE_OFF;
	struct ois_mcu_dev *mcu = NULL;
	struct is_mcu *is_mcu = NULL;

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if(!mcu) {
		err_mcu("%s, mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if(!is_mcu) {
		err_mcu("%s, is_mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	mode = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_MODE);

	switch(mode) {
		case 0x00:
			mode = OPTICAL_STABILIZATION_MODE_STILL;
			break;
		case 0x01:
			mode = OPTICAL_STABILIZATION_MODE_VIDEO;
			break;
		case 0x05:
			mode = OPTICAL_STABILIZATION_MODE_CENTERING;
			break;
		case 0x13:
			mode = OPTICAL_STABILIZATION_MODE_STILL_ZOOM;
			break;
		case 0x14:
			mode = OPTICAL_STABILIZATION_MODE_VDIS;
			break;
		default:
			dbg_ois("%s: ois_mode value(%d)\n", __func__, mode);
			break;
	}

	return mode;
}

bool is_vendor_ois_gyro_cal(struct is_core *core, long *x_value, long *y_value, long *z_value)
{
	u8 val = 0, x = 0, y = 0, z = 0;
	int retries = 30;
	int scale_factor = OIS_GYRO_SCALE_FACTOR;
	int x_sum = 0, y_sum = 0, z_sum = 0;
	bool result = false;
	struct ois_mcu_dev *mcu = NULL;

	mcu = core->mcu;

	info_mcu("%s : E\n", __func__);

	is_vendor_ois_set_aois_fac_mode_on();

	/* check ois status */
	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
		msleep(20);
		if (--retries < 0) {
			err_mcu("%s Read status failed!!!!, data = 0x%04x", __func__, val);
#ifdef USE_OIS_DEBUGGING_LOG
			ois_mcu_debug_log(mcu);
#endif
			is_vendor_ois_set_aois_fac_mode_off();
			return false;
		}
	} while (val != 0x01);

	retries = 30;

	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_CAL, 0x01);

	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_GYRO_CAL);
		msleep(15);
		if (--retries < 0) {
			err_mcu("Read register failed!!!!, data = 0x%04x\n", val);
#ifdef USE_OIS_DEBUGGING_LOG
			ois_mcu_debug_log(mcu);
#endif
			break;
		}
	} while (val);

	/* Gyro result check */
	val= is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_ERROR_STATUS);

	if ((val & 0x63) == 0x0) {
		info_mcu("[%s] Written cal is OK. val = 0x%02x", __func__, val);
		result = true;
	} else {
		info_mcu("[%s] Written cal is NG. val = 0x%02x", __func__, val);
		result = false;
	}

	val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_X1);
	x = val;
	val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_X2);
	x_sum = (val << 8) | x;
	if (x_sum > 0x7FFF) {
		x_sum = -((x_sum ^ 0xFFFF) + 1);
	}

	val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_Y1);
	y = val;
	val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_Y2);
	y_sum = (val << 8) | y;
	if (y_sum > 0x7FFF) {
		y_sum = -((y_sum ^ 0xFFFF) + 1);
	}

	val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_Z1);
	z = val;
	val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_RAW_DEBUG_Z2);
	z_sum = (val << 8) | z;
	if (z_sum > 0x7FFF) {
		z_sum = -((z_sum ^ 0xFFFF) + 1);
	}

	*x_value = x_sum * 1000 / scale_factor;
	*y_value = y_sum * 1000 / scale_factor;
	*z_value = z_sum * 1000 / scale_factor;

	is_vendor_ois_set_aois_fac_mode_off();

	info_mcu("%s X (x = %ld/y = %ld/z = %ld) : result = %d\n", __func__, *x_value, *y_value, *z_value, result);

	return result;
}

bool is_vendor_ois_read_gyro_noise(struct is_core *core, long *x_value, long *y_value)
{
	u8 val = 0, x = 0, y = 0;
	int retries = 30;
	int scale_factor = OIS_GYRO_SCALE_FACTOR;
	int x_sum = 0, y_sum = 0;
	bool result = true;
	struct ois_mcu_dev *mcu = NULL;

	mcu = core->mcu;

	info_mcu("%s : E\n", __func__);

	msleep(500);

	is_vendor_ois_set_aois_fac_mode_on();

	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_START, 0x00);
	usleep_range(1000, 1100);

	/* check ois status */
	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
		msleep(20);
		if (--retries < 0) {
			err_mcu("%s Read status failed!!!!, data = 0x%04x", __func__, val);
			result = false;
			break;
		}
	} while (val != 0x01);

	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_SET_GYRO_NOISE, 0x01);

	msleep(1000);

	val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_READ_GYRO_NOISE_X1);
	x = val;
	val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_READ_GYRO_NOISE_X2);
	x_sum = (val << 8) | x;
	if (x_sum > 0x7FFF) {
		x_sum = -((x_sum ^ 0xFFFF) + 1);
	}

	val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_READ_GYRO_NOISE_Y1);
	y = val;
	val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_READ_GYRO_NOISE_Y2);
	y_sum = (val << 8) | y;
	if (y_sum > 0x7FFF) {
		y_sum = -((y_sum ^ 0xFFFF) + 1);
	}

	*x_value = x_sum * 1000 / scale_factor;
	*y_value = y_sum * 1000 / scale_factor;

	info_mcu("%s X (x = %ld/y = %ld) : result = %d\n", __func__, *x_value, *y_value, result);

	is_vendor_ois_set_aois_fac_mode_off();

	return result;
}

#ifdef USE_OIS_HALL_DATA_FOR_VDIS
int is_vendor_ois_get_hall_data(struct v4l2_subdev *subdev, struct is_ois_hall_data *halldata)
{
	int ret = 0;
	struct ois_mcu_dev *mcu = NULL;
	struct is_ois *ois = NULL;
	struct is_mcu *is_mcu = NULL;
	u8 val[4] = {0, };
	u64 timeStamp = 0;
	int val_sum = 0;
	int max_cnt = 192;
	int index = 0;
	int i = 0;
	int valid_cnt = 0;
	u8 valid_num = 0;
	u64 prev_timestampboot = timestampboot;

	WARN_ON(!subdev);

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err("%s, mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err_mcu("%s, is_mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	ois = is_mcu->ois;
	OIS_LOCK(ois->ixc_lock);

	/* SVDIS CTRL READ HALLDATA */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_SVDIS_CTRL, 0x02);
	OIS_UNLOCK(ois->ixc_lock);
	usleep_range(150, 160);

#if defined(CONFIG_CAMERA_USE_INTERNAL_MCU)
	/* S/W interrupt to MCU */
	is_mcu_hw_set_field(mcu->regs[OM_REG_SFR], OIS_CM0P_IRQ, F_OIS_CM0P_IRQ_REQ, 0x01);
#endif
	usleep_range(200, 210);

	/* get current AP time stamp (read irq timing) */
	timestampboot = ktime_get_boottime_ns();
	OIS_LOCK(ois->ixc_lock);

	val[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_TIME_STAMP1);
	val[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_TIME_STAMP2);
	val[2] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_TIME_STAMP3);
	val[3] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_TIME_STAMP4);
	timeStamp =  ((uint64_t)val[3] << 24) | ((uint64_t)val[2] << 16) | ((uint64_t)val[1] << 8) | (uint64_t)val[0];
	halldata->timeStamp = prev_timestampboot + (timeStamp * 1000);

	valid_num = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_VALID_NUMBER);
	halldata->validData = valid_num;

	valid_cnt = (int)valid_num * 8;
	if (valid_cnt > max_cnt) {
		valid_cnt = max_cnt;
	}

	/* Wide data */
	for (i = 0; i < valid_cnt; i += 8) {
		val[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_WIDE_X_ANG_0 + i);
		val[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_WIDE_X_ANG_0 + i + 1);
		val_sum = (val[1] << 8) | val[0];
		halldata->xAngleWide[index] = val_sum;

		val[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_WIDE_Y_ANG_0 + i);
		val[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_WIDE_Y_ANG_0 + i + 1);
		val_sum = (val[1] << 8) | val[0];
		halldata->yAngleWide[index] = val_sum;

		index++;

		if (index >= NUM_OF_HALLDATA_AT_ONCE)
			break;
	}

#if defined(CAMERA_2ND_OIS)
	/* Tele data */
	index = 0;

	for (i = 0; i < valid_cnt; i += 8) {
		val[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_TELE_X_ANG_0 + i);
		val[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_TELE_X_ANG_0 + i + 1);
		val_sum = (val[1] << 8) | val[0];
		halldata->xAngleTele[index] = val_sum;

		val[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_TELE_Y_ANG_0 + i);
		val[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_TELE_Y_ANG_0 + i + 1);
		val_sum = (val[1] << 8) | val[0];
		halldata->yAngleTele[index] = val_sum;

		index++;

		if (index >= NUM_OF_HALLDATA_AT_ONCE)
			break;
	}
#endif

#if defined(CAMERA_3RD_OIS)
	/* Tele2 data */
	index = 0;

	for (i = 0; i < valid_cnt; i += 8) {
		val[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_TELE2_X_ANG_0 + i);
		val[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_TELE2_X_ANG_0 + i + 1);
		val_sum = (val[1] << 8) | val[0];
		halldata->xAngleTele2[index] = val_sum;

		val[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_TELE2_Y_ANG_0 + i);
		val[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_HALL_TELE2_Y_ANG_0 + i + 1);
		val_sum = (val[1] << 8) | val[0];
		halldata->yAngleTele2[index] = val_sum;

		index++;

		if (index >= NUM_OF_HALLDATA_AT_ONCE)
			break;
	}
#endif
	OIS_UNLOCK(ois->ixc_lock);

	OIS_LOCK(ois->ixc_lock);
	/* delay between write irq & read irq */
	usleep_range(250, 260);
	OIS_UNLOCK(ois->ixc_lock);

	/* SVDIS CTRL WRITE TIMESTAMP */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_SVDIS_CTRL, 0x01);
	usleep_range(300, 310);

#if defined(CONFIG_CAMERA_USE_INTERNAL_MCU)
	/* S/W interrupt to MCU */
	is_mcu_hw_set_field(mcu->regs[OM_REG_SFR], OIS_CM0P_IRQ, F_OIS_CM0P_IRQ_REQ, 0x01);
#endif
	return ret;
}
#endif

void is_vendor_ois_check_valid(struct v4l2_subdev *subdev, u8 *value)
{
	struct ois_mcu_dev *mcu = NULL;
	u8 error_reg[2] = {0, };

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if(!mcu) {
		err_mcu("%s, mcu is NULL", __func__);
		return;
	}

	is_vendor_ois_init_factory(subdev);

	is_vendor_ois_set_aois_fac_mode_on();

	error_reg[0] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_ERROR_STATUS);
	error_reg[1] = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CHECKSUM);
	is_vendor_ois_check_errors(error_reg[0], error_reg[1]);

	*value = error_reg[1];
	is_vendor_ois_set_aois_fac_mode_off();
	return;
}

bool is_vendor_ois_get_active(struct v4l2_subdev *subdev)
{
	struct ois_mcu_dev *mcu = NULL;

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if(!mcu) {
		err_mcu("%s, mcu is NULL", __func__);
		return false;
	}

	return mcu->ois_hw_check;
}
