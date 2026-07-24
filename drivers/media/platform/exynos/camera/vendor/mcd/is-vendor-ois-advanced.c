// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos SoC series Pablo driver
 *
 * Exynos Pablo image subsystem functions
 *
 * Copyright (c) 2023 Samsung Electronics Co., Ltd
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
#include "is-interface.h"
#include "is-sec-define.h"
#include "is-device-ischain.h"
#include "is-dt.h"
#include "is-device-ois_common.h"
#include "is-vendor-ois-core.h"
#include "is-vendor-ois.h"
#include "is-vendor-ois-reg.h"
#include "is-vendor-private.h"
#ifdef CONFIG_AF_HOST_CONTROL
#include "is-device-af.h"
#endif
#include <linux/pinctrl/pinctrl.h>
#include "is-core.h"
#include "is-vendor-ois-advanced.h"
#include "is-interface-aois.h"
#include "is-ixc-config.h"

#define AOIS_NAME "Advanced OIS"

int is_ois_advanced_read_u8(int cmd, u8 *data) {
	int ret = 0;
	u8 rxbuf[1];

	ret = cam_ois_reg_read_notifier_call_chain(0, ois_mcu_regs[cmd].sfr_offset, &rxbuf[0], 1);
	*data = rxbuf[0];

	dbg_ois("[GET_REG] reg:[%s][0x%04X], reg_value(R):[0x%02X]\n",
		ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset, *data);

	if (unlikely(ret != 2)) {
		err_mcu("get fail (%s:%X)", ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset);
		return -EIO;
	}

	return ret;
}

int is_ois_advanced_read_multi(int cmd, u8 *data, size_t size)
{
	int i;

	u8 rxbuf[256];
	int ret = 0;

	ret = cam_ois_reg_read_notifier_call_chain(0, ois_mcu_regs[cmd].sfr_offset, rxbuf, size);
	memcpy(data, rxbuf, size);

	for (i = 0; i < size; i++) {
		dbg_ois("[GET_REG] reg:[%s][0x%04X], reg_value(R):[0x%02X]\n",
			ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset + i, data[i]);
	}

	if (unlikely(ret != 2)) {
		err_mcu("get multi fail (%s:%X)", ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset);
		return -EIO;
	}

	return ret;
}

int is_ois_advanced_read_u16(int cmd, u8 *data)
{
	return is_ois_advanced_read_multi(cmd, data, 2);
}

int is_ois_advanced_write_u8(int cmd, u8 data)
{
	int ret = 0;

	ret = cam_ois_cmd_notifier_call_chain(0, ois_mcu_regs[cmd].sfr_offset, &data, 1);

	dbg_ois("[SET_REG] reg:[%s][0x%04X], reg_value(W):[0x%02X]\n",
		ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset, data);

	if (unlikely(ret != 1)) {
		err_mcu("set fail (%s:%X)", ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset);
		return -EIO;
	}

	return ret;
}

int is_ois_advanced_write_multi(int cmd, u8 *data, size_t size)
{
	int ret = 0;
	int i;

	ret = cam_ois_cmd_notifier_call_chain(0, ois_mcu_regs[cmd].sfr_offset, data, size);

	for (i = 0 ; i < size; i++) {
		dbg_ois("[SET_REG] reg:[%s][0x%04X], reg_value(W):[0x%02X]\n",
			ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset + i, data[i]);
	}

	if (unlikely(ret != 1)) {
		err_mcu("set multi fail (%s:%X)", ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset);
		return -EIO;
	}

	return ret;
}

int is_ois_advanced_write_u16(int cmd, u8 *data)
{
	return is_ois_advanced_write_multi(cmd, data, 2);
}

static struct ois_comm_ops advanced_ois_ops = {
	.read_u8 = is_ois_advanced_read_u8,
	.read_u16 = is_ois_advanced_read_u16,
	.read_multi = is_ois_advanced_read_multi,
	.write_u8 = is_ois_advanced_write_u8,
	.write_u16 = is_ois_advanced_write_u16,
	.write_multi = is_ois_advanced_write_multi,
};

bool is_mcu_fw_version(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u8 hwver[4] = {0, };
	u8 vdrinfo[4] = {0, };
	struct is_mcu *is_mcu = NULL;
	struct is_ois_info *ois_minfo = NULL;
	u16 reg;

	info("%s started", __func__);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("is_mcu is NULL");
		return -EINVAL;
	}

	reg = OIS_CMD_HW_VERSION;
	ret = ois_read_multi(reg, &hwver[0], 4);
	if (ret) {
		MCU_GET_ERR_PRINT(reg);
		goto exit;
	}

	reg = OIS_CMD_VDR_VERSION;
	ret = ois_read_multi(reg, &vdrinfo[0], 4);
	if (ret) {
		MCU_GET_ERR_PRINT(reg);
		goto exit;
	}

	is_ois_get_module_version(&ois_minfo);

	memcpy(&is_mcu->vdrinfo_mcu[0], &vdrinfo[0], 4);
	is_mcu->hw_mcu[0] = hwver[3];
	is_mcu->hw_mcu[1] = hwver[2];
	is_mcu->hw_mcu[2] = hwver[1];
	is_mcu->hw_mcu[3] = hwver[0];
	memcpy(ois_minfo->header_ver, &is_mcu->hw_mcu[0], 4);
	memcpy(&ois_minfo->header_ver[4], &vdrinfo[0], 4);

	info("[%s] mcu module hw ver = %c%c%c%c, vdrinfo ver = %c%c%c%c", __func__,
		hwver[3], hwver[2], hwver[1], hwver[0], vdrinfo[0], vdrinfo[1], vdrinfo[2], vdrinfo[3]);

	return true;

exit:
	return false;
}

int is_mcu_fw_revision_vdrinfo(u8 *fw_ver)
{
	int revision = 0;
	revision = revision + ((int)fw_ver[FW_RELEASE_YEAR] - 58) * 10000;
	revision = revision + ((int)fw_ver[FW_RELEASE_MONTH] - 64) * 100;
	revision = revision + ((int)fw_ver[FW_RELEASE_COUNT] - 48) * 10;
	revision = revision + (int)fw_ver[FW_RELEASE_COUNT + 1] - 48;

	return revision;
}

bool is_mcu_version_compare(u8 *fw_ver1, u8 *fw_ver2)
{
	if (fw_ver1[FW_DRIVER_IC] != fw_ver2[FW_DRIVER_IC]
		|| fw_ver1[FW_GYRO_SENSOR] != fw_ver2[FW_GYRO_SENSOR]
		|| fw_ver1[FW_MODULE_TYPE] != fw_ver2[FW_MODULE_TYPE]
		|| fw_ver1[FW_PROJECT] != fw_ver2[FW_PROJECT]) {
		return false;
	}

	return true;
}

void is_advanced_ois_fw_update(struct is_core *core)
{
	int ret = 0;
	int vdrinfo_bin = 0;
	int vdrinfo_mcu = 0;

	struct is_mcu *is_mcu = NULL;
	struct v4l2_subdev *subdev = NULL;

	is_mcu = is_ois_get_mcu(core);
	subdev = is_mcu->subdev;

	info("%s started", __func__);

	msleep(30);

	ret = is_mcu_fw_version(subdev);
	if (ret) {
#ifdef CONFIG_CHECK_HW_VERSION_FOR_MCU_FW_UPLOAD
		int isUpload = 0;

		if (!is_mcu_version_compare(is_mcu->hw_bin, is_mcu->hw_mcu))
			isUpload = 1;

		info("HW binary ver = %c%c%c%c, module ver = %c%c%c%c",
			is_mcu->hw_bin[0], is_mcu->hw_bin[1], is_mcu->hw_bin[2], is_mcu->hw_bin[3],
			is_mcu->hw_mcu[0], is_mcu->hw_mcu[1], is_mcu->hw_mcu[2], is_mcu->hw_mcu[3]);

		vdrinfo_bin = is_mcu_fw_revision_vdrinfo(is_mcu->vdrinfo_bin);
		vdrinfo_mcu = is_mcu_fw_revision_vdrinfo(is_mcu->vdrinfo_mcu);

		if (vdrinfo_bin > vdrinfo_mcu)
			isUpload = 1;

		info("VDRINFO binary ver = %c%c%c%c, module ver = %c%c%c%c",
			is_mcu->vdrinfo_bin[0], is_mcu->vdrinfo_bin[1], is_mcu->vdrinfo_bin[2], is_mcu->vdrinfo_bin[3],
			is_mcu->vdrinfo_mcu[0], is_mcu->vdrinfo_mcu[1], is_mcu->vdrinfo_mcu[2], is_mcu->vdrinfo_mcu[3]);

		if (isUpload)
			info("Update MCU firmware!!");
		else {
			info("Do not update MCU firmware");
		}
#else
		if (!is_mcu_version_compare(is_mcu->hw_bin, is_mcu->hw_mcu)) {
			info("Do not update MCU firmware. HW binary ver = %c%c%c%c, module ver = %c%c%c%c",
				is_mcu->hw_bin[0], is_mcu->hw_bin[1], is_mcu->hw_bin[2], is_mcu->hw_bin[3],
				is_mcu->hw_mcu[0], is_mcu->hw_mcu[1], is_mcu->hw_mcu[2], is_mcu->hw_mcu[3]);
		}

		vdrinfo_bin = is_mcu_fw_revision_vdrinfo(is_mcu->vdrinfo_bin);
		vdrinfo_mcu = is_mcu_fw_revision_vdrinfo(is_mcu->vdrinfo_mcu);

		if (vdrinfo_bin <= vdrinfo_mcu) {
			info("Do not update MCU firmware. VDRINFO binary ver = %c%c%c%c, module ver = %c%c%c%c",
				is_mcu->vdrinfo_bin[0], is_mcu->vdrinfo_bin[1], is_mcu->vdrinfo_bin[2], is_mcu->vdrinfo_bin[3],
				is_mcu->vdrinfo_mcu[0], is_mcu->vdrinfo_mcu[1], is_mcu->vdrinfo_mcu[2], is_mcu->vdrinfo_mcu[3]);
		}
#endif
	}

	msleep(50);

	info("%s end", __func__);
}

int is_mcu_set_aperture(struct v4l2_subdev *subdev, int onoff)
{
	int ret = 0;
	u8 data = 0;
	int retry = 5;
	int value = 0;
	struct ois_mcu_dev *mcu = NULL;
	struct is_mcu *is_mcu = NULL;

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err("%s, mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("%s, is_mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	info("%s started onoff = %d", __func__, onoff);

	switch (onoff) {
	case F1_5:
		value = 2;
		break;
	case F2_4:
		value = 1;
		break;
	default:
		info("%s: mode is not set.(mode = %d)\n", __func__, onoff);
		is_mcu->aperture->step = APERTURE_STEP_STATIONARY;
		goto exit;
	}

	/* wait control register to idle */
	do {
		ret = ois_read_u8(0x61, &data);
		if (ret) {
			err("i2c read fail\n");
			goto exit;
		}

		if (retry-- < 0) {
			err("wait idle state failed..");
			break;
		}
	} while (data);

	info("mcu status = %d", data);

	ret = ois_write_u8(0x63, value);
	if (ret) {
		err("i2c read fail\n");
		goto exit;
	}

	/* start aperture control */
	ret = ois_write_u8(0x61, 0x01);
	if (ret) {
		err("i2c read fail\n");
		goto exit;
	}

	if (value == 2)
		is_mcu->aperture->cur_value = F1_5;
	else if (value == 1)
		is_mcu->aperture->cur_value = F2_4;

	is_mcu->aperture->step = APERTURE_STEP_STATIONARY;

	msleep(mcu->aperture_delay_list[0]);

	return true;

exit:
	info("%s Do not set aperture. onoff = %d", __func__, onoff);

	return false;
}

int is_mcu_deinit_aperture(struct v4l2_subdev *subdev, int onoff)
{
	int ret = 0;
	u8 data = 0;
	int retry = 5;
	struct ois_mcu_dev *mcu = NULL;
	struct is_mcu *is_mcu = NULL;

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err("%s, mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("%s, is_mcu is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	info("%s started onoff = %d", __func__, onoff);

	/* wait control register to idle */
	do {
		ret = ois_read_u8(0x61, &data);
		if (ret) {
			err("i2c read fail\n");
			goto exit;
		}

		if (retry-- < 0) {
			err("wait idle state failed..");
			break;
		}
	} while (data);

	info("mcu status = %d", data);

	ret = ois_write_u8(0x63, 0x2);
	if (ret) {
		err("i2c read fail\n");
		goto exit;
	}

	/* start aperture control */
	ret = ois_write_u8(0x61, 0x01);
	if (ret) {
		err("i2c read fail\n");
		goto exit;
	}

	is_mcu->aperture->cur_value = F1_5;

	msleep(mcu->aperture_delay_list[0]);

	return true;

exit:
	return false;
}

void is_mcu_set_aperture_onboot(struct is_core *core)
{
	int ret = 0;
	u8 data = 0;
	int retry = 5;
	struct ois_mcu_dev *mcu = NULL;
	struct is_mcu *is_mcu = NULL;
	struct v4l2_subdev *subdev = NULL;
	struct is_device_sensor *device = NULL;

	info("%s : E\n", __func__);

	is_mcu = is_ois_get_mcu(core);
	subdev = is_mcu->subdev;
	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err("%s, mcu is NULL", __func__);
		return;
	}

	device = &core->sensor[0];

	if (!device->mcu || !device->mcu->aperture) {
		err("%s, ois subdev is NULL", __func__);
		return;
	}

	/* wait control register to idle */
	do {
		ret = ois_read_u8(0x61, &data);
		if (ret) {
			err("i2c read fail\n");
		}

		if (retry-- < 0) {
			err("wait idle state failed..");
			break;
		}
	} while (data);

	info("mcu status = %d", data);

	ret = ois_write_u8(0x63, 0x2);
	if (ret) {
		err("i2c read fail\n");
	}

	/* start aperture control */
	ret = ois_write_u8(0x61, 0x01);
	if (ret) {
		err("i2c read fail\n");
	}

	device->mcu->aperture->cur_value = F1_5;

	msleep(mcu->aperture_delay_list[1]);

	info("%s : X\n", __func__);
}

bool is_mcu_halltest_aperture(struct v4l2_subdev *subdev, u16 *hall_value)
{
	int ret = 0;
	u8 data = 0;
	u8 data_array[2] = {0, };
	int retry = 3;
	bool result = true;
	struct is_mcu *is_mcu = NULL;

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("%s, is_mcu is NULL", __func__);
		return false;
	}

	info("%s started hall check", __func__);

	/* wait control register to idle */
	do {
		ret = ois_read_u8(0x61, &data);
		if (ret) {
			err("i2c read fail\n");
			result = false;
			goto exit;
		}

		if (retry-- < 0) {
			err("wait idle state failed..");
			result = false;
			goto exit;
		}

		msleep(15);
	} while (data);

	info("mcu status = %d", data);

	ret = ois_write_u8(0x61, 0x10);
	if (ret) {
		err("i2c read fail\n");
		result = false;
		goto exit;
	}

	/* wait control register to idle */
	retry = 3;

	do {
		ret = ois_read_u8(0x61, &data);
		if (ret) {
			err("i2c read fail\n");
			result = false;
			goto exit;
		}

		if (retry-- < 0) {
			err("wait idle state failed..");
			result = false;
			goto exit;
		}

		msleep(5);
	} while (data);

	ret = ois_read_u16(0x002C, data_array);
	if (ret) {
		err("i2c read fail\n");
		result = false;
		goto exit;
	}

	*hall_value = (data_array[1] << 8) | data_array[0];

exit:
	info("%s aperture mode = %d, hall_value = 0x%04x, result = %d",
		__func__, is_mcu->aperture->cur_value, *hall_value, result);

	return result;
}

signed long long hex2float_kernel(unsigned int hex_data, int endian)
{
	const signed long long scale = SCALE;
	unsigned int s,e,m;
	signed long long res;

	if (endian == eBIG_ENDIAN)
		hex_data = SWAP32(hex_data);

	s = hex_data >> 31, e = (hex_data >> 23) & 0xff, m = hex_data & 0x7fffff;
	res = (e >= 150) ? ((scale * (8388608 + m)) << (e - 150)) : ((scale * (8388608 + m)) >> (150 - e));
	if (s == 1)
		res *= -1;

	return res;
}

void is_status_check_mcu(void)
{
	u8 ois_status_check = 0;
	int retry_count = 0;

	do {
		ois_read_u8(0x000E, &ois_status_check);
		if (ois_status_check == 0x14)
			break;
		usleep_range(1000,1000);
		retry_count++;
	} while (retry_count < OIS_MEM_STATUS_RETRY);

	if (retry_count == OIS_MEM_STATUS_RETRY)
		err("%s, ois status check fail, retry_count(%d)\n", __func__, retry_count);

	if (ois_status_check == 0)
		err("%s, ois Memory access fail\n", __func__);
}

bool is_advanced_ois_check_fw(struct is_core *core)
{
	int ret = 0;
	struct is_mcu *is_mcu = NULL;
	struct v4l2_subdev *subdev = NULL;

	is_mcu = is_ois_get_mcu(core);
	subdev = is_mcu->subdev;

	is_advanced_ois_fw_update(core);

	msleep(20);

	ret = is_mcu_fw_version(subdev);
	if (!ret) {
		err("Failed to read ois fw version.");
		return false;
	}

	return true;
}

#ifdef CONFIG_SENSORCORE_MCU_CONTROL
void is_ois_reset_mcu(void *ois_core)
{
	int ret = 0;
	struct is_core *core = (struct is_core *)ois_core;
	struct is_device_sensor *device = NULL;
	struct is_ois_info *ois_pinfo = NULL;
	bool camera_running = false;
	u16 reg;

	info("%s : E\n", __func__);

	device = &core->sensor[0];

	if (!device->mcu || !device->mcu->ois) {
		err("%s, ois subdev is NULL", __func__);
		return;
	}

	is_ois_get_phone_version(&ois_pinfo);

	camera_running = is_vendor_check_camera_running(SENSOR_POSITION_REAR);

	if (camera_running) {
		info("%s : camera is running. reset ois gyro.\n", __func__);

		reg = OIS_CMD_MODE;
		ret = ois_write_u8(reg, 0x16);
		if (ret)
			MCU_SET_ERR_PRINT(reg);

		ois_pinfo->reset_check= true;
	} else {
		ois_pinfo->reset_check= false;
		info("%s : camera is not running.\n", __func__);
	}

	info("%s : X\n", __func__);
}
#endif

int is_advanced_ois_read_fw_ver(char *name, char *ver)
{
	int ret = 0;
#if 0
	ulong size = 0;
	char buf[100] = {0, };
	struct file *fp = NULL;
	mm_segment_t old_fs;
	long nread;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(name, O_RDONLY, 0);
	if (IS_ERR_OR_NULL(fp)) {
		info("failed to open fw!!!\n");
		ret = -EIO;
		goto exit;
	}

	size = 4;
	fp->f_pos = 0x80F8;

	nread = kernel_read(fp, (char __user *)(buf), size, &fp->f_pos);
	if (nread != size) {
		err("failed to read firmware file, %ld Bytes\n", nread);
		ret = -EIO;
		goto exit;
	}

exit:
	if (!IS_ERR_OR_NULL(fp))
		filp_close(fp, current->files);
	set_fs(old_fs);

	memcpy(ver, &buf[4], 3);
	memcpy(&ver[3], buf, 4);
#endif
	return ret;
}

static struct is_ois_ops ois_ops_mcu = {
	.ois_fw_update = is_advanced_ois_fw_update,
	.ois_check_fw = is_advanced_ois_check_fw,
	.ois_read_fw_ver = is_advanced_ois_read_fw_ver,
	.ois_init = is_vendor_ois_init,
	.ois_init_fac = is_vendor_ois_init_factory,
	.ois_deinit = is_vendor_ois_deinit,
	.ois_set_mode = is_vendor_ois_set_mode,
	.ois_shift_compensation = is_vendor_ois_shift_compensation,
	.ois_self_test = is_vendor_ois_self_test,
	.ois_auto_test = is_vendor_ois_auto_test_all,
#ifdef CAMERA_2ND_OIS
	.ois_auto_test_rear2 = is_vendor_ois_auto_test_rear2,
	.ois_set_power_mode = is_vendor_ois_set_power_mode,
#endif
	.ois_enable = is_vendor_ois_enable,
	.ois_offset_test = is_vendor_ois_offset_test,
	.ois_get_offset_data = is_vendor_ois_get_offset_data,
	.ois_gyro_sleep = is_vendor_ois_gyro_sleep,
	.ois_exif_data = is_vendor_ois_exif_data,
	.ois_read_status = is_vendor_ois_read_status,
	.ois_read_cal_checksum = is_vendor_ois_read_cal_checksum,
	.ois_set_coef = is_vendor_ois_set_coef,
	.ois_set_center = is_vendor_ois_set_centering,
	.ois_read_mode = is_vendor_ois_read_mode,
	.ois_calibration_test = is_vendor_ois_gyro_cal,
	.ois_read_gyro_noise = is_vendor_ois_read_gyro_noise,
	.ois_get_hall_pos = is_vendor_ois_get_hall_position,
	.ois_check_valid = is_vendor_ois_check_valid,
#ifdef USE_OIS_HALL_DATA_FOR_VDIS
	.ois_get_hall_data = is_vendor_ois_get_hall_data,
#endif
	.ois_get_active = is_vendor_ois_get_active,
	.ois_parsing_raw_data = is_vendor_ois_parsing_raw_data,
	.ois_center_shift = is_vendor_ois_set_center_shift,
};

static struct is_aperture_ops aperture_ops_mcu = {
	.set_aperture_value = is_mcu_set_aperture,
	.aperture_deinit = is_mcu_deinit_aperture,
};

#ifdef CONFIG_SENSORCORE_MCU_CONTROL
struct ois_sensor_interface {
	void *core;
	void (*ois_func)(void *);
};

static struct ois_sensor_interface ois_control;
static struct ois_sensor_interface ois_reset;

extern int ois_fw_update_register(struct ois_sensor_interface *ois);
extern void ois_fw_update_unregister(void);
extern int ois_reset_register(struct ois_sensor_interface *ois);
extern void ois_reset_unregister(void);
#endif

static int is_aois_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct is_core *core;
	struct ois_mcu_dev *mcu = NULL;
	struct device_node *dnode;
	struct is_mcu *is_mcu = NULL;
	struct is_device_sensor *device;
	struct v4l2_subdev *subdev_mcu = NULL;
	struct is_vendor_private *vendor_priv;
	struct v4l2_subdev *subdev_ois = NULL;
	struct is_ois *ois = NULL;
	struct is_aperture *aperture = NULL;
	struct v4l2_subdev *subdev_aperture = NULL;
	u32 sensor_id_len;
	const u32 *sensor_id_spec;
	const u32 *gyro_direction_spec;
	u32 gyro_direction_len;
	const u32 *aperture_delay_spec;
	u32 sensor_id[IS_SENSOR_COUNT] = {0, };
	int i = 0;

	core = is_get_is_core();
	if (!core) {
		err("core device is not yet probed");
		ret = -EPROBE_DEFER;
		goto p_err;
	}

	dnode = pdev->dev.of_node;

	sensor_id_spec = of_get_property(dnode, "id", &sensor_id_len);
	if (!sensor_id_spec) {
		err("sensor_id num read is fail(%d)", ret);
		goto p_err;
	}

	sensor_id_len /= (unsigned int)sizeof(*sensor_id_spec);

	ret = of_property_read_u32_array(dnode, "id", sensor_id, sensor_id_len);
	if (ret) {
		err("sensor_id read is fail(%d)", ret);
		goto p_err;
	}

	if (sensor_id_len <= 0) {
		err("sensor_id_len is not valid");
		goto p_err;
	}

	mcu = pablo_zalloc(sizeof(struct ois_mcu_dev), GFP_KERNEL);
	if (!mcu)
		return -ENOMEM;

	gyro_direction_spec = of_get_property(dnode, "ois_gyro_direction", &gyro_direction_len);
	if (gyro_direction_spec) {
		gyro_direction_len /= (unsigned int)sizeof(*gyro_direction_spec);
		ret = of_property_read_u32_array(dnode, "ois_gyro_direction",
				mcu->ois_gyro_direction, gyro_direction_len);
		if (ret)
			probe_err("ois_gyro_direction read is fail(%d)", ret);
	}

	aperture_delay_spec = of_get_property(dnode, "aperture_control_delay", &mcu->aperture_delay_list_len);
	if (aperture_delay_spec) {
		mcu->aperture_delay_list_len /= (unsigned int)sizeof(*aperture_delay_spec);
		ret = of_property_read_u32_array(dnode, "aperture_control_delay",
				mcu->aperture_delay_list, mcu->aperture_delay_list_len);
		if (ret)
				info("aperture_control_delay read is fail(%d)", ret);
	}

	for (i = 0; i < sensor_id_len; i++) {
		device = &core->sensor[sensor_id[i]];
		if (!device) {
			err("sensor device is NULL");
			ret = -EPROBE_DEFER;
			goto p_err;
		}
	}

	is_mcu = pablo_zalloc(sizeof(struct is_mcu) * sensor_id_len, GFP_KERNEL);
	if (!is_mcu) {
		err("is_mcu is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	subdev_mcu = pablo_zalloc(sizeof(struct v4l2_subdev) * sensor_id_len, GFP_KERNEL);
	if (!subdev_mcu) {
		err("subdev_mcu is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	ois = pablo_zalloc(sizeof(struct is_ois) * sensor_id_len, GFP_KERNEL);
	if (!ois) {
		err("is_ois is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	subdev_ois = pablo_zalloc(sizeof(struct v4l2_subdev) * sensor_id_len, GFP_KERNEL);
	if (!subdev_ois) {
		err("subdev_ois is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	aperture = pablo_zalloc(sizeof(struct is_aperture) * sensor_id_len, GFP_KERNEL);
	if (!aperture) {
		err("aperture is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	subdev_aperture = pablo_zalloc(sizeof(struct v4l2_subdev)  * sensor_id_len, GFP_KERNEL);
	if (!subdev_aperture) {
		err("subdev_aperture is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	vendor_priv = core->vendor.private_data;

	mcu->ois_wide_init = false;
	mcu->ois_tele_init = false;
	mcu->ois_hw_check = false;

	for (i = 0; i < sensor_id_len; i++) {
		probe_info("%s sensor_id %d\n", __func__, sensor_id[i]);

		is_mcu[i].name = MCU_NAME_STM32;
		is_mcu[i].subdev = &subdev_mcu[i];
		is_mcu[i].device = sensor_id[i];
		is_mcu[i].private_data = core;

		ois[i].subdev = &subdev_ois[i];
		ois[i].device = sensor_id[i];
		ois[i].ois_mode = OPTICAL_STABILIZATION_MODE_OFF;
		ois[i].pre_ois_mode = OPTICAL_STABILIZATION_MODE_OFF;
		ois[i].ois_shift_available = false;
		ois[i].ois_ops = &ois_ops_mcu;
		set_ois_comm_ops(&advanced_ois_ops);

		is_mcu[i].subdev_ois = &subdev_ois[i];
		is_mcu[i].ois = &ois[i];

		if (i == 0) {
			aperture[i].start_value = F2_4;
			aperture[i].new_value = 0;
			aperture[i].cur_value = 0;
			aperture[i].step = APERTURE_STEP_STATIONARY;
			aperture[i].subdev = &subdev_aperture[i];
			aperture[i].aperture_ops = &aperture_ops_mcu;

			mutex_init(&aperture[i].control_lock);

			is_mcu[i].aperture = &aperture[i];
			is_mcu[i].subdev_aperture = aperture[i].subdev;
			is_mcu[i].aperture_ops = &aperture_ops_mcu;
		}

		device = &core->sensor[sensor_id[i]];
		device->subdev_mcu = &subdev_mcu[i];
		device->mcu = &is_mcu[i];

		v4l2_set_subdevdata(&subdev_mcu[i], mcu);
		v4l2_set_subdev_hostdata(&subdev_mcu[i], &is_mcu[i]);
	}

#ifdef CONFIG_SENSORCORE_MCU_CONTROL
	ois_control.core = core;
	ois_control.ois_func = &is_ois_fw_update_from_sensor;
	ret = ois_fw_update_register(&ois_control);
	if (ret)
		err("ois_fw_update_register failed: %d\n", ret);

	ois_reset.core = core;
	ois_reset.ois_func = &is_ois_reset_mcu;
	ret = ois_reset_register(&ois_reset);
	if (ret)
		err("ois_reset_register failed: %d\n", ret);
#endif
	set_bit(OM_HW_NONE, &mcu->state);
	probe_info("%s done\n", __func__);

	return ret;

p_err:
	if (is_mcu)
		pablo_free(is_mcu);

	if (subdev_mcu)
		pablo_free(subdev_mcu);

	if (ois)
		pablo_free(ois);

	if (subdev_ois)
		pablo_free(subdev_ois);

	if (aperture)
		pablo_free(aperture);

	if (subdev_aperture)
		pablo_free(subdev_aperture);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id sensor_aois_match[] = {
	{
		.compatible = "samsung,sensor-aois-mcu",
	},
	{},
};
#endif

struct platform_driver sensor_aois_platform_driver = {
	.probe	= is_aois_probe,
	.driver = {
		.name	= AOIS_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sensor_aois_match,
#endif
	},
};

struct platform_driver *get_aois_platform_driver(void)
{
	return &sensor_aois_platform_driver;
}

#ifndef MODULE
static int __init sensor_aois_init(void)
{
	int ret;

	ret = platform_driver_probe(&sensor_aois_platform_driver,
							is_aois_probe);
	if (ret)
		err("failed to probe %s driver: %d\n",
			sensor_aois_platform_driver.driver.name, ret);

	return ret;
}
late_initcall_sync(sensor_aois_init);
#endif

MODULE_LICENSE("GPL v2");
