/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 fimc-is core functions
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
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
#include "is-core.h"
#include "is-device-sensor-peri.h"
#include "is-interface.h"
#include "is-sec-define.h"
#include "is-device-ischain.h"
#include "is-dt.h"
#include "is-device-ois_common.h"
#include "is-vendor-private.h"
#include "is-vendor-ois-core.h"
#ifdef CONFIG_AF_HOST_CONTROL
#include "is-device-af.h"
#endif
#include <linux/pinctrl/pinctrl.h>
#if defined(CONFIG_CAMERA_USE_INTERNAL_MCU)
#include "is-vendor-ois-internal-mcu.h"
#elif defined(CONFIG_CAMERA_USE_AOIS)
#include "is-interface-aois.h"
#endif

#define IS_OIS_DEV_NAME		"exynos-is-ois"

struct is_ois_info ois_minfo;
struct is_ois_info ois_pinfo;
struct is_ois_info ois_uinfo;
struct is_ois_exif ois_exif_data;

static struct ois_comm_ops *g_ois_comm_ops = NULL;

int ois_read_u8(int cmd, u8 *data) {
	int ret = 0;
	if (g_ois_comm_ops && g_ois_comm_ops->read_u8)
		ret = g_ois_comm_ops->read_u8(cmd, data);
	else {
		err_mcu("OIS read_u8 not available");
		ret = -EINVAL;
	}
	return ret;
};
int ois_write_u8(int cmd, u8 data) {
	int ret = 0;
	if (g_ois_comm_ops && g_ois_comm_ops->write_u8)
		ret = g_ois_comm_ops->write_u8(cmd, data);
	else {
		err_mcu("OIS write_u8 not available");
		ret = -EINVAL;
	}
	return ret;
}

int ois_read_multi(int cmd, u8 *data, size_t size) {
	int ret = 0;
	if (g_ois_comm_ops && g_ois_comm_ops->read_multi)
		ret = g_ois_comm_ops->read_multi(cmd, data, size);
	else {
		err_mcu("OIS read_multi not available");
		ret = -EINVAL;
	}

	return ret;
};
int ois_write_multi(int cmd, u8 *data, size_t size) {
	int ret = 0;
	if (g_ois_comm_ops && g_ois_comm_ops->write_multi)
		ret = g_ois_comm_ops->write_multi(cmd, data, size);
	else {
		err_mcu("OIS write_multi not available");
		ret = -EINVAL;
	}
	return ret;
}

int ois_read_u16(int cmd, u8 *data) {
	int ret = 0;
	if (g_ois_comm_ops && g_ois_comm_ops->read_u16)
		ret = g_ois_comm_ops->read_u16(cmd, data);
	else {
		err_mcu("OIS read_u16 not available");
		ret = -EINVAL;
	}
	return ret;
};
int ois_write_u16(int cmd, u8 *data) {
	int ret = 0;
	if (g_ois_comm_ops && g_ois_comm_ops->write_u16)
		ret = g_ois_comm_ops->write_u16(cmd, data);
	else {
		err_mcu("OIS write_u16 not available");
		ret = -EINVAL;
	}
	return ret;
}

int set_ois_comm_ops(struct ois_comm_ops *ops)
{
	if (ops == NULL) {
		err_mcu("ois_comm_ops is NULL");
		return -EINVAL;
	}

	g_ois_comm_ops = ops;
	return 0;
}

int is_ois_control_gpio(struct is_core *core, int position, int onoff)
{
	int ret = 0;
	struct exynos_platform_is_module *module_pdata;
	struct is_module_enum *module = NULL;
	int i = 0;
	struct ois_mcu_dev *mcu = NULL;

	info("%s E", __func__);

	mcu = core->mcu;

	for (i = 0; i < IS_SENSOR_COUNT; i++) {
		is_search_sensor_module_with_position(&core->sensor[i], position, &module);
		if (module)
			break;
	}

	if (!module) {
		err("%s: Could not find sensor id.", __func__);
		ret = -EINVAL;
		goto p_err;
	}

	module_pdata = module->pdata;

	if (!module_pdata->gpio_cfg) {
		err("gpio_cfg is NULL");
		ret = -EINVAL;
		goto p_err;
	}

#if defined(CONFIG_CAMERA_USE_INTERNAL_MCU)
	mutex_lock(&mcu->power_mutex);
#endif

	ret = module_pdata->gpio_cfg(module, SENSOR_SCENARIO_OIS_FACTORY, onoff);
	if (ret) {
		err("gpio_cfg is fail(%d)", ret);
	}

#if defined(CONFIG_CAMERA_USE_INTERNAL_MCU)
	mutex_unlock(&mcu->power_mutex);
#endif

p_err:
	info("%s X", __func__);

	return ret;
}

int is_ois_gpio_on(struct is_core *core)
{
	int ret = 0;

	info("%s E", __func__);

	is_ois_control_gpio(core, SENSOR_POSITION_REAR, GPIO_SCENARIO_ON);
#ifdef CAMERA_2ND_OIS
	is_ois_control_gpio(core, SENSOR_POSITION_REAR2, GPIO_SCENARIO_ON);
#endif
#ifdef CAMERA_3RD_OIS
	is_ois_control_gpio(core, SENSOR_POSITION_REAR4, GPIO_SCENARIO_ON);
#endif

#if defined(CONFIG_CAMERA_USE_INTERNAL_MCU)
	is_vendor_mcu_power_on(false);
#endif

	info("%s X", __func__);

	return ret;
}

int is_ois_gpio_off(struct is_core *core)
{
	int ret = 0;

	info("%s E", __func__);

#if defined(CONFIG_CAMERA_USE_INTERNAL_MCU)
	is_vendor_mcu_power_off(false);
#endif

	is_ois_control_gpio(core, SENSOR_POSITION_REAR, GPIO_SCENARIO_OFF);
#ifdef CAMERA_2ND_OIS
	is_ois_control_gpio(core, SENSOR_POSITION_REAR2, GPIO_SCENARIO_OFF);
#endif
#ifdef CAMERA_3RD_OIS
	is_ois_control_gpio(core, SENSOR_POSITION_REAR4, GPIO_SCENARIO_OFF);
#endif

	info("%s X", __func__);

	return ret;
}

struct is_ois *is_ois_get_device(struct is_core *core)
{
	struct is_ois *ois_device = NULL;
	struct is_device_sensor *device = NULL;

	device = &core->sensor[0];
	if (!device) {
		err("device is NULL");
		return NULL;
	}
	if (!device->mcu) {
		err("device->mcu is NULL");
		return NULL;
	}
	if (!device->mcu->ois) {
		err("device->mcu->ois is NULL");
		return NULL;
	}
	ois_device = device->mcu->ois;

	return ois_device;
}

struct is_mcu *is_ois_get_mcu(struct is_core *core)
{
	struct is_vendor_private *vendor_priv = core->vendor.private_data;
	u32 sensor_idx = vendor_priv->mcu_sensor_index;

	if (core->sensor[sensor_idx].mcu != NULL)
		return core->sensor[sensor_idx].mcu;

	return NULL;
};

bool is_ois_offset_test(struct is_core *core, long *raw_data_x, long *raw_data_y, long *raw_data_z)
{
	bool result = false;
	struct is_ois *ois_device = NULL;

	ois_device = is_ois_get_device(core);
	result = CALL_OISOPS(ois_device, ois_offset_test, core, raw_data_x, raw_data_y, raw_data_z);

	return result;
}

int is_ois_self_test(struct is_core *core)
{
	int ret = 0;
	struct is_ois *ois_device = NULL;

	ois_device = is_ois_get_device(core);
	ret = CALL_OISOPS(ois_device, ois_self_test, core);

	return ret;
}

bool is_ois_gyrocal_test(struct is_core *core, long *raw_data_x, long *raw_data_y, long *raw_data_z)
{
	bool result = false;
	struct is_ois *ois_device = NULL;

	ois_device = is_ois_get_device(core);
	result = CALL_OISOPS(ois_device, ois_calibration_test, core, raw_data_x, raw_data_y, raw_data_z);

	return result;
}

bool is_ois_gyronoise_test(struct is_core *core, long *raw_data_x, long *raw_data_y)
{
	bool result = false;
	struct is_ois *ois_device = NULL;

	ois_device = is_ois_get_device(core);
	result = CALL_OISOPS(ois_device, ois_read_gyro_noise, core, raw_data_x, raw_data_y);

	return result;
}

bool is_ois_auto_test(struct is_core *core,
				int threshold, bool *x_result, bool *y_result, int *sin_x, int *sin_y,
				bool *x_result_2nd, bool *y_result_2nd, int *sin_x_2nd, int *sin_y_2nd,
				bool *x_result_3rd, bool *y_result_3rd, int *sin_x_3rd, int *sin_y_3rd)
{
	bool result = false;
	struct is_ois *ois_device = NULL;

	ois_device = is_ois_get_device(core);

	result = CALL_OISOPS(ois_device, ois_auto_test, core,
			threshold, x_result, y_result, sin_x, sin_y,
			x_result_2nd, y_result_2nd, sin_x_2nd, sin_y_2nd,
			x_result_3rd, y_result_3rd, sin_x_3rd, sin_y_3rd);

	return result;
}

#ifdef CAMERA_2ND_OIS
bool is_ois_auto_test_rear2(struct is_core *core,
				int threshold, bool *x_result, bool *y_result, int *sin_x, int *sin_y,
				bool *x_result_2nd, bool *y_result_2nd, int *sin_x_2nd, int *sin_y_2nd)
{
	bool result = false;
	struct is_ois *ois_device = NULL;

	ois_device = is_ois_get_device(core);
	result = CALL_OISOPS(ois_device, ois_auto_test_rear2, core,
			threshold, x_result, y_result, sin_x, sin_y,
			x_result_2nd, y_result_2nd, sin_x_2nd, sin_y_2nd);

	return result;
}
#endif

void is_ois_gyro_sleep(struct is_core *core)
{
	struct is_ois *ois_device = NULL;

	ois_device = is_ois_get_device(core);
	CALL_OISOPS(ois_device, ois_gyro_sleep, core);
}

int is_ois_get_exif_data(struct is_ois_exif **exif_info)
{
	*exif_info = &ois_exif_data;
	return 0;
}

int is_ois_get_module_version(struct is_ois_info **minfo)
{
	*minfo = &ois_minfo;
	return 0;
}

int is_ois_get_phone_version(struct is_ois_info **pinfo)
{
	*pinfo = &ois_pinfo;
	return 0;
}

int is_ois_get_user_version(struct is_ois_info **uinfo)
{
	*uinfo = &ois_uinfo;
	return 0;
}

bool is_ois_check_fw(struct is_core *core)
{
	bool ret = false;
	struct is_ois *ois_device = NULL;

	ois_device = is_ois_get_device(core);
	ret = CALL_OISOPS(ois_device, ois_check_fw, core);

	return ret;
}

void is_ois_fw_update(struct is_core *core)
{
	struct is_ois *ois_device = NULL;

	ois_device = is_ois_get_device(core);

	is_ois_gpio_on(core);
	msleep(30);
	CALL_OISOPS(ois_device, ois_fw_update, core);
	is_ois_gpio_off(core);

	return;
}

void is_ois_get_hall_pos(struct is_core *core, u16 *targetPos, u16 *hallPos)
{
	struct is_ois *ois_device = NULL;

	ois_device = is_ois_get_device(core);

	CALL_OISOPS(ois_device, ois_get_hall_pos, core, targetPos, hallPos);
}

void is_ois_check_cross_talk(struct is_core *core, u16 *hall_data)
{
	struct is_ois *ois_device = NULL;
	struct is_device_sensor *device = NULL;

	ois_device = is_ois_get_device(core);
	device = &core->sensor[0];

	CALL_OISOPS(ois_device, ois_check_cross_talk, device->subdev_mcu, hall_data);
}

void is_ois_get_hall_data(struct is_core *core, struct is_ois_hall_data *halldata)
{
	struct is_ois *ois_device = NULL;
	struct is_device_sensor *device = NULL;

	ois_device = is_ois_get_device(core);
	device = &core->sensor[0];

#ifdef USE_OIS_HALL_DATA_FOR_VDIS
	if (CALL_OISOPS(device->mcu->ois, ois_get_active, device->subdev_mcu)) {
		CALL_OISOPS(device->mcu->ois, ois_get_hall_data, device->subdev_mcu, halldata);
	}
#endif
}

void is_ois_check_hall_cal(struct is_core *core, u16 *hall_cal_data)
{
	struct is_ois *ois_device = NULL;
	struct is_device_sensor *device = NULL;

	ois_device = is_ois_get_device(core);
	device = &core->sensor[0];

	CALL_OISOPS(ois_device, ois_check_hall_cal, device->subdev_mcu, hall_cal_data);
}

void is_ois_check_valid(struct is_core *core, u8 *value)
{
	struct is_ois *ois_device = NULL;
	struct is_device_sensor *device = NULL;

	ois_device = is_ois_get_device(core);
	device = &core->sensor[0];

	CALL_OISOPS(ois_device, ois_check_valid, device->subdev_mcu, value);
}

int is_ois_read_ext_clock(struct is_core *core, u32 *clock)
{
	struct is_ois *ois_device = NULL;
	struct is_device_sensor *device = NULL;
	int ret = 0;

	ois_device = is_ois_get_device(core);
	device = &core->sensor[0];

	ret = CALL_OISOPS(ois_device, ois_read_ext_clock, device->subdev_mcu, clock);

	return ret;
}
#if defined(CAMERA_3RD_OIS)
void is_ois_init_rear2(struct is_core *core)
{
	struct is_ois *ois_device = NULL;

	ois_device = is_ois_get_device(core);

	CALL_OISOPS(ois_device, ois_init_rear2, core);
}
#endif
void is_ois_init_factory(struct is_core *core)
{
	struct is_device_sensor *device = NULL;
	struct is_mcu *mcu = NULL;

	device = &core->sensor[0];
	mcu = device->mcu;

	CALL_OISOPS(mcu->ois, ois_init_fac, device->subdev_mcu);
}

void is_ois_set_mode(struct is_core *core, int mode)
{
	struct is_device_sensor *device = NULL;
	struct is_mcu *mcu = NULL;
	int internal_mode = 0;

	device = &core->sensor[0];
	mcu = device->mcu;

	switch(mode) {
		case 0x0:
			internal_mode = OPTICAL_STABILIZATION_MODE_STILL;
			break;
		case 0x1:
			internal_mode = OPTICAL_STABILIZATION_MODE_VIDEO;
			break;
		case 0x5:
			internal_mode = OPTICAL_STABILIZATION_MODE_CENTERING;
			break;
		case 0x13:
			internal_mode = OPTICAL_STABILIZATION_MODE_STILL_ZOOM;
			break;
		case 0x14:
			internal_mode = OPTICAL_STABILIZATION_MODE_VDIS;
			break;
		default:
			dbg_ois("%s: ois_mode value(%d)\n", __func__, mode);
			break;
	}

	CALL_OISOPS(mcu->ois, ois_init_fac, device->subdev_mcu);
	CALL_OISOPS(mcu->ois, ois_set_mode, device->subdev_mcu, internal_mode);
}

void is_ois_parsing_raw_data(struct is_core *core, uint8_t *buf, long efs_size, long *raw_data_x, long *raw_data_y, long *raw_data_z)
{
	struct is_device_sensor *device = NULL;
	struct is_mcu *mcu = NULL;

	device = &core->sensor[0];
	mcu = device->mcu;

	CALL_OISOPS(mcu->ois, ois_parsing_raw_data, buf, efs_size, raw_data_x, raw_data_y, raw_data_z);
}

void is_ois_set_center_shift(struct is_core *core, int16_t *value)
{
	struct is_ois *ois_device = NULL;
	struct is_device_sensor *device = NULL;

	ois_device = is_ois_get_device(core);
	device = &core->sensor[0];

	CALL_OISOPS(ois_device, ois_center_shift, device->subdev_mcu, value);
}

MODULE_DESCRIPTION("OIS driver for Rumba");
MODULE_AUTHOR("kyoungho yun <kyoungho.yun@samsung.com>");
MODULE_LICENSE("GPL v2");
