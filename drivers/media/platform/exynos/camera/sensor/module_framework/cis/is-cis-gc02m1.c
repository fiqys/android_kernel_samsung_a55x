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

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/videodev2.h>
#include <videodev2_exynos_camera.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <exynos-is-sensor.h>
#include "is-hw.h"
#include "is-core.h"
#include "is-param.h"
#include "is-device-sensor.h"
#include "is-device-sensor-peri.h"
#include "is-resourcemgr.h"
#include "is-dt.h"
#include "is-cis-gc02m1.h"
#include "is-cis-gc02m1-setA.h"
#include "is-helper-ixc.h"

#define SENSOR_NAME "GC02M1"
#define POLL_TIME_US (1000)

static u16 sensor_gc02m1_fcount;

static bool sensor_gc02m1_check_master_stream_off(struct is_core *core)
{
	if (test_bit(IS_SENSOR_OPEN, &(core->sensor[0].state)) &&	/* Dual mode and master stream off */
			!test_bit(IS_SENSOR_FRONT_START, &(core->sensor[0].state)))
		return true;
	else
		return false;
}

int sensor_gc02m1_cis_check_rev(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u8 rev_msb = 0, rev_lsb = 0;
	u32 rev = 0;
	struct i2c_client *client;
	struct is_cis *cis = NULL;

	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		return ret;
	}

	IXC_MUTEX_LOCK(cis->ixc_lock);

	/* OTP Read initial setting */
	ret = sensor_cis_set_registers_addr8(subdev, sensor_gc02m1_otp_read, ARRAY_SIZE(sensor_gc02m1_otp_read));
	if (ret < 0) {
		err("set sensor_gc02m1_otp_read fail!!");
		goto p_err;
	}
	/* OTP read setting - version info (0x00 : COLOR , 0x04 : MONO) */
	ret = cis->ixc_ops->addr8_write8(client, 0x17, 0x10);
	if (ret < 0)
		goto p_err;

	ret = cis->ixc_ops->addr8_write8(client, 0xF3, 0x34);
	if (ret < 0)
		goto p_err;

	ret = cis->ixc_ops->addr8_read8(client, cis->rev_addr, &rev_msb);
	if (ret < 0)
		goto p_err;

	ret = cis->ixc_ops->addr8_read8(client, cis->rev_addr + 1, &rev_lsb);
	if (ret < 0)
		goto p_err;
	/* OTP off setting */
	ret = cis->ixc_ops->addr8_write8(client, 0xF3, 0x00);
	if (ret < 0)
		goto p_err;

	ret = cis->ixc_ops->addr8_write8(client, 0xFE, 0x00);
	if (ret < 0)
		goto p_err;

	rev = ((u32)rev_msb << 8) | rev_lsb;
	cis->cis_data->cis_rev = rev;
	pr_info("%s [%d] Sensor version. Rev(addr:0x%X). 0x%X\n",
		__func__, cis->id, cis->rev_addr, cis->cis_data->cis_rev);

p_err:
	IXC_MUTEX_UNLOCK(cis->ixc_lock);

	if (ret < 0) {
		err("%s fail!!", __func__);
		ret = -EAGAIN;
	}

	return ret;
}

//For finding the nearest value in the gain table
u32 sensor_gc02m1_cis_calc_again_closest(u32 permile)
{
	int i;

	if (permile <= sensor_gc02m1_analog_gain[CODE_GAIN_INDEX][PERMILE_GAIN_INDEX])
		return sensor_gc02m1_analog_gain[CODE_GAIN_INDEX][PERMILE_GAIN_INDEX];
	if (permile >= sensor_gc02m1_analog_gain[MAX_GAIN_INDEX][PERMILE_GAIN_INDEX])
		return sensor_gc02m1_analog_gain[MAX_GAIN_INDEX][PERMILE_GAIN_INDEX];

	for (i = 0; i <= MAX_GAIN_INDEX; i++) {
		if (sensor_gc02m1_analog_gain[i][PERMILE_GAIN_INDEX] == permile)
			return sensor_gc02m1_analog_gain[i][PERMILE_GAIN_INDEX];

		if ((int)(permile - sensor_gc02m1_analog_gain[i][PERMILE_GAIN_INDEX]) < 0)
			return sensor_gc02m1_analog_gain[i-1][PERMILE_GAIN_INDEX];
	}

	return sensor_gc02m1_analog_gain[MAX_GAIN_INDEX][PERMILE_GAIN_INDEX];
}

u32 sensor_gc02m1_cis_calc_again_permile(u32 code)
{
	u32 ret = 0;
	int i;

	for (i = 0; i <= MAX_GAIN_INDEX; i++) {
		if (sensor_gc02m1_analog_gain[i][0] == code) {
			ret = sensor_gc02m1_analog_gain[i][1];
			break;
		}
	}

	return ret;
}

u32 sensor_gc02m1_cis_calc_again_code(u32 permile)
{
	u32 ret = 0, nearest_val = 0;
	int i;

	dbg_sensor(1, "[%s] permile %d\n", __func__, permile);
	nearest_val = sensor_gc02m1_cis_calc_again_closest(permile);
	dbg_sensor(1, "[%s] nearest_val %d\n", __func__, nearest_val);

	for (i = 0; i <= MAX_GAIN_INDEX; i++) {
		if (sensor_gc02m1_analog_gain[i][1] == nearest_val) {
			ret = sensor_gc02m1_analog_gain[i][0];
			break;
		}
	}

	return ret;
}

u16 sensor_gc02m1_cis_get_framecount(struct is_cis *cis)
{
	struct i2c_client *client;
	u16 fcount = 0xffff;
	int ret = 0;
	u8 sensor_fcount_msb = 0, sensor_fcount_lsb = 0;

	client = cis->client;
	IXC_MUTEX_LOCK(cis->ixc_lock);
	ret = cis->ixc_ops->addr8_write8(client, 0xfe, 0x00);
	ret |= cis->ixc_ops->addr8_read8(client, 0xe1, &sensor_fcount_msb);
	ret |= cis->ixc_ops->addr8_read8(client, 0xe2, &sensor_fcount_lsb);
	if (ret < 0) {
		err("i2c transfer fail addr(%x), val(%x), ret = %d\n", 0xe2, sensor_fcount_lsb, ret);
		goto p_err_i2c;
	} else {
		fcount = (sensor_fcount_msb << 8) | sensor_fcount_lsb;
	}

p_err_i2c:
	IXC_MUTEX_UNLOCK(cis->ixc_lock);

	return fcount;
}

int sensor_gc02m1_set_flip_register(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct is_cis *cis = NULL;
	struct i2c_client *client;
	FIMC_BUG(!subdev);
	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);
	FIMC_BUG(!cis);
	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		return ret;
	}
	switch (cis->orientation) {
	case SENSOR_FLIP_X:
		ret = cis->ixc_ops->addr8_write8(client, 0xfe, 0x00);
		if (ret < 0)
			goto p_err;
		ret = cis->ixc_ops->addr8_write8(client, 0x17, 0x81);
		if (ret < 0)
			goto p_err;
		break;
	case SENSOR_FLIP_Y:
		ret = cis->ixc_ops->addr8_write8(client, 0xfe, 0x00);
		if (ret < 0)
			goto p_err;
		ret = cis->ixc_ops->addr8_write8(client, 0x17, 0x82);
		if (ret < 0)
			goto p_err;
		break;
	case SENSOR_FLIP_XY:
		ret = cis->ixc_ops->addr8_write8(client, 0xfe, 0x00);
		if (ret < 0)
			goto p_err;
		ret = cis->ixc_ops->addr8_write8(client, 0x17, 0x83);
		if (ret < 0)
			goto p_err;
		break;
	default:
		break;
	}
p_err:
	return ret;
}

int sensor_gc02m1_cis_set_exposure_time(struct v4l2_subdev *subdev, u16 multiple_exp)
{
	int ret = 0;
	struct is_cis *cis;
	struct i2c_client *client;
	ktime_t st = ktime_get();

	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		return ret;
	}

	dbg_sensor(1, "[%s] multiple_exp %d\n", __func__, multiple_exp);

	IXC_MUTEX_LOCK(cis->ixc_lock);

	/* Page Selection */
	ret = cis->ixc_ops->addr8_write8(client, 0xfe, 0x00);
	if (ret < 0)
		goto p_err;

	/* Short exposure */
	ret = cis->ixc_ops->addr8_write8(client, 0x03, (multiple_exp >> 8) & 0x3f);
	if (ret < 0)
		goto p_err;
	ret = cis->ixc_ops->addr8_write8(client, 0x04, (multiple_exp & 0xfc));
	if (ret < 0)
		goto p_err;

	if (IS_ENABLED(DEBUG_SENSOR_TIME))
		dbg_sensor(1, "[%s] time %lldus\n", __func__, PABLO_KTIME_US_DELTA_NOW(st));

p_err:
	IXC_MUTEX_UNLOCK(cis->ixc_lock);

	return ret;
}

u32 sensor_gc02m1_cis_calc_dgain_code(u32 input_gain, u32 permile)
{
	u32 calc_value = 0;
	u32 digital_gain = 0;

	if (permile > 0) {
		calc_value = input_gain * 1000 / permile;
		digital_gain = (calc_value * 1024) / 1000;
	}

	dbg_sensor(1, "[%s] input_gain : %d, calc_value : %d, digital_gain : %d\n",
		__func__, input_gain, calc_value, digital_gain);

	return digital_gain;
}

int sensor_gc02m1_cis_set_analog_gain(struct v4l2_subdev *subdev, u32 input_again)
{
	int ret = 0;
	struct is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u32 analog_gain = 0;
	u32 analog_permile = 0;
	u32 digital_gain = 0;
	ktime_t st = ktime_get();

	FIMC_BUG(!subdev);

	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		return ret;
	}

	cis_data = cis->cis_data;

	analog_gain = sensor_gc02m1_cis_calc_again_code(input_again);
	if (analog_gain < cis->cis_data->min_analog_gain[0])
		analog_gain = cis->cis_data->min_analog_gain[0];

	if (analog_gain > cis->cis_data->max_analog_gain[0])
		analog_gain = cis->cis_data->max_analog_gain[0];

	analog_permile = sensor_gc02m1_cis_calc_again_permile(analog_gain);
	digital_gain = sensor_gc02m1_cis_calc_dgain_code(input_again, analog_permile);

	dbg_sensor(1, "[MOD:D:%d] %s, input_again = %d us, analog_gain(%#x), digital_gain(%#x)\n",
			cis->id, __func__, input_again, analog_gain, digital_gain);

	IXC_MUTEX_LOCK(cis->ixc_lock);

	ret = cis->ixc_ops->addr8_write8(client, 0xfe, 0x00);
	if (ret < 0)
		goto p_err;

	/* Analog gain */
	ret = cis->ixc_ops->addr8_write8(client, 0xb6, analog_gain);
	if (ret < 0)
		goto p_err;

	/* Digital gain int*/
	if (digital_gain >= 1280)
		ret = cis->ixc_ops->addr8_write8(client, 0xb1, 0x05);
	else
		ret = cis->ixc_ops->addr8_write8(client, 0xb1, 0x04);
	if (ret < 0)
		goto p_err;

	/* Digital gain decimal*/
	ret = cis->ixc_ops->addr8_write8(client, 0xb2, digital_gain);
	if (ret < 0)
		goto p_err;


	if (IS_ENABLED(DEBUG_SENSOR_TIME))
		dbg_sensor(1, "[%s] time %lldus\n", __func__, PABLO_KTIME_US_DELTA_NOW(st));

p_err:
	IXC_MUTEX_UNLOCK(cis->ixc_lock);

	return ret;
}

#if USE_OTP_AWB_CAL_DATA
// Do nothing ! Digital gains are used to compensate for the AWB M2M (module to mudule) variation
int sensor_gc02m1_set_digital_gain(struct v4l2_subdev *subdev, struct ae_param *dgain)
{
	return 0;
}
#else
int sensor_gc02m1_set_digital_gain(struct v4l2_subdev *subdev, struct ae_param *dgain)
{
	int ret = 0;
	struct is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u16 long_gain = 0;
	u16 short_gain = 0;
	u8 dgains[2] = {0};
	ktime_t st = ktime_get();

	FIMC_BUG(!subdev);
	FIMC_BUG(!dgain);

	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		return ret;
	}

	/*skip to set dgain when use_dgain is false */
	if (cis->use_dgain == false)
		return 0;

	cis_data = cis->cis_data;

	long_gain = (u16)sensor_cis_calc_dgain_code(dgain->long_val);
	short_gain = (u16)sensor_cis_calc_dgain_code(dgain->short_val);

	if (long_gain < cis->cis_data->min_digital_gain[0])
		long_gain = cis->cis_data->min_digital_gain[0];

	if (long_gain > cis->cis_data->max_digital_gain[0])
		long_gain = cis->cis_data->max_digital_gain[0];

	if (short_gain < cis->cis_data->min_digital_gain[0])
		short_gain = cis->cis_data->min_digital_gain[0];

	if (short_gain > cis->cis_data->max_digital_gain[0])
		short_gain = cis->cis_data->max_digital_gain[0];

	dbg_sensor(1, "[MOD:D:%d] %s, input_dgain = %d/%d us, long_gain(%#x), short_gain(%#x)\n",
			cis->id, __func__, dgain->long_val, dgain->short_val, long_gain, short_gain);

	IXC_MUTEX_LOCK(cis->ixc_lock);

	dgains[0] = dgains[1] = short_gain;

	ret = cis->ixc_ops->addr8_write8(client, 0xfe, 0x00);
	if (ret < 0)
		goto p_err;

	/* Digital gain int*/
	ret = cis->ixc_ops->addr8_write8(client, 0xb1, (short_gain >> 8) & 0x0f);
	if (ret < 0)
		goto p_err;

	/* Digital gain decimal*/
	ret = cis->ixc_ops->addr8_write8(client, 0xb2, short_gain & 0xfc);
		if (ret < 0)
			goto p_err;

	if (IS_ENABLED(DEBUG_SENSOR_TIME))
		dbg_sensor(1, "[%s] time %lldus", __func__, PABLO_KTIME_US_DELTA_NOW(st));

p_err:
	IXC_MUTEX_UNLOCK(cis->ixc_lock);

	return ret;
}
#endif

int sensor_gc02m1_cis_init(struct v4l2_subdev *subdev)
{
	int ret = 0;
	cis_setting_info setinfo;

#if USE_OTP_AWB_CAL_DATA
	struct i2c_client *client = NULL;
	u8 selected_page;
	u16 data16[4];
	u8 cal_map_ver[4];
	bool skip_cal_write = false;
#endif
	ktime_t st = ktime_get();

	setinfo.param = NULL;
	setinfo.return_value = 0;

	FIMC_BUG(!subdev);
	struct is_cis *cis = sensor_cis_get_cis(subdev);

#if USE_OTP_AWB_CAL_DATA
	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}
#endif
	cis->cis_data->stream_on = false;
	cis->cis_data->cur_width = cis->sensor_info->max_width;
	cis->cis_data->cur_height = cis->sensor_info->max_height;
	cis->cis_data->low_expo_start = 33000;
	cis->need_mode_change = false;

	cis->mipi_clock_index_cur = CAM_MIPI_NOT_INITIALIZED;
	cis->mipi_clock_index_new = CAM_MIPI_NOT_INITIALIZED;
	cis->cis_data->sens_config_index_pre = SENSOR_GC02M1_MODE_MAX;
	cis->cis_data->sens_config_index_cur = 0;

	CALL_CISOPS(cis, cis_data_calculation, subdev, cis->cis_data->sens_config_index_cur);

	if (IS_ENABLED(DEBUG_SENSOR_TIME))
		dbg_sensor(1, "[%s] time %lldus", __func__, PABLO_KTIME_US_DELTA_NOW(st));

#if USE_OTP_AWB_CAL_DATA
p_err:
#endif
	return ret;
}

static const struct is_cis_log log_gc02m1[] = {
	{I2C_WRITE, 8, 0xfe, 0x00, "page_select"},
	{I2C_READ, 8, 0xe1, 0, "frame_count_msb"},
	{I2C_READ, 8, 0xe2, 0, "frame_count_lsb"},
	{I2C_READ, 8, 0xb6, 0, "again"},
	{I2C_READ, 8, 0xb1, 0, "dgain"},
	{I2C_READ, 8, 0x41, 0, "fll"},
};

int sensor_gc02m1_cis_log_status(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct is_cis *cis = sensor_cis_get_cis(subdev);

	sensor_cis_log_status(cis, log_gc02m1, ARRAY_SIZE(log_gc02m1), (char *)__func__);

	return ret;
}

int sensor_gc02m1_cis_set_global_setting(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct is_cis *cis = sensor_cis_get_cis(subdev);
	struct sensor_gc02m1_private_data *priv = (struct sensor_gc02m1_private_data *)cis->sensor_info->priv;

	info("[%s] start\n", __func__);

	ret = sensor_cis_write_registers_locked(subdev, priv->global);
	if (ret < 0)
		err("global setting fail!!");

	info("[%s] done\n", __func__);

	return ret;
}

int sensor_gc02m1_cis_mode_change(struct v4l2_subdev *subdev, u32 mode)
{
	int ret = 0;
	const struct sensor_cis_mode_info *mode_info;
	struct is_cis *cis = NULL;
	struct i2c_client *client;

	FIMC_BUG(!subdev);

	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);
	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		return ret;
	}

	if (mode >= cis->sensor_info->mode_count) {
		err("invalid mode(%d)!!", mode);
		ret = -EINVAL;
		goto p_err;
	}

	info("[%s] sensor mode(%d)\n", __func__, mode);

	mode_info = cis->sensor_info->mode_infos[mode];

	ret = sensor_cis_write_registers_locked(subdev, mode_info->setfile);
	if (ret < 0)
		err("sensor_gc02m1_setfiles fail!!");

	ret = sensor_gc02m1_set_flip_register(subdev);

	cis->cis_data->frame_time = (cis->cis_data->line_readOut_time * cis->cis_data->cur_height / 1000);
	cis->cis_data->rolling_shutter_skew = (cis->cis_data->cur_height - 1) * cis->cis_data->line_readOut_time;
	dbg_sensor(1, "[%s] frame_time(%d), rolling_shutter_skew(%lld)\n", __func__,
		cis->cis_data->frame_time, cis->cis_data->rolling_shutter_skew);

	dbg_sensor(1, "[%s] mode changed(%d)\n", __func__, mode);

p_err:
	return ret;
}

int sensor_gc02m1_cis_stream_on(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct is_core *core = NULL;
	struct is_cis *cis = NULL;
	struct i2c_client *client = NULL;
	cis_shared_data *cis_data = NULL;
	struct is_device_sensor *this_device = NULL;
	bool single_mode = true; /* default single */
	ktime_t st = ktime_get();

	FIMC_BUG(!subdev);

	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);
	struct sensor_gc02m1_private_data *priv = (struct sensor_gc02m1_private_data *)cis->sensor_info->priv;

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	this_device = (struct is_device_sensor *)v4l2_get_subdev_hostdata(subdev);
	FIMC_BUG(!this_device);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	core = is_get_is_core();
	if (!core) {
		err("The core device is null");
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

#if !defined(DISABLE_DUAL_SYNC)
	if ((this_device != &core->sensor[0]) && test_bit(IS_SENSOR_OPEN, &(core->sensor[0].state)))
		single_mode = false;
#endif

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	/* Sensor Dual sync on/off */
	if (single_mode == false) {
		info("[%s] dual sync slave mode\n", __func__);
		ret = sensor_cis_write_registers_locked(subdev, priv->fsync_slave);
		if (ret < 0)
			err("[%s] sensor_gc02m1_fsync_slave fail\n", __func__);

		/* The delay which can change the frame-length of first frame was removed here*/
	}

	/* Sensor stream on */
	/* 1. page_select */
	ret = cis->ixc_ops->addr8_write8(client, 0xfe, 0x00);
	if (ret < 0)
		goto p_err;
	ret = cis->ixc_ops->addr8_write8(client, 0x3E, 0x90);
	if (ret < 0) {
		err("i2c transfer fail addr(%x), val(%x), ret(%d)\n", 0x3e, 0x90, ret);
		goto p_err;
	}

	cis_data->stream_on = true;
	sensor_gc02m1_fcount = 0;
	info("%s done, (single_mode : %d)\n", __func__, single_mode);

	if (IS_ENABLED(DEBUG_SENSOR_TIME))
		dbg_sensor(1, "[%s] time %lldus", __func__, PABLO_KTIME_US_DELTA_NOW(st));

p_err:
	return ret;
}

int sensor_gc02m1_cis_stream_off(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;
	ktime_t st = ktime_get();

	FIMC_BUG(!subdev);

	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
	}

	cis_data = cis->cis_data;

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	sensor_gc02m1_fcount = sensor_gc02m1_cis_get_framecount(cis);

	/* Page Selection */
	ret = cis->ixc_ops->addr8_write8(client, 0xfe, 0x00);
	if (ret < 0)
		goto p_err;

	/* Sensor stream off */
	ret = cis->ixc_ops->addr8_write8(client, 0x3e, 0x00);
	if (ret < 0) {
		err("i2c transfer fail addr(%x), val(%x), ret(%d)\n", 0x3e, 0x00, ret);
		goto p_err;
	}

	cis_data->stream_on = false;

	info("%s done, frame_count(%d) cur_frame_ms_time(%d)\n",
		__func__, sensor_gc02m1_fcount, cis_data->cur_frame_us_time/1000);

	if (IS_ENABLED(DEBUG_SENSOR_TIME))
		dbg_sensor(1, "[%s] time %lldus", __func__, PABLO_KTIME_US_DELTA_NOW(st));

p_err:
	IXC_MUTEX_UNLOCK(cis->ixc_lock);
	return ret;
}

int sensor_gc02m1_cis_set_frame_duration(struct v4l2_subdev *subdev, u32 frame_duration)
{
	int ret = 0;
	struct is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;
	struct is_core *core;

	u64 vt_pix_clk_freq_khz = 0;
	u32 line_length_pck = 0;
	u16 frame_length_lines = 0;
	ktime_t st = ktime_get();

	FIMC_BUG(!subdev);

	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		return ret;
	}

	cis_data = cis->cis_data;

	core = is_get_is_core();
	if (!core) {
		err("core device is null");
		ret = -EINVAL;
		return ret;
	}

	if (sensor_gc02m1_check_master_stream_off(core)) {
		dbg_sensor(1, "%s: Master cam did not enter in stream_on yet. Stop updating frame_length_lines", __func__);
		return ret;
	}

	if (frame_duration < cis_data->min_frame_us_time) {
		dbg_sensor(1, "frame duration is less than min(%d)\n", frame_duration);
		frame_duration = cis_data->min_frame_us_time;
	}

	vt_pix_clk_freq_khz = cis_data->pclk / 1000;
	line_length_pck = cis_data->line_length_pck;

	frame_length_lines = (u16)((vt_pix_clk_freq_khz * frame_duration) / (line_length_pck * 1000));

	/* Frame length lines should be a multiple of 4 */
	frame_length_lines = MULTIPLE_OF_4(frame_length_lines);
	if (frame_length_lines > 0x3ffc) {
		warn("%s: frame_length_lines is above the maximum value : 0x%04x (should be lower than 0x3ffc)\n", __func__, frame_length_lines);
		frame_length_lines = 0x3ffc;
	}

	dbg_sensor(1, "[MOD:D:%d] %s, vt_pix_clk_freq_khz(%llu) frame_duration = %d us,"
		KERN_CONT "line_length_pck(%#x), frame_length_lines(%#x)\n",
		cis->id, __func__, vt_pix_clk_freq_khz, frame_duration, line_length_pck, frame_length_lines);

	IXC_MUTEX_LOCK(cis->ixc_lock);

	ret = cis->ixc_ops->addr8_write8(client, 0xfe, 0x00);
	if (ret < 0)
		goto p_err;
	ret = cis->ixc_ops->addr8_write8(client, 0x41, (frame_length_lines >> 8) & 0x3f);
	if (ret < 0)
		goto p_err;
	ret = cis->ixc_ops->addr8_write8(client, 0x42, (frame_length_lines & 0xfc));
	if (ret < 0)
		goto p_err;

	cis_data->cur_frame_us_time = frame_duration;
	cis_data->frame_length_lines = frame_length_lines;
	cis_data->max_coarse_integration_time = cis_data->frame_length_lines - cis_data->max_margin_coarse_integration_time;

	if (IS_ENABLED(DEBUG_SENSOR_TIME))
		dbg_sensor(1, "[%s] time %lldus", __func__, PABLO_KTIME_US_DELTA_NOW(st));

p_err:
	IXC_MUTEX_UNLOCK(cis->ixc_lock);

	return ret;
}

int sensor_gc02m1_cis_get_analog_gain(struct v4l2_subdev *subdev, u32 *again)
{
	int ret = 0;
	struct is_cis *cis;
	struct i2c_client *client;

	u8 analog_gain = 0;
	ktime_t st = ktime_get();

	FIMC_BUG(!subdev);
	FIMC_BUG(!again);

	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		return ret;
	}

	IXC_MUTEX_LOCK(cis->ixc_lock);

	/*page_select */
	ret = cis->ixc_ops->addr8_write8(client, 0xfe, 0x00);
	if (ret < 0)
		 goto p_err;

	ret = cis->ixc_ops->addr8_read8(client, 0xb6, &analog_gain);
	if (ret < 0)
		goto p_err;

	*again = sensor_gc02m1_cis_calc_again_permile(analog_gain);

	dbg_sensor(1, "[MOD:D:%d] %s, cur_again = %d us, analog_gain(%#x)\n",
			cis->id, __func__, *again, analog_gain);

	if (IS_ENABLED(DEBUG_SENSOR_TIME))
		dbg_sensor(1, "[%s] time %lldus", __func__, PABLO_KTIME_US_DELTA_NOW(st));

p_err:
	IXC_MUTEX_UNLOCK(cis->ixc_lock);

	return ret;
}

int sensor_gc02m1_cis_get_digital_gain(struct v4l2_subdev *subdev, u32 *dgain)
{
	int ret = 0;
	struct is_cis *cis;
	struct i2c_client *client;

	u8 digital_gain = 0;
	u8 digital_gain_dec = 0;
	ktime_t st = ktime_get();

	FIMC_BUG(!subdev);
	FIMC_BUG(!dgain);

	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		return ret;
	}

	IXC_MUTEX_LOCK(cis->ixc_lock);

	ret = cis->ixc_ops->addr8_write8(client, 0xfe, 0x00);
	if (ret < 0)
		 goto p_err;

	/* Digital gain int*/
	ret = cis->ixc_ops->addr8_read8(client, 0xb1, &digital_gain);
	if (ret < 0)
		goto p_err;

	/* Digital gain decimal*/
	ret = cis->ixc_ops->addr8_read8(client, 0xb2, &digital_gain_dec);
	if (ret < 0)
		goto p_err;

	*dgain = sensor_cis_calc_dgain_permile(digital_gain);

	dbg_sensor(1, "[MOD:D:%d] %s, cur_dgain = %d us, digital_gain(%#x)\n",
			cis->id, __func__, *dgain, digital_gain);

	if (IS_ENABLED(DEBUG_SENSOR_TIME))
		dbg_sensor(1, "[%s] time %lldus", __func__, PABLO_KTIME_US_DELTA_NOW(st));

p_err:
	IXC_MUTEX_UNLOCK(cis->ixc_lock);

	return ret;
}

int sensor_gc02m1_cis_set_totalgain(struct v4l2_subdev *subdev, struct ae_param *target_exposure,
	struct ae_param *again, struct ae_param *dgain)
{
	int ret = 0;
	struct is_cis *cis;
	cis_shared_data *cis_data;
	struct is_core *core;

	u16 origin_exp = 0;
	u16 multiple_exp = 0;
	u64 vt_pix_clk_freq_khz = 0;
	u32 line_length_pck = 0;
	u32 min_fine_int = 0;
	u32 input_again = 0;

	FIMC_BUG(!subdev);
	FIMC_BUG(!target_exposure);
	FIMC_BUG(!again);
	FIMC_BUG(!dgain);

	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	if ((target_exposure->long_val <= 0) || (again->val <= 0)) {
		err("[%s] invalid target exposure & again(%d, %d)\n", __func__,
				target_exposure->long_val, again->val);
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	core = is_get_is_core();
	if (!core) {
		err("core device is null");
		ret = -EINVAL;
		goto p_err;
	}

	if (sensor_gc02m1_check_master_stream_off(core)) {
		dbg_sensor(1, "%s: Master cam did not enter in stream_on yet. Stop updating exposure time", __func__);
		return ret;
	}

	dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), target long(%d), again(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, target_exposure->long_val, again->val);

	vt_pix_clk_freq_khz = cis_data->pclk / 1000;
	line_length_pck = cis_data->line_length_pck;
	min_fine_int = cis_data->min_fine_integration_time;

	dbg_sensor(1, "[MOD:D:%d] %s, vt_pix_clk_freq_khz (%llu), line_length_pck(%d), min_fine_int (%d)\n",
		cis->id, __func__, vt_pix_clk_freq_khz, line_length_pck, min_fine_int);

	origin_exp = (u16)(((target_exposure->long_val * vt_pix_clk_freq_khz) - min_fine_int) / (line_length_pck * 1000));

	if (origin_exp > cis_data->max_coarse_integration_time) {
		origin_exp = cis_data->max_coarse_integration_time;
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), origin exp(%d) max\n", cis->id, __func__,
			cis_data->sen_vsync_count, origin_exp);
	}

	if (origin_exp < cis_data->min_coarse_integration_time) {
		origin_exp = cis_data->min_coarse_integration_time;
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), origin exp(%d) min\n", cis->id, __func__,
			cis_data->sen_vsync_count, origin_exp);
	}

	/* Exposure time should be a multiple of 4 */
	multiple_exp = MULTIPLE_OF_4(origin_exp);
	if (multiple_exp > 0x3ffc) {
		warn("%s: long_coarse_int is above the maximum value : 0x%04x (should be lower than 0x3ffc)\n", __func__, multiple_exp);
		multiple_exp = 0x3ffc;
	}

	/* Set Exposure Time */
	ret = sensor_gc02m1_cis_set_exposure_time(subdev, multiple_exp);
	if (ret < 0) {
		err("[%s] sensor_gc02m1_cis_set_exposure_time fail\n", __func__);
		goto p_err;
	}

	/* Set Analog & Digital gains */
	/* Following formular was changed to correct build error (Previous one is ((float)origin_exp/(float)multiple_exp)*again->val) */
	input_again = (origin_exp * 100000 / multiple_exp) * again->val / 100000;
	ret = sensor_gc02m1_cis_set_analog_gain(subdev, input_again);
	if (ret < 0) {
		err("[%s] sensor_gc02m1_cis_set_analog_gain fail\n", __func__);
		goto p_err;
	}

	dbg_sensor(1, "[MOD:D:%d] %s, frame_length_lines:%d(%#x), multiple_exp:%d(%#x), input_again:%d(%#x)\n",
		cis->id, __func__, cis_data->frame_length_lines, cis_data->frame_length_lines,
		multiple_exp, multiple_exp, input_again, input_again);

p_err:

	return ret;
}


int sensor_gc02m1_cis_wait_streamoff(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u32 poll_time_ms = 0;
	struct is_cis *cis = sensor_cis_get_cis(subdev);
	struct i2c_client *client;
	cis_shared_data *cis_data;
	u16 sensor_fcount = 0;
	int sensor_delay = 0;

	cis_data = cis->cis_data;
	client = cis->client;

	/*
	 * sensor frame counter (0xE1, 0xE2)
	 * stream on (0x0000 ~ 0xFFFF), stream off (0x0000)
	 */
	if (sensor_gc02m1_fcount == 0) {
		sensor_delay = cis_data->cur_frame_us_time/1000;

		if (sensor_delay == 0)
			sensor_delay = 33; /* 30 fps */
		else if (sensor_delay > 125)
			sensor_delay = 125; /* 8 fps */

		msleep(sensor_delay);
		poll_time_ms += sensor_delay;
		info("%s: add 1 frame delay %d ms\n", __func__, sensor_delay);
	}

	do {
		/* Page Selection */

		sensor_fcount = sensor_gc02m1_cis_get_framecount(cis);

		if (sensor_fcount == 0) /* stream off done */
			break;

		usleep_range(POLL_TIME_US, POLL_TIME_US + 10);
		poll_time_ms += POLL_TIME_MS;

		dbg_sensor(1, "[MOD:D:%d] %s, sensor_fcount(%d), (poll_time_ms(%d) < STREAM_OFF_POLL_TIME_MS(%d))\n",
				cis->id, __func__, sensor_fcount, poll_time_ms, STREAM_OFF_POLL_TIME_MS);
	} while (poll_time_ms < STREAM_OFF_POLL_TIME_MS);

	if (poll_time_ms < STREAM_OFF_POLL_TIME_MS)
		info("%s: finished after %d ms\n", __func__, poll_time_ms);
	else
		warn("%s: finished : polling timeout occured after %d ms\n", __func__, poll_time_ms);

	return ret;

}

int sensor_gc02m1_cis_wait_streamon(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u32 poll_time_ms = 0;
	struct is_cis *cis = sensor_cis_get_cis(subdev);
	struct i2c_client *client;
	cis_shared_data *cis_data;
	u16 cur_frame_count = 0;
	u16 next_frame_count = 0;

	cis_data = cis->cis_data;
	client = cis->client;

	/* Page Selection */

	cur_frame_count = sensor_gc02m1_cis_get_framecount(cis);

	/* Checking stream on */
	do {
		/* Page Selection */

		next_frame_count = sensor_gc02m1_cis_get_framecount(cis);

		if (cur_frame_count != next_frame_count)
			break;

		cur_frame_count = next_frame_count;

		usleep_range(POLL_TIME_US, POLL_TIME_US + 10);
		poll_time_ms += POLL_TIME_MS;
		dbg_sensor(1, "[MOD:D:%d] %s, sensor_fcount(%d), (poll_time_ms(%d) < STREAM_ON_POLL_TIME_MS(%d))\n",
				cis->id, __func__, cur_frame_count, poll_time_ms, STREAM_ON_POLL_TIME_MS);
	} while (poll_time_ms < STREAM_ON_POLL_TIME_MS);

	if (poll_time_ms < STREAM_ON_POLL_TIME_MS)
		info("%s: finished after %d ms\n", __func__, poll_time_ms);
	else
		warn("%s: finished : polling timeout occured after %d ms\n", __func__, poll_time_ms);

	return ret;
}

static struct is_cis_ops cis_ops = {
	.cis_init = sensor_gc02m1_cis_init,
	.cis_log_status = sensor_gc02m1_cis_log_status,
	.cis_set_global_setting = sensor_gc02m1_cis_set_global_setting,
	.cis_mode_change = sensor_gc02m1_cis_mode_change,
	.cis_stream_on = sensor_gc02m1_cis_stream_on,
	.cis_stream_off = sensor_gc02m1_cis_stream_off,
	.cis_wait_streamon = sensor_gc02m1_cis_wait_streamon,
	.cis_wait_streamoff = sensor_gc02m1_cis_wait_streamoff,
	.cis_data_calculation = sensor_cis_data_calculation,
	.cis_set_exposure_time = NULL,
	.cis_get_min_exposure_time = sensor_cis_get_min_exposure_time,
	.cis_get_max_exposure_time = sensor_cis_get_max_exposure_time,
	.cis_adjust_frame_duration = sensor_cis_adjust_frame_duration,
	.cis_set_frame_duration = sensor_gc02m1_cis_set_frame_duration,
	.cis_set_frame_rate = sensor_cis_set_frame_rate,
	.cis_adjust_analog_gain = sensor_cis_adjust_analog_gain,
	.cis_set_analog_gain = NULL,
	.cis_get_analog_gain = sensor_gc02m1_cis_get_analog_gain,
	.cis_get_min_analog_gain = sensor_cis_get_min_analog_gain,
	.cis_get_max_analog_gain = sensor_cis_get_max_analog_gain,
	.cis_calc_again_code = sensor_gc02m1_cis_calc_again_code,
	.cis_calc_again_permile = sensor_gc02m1_cis_calc_again_permile,
	.cis_calc_dgain_code = NULL,
	.cis_calc_dgain_permile = sensor_cis_calc_dgain_permile,
	.cis_set_digital_gain = NULL,
	.cis_get_digital_gain = sensor_gc02m1_cis_get_digital_gain,
	.cis_get_min_digital_gain = sensor_cis_get_min_digital_gain,
	.cis_get_max_digital_gain = sensor_cis_get_max_digital_gain,
	.cis_compensate_gain_for_extremely_br = sensor_cis_compensate_gain_for_extremely_br,
	.cis_set_totalgain = sensor_gc02m1_cis_set_totalgain,
	.cis_check_rev_on_init = sensor_gc02m1_cis_check_rev,
	.cis_set_initial_exposure = sensor_cis_set_initial_exposure,
};

int cis_gc02m1_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret;
	struct is_cis *cis = NULL;
	struct is_device_sensor_peri *sensor_peri = NULL;
	char const *setfile;
	struct device_node *dnode = client->dev.of_node;

	ret = sensor_cis_probe(client, &(client->dev), &sensor_peri, I2C_TYPE);
	if (ret) {
		probe_info("%s: sensor_cis_probe ret(%d)\n", __func__, ret);
		return ret;
	}

	cis = &sensor_peri->cis;
	cis->ctrl_delay = N_PLUS_TWO_FRAME;
	cis->cis_ops = &cis_ops;

	/* belows are depend on sensor cis. MUST check sensor spec */
	switch (cis->orientation) {
	case SENSOR_FLIP_X:
		cis->bayer_order = OTF_INPUT_ORDER_BAYER_GR_BG;
		break;
	case SENSOR_FLIP_Y:
		cis->bayer_order = OTF_INPUT_ORDER_BAYER_GB_RG;
		break;
	case SENSOR_FLIP_XY:
		cis->bayer_order = OTF_INPUT_ORDER_BAYER_BG_GR;
		break;
	default:
		cis->bayer_order = OTF_INPUT_ORDER_BAYER_RG_GB;
		break;
	}

	cis->reg_addr = &sensor_gc02m1_reg_addr;

	/* Use total gain instead of using dgain */
	cis->use_dgain = false;
	cis->hdr_ctrl_by_again = false;
	cis->use_total_gain = true;

	ret = of_property_read_string(dnode, "setfile", &setfile);
	if (ret) {
		err("setfile index read fail(%d), take default setfile!!", ret);
		setfile = "default";
	}

	if (strcmp(setfile, "default") == 0 || strcmp(setfile, "setA") == 0)
		probe_info("[%s] setfile_A mclk: 19.2Mhz\n", __func__);
	else
		err("setfile index out of bound");

	cis->sensor_info = &sensor_gc02m1_info_A;

	probe_info("%s done\n", __func__);
	return ret;
}

static const struct of_device_id sensor_cis_gc02m1_match[] = {
	{
		.compatible = "samsung,exynos-is-cis-gc02m1",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sensor_cis_gc02m1_match);

static const struct i2c_device_id sensor_cis_gc02m1_idt[] = {
	{ SENSOR_NAME, 0 },
	{},
};

static struct i2c_driver sensor_cis_gc02m1_driver = {
	.probe	= cis_gc02m1_probe,
	.driver = {
		.name	= SENSOR_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = sensor_cis_gc02m1_match,
		.suppress_bind_attrs = true,
	},
	.id_table = sensor_cis_gc02m1_idt,
};

#ifdef MODULE
builtin_i2c_driver(sensor_cis_gc02m1_driver);
#else
static int __init sensor_cis_gc02m1_init(void)
{
	int ret;

	ret = i2c_add_driver(&sensor_cis_gc02m1_driver);
	if (ret)
		err("failed to add %s driver: %d\n",
			sensor_cis_gc02m1_driver.driver.name, ret);

	return ret;
}
late_initcall_sync(sensor_cis_gc02m1_init);
#endif

MODULE_LICENSE("GPL");
MODULE_SOFTDEP("pre: fimc-is");
