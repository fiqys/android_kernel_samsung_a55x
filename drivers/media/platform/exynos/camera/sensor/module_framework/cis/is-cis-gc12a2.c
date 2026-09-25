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
#include "is-cis-gc12a2.h"
#include "is-cis-gc12a2-setA.h"

#include "is-helper-ixc.h"

#define SENSOR_NAME "GC12A2"
/* #define DEBUG_GC12A2_PLL */

#define MAX_WAIT_STREAM_ON_CNT (250)
#define MAX_WAIT_STREAM_OFF_CNT (250)

/*************************************************
 *  [GC12A2 Analog / Digital gain formular]
 *
 *  Analog / Digital Gain = (Reg value) / 1024
 *
 *  Analog / Digital Gain Range = x1.0 to x16.0
 *
 *************************************************/

u32 sensor_gc12a2_cis_calc_gain_code(u32 permile)
{
	return ((permile * 8192) / 1000);
}

u32 sensor_gc12a2_cis_calc_gain_permile(u32 code)
{
	return ((code * 1000 / 8192));
}

int sensor_gc12a2_cis_init_state(struct v4l2_subdev *subdev, int mode)
{
	struct is_cis *cis = sensor_cis_get_cis(subdev);

	cis->cis_data->stream_on = false;
	cis->mipi_clock_index_cur = CAM_MIPI_NOT_INITIALIZED;
	cis->mipi_clock_index_new = CAM_MIPI_NOT_INITIALIZED;
	cis->cis_data->cur_pattern_mode = SENSOR_TEST_PATTERN_MODE_OFF;

	return 0;
}

int sensor_gc12a2_cis_init(struct v4l2_subdev *subdev)
{
	int ret = 0;
	ktime_t st = ktime_get();
	struct is_cis *cis = sensor_cis_get_cis(subdev);

	cis->cis_data->stream_on = false;
	cis->cis_data->cur_width = cis->sensor_info->max_width;
	cis->cis_data->cur_height = cis->sensor_info->max_height;
	cis->cis_data->low_expo_start = 33000;
	cis->need_mode_change = false;

	cis->mipi_clock_index_cur = CAM_MIPI_NOT_INITIALIZED;
	cis->mipi_clock_index_new = CAM_MIPI_NOT_INITIALIZED;
	cis->cis_data->sens_config_index_pre = SENSOR_GC12A2_MODE_MAX;
	cis->cis_data->sens_config_index_cur = 0;

	CALL_CISOPS(cis, cis_data_calculation, subdev, cis->cis_data->sens_config_index_cur);

	if (IS_ENABLED(DEBUG_SENSOR_TIME))
		dbg_sensor(1, "[%s] time %lldus", __func__, PABLO_KTIME_US_DELTA_NOW(st));

	return ret;
}

static const struct is_cis_log log_gc12a2[] = {
	{I2C_READ, 16, 0x0000, 0, "model_id"},
	{I2C_READ, 8, 0x0002, 0, "rev_number"},
	{I2C_READ, 16, 0x115, 0, "frame_count"},
	{I2C_READ, 8, 0x0100, 0, "0x0100"},
	{I2C_READ, 8, 0x0102, 0, "0x0102"},
	{I2C_READ, 16, 0x0136, 0, ""},
	{I2C_READ, 16, 0x0202, 0, "cit"},
	{I2C_READ, 16, 0x0205, 0, "again"},
	{I2C_READ, 16, 0x0340, 0, "fll"},
	{I2C_READ, 16, 0x0342, 0, "llp"},
};

int sensor_gc12a2_cis_log_status(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct is_cis *cis = sensor_cis_get_cis(subdev);

	sensor_cis_log_status(cis, log_gc12a2, ARRAY_SIZE(log_gc12a2), (char *)__func__);

	return ret;
}

int sensor_gc12a2_cis_set_global_setting(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct is_cis *cis = sensor_cis_get_cis(subdev);
	struct sensor_gc12a2_private_data *priv = (struct sensor_gc12a2_private_data *)cis->sensor_info->priv;

	info("[%s] start\n", __func__);

	ret = sensor_cis_write_registers_locked(subdev, priv->global);
	if (ret < 0)
		err("global setting fail!!");

	info("[%s] done\n", __func__);

	return ret;
}

int sensor_gc12a2_cis_mode_change(struct v4l2_subdev *subdev, u32 mode)
{
	int ret = 0;
	const struct sensor_cis_mode_info *mode_info;
	struct is_cis *cis = sensor_cis_get_cis(subdev);

	if (mode >= cis->sensor_info->mode_count) {
		err("invalid mode(%d)!!", mode);
		return -EINVAL;
	}

	cis->mipi_clock_index_cur = CAM_MIPI_NOT_INITIALIZED;

	info("[%s] sensor mode(%d)\n", __func__, mode);

	mode_info = cis->sensor_info->mode_infos[mode];

	ret = sensor_cis_write_registers_locked(subdev, mode_info->setfile);
	if (ret < 0) {
		err("sensor_setfiles fail!!");
		return ret;
	}

	cis->cis_data->sens_config_index_pre = mode;

	info("[%s] mode changed(%d)\n", __func__, mode);

	return ret;
}

int sensor_gc12a2_cis_stream_on(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct is_device_sensor *device;
	struct is_cis *cis = sensor_cis_get_cis(subdev);
	cis_shared_data *cis_data;
	ktime_t st = ktime_get();

	device = (struct is_device_sensor *)v4l2_get_subdev_hostdata(subdev);
	WARN_ON(!device);

	cis_data = cis->cis_data;

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	is_vendor_set_mipi_clock(device);

	IXC_MUTEX_LOCK(cis->ixc_lock);
	/* Sensor stream on */
	cis->ixc_ops->write8(cis->client, 0x0100, 0x01);
	IXC_MUTEX_UNLOCK(cis->ixc_lock);

	info("%s\n", __func__);

	cis_data->stream_on = true;

	if (IS_ENABLED(DEBUG_SENSOR_TIME))
		dbg_sensor(1, "[%s] time %lldus", __func__, PABLO_KTIME_US_DELTA_NOW(st));

	return ret;
}

int sensor_gc12a2_cis_stream_off(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct is_cis *cis = sensor_cis_get_cis(subdev);
	u16 frame_count = 0;
	ktime_t st = ktime_get();

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	IXC_MUTEX_LOCK(cis->ixc_lock);
	cis->ixc_ops->read16(cis->client, 0x115, &frame_count);
	/* Sensor stream off */
	ret = cis->ixc_ops->write8(cis->client, 0x0100, 0x00);
	IXC_MUTEX_UNLOCK(cis->ixc_lock);

	info("%s done frame_count(%d)\n", __func__, frame_count);

	cis->cis_data->stream_on = false;

	if (IS_ENABLED(DEBUG_SENSOR_TIME))
		dbg_sensor(1, "[%s] time %lldus\n", __func__, PABLO_KTIME_US_DELTA_NOW(st));

	return ret;
}

int sensor_gc12a2_cis_wait_streamon(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u32 polling_cnt = 0;
	struct is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;
	u16 prev_frame_value = 0;
	u16 cur_frame_value = 0;

	FIMC_BUG(!subdev);

	cis = (struct is_cis *)v4l2_get_subdevdata(subdev);
	if (unlikely(!cis)) {
		err("cis is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;
	if (unlikely(!cis_data)) {
		err("cis_data is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	IXC_MUTEX_LOCK(cis->ixc_lock);
	ret = cis->ixc_ops->read16(client, 0x115, &prev_frame_value);
	IXC_MUTEX_UNLOCK(cis->ixc_lock);

	if (ret < 0) {
		err("i2c transfer fail addr(%x), val(%x), ret = %d\n", 0x115, prev_frame_value, ret);
		goto p_err;
	}

	/* Checking stream on */
	do {
		IXC_MUTEX_LOCK(cis->ixc_lock);
		ret = cis->ixc_ops->read16(client, 0x115, &cur_frame_value);
		IXC_MUTEX_UNLOCK(cis->ixc_lock);
		if (ret < 0) {
			err("i2c transfer fail addr(%x), val(%x), ret = %d\n", 0x115, cur_frame_value, ret);
			break;
		}

		if (cur_frame_value != prev_frame_value)
			break;

		prev_frame_value = cur_frame_value;

		usleep_range(2000, 2100);
		polling_cnt++;
		dbg_sensor(1, "[MOD:D:%d] %s, fcount(%d), (polling_cnt(%d) < MAX_WAIT_STREAM_ON_CNT(%d))\n",
				cis->id, __func__, prev_frame_value, polling_cnt, MAX_WAIT_STREAM_ON_CNT);
	} while (polling_cnt < MAX_WAIT_STREAM_ON_CNT);

	if (polling_cnt < MAX_WAIT_STREAM_ON_CNT)
		info("%s: finished after %d ms\n", __func__, polling_cnt);
	else
		warn("%s: finished : polling timeout occurred after %d ms\n", __func__, polling_cnt);

p_err:
	return ret;
}

int sensor_gc12a2_cis_wait_streamoff(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct is_cis *cis = sensor_cis_get_cis(subdev);
	u32 wait_cnt = 0;
	u8 mipi_streaming_state = 0;

	/* Checking stream off */
	do {
		IXC_MUTEX_LOCK(cis->ixc_lock);
		ret = cis->ixc_ops->read8(cis->client, 0x0102, &mipi_streaming_state);
		IXC_MUTEX_UNLOCK(cis->ixc_lock);

		if (mipi_streaming_state & 0x40) //0x102[6]: 1 streamoff
			break;

		usleep_range(2000, 2100);
		wait_cnt++;

		dbg_sensor(1, "[MOD:D:%d] %s, mipi_streaming_state(%x), (wait_limit(%d) < time_out(%d))\n",
				cis->id, __func__, mipi_streaming_state, wait_cnt, MAX_WAIT_STREAM_OFF_CNT);
	} while (wait_cnt < MAX_WAIT_STREAM_OFF_CNT);

	if (wait_cnt < MAX_WAIT_STREAM_OFF_CNT)
		info("%s: finished after wait_cnt(%d)\n", __func__, wait_cnt);
	else
		warn("%s: failed, wait_cnt(%d) > time_out_cnt(%d)\n", __func__, wait_cnt, MAX_WAIT_STREAM_OFF_CNT);

	return ret;
}

int sensor_gc12a2_cis_recover_stream_on(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct is_cis *cis = sensor_cis_get_cis(subdev);;

	info("%s start\n", __func__);

	ret = sensor_gc12a2_cis_set_global_setting(subdev);
	if (ret < 0)
		goto p_err;
	ret = sensor_gc12a2_cis_mode_change(subdev, cis->cis_data->sens_config_index_cur);
	if (ret < 0)
		goto p_err;
	ret = sensor_gc12a2_cis_stream_on(subdev);
	if (ret < 0)
		goto p_err;
	ret = sensor_gc12a2_cis_wait_streamon(subdev);
	if (ret < 0)
		goto p_err;

	info("%s end\n", __func__);
p_err:
	return ret;
}

int sensor_gc12a2_cis_set_test_pattern(struct v4l2_subdev *subdev, struct camera2_sensor_ctl *sensor_ctl)
{
	int ret = 0;
	struct is_cis *cis = sensor_cis_get_cis(subdev);
	const struct sensor_cis_mode_info *mode_info;
	u8 isp_out;

	dbg_sensor(1, "[MOD:D:%d] %s, cur_pattern_mode(%d), testPatternMode(%d)\n", cis->id, __func__,
		cis->cis_data->cur_pattern_mode, sensor_ctl->testPatternMode);

	if (cis->cis_data->cur_pattern_mode != sensor_ctl->testPatternMode) {
		if (sensor_ctl->testPatternMode == SENSOR_TEST_PATTERN_MODE_OFF) {
			mode_info = cis->sensor_info->mode_infos[cis->cis_data->sens_config_index_cur];
			if (mode_info->state_12bit == SENSOR_12BIT_STATE_REAL_12BIT)
				isp_out = 0x02;
			else
				isp_out = 0x04;

			info("[%d][%s] set DEFAULT pattern! (mode : %d) isp_out(0x%x)\n", cis->id, __func__, sensor_ctl->testPatternMode, isp_out);

			IXC_MUTEX_LOCK(cis->ixc_lock);
			cis->ixc_ops->write8(cis->client, 0x0089, isp_out);
			cis->ixc_ops->write8(cis->client, 0x0af4, 0x68);
			cis->ixc_ops->write8(cis->client, 0x0af5, 0x01);
			cis->ixc_ops->write8(cis->client, 0x00ce, 0x09);
			cis->ixc_ops->write8(cis->client, 0x00cf, 0x10);
			cis->ixc_ops->write8(cis->client, 0x0af5, 0x00);
			IXC_MUTEX_UNLOCK(cis->ixc_lock);

			cis->cis_data->cur_pattern_mode = sensor_ctl->testPatternMode;
		} else if (sensor_ctl->testPatternMode == SENSOR_TEST_PATTERN_MODE_BLACK) {
			info("[%d][%s] set BLACK pattern! (mode :%d), Data : 0x(%x, %x, %x, %x)\n",
				cis->id, __func__, sensor_ctl->testPatternMode,
				(unsigned short)sensor_ctl->testPatternData[0],
				(unsigned short)sensor_ctl->testPatternData[1],
				(unsigned short)sensor_ctl->testPatternData[2],
				(unsigned short)sensor_ctl->testPatternData[3]);

			IXC_MUTEX_LOCK(cis->ixc_lock);
			cis->ixc_ops->write8(cis->client, 0x0089, 0x00);
			cis->ixc_ops->write8(cis->client, 0x00cf, 0x00);
			cis->ixc_ops->write8(cis->client, 0x0af4, 0x68);
			cis->ixc_ops->write8(cis->client, 0x0af5, 0x01);
			cis->ixc_ops->write8(cis->client, 0x00ce, 0x19);
			cis->ixc_ops->write8(cis->client, 0x0af5, 0x00);
			IXC_MUTEX_UNLOCK(cis->ixc_lock);

			cis->cis_data->cur_pattern_mode = sensor_ctl->testPatternMode;
		}
	}

	return ret;
}

static struct is_cis_ops cis_ops = {
	.cis_init = sensor_gc12a2_cis_init,
	.cis_init_state = sensor_gc12a2_cis_init_state,
	.cis_log_status = sensor_gc12a2_cis_log_status,
	.cis_set_global_setting = sensor_gc12a2_cis_set_global_setting,
	.cis_mode_change = sensor_gc12a2_cis_mode_change,
	.cis_stream_on = sensor_gc12a2_cis_stream_on,
	.cis_stream_off = sensor_gc12a2_cis_stream_off,
	.cis_wait_streamon = sensor_gc12a2_cis_wait_streamon,
	.cis_wait_streamoff = sensor_gc12a2_cis_wait_streamoff,
	.cis_data_calculation = sensor_cis_data_calculation,
	.cis_set_exposure_time = sensor_cis_set_exposure_time,
	.cis_get_min_exposure_time = sensor_cis_get_min_exposure_time,
	.cis_get_max_exposure_time = sensor_cis_get_max_exposure_time,
	.cis_adjust_frame_duration = sensor_cis_adjust_frame_duration,
	.cis_set_frame_duration = sensor_cis_set_frame_duration,
	.cis_set_frame_rate = sensor_cis_set_frame_rate,
	.cis_adjust_analog_gain = sensor_cis_adjust_analog_gain,
	.cis_set_analog_gain = sensor_cis_set_analog_gain,
	.cis_get_analog_gain = sensor_cis_get_analog_gain,
	.cis_get_min_analog_gain = sensor_cis_get_min_analog_gain,
	.cis_get_max_analog_gain = sensor_cis_get_max_analog_gain,
	.cis_calc_again_code = sensor_gc12a2_cis_calc_gain_code,
	.cis_calc_again_permile = sensor_gc12a2_cis_calc_gain_permile,
	.cis_calc_dgain_code = sensor_gc12a2_cis_calc_gain_code,
	.cis_calc_dgain_permile = sensor_gc12a2_cis_calc_gain_permile,
	.cis_set_digital_gain = sensor_cis_set_digital_gain,
	.cis_get_digital_gain = sensor_cis_get_digital_gain,
	.cis_get_min_digital_gain = sensor_cis_get_min_digital_gain,
	.cis_get_max_digital_gain = sensor_cis_get_max_digital_gain,
	.cis_compensate_gain_for_extremely_br = sensor_cis_compensate_gain_for_extremely_br,
	.cis_set_initial_exposure = sensor_cis_set_initial_exposure,

	.cis_recover_stream_on = sensor_gc12a2_cis_recover_stream_on,

	.cis_check_rev_on_init = sensor_cis_check_rev_on_init,
	.cis_set_test_pattern = sensor_gc12a2_cis_set_test_pattern,
};

int cis_gc12a2_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
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
	cis->bayer_order = OTF_INPUT_ORDER_BAYER_GR_BG;
	cis->reg_addr = &sensor_gc12a2_reg_addr;
	cis->use_dgain = false;
	cis->hdr_ctrl_by_again = false;

	cis->use_initial_ae = of_property_read_bool(dnode, "use_initial_ae");
	probe_info("%s use initial_ae(%d)\n", __func__, cis->use_initial_ae);

	ret = of_property_read_string(dnode, "setfile", &setfile);
	if (ret) {
		err("setfile index read fail(%d), take default setfile!!", ret);
		setfile = "default";
	}

	if (strcmp(setfile, "default") == 0 || strcmp(setfile, "setA") == 0)
		probe_info("[%s] setfile_A mclk: 19.2Mhz \n", __func__);
	else
		err("setfile index out of bound, take default (setfile_A mclk: 19.2Mhz)");

	cis->sensor_info = &sensor_gc12a2_info_A;

	is_vendor_set_mipi_mode(cis);

	probe_info("%s done\n", __func__);

	return ret;
}

void cis_gc12a2_remove(struct i2c_client *client)
{
	return;
}

static const struct of_device_id sensor_cis_gc12a2_match[] = {
	{
		.compatible = "samsung,exynos-is-cis-gc12a2",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sensor_cis_gc12a2_match);

static const struct i2c_device_id sensor_cis_gc12a2_idt[] = {
	{ SENSOR_NAME, 0 },
	{},
};

static struct i2c_driver sensor_cis_gc12a2_driver = {
	.driver = {
		.name	= SENSOR_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = sensor_cis_gc12a2_match
	},
	.probe	= cis_gc12a2_probe,
	.remove	= cis_gc12a2_remove,
	.id_table = sensor_cis_gc12a2_idt
};

#ifdef MODULE
builtin_i2c_driver(sensor_cis_gc12a2_driver);
#else
static int __init sensor_cis_gc12a2_init(void)
{
	int ret;

	ret = i2c_add_driver(&sensor_cis_gc12a2_driver);
	if (ret)
		err("failed to add %s driver: %d\n",
			sensor_cis_gc12a2_driver.driver.name, ret);

	return ret;
}
late_initcall_sync(sensor_cis_gc12a2_init);
#endif

MODULE_LICENSE("GPL");
MODULE_SOFTDEP("pre: fimc-is");
