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
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/file.h>
#include <soc/samsung/exynos-pmu-if.h>

#include <exynos-is-sensor.h>
#include "is-device-sensor-peri.h"
#include "pablo-hw-api-common.h"
#include "is-hw-api-ois-mcu.h"
#include "is-vendor-ois-internal-mcu.h"
#include "is-vendor-ois.h"
#include "is-vendor-ois-core.h"
#include "is-vendor-ois-reg.h"
#include "is-device-ois_common.h"
#ifdef CONFIG_AF_HOST_CONTROL
#include "is-device-af.h"
#endif
#include "is-vendor-private.h"
#include "is-sec-define.h"

static const struct v4l2_subdev_ops subdev_ops;

int is_ois_internal_mcu_read_u8(int cmd, u8 *data) {
	struct is_core *core = is_get_is_core();
	struct ois_mcu_dev *mcu = core->mcu;
	void __iomem *reg = mcu->regs[OM_REG_CORE];

	*data = is_hw_get_reg_u8(reg, &ois_mcu_regs[cmd]);

	dbg_ois("[GET_REG] reg:[%s][0x%04X], reg_value(R):[0x%02X]\n",
		ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset, *data);

	return 0;
}

int is_ois_internal_mcu_read_multi(int cmd, u8 *data, size_t size)
{
	int i;

	for (i = 0; i < size; i++) {
		dbg_ois("[GET_REG] reg:[%s][0x%04X], reg_value(R):[0x%02X]\n",
			ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset + i, data[i]);
	}

	return 0;
}

int is_ois_internal_mcu_read_u16(int cmd, u8 *data)
{
	return is_ois_internal_mcu_read_multi(cmd, data, 2);
}

int is_ois_internal_mcu_write_u8(int cmd, u8 data)
{
	struct is_core *core = is_get_is_core();
	struct ois_mcu_dev *mcu = core->mcu;
	void __iomem *reg = mcu->regs[OM_REG_CORE];

	is_hw_set_reg_u8(reg, &ois_mcu_regs[cmd], data);

	dbg_ois("[SET_REG] reg:[%s][0x%04X], reg_value(W):[0x%02X]\n",
		ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset, data);

	return 0;
}

int is_ois_internal_mcu_write_multi(int cmd, u8 *data, size_t size)
{
	int i;

	for (i = 0 ; i < size; i++) {
		dbg_ois("[SET_REG] reg:[%s][0x%04X], reg_value(W):[0x%02X]\n",
			ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset + i, data[i]);
	}

	return 0;
}

int is_ois_internal_mcu_write_u16(int cmd, u8 *data)
{
	return is_ois_internal_mcu_write_multi(cmd, data, 2);
}

static struct ois_comm_ops internal_mcu_ops = {
	.read_u8 = is_ois_internal_mcu_read_u8,
	.read_u16 = is_ois_internal_mcu_read_u16,
	.read_multi = is_ois_internal_mcu_read_multi,
	.write_u8 = is_ois_internal_mcu_write_u8,
	.write_u16 = is_ois_internal_mcu_write_u16,
	.write_multi = is_ois_internal_mcu_write_multi,
};

static int is_vendor_ois_clk_get(struct ois_mcu_dev *mcu)
{
	mcu->clk = clk_get(mcu->dev, "user_mux");
	mcu->spi_clk = clk_get(mcu->dev, "ipclk_spi");
	if (!IS_ERR(mcu->clk) && !IS_ERR(mcu->spi_clk))
		return 0;
	else
		goto err;

err:
	if (PTR_ERR(mcu->clk) != -ENOENT) {
		dev_err(mcu->dev, "Failed to get 'user_mux' clock: %ld",
			PTR_ERR(mcu->clk));
		return PTR_ERR(mcu->clk);
	}
	dev_info(mcu->dev, "[@] 'user_mux' clock is not present\n");

	if (PTR_ERR(mcu->spi_clk) != -ENOENT) {
		dev_err(mcu->dev, "Failed to get 'spiclk' clock: %ld",
			PTR_ERR(mcu->spi_clk));
		return PTR_ERR(mcu->spi_clk);
	}
	dev_info(mcu->dev, "[@] 'spiclk' clock is not present\n");

	return -EIO;
}

static void is_vendor_ois_clk_put(struct ois_mcu_dev *mcu)
{
	if (!IS_ERR(mcu->clk))
		clk_put(mcu->clk);

	if (!IS_ERR(mcu->spi_clk))
		clk_put(mcu->spi_clk);
}

static int is_vendor_ois_clk_enable(struct ois_mcu_dev *mcu)
{
	int ret = 0;

	if (IS_ERR(mcu->clk)) {
		dev_info(mcu->dev, "[@] 'user_mux' clock is not present\n");
		return -EIO;
	}

	ret = clk_prepare_enable(mcu->clk);
	if (ret) {
		dev_err(mcu->dev, "%s: failed to enable clk (err %d)\n",
					__func__, ret);
		return ret;
	}

	if (IS_ERR(mcu->spi_clk)) {
		dev_info(mcu->dev, "[@] 'spi_clk' clock is not present\n");
		return -EIO;
	}

	/* set spi clock to 10Mhz */
	clk_set_rate(mcu->spi_clk, 19200000);
	ret = clk_prepare_enable(mcu->spi_clk);
	if (ret) {
		dev_err(mcu->dev, "%s: failed to enable clk (err %d)\n",
					__func__, ret);
		return ret;
	}

	return ret;
}

static void is_vendor_ois_clk_disable(struct ois_mcu_dev *mcu)
{
	if (!IS_ERR(mcu->clk))
		clk_disable_unprepare(mcu->clk);

	if (!IS_ERR(mcu->spi_clk))
		clk_disable_unprepare(mcu->spi_clk);
}

static int is_vendor_ois_runtime_resume(struct device *dev)
{
	struct ois_mcu_dev *mcu = dev_get_drvdata(dev);
	int ret = 0;

	info_mcu("%s E\n", __func__);

	ret = is_vendor_ois_clk_get(mcu);
	if (ret) {
		err_mcu("Failed to get ois mcu clk");
		return ret;
	}

	ret = is_vendor_ois_clk_enable(mcu);

	ret |= __is_mcu_pmu_control(1);
	usleep_range(1000, 1100);

	__is_mcu_hw_enable(mcu->regs[OM_REG_SFR]);
	ret |= __is_mcu_hw_reset_peri(mcu->regs[OM_REG_PERI1], 0); /* clear USI reset reg USI10 */
	usleep_range(2000, 2100);
	ret |= __is_mcu_hw_reset_peri(mcu->regs[OM_REG_PERI2], 0); /* clear USI reset reg USI09 */
	usleep_range(2000, 2100);
	ret |= __is_mcu_hw_set_init_peri(mcu->regs[OM_REG_PERI_SETTING]); /* GPP1, GPP2 setting */
	ret |= __is_mcu_hw_set_clock_peri(mcu->regs[OM_REG_PERI1]); /* set i2c clock to 1MH */

	clear_bit(OM_HW_SUSPENDED, &mcu->state);

	info_mcu("%s X\n", __func__);

	return ret;
}

static int is_vendor_ois_runtime_suspend(struct device *dev)
{
	struct ois_mcu_dev *mcu = dev_get_drvdata(dev);
	int ret = 0;

	info_mcu("%s E\n", __func__);

	__is_mcu_hw_disable(mcu->regs[OM_REG_SFR]);
	ret |= __is_mcu_hw_set_clear_peri(mcu->regs[OM_REG_PERI_SETTING]); /* GPP1, GPP2 setting */
	usleep_range(2000, 2100); //TEMP_2020 Need to be checked
	ret |= __is_mcu_hw_reset_peri(mcu->regs[OM_REG_PERI1], 1); /* clear USI reset reg USI10 */
	ret |= __is_mcu_hw_reset_peri(mcu->regs[OM_REG_PERI2], 1); /* clear USI reset reg USI09 */
	ret |= __is_mcu_hw_clear_peri(mcu->regs[OM_REG_PERI1]);
	ret |= __is_mcu_hw_clear_peri(mcu->regs[OM_REG_PERI2]);

	is_vendor_ois_clk_disable(mcu);
	is_vendor_ois_clk_put(mcu);

	ret |= __is_mcu_pmu_control(0);
	/* wait for ois cpu power down, before csis block off (pm_runtime_put_sync). max 1ms. */
	usleep_range(1000, 1100);

	set_bit(OM_HW_SUSPENDED, &mcu->state);

	info_mcu("%s X\n", __func__);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int is_vendor_ois_resume(struct device *dev)
{
	/* TODO: */
	return 0;
}

static int is_vendor_ois_suspend(struct device *dev)
{
	struct ois_mcu_dev *mcu = dev_get_drvdata(dev);

	/* TODO: */
	if (!test_bit(OM_HW_SUSPENDED, &mcu->state))
		return -EBUSY;

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static irqreturn_t is_isr_is_vendor_ois(int irq, void *data)
{
	struct ois_mcu_dev *mcu;
	unsigned int state;

	mcu = (struct ois_mcu_dev *)data;
	state = is_mcu_hw_g_irq_state(mcu->regs[OM_REG_SFR], true);

	/* FIXME: temp log for testing */
	//info_mcu("IRQ: %d\n", state);
	if (is_mcu_hw_g_irq_type(state, MCU_IRQ_WDT)) {
		/* TODO: WDR IRQ handling */
		dbg_ois("IRQ: MCU_IRQ_WDT");
	}

	if (is_mcu_hw_g_irq_type(state, MCU_IRQ_WDT_RST)) {
		/* TODO: WDR RST handling */
		dbg_ois("IRQ: MCU_IRQ_WDT_RST");
	}

	if (is_mcu_hw_g_irq_type(state, MCU_IRQ_LOCKUP_RST)) {
		/* TODO: LOCKUP RST handling */
		dbg_ois("IRQ: MCU_IRQ_LOCKUP_RST");
	}

	if (is_mcu_hw_g_irq_type(state, MCU_IRQ_SYS_RST)) {
		/* TODO: SYS RST handling */
		dbg_ois("IRQ: MCU_IRQ_SYS_RST");
	}

	return IRQ_HANDLED;
}

/*
 * API functions
 */
int is_vendor_ois_power_ctrl(struct ois_mcu_dev *mcu, int on)
{
	int ret = 0;
#if defined(CONFIG_PM)
	int rpm_ret;
#endif
	BUG_ON(!mcu);

	info_mcu("%s E\n", __func__);

	if (on) {
		if (!test_bit(OM_HW_SUSPENDED, &mcu->state)) {
			warn_mcu("already power on");
			goto p_err;
		}
#if defined(CONFIG_PM)
		rpm_ret = pm_runtime_get_sync(mcu->dev);
		if (rpm_ret < 0)
			err_mcu("pm_runtime_get_sync() err: %d", rpm_ret);
#else
		ret = is_vendor_ois_runtime_resume(mcu->dev);
#endif
		clear_bit(OM_HW_SUSPENDED, &mcu->state);
		mcu->current_error_reg = 0;
		mcu->current_power_mode = OIS_POWER_MODE_NONE;
	} else {
		if (test_bit(OM_HW_SUSPENDED, &mcu->state)) {
			warn_mcu("already power off");
			goto p_err;
		}
#if defined(CONFIG_PM)
		info_mcu("%s: pm_runtime_put_sync start.\n", __func__);
		rpm_ret = pm_runtime_put_sync(mcu->dev);
		if (rpm_ret < 0)
			err_mcu("pm_runtime_put_sync() err: %d", rpm_ret);
		else
			info_mcu("%s: pm_runtime_put_sync end.\n", __func__);
#else
		ret = is_vendor_ois_runtime_suspend(mcu->dev);
#endif
		set_bit(OM_HW_SUSPENDED, &mcu->state);
		clear_bit(OM_HW_FW_LOADED, &mcu->state);
		clear_bit(OM_HW_RUN, &mcu->state);
		mcu->dev_ctrl_state = false;
	}

	info_mcu("%s: (%d) X\n", __func__, on);

p_err:
	return ret;
}

int is_vendor_ois_load_binary(struct ois_mcu_dev *mcu)
{
	int ret = 0;
	long size = 0;

	BUG_ON(!mcu);

	if (test_bit(OM_HW_FW_LOADED, &mcu->state)) {
		warn_mcu("already fw was loaded");
		return ret;
	}

	size = __is_mcu_load_fw(mcu->regs[OM_REG_CORE], mcu->dev);
	if (size <= 0)
		return -EINVAL;

	set_bit(OM_HW_FW_LOADED, &mcu->state);

	return ret;
}

int is_vendor_ois_core_ctrl(struct ois_mcu_dev *mcu, int on)
{
	int ret = 0;

	BUG_ON(!mcu);

	info_mcu("%s E\n", __func__);

	if (on) {
		if (test_bit(OM_HW_RUN, &mcu->state)) {
			warn_mcu("already started");
			return ret;
		}
		__is_mcu_hw_s_irq_enable(mcu->regs[OM_REG_SFR], 0x0);
		set_bit(OM_HW_RUN, &mcu->state);
	} else {
		if (!test_bit(OM_HW_RUN, &mcu->state)) {
			warn_mcu("already stopped");
			return ret;
		}
		clear_bit(OM_HW_RUN, &mcu->state);
	}

	ret = __is_mcu_core_control(mcu->regs[OM_REG_SFR], on);

	info_mcu("%s: %d X\n", __func__, on);

	return ret;
}

int is_vendor_ois_dump(struct ois_mcu_dev *mcu, int type)
{
	int ret = 0;

	BUG_ON(!mcu);

	if (test_bit(OM_HW_SUSPENDED, &mcu->state))
		return 0;

	switch (type) {
	case OM_REG_CORE:
		__is_mcu_hw_cr_dump(mcu->regs[OM_REG_CORE]);
		__is_mcu_hw_sram_dump(mcu->regs[OM_REG_CORE], __is_mcu_get_sram_size());
		break;
	case OM_REG_PERI1:
		__is_mcu_hw_peri1_dump(mcu->regs[OM_REG_PERI1]);
		break;
	case OM_REG_PERI2:
		__is_mcu_hw_peri2_dump(mcu->regs[OM_REG_PERI2]);
		break;
	case OM_REG_PERI_SETTING:
		__is_mcu_hw_show_peri_status(mcu->regs[OM_REG_PERI_SETTING]);
		break;
	default:
		err_mcu("undefined type (%d)", type);
	}

	return ret;
}

#if IS_ENABLED(CONFIG_CAMERA_HW_BIG_DATA)
void is_vendor_ois_get_hw_param(struct cam_hw_param *hw_param, u16 i2c_error_reg)
{
	if (i2c_error_reg & MCU_REAR_OIS_ERR_REG)
		is_sec_get_hw_param(&hw_param, SENSOR_POSITION_REAR);
	else if (i2c_error_reg & MCU_REAR_2ND_OIS_ERR_REG)
		is_sec_get_hw_param(&hw_param, SENSOR_POSITION_REAR2);
	else if (i2c_error_reg & MCU_REAR_3RD_OIS_ERR_REG)
		is_sec_get_hw_param(&hw_param, SENSOR_POSITION_REAR4);
}
#endif

int is_vendor_ois_af_get_position(struct v4l2_subdev *subdev, struct v4l2_control *ctrl)
{
	ctrl->value = ACTUATOR_STATUS_NO_BUSY;

	return 0;
}

int is_vendor_ois_af_valid_check(void)
{
	int i;
	struct is_sysfs_actuator *sysfs_actuator;

	sysfs_actuator = is_get_sysfs_actuator();

	if (sysfs_actuator->init_step > 0) {
		for (i = 0; i < sysfs_actuator->init_step; i++) {
			if (sysfs_actuator->init_positions[i] < 0) {
				warn("invalid position value, default setting to position");
				return 0;
			} else if (sysfs_actuator->init_delays[i] < 0) {
				warn("invalid delay value, default setting to delay");
				return 0;
			}
		}
	} else
		return 0;

	return sysfs_actuator->init_step;
}

int is_vendor_ois_af_write_position(struct ois_mcu_dev *mcu, u32 val)
{
	u8 val_high = 0, val_low = 0;

	dbg_ois("%s : E\n", __func__);

	val_high = (val & 0x0FFF) >> 4;
	val_low = (val & 0x000F) << 4;

#if defined(USE_TELE_OIS_AF_COMMON_INTERFACE)
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_POS1_REAR2_AF, &val_high);
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_POS2_REAR2_AF, &val_low);
#elif defined(USE_TELE2_OIS_AF_COMMON_INTERFACE)
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_POS1_REAR3_AF, &val_high);
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_POS2_REAR3_AF, &val_low);
#endif
	usleep_range(2000, 2100);

	dbg_ois("%s : [val : 0x%08x] X\n", __func__, val);

	return 0;
}

int is_vendor_ois_af_init(struct v4l2_subdev *subdev, u32 val)
{
	struct is_actuator *actuator = NULL;

	WARN_ON(!subdev);

	actuator = (struct is_actuator *)v4l2_get_subdevdata(subdev);
	WARN_ON(!actuator);

	actuator->position = val;

	dbg_ois("%s : X\n", __func__);

	return 0;
}

#ifdef USE_TELE2_OIS_AF_COMMON_INTERFACE
static int is_vendor_ois_af_init_position(struct ois_mcu_dev *mcu,
		struct is_actuator *actuator)
{
	int i;
	int ret = 0;
	int init_step = 0;
	struct is_sysfs_actuator *sysfs_actuator;

	sysfs_actuator = is_get_sysfs_actuator();
	init_step = is_vendor_ois_af_valid_check();

	if (init_step > 0) {
		for (i = 0; i < init_step; i++) {
			ret = is_vendor_ois_af_write_position(mcu, sysfs_actuator->init_positions[i]);
			if (ret < 0)
				goto p_err;

			mdelay(sysfs_actuator->init_delays[i]);
		}

		actuator->position = sysfs_actuator->init_positions[i];
	} else {
		/* wide, tele camera uses previous position at initial time */
		if (actuator->device == 1 || actuator->position == 0)
			actuator->position = MCU_ACT_DEFAULT_FIRST_POSITION;

		ret = is_vendor_ois_af_write_position(mcu, actuator->position);
		if (ret < 0)
			goto p_err;
	}

p_err:
	return ret;
}

int is_vendor_ois_af_set_active(struct v4l2_subdev *subdev, int enable)
{
	int ret = 0;
	struct ois_mcu_dev *mcu = NULL;
	struct is_core *core;
	struct is_mcu *is_mcu = NULL;
	struct is_actuator *actuator = NULL;

	WARN_ON(!subdev);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err_mcu("is_mcu is NULL");
		ret = -EINVAL;
		return ret;
	}

	core = is_get_is_core();
	if (!core) {
		err_mcu("core is null");
		return -EINVAL;
	}

	mcu = core->mcu;
	actuator = is_mcu->actuator;

	info_mcu("%s : E\n", __func__);

	if (enable) {
		if (mcu->need_af_delay) {
			/* delay for mcu init <-> af ctrl */
			usleep_range(10000, 11000);
			mcu->need_af_delay = false;
			info_mcu("%s : set af delay\n", __func__);
		}

		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CTRL_AF, MCU_AF_MODE_ACTIVE);
		usleep_range(10000, 11000);
		is_vendor_ois_af_init_position(mcu, actuator);
	} else {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CTRL_AF, MCU_AF_MODE_STANDBY);
	}

	info_mcu("%s : enable = %d X\n", __func__, enable);

	return 0;
}
#endif

int is_vendor_ois_af_set_position(struct v4l2_subdev *subdev, struct v4l2_control *ctrl)
{
	struct ois_mcu_dev *mcu = NULL;
	struct is_actuator *actuator = NULL;
	struct is_core *core;
	u32 position = 0;

	WARN_ON(!subdev);

	core = is_get_is_core();
	if (!core) {
		err_mcu("core is null");
		return -EINVAL;
	}

	actuator = (struct is_actuator *)v4l2_get_subdevdata(subdev);
	WARN_ON(!actuator);

	mcu = core->mcu;
	position = ctrl->value;

	is_vendor_ois_af_write_position(mcu, position);

	actuator->position = position;

	dbg_ois("%s : [position : 0x%08x] X\n", __func__, position);

	return 0;
}

long is_vendor_ois_actuator_ioctl(struct v4l2_subdev *subdev, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct v4l2_control *ctrl;

	ctrl = (struct v4l2_control *)arg;
	switch (cmd) {
	case SENSOR_IOCTL_ACT_S_CTRL:
		ret = is_vendor_ois_af_set_position(subdev, ctrl);
		if (ret) {
			err_mcu("mcu actuator_s_ctrl failed(%d)", ret);
			goto p_err;
		}
		break;
	case SENSOR_IOCTL_ACT_G_CTRL:
		ret = is_vendor_ois_af_get_position(subdev, ctrl);
		if (ret) {
			err_mcu("mcu actuator_g_ctrl failed(%d)", ret);
			goto p_err;
		}
		break;
	default:
		err_mcu("Unknown command(%#x)", cmd);
		ret = -EINVAL;
		goto p_err;
	}

p_err:
	return (long)ret;
}

#if defined(CAMERA_3RD_OIS) && defined(USE_TELE2_OIS_AF_COMMON_INTERFACE)
int is_vendor_ois_set_sleep_mode_folded_zoom(void)
{
	struct ois_mcu_dev *mcu = NULL;
	struct is_core *core;
	u8 state = 0;

	core = is_get_is_core();
	if (!core) {
		err_mcu("core is null");
		return -EINVAL;
	}

	mcu = core->mcu;

	state = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CTRL_AF);
	state = state & MCU_AF_MODE_STANDBY;
	if (state == MCU_AF_MODE_ACTIVE) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_CTRL_AF, MCU_AF_MODE_STANDBY);
		usleep_range(5000, 5010);
	}

	info_mcu("%s : set sleep mode folded zoom. state = %d\n", __func__, state);

	return 0;
}
#endif

void is_vendor_ois_device_ctrl(struct ois_mcu_dev *mcu, u8 value)
{
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_DEVCTRL, value);
}

int is_vendor_ois_set_dev_ctrl(struct v4l2_subdev *subdev, int forceMode)
{
	struct ois_mcu_dev *mcu = NULL;
	u8 val = 0;
	int retry = 200;

	mcu = (struct ois_mcu_dev*)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err_mcu("mcu subdev is NULL");
		return -EINVAL;
	}

	info_mcu("%s : E\n", __func__);

#if defined(CONFIG_SEC_FACTORY) //Factory timing issue.
	retry = 600;
#endif

	if (!(mcu->ois_wide_init
#if defined(CAMERA_2ND_OIS)
		|| mcu->ois_tele_init
#endif
#if defined(CAMERA_3RD_OIS)
		|| mcu->ois_tele2_init
#endif
	) || forceMode) {
		if (mcu->dev_ctrl_state == false || forceMode) {
			if (forceMode)
				is_vendor_ois_device_ctrl(mcu, 0x02);
			else
				is_vendor_ois_device_ctrl(mcu, 0x01);
			do {
				usleep_range(500, 510);
				val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_DEVCTRL);
				if (--retry < 0) {
					err_mcu("Read status failed!!!!, data = 0x%04x", val);
					break;
				}
			} while (val != 0x00);

			if (val == 0x00) {
				mcu->dev_ctrl_state = true;
				info_mcu("%s dev ctrl done.", __func__);
			}
		}
	}

	info_mcu("%s : X\n", __func__);

	return 0;
}

int is_vendor_ois_bypass_read(struct ois_mcu_dev *mcu, u16 id, u16 reg, u8 reg_size, u8 *buf, u8 data_size)
{
	u8 mode = 0;
	u8 rcvdata = 0;
	u8 dev_id[2] = {0, };
	u8 reg_add[2] = {0, };
	int retries = 1000;
	int i = 0;

	info_mcu("%s E\n", __func__);

	/* device id */
	dev_id[0] = id & 0xFF;
	dev_id[1] = (id >> 8) & 0xFF;
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_DEVICE_ID1, dev_id[0]);
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_DEVICE_ID2, dev_id[1]);

	/* register address */
	reg_add[0] = reg & 0xFF;
	reg_add[1] = (reg >> 8) & 0xFF;
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_REG_ADD1, reg_add[0]);
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_REG_ADD2, reg_add[1]);

	/* reg size */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_REG_SIZE, reg_size);

	/* data size */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_DATA_SIZE, data_size);

	/* run bypass mode */
	mode = 0x02;
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_CTRL, mode);

	do {
		rcvdata = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_CTRL);
		usleep_range(1000, 1100);
		if (--retries < 0) {
			err_mcu("read status failed!!!!, data = 0x%04x", rcvdata);
			break;
		}
	} while (rcvdata != 0x00);

	/* get data */
	for (i = 0; i < data_size; i++) {
		rcvdata = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_DATA_TRANSFER + i);
		*(buf + i) = rcvdata & 0xFF;
	}

	info_mcu("%s X\n", __func__);

	return 0;
}

int is_vendor_ois_bypass_write(struct ois_mcu_dev *mcu, u16 id, u16 reg, u8 reg_size, u8 *buf, u8 data_size)
{
	u8 mode = 0;
	u8 rcvdata = 0;
	u8 dev_id[2] = {0, };
	u8 reg_add[2] = {0, };
	int retries = 1000;
	int i = 0;

	info_mcu("%s E\n", __func__);

	/* device id */
	dev_id[0] = id & 0xFF;
	dev_id[1] = (id >> 8) & 0xFF;
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_DEVICE_ID1, dev_id[0]);
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_DEVICE_ID2, dev_id[1]);

	/* register address */
	reg_add[0] = reg& 0xFF;
	reg_add[1] = (reg >> 8) & 0xFF;
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_REG_ADD1, reg_add[0]);
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_REG_ADD2, reg_add[1]);

	/* reg size */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_REG_SIZE, reg_size);

	/* data size */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_DATA_SIZE, data_size);

	/* send data */
	for (i = 0; i < data_size; i++) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_DATA_TRANSFER + i, *(buf + i) & 0xFF);
	}

	/* run bypass mode */
	mode = 0x02;
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_CTRL, mode);

	do {
		rcvdata = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_CTRL);
		usleep_range(1000, 1100);
		if (--retries < 0) {
			err_mcu("read status failed!!!!, data = 0x%04x", rcvdata);
			break;
		}
		i++;
	} while (rcvdata != 0x00);

	info_mcu("%s X\n", __func__);

	return 0;
}

int is_vendor_ois_check_cross_talk(struct v4l2_subdev *subdev, u16 *hall_data)
{
	int ret = 0;
	u8 val = 0;
	u16 x_target = 0;
	int retries = 600;
	u8 addr_size = 0x02;
	u8 data[2] = {0, };
	u8 hall_value[2] = {0, };
	int i = 0;
	struct ois_mcu_dev *mcu = NULL;

	WARN_ON(!subdev);

	info_mcu("%s E\n", __func__);

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err_mcu("mcu is NULL");
		ret = -EINVAL;
		return ret;
	}

	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
		usleep_range(500, 510);
		if (--retries < 0) {
			err_mcu("Read status failed!!!!, data = 0x%04x", val);
			break;
		}
	} while (val != 0x01);

	data[0] = 0x08;
	is_vendor_ois_bypass_write(mcu, MCU_BYPASS_MODE_WRITE_ID, 0x0002, addr_size, data, 0x01);
	data[0] = 0x01;
	is_vendor_ois_bypass_write(mcu, MCU_BYPASS_MODE_WRITE_ID, 0x0080, addr_size, data, 0x01);
	data[0] = 0x01;
	is_vendor_ois_bypass_write(mcu, MCU_BYPASS_MODE_WRITE_ID, 0x0000, addr_size, data, 0x01);

	data[0] = 0x20;
	data[1] = 0x03;
	is_vendor_ois_bypass_write(mcu, MCU_BYPASS_MODE_WRITE_ID, 0x0022, addr_size, data, 0x02);
	data[0] = 0x00;
	data[1] = 0x08;
	is_vendor_ois_bypass_write(mcu, MCU_BYPASS_MODE_WRITE_ID, 0x0024, addr_size, data, 0x02);

	x_target = 800;
	for (i = 0; i < 10; i++) {
		data[0] = x_target & 0xFF;
		data[1] = (x_target >> 8) & 0xFF;
		is_vendor_ois_bypass_write(mcu, MCU_BYPASS_MODE_WRITE_ID, 0x0022, addr_size, data, 0x02);
		msleep(45);

		is_vendor_ois_bypass_read(mcu, MCU_BYPASS_MODE_READ_ID, 0x0090, addr_size, hall_value, 0x02);
		*(hall_data + i) = (hall_value[1] << 8) | hall_value[0];
		info_mcu("%s hall_data[0] = 0x%02x, hall_value[1] = 0x%02x", __func__, hall_value[0], hall_value[1]);
		x_target += 300;
	}

	info_mcu("%s  X\n", __func__);

	return ret;
}

int is_vendor_ois_bypass_read_mode1(struct ois_mcu_dev *mcu, u8 id, u8 reg, u8 *buf, u8 data_size)
{
	u8 mode = 0;
	u8 rcvdata = 0;
	int retries = 1000;
	int i = 0;

	info_mcu("%s E\n", __func__);

	/* device id */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_DEVICE_ID1, id);

	/* register address */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_DEVICE_ID2, reg);

	/* data size */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_REG_ADD1, data_size);

	/* run bypass mode */
	mode = 0x01;
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_CTRL, mode);

	do {
		rcvdata = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_CTRL);
		usleep_range(1000, 1100);
		if (--retries < 0) {
			err_mcu("read status failed!!!!, data = 0x%04x", rcvdata);
			break;
		}
	} while (rcvdata != 0x00);

	/* get data */
	for (i = 0; i < data_size; i++) {
		rcvdata = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_REG_ADD2 + i);
		*(buf + i) = rcvdata & 0xFF;
	}

	info_mcu("%s X\n", __func__);

	return 0;
}

int is_vendor_ois_bypass_write_mode1(struct ois_mcu_dev *mcu, u8 id, u8 reg, u8 *buf, u8 data_size)
{
	u8 mode = 0;
	u8 rcvdata = 0;
	int retries = 1000;
	int i = 0;

	info_mcu("%s E\n", __func__);

	/* device id */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_DEVICE_ID1, id);

	/* register address */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_DEVICE_ID2, reg);

	/* data size */
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_REG_ADD1, data_size);

	/* send data */
	for (i = 0; i < data_size; i++) {
		is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_REG_ADD2 + i, *(buf + i) & 0xFF);
	}

	/* run bypass mode */
	mode = 0x01;
	is_mcu_set_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_CTRL, mode);

	do {
		rcvdata = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_BYPASS_CTRL);
		usleep_range(1000, 1100);
		if (--retries < 0) {
			err_mcu("read status failed!!!!, data = 0x%04x", rcvdata);
			break;
		}
		i++;
	} while (rcvdata != 0x00);

	info_mcu("%s X\n", __func__);

	return 0;
}

int is_vendor_ois_check_hall_cal(struct v4l2_subdev *subdev, u16 *hall_cal_data)
{
	int ret = 0;
	u8 val = 0;
	int retries = 600;
	u8 rxbuf[32] = {0, };
	u8 txbuf[32] = {0, };
	u16 af_best_pos = 0;
	u16 temp = 0;
	int pre_pcal[2] = {0, };
	int pre_ncal[2] = {0, };
	int cur_pcal[2] = {0, };
	int cur_ncal[2] = {0, };
	struct ois_mcu_dev *mcu = NULL;
	struct is_core *core = NULL;

	WARN_ON(!subdev);

	info_mcu("%s E\n", __func__);

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err_mcu("mcu is NULL");
		ret = -EINVAL;
		return ret;
	}

	core = is_get_is_core();
	if (!core) {
		err_mcu("core is null");
		ret = -EINVAL;
		return ret;
	}

	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
		usleep_range(500, 510);
		if (--retries < 0) {
			err_mcu("Read status failed!!!!, data = 0x%04x", val);
			break;
		}
	} while (val != 0x01);

	/* Read stored calibration mark */
	is_vendor_ois_bypass_read_mode1(mcu, 0xE9, 0xE4, rxbuf, 0x01);
	info_mcu("read reg(0xE4) = 0x%02x", rxbuf[0]);

	if (rxbuf[0] != 0x01) {
		info_mcu("calibration data empty(0x%02x)", rxbuf[0]);
		return ret;
	}

	/* Read stored AF best position*/
	is_vendor_ois_bypass_read_mode1(mcu, 0xE9, 0xE5, rxbuf, 0x01);
	af_best_pos = (u16)rxbuf[0] << 4;
	info_mcu("read reg(0xE5) = 0x%04x", af_best_pos);


	/* Read stored PCAL and NCAL of X axis */
	is_vendor_ois_bypass_read_mode1(mcu, 0xE9, 0x04, rxbuf, 0x04);
	temp = ((u16)rxbuf[0] << 8) & 0x8000;
	temp |= ((u16)rxbuf[0] << 1) & 0x00FE;
	temp |= ((u16)rxbuf[1] >> 7) & 0x0001;
	pre_pcal[0] = (int)temp;
	info_mcu("read reg(0x04) = 0x%04x", pre_pcal[0]);

	temp = 0x0;
	temp = ((u16)rxbuf[2] << 8) & 0x8000;
	temp |= ((u16)rxbuf[2] << 1) & 0x00FE;
	temp |= ((u16)rxbuf[3] >> 7) & 0x0001;
	pre_ncal[0] = (int)temp;
	info_mcu("read reg(0x06) = 0x%04x", pre_ncal[0]);


	/* Read stored PCAL and NCAL for Y axis */
	memset(rxbuf, 0x0, sizeof(rxbuf));
	is_vendor_ois_bypass_read_mode1(mcu, 0x69, 0x04, rxbuf, 0x04);
	temp = ((u16)rxbuf[0] << 8) & 0x8000;
	temp |= ((u16)rxbuf[0] << 1) & 0x00FE;
	temp |= ((u16)rxbuf[1] >> 7) & 0x0001;
	pre_pcal[1] = (int)temp;
	info_mcu("read reg(0x04) = 0x%04x", pre_pcal[1]);

	temp = 0x0;
	temp = ((u16)rxbuf[2] << 8) & 0x8000;
	temp |= ((u16)rxbuf[2] << 1) & 0x00FE;
	temp |= ((u16)rxbuf[3] >> 7) & 0x0001;
	pre_ncal[1] = (int)temp;
	info_mcu("read reg(0x06) = 0x%04x", pre_ncal[1]);

	/* Move AF to best position which read from EEPROM */
#ifdef CONFIG_AF_HOST_CONTROL
	is_af_move_lens_pos(core, SENSOR_POSITION_REAR2, af_best_pos);
#endif
	msleep(50);

	/* Change setting  Mode for Hall cal */
	txbuf[0] = 0x3B;
	is_vendor_ois_bypass_write_mode1(mcu, 0xE8, 0xAE, txbuf, 0x01);
	is_vendor_ois_bypass_write_mode1(mcu, 0x68, 0xAE, txbuf, 0x01);
	info_mcu("write reg(0xAE) = 0x%02x", txbuf[0]);

	/* Start hall calibration for X axis */
	txbuf[0] = 0x01;
	is_vendor_ois_bypass_write_mode1(mcu, 0xE8, 0x02, txbuf, 0x01);
	msleep(150);

	/* Start hall calibration for Y axis */
	is_vendor_ois_bypass_write_mode1(mcu, 0x68, 0x02, txbuf, 0x01);
	msleep(150);

	/*Clear setting  Mode */
	txbuf[0] = 0x00;
	is_vendor_ois_bypass_write_mode1(mcu, 0xE8, 0xAE, txbuf, 0x01);
	is_vendor_ois_bypass_write_mode1(mcu, 0x68, 0xAE, txbuf, 0x01);
	info_mcu("write reg(0xAE) = 0x%02x", txbuf[0]);

	/*Read new PCAL and NCAL for X axis*/
	memset(rxbuf, 0x0, sizeof(rxbuf));
	is_vendor_ois_bypass_read_mode1(mcu, 0xE9, 0x04, rxbuf, 0x04);
	temp = ((u16)rxbuf[0] << 8) & 0x8000;
	temp |= ((u16)rxbuf[0] << 1) & 0x00FE;
	temp |= ((u16)rxbuf[1] >> 7) & 0x0001;
	cur_pcal[0] = (int)temp;
	info_mcu("read reg(0x04) = 0x%04x", cur_pcal[0]);

	temp = 0x0;
	temp = ((u16)rxbuf[2] << 8) & 0x8000;
	temp |= ((u16)rxbuf[2] << 1) & 0x00FE;
	temp |= ((u16)rxbuf[3] >> 7) & 0x0001;
	cur_ncal[0] = (int)temp;
	info_mcu("read reg(0x06) = 0x%04x", cur_ncal[0]);

	/*Read new PCAL and NCAL for Y axis*/
	memset(rxbuf, 0x0, sizeof(rxbuf));
	is_vendor_ois_bypass_read_mode1(mcu, 0x69, 0x04, rxbuf, 0x04);
	temp = ((u16)rxbuf[0] << 8) & 0x8000;
	temp |= ((u16)rxbuf[0] << 1) & 0x00FE;
	temp |= ((u16)rxbuf[1] >> 7) & 0x0001;
	cur_pcal[1] = (int)temp;
	info_mcu("read reg(0x04) = 0x%04x", cur_pcal[1]);

	temp = 0x0;
	temp = ((u16)rxbuf[2] << 8) & 0x8000;
	temp |= ((u16)rxbuf[2] << 1) & 0x00FE;
	temp |= ((u16)rxbuf[3] >> 7) & 0x0001;
	cur_ncal[1] = (int)temp;
	info_mcu("read reg(0x06) = 0x%04x", cur_ncal[1]);

	hall_cal_data[0] = pre_pcal[0];
	hall_cal_data[1] = pre_ncal[0];
	hall_cal_data[2] = pre_pcal[1];
	hall_cal_data[3] = pre_ncal[1];
	hall_cal_data[4] = cur_pcal[0];
	hall_cal_data[5] = cur_ncal[0];
	hall_cal_data[6] = cur_pcal[1];
	hall_cal_data[7] = cur_ncal[1];

	info_mcu("%s  X\n", __func__);

	return ret;
}

int is_vendor_ois_read_ext_clock(struct v4l2_subdev *subdev, u32 *clock)
{
	int ret = 0;
	u8 val = 0;
	int retries = 600;
	u8 addr_size = 0x02;
	u8 data[4] = {0, };

	struct ois_mcu_dev *mcu = NULL;

	WARN_ON(!subdev);

	info_mcu("%s E\n", __func__);

	mcu = (struct ois_mcu_dev *)v4l2_get_subdevdata(subdev);
	if (!mcu) {
		err_mcu("mcu is NULL");
		ret = -EINVAL;
		return ret;
	}

	do {
		val = is_mcu_get_reg_u8(mcu->regs[OM_REG_CORE], OIS_CMD_STATUS);
		usleep_range(500, 510);
		if (--retries < 0) {
			err_mcu("Read status failed!!!!, data = 0x%04x", val);
			break;
		}
	} while (val != 0x01);

	is_vendor_ois_bypass_read(mcu, MCU_BYPASS_MODE_READ_ID, 0x03F0, addr_size, data, 0x02);
	is_vendor_ois_bypass_read(mcu, MCU_BYPASS_MODE_READ_ID, 0x03F2, addr_size, &data[2], 0x02);
	*clock = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];

	info_mcu("%s  X\n", __func__);

	return ret;
}

long is_vendor_ois_open_fw(struct is_core *core)
{
	int ret = 0;
	struct is_binary mcu_bin;
	struct is_mcu *is_mcu = NULL;
	struct is_device_sensor *device = NULL;
	struct ois_mcu_dev *mcu = NULL;
	struct is_ois_info *ois_minfo = NULL;
	struct is_ois_info *ois_pinfo = NULL;

	info_mcu("%s started", __func__);

	mcu = core->mcu;

	device = &core->sensor[0];
	is_mcu = device->mcu;

	is_ois_get_module_version(&ois_minfo);
	is_ois_get_phone_version(&ois_pinfo);

	setup_binary_loader(&mcu_bin, 3, -EAGAIN, NULL, NULL);
	ret = request_binary(&mcu_bin, IS_MCU_PATH, IS_MCU_FW_NAME, mcu->dev);
	if (ret) {
		err_mcu("request_firmware was failed(%d)", ret);
		ret = 0;
		goto request_err;
	}

	memcpy(&is_mcu->vdrinfo_bin[0], mcu_bin.data + OIS_CMD_BASE + MCU_HW_VERSION_OFFSET, sizeof(is_mcu->vdrinfo_bin));
	is_mcu->hw_bin[0] = *((u8 *)mcu_bin.data + OIS_CMD_BASE + MCU_BIN_VERSION_OFFSET + 3);
	is_mcu->hw_bin[1] = *((u8 *)mcu_bin.data + OIS_CMD_BASE + MCU_BIN_VERSION_OFFSET + 2);
	is_mcu->hw_bin[2] = *((u8 *)mcu_bin.data + OIS_CMD_BASE + MCU_BIN_VERSION_OFFSET + 1);
	is_mcu->hw_bin[3] = *((u8 *)mcu_bin.data + OIS_CMD_BASE + MCU_BIN_VERSION_OFFSET);
	memcpy(ois_pinfo->header_ver, is_mcu->hw_bin, 4);
	memcpy(&ois_pinfo->header_ver[4], mcu_bin.data + OIS_CMD_BASE + MCU_HW_VERSION_OFFSET, 4);
	memcpy(ois_minfo->header_ver, ois_pinfo->header_ver, sizeof(ois_pinfo->header_ver));

	info_mcu("Request FW was done (%s%s, %ld)\n",
		IS_MCU_PATH, IS_MCU_FW_NAME, mcu_bin.size);

	ret = mcu_bin.size;

request_err:
	release_binary(&mcu_bin);

	info_mcu("%s %d end", __func__, __LINE__);

	return ret;
}

bool is_vendor_ois_check_fw(struct is_core *core)
{
	long ret = 0;
	struct is_vendor_private *vendor_priv;

	info_mcu("%s", __func__);

	ret = is_vendor_ois_open_fw(core);
	if (ret == 0) {
		err_mcu("mcu fw open failed");
		return false;
	}

	vendor_priv = core->vendor.private_data;
	vendor_priv->ois_ver_read = true;

	return true;
}

static struct is_ois_ops ois_ops_mcu = {
	.ois_init = is_vendor_ois_init,
	.ois_init_fac = is_vendor_ois_init_factory,
#if defined(CAMERA_3RD_OIS)
	.ois_init_rear2 = is_vendor_ois_init_rear2,
#endif
	.ois_deinit = is_vendor_ois_deinit,
	.ois_set_mode = is_vendor_ois_set_mode,
	.ois_shift_compensation = is_vendor_ois_shift_compensation,
	.ois_self_test = is_vendor_ois_self_test,
	.ois_auto_test = is_vendor_ois_auto_test_all,
#if defined(CAMERA_2ND_OIS)
	.ois_auto_test_rear2 = is_vendor_ois_auto_test_rear2,
	.ois_set_power_mode = is_vendor_ois_set_power_mode,
#endif
	.ois_set_dev_ctrl = is_vendor_ois_set_dev_ctrl,
	.ois_check_fw = is_vendor_ois_check_fw,
	.ois_enable = is_vendor_ois_enable,
	.ois_disable = is_vendor_ois_disable,
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
#ifdef USE_TELE2_OIS_AF_COMMON_INTERFACE
	.ois_set_af_active = is_vendor_ois_af_set_active,
#endif
	.ois_get_hall_pos = is_vendor_ois_get_hall_position,
	.ois_check_cross_talk = is_vendor_ois_check_cross_talk,
	.ois_check_hall_cal = is_vendor_ois_check_hall_cal,
	.ois_check_valid = is_vendor_ois_check_valid,
#ifdef USE_OIS_HALL_DATA_FOR_VDIS
	.ois_get_hall_data = is_vendor_ois_get_hall_data,
#endif
	.ois_get_active = is_vendor_ois_get_active,
	.ois_read_ext_clock = is_vendor_ois_read_ext_clock,
	.ois_parsing_raw_data = is_vendor_ois_parsing_raw_data,
	.ois_center_shift = is_vendor_ois_set_center_shift,
};

static const struct v4l2_subdev_core_ops core_ops = {
	.init = is_vendor_ois_af_init,
	.ioctl = is_vendor_ois_actuator_ioctl,
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core = &core_ops,
};

void ois_mcu_power_on_work(struct work_struct *data)
{
	struct ois_mcu_dev *mcu = NULL;

	FIMC_BUG_VOID(!data);

	mcu = container_of(data, struct ois_mcu_dev, mcu_power_on_work);

	if (mcu == NULL) {
		err_mcu("%s ois_mcu_dev NULL! power on failed", __func__);
		return;
	}
	is_vendor_ois_power_ctrl(mcu, 0x1);
	is_vendor_ois_load_binary(mcu);
	is_vendor_ois_core_ctrl(mcu, 0x1);

	info_mcu("%s: mcu on.\n", __func__);
}

static int is_vendor_ois_probe(struct platform_device *pdev)
{
	struct is_core *core;
	struct ois_mcu_dev *mcu = NULL;
	struct resource *res;
	int ret = 0;
	struct device_node *dnode;
	struct is_mcu *is_mcu = NULL;
	struct is_device_sensor *device;
	struct v4l2_subdev *subdev_mcu = NULL;
	struct v4l2_subdev *subdev_ois = NULL;
	struct is_ois *ois = NULL;
	struct is_actuator *actuator = NULL;
	struct v4l2_subdev *subdev_actuator = NULL;
	const u32 *sensor_id_spec;
	const u32 *mcu_actuator_spec;
	u32 sensor_id_len;
	u32 sensor_id[IS_SENSOR_COUNT] = {0, };
	u32 mcu_actuator_list[IS_SENSOR_COUNT] = {0, };
	int i;
	u32 mcu_actuator_len;
	struct is_vendor_private *vendor_priv;
	bool support_photo_fastae = false;
	bool skip_video_fastae = false;
	bool off_during_uwonly_mode = false;
	const u32 *gyro_direction_spec;
	u32 gyro_direction_len;

	core = pablo_get_core_async();
	if (!core) {
		err_mcu("core device is not yet probed");
		ret = -EPROBE_DEFER;
		goto p_err;
	}

	dnode = pdev->dev.of_node;

	sensor_id_spec = of_get_property(dnode, "id", &sensor_id_len);
	if (!sensor_id_spec) {
		err_mcu("sensor_id num read is fail(%d)", ret);
		goto p_err;
	}

	sensor_id_len /= (unsigned int)sizeof(*sensor_id_spec);

	ret = of_property_read_u32_array(dnode, "id", sensor_id, sensor_id_len);
	if (ret) {
		err_mcu("sensor_id read is fail(%d)", ret);
		goto p_err;
	}

	mcu_actuator_spec = of_get_property(dnode, "mcu_ctrl_actuator", &mcu_actuator_len);
	if (mcu_actuator_spec) {
		mcu_actuator_len /= (unsigned int)sizeof(*mcu_actuator_spec);
		ret = of_property_read_u32_array(dnode, "mcu_ctrl_actuator",
		        mcu_actuator_list, mcu_actuator_len);
		if (ret)
		        info_mcu("mcu_ctrl_actuator read is fail(%d)", ret);
	}

	support_photo_fastae = of_property_read_bool(dnode, "mcu_support_photo_fastae");
	if (!support_photo_fastae) {
		info_mcu("support_photo_fastae not use");
	}

	skip_video_fastae = of_property_read_bool(dnode, "mcu_skip_video_fastae");
	if (!skip_video_fastae) {
		info_mcu("skip_video_fastae not use");
	}

	off_during_uwonly_mode = of_property_read_bool(dnode, "mcu_off_during_uwonly_mode");
	if (!off_during_uwonly_mode) {
		info_mcu("off_during_uwonly_mode not use");
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

	is_mcu = pablo_zalloc(sizeof(struct is_mcu) * sensor_id_len, GFP_KERNEL);
	if (!mcu) {
		err_mcu("fimc_is_mcu is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	subdev_mcu = pablo_zalloc(sizeof(struct v4l2_subdev) * sensor_id_len, GFP_KERNEL);
	if (!subdev_mcu) {
		err_mcu("subdev_mcu is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	ois = pablo_zalloc(sizeof(struct is_ois) * sensor_id_len, GFP_KERNEL);
	if (!ois) {
		err_mcu("fimc_is_ois is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	subdev_ois = pablo_zalloc(sizeof(struct v4l2_subdev) * sensor_id_len, GFP_KERNEL);
	if (!subdev_ois) {
		err_mcu("subdev_ois is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	actuator = pablo_zalloc(sizeof(struct is_actuator), GFP_KERNEL);
	if (!actuator) {
		err_mcu("actuator is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	subdev_actuator = pablo_zalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!subdev_actuator) {
		err_mcu("subdev_actuator is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	mcu->dev = &pdev->dev;

	for (int i = 0; i < OM_REG_MAX; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			dev_err(mcu->dev, "[@] can't get memory resource\n");
			return -ENODEV;
		}

		mcu->regs[i] = devm_ioremap(mcu->dev, res->start, resource_size(res));
		if (!mcu->regs[i]) {
			dev_err(&pdev->dev, "[@] ioremap (%d) failed\n", i);
			ret = -ENOMEM;
			goto err_ioremap;
		}
		mcu->regs_start[i] = res->start;
		mcu->regs_end[i] = res->end;
	}

	mcu->irq = platform_get_irq(pdev, 0);
	if (mcu->irq < 0) {
		dev_err(mcu->dev, "[@] failed to get IRQ resource: %d\n",
							mcu->irq);
		ret = mcu->irq;
		goto err_get_irq;
	}
	ret = devm_request_irq(mcu->dev, mcu->irq, is_isr_is_vendor_ois,
			0,
			dev_name(mcu->dev), mcu);
	if (ret) {
		dev_err(mcu->dev, "[@] failed to request IRQ(%d): %d\n",
							mcu->irq, ret);
		goto err_req_irq;
	}

	platform_set_drvdata(pdev, mcu);
	core->mcu = mcu;
	atomic_set(&mcu->shared_rsc_count, 0);
	mutex_init(&mcu->power_mutex);
	INIT_WORK(&mcu->mcu_power_on_work, ois_mcu_power_on_work);

	vendor_priv = core->vendor.private_data;
	vendor_priv->ois_ver_read = false;

	for (i = 0; i < sensor_id_len; i++) {
		probe_info("%s sensor_id %d\n", __func__, sensor_id[i]);

		probe_info("%s mcu_actuator_list %d\n", __func__, mcu_actuator_list[i]);

		device = &core->sensor[sensor_id[i]];

		is_mcu[i].name = MCU_NAME_INTERNAL;
		is_mcu[i].subdev = &subdev_mcu[i];
		is_mcu[i].device = sensor_id[i];
		is_mcu[i].private_data = core;
		is_mcu[i].support_photo_fastae = support_photo_fastae;
		is_mcu[i].skip_video_fastae = skip_video_fastae;
		is_mcu[i].off_during_uwonly_mode = off_during_uwonly_mode;

		ois[i].subdev = &subdev_ois[i];
		ois[i].device = sensor_id[i];
		ois[i].ois_mode = OPTICAL_STABILIZATION_MODE_OFF;
		ois[i].pre_ois_mode = OPTICAL_STABILIZATION_MODE_OFF;
		ois[i].ois_shift_available = false;
		ois[i].ixc_lock = NULL;
		ois[i].ois_ops = &ois_ops_mcu;
		set_ois_comm_ops(&internal_mcu_ops);

#if defined(USE_TELE_OIS_AF_COMMON_INTERFACE) || defined(USE_TELE2_OIS_AF_COMMON_INTERFACE)
		if (mcu_actuator_list[i] == 1) {
			actuator->id = ACTUATOR_NAME_AK737X;
			actuator->subdev = subdev_actuator;
			actuator->device = sensor_id[i];
			actuator->position = 0;
			actuator->need_softlanding = 0;
			actuator->max_position = MCU_ACT_POS_MAX_SIZE;
			actuator->pos_size_bit = MCU_ACT_POS_SIZE_BIT;
			actuator->pos_direction = MCU_ACT_POS_DIRECTION;

			is_mcu[i].subdev_actuator = subdev_actuator;
			is_mcu[i].actuator = actuator;

			device->subdev_actuator[sensor_id[i]] = subdev_actuator;
			device->actuator[sensor_id[i]] = actuator;

			v4l2_subdev_init(subdev_actuator, &subdev_ops);
			v4l2_set_subdevdata(subdev_actuator, actuator);
			v4l2_set_subdev_hostdata(subdev_actuator, device);
		}
#endif

		is_mcu[i].mcu_ctrl_actuator = mcu_actuator_list[i];
		is_mcu[i].subdev_ois = &subdev_ois[i];
		is_mcu[i].ois = &ois[i];

		device->subdev_mcu = &subdev_mcu[i];
		device->mcu = &is_mcu[i];

		v4l2_set_subdevdata(&subdev_mcu[i], mcu);
		v4l2_set_subdev_hostdata(&subdev_mcu[i], &is_mcu[i]);

		probe_info("%s done\n", __func__);
	}

#if defined(CONFIG_PM)
	pm_runtime_enable(mcu->dev);
	set_bit(OM_HW_SUSPENDED, &mcu->state);
#endif
	set_bit(OM_HW_NONE, &mcu->state);

	probe_info("[@] %s device probe success\n", dev_name(mcu->dev));

	return 0;

err_req_irq:
err_get_irq:
	for (int i = 0; i < OM_REG_MAX; i++)
		devm_iounmap(mcu->dev, mcu->regs[i]);
err_ioremap:
	devm_release_mem_region(mcu->dev, res->start, resource_size(res));
p_err:
	if (mcu)
		pablo_free(mcu);

	if (is_mcu)
		pablo_free(is_mcu);

	if (subdev_mcu)
		pablo_free(subdev_mcu);

	if (ois)
		pablo_free(ois);

	if (subdev_ois)
		pablo_free(subdev_ois);

	if (actuator)
		pablo_free(actuator);

	if (subdev_actuator)
		pablo_free(subdev_actuator);

	return ret;
}

static const struct dev_pm_ops is_vendor_ois_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(is_vendor_ois_suspend, is_vendor_ois_resume)
	SET_RUNTIME_PM_OPS(is_vendor_ois_runtime_suspend, is_vendor_ois_runtime_resume,
			   NULL)
};

static const struct of_device_id sensor_is_vendor_ois_match[] = {
	{
		.compatible = "samsung,sensor-ois-mcu",
	},
	{},
};

struct platform_driver sensor_ois_mcu_platform_driver = {
	.probe = is_vendor_ois_probe,
	.driver = {
		.name   = "Sensor-OIS-MCU",
		.owner  = THIS_MODULE,
		.pm	= &is_vendor_ois_pm_ops,
		.of_match_table = sensor_is_vendor_ois_match,
	}
};

struct platform_driver *get_internal_ois_platform_driver(void)
{
	return &sensor_ois_mcu_platform_driver;
}

#ifndef MODULE
static int __init sensor_is_vendor_ois_init(void)
{
	int ret;

	ret = platform_driver_probe(&sensor_ois_mcu_platform_driver,
							is_vendor_ois_probe);
	if (ret)
		err("failed to probe %s driver: %d",
			sensor_ois_mcu_platform_driver.driver.name, ret);

	return ret;
}
late_initcall_sync(sensor_is_vendor_ois_init);
#endif

MODULE_DESCRIPTION("Exynos Pablo OIS-Internal MCU driver");
MODULE_AUTHOR("Younghwan Joo <yhwan.joo@samsung.com>");
MODULE_LICENSE("GPL v2");
