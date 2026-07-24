/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 is core functions
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
#include "is-vendor-ois-external-mcu.h"
#include "is-ixc-config.h"
#ifdef USE_OIS_PAD_RETENTION
#include <soc/samsung/exynos-pmu-if.h>
#endif
#define MCU_NAME "MCU_STM32"
#define OIS_I2C_RETRY_COUNT	2
static const struct v4l2_subdev_ops subdev_ops;

/* Flash memory page(or sector) structure */
static struct sysboot_page_type memory_pages[] = {
	{2048, 32},
	{   0,  0}
};

static struct sysboot_map_type memory_map = {
	0x08000000, /* flash memory starting address */
	0x1FFF0000, /* system memory starting address */
	0x1FFF7800, /* option byte starting address */
	(struct sysboot_page_type *)memory_pages,
};

int is_ois_i2c_read(struct i2c_client *client, u16 addr, u8 *data)
{
	int ret;
	u8 txbuf[2], rxbuf[1];
	struct i2c_msg msg[2];

	*data = 0;
	txbuf[0] = (addr & 0xff00) >> 8;
	txbuf[1] = (addr & 0xff);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = txbuf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = rxbuf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (unlikely(ret != 2)) {
		err("%s: register read fail. ret = %d\n", __func__, ret);
		return -EIO;
	}

	*data = rxbuf[0];

	return ret;
}

int is_ois_i2c_read_multi(struct i2c_client *client, u16 addr, u8 *data, size_t size)
{
	int ret;
	u8 rxbuf[256], txbuf[2];
	struct i2c_msg msg[2];

	if (!client) {
		err("client is NULL");
		return -EINVAL;
	}

	txbuf[0] = (addr & 0xff00) >> 8;
	txbuf[1] = (addr & 0xff);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = txbuf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size;
	msg[1].buf = rxbuf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (unlikely(ret != 2)) {
		err("%s: register read fail", __func__);
		return -EIO;
	}

	memcpy(data, rxbuf, size);

	return ret;
}

int is_ois_i2c_write(struct i2c_client *client, u16 addr, u8 data)
{
	int retries = OIS_I2C_RETRY_COUNT;
	int ret = 0;
	u8 buf[3] = {0,};
	struct i2c_msg msg;

	if (!client) {
		err("client is NULL");
		return -EINVAL;
	}

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;

	buf[0] = (addr & 0xff00) >> 8;
	buf[1] = addr & 0xff;
	buf[2] = data;

	do {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (likely(ret == 1))
			break;

		usleep_range(10000,11000);
	} while (--retries > 0);

	/* Retry occured */
	if (unlikely(retries < OIS_I2C_RETRY_COUNT)) {
		err("i2c_write: ret %d, write (%04X, %04X), retry %d\n",
			ret, addr, data, retries);
	}

	if (unlikely(ret != 1)) {
		err("I2C does not work\n\n");
		return -EIO;
	}

	return ret;
}

int is_ois_i2c_write_multi(struct i2c_client *client, u16 addr, u8 *data, size_t size)
{
	int retries = OIS_I2C_RETRY_COUNT;
	int ret = 0;
	ulong i = 0;
	u8 buf[258] = {0,};
	struct i2c_msg msg;

	if (!client) {
		err("client is NULL");
		return -EINVAL;
	}

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = size + 2;
	msg.buf = buf;

	buf[0] = (addr & 0xFF00) >> 8;
	buf[1] = addr & 0xFF;

	for (i = 0; i < size; i++)
		buf[i + 2] = *(data + i);

	do {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (likely(ret == 1))
			break;

		usleep_range(10000,11000);
	} while (--retries > 0);

	/* Retry occured */
	if (unlikely(retries < OIS_I2C_RETRY_COUNT)) {
		err("i2c_write: ret %d, write (%04X, %04X), retry %d\n",
			ret, addr, *data, retries);
	}

	if (unlikely(ret != 1)) {
		err("I2C does not work\n\n");
		return -EIO;
	}

	return ret;
}

int is_ois_external_mcu_read_u8(int cmd, u8 *data) {
	int ret = 0;
	struct i2c_client *client = is_mcu_i2c_get_client();

	ret = is_ois_i2c_read(client, ois_mcu_regs[cmd].sfr_offset, data);

	dbg_ois("[GET_REG] reg:[%s][0x%04X], reg_value(R):[0x%02X]\n",
		ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset, *data);

	if (unlikely(ret != 2)) {
		err_mcu("get fail (%s:%X)", ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset);
		return -EIO;
	}

	return ret;
}

int is_ois_external_mcu_read_multi(int cmd, u8 *data, size_t size)
{
	int i;
	struct i2c_client *client = is_mcu_i2c_get_client();
	int ret = 0;

	ret = is_ois_i2c_read_multi(client, ois_mcu_regs[cmd].sfr_offset, data, size);

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

int is_ois_external_mcu_read_u16(int cmd, u8 *data)
{
	return is_ois_external_mcu_read_multi(cmd, data, 2);
}

int is_ois_external_mcu_write_u8(int cmd, u8 data)
{
	int ret = 0;
	struct i2c_client *client = is_mcu_i2c_get_client();

	ret = is_ois_i2c_write(client, ois_mcu_regs[cmd].sfr_offset, data);

	dbg_ois("[SET_REG] reg:[%s][0x%04X], reg_value(W):[0x%02X]\n",
		ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset, data);

	if (unlikely(ret != 1)) {
		err_mcu("set fail (%s:%X)", ois_mcu_regs[cmd].reg_name, ois_mcu_regs[cmd].sfr_offset);
		return -EIO;
	}

	return ret;
}

int is_ois_external_mcu_write_multi(int cmd, u8 *data, size_t size)
{
	int ret = 0;
	int i;
	struct i2c_client *client = is_mcu_i2c_get_client();

	ret = is_ois_i2c_write_multi(client, ois_mcu_regs[cmd].sfr_offset, data, size);

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

int is_ois_external_mcu_write_u16(int cmd, u8 *data)
{
	return is_ois_external_mcu_write_multi(cmd, data, 2);
}

static struct ois_comm_ops external_mcu_ops = {
	.read_u8 = is_ois_external_mcu_read_u8,
	.read_u16 = is_ois_external_mcu_read_u16,
	.read_multi = is_ois_external_mcu_read_multi,
	.write_u8 = is_ois_external_mcu_write_u8,
	.write_u16 = is_ois_external_mcu_write_u16,
	.write_multi = is_ois_external_mcu_write_multi,
};

struct i2c_client *is_mcu_i2c_get_client(void)
{
	struct i2c_client *client = NULL;
	struct is_core *core = is_get_is_core();
	struct is_vendor_private *vendor_priv = core->vendor.private_data;
	u32 sensor_idx = vendor_priv->mcu_sensor_index;

	if (core->sensor[sensor_idx].mcu != NULL)
		client = core->sensor[sensor_idx].mcu->client;

	return client;
};

int is_mcu_wait_ack(struct i2c_client *client, ulong timeout)
{
	int i;
	int ret = 0;
	u8 recv = 0;

	for (i = 0; i < BOOT_I2C_WAIT_RESP_POLL_RETRY; i++) {
		ret = i2c_master_recv(client, &recv, 1);
		if (ret > 0) {
			if (recv == BOOT_I2C_RESP_ACK) {
				info("mcu ack success");
				return 0;
			} else if (recv == BOOT_I2C_RESP_BUSY) {
				msleep(3);
				ret = -EBUSY;
				continue;
			} else if (recv == BOOT_I2C_RESP_NACK) {
				msleep(3);
				return -EFAULT;
			} else {
				msleep(3);
				ret = -ENODEV;
				continue;
			}
		} else {
			err("receive i2c reve failed");
			if (time_after(jiffies, timeout)) {
				/* Bus was not idle, try to reset */
				err("wait timeout");
				return -EBUSY;
			}
		}
		msleep(5);
	}

	msleep(5);

	return ret;
}

int is_mcu_info(struct v4l2_subdev *subdev, int info, int size)
{
	int i;
	int ret = 0;
	u8 cmd[2] = {0, };
	u8 recv[BOOT_I2C_RESP_GET_ID_LEN] = {0, };
	struct is_mcu *is_mcu = NULL;
	struct i2c_client *client = NULL;

	info("%s started", __func__);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("is_mcu is NULL");
		return -EINVAL;
	}

	client = is_mcu->client;

	/* build command */
	cmd[0] = info;
	cmd[1] = ~cmd[0];

	for (i = 0; i < BOOT_I2C_SYNC_RETRY_COUNT; i++) {
		/* transmit command */
		ret = i2c_master_send(client, &cmd[0], 2);
		if (ret < 0) {
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			err("cmd transfer fail cmd = 0x%x", cmd[0]);
			continue;
		} else
			info("%s cmd transfer success, ret = %d", __func__, ret);

		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_WAIT_RESP_TMOUT);
		if (ret) {
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			err("wait ack fail ret = %d", ret);
			continue;
		}

		/* receive payload */
		ret = i2c_master_recv(client, recv, size);
		if (ret < 0) {
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			err("receive payload fail");
			continue;
		}

		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_WAIT_RESP_TMOUT);
		if (ret) {
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			err("wait ack fail ret = %d", ret);
			continue;
		}

		if (info == BOOT_I2C_CMD_GET_ID) {
			memcpy((void *)&(is_mcu->id), &recv[1], recv[0] + 1);
			is_mcu->id = NTOHS(is_mcu->id);
			info("mcu info(id) = %d", is_mcu->id);
		} else if (info == BOOT_I2C_CMD_GET_VER) {
			memcpy((void *)&(is_mcu->ver), recv, 1);
			info("mcu info(ver) = %d", is_mcu->ver);
		}

		return 0;
	}

	return ret;
}

int is_mcu_sync(struct i2c_client *client, struct is_mcu *mcu)
{
	int i;
	int ret = 0;
	u8 data = 0;

	info("%s started", __func__);

	data = 0xFF;

	for (i = 0; i < BOOT_I2C_SYNC_RETRY_COUNT; i++) {
		ret = i2c_master_send(client, &data, 1);
		if (ret >= 0) {
			info("mcu sync success ret = %d", ret);
			return 0;
		} else {
			err("mcu sync failed, ret = %d", ret);
		}
	}

	return ret;
}

int  is_mcu_connect(struct v4l2_subdev *subdev, struct is_core *core)
{
	int ret = 0;
	struct is_mcu *is_mcu = NULL;
	struct i2c_client *client = NULL;
	int gpio_mcu_reset;
	int gpio_mcu_boot0;

	info("%s started", __func__);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("is_mcu is NULL");
		return -EINVAL;
	}

	client = is_mcu->client;

	if (is_mcu->gpio_mcu_boot0) {
		gpio_mcu_boot0 = is_mcu->gpio_mcu_boot0;
	} else {
		err("gpio_mcu_boot0 is not valid");
		goto exit;
	}

	if (is_mcu->gpio_mcu_reset) {
		gpio_mcu_reset = is_mcu->gpio_mcu_reset;
	} else {
		err("gpio_mcu_reset is not valid");
		goto exit;
	}

#ifdef USE_OIS_PAD_RETENTION
	exynos_pmu_update(0x20a0, 0x00000800, 0x00000800);
#endif
	gpio_direction_output(gpio_mcu_reset, GPIO_PIN_RESET);
	gpio_direction_output(gpio_mcu_boot0, GPIO_PIN_SET);
	msleep(BOOT_NRST_PULSE_INTVL);
	gpio_direction_output(gpio_mcu_reset, GPIO_PIN_SET);
	msleep(BOOT_I2C_SYNC_RETRY_INTVL);
	gpio_direction_output(gpio_mcu_boot0, GPIO_PIN_RESET);

	ret = is_mcu_sync(client, is_mcu);
	if (!ret) {
		info("mcu sync success, reconnect.");
		gpio_direction_output(gpio_mcu_reset, GPIO_PIN_RESET);
		gpio_direction_output(gpio_mcu_boot0, GPIO_PIN_SET);
		msleep(BOOT_NRST_PULSE_INTVL);
		gpio_direction_output(gpio_mcu_reset, GPIO_PIN_SET);
		msleep(BOOT_I2C_SYNC_RETRY_INTVL);
		gpio_direction_output(gpio_mcu_boot0, GPIO_PIN_RESET);
	}
#ifdef USE_OIS_PAD_RETENTION
	exynos_pmu_update(0x20a0, 0x0, 0x00000800);
#endif

	info("%s end", __func__);

exit:
	return ret;
}

void is_mcu_disconnect(struct v4l2_subdev *subdev, struct is_core *core)
{
	struct is_mcu *is_mcu = NULL;
	int gpio_mcu_reset;
	int gpio_mcu_boot0;

	info("%s started", __func__);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("is_mcu is NULL");
		return;
	}

	if (is_mcu->gpio_mcu_boot0) {
		gpio_mcu_boot0 = is_mcu->gpio_mcu_boot0;
	} else {
		err("gpio_mcu_boot0 is not valid");
		goto exit;
	}

	if (is_mcu->gpio_mcu_reset) {
		gpio_mcu_reset = is_mcu->gpio_mcu_reset;
	} else {
		err("gpio_mcu_reset is not valid");
		goto exit;
	}
#ifdef USE_OIS_PAD_RETENTION
	exynos_pmu_update(0x20a0, 0x00000800, 0x00000800);
#endif
	gpio_direction_output(gpio_mcu_boot0, GPIO_PIN_RESET);
	msleep(BOOT_NRST_PULSE_INTVL);
	gpio_direction_output(gpio_mcu_reset, GPIO_PIN_RESET);
	msleep(BOOT_NRST_PULSE_INTVL);
	gpio_direction_output(gpio_mcu_reset, GPIO_PIN_SET);
#ifdef USE_OIS_PAD_RETENTION
	exynos_pmu_update(0x20a0, 0x0, 0x00000800);
#endif
exit:
	return;
}

u8 is_mcu_checksum(u8 *src, u32 len)
{
	u16 csum = *src++;

	if (len) {
		while (--len) {
			csum ^= *src++;
		}
	} else {
		csum = 0; /* error (no length param) */
	}

	info("mcu checksum = 0x%04x.", csum);

	return csum;
}

int is_mcu_i2c_read(struct i2c_client *client, u32 address, u8 *dst, size_t len)
{
	u8 cmd[2] = {0, };
	u8 startaddr[5] = {0, };
	u8 nbytes[2] = {0, };
	int ret = 0;
	int retry = 0;

	/* build command */
	cmd[0] = BOOT_I2C_CMD_READ;
	cmd[1] = ~cmd[0];

	/* build address + checksum */
	*(u32 *)startaddr = HTONL(address);
	startaddr[4] = is_mcu_checksum(startaddr, 4);

	/* build number of bytes + checksum */
	nbytes[0] = len - 1;
	nbytes[1] = ~nbytes[0];

	for (retry = 0; retry < BOOT_I2C_SYNC_RETRY_COUNT; ++retry) {
		/* transmit command */
		ret = i2c_master_send(client, cmd, 2);
		if (ret < 0) {
			err("[MCU] failed to send cmd");
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_WAIT_RESP_TMOUT);
		if (ret < 0) {
			err("wait ack fail ret = %d", ret);
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		/* transmit address */
		ret = i2c_master_send(client, startaddr, 5);
		if (ret < 0) {
			err("[MCU] failed to send address");
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_WAIT_RESP_TMOUT);
		if (ret < 0) {
			err("wait ack fail ret = %d", ret);
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		/* transmit number of bytes + datas */
		ret = i2c_master_send(client, nbytes, sizeof(nbytes));
		if (ret < 0) {
			err("[MCU] failed to send data");
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_WAIT_RESP_TMOUT);
		if (ret < 0) {
			err("wait ack fail ret = %d", ret);
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		/* receive payload */
		ret = i2c_master_recv(client, dst, len);
		if (ret < 0) {
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		return 0;
	}

	return -EINVAL;
}

int is_mcu_i2c_write(struct i2c_client *client, u32 address, u8 *src, size_t len)
{
	u8 cmd[2] = {0, };
	u8 startaddr[5] = {0, };
	int ret = 0;
	int retry = 0;
	char * buf = NULL;

	/* build command */
	cmd[0] = BOOT_I2C_CMD_WRITE;
	cmd[1] = ~cmd[0];

	/* build address + checksum */
	*(u32 *)startaddr = HTONL(address);
	startaddr[4] = is_mcu_checksum(startaddr, 4);

	/* build number of bytes + checksum */
	buf = pablo_zalloc(len + 2, GFP_KERNEL);
	if (!buf) {
		err("[MCU] failed to alloc memory");
		return -ENOMEM;
	}

	buf[0] = len -1;
	memcpy(&buf[1], src, len);
	buf[len+1] = is_mcu_checksum(buf, len + 1);

	info("mcu write cmd started");

	for (retry = 0; retry < BOOT_I2C_SYNC_RETRY_COUNT; ++retry) {
		/* transmit command */
		ret = i2c_master_send(client, cmd, 2);
		if (ret < 0) {
			err("[MCU] failed to send cmd");
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_WAIT_RESP_TMOUT);
		if (ret < 0) {
			err("wait ack fail ret = %d", ret);
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		/* transmit address */
		ret = i2c_master_send(client, startaddr, 5);
		if (ret < 0) {
			err("[MCU] failed to send address");
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_WAIT_RESP_TMOUT);
		if (ret < 0) {
			err("wait ack fail ret = %d", ret);
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		/* transmit number of bytes + datas */
		ret = i2c_master_send(client, buf, len + 2);
		if (ret < 0) {
			err("[MCU] failed to send data");
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_WRITE_TMOUT);
		if (ret < 0) {
			err("wait ack fail ret = %d", ret);
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		pablo_free(buf);

		return 0;
	}

	pablo_free(buf);

	return -EINVAL;
}

int is_mcu_conv_memory_map(uint32_t address, size_t len, struct sysboot_erase_param_type *erase)
{
	struct sysboot_page_type *map = memory_map.pages;
	int found = 0;
	int total_bytes = 0, total_pages = 0;
	int ix = 0;
	int unit = 0;

	info("%s started", __func__);

	/* find out the matched starting page number and total page count */
	for (ix = 0; map[ix].size != 0; ++ix) {
		for (unit = 0; unit < map[ix].count; ++unit) {
			/* MATCH CASE: Starting address aligned and page number to be erased */
			if (address == memory_map.flashbase + total_bytes) {
				found++;
				erase->page = total_pages;
			}
			total_bytes += map[ix].size;
			total_pages++;
			/* MATCH CASE: End of page number to be erased */
			if ((found == 1) && (len <= total_bytes)) {
				found++;
				erase->count = total_pages - erase->page;
			}
		}
	}

	if (found < 2) {
		/* Not aligned address or too much length inputted */
		err("conv failed.");
		return -EFAULT;
	}

	if ((address == memory_map.flashbase) && (erase->count == total_pages))
		erase->page = 0xFFFF; /* mark the full erase */

	info("mcu conv cmd end.");

	return 0;
}

int is_mcu_erase(struct v4l2_subdev *subdev, u32 address, size_t len)
{
	u8 cmd[2] = {0, };
	struct sysboot_erase_param_type erase;
	u8 xmit_bytes = 0;
	int ret = 0;
	int retry = 0;
	uint8_t *xmit = NULL;
	struct is_mcu *is_mcu = NULL;
	struct i2c_client *client = NULL;

	info("%s started", __func__);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("is_mcu is NULL");
		return -EINVAL;
	}

	client = is_mcu->client;

	/* build command */
	cmd[0] = BOOT_I2C_CMD_ERASE;
	cmd[1] = ~cmd[0];

	/* build erase parameter */
	ret = is_mcu_conv_memory_map(address, len, &erase);
	if (ret < 0)
		return -EINVAL;

	info("mcu erase page 0x%x", erase.page);

	xmit = pablo_zalloc(1024, GFP_KERNEL);
	if (!xmit) {
		err("xmit is NULL");
		return -EINVAL;
	}

	for (retry = 0; retry < BOOT_I2C_SYNC_RETRY_COUNT; ++retry) {
		/* build full erase command */
		if (erase.page == 0xFFFF) {
			*(u16 *)xmit = (uint16_t)erase.page;
		}

		/* build page erase command */
		else {
			*(u16 *)xmit = HTONS((erase.count - 1));
		}
		xmit_bytes = sizeof(u16);
		xmit[xmit_bytes] = is_mcu_checksum(xmit, xmit_bytes);
		xmit_bytes++;

		/* transmit command */
		ret = i2c_master_send(client, cmd, sizeof(cmd));
		if (ret < 0) {
			err("send data failed");
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		} else
			info("%s cmd transfer success, ret = %d", __func__, ret);

		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_WAIT_RESP_TMOUT);
		if (ret < 0) {
			err("wait ack fail ret = %d", ret);
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		/* transmit parameter */
		ret = i2c_master_send(client, xmit, xmit_bytes);
		if (ret < 0) {
			err("send data failed");
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		} else
			info("%s xmit transfer success, ret = %d", __func__, ret);

		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, (erase.page == 0xFFFF) ?
			BOOT_I2C_FULL_ERASE_TMOUT : BOOT_I2C_WAIT_RESP_TMOUT);
		if (ret < 0) {
			err("wait ack fail ret = %d", ret);
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		info("mcu erase page started.");

		/* case of page erase */
		if (erase.page != 0xFFFF) {
			/* build page erase parameter */
			register int ix;
			register u16 *pbuf = (uint16_t *)xmit;

			for (ix = 0; ix < erase.count; ++ix)
				pbuf[ix] = HTONS((erase.page + ix));

			xmit_bytes = 2 * erase.count;
			*((uint8_t *)&pbuf[ix]) = is_mcu_checksum(xmit, xmit_bytes);
			xmit_bytes++;
			info("mcu xmit=0x%02X, xmit_bytes = %d, erase.count = %d", *xmit, xmit_bytes, erase.count);

			/* transmit parameter */
			ret = i2c_master_send(client, xmit, xmit_bytes);
			if (ret < 0) {
				err("send data failed");
				msleep(BOOT_I2C_SYNC_RETRY_INTVL);
				continue;
			} else
				info("%s xmit transfer success, ret = %d", __func__, ret);

#if 1
			/* 36ms * 31 (erase.count) delay is required */
			msleep(1200);
#endif
			/* wait for ACK response */
			ret = is_mcu_wait_ack(client, BOOT_I2C_PAGE_ERASE_TMOUT(erase.count + 1));
			if (ret < 0) {
				err("wait ack fail ret = %d", ret);
				msleep(BOOT_I2C_SYNC_RETRY_INTVL);
				continue;
			}
		}

		if (xmit)
			pablo_free(xmit);

		return 0;
	}

	if (xmit)
		pablo_free(xmit);

	return -EINVAL;
}

int is_mcu_write_unprotect(struct i2c_client *client)
{
	u8 cmd[2] = {0, };
	int ret = 0;
	int retry;

	info("%s started", __func__);

	/* build command */
	cmd[0] = BOOT_I2C_CMD_WRITE_UNPROTECT;
	cmd[1] = ~cmd[0];

	for (retry = 0; retry < BOOT_I2C_SYNC_RETRY_COUNT; ++retry) {
		/* transmit co`mmand */
		ret = i2c_master_send(client, cmd, sizeof(cmd));
		if (ret < 0) {
			err("send data failed");
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		} else
			info("%s cmd transfer success, ret = %d", __func__, ret);
		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_FULL_ERASE_TMOUT);
		if (ret < 0) {
			err("wait ack fail ret = %d", ret);
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}
		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_FULL_ERASE_TMOUT);
		if (ret < 0) {
			err("wait ack fail ret = %d", ret);
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		return 0;
	}

	return -EINVAL;
}

int is_mcu_read_unprotect(struct i2c_client *client)
{
	uint8_t cmd[2] = {0, };
	int ret = 0;
	int retry;

	info("%s started", __func__);

	/* build command */
	cmd[0] = BOOT_I2C_CMD_READ_UNPROTECT;
	cmd[1] = ~cmd[0];

	for (retry = 0; retry < BOOT_I2C_SYNC_RETRY_COUNT; ++retry) {
		/* transmit command */
		ret = i2c_master_send(client, cmd, sizeof(cmd));
		if (ret < 0) {
			err("send data failed");
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		} else
			info("%s cmd transfer success, ret = %d", __func__, ret);
		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_FULL_ERASE_TMOUT);
		if (ret < 0) {
			err("wait ack fail ret = %d", ret);
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}
		/* wait for ACK response */
		ret = is_mcu_wait_ack(client, BOOT_I2C_FULL_ERASE_TMOUT);
		if (ret < 0) {
			err("wait ack fail ret = %d", ret);
			msleep(BOOT_I2C_SYNC_RETRY_INTVL);
			continue;
		}

		return 0;
	}

	return -EINVAL;
}

int is_mcu_empty_check_status(struct v4l2_subdev *subdev)
{
	u32 value = 0;
	int ret = 0;
	struct is_mcu *is_mcu = NULL;
	struct i2c_client *client = NULL;

	info("%s started", __func__);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("is_mcu is NULL");
		return -EINVAL;
	}

	client = is_mcu->client;

	/* Read first flash memory word ------------------------------------------- */
	ret = is_mcu_i2c_read(client, memory_map.flashbase, (u8 *)&value, sizeof(value));
	if (ret < 0) {
		err("failed to read word for empty check (%d)", ret);
		goto empty_check_status_fail;
	}

	info("mcu flash word: 0x%08X", value);

	if (value == 0xFFFFFFFF) {
		return 1;
	}

	return 0;

empty_check_status_fail:

	return -1;
}

int is_mcu_empty_check_clear(struct v4l2_subdev *subdev, struct is_core *core)
{
	int ret = 0;
	uint32_t optionbyte = 0;
	struct is_mcu *is_mcu = NULL;
	struct i2c_client *client = NULL;

	info("%s started", __func__);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("is_mcu is NULL");
		return -EINVAL;
	}

	client = is_mcu->client;

	/* Option Byte read ------------------------------------------------------- */
	ret = is_mcu_i2c_read(client, memory_map.optionbyte, (uint8_t *)&optionbyte, sizeof(optionbyte));
	if (ret < 0) {
		err("mcu read  failed (%d)", ret);
		goto empty_check_clear_fail;
	}

	/* Option byte write (dummy: readed value) -------------------------------- */
	ret = is_mcu_i2c_write(client, memory_map.optionbyte, (uint8_t *)&optionbyte, sizeof(optionbyte));
	if (ret < 0) {
		err("mcu write  failed (%d)", ret);
		goto empty_check_clear_fail;
	}

	/* Put little delay for Target program option byte and self-reset */
	mdelay(150);

	/* Option byte read for checking protection status ------------------------ */
	/* 1> Re-connect to the target */
	ret = is_mcu_connect(subdev, core);
	if (ret) {
		err("[INF] Cannot connect to the target for RDP check (%d)", ret);
		goto empty_check_clear_fail;
	}

	info("[INF] Re-Connection OK");

	/* 2> Read from target for status checking and recover it if needed */
	ret = is_mcu_i2c_read(client, memory_map.optionbyte, (uint8_t *)&optionbyte, sizeof(optionbyte));
	if ((ret < 0) || ((optionbyte & 0x000000FF) != 0xAA)) {
		err("[INF]  Failed to read option byte from target (%d)", ret);

		/* Tryout the RDP level to 0 */
		ret = is_mcu_read_unprotect(client);
		if (ret) {
			info("[INF] Readout unprotect KO ... Host restart and try again");
		} else {
			info("[INF] Readout unprotect OK ... Host restart and try again");
		}

		/* Put little delay for Target erase all of pages */
		msleep(50);
		goto empty_check_clear_fail;
	}

	return 0;

empty_check_clear_fail:

	return -1;
}

int is_mcu_optionbyte_update(struct v4l2_subdev *subdev, struct is_core *core)
{
	int ret = 0;
	u32 optionbyte = 0;
	int retry = 3;
	struct is_mcu *is_mcu = NULL;
	struct i2c_client *client = NULL;

	info("%s started", __func__);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("is_mcu is NULL");
		return -EINVAL;
	}

	client = is_mcu->client;

optionbyte_update_entry:

	if (--retry <= 0) {
		goto optionbyte_update_fail;
	}

	/* Option Byte read ------------------------------------------------------- */
	ret = is_mcu_i2c_read(client, memory_map.optionbyte, (u8 *)&optionbyte, sizeof(optionbyte));
	if ((ret < 0) ||((optionbyte & 0x000000ff) != 0xaa)) {
		info("mcu read fail. read unprotest.");
		/* Tryout the RDP level to 0 */
		ret = is_mcu_read_unprotect(client);
		if (ret) {
			err("read unprotected fail");
		} else {
			info("mcu read unprotected success");
		}

		/* Put little delay for Target erase all of pages */
		msleep(60);

		/* Re-connect to the target */
		ret = is_mcu_connect(subdev, core);
		if (ret) {
			err("mcu connect at optionbyte failed.");
			goto optionbyte_update_fail;
		}
		info("mcu connect at optionbyte success.");
		goto optionbyte_update_entry;
	}

	info("mcu optionbyte update processing.");

	/* Clear nBOOT_SEL bit in Option byte1 if it is set.
	* NOTE: nBOOT_SEL bit is 24th-bit */
	if (optionbyte & (1 << 24)) {
		/* Option byte write ---------------------------------------------------- */
		optionbyte &= ~(1 << 24);
		ret = is_mcu_i2c_write(client, memory_map.optionbyte, (uint8_t *)&optionbyte, sizeof(optionbyte));
		if (ret) {
			err("mcu write failed.");
			goto optionbyte_update_fail;
		}
		info("mcu write option byte OK");

		/* Put little delay for Target program option byte and self-reset */
		msleep(100);

		/* Reconnection required after the target option byte will be written */
		ret = is_mcu_connect(subdev, core);
		if (ret) {
			err("[MCU] mcu connect failed.");
			goto optionbyte_update_fail;
		}

		info("mcu Re-Connection OK");

		/* Retry sequence for verification of option byte */
		goto optionbyte_update_entry;
	}

	info("mcu optionbyte update done.");

	return 0;

optionbyte_update_fail:
	err("mcu option byte failed.");

	return -1;
}

int is_mcu_validation(struct v4l2_subdev *subdev, struct is_core *core)
{
	int ret = 0;

	info("%s started", __func__);

	ret = is_mcu_connect(subdev, core);
	if (ret) {
		err("[MCU] mcu connect failed.");
		goto validation_fail;
	}

	ret = is_mcu_info(subdev, BOOT_I2C_CMD_GET_ID, 3);
	if (ret < 0) {
		err("get id info failed");
		goto validation_fail;
	}

	ret = is_mcu_info(subdev, BOOT_I2C_CMD_GET_VER, 1);
	if (ret < 0) {
		err("get ver info failed");
		goto validation_fail;
	}

	ret = is_mcu_optionbyte_update(subdev, core);
	if (ret < 0) {
		err("optionbyte update failed");
		goto validation_fail;
	}

	return 0;

validation_fail:
	is_mcu_disconnect(subdev, core);

	return -1;
}

#ifdef USE_KERNEL_VFS_READ_WRITE
int is_mcu_open_fw(struct v4l2_subdev *subdev, char *name, u8 **buf, ulong *buf_size)
{
	int ret = 0;
#if 0
	ulong size = 0;
	const struct firmware *fw_blob = NULL;
	static char fw_name[100];
	struct file *fp = NULL;
	mm_segment_t old_fs;
	long nread;
	int fw_requested = 1;
	int retry_count = 0;
	struct is_mcu *is_mcu = NULL;
	struct i2c_client *client = NULL;
	struct is_ois_info *ois_pinfo = NULL;

	info("%s started", __func__);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("is_mcu is NULL");
		return -EINVAL;
	}

	client = is_mcu->client;

	//fw_sdcard = false;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	snprintf(fw_name, sizeof(fw_name), "%s%s", IS_OIS_SDCARD_PATH, name);
	fp = filp_open(fw_name, O_RDONLY, 0);
	if (IS_ERR_OR_NULL(fp)) {
		info("mcu failed to open SDCARD fw!!!\n");
		goto request_fw;
	}

	fw_requested = 0;
	size = fp->f_path.dentry->d_inode->i_size;
	info("mcu start read sdcard, file path %s, size %lu Bytes\n", fw_name, size);

	*buf = pablo_malloc(size, GFP_KERNEL);
	if (!(*buf)) {
		err("failed to allocate memory");
		ret = -ENOMEM;
		goto p_err;
	}

	nread = kernel_read(fp, (char __user *)(*buf), size, &fp->f_pos);
	if (nread != size) {
		err("failed to read firmware file, %ld Bytes\n", nread);
		ret = -EIO;
		goto p_err;
	}

	is_ois_get_phone_version(&ois_pinfo);

	memcpy(&is_mcu->vdrinfo_bin[0], *buf + 0x807C, sizeof(is_mcu->vdrinfo_bin));
	is_mcu->hw_bin[0] = *(*buf + 0x80FB);
	is_mcu->hw_bin[1] = *(*buf + 0x80FA);
	is_mcu->hw_bin[2] = *(*buf + 0x80F9);
	is_mcu->hw_bin[3] = *(*buf + 0x80F8);
	memcpy(ois_pinfo->header_ver, is_mcu->hw_bin, 4);
	memcpy(&ois_pinfo->header_ver[4], *buf + 0x807C, 4);

	//fw_sdcard = true;
	if (OIS_BIN_LEN >= nread) {
		//ois_device->not_crc_bin = true;
		info("mcu fw binary size = %ld.\n", nread);
	}

request_fw:
	if (fw_requested) {
		snprintf(fw_name, sizeof(fw_name), "%s", name);
		set_fs(old_fs);
		retry_count = 3;
		ret = request_firmware(&fw_blob, fw_name, &client->dev);
		while (--retry_count && ret == -EAGAIN) {
			err("request_firmware retry(count:%d)", retry_count);
			ret = request_firmware(&fw_blob, fw_name, &client->dev);
		}

		if (ret) {
			err("request_firmware is fail(ret:%d)", ret);
			ret = -EINVAL;
			goto p_err;
		}

		if (!fw_blob) {
			err("fw_blob is NULL");
			ret = -EINVAL;
			goto p_err;
		}

		if (!fw_blob->data) {
			err("fw_blob->data is NULL");
			ret = -EINVAL;
			goto p_err;
		}

		size = fw_blob->size;

		*buf = pablo_malloc(size, GFP_KERNEL);
		if (!(*buf)) {
			err("failed to allocate memory");
			ret = -ENOMEM;
			goto p_err;
		}

		memcpy((void *)(*buf), fw_blob->data, size);
		memcpy(&is_mcu->vdrinfo_bin[0], *buf + 0x807C, sizeof(is_mcu->vdrinfo_bin));
		is_mcu->hw_bin[0] = *(*buf + 0x80FB);
		is_mcu->hw_bin[1] = *(*buf + 0x80FA);
		is_mcu->hw_bin[2] = *(*buf + 0x80F9);
		is_mcu->hw_bin[3] = *(*buf + 0x80F8);
		memcpy(ois_pinfo->header_ver, is_mcu->hw_bin, 4);
		memcpy(&ois_pinfo->header_ver[4], *buf + 0x807C, 4);

		if (OIS_BIN_LEN >= size) {
			//ois_device->not_crc_bin = true;
			info("mcu fw binary size = %lu.", size);
		}

		info("mcu firmware is loaded from Phone binary.");
	}

p_err:
	*buf_size = size;
	info("[%s] mcu binary hw ver = %c%c%c%c, vdrinfo ver = %c%c%c%c", __func__,
		is_mcu->hw_bin[0], is_mcu->hw_bin[1], is_mcu->hw_bin[2], is_mcu->hw_bin[3],
		is_mcu->vdrinfo_bin[0], is_mcu->vdrinfo_bin[1], is_mcu->vdrinfo_bin[2], is_mcu->vdrinfo_bin[3]);

	if (!fw_requested) {
		if (!IS_ERR_OR_NULL(fp)) {
			filp_close(fp, current->files);
		}
		set_fs(old_fs);
	} else {
		if (!IS_ERR_OR_NULL(fw_blob))
			release_firmware(fw_blob);
	}
#endif
	return ret;
}
#else
int is_mcu_open_fw(struct v4l2_subdev *subdev, char *name, u8 **buf, ulong *buf_size)
{
	int ret = 0;
#if 0
	ulong size = 0;
	const struct firmware *fw_blob = NULL;
	static char fw_name[100];
	mm_segment_t old_fs;
	int fw_requested = 1;
	int retry_count = 0;
	struct is_mcu *is_mcu = NULL;
	struct i2c_client *client = NULL;
	struct is_binary bin;
	struct is_ois_info *ois_pinfo = NULL;

	info("%s started", __func__);

	is_mcu = (struct is_mcu *)v4l2_get_subdev_hostdata(subdev);
	if (!is_mcu) {
		err("is_mcu is NULL");
		return -EINVAL;
	}

	client = is_mcu->client;

	//fw_sdcard = false;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	snprintf(fw_name, sizeof(fw_name), "%s%s", IS_OIS_SDCARD_PATH, name);

	setup_binary_loader(&bin, 3, -EAGAIN, NULL, NULL);
	ret = request_binary(&bin, IS_OIS_SDCARD_PATH, name, &client->dev);

	if (ret) {
		info("mcu failed to open SDCARD fw!!!\n");
		goto request_fw;
	}

	fw_requested = 0;
	size = bin.size;
	info("mcu start read sdcard, file path %s, size %lu Bytes\n", fw_name, size);

	*buf = pablo_malloc(size, GFP_KERNEL);
	if (!(*buf)) {
		err("failed to allocate memory");
		ret = -ENOMEM;
		goto p_err;
	}

	memcpy(*buf, (char *)bin.data, bin.size);

	is_ois_get_phone_version(&ois_pinfo);

	memcpy(&is_mcu->vdrinfo_bin[0], *buf + 0x807C, sizeof(is_mcu->vdrinfo_bin));
	is_mcu->hw_bin[0] = *(*buf + 0x80FB);
	is_mcu->hw_bin[1] = *(*buf + 0x80FA);
	is_mcu->hw_bin[2] = *(*buf + 0x80F9);
	is_mcu->hw_bin[3] = *(*buf + 0x80F8);
	memcpy(ois_pinfo->header_ver, is_mcu->hw_bin, 4);
	memcpy(&ois_pinfo->header_ver[4], *buf + 0x807C, 4);

	//fw_sdcard = true;
	if (size <= OIS_BIN_LEN) {
		//ois_device->not_crc_bin = true;
		info("mcu fw binary size = %ld.\n", size);
	}

request_fw:
	if (fw_requested) {
		snprintf(fw_name, sizeof(fw_name), "%s", name);
		set_fs(old_fs);
		retry_count = 3;
		ret = request_firmware(&fw_blob, fw_name, &client->dev);
		while (--retry_count && ret == -EAGAIN) {
			err("request_firmware retry(count:%d)", retry_count);
			ret = request_firmware(&fw_blob, fw_name, &client->dev);
		}

		if (ret) {
			err("request_firmware is fail(ret:%d)", ret);
			ret = -EINVAL;
			goto p_err;
		}

		if (!fw_blob) {
			err("fw_blob is NULL");
			ret = -EINVAL;
			goto p_err;
		}

		if (!fw_blob->data) {
			err("fw_blob->data is NULL");
			ret = -EINVAL;
			goto p_err;
		}

		size = fw_blob->size;

		*buf = pablo_malloc(size, GFP_KERNEL);
		if (!(*buf)) {
			err("failed to allocate memory");
			ret = -ENOMEM;
			goto p_err;
		}

		memcpy((void *)(*buf), fw_blob->data, size);
		memcpy(&is_mcu->vdrinfo_bin[0], *buf + 0x807C, sizeof(is_mcu->vdrinfo_bin));
		is_mcu->hw_bin[0] = *(*buf + 0x80FB);
		is_mcu->hw_bin[1] = *(*buf + 0x80FA);
		is_mcu->hw_bin[2] = *(*buf + 0x80F9);
		is_mcu->hw_bin[3] = *(*buf + 0x80F8);
		memcpy(ois_pinfo->header_ver, is_mcu->hw_bin, 4);
		memcpy(&ois_pinfo->header_ver[4], *buf + 0x807C, 4);

		if (size <= OIS_BIN_LEN) {
			//ois_device->not_crc_bin = true;
			info("mcu fw binary size = %lu.", size);
		}

		info("mcu firmware is loaded from Phone binary.");
	}

p_err:
	*buf_size = size;
	info("[%s] mcu binary hw ver = %c%c%c%c, vdrinfo ver = %c%c%c%c", __func__,
		is_mcu->hw_bin[0], is_mcu->hw_bin[1], is_mcu->hw_bin[2], is_mcu->hw_bin[3],
		is_mcu->vdrinfo_bin[0], is_mcu->vdrinfo_bin[1], is_mcu->vdrinfo_bin[2], is_mcu->vdrinfo_bin[3]);

	if (!fw_requested) {
		release_binary(&bin);
		set_fs(old_fs);
	} else {
		if (!IS_ERR_OR_NULL(fw_blob))
			release_firmware(fw_blob);
	}
#endif
	return ret;
}
#endif /* USE_KERNEL_VFS_READ_WRITE */

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

void is_external_mcu_fw_update(struct is_core *core)
{
	u8 *buf = NULL;
	u8 *buf_cal = NULL;
	int ret = 0;
	int empty_check = 0;
	bool need_reset = false;
	int retry_count = 3;
	u32 address = 0;
	u32 data_size = 0;
	ulong size = 0;
	u8 SendData[256] = {0, };
#ifdef UPDATE_OIS_FIRMWARE_ONLY_WHEN_NECESSARY
	int vdrinfo_bin = 0;
	int vdrinfo_mcu = 0;
#endif
	struct i2c_client *client = NULL;
	struct is_mcu *is_mcu = NULL;
	struct v4l2_subdev *subdev = NULL;

	client = is_mcu_i2c_get_client();
	is_mcu = i2c_get_clientdata(client);
	subdev = is_mcu->subdev;

	info("%s started", __func__);

	ret = is_mcu_open_fw(subdev, FIMC_MCU_FW_NAME, &buf, &size);
	if (ret < 0) {
		err("mcu fw open failed");
		goto p_err;
	}

	ret = is_mcu_fw_version(subdev);
#ifdef UPDATE_OIS_FIRMWARE_ONLY_WHEN_NECESSARY
	if (ret) {
		int isUpload = 0;
		if (is_mcu_version_compare(is_mcu->hw_bin, is_mcu->hw_mcu)) {
			info("Both mcu fw version are same. (HW ver = %c%c%c%c)",
				is_mcu->hw_bin[0], is_mcu->hw_bin[1], is_mcu->hw_bin[2], is_mcu->hw_bin[3]);
		} else {
			isUpload = 1;
			info("Both hw ver are different (binary ver:%c%c%c%c, module ver:%c%c%c%c)",
			is_mcu->hw_bin[0], is_mcu->hw_bin[1], is_mcu->hw_bin[2], is_mcu->hw_bin[3],
			is_mcu->hw_mcu[0], is_mcu->hw_mcu[1], is_mcu->hw_mcu[2], is_mcu->hw_mcu[3]);
		}

		vdrinfo_bin = is_mcu_fw_revision_vdrinfo(is_mcu->vdrinfo_bin);
		vdrinfo_mcu = is_mcu_fw_revision_vdrinfo(is_mcu->vdrinfo_mcu);

		if (vdrinfo_bin == vdrinfo_mcu) {
			info("Both VDRINFO are same. (VDRINFO ver = %c%c%c%c)",
				is_mcu->vdrinfo_bin[0], is_mcu->vdrinfo_bin[1], is_mcu->vdrinfo_bin[2], is_mcu->vdrinfo_bin[3]);
		} else {
			isUpload = 1;
			info("Both vdrinfo are different (binary ver:%c%c%c%c, module ver:%c%c%c%c)",
			is_mcu->vdrinfo_bin[0], is_mcu->vdrinfo_bin[1], is_mcu->vdrinfo_bin[2], is_mcu->vdrinfo_bin[3],
			is_mcu->vdrinfo_mcu[0], is_mcu->vdrinfo_mcu[1], is_mcu->vdrinfo_mcu[2], is_mcu->vdrinfo_mcu[3]);
		}

		if (!isUpload)
			goto p_err;
		else
			info("Update MCU firmware");
	}
#else
	info("Force to update MCU firmware !!!");
#endif
	msleep(50);

retry:
	ret = is_mcu_validation(subdev, core);
	if (ret) {
		err("mcu fw validation failed, retry count = %d", retry_count);
		if (retry_count > 0) {
			msleep(500);
			need_reset = true;
			retry_count--;
			goto retry;
		} else {
			err("mcu fw validation failed. Do not try again.");
			goto p_err;
		}
	}

	empty_check = is_mcu_empty_check_status(subdev);

	ret = is_mcu_erase(subdev, memory_map.flashbase, 65536 - 2048);
	if (ret < 0) {
		err("mcu erase failed, retry count = %d", retry_count);
		if (retry_count > 0) {
			msleep(500);
			need_reset = true;
			retry_count--;
			goto retry;
		} else {
			err("mcu erase failed. Do not try again.");
			goto p_err;
		}
	}

	address = memory_map.flashbase;

	info("mcu start write fw data");

	buf_cal = buf;

	/* Write Data */
	while (size > 0) {
		int i;

		data_size = (u32)((size > FW_TRANS_SIZE) ? FW_TRANS_SIZE : size);
		/* write fw */
		for (i = 0; i < data_size; i++) {
			SendData[i] = buf_cal[i];
		}

		ret = is_mcu_i2c_write(client, address, SendData, data_size);
		if (ret < 0) {
			err("failed to write fw");
			break;
		}
		address += data_size;
		buf_cal += data_size;
		size -= data_size;
	}

	info("mcu end write fw data");

	if (empty_check > 0) {
		if (is_mcu_empty_check_clear(subdev, core) < 0) {
			if (retry_count > 0) {
				retry_count--;
				goto retry;
			} else {
				goto p_err;
			}
		} else {
			is_mcu_disconnect(subdev, core);
		}
	} else {
		is_mcu_disconnect(subdev, core);
	}

	msleep(100);

	info("%s mcu fw update completed.", __func__);

p_err:
	if (buf) {
		pablo_free(buf);
	}

	info("%s end", __func__);

	return;
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

	return 0;

exit:
	info("%s Do not set aperture. onoff = %d", __func__, onoff);

	return -1;
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

	return 0;

exit:
	return -1;
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

bool is_external_mcu_check_fw(struct is_core *core)
{
	int ret = 0;
	struct is_mcu *is_mcu = NULL;
	struct v4l2_subdev *subdev = NULL;

	is_mcu = is_ois_get_mcu(core);
	subdev = is_mcu->subdev;

	is_external_mcu_fw_update(core);

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

int is_external_mcu_read_fw_ver(char *name, char *ver)
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
	.ois_fw_update = is_external_mcu_fw_update,
	.ois_check_fw = is_external_mcu_check_fw,
	.ois_read_fw_ver = is_external_mcu_read_fw_ver,
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

static int is_mcu_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct is_core *core;
	struct ois_mcu_dev *mcu = NULL;
	struct device *dev;
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
	int gpio_mcu_reset = 0;
	int gpio_mcu_boot0 = 0;

	WARN_ON(!client);

	core = is_get_is_core();
	if (!core) {
		err("core device is not yet probed");
		ret = -EPROBE_DEFER;
		goto p_err;
	}

	dev = &client->dev;
	dnode = dev->of_node;

	WARN_ON(!dev);

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

	gpio_mcu_boot0 = of_get_named_gpio(dnode, "gpio_mcu_boot0", 0);
	if (gpio_is_valid(gpio_mcu_boot0)) {
		gpio_request_one(gpio_mcu_boot0, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_mcu_boot0);
	} else {
		err("[MCU] Fail to get mcu boot0 gpio.");
		gpio_mcu_boot0 = 0;
	}

	gpio_mcu_reset = of_get_named_gpio(dnode, "gpio_mcu_reset", 0);
	if (gpio_is_valid(gpio_mcu_reset)) {
		gpio_request_one(gpio_mcu_reset, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_mcu_reset);
	} else {
		err("[MCU] Fail to get mcu reset gpio.");
		gpio_mcu_reset = 0;
	}

	vendor_priv = core->vendor.private_data;

	mcu->ois_wide_init = false;
	mcu->ois_tele_init = false;
	mcu->ois_hw_check = false;

	for (i = 0; i < sensor_id_len; i++) {
		probe_info("%s sensor_id %d\n", __func__, sensor_id[i]);

		is_mcu[i].client = client;
		is_mcu[i].name = MCU_NAME_STM32;
		is_mcu[i].subdev = &subdev_mcu[i];
		is_mcu[i].device = sensor_id[i];
		is_mcu[i].gpio_mcu_boot0 = gpio_mcu_boot0;
		is_mcu[i].gpio_mcu_reset = gpio_mcu_reset;
		is_mcu[i].private_data = core;
		is_mcu[i].ixc_lock = NULL;

		ois[i].subdev = &subdev_ois[i];
		ois[i].device = sensor_id[i];
		ois[i].ois_mode = OPTICAL_STABILIZATION_MODE_OFF;
		ois[i].pre_ois_mode = OPTICAL_STABILIZATION_MODE_OFF;
		ois[i].ois_shift_available = false;
		ois[i].ixc_lock = NULL;
		ois[i].client = client;
		ois[i].ois_ops = &ois_ops_mcu;
		set_ois_comm_ops(&external_mcu_ops);

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

		v4l2_i2c_subdev_init(&subdev_mcu[i], client, &subdev_ops);
		v4l2_set_subdevdata(&subdev_mcu[i], mcu);
		v4l2_set_subdev_hostdata(&subdev_mcu[i], &is_mcu[i]);
	}

	i2c_set_clientdata(client, mcu);

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

static void mcu_remove(struct i2c_client *client)
{
#ifdef CONFIG_SENSORCORE_MCU_CONTROL
	ois_fw_update_unregister();
	ois_reset_unregister();
#endif
}

#ifdef CONFIG_OF
static const struct of_device_id exynos_is_mcu_match[] = {
	{
		.compatible = "samsung,exynos-is-ois-external-mcu",
	},
	{},
};
#endif

static const struct i2c_device_id mcu_idt[] = {
	{ MCU_NAME, 0 },
	{},
};

struct i2c_driver sensor_mcu_driver = {
	.driver = {
		.name	= MCU_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = exynos_is_mcu_match
#endif
	},
	.probe	= is_mcu_probe,
	.remove	= mcu_remove,
	.id_table = mcu_idt
};

struct i2c_driver *get_external_ois_i2c_driver(void)
{
	return &sensor_mcu_driver;
}

#ifndef MODULE
static int __init sensor_mcu_init(void)
{
	int ret;

	ret = i2c_add_driver(&sensor_mcu_driver);
	if (ret)
		err("failed to add %s driver: %d\n",
				sensor_mcu_driver.driver.name, ret);

	return ret;
}
late_initcall_sync(sensor_mcu_init);
#endif

MODULE_LICENSE("GPL v2");
