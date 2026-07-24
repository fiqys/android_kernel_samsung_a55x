/*
 * Samsung Exynos5 SoC series FIMC-IS OIS driver
 *
 * exynos5 fimc-is core functions
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef IS_VENDOR_OIS_ADVANCED_H
#define IS_VENDOR_OIS_ADVANCED_H

#include "pablo-hw-api-common.h"

/* Utility MACROs */

#ifndef NTOHL
#define NTOHL(x)	((((x) & 0xFF000000U) >> 24) | \
					(((x) & 0x00FF0000U) >>  8) | \
					(((x) & 0x0000FF00U) <<  8) | \
					(((x) & 0x000000FFU) << 24))
#endif
#ifndef HTONL
#define HTONL(x)	NTOHL(x)
#endif

#ifndef NTOHS
#define NTOHS(x)	(((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00))
#endif
#ifndef HTONS
#define HTONS(x)	NTOHS(x)
#endif

#define	FIMC_MCU_FW_NAME	"is_fw_mcu.bin"
#define	FW_TRANS_SIZE	256
#define	OIS_BIN_LEN		45056

#define	FW_RELEASE_YEAR  0
#define	FW_RELEASE_MONTH 1
#define	FW_RELEASE_COUNT 2

#define	FW_DRIVER_IC     0
#define	FW_GYRO_SENSOR   1
#define	FW_MODULE_TYPE   2
#define	FW_PROJECT       3
#define	FW_CORE_VERSION  4

#define	OIS_MEM_STATUS_RETRY	6
#define	POSITION_NUM	512
#define	AF_BOUNDARY		(1 << 6)

#define	MAX_GYRO_EFS_DATA_LENGTH	30

enum{
	eBIG_ENDIAN = 0, // big endian
	eLIT_ENDIAN = 1  // little endian
};

#define	SWAP32(x)	((((x) & 0xff000000) >> 24) | \
				(((x) & 0x00ff0000) >> 8) | \
				(((x) & 0x0000ff00) << 8) | \
				(((x) & 0x000000ff) << 24))
#define	RND_DIV(num, den) ((num > 0) ? (num + (den >> 1)) / den : (num - (den >> 1)) / den)
#define	SCALE				10000
#define	Coef_angle_max		3500		// unit : 1/SCALE, OIS Maximum compensation angle, 0.35*SCALE
#define	SH_THRES			798000		// unit : 1/SCALE, 39.9*SCALE
#define	Gyrocode			1000			// Gyro input code for 1 angle degree

bool is_mcu_halltest_aperture(struct v4l2_subdev *subdev, u16 *hall_value);
void is_mcu_set_aperture_onboot(struct is_core *core);
int is_ois_advanced_read_u8(int cmd, u8 *data);
int is_ois_advanced_write_u8(int cmd, u8 data);

struct platform_driver *get_aois_platform_driver(void);
#endif
