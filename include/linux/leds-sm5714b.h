/*
 * sm5714b.h - mfd driver for SM5714.
 *
 * Copyright (C) 2025 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LEDS_SM5714B_H__
#define __LEDS_SM5714B_H__

#include <linux/mfd/sm/sm5714b/sm5714b.h>
#include <linux/mfd/sm/sm5714b/sm5714b-private.h>

#define MAX_TORCH_LEVELS_5714 5

extern int32_t sm5714b_fled_mode_ctrl(int state, uint32_t brightness);
int sm5714b_fled_torch_gpio(u8 intensity);
int sm5714b_create_sysfs(struct class *class);

#endif /* __LEDS_SM5714B_H__ */
