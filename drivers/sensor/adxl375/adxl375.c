/*
 * Copyright (c) 2023 Aaron Chan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_adxl375

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <stdlib.h>
#include <zephyr/logging/log.h>

#include "adxl375.h"

LOG_MODULE_REGISTER(ADXL375, CONFIG_SENSOR_LOG_LEVEL);

static int adxl375_init() {


}

static int adxl375_sample_fetch(const struct device *dev, enum sensor_channel chan) {

	return -1;
}

static int adxl375_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
	return -1;
}

static const struct sensor_driver_api adxl375_driver_api = {
	.sample_fetch = adxl375_sample_fetch,
	.channel_get = adxl375_channel_get
}

int adxl375_init(const struct device *dev) {
	return -1;
}


