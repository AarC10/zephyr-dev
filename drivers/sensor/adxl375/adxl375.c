/*
 * Copyright (c) 2023 Aaron Chan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asm-generic/errno-base.h>
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

static int adxl375_check_id(const struct device *dev) {
	struct adxl375_data *data = dev->data;
	uint8_t device_id = 0;

	int ret = data->hw_tf->read_reg(dev, ADXL375_DEVID, &device_id);

	if (ret != 0) {
		return ret;
	}

	if (ADXL375_DEVID_VAL != device_id) {
		return -ENODEV;
	}

	return 0;
}

static int adxl375_set_odr()
{


}

static int adxl375_set_op_mode()
{

}

static int adxl375_wakeup()
{

}

static int adxl375_init()
{

	int ret;
	const struct adxl375_dev_config *cfg = dev->config;

	ret = cfg->bus_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialzie sensor bus.");
		return ret;
	}

	int ret = adxl375_check_id();
	if (!ret) {
		return ret;
	}


	ret = adxl375_set_odr();

	ret = adxl375_set_op_mode();

	ret = adxl375_wakeup();

	return -1;

}

static int adxl375_sample_fetch(const struct device *dev, enum sensor_channel chan)
{

	return -1;
}

static int adxl375_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	return -1;
}

static const struct sensor_driver_api adxl375_driver_api =
{
	.sample_fetch = adxl375_sample_fetch,
	.channel_get = adxl375_channel_get
};
