/*
 * Copyright (c) 2023 Aaron Chan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/eeprom.h>
// #include <zephyr/drivers/sensor/adxl375.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>

#define ADXL375_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(adi_adxl375)


int main(void)
{
	const struct device *const dev = DEVICE_DT_GET(ADXL375_NODE);
	struct sensor_value accel_x;
	struct sensor_value accel_y;
	struct sensor_value accel_z;

	__ASSERT(device_is_ready(dev), "ADXL375 device not ready");

	printk("Device %s - %p is ready\n", dev->name, dev);

	while (1) {
		int ret = sensor_sample_fetch(dev);
		if (ret) {
			printk("Failed to fetch measurements (%d)\n", ret);
			return 0;
		}

		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

		printk("X: %f\n", sensor_value_to_float(&accel_x));
		printk("Y: %f\n", sensor_value_to_float(&accel_y));
		printk("Z: %f\n", sensor_value_to_float(&accel_z));


		k_sleep(K_MSEC(1000));
	}
	return 0;
}
