/*
 * Copyright (c) 2025 Aaron Chan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>

static void process_sample(const struct device *dev)
{
	struct sensor_value temp, press;
	static int sample_count;

	if (sensor_sample_fetch(dev) < 0) {
		printk("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		printk("Cannot read MS56XX temperature channel\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press) < 0) {
		printk("Cannot read MS56XX pressure channel\n");
		return;
	}

	++sample_count;
	printk("Sample %d:\n", sample_count);
	printk("  Temperature: %.2f C\n", sensor_value_to_double(&temp));
	printk("  Pressure: %.2f kPa\n", sensor_value_to_double(&press));
}

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(pressure_sensor));
	const char *sensor_name;

	if (!device_is_ready(dev)) {
		printk("MS56XX sensor device not ready\n");
		return 0;
	}

	/* Determine which sensor is connected based on compatible string */
	if (DT_NODE_HAS_COMPAT(DT_ALIAS(pressure_sensor), meas_ms5607)) {
		sensor_name = "MS5607";
	} else if (DT_NODE_HAS_COMPAT(DT_ALIAS(pressure_sensor), meas_ms5611)) {
		sensor_name = "MS5611";
	} else {
		sensor_name = "MS56XX (unknown)";
	}

	printk("%s sensor device ready\n", sensor_name);

	while (1) {
		process_sample(dev);
		k_sleep(K_MSEC(2000));
	}
	return 0;
}

