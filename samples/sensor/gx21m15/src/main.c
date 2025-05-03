/*
 * Copyright (c) 2025 GXCAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#define SLEEP_TIME_MS   1000

static void process_sample(const struct device *dev)
{
	static unsigned int count;
	struct sensor_value temp;
	int rc;

	rc = sensor_sample_fetch(dev);
	if (rc) {
		printk("Sensor sample fetch error: %d\n", rc);
		return;
	}

	rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	if (rc) {
		printk("Sensor channel get error: %d\n", rc);
		return;
	}

	printk("[%u] Temperature: %.2f °C\n", ++count, 
	       (double)temp.val1 + (double)temp.val2 / 1000000.0);
}

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(gxcas_gx21m15);

	if (!dev) {
		printk("No GX21M15 device found.\n");
		return 0;
	}

	if (!device_is_ready(dev)) {
		printk("GX21M15 device %s is not ready\n", dev->name);
		return 0;
	}

	printk("Found device \"%s\", getting temperature samples\n", dev->name);

	while (1) {
		process_sample(dev);
		k_sleep(K_MSEC(SLEEP_TIME_MS));
	}

	return 0;
}