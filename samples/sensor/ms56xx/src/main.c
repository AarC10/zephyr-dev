/*
 * Copyright (c) 2024 Aaron Chan <aarchan108@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

void main(void)
{
	struct sensor_value temp, press;
	const struct device *dev;

	/* Try to find a MS56XX/MS5607/MS5611 device by compatible */
#if DT_HAS_COMPAT_STATUS_OKAY(meas_ms56xx)
	dev = DEVICE_DT_GET_ANY(meas_ms56xx);
#elif DT_HAS_COMPAT_STATUS_OKAY(meas_ms5607)
	dev = DEVICE_DT_GET_ANY(meas_ms5607);
#elif DT_HAS_COMPAT_STATUS_OKAY(meas_ms5611)
	dev = DEVICE_DT_GET_ANY(meas_ms5611);
#else
	printk("No MS56XX/MS5607/MS5611 devices found.\n");
	return;
#endif

	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev->name);
		return;
	}

	printk("Found device %s - ", dev->name);
	
#if DT_HAS_COMPAT_STATUS_OKAY(meas_ms56xx)
	if (dev == DEVICE_DT_GET_ANY(meas_ms56xx)) {
		printk("MS56XX generic driver\n");
	} else
#endif
#if DT_HAS_COMPAT_STATUS_OKAY(meas_ms5607)
	if (dev == DEVICE_DT_GET_ANY(meas_ms5607)) {
		printk("MS5607 specific driver\n");
	} else
#endif
#if DT_HAS_COMPAT_STATUS_OKAY(meas_ms5611)
	if (dev == DEVICE_DT_GET_ANY(meas_ms5611)) {
		printk("MS5611 specific driver\n");
	} else
#endif
	{
		printk("Unknown driver variant\n");
	}

	while (1) {
		if (sensor_sample_fetch(dev) < 0) {
			printk("Cannot fetch sample from %s\n", dev->name);
			continue;
		}

		if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
			printk("Cannot get temperature from %s\n", dev->name);
			continue;
		}

		if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press) < 0) {
			printk("Cannot get pressure from %s\n", dev->name);
			continue;
		}

		printk("Temperature: %d.%06d °C | Pressure: %d.%06d kPa\n",
			temp.val1, temp.val2, press.val1, press.val2);

		k_sleep(K_MSEC(1000));
	}
}