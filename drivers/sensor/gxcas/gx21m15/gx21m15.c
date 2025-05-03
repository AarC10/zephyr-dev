/*
 * Copyright (c) 2025 Aaron Chan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gxcas_gx21m15

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(GX21M15, CONFIG_SENSOR_LOG_LEVEL);

/* GX21M15 registers */
#define GX21M15_REG_TEMP   0x00 /* Temperature register */
#define GX21M15_REG_CONFIG 0x01 /* Configuration register */
#define GX21M15_REG_THYST  0x02 /* Temperature hysteresis register */
#define GX21M15_REG_TOS    0x03 /* Over-temperature shutdown threshold register */

/* Configuration register bits */
union gx21m15_config_reg {
	uint8_t reg;
	struct {
		uint8_t shutdown: 1;    /* 0: Normal, 1: Shutdown */
		uint8_t int_mode: 1;    /* 0: Comparator, 1: Interrupt */
		uint8_t int_pol: 1;     /* 0: Active low, 1: Active high */
		uint8_t fault_queue: 2; /* Consecutive faults before alert */
		uint8_t reserved: 3;    /* Reserved bits */
	} __packed;
};

struct gx21m15_data {
	int16_t temp;
};

struct gx21m15_config {
	struct i2c_dt_spec i2c;
};

static inline int gx21m15_reg_read(const struct device *dev, uint8_t reg, uint8_t *buf,
				   uint32_t size)
{
	const struct gx21m15_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->i2c, reg, buf, size);
}

static inline int gx21m15_reg_write(const struct device *dev, uint8_t reg, const uint8_t *buf,
				    uint32_t size)
{
	const struct gx21m15_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->i2c, reg, buf, size);
}

static inline int gx21m15_fetch_temp(const struct device *dev)
{
	struct gx21m15_data *data = dev->data;
	uint8_t buf[2];
	int ret;

	ret = gx21m15_reg_read(dev, GX21M15_REG_TEMP, buf, sizeof(buf));
	if (ret) {
		LOG_ERR("Could not fetch temperature [%d]", ret);
		return -EIO;
	}

	/* Temperature data is 11-bit, 2's complement in the upper 11 bits */
	data->temp = sys_get_be16(buf);

	return 0;
}

static int gx21m15_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret;

	switch (chan) {
	case SENSOR_CHAN_ALL:
	case SENSOR_CHAN_AMBIENT_TEMP:
		ret = gx21m15_fetch_temp(dev);
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static void gx21m15_temp_to_sensor_value(int16_t temp, struct sensor_value *val)
{
	/* Temperature data is stored as 11-bit, 2's complement with
       0.125°C resolution, bit-shifted by 5 */
	int32_t temp_milli = (temp >> 5) * 125; /* 0.125 degree C steps */

	val->val1 = temp_milli / 1000;
	val->val2 = (temp_milli % 1000) * 1000;
}

static int gx21m15_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct gx21m15_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		gx21m15_temp_to_sensor_value(data->temp, val);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(sensor, gx21m15_driver_api) = {
	.sample_fetch = gx21m15_sample_fetch,
	.channel_get = gx21m15_channel_get,
};

int gx21m15_init(const struct device *dev)
{
	const struct gx21m15_config *cfg = dev->config;
	union gx21m15_config_reg config;
	int ret;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Configure the sensor - normal mode */
	config.reg = 0;
	config.shutdown = 0;    /* Normal mode (not shutdown) */
	config.int_mode = 0;    /* Comparator mode */
	config.int_pol = 0;     /* Active low */
	config.fault_queue = 1; /* 2 consecutive faults (value 1) */

	ret = gx21m15_reg_write(dev, GX21M15_REG_CONFIG, &config.reg, 1);
	if (ret < 0) {
		LOG_ERR("Failed to write configuration register");
		return -EIO;
	}

	LOG_INF("GX21M15 initialized successfully");
	return 0;
}

#define GX21M15_INST(inst)                                                                         \
	static struct gx21m15_data gx21m15_data_##inst;                                            \
	static const struct gx21m15_config gx21m15_config_##inst = {                               \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, gx21m15_init, NULL, &gx21m15_data_##inst,               \
				     &gx21m15_config_##inst, POST_KERNEL,                          \
				     CONFIG_SENSOR_INIT_PRIORITY, &gx21m15_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GX21M15_INST)
