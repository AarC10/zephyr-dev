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

/**
 * Set the threshold for activity detection for a single axis
 * @param dev - The device structure.
 * @param threshold_val - Threshold value scaled at 780 mg/LSB for detecting activity
 * @param act - The activity config structure.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl375_set_activity_threshold(const struct device *dev,
					  uint8_t threshold_val,
					  const struct adxl375_activity_threshold *act)
{
	struct adxl375_data *data = dev->data;
	return data->hw_tf->write_reg(dev, ADXL375_THRESH_ACT, threshold_val);
}

/**
 * Set the threshold for activity detection for all 3-axis
 * @param dev - The device structure.
 * @param axis_reg_h - The high part of the activity register.
 * @param act - The activity config structure.
 * @return 0 in case of success, negative error code otherwise.
 */
/*static int adxl375_set_activity_threshold_xyz(const struct device *dev,
					      uint8_t axis_reg_h,
					      const struct adxl375_activity_threshold *act)
{
	int i, ret;

	for (i = 0; i < 3; i++) {
		ret = adxl375_set_activity_threshold(dev, axis_reg_h, act);
		if (ret) {
			return ret;
		}
		axis_reg_h += 2U;
	}

	return 0;
}
*/


static int adxl375_get_power_ctl(const struct device *dev, uint8_t *power_ctl_val)
{
	struct adxl375_data *data = dev->data;
	return data->hw_tf->read_reg(dev, ADXL375_POWER_CTL, power_ctl_val);
}

static int adxl375_set_power_ctl_link_bit(const struct device *dev, uint8_t link_bit) {
	struct adxl375_data *data = dev->data;

	return data->hw_tf->write_reg_mask(dev, ADXL375_POWER_CTL,
					   ADXL375_POWER_CTL_LINK_MSK,
					   ADXL375_POWER_CTL_LINK_MODE(link_bit));
}


static int adxl375_set_power_ctl_autosleep_bit(const struct device *dev, uint8_t autosleep_bit) {
	struct adxl375_data *data = dev->data;

	return data->hw_tf->write_reg_mask(dev, ADXL375_POWER_CTL,
					   ADXL375_POWER_CTL_AUTO_SLEEP_MSK,
					   ADXL375_POWER_CTL_AUTO_SLEEP_MODE(autosleep_bit));
}



static int adxl375_set_power_ctl_measure_bit(const struct device *dev, uint8_t measure_bit) {
	struct adxl375_data *data = dev->data;

	return data->hw_tf->write_reg_mask(dev, ADXL375_POWER_CTL,
					   ADXL375_POWER_CTL_MEASURE_MSK,
					   ADXL375_POWER_CTL_MEASURE_MODE(measure_bit));
}


static int adxl375_set_power_ctl_sleep_bit(const struct device *dev, uint8_t link_bit) {
	struct adxl375_data *data = dev->data;

	return data->hw_tf->write_reg_mask(dev, ADXL375_POWER_CTL,
					   ADXL375_POWER_CTL_LINK_MSK,
					   ADXL375_POWER_CTL_LINK_MODE(link_bit));
}


static int adxl375_set_power_ctl_wakeup_bits(const struct device *dev, uint8_t wakeup_bit) {
	struct adxl375_data *data = dev->data;

	return data->hw_tf->write_reg_mask(dev, ADXL375_POWER_CTL,
					   ADXL375_POWER_CTL_WAKEUP_MSK,
					   ADXL375_POWER_CTL_WAKEUP_MODE(wakeup_bit));
}

/**
 * Select the desired output signal bandwidth.
 * @param dev - The device structure.
 * @param bw - bandwidth.
 *		Accepted values: ADXL375_BW_200HZ
 *				 ADXL375_BW_400HZ
 *				 ADXL375_BW_800HZ
 *				 ADXL375_BW_1600HZ
 *				 ADXL375_BW_3200HZ
 *				 ADXL375_BW_LPF_DISABLED
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl375_set_bandwidth(const struct device *dev,
				 enum adxl375_bandwidth bw)
{
	int ret;
	uint8_t mask;
	struct adxl375_data *data = dev->data;

	if (bw == ADXL375_BW_LPF_DISABLED) {
		mask = ADXL375_POWER_CTL_LPF_DIS_MSK;
	} else {
		mask = 0U;
	}

	ret = data->hw_tf->write_reg_mask(dev, ADXL375_POWER_CTL,
					  ADXL375_POWER_CTL_LPF_DIS_MSK, mask);
	if (ret) {
		return ret;
	}

	return data->hw_tf->write_reg_mask(dev, ADXL375_MEASURE,
					   ADXL375_MEASURE_BANDWIDTH_MSK,
					   ADXL375_MEASURE_BANDWIDTH_MODE(bw));
}

/**
 * Set Output data rate.
 * @param dev - The device structure.
 * @param odr - Output data rate.
 *		Accepted values: ADXL375_ODR_400HZ
 *				 ADXL375_ODR_800HZ
 *				 ADXL375_ODR_1600HZ
 *				 ADXL375_ODR_3200HZ
 *				 ADXL375_ODR_6400HZ
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl375_set_odr(const struct device *dev, enum adxl375_odr odr)
{
	struct adxl375_data *data = dev->data;

	return data->hw_tf->write_reg_mask(dev, ADXL375_TIMING,
					   ADXL375_TIMING_ODR_MSK,
					   ADXL375_TIMING_ODR_MODE(odr));
}

/**
 * Select instant on threshold
 * @param dev - The device structure.
 * @param mode - 0 = low threshold, 1 = high threshold.
 *		Accepted values: ADXL375_INSTANT_ON_LOW_TH
 *				 ADXL375_INSTANT_ON_HIGH_TH
 * @return 0 in case of success, negative error code otherwise.
 */
/* static int adxl375_set_instant_on_th(const struct device *dev,
				     enum adxl375_instant_on_th_mode mode)
{
	struct adxl375_data *data = dev->data;

	return data->hw_tf->write_reg_mask(dev, ADXL375_POWER_CTL,
					   ADXL375_POWER_CTL_INSTANT_ON_TH_MSK,
					   ADXL375_POWER_CTL_INSTANT_ON_TH_MODE(mode));
} */

/**
 * Set the filter settling period.
 * @param dev - The device structure.
 * @param mode - settle period
 *		Accepted values: ADXL375_FILTER_SETTLE_370
 *				 ADXL375_FILTER_SETTLE_16
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl375_set_filter_settle(const struct device *dev,
				     enum adxl375_filter_settle mode)
{
	struct adxl375_data *data = dev->data;

	return data->hw_tf->write_reg_mask(dev, ADXL375_POWER_CTL,
					   ADXL375_POWER_CTL_FIL_SETTLE_MSK,
					   ADXL375_POWER_CTL_FIL_SETTLE_MODE(mode));
}

/**
 * Configure the INT1 and INT2 interrupt pins.
 * @param dev - The device structure.
 * @param int1 -  INT1 interrupt pins.
 * @param int2 -  INT2 interrupt pins.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl375_interrupt_config(const struct device *dev,
				    uint8_t int1,
				    uint8_t int2)
{
	int ret;
	struct adxl375_data *data = dev->data;

	ret = data->hw_tf->write_reg(dev, ADXL375_INT1_MAP, int1);
	if (ret) {
		return ret;
	}

	return  data->hw_tf->write_reg(dev, ADXL375_INT2_MAP, int2);

}

/**
 * Get the STATUS, STATUS2, FIFO_ENTRIES and FIFO_ENTRIES2 registers data
 * @param dev - The device structure.
 * @param status1 - Data stored in the STATUS1 register
 * @param status2 - Data stored in the STATUS2 register
 * @param fifo_entries - Number of valid data samples present in the
 *			 FIFO buffer (0 to 512)
 * @return 0 in case of success, negative error code otherwise.
 */
int adxl375_get_status(const struct device *dev,
			   uint8_t *status1,
			   uint8_t *status2,
			   uint16_t *fifo_entries)
{
	struct adxl375_data *data = dev->data;
	uint8_t buf[4], length = 1U;
	int ret;

	if (status2) {
		length++;
	}

	if (fifo_entries) {
		length += 2U;
	}

	ret = data->hw_tf->read_reg_multiple(dev, ADXL375_STATUS_1, buf, length);

	*status1 = buf[0];

	if (status2) {
		*status2 = buf[1];
	}

	if (fifo_entries) {
		*fifo_entries = ((buf[2] & 0x3) << 8) | buf[3];
	}

	return ret;
}


/**
 * Retrieve 3-axis acceleration data
 * @param dev - The device structure.
 * @param maxpeak - Retrieve the highest magnitude (x, y, z) sample recorded
 *		    since the last read of the MAXPEAK registers
 * @param accel_data - pointer to a variable of type adxl375_xyz_accel_data
 *		      where (x, y, z) acceleration data will be stored.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl375_get_accel_data(const struct device *dev, bool maxpeak,
				  struct adxl375_xyz_accel_data *accel_data)
{
	struct adxl375_data *data = dev->data;
	uint8_t buf[6];
	uint8_t status1;
	int ret;

	if (!IS_ENABLED(CONFIG_ADXL375_TRIGGER)) {
		do {
			adxl375_get_status(dev, &status1, NULL, NULL);
		} while (!(ADXL375_STATUS_1_DATA_RDY(status1)));
	}

	ret = data->hw_tf->read_reg_multiple(dev, maxpeak ? ADXL375_X_MAXPEAK_H :
					     ADXL375_X_DATA_H, buf, 6);

	accel_data->x = (buf[0] << 8) | (buf[1] & 0xF0);
	accel_data->y = (buf[2] << 8) | (buf[3] & 0xF0);
	accel_data->z = (buf[4] << 8) | (buf[5] & 0xF0);

	return ret;
}

static int adxl375_attr_set_odr(const struct device *dev,
				enum sensor_channel chan,
				enum sensor_attribute attr,
				const struct sensor_value *val)
{
	enum adxl375_odr odr;

	switch (val->val1) {
	case 400:
		odr = ADXL375_ODR_400HZ;
		break;
	case 800:
		odr = ADXL375_ODR_800HZ;
		break;
	case 1600:
		odr = ADXL375_ODR_1600HZ;
		break;
	case 3200:
		odr = ADXL375_ODR_3200HZ;
		break;
	case 6400:
		odr = ADXL375_ODR_6400HZ;
		break;
	default:
		return -EINVAL;
	}

	return adxl375_set_odr(dev, odr);
}

static int adxl375_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return adxl375_attr_set_odr(dev, chan, attr, val);
	case SENSOR_ATTR_UPPER_THRESH:
	case SENSOR_ATTR_LOWER_THRESH:
		return adxl375_attr_set_thresh(dev, chan, attr, val);
	default:
		return -ENOTSUP;
	}
}

static int adxl375_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	const struct adxl375_dev_config *cfg = dev->config;
	struct adxl375_data *data = dev->data;

	return adxl375_get_accel_data(dev, cfg->max_peak_detect_mode,
				      &data->sample);
}

static void adxl375_accel_convert(struct sensor_value *val, int16_t value)
{
	/*
	 * Sensor resolution is 100mg/LSB, 12-bit value needs to be right
	 * shifted by 4 or divided by 16. Overall this results in a scale of 160
	 */
	int32_t micro_ms2 = value * (SENSOR_G / (16 * 1000 / 100));

	val->val1 = micro_ms2 / 1000000;
	val->val2 = micro_ms2 % 1000000;
}

static int adxl375_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct adxl375_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		adxl375_accel_convert(val, data->sample.x);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		adxl375_accel_convert(val, data->sample.y);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		adxl375_accel_convert(val, data->sample.z);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		adxl375_accel_convert(val++, data->sample.x);
		adxl375_accel_convert(val++, data->sample.y);
		adxl375_accel_convert(val, data->sample.z);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api adxl375_api_funcs = {
	.attr_set     = adxl375_attr_set,
	.sample_fetch = adxl375_sample_fetch,
	.channel_get  = adxl375_channel_get,
#ifdef CONFIG_ADXL375_TRIGGER
	.trigger_set = adxl375_trigger_set,
#endif

};

static int adxl375_probe(const struct device *dev)
{
	const struct adxl375_dev_config *cfg = dev->config;
	struct adxl375_data *data = dev->data;
	uint8_t dev_id;
	uint8_t part_id;
	int ret;

	ret = data->hw_tf->read_reg(dev, ADXL375_DEVID, &dev_id);
	if (ret) {
		return ret;
	}


	if (dev_id != ADXL375_DEVID_VAL) {
		LOG_ERR("failed to read id (0x%X)", dev_id);
		return -ENODEV;
	}

#ifdef CONFIG_ADXL375_TRIGGER
	data->act_proc_mode = ADXL375_LINKED,
#else
	data->act_proc_mode = ADXL375_LOOPED,
#endif

	/* Device settings */
	ret = adxl375_set_op_mode(dev, ADXL375_STANDBY);
	if (ret) {
		return ret;
	}

	ret = adxl375_set_bandwidth(dev, cfg->bw);
	if (ret) {
		return ret;
	}

	ret = adxl375_set_odr(dev, cfg->odr);
	if (ret) {
		return ret;
	}

	ret = adxl375_set_wakeup_rate(dev, cfg->wur);
	if (ret) {
		return ret;
	}

	ret = adxl375_set_autosleep(dev, cfg->autosleep);
	if (ret) {
		return ret;
	}

	ret = adxl375_set_instant_on_th(dev, cfg->th_mode);
	if (ret) {
		return ret;
	}

#ifdef CONFIG_ADXL375_TRIGGER
	if (adxl375_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupt!");
		return -EIO;
	}
#endif

	// ret = adxl375_interrupt_config(dev, cfg->int1_config, cfg->int2_config);
	// if (ret) {
	// 	return ret;
	// }

	return adxl375_set_op_mode(dev, cfg->op_mode);
	// if (ret) {
	// 	return ret;
	// }
	//
	// return adxl375_set_act_proc_mode(dev, data->act_proc_mode);
}

static int adxl375_init(const struct device *dev)
{
	int ret;
	const struct adxl375_dev_config *cfg = dev->config;

	ret = cfg->bus_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize sensor bus");
		return ret;
	}

	if (adxl375_probe(dev) < 0) {
		return -ENODEV;
	}

	return 0;
}

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "ADXL375 driver enabled without any devices"
#endif

/*
 * Device creation macro, shared by ADXL375_DEFINE_SPI() and
 * ADXL375_DEFINE_I2C().
 */

#define ADXL375_DEVICE_INIT(inst)					\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			      adxl375_init,				\
			      NULL,					\
			      &adxl375_data_##inst,			\
			      &adxl375_config_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_SENSOR_INIT_PRIORITY,		\
			      &adxl375_api_funcs);

/*
 * Instantiation macros used when a device is on a SPI bus.
 */

#ifdef CONFIG_ADXL375_TRIGGER
#define ADXL375_CFG_IRQ(inst) \
		.interrupt = GPIO_DT_SPEC_INST_GET(inst, int1_gpios),
#else
#define ADXL375_CFG_IRQ(inst)
#endif /* CONFIG_ADXL375_TRIGGER */

#define ADXL375_CONFIG(inst)								\
		.bw = DT_INST_PROP(inst, bw),						\
		.hpf = DT_INST_PROP(inst, hpf),						\
		.odr = DT_INST_PROP(inst, odr),						\
		.max_peak_detect_mode = IS_ENABLED(CONFIG_ADXL375_PEAK_DETECT_MODE),	\
		.th_mode = ADXL375_INSTANT_ON_LOW_TH,					\
		.autosleep = false,							\
		.wur = ADXL375_WUR_52ms,						\
		.activity_th.thresh = CONFIG_ADXL375_ACTIVITY_THRESHOLD / 100,		\
		.activity_th.referenced =						\
			IS_ENABLED(CONFIG_ADXL375_REFERENCED_ACTIVITY_DETECTION_MODE),	\
		.activity_th.enable = 1,						\
		.activity_time = CONFIG_ADXL375_ACTIVITY_TIME,				\
		.inactivity_th.thresh = CONFIG_ADXL375_INACTIVITY_THRESHOLD / 100,	\
		.inactivity_th.referenced =						\
			IS_ENABLED(CONFIG_ADXL375_REFERENCED_ACTIVITY_DETECTION_MODE),	\
		.inactivity_th.enable = 1,						\
		.inactivity_time = CONFIG_ADXL375_INACTIVITY_TIME,			\
		.filter_settle = ADXL375_FILTER_SETTLE_370,				\
		.fifo_config.fifo_mode = ADXL375_FIFO_STREAMED,				\
		.fifo_config.fifo_format = ADXL375_XYZ_PEAK_FIFO,			\
		.fifo_config.fifo_samples = 128,					\
		.op_mode = ADXL375_FULL_BW_MEASUREMENT,					\

#define ADXL375_CONFIG_SPI(inst)					\
	{								\
		.bus_init = adxl375_spi_init,				\
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) |	\
					SPI_TRANSFER_MSB, 0),		\
		ADXL375_CONFIG(inst)					\
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, int1_gpios),	\
		(ADXL375_CFG_IRQ(inst)), ())				\
	}

#define ADXL375_DEFINE_SPI(inst)					\
	static struct adxl375_data adxl375_data_##inst;			\
	static const struct adxl375_dev_config adxl375_config_##inst =	\
		ADXL375_CONFIG_SPI(inst);				\
	ADXL375_DEVICE_INIT(inst)

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define ADXL375_CONFIG_I2C(inst)					\
	{								\
		.bus_init = adxl375_i2c_init,				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),			\
		ADXL375_CONFIG(inst)					\
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, int1_gpios),	\
		(ADXL375_CFG_IRQ(inst)), ())				\
	}

#define ADXL375_DEFINE_I2C(inst)					\
	static struct adxl375_data adxl375_data_##inst;			\
	static const struct adxl375_dev_config adxl375_config_##inst =	\
		ADXL375_CONFIG_I2C(inst);				\
	ADXL375_DEVICE_INIT(inst)
/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define ADXL375_DEFINE(inst)						\
	COND_CODE_1(DT_INST_ON_BUS(inst, spi),				\
		    (ADXL375_DEFINE_SPI(inst)),				\
		    (ADXL375_DEFINE_I2C(inst)))

DT_INST_FOREACH_STATUS_OKAY(ADXL375_DEFINE)
