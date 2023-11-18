/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ADXL375_adxl375_H_
#define ZEPHYR_DRIVERS_SENSOR_ADXL375_adxl375_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <zephyr/drivers/spi.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <zephyr/drivers/i2c.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

/*
 * ADXL375 registers definition
 */
#define ADXL375_DEVID		0x00u  /* Analog Devices accelerometer ID */
#define ADXL375_DEVID_MST	0x01u  /* Analog Devices MEMS device ID */
#define ADXL375_PARTID		0x02u  /* Device ID */
#define ADXL375_REVID		0x03u  /* product revision ID*/
#define ADXL375_STATUS_1	0x04u  /* Status register 1 */
#define ADXL375_STATUS_2	0x05u  /* Status register 2 */
#define ADXL375_FIFO_ENTRIES_2	0x06u  /* Valid data samples in the FIFO */
#define ADXL375_FIFO_ENTRIES_1	0x07u  /* Valid data samples in the FIFO */
#define ADXL375_X_DATA_H	0x08u  /* X-axis acceleration data [11:4] */
#define ADXL375_X_DATA_L	0x09u  /* X-axis acceleration data [3:0] */
#define ADXL375_Y_DATA_H	0x0Au  /* Y-axis acceleration data [11:4] */
#define ADXL375_Y_DATA_L	0x0Bu  /* Y-axis acceleration data [3:0] */
#define ADXL375_Z_DATA_H	0x0Cu  /* Z-axis acceleration data [11:4] */
#define ADXL375_Z_DATA_L	0x0Du  /* Z-axis acceleration data [3:0] */
#define ADXL375_X_MAXPEAK_H	0x15u  /* X-axis MaxPeak acceleration data */
#define ADXL375_X_MAXPEAK_L	0x16u  /* X-axis MaxPeak acceleration data */
#define ADXL375_Y_MAXPEAK_H	0x17u  /* Y-axis MaxPeak acceleration data */
#define ADXL375_Y_MAXPEAK_L	0x18u  /* Y-axis MaxPeak acceleration data */
#define ADXL375_Z_MAXPEAK_H	0x19u  /* Z-axis MaxPeak acceleration data */
#define ADXL375_Z_MAXPEAK_L	0x1Au  /* Z-axis MaxPeak acceleration data */
#define ADXL375_OFFSET_X	0x20u  /* X axis offset */
#define ADXL375_OFFSET_Y	0x21u  /* Y axis offset */
#define ADXL375_OFFSET_Z	0x22u  /* Z axis offset */
#define ADXL375_X_THRESH_ACT_H	0x23u  /* X axis Activity Threshold [15:8] */
#define ADXL375_X_THRESH_ACT_L	0x24u  /* X axis Activity Threshold [7:0] */
#define ADXL375_Y_THRESH_ACT_H	0x25u  /* Y axis Activity Threshold [15:8] */
#define ADXL375_Y_THRESH_ACT_L	0x26u  /* Y axis Activity Threshold [7:0] */
#define ADXL375_Z_THRESH_ACT_H	0x27u  /* Z axis Activity Threshold [15:8] */
#define ADXL375_Z_THRESH_ACT_L	0x28u  /* Z axis Activity Threshold [7:0] */
#define ADXL375_TIME_ACT	0x29u  /* Activity Time */
#define ADXL375_X_THRESH_INACT_H	0x2Au  /* X axis Inactivity Threshold */
#define ADXL375_X_THRESH_INACT_L	0x2Bu  /* X axis Inactivity Threshold */
#define ADXL375_Y_THRESH_INACT_H	0x2Cu  /* Y axis Inactivity Threshold */
#define ADXL375_Y_THRESH_INACT_L	0x2Du  /* Y axis Inactivity Threshold */
#define ADXL375_Z_THRESH_INACT_H	0x2Eu  /* Z axis Inactivity Threshold */
#define ADXL375_Z_THRESH_INACT_L	0x2Fu  /* Z axis Inactivity Threshold */
#define ADXL375_TIME_INACT_H	0x30u  /* Inactivity Time [15:8] */
#define ADXL375_TIME_INACT_L	0x31u  /* Inactivity Time [7:0] */
#define ADXL375_X_THRESH_ACT2_H	0x32u  /* X axis Activity2 Threshold [15:8] */
#define ADXL375_X_THRESH_ACT2_L	0x33u  /* X axis Activity2 Threshold [7:0] */
#define ADXL375_Y_THRESH_ACT2_H	0x34u  /* Y axis Activity2 Threshold [15:8] */
#define ADXL375_Y_THRESH_ACT2_L	0x35u  /* Y axis Activity2 Threshold [7:0] */
#define ADXL375_Z_THRESH_ACT2_H	0x36u  /* Z axis Activity2 Threshold [15:8] */
#define ADXL375_Z_THRESH_ACT2_L	0x37u  /* Z axis Activity2 Threshold [7:0] */
#define ADXL375_HPF		0x38u  /* High Pass Filter */
#define ADXL375_FIFO_SAMPLES	0x39u  /* FIFO Samples */
#define ADXL375_FIFO_CTL	0x3Au  /* FIFO Control */
#define ADXL375_INT1_MAP	0x3Bu  /* Interrupt 1 mapping control */
#define ADXL375_INT2_MAP        0x3Cu  /* Interrupt 2 mapping control */
#define ADXL375_TIMING		0x3Du  /* Timing */
#define ADXL375_MEASURE		0x3Eu  /* Measure */
#define ADXL375_POWER_CTL	0x3Fu  /* Power control */
#define ADXL375_SELF_TEST	0x40u  /* Self Test */
#define ADXL375_RESET		0x41u  /* Reset */
#define ADXL375_FIFO_DATA	0x42u  /* FIFO Data */

#define ADXL375_DEVID_VAL	0xADu  /* Analog Devices accelerometer ID */
#define ADXL375_MST_DEVID_VAL	0x1Du  /* Analog Devices MEMS device ID */
#define ADXL375_PARTID_VAL	0xFAu  /* Device ID */
#define ADXL375_REVID_VAL	0x02u  /* product revision ID*/
#define ADXL375_RESET_CODE	0x52u  /* Writing code 0x52 resets the device */

#define ADXL375_READ		0x01u
#define ADXL375_REG_READ(x)	(((x & 0xFF) << 1) | adxl375_READ)
#define ADXL375_REG_WRITE(x)	((x & 0xFF) << 1)
#define ADXL375_TO_I2C_REG(x)	((x) >> 1)

/* ADXL375_POWER_CTL */
#define ADXL375_POWER_CTL_INSTANT_ON_TH_MSK	BIT(5)
#define ADXL375_POWER_CTL_INSTANT_ON_TH_MODE(x)	(((x) & 0x1) << 5)
#define ADXL375_POWER_CTL_FIL_SETTLE_MSK	BIT(4)
#define ADXL375_POWER_CTL_FIL_SETTLE_MODE(x)	(((x) & 0x1) << 4)
#define ADXL375_POWER_CTL_LPF_DIS_MSK		BIT(3)
#define ADXL375_POWER_CTL_LPF_DIS_MODE(x)	(((x) & 0x1) << 3)
#define ADXL375_POWER_CTL_HPF_DIS_MSK		BIT(2)
#define ADXL375_POWER_CTL_HPF_DIS_MODE(x)	(((x) & 0x1) << 2)
#define ADXL375_POWER_CTL_MODE_MSK		GENMASK(1, 0)
#define ADXL375_POWER_CTL_MODE(x)		(((x) & 0x3) << 0)

/* ADXL375_MEASURE */
#define ADXL375_MEASURE_AUTOSLEEP_MSK		BIT(6)
#define ADXL375_MEASURE_AUTOSLEEP_MODE(x)	(((x) & 0x1) << 6)
#define ADXL375_MEASURE_LINKLOOP_MSK		GENMASK(5, 4)
#define ADXL375_MEASURE_LINKLOOP_MODE(x)	(((x) & 0x3) << 4)
#define ADXL375_MEASURE_LOW_NOISE_MSK		BIT(3)
#define ADXL375_MEASURE_LOW_NOISE_MODE(x)	(((x) & 0x1) << 3)
#define ADXL375_MEASURE_BANDWIDTH_MSK		GENMASK(2, 0)
#define ADXL375_MEASURE_BANDWIDTH_MODE(x)	(((x) & 0x7) << 0)

/* ADXL375_TIMING */
#define ADXL375_TIMING_ODR_MSK			GENMASK(7, 5)
#define ADXL375_TIMING_ODR_MODE(x)		(((x) & 0x7) << 5)
#define ADXL375_TIMING_WAKE_UP_RATE_MSK		GENMASK(4, 2)
#define ADXL375_TIMING_WAKE_UP_RATE_MODE(x)	(((x) & 0x7) << 2)
#define ADXL375_TIMING_EXT_CLK_MSK		BIT(1)
#define ADXL375_TIMING_EXT_CLK_MODE(x)		(((x) & 0x1) << 1)
#define ADXL375_TIMING_EXT_SYNC_MSK		BIT(0)
#define ADXL375_TIMING_EXT_SYNC_MODE(x)		(((x) & 0x1) << 0)

/* ADXL375_FIFO_CTL */
#define ADXL375_FIFO_CTL_FORMAT_MSK		GENMASK(5, 3)
#define ADXL375_FIFO_CTL_FORMAT_MODE(x)		(((x) & 0x7) << 3)
#define ADXL375_FIFO_CTL_MODE_MSK		GENMASK(2, 1)
#define ADXL375_FIFO_CTL_MODE_MODE(x)		(((x) & 0x3) << 1)
#define ADXL375_FIFO_CTL_SAMPLES_MSK		BIT(0)
#define ADXL375_FIFO_CTL_SAMPLES_MODE(x)	(((x) > 0xFF) ? 1 : 0)

/* ADXL375_STATUS_1 */
#define ADXL375_STATUS_1_DATA_RDY(x)		(((x) >> 0) & 0x1)
#define ADXL375_STATUS_1_FIFO_RDY(x)		(((x) >> 1) & 0x1)
#define ADXL375_STATUS_1_FIFO_FULL(x)		(((x) >> 2) & 0x1)
#define ADXL375_STATUS_1_FIFO_OVR(x)		(((x) >> 3) & 0x1)
#define ADXL375_STATUS_1_USR_NVM_BUSY(x)	(((x) >> 5) & 0x1)
#define ADXL375_STATUS_1_AWAKE(x)		(((x) >> 6) & 0x1)
#define ADXL375_STATUS_1_ERR_USR_REGS(x)	(((x) >> 7) & 0x1)

/* ADXL375_STATUS_2 */
#define ADXL375_STATUS_2_INACT(x)		(((x) >> 4) & 0x1)
#define ADXL375_STATUS_2_ACTIVITY(x)		(((x) >> 5) & 0x1)
#define ADXL375_STATUS_2_ACTIVITY2(x)		(((x) >> 6) & 0x1)

/* ADXL375_INT1_MAP */
#define ADXL375_INT1_MAP_DATA_RDY_MSK		BIT(0)
#define ADXL375_INT1_MAP_DATA_RDY_MODE(x)	(((x) & 0x1) << 0)
#define ADXL375_INT1_MAP_FIFO_RDY_MSK		BIT(1)
#define ADXL375_INT1_MAP_FIFO_RDY_MODE(x)	(((x) & 0x1) << 1)
#define ADXL375_INT1_MAP_FIFO_FULL_MSK		BIT(2)
#define ADXL375_INT1_MAP_FIFO_FULL_MODE(x)	(((x) & 0x1) << 2)
#define ADXL375_INT1_MAP_FIFO_OVR_MSK		BIT(3)
#define ADXL375_INT1_MAP_FIFO_OVR_MODE(x)	(((x) & 0x1) << 3)
#define ADXL375_INT1_MAP_INACT_MSK		BIT(4)
#define ADXL375_INT1_MAP_INACT_MODE(x)		(((x) & 0x1) << 4)
#define ADXL375_INT1_MAP_ACT_MSK		BIT(5)
#define ADXL375_INT1_MAP_ACT_MODE(x)		(((x) & 0x1) << 5)
#define ADXL375_INT1_MAP_AWAKE_MSK		BIT(6)
#define ADXL375_INT1_MAP_AWAKE_MODE(x)		(((x) & 0x1) << 6)
#define ADXL375_INT1_MAP_LOW_MSK		BIT(7)
#define ADXL375_INT1_MAP_LOW_MODE(x)		(((x) & 0x1) << 7)

/* ADXL375_INT2_MAP */
#define ADXL375_INT2_MAP_DATA_RDY_MSK		BIT(0)
#define ADXL375_INT2_MAP_DATA_RDY_MODE(x)	(((x) & 0x1) << 0)
#define ADXL375_INT2_MAP_FIFO_RDY_MSK		BIT(1)
#define ADXL375_INT2_MAP_FIFO_RDY_MODE(x)	(((x) & 0x1) << 1)
#define ADXL375_INT2_MAP_FIFO_FULL_MSK		BIT(2)
#define ADXL375_INT2_MAP_FIFO_FULL_MODE(x)	(((x) & 0x1) << 2)
#define ADXL375_INT2_MAP_FIFO_OVR_MSK		BIT(3)
#define ADXL375_INT2_MAP_FIFO_OVR_MODE(x)	(((x) & 0x1) << 3)
#define ADXL375_INT2_MAP_INACT_MSK		BIT(4)
#define ADXL375_INT2_MAP_INACT_MODE(x)		(((x) & 0x1) << 4)
#define ADXL375_INT2_MAP_ACT_MSK		BIT(5)
#define ADXL375_INT2_MAP_ACT_MODE(x)		(((x) & 0x1) << 5)
#define ADXL375_INT2_MAP_AWAKE_MSK		BIT(6)
#define ADXL375_INT2_MAP_AWAKE_MODE(x)		(((x) & 0x1) << 6)
#define ADXL375_INT2_MAP_LOW_MSK		BIT(7)
#define ADXL375_INT2_MAP_LOW_MODE(x)		(((x) & 0x1) << 7)

/* ADXL375_HPF */
#define ADXL375_HPF_CORNER(x)			(((x) & 0x3) << 0)

enum adxl375_axis {
	ADXL375_X_AXIS,
	ADXL375_Y_AXIS,
	ADXL375_Z_AXIS
};

enum adxl375_op_mode {
	ADXL375_STANDBY,
	ADXL375_WAKE_UP,
	ADXL375_INSTANT_ON,
	ADXL375_FULL_BW_MEASUREMENT
};

enum adxl375_bandwidth {
	ADXL375_BW_200HZ,
	ADXL375_BW_400HZ,
	ADXL375_BW_800HZ,
	ADXL375_BW_1600HZ,
	ADXL375_BW_3200HZ,
	ADXL375_BW_LPF_DISABLED = 0xC,
};

enum adxl375_hpf_corner {
	ADXL375_HPF_CORNER_0,
	ADXL375_HPF_CORNER_1,
	ADXL375_HPF_CORNER_2,
	ADXL375_HPF_CORNER_3,
	ADXL375_HPF_DISABLED,
};

enum adxl375_act_proc_mode {
	ADXL375_DEFAULT,
	ADXL375_LINKED,
	ADXL375_LOOPED
};

enum adxl375_odr {
	ADXL375_ODR_400HZ,
	ADXL375_ODR_800HZ,
	ADXL375_ODR_1600HZ,
	ADXL375_ODR_3200HZ,
	ADXL375_ODR_6400HZ
};

enum adxl375_instant_on_th_mode {
	ADXL375_INSTANT_ON_LOW_TH,
	ADXL375_INSTANT_ON_HIGH_TH
};

enum adxl375_wakeup_rate {
	ADXL375_WUR_52ms,
	ADXL375_WUR_104ms,
	ADXL375_WUR_208ms,
	ADXL375_WUR_512ms,
	ADXL375_WUR_2048ms,
	ADXL375_WUR_4096ms,
	ADXL375_WUR_8192ms,
	ADXL375_WUR_24576ms
};

enum adxl375_filter_settle {
	ADXL375_FILTER_SETTLE_370,
	ADXL375_FILTER_SETTLE_16
};

enum adxl375_fifo_format {
	ADXL375_XYZ_FIFO,
	ADXL375_X_FIFO,
	ADXL375_Y_FIFO,
	ADXL375_XY_FIFO,
	ADXL375_Z_FIFO,
	ADXL375_XZ_FIFO,
	ADXL375_YZ_FIFO,
	ADXL375_XYZ_PEAK_FIFO,
};

enum adxl375_fifo_mode {
	ADXL375_FIFO_BYPASSED,
	ADXL375_FIFO_STREAMED,
	ADXL375_FIFO_TRIGGERED,
	ADXL375_FIFO_OLD_SAVED
};

struct adxl375_fifo_config {
	enum adxl375_fifo_mode fifo_mode;
	enum adxl375_fifo_format fifo_format;
	uint16_t fifo_samples;
};

struct adxl375_activity_threshold {
	uint16_t thresh;
	bool referenced;
	bool enable;
};

struct adxl375_xyz_accel_data {
	int16_t x;
	int16_t y;
	int16_t z;
};

struct adxl375_transfer_function {
	int (*read_reg_multiple)(const struct device *dev, uint8_t reg_addr,
				 uint8_t *value, uint16_t len);
	int (*write_reg)(const struct device *dev, uint8_t reg_addr,
			 uint8_t value);
	int (*read_reg)(const struct device *dev, uint8_t reg_addr,
			uint8_t *value);
	int (*write_reg_mask)(const struct device *dev, uint8_t reg_addr,
			      uint32_t mask, uint8_t value);
};

struct adxl375_data {
	struct adxl375_xyz_accel_data sample;
	const struct adxl375_transfer_function *hw_tf;
	struct adxl375_fifo_config fifo_config;
	enum adxl375_act_proc_mode act_proc_mode;
#ifdef CONFIG_ADXL375_TRIGGER
	struct gpio_callback gpio_cb;

	sensor_trigger_handler_t th_handler;
	const struct sensor_trigger *th_trigger;
	sensor_trigger_handler_t drdy_handler;
	const struct sensor_trigger *drdy_trigger;
	const struct device *dev;

#if defined(CONFIG_ADXL375_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ADXL375_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
#elif defined(CONFIG_ADXL375_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_ADXL375_TRIGGER */
};

struct adxl375_dev_config {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	struct i2c_dt_spec i2c;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_dt_spec spi;
#endif
	int (*bus_init)(const struct device *dev);
#ifdef CONFIG_ADXL375_TRIGGER
	struct gpio_dt_spec interrupt;
#endif

	enum adxl375_bandwidth bw;
	enum adxl375_hpf_corner hpf;
	enum adxl375_odr odr;

	bool max_peak_detect_mode;

	/* Device Settings */
	bool autosleep;

	struct adxl375_activity_threshold activity_th;
	struct adxl375_activity_threshold activity2_th;
	struct adxl375_activity_threshold inactivity_th;
	struct adxl375_fifo_config fifo_config;

	enum adxl375_wakeup_rate wur;
	enum adxl375_instant_on_th_mode	th_mode;
	enum adxl375_filter_settle filter_settle;
	enum adxl375_op_mode op_mode;

	uint16_t inactivity_time;
	uint8_t activity_time;
	uint8_t int1_config;
	uint8_t int2_config;
};

int adxl375_spi_init(const struct device *dev);
int adxl375_i2c_init(const struct device *dev);

#ifdef CONFIG_ADXL375_TRIGGER
int adxl375_get_status(const struct device *dev,
		       uint8_t *status1, uint8_t *status2, uint16_t *fifo_entries);

int adxl375_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int adxl375_init_interrupt(const struct device *dev);
#endif /* CONFIG_ADXL375_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_ADXL375_adxl375_H_ */
