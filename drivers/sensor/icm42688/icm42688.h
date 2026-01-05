/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========= Driver-specific (private) attributes ========= */
#ifndef SENSOR_ATTR_PRIV_START
#define SENSOR_ATTR_PRIV_START 0x100
#endif

#define ICM42688_ATTR_STEP_COUNT           ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 0))
#define ICM42688_ATTR_STEP_CADENCE         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 1))
#define ICM42688_ATTR_ACTIVITY_CLASS       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 2))

#define ICM42688_ATTR_TAP_NUM              ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 3))
#define ICM42688_ATTR_TAP_AXIS             ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 4))
#define ICM42688_ATTR_TAP_DIR              ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 5))
#define ICM42688_ATTR_DOUBLE_TAP_TIMING    ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 6))

#define ICM42688_ATTR_PEDOMETER_ENABLE     ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 7))
#define ICM42688_ATTR_TILT_ENABLE          ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 8))
#define ICM42688_ATTR_R2W_ENABLE           ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 9))
#define ICM42688_ATTR_TAP_ENABLE           ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 10))

/* LEG (legacy) attribute aliases for compatibility */
#define ICM42688_ATTR_LEG_STEP_COUNT           ICM42688_ATTR_STEP_COUNT
#define ICM42688_ATTR_LEG_STEP_CADENCE         ICM42688_ATTR_STEP_CADENCE
#define ICM42688_ATTR_LEG_ACTIVITY_CLASS       ICM42688_ATTR_ACTIVITY_CLASS
#define ICM42688_ATTR_LEG_TAP_NUM              ICM42688_ATTR_TAP_NUM
#define ICM42688_ATTR_LEG_TAP_AXIS             ICM42688_ATTR_TAP_AXIS
#define ICM42688_ATTR_LEG_TAP_DIR              ICM42688_ATTR_TAP_DIR
#define ICM42688_ATTR_LEG_DOUBLE_TAP_TIMING    ICM42688_ATTR_DOUBLE_TAP_TIMING
#define ICM42688_ATTR_LEG_PEDOMETER_ENABLE     ICM42688_ATTR_PEDOMETER_ENABLE
#define ICM42688_ATTR_LEG_TILT_ENABLE          ICM42688_ATTR_TILT_ENABLE
#define ICM42688_ATTR_LEG_R2W_ENABLE           ICM42688_ATTR_R2W_ENABLE
#define ICM42688_ATTR_LEG_TAP_ENABLE           ICM42688_ATTR_TAP_ENABLE

#define ICM42688_ATTR_SELF_TEST_RUN        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 11))

/* FSYNC configuration attributes */
#define ICM42688_ATTR_FSYNC_CONFIG         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 31))
#define ICM42688_ATTR_FSYNC_UI_SEL         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 32))
#define ICM42688_ATTR_FSYNC_POLARITY       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 33))
#define ICM42688_ATTR_FSYNC_ODR_DELAY      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 34))
#define ICM42688_ATTR_FIFO_FLUSH           ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 12))

/* Wake-on-Motion attributes */
#define ICM42688_ATTR_WOM_ENABLE           ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 13))
#define ICM42688_ATTR_WOM_THRESHOLD        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 14))
#define ICM42688_ATTR_WOM_INT_MODE         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 15))

/* Self-test result attribute: val1 = result bitmask, 0 = all passed */
#define ICM42688_ATTR_SELF_TEST_RESULT     ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 16))

/* UI Filter attributes */
#define ICM42688_ATTR_ACCEL_FILT_BW        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 17))
#define ICM42688_ATTR_GYRO_FILT_BW         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 18))
#define ICM42688_ATTR_ACCEL_FILT_ORD       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 19))
#define ICM42688_ATTR_GYRO_FILT_ORD        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 20))

/* Offset Calibration attributes */
/* CALIB_RUN: val1=1 triggers auto-calibration (sensor must be stationary, Z-up) */
#define ICM42688_ATTR_CALIB_RUN            ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 21))
/* Offset read/write: val1 = offset in mdps (gyro) or mg (accel) */
#define ICM42688_ATTR_GYRO_OFFSET_X        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 22))
#define ICM42688_ATTR_GYRO_OFFSET_Y        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 23))
#define ICM42688_ATTR_GYRO_OFFSET_Z        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 24))
#define ICM42688_ATTR_ACCEL_OFFSET_X       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 25))
#define ICM42688_ATTR_ACCEL_OFFSET_Y       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 26))
#define ICM42688_ATTR_ACCEL_OFFSET_Z       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 27))

/* Timestamp attributes */
#define ICM42688_ATTR_TIMESTAMP_EN         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 28))
#define ICM42688_ATTR_TIMESTAMP_RES        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 29))
#define ICM42688_ATTR_TIMESTAMP_VALUE      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 30))

/* Notch Filter attributes (Bank1) */
#define ICM42688_ATTR_GYRO_NOTCH_ENABLE    ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 31))
#define ICM42688_ATTR_GYRO_NOTCH_FREQ_X    ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 32))
#define ICM42688_ATTR_GYRO_NOTCH_FREQ_Y    ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 33))
#define ICM42688_ATTR_GYRO_NOTCH_FREQ_Z    ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 34))
#define ICM42688_ATTR_GYRO_NOTCH_BW        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 35))

/* Anti-Alias Filter attributes (Bank1/Bank2) */
#define ICM42688_ATTR_GYRO_AAF_ENABLE      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 36))
#define ICM42688_ATTR_GYRO_AAF_FREQ        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 37))
#define ICM42688_ATTR_ACCEL_AAF_ENABLE     ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 38))
#define ICM42688_ATTR_ACCEL_AAF_FREQ       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 39))

/* Temperature DLPF attribute */
#define ICM42688_ATTR_TEMP_DLPF_BW         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 40))

/* DEC2_M2 Filter Order attributes */
#define ICM42688_ATTR_GYRO_DEC2_M2_ORD     ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 41))
#define ICM42688_ATTR_ACCEL_DEC2_M2_ORD    ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 42))

/* Per-axis enable/disable (Bank1) */
#define ICM42688_ATTR_GYRO_X_ENABLE        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 43))
#define ICM42688_ATTR_GYRO_Y_ENABLE        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 44))
#define ICM42688_ATTR_GYRO_Z_ENABLE        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 45))
#define ICM42688_ATTR_ACCEL_X_ENABLE       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 46))
#define ICM42688_ATTR_ACCEL_Y_ENABLE       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 47))
#define ICM42688_ATTR_ACCEL_Z_ENABLE       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 48))

/* FIFO Mode attributes */
#define ICM42688_ATTR_FIFO_MODE            ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 49))

/* Signal Path Reset attributes */
#define ICM42688_ATTR_SIGNAL_PATH_RESET    ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 50))

/* SMD (Significant Motion Detection) attributes */
#define ICM42688_ATTR_SMD_ENABLE           ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 51))
#define ICM42688_ATTR_SMD_THRESHOLD        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 52))
#define ICM42688_ATTR_SMD_MODE             ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 53))

/* INT2 Routing attributes (for APEX events) */
#define ICM42688_ATTR_INT2_ROUTE_TAP       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 54))
#define ICM42688_ATTR_INT2_ROUTE_TILT     ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 55))
#define ICM42688_ATTR_INT2_ROUTE_R2W       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 56))
#define ICM42688_ATTR_INT2_ROUTE_PEDOMETER ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 57))

/* Low Power Mode attributes */
#define ICM42688_ATTR_GYRO_LOW_POWER       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 58))
#define ICM42688_ATTR_ACCEL_LOW_POWER      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 59))
#define ICM42688_ATTR_TEMP_DISABLE         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 60))

/* Temperature Resolution Mode attributes */
#define ICM42688_ATTR_TEMP_RESOLUTION      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 61))

/* Interrupt Routing attributes */
#define ICM42688_ATTR_UI_DRDY_INT1_EN      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 62))
#define ICM42688_ATTR_UI_DRDY_INT2_EN      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 63))

/* Timestamp Mode attributes */
#define ICM42688_ATTR_DELTA_TIMESTAMP      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 64))

/* FIFO Threshold attributes */
#define ICM42688_ATTR_FIFO_THRESHOLD       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 65))

/* Mounting Matrix attributes */
#define ICM42688_ATTR_MOUNTING_MATRIX      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 66))

/* Tap Tuning attributes */
#define ICM42688_ATTR_TAP_THRESHOLD        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 67))
#define ICM42688_ATTR_TAP_WINDOW           ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 68))
#define ICM42688_ATTR_TAP_COUNTS           ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 69))

/* FIFO Full Interrupt attributes */
#define ICM42688_ATTR_FIFO_FULL_INT_EN     ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 70))

/* Advanced Pedometer attributes */
#define ICM42688_ATTR_STRIDE_LENGTH        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 71))
#define ICM42688_ATTR_STEP_COUNT_ENABLE    ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 72))

/* External Clock (CLKIN/RTC) attributes */
#define ICM42688_ATTR_RTC_MODE             ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 73))
#define ICM42688_ATTR_CLKSEL               ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 74))

/* Timestamp resolution values */
#define ICM42688_TIMESTAMP_RES_1US         0u
#define ICM42688_TIMESTAMP_RES_16US        1u

/* Delta Timestamp values */
#define ICM42688_DELTA_TIMESTAMP_DISABLED  0u  /* Absolute timestamps (full counter value) */
#define ICM42688_DELTA_TIMESTAMP_ENABLED   1u  /* Delta timestamps (time since last sample) */

/* FIFO Threshold values */
#define ICM42688_FIFO_THRESHOLD_MIN        0u    /* 0 bytes (no threshold, interrupt on any data) */
#define ICM42688_FIFO_THRESHOLD_MAX        255u  /* 255 bytes (max FIFO watermark level) */

/* Mounting Matrix defines (3x3 = 9 elements max per sensor_value array) */
#define ICM42688_MOUNTING_MATRIX_SIZE      9    /* 3x3 matrix elements */

/* Tap Tuning values (sensitivity and timing configuration) */
#define ICM42688_TAP_THRESHOLD_MIN         0u    /* Minimum tap sensitivity (most sensitive) */
#define ICM42688_TAP_THRESHOLD_MAX         15u   /* Maximum tap sensitivity (least sensitive), 1LSB = 31mg */
#define ICM42688_TAP_WINDOW_MIN            0u    /* Minimum time window (fastest detection) */
#define ICM42688_TAP_WINDOW_MAX            15u   /* Maximum time window (longest detection), ~10ms per unit */
#define ICM42688_TAP_COUNTS_MIN            1u    /* Single tap detection */
#define ICM42688_TAP_COUNTS_MAX            15u   /* Maximum peaks per tap event */

/* FIFO Full Interrupt values */
#define ICM42688_FIFO_FULL_INT_DISABLED    0u  /* FIFO full interrupt disabled */
#define ICM42688_FIFO_FULL_INT_ENABLED     1u  /* FIFO full interrupt enabled (fires when FIFO fills) */

/* Pedometer Stride Length values */
#define ICM42688_STRIDE_LENGTH_MIN         0u    /* Minimum stride (shortest step) */
#define ICM42688_STRIDE_LENGTH_MAX         255u  /* Maximum stride (longest step), each unit ~0.78m */

/* RTC Mode values (INTF_CONFIG1[2]) */
#define ICM42688_RTC_MODE_DISABLED         0u  /* RTC/CLKIN input disabled (default) */
#define ICM42688_RTC_MODE_ENABLED          1u  /* RTC/CLKIN input enabled (external clock) */

/* CLKSEL values (INTF_CONFIG1[1:0]) - Clock source selection */
#define ICM42688_CLKSEL_INTERNAL           0u  /* Internal RC oscillator (default) */
#define ICM42688_CLKSEL_AUTO               1u  /* Auto-select: PLL when available, RC otherwise */
#define ICM42688_CLKSEL_DISABLED           2u  /* Clock source disabled (low power) */
#define ICM42688_CLKSEL_EXTERNAL           3u  /* External clock via CLKIN pin (RTC mode) */

/* External Clock (CLKIN/RTC) values */
#define ICM42688_RTC_MODE_DISABLED         0u  /* No input RTC clock required (internal RC oscillator) */
#define ICM42688_RTC_MODE_ENABLED          1u  /* RTC clock input required (31-50kHz external clock) */

/* Clock Selection (CLKSEL) values */
#define ICM42688_CLKSEL_INT_RC             0u  /* Always select internal RC oscillator */
#define ICM42688_CLKSEL_AUTO_PLL_RC        1u  /* Select PLL when available, else RC (default) */
#define ICM42688_CLKSEL_DISABLE_ALL        3u  /* Disable all clocks (low power) */

/* FIFO Mode values */
#define ICM42688_FIFO_MODE_BYPASS          0u  /* FIFO disabled, data goes directly to registers */
#define ICM42688_FIFO_MODE_STREAM          1u  /* Stream mode, new data overwrites old (default) */
#define ICM42688_FIFO_MODE_STOP_ON_FULL    2u  /* Stop-on-full, FIFO stops when full */

/* Signal Path Reset values (bit mask, can be ORed together) */
#define ICM42688_SPR_ACCEL_ONLY            0u  /* Reset accel signal path only */
#define ICM42688_SPR_GYRO_ONLY             1u  /* Reset gyro signal path only */
#define ICM42688_SPR_TEMP_ONLY             2u  /* Reset temp signal path only */
#define ICM42688_SPR_ALL                   3u  /* Reset all signal paths */

/* SMD (Significant Motion Detection) modes */
#define ICM42688_SMD_MODE_DISABLED         0u  /* SMD disabled */
#define ICM42688_SMD_MODE_SHORT            2u  /* 2-second window */
#define ICM42688_SMD_MODE_LONG             3u  /* 10-second window */

/* Activity classification values (for ACTIVITY_CLASS attribute) */
#define ICM42688_ACTIVITY_UNKNOWN          0u
#define ICM42688_ACTIVITY_WALK             1u
#define ICM42688_ACTIVITY_RUN              2u

/* Temperature DLPF Bandwidth values (GYRO_CONFIG1 bits [7:5]) */
#define ICM42688_TEMP_DLPF_BW_4000HZ       0u  /* 4000 Hz, 0.125ms latency */
#define ICM42688_TEMP_DLPF_BW_170HZ        1u  /* 170 Hz, 1ms latency */
#define ICM42688_TEMP_DLPF_BW_82HZ         2u  /* 82 Hz, 2ms latency */
#define ICM42688_TEMP_DLPF_BW_40HZ         3u  /* 40 Hz, 4ms latency */
#define ICM42688_TEMP_DLPF_BW_20HZ         4u  /* 20 Hz, 8ms latency */
#define ICM42688_TEMP_DLPF_BW_10HZ         5u  /* 10 Hz, 16ms latency */
#define ICM42688_TEMP_DLPF_BW_5HZ          6u  /* 5 Hz, 32ms latency */

/* Temperature Resolution Mode values (for FIFO temperature data format) */
#define ICM42688_TEMP_RES_8BIT             0u  /* 8-bit FIFO_TEMP_DATA: (value/2.07)+25°C, ~480Hz, low power */
#define ICM42688_TEMP_RES_16BIT            1u  /* 16-bit temperature: more accurate, normal power consumption */

/* DEC2_M2 Filter Order values */
#define ICM42688_DEC2_M2_ORD_3RD           2u  /* 3rd order */

/* Notch Filter Bandwidth values (8 options: 1449 - 10 Hz) */
#define ICM42688_NF_BW_1449HZ              0u
#define ICM42688_NF_BW_724HZ               1u
#define ICM42688_NF_BW_483HZ               2u
#define ICM42688_NF_BW_362HZ               3u
#define ICM42688_NF_BW_290HZ               4u
#define ICM42688_NF_BW_241HZ               5u
#define ICM42688_NF_BW_207HZ               6u
#define ICM42688_NF_BW_10HZ                7u

/* ======== APEX feature mask ======== */
#define ICM42688_APEX_FEAT_PEDOMETER       (1u << 0)
#define ICM42688_APEX_FEAT_TILT            (1u << 1)
#define ICM42688_APEX_FEAT_R2W             (1u << 2)
#define ICM42688_APEX_FEAT_TAP             (1u << 3)

/* WOM mode defines */
#define ICM42688_WOM_INT_MODE_OR           0u /* interrupt if ANY axis exceeds threshold */
#define ICM42688_WOM_INT_MODE_AND          1u /* interrupt if ALL axes exceed threshold */

/* Self-test result bits */
#define ICM42688_ST_ACCEL_X_FAIL           (1u << 0)
#define ICM42688_ST_ACCEL_Y_FAIL           (1u << 1)
#define ICM42688_ST_ACCEL_Z_FAIL           (1u << 2)
#define ICM42688_ST_GYRO_X_FAIL            (1u << 3)
#define ICM42688_ST_GYRO_Y_FAIL            (1u << 4)
#define ICM42688_ST_GYRO_Z_FAIL            (1u << 5)

/* UI Filter Bandwidth values */
#define ICM42688_FILT_BW_ODR_DIV_2         0u
#define ICM42688_FILT_BW_ODR_DIV_4         1u
#define ICM42688_FILT_BW_ODR_DIV_5         2u
#define ICM42688_FILT_BW_ODR_DIV_8         3u
#define ICM42688_FILT_BW_ODR_DIV_10        4u
#define ICM42688_FILT_BW_ODR_DIV_16        5u
#define ICM42688_FILT_BW_ODR_DIV_20        6u
#define ICM42688_FILT_BW_ODR_DIV_40        7u

/* UI Filter Order values */
#define ICM42688_FILT_ORD_1ST              0u
#define ICM42688_FILT_ORD_2ND              1u
#define ICM42688_FILT_ORD_3RD              2u



/* ====================================================================
 * FIFO (public packet struct + APIs)
 * ====================================================================
 */

struct icm42688_fifo_data {
	bool valid;

	uint8_t header;

	bool has_accel;
	bool has_gyro;
	bool has_timestamp;
	bool is_20bit;

	int32_t accel_x;
	int32_t accel_y;
	int32_t accel_z;

	int32_t gyro_x;
	int32_t gyro_y;
	int32_t gyro_z;

	int8_t  temp_raw8;
	int16_t temp_raw16;

	uint16_t timestamp16;
};

#ifdef CONFIG_ICM42688_FIFO

/* FIFO control */
int icm42688_fifo_init(const struct device *dev);
int icm42688_fifo_flush(const struct device *dev);

/* Raw FIFO read (bytes) */
int icm42688_fifo_read_all(const struct device *dev, uint8_t *buf, size_t cap, size_t *out_len);

/* Parsing */
int icm42688_fifo_parse_packet(const uint8_t *buf, size_t len, struct icm42688_fifo_data *out);

/* Convenience: read once (<=256B) and parse packets */
int icm42688_fifo_read_packets(const struct device *dev,
			       struct icm42688_fifo_data *packets,
			       size_t max_packets,
			       size_t *num_packets);

/* Robust drain helper (tail-aware) */
int icm42688_fifo_drain_and_parse(const struct device *dev,
				 struct icm42688_fifo_data *out_pkts,
				 size_t max_pkts,
				 size_t *out_n,
				 size_t max_bytes);

/* Timestamp in FIFO / TMSTVAL regs */
int icm42688_fifo_timestamp_enable(const struct device *dev, bool enable);
int icm42688_read_timestamp(const struct device *dev, uint32_t *timestamp);
int icm42688_timestamp_set_resolution(const struct device *dev, uint8_t res);

/* Debug helper */
int icm42688_fifo_get_latest_timestamp(const struct device *dev, uint16_t *timestamp);

/* Queue API - pushed to irq/work, popped in app (always declared) */
int icm42688_fifo_queue_reset(const struct device *dev);

#endif /* CONFIG_ICM42688_FIFO */

/* Queue API - declarations always available (enabled when CONFIG_ICM42688_FIFO is set) */
void icm42688_fifo_msgq_init_once(const struct device *dev);
int icm42688_fifo_drain_to_queue(const struct device *dev);
int icm42688_fifo_pop(const struct device *dev, struct icm42688_fifo_data *out, k_timeout_t timeout);


/* ====================================================================
 * Driver config (const)
 * ====================================================================
 */
struct icm42688_config {
	enum {
		ICM42688_BUS_I2C,
		ICM42688_BUS_SPI,
		ICM42688_BUS_I3C
	} bus_type;

	union {
		struct {
			struct i2c_dt_spec bus;
		} i2c;
		struct {
			struct spi_dt_spec bus;
		} spi;
#ifdef CONFIG_I3C
		struct {
			struct i3c_device_desc *bus;
		} i3c;
#endif
	};

#ifdef CONFIG_ICM42688_TRIGGER
	struct gpio_dt_spec int_gpio;
	struct gpio_dt_spec int2_gpio;  /* Optional second interrupt pin */
#endif

	/* Defaults for accel/gyro */
	uint16_t accel_fs_g;
	uint16_t gyro_fs_dps;
	uint32_t accel_odr_hz;
	uint32_t gyro_odr_hz;

#ifdef CONFIG_ICM42688_FIFO
	bool fifo_enable;
	bool fifo_hires;
	uint16_t fifo_watermark;
#endif

#ifdef CONFIG_ICM42688_APEX
	bool apex_enable;
	uint8_t apex_features;
	bool apex_dmp_power_save;
	uint8_t apex_dmp_odr_sel;
	uint8_t apex_sensitivity_mode;
	uint8_t apex_tilt_wait_time_sel;
	uint8_t apex_sleep_timeout_sel;
	uint8_t apex_mounting_matrix;
	uint8_t apex_sleep_gesture_delay;
	bool apex_route_int2;
#endif
};

/* ====================================================================
 * Driver runtime data (mutable)
 * ====================================================================
 */
struct icm42688_data {
	/* cached samples in SI units for Zephyr sensor API
	 * accel_*: micro m/s^2
	 * gyro_* : micro rad/s
	 * temp_mc: milli degC
	 */
	int32_t accel_x;
	int32_t accel_y;
	int32_t accel_z;

	int32_t gyro_x;
	int32_t gyro_y;
	int32_t gyro_z;

	int32_t temp_mc;

	/* Runtime configuration cached */
	uint16_t accel_fs_g;
	uint16_t gyro_fs_dps;
	uint32_t accel_odr_hz;
	uint32_t gyro_odr_hz;

#ifdef CONFIG_ICM42688_TRIGGER
	const struct device *dev;

	struct gpio_callback int_cb;
	struct gpio_callback int2_cb;
	struct k_work work;

	/* Data-ready trigger */
	struct sensor_trigger drdy_trig;
	sensor_trigger_handler_t drdy_handler;

	/* WOM trigger */
	struct sensor_trigger wom_trig;
	sensor_trigger_handler_t wom_handler;

#ifdef CONFIG_ICM42688_FIFO
	struct sensor_trigger fifo_wm_trig;
	sensor_trigger_handler_t fifo_wm_handler;

	struct sensor_trigger fifo_full_trig;
	sensor_trigger_handler_t fifo_full_handler;
#endif
#endif /* CONFIG_ICM42688_TRIGGER */

	/* WOM (Wake-on-Motion) */
	bool wom_enabled;
	uint8_t wom_threshold;
	uint8_t wom_int_mode;

#ifdef CONFIG_ICM42688_APEX
	/* APEX motion detection */
	bool apex_enabled;
	uint8_t apex_features_enabled;

	/* SMD (Significant Motion Detection) */
	bool smd_enabled;
	uint8_t smd_threshold;
	uint8_t smd_mode;

	/* INT2 routing for APEX events */
	bool int2_route_tap;
	bool int2_route_tilt;
	bool int2_route_r2w;
	bool int2_route_pedometer;

	/* Tap Detection */
	uint8_t tap_threshold;
	uint8_t tap_window;
	uint8_t tap_counts;
	uint8_t last_tap_num;
	uint8_t last_tap_axis;
	uint8_t last_tap_dir;
	uint8_t last_double_tap_timing;

	/* Pedometer */
	uint8_t stride_length;
	bool step_count_enabled;

	/* Tilt detection */
	bool tilt_enabled;
#endif /* CONFIG_ICM42688_APEX */

	/* Low Power Modes */
	bool gyro_low_power;
	bool accel_low_power;
	bool temp_disabled;
	uint8_t temp_resolution;

	/* Filters */
	bool notch_filter_enabled;
	bool aaf_filter_enabled;

	/* Interrupt routing */
	bool ui_drdy_int1_enabled;
	bool ui_drdy_int2_enabled;
	bool fifo_full_int_enabled;

	/* Offset Calibration */
	int16_t accel_offset_x;
	int16_t accel_offset_y;
	int16_t accel_offset_z;
	int16_t gyro_offset_x;
	int16_t gyro_offset_y;
	int16_t gyro_offset_z;

	/* Self-test result */
	uint8_t self_test_result;

	/* Timestamp */
	bool delta_timestamp_enabled;
	bool tmst_delta_en;
	uint32_t fifo_tmst_accum;
	uint16_t fifo_tmst_prev16;
	bool fifo_tmst_have_prev;

	/* FIFO configuration */
	uint8_t fifo_threshold;
	int16_t mounting_matrix[ICM42688_MOUNTING_MATRIX_SIZE];
	uint8_t clksel;
	bool rtc_mode_enabled;

#ifdef CONFIG_ICM42688_FIFO
	/* FIFO state */
	uint8_t fifo_tail[32];
	uint8_t fifo_tail_len;
	bool fifo_overflowed;

	/* Message queue for FIFO packets */
	bool fifo_msgq_inited;
	struct k_msgq *fifo_msgq;
#endif

	/* Thread safety */
	struct k_mutex lock;
};

/* ====================================================================
 * Low-level access (implemented in icm42688.c)
 * ==================================================================== */
int icm42688_read_reg(const struct device *dev, uint8_t reg, uint8_t *val);
int icm42688_write_reg(const struct device *dev, uint8_t reg, uint8_t val);
int icm42688_read_burst(const struct device *dev, uint8_t reg, uint8_t *buf, size_t len);
int icm42688_reg_bank_sel(const struct device *dev, uint8_t bank);

/* ====================================================================
 * Notch Filter functions (icm42688_notch.c)
 * ==================================================================== */
int icm42688_notch_enable(const struct device *dev, bool enable);
int icm42688_notch_set_freq(const struct device *dev, uint8_t axis, uint8_t coswz_value);
int icm42688_notch_get_freq(const struct device *dev, uint8_t axis, uint8_t *coswz_value);
int icm42688_notch_set_bandwidth(const struct device *dev, uint8_t bw_sel);
int icm42688_notch_get_bandwidth(const struct device *dev, uint8_t *bw_sel);

/* ====================================================================
 * Anti-Alias Filter functions (icm42688_aaf.c)
 * ==================================================================== */
int icm42688_gyro_aaf_enable(const struct device *dev, bool enable);
int icm42688_gyro_aaf_set_freq(const struct device *dev, uint16_t freq_hz);
int icm42688_gyro_aaf_get_freq(const struct device *dev, uint16_t *freq_hz);
int icm42688_accel_aaf_enable(const struct device *dev, bool enable);
int icm42688_accel_aaf_set_freq(const struct device *dev, uint16_t freq_hz);
int icm42688_accel_aaf_get_freq(const struct device *dev, uint16_t *freq_hz);

/* Small shared helper used across modules */
static inline int icm42688_reg_update8(const struct device *dev,
				       uint8_t reg,
				       uint8_t mask,
				       uint8_t value)
{
	uint8_t v = 0;
	int rc = icm42688_read_reg(dev, reg, &v);
	if (rc < 0) {
		return rc;
	}
	v = (uint8_t)((v & ~mask) | (value & mask));
	return icm42688_write_reg(dev, reg, v);
}

/* Function to check if an attribute matches one of two values */
static inline bool attr_is(int a, int attr1, int attr2)
{
    return (a == attr1) || (a == attr2);
}

#ifdef CONFIG_ICM42688_APEX
int icm42688_apex_init(const struct device *dev, uint8_t feature_mask);
int icm42688_apex_enable_feature(const struct device *dev, uint8_t feature_mask, bool enable);
int icm42688_apex_route_to_int2(const struct device *dev, uint8_t feature_mask);

int icm42688_apex_read_step_count_u16(const struct device *dev, uint16_t *steps);
int icm42688_apex_read_step_cadence_u8(const struct device *dev, uint8_t *cad);
int icm42688_apex_read_activity_class_u8(const struct device *dev, uint8_t *cls);

int icm42688_apex_read_tap(const struct device *dev,
			   uint8_t *tap_num,
			   uint8_t *tap_axis,
			   uint8_t *tap_dir,
			   uint8_t *double_tap_timing);
#endif

#ifdef CONFIG_ICM42688_TRIGGER
int icm42688_trigger_init(const struct device *dev);
int icm42688_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler);
#endif

/* WOM functions */
int icm42688_wom_enable(const struct device *dev, uint8_t threshold_mg_div4, uint8_t int_mode);
int icm42688_wom_disable(const struct device *dev);

/* FSYNC functions */
int icm42688_fsync_enable(const struct device *dev, uint8_t ui_sel, bool polarity);
int icm42688_fsync_disable(const struct device *dev);
int icm42688_fsync_set_delay(const struct device *dev, uint8_t delay);

/* Self-test */
int icm42688_self_test(const struct device *dev, uint8_t *result);

/* Signal Path Reset */
int icm42688_signal_path_reset(const struct device *dev, uint8_t reset_mask);

/* SMD (Significant Motion Detection) */
int icm42688_smd_enable(const struct device *dev, uint8_t threshold_mg_div4, uint8_t mode);
int icm42688_smd_disable(const struct device *dev);

/* Low Power Mode */
int icm42688_set_gyro_low_power(const struct device *dev, bool enable);
int icm42688_set_accel_low_power(const struct device *dev, bool enable);
int icm42688_set_temp_disable(const struct device *dev, bool disable);
int icm42688_set_temp_resolution(const struct device *dev, uint8_t mode);

/* Interrupt Routing */
int icm42688_set_ui_drdy_int1(const struct device *dev, bool enable);
int icm42688_set_ui_drdy_int2(const struct device *dev, bool enable);

/* Timestamp Mode */
int icm42688_set_delta_timestamp(const struct device *dev, bool enable);

/* FIFO Configuration */
int icm42688_set_fifo_threshold(const struct device *dev, uint8_t threshold);

/* Mounting Matrix */
int icm42688_set_mounting_matrix(const struct device *dev, const int16_t *matrix, size_t size);

/* Tap Detection Tuning */
int icm42688_set_tap_threshold(const struct device *dev, uint8_t threshold);
int icm42688_set_tap_window(const struct device *dev, uint8_t window);
int icm42688_set_tap_counts(const struct device *dev, uint8_t counts);

/* FIFO Full Interrupt */
int icm42688_set_fifo_full_interrupt(const struct device *dev, bool enable);

/* Advanced Pedometer */
int icm42688_set_stride_length(const struct device *dev, uint8_t stride);
int icm42688_set_step_count_enable(const struct device *dev, bool enable);

/* External Clock (CLKIN/RTC) */
int icm42688_set_rtc_mode(const struct device *dev, bool enable);
int icm42688_set_clksel(const struct device *dev, uint8_t clksel);

/* Offset calibration */
int icm42688_calibrate(const struct device *dev);
int icm42688_set_gyro_offset(const struct device *dev, int axis, int32_t offset_mdps);
int icm42688_get_gyro_offset(const struct device *dev, int axis, int32_t *offset_mdps);
int icm42688_set_accel_offset(const struct device *dev, int axis, int32_t offset_mg);
int icm42688_get_accel_offset(const struct device *dev, int axis, int32_t *offset_mg);

#ifdef __cplusplus
}
#endif
