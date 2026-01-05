#pragma once
#include <zephyr/drivers/sensor.h>

/* Keep in sync with drivers/sensor/icm42688/icm42688.h */
#ifndef SENSOR_ATTR_PRIV_START
#define SENSOR_ATTR_PRIV_START 0x100
#endif

#define ICM42688_ATTR_STEP_COUNT         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 0))
#define ICM42688_ATTR_STEP_CADENCE       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 1))
#define ICM42688_ATTR_ACTIVITY_CLASS     ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 2))

#define ICM42688_ATTR_TAP_NUM            ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 3))
#define ICM42688_ATTR_TAP_AXIS           ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 4))
#define ICM42688_ATTR_TAP_DIR            ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 5))
#define ICM42688_ATTR_DOUBLE_TAP_TIMING  ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 6))

#define ICM42688_ATTR_PEDOMETER_ENABLE   ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 7))
#define ICM42688_ATTR_TILT_ENABLE        ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 8))
#define ICM42688_ATTR_R2W_ENABLE         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 9))
#define ICM42688_ATTR_TAP_ENABLE         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 10))

/* Optional */
#define ICM42688_ATTR_SELF_TEST_RUN      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 11))
#define ICM42688_ATTR_FIFO_FLUSH         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 12))

/* Wake-on-Motion attributes */
#define ICM42688_ATTR_WOM_ENABLE         ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 13))
#define ICM42688_ATTR_WOM_THRESHOLD      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 14))
#define ICM42688_ATTR_WOM_INT_MODE       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 15))

/* Self-test result attribute: val1 = result bitmask, 0 = all passed */
#define ICM42688_ATTR_SELF_TEST_RESULT   ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 16))

/* UI Filter attributes */
#define ICM42688_ATTR_ACCEL_FILT_BW      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 17))
#define ICM42688_ATTR_GYRO_FILT_BW       ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 18))
#define ICM42688_ATTR_ACCEL_FILT_ORD     ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 19))
#define ICM42688_ATTR_GYRO_FILT_ORD      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 20))

/* Offset Calibration attributes */
#define ICM42688_ATTR_CALIB_RUN          ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 21))
#define ICM42688_ATTR_GYRO_OFFSET_X      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 22))
#define ICM42688_ATTR_GYRO_OFFSET_Y      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 23))
#define ICM42688_ATTR_GYRO_OFFSET_Z      ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 24))
#define ICM42688_ATTR_ACCEL_OFFSET_X     ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 25))
#define ICM42688_ATTR_ACCEL_OFFSET_Y     ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 26))
#define ICM42688_ATTR_ACCEL_OFFSET_Z     ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 27))

/* WOM mode defines */
#define ICM42688_WOM_INT_MODE_OR      0u  /* Interrupt if ANY axis exceeds threshold */
#define ICM42688_WOM_INT_MODE_AND     1u  /* Interrupt if ALL axes exceed threshold */

/* Self-test result bits (returned in ICM42688_ATTR_SELF_TEST_RESULT) */
#define ICM42688_ST_ACCEL_X_FAIL      (1u << 0)
#define ICM42688_ST_ACCEL_Y_FAIL      (1u << 1)
#define ICM42688_ST_ACCEL_Z_FAIL      (1u << 2)
#define ICM42688_ST_GYRO_X_FAIL       (1u << 3)
#define ICM42688_ST_GYRO_Y_FAIL       (1u << 4)
#define ICM42688_ST_GYRO_Z_FAIL       (1u << 5)

/* UI Filter Bandwidth values (for ACCEL_FILT_BW / GYRO_FILT_BW) */
#define ICM42688_FILT_BW_ODR_DIV_2    0   /* ODR/2 (Nyquist) - max bandwidth */
#define ICM42688_FILT_BW_ODR_DIV_4    1   /* ODR/4 */
#define ICM42688_FILT_BW_ODR_DIV_5    2   /* ODR/5 */
#define ICM42688_FILT_BW_ODR_DIV_8    3   /* ODR/8 */
#define ICM42688_FILT_BW_ODR_DIV_10   4   /* ODR/10 */
#define ICM42688_FILT_BW_ODR_DIV_16   5   /* ODR/16 */
#define ICM42688_FILT_BW_ODR_DIV_20   6   /* ODR/20 */
#define ICM42688_FILT_BW_ODR_DIV_40   7   /* ODR/40 - smoothest */

/* UI Filter Order values (for ACCEL_FILT_ORD / GYRO_FILT_ORD) */
#define ICM42688_FILT_ORD_1ST         0   /* 1st order - fastest response */
#define ICM42688_FILT_ORD_2ND         1   /* 2nd order */
#define ICM42688_FILT_ORD_3RD         2   /* 3rd order - smoothest */

/* Timestamp attributes */
#define ICM42688_ATTR_TIMESTAMP_EN    ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 28))
#define ICM42688_ATTR_TIMESTAMP_RES   ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 29))
#define ICM42688_ATTR_TIMESTAMP_VALUE ((enum sensor_attribute)(SENSOR_ATTR_PRIV_START + 30))

/* Timestamp resolution values */
#define ICM42688_TIMESTAMP_RES_1US    0   /* 1 microsecond resolution */
#define ICM42688_TIMESTAMP_RES_16US   1   /* 16 microsecond resolution */

/* Legacy aliases kept for compatibility with older application code */
#define ICM42688_ATTR_LEG_STEP_COUNT         ICM42688_ATTR_STEP_COUNT
#define ICM42688_ATTR_LEG_STEP_CADENCE       ICM42688_ATTR_STEP_CADENCE
#define ICM42688_ATTR_LEG_ACTIVITY_CLASS     ICM42688_ATTR_ACTIVITY_CLASS
#define ICM42688_ATTR_LEG_TAP_NUM            ICM42688_ATTR_TAP_NUM
#define ICM42688_ATTR_LEG_TAP_AXIS           ICM42688_ATTR_TAP_AXIS
#define ICM42688_ATTR_LEG_TAP_DIR            ICM42688_ATTR_TAP_DIR
#define ICM42688_ATTR_LEG_DOUBLE_TAP_TIMING  ICM42688_ATTR_DOUBLE_TAP_TIMING
#define ICM42688_ATTR_LEG_PEDOMETER_ENABLE   ICM42688_ATTR_PEDOMETER_ENABLE
#define ICM42688_ATTR_LEG_TILT_ENABLE        ICM42688_ATTR_TILT_ENABLE
#define ICM42688_ATTR_LEG_R2W_ENABLE         ICM42688_ATTR_R2W_ENABLE
#define ICM42688_ATTR_LEG_TAP_ENABLE         ICM42688_ATTR_TAP_ENABLE

#define ICM42688_ACTIVITY_UNKNOWN 0u
#define ICM42688_ACTIVITY_WALK    1u
#define ICM42688_ACTIVITY_RUN     2u

/* ============================================================================
 * FIFO (public packet struct + APIs)
 * ============================================================================
 */
#ifdef CONFIG_ICM42688_FIFO

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

/* ===================== Queue API =====================
 * push from IRQ/work, pop in app
 */
void icm42688_fifo_msgq_init_once(const struct device *dev);

int icm42688_fifo_drain_to_queue(const struct device *dev);

int icm42688_fifo_pop(const struct device *dev,
		      struct icm42688_fifo_data *out,
		      k_timeout_t timeout);

int icm42688_fifo_queue_reset(const struct device *dev);

#endif /* CONFIG_ICM42688_FIFO */
