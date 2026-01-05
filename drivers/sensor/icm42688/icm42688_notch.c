/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "icm42688.h"
#include "icm42688_reg.h"

LOG_MODULE_REGISTER(ICM42688_NOTCH, CONFIG_SENSOR_LOG_LEVEL);

/**
 * @brief Enable or disable Gyroscope Notch Filter
 *
 * Notch Filter is used to suppress mechanical resonances (1-3 kHz range).
 * IMPORTANT: Notch filter only works in Low Noise (LN) mode (datasheet section 12.4).
 *
 * @param dev ICM42688 device
 * @param enable true to enable, false to disable
 * @return 0 on success, negative errno on failure
 */
int icm42688_notch_enable(const struct device *dev, bool enable)
{
	int rc;
	uint8_t mask = enable ? 0 : ICM42688_GYRO_NF_DIS;

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK1);
	if (rc != 0) {
		return rc;
	}

	rc = icm42688_reg_update8(dev, ICM42688_REG_GYRO_CONFIG_STATIC2,
				  ICM42688_GYRO_NF_DIS, mask);

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}

/**
 * @brief Set Notch Filter center frequency for specific axis
 *
 * The COSWZ value is factory-trimmed for each device and axis.
 * Typical resonance range: 1-3 kHz.
 *
 * @param dev ICM42688 device
 * @param axis Axis index: 0=X, 1=Y, 2=Z
 * @param coswz_value 8-bit COSWZ value (factory-trimmed, read from device)
 * @return 0 on success, negative errno on failure
 */
int icm42688_notch_set_freq(const struct device *dev, uint8_t axis, uint8_t coswz_value)
{
	int rc;
	uint8_t reg;

	if (axis > 2) {
		return -EINVAL;
	}

	/* Select register based on axis */
	switch (axis) {
	case 0: /* X-axis */
		reg = ICM42688_REG_GYRO_CONFIG_STATIC6;
		break;
	case 1: /* Y-axis */
		reg = ICM42688_REG_GYRO_CONFIG_STATIC7;
		break;
	case 2: /* Z-axis */
		reg = ICM42688_REG_GYRO_CONFIG_STATIC8;
		break;
	default:
		return -EINVAL;
	}

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK1);
	if (rc != 0) {
		return rc;
	}

	rc = icm42688_write_reg(dev, reg, coswz_value);

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}

/**
 * @brief Read current Notch Filter center frequency for specific axis
 *
 * @param dev ICM42688 device
 * @param axis Axis index: 0=X, 1=Y, 2=Z
 * @param coswz_value Pointer to store 8-bit COSWZ value
 * @return 0 on success, negative errno on failure
 */
int icm42688_notch_get_freq(const struct device *dev, uint8_t axis, uint8_t *coswz_value)
{
	int rc;
	uint8_t reg;

	if (axis > 2 || coswz_value == NULL) {
		return -EINVAL;
	}

	switch (axis) {
	case 0:
		reg = ICM42688_REG_GYRO_CONFIG_STATIC6;
		break;
	case 1:
		reg = ICM42688_REG_GYRO_CONFIG_STATIC7;
		break;
	case 2:
		reg = ICM42688_REG_GYRO_CONFIG_STATIC8;
		break;
	default:
		return -EINVAL;
	}

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK1);
	if (rc != 0) {
		return rc;
	}

	rc = icm42688_read_reg(dev, reg, coswz_value);

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}

/**
 * @brief Set Notch Filter bandwidth
 *
 * Bandwidth options (8 values):
 * 0: 1449 Hz
 * 1: 724 Hz
 * 2: 483 Hz
 * 3: 362 Hz
 * 4: 290 Hz
 * 5: 241 Hz
 * 6: 207 Hz
 * 7: 10 Hz
 *
 * @param dev ICM42688 device
 * @param bw_sel Bandwidth selector (0-7)
 * @return 0 on success, negative errno on failure
 */
int icm42688_notch_set_bandwidth(const struct device *dev, uint8_t bw_sel)
{
	int rc;

	if (bw_sel > 7) {
		return -EINVAL;
	}

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK1);
	if (rc != 0) {
		return rc;
	}

	rc = icm42688_reg_update8(dev, ICM42688_REG_GYRO_CONFIG_STATIC9,
				  ICM42688_GYRO_NF_BW_SEL_MASK,
				  (uint8_t)(bw_sel << ICM42688_GYRO_NF_BW_SEL_SHIFT));

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}

/**
 * @brief Get current Notch Filter bandwidth setting
 *
 * @param dev ICM42688 device
 * @param bw_sel Pointer to store bandwidth selector (0-7)
 * @return 0 on success, negative errno on failure
 */
int icm42688_notch_get_bandwidth(const struct device *dev, uint8_t *bw_sel)
{
	int rc;
	uint8_t val;

	if (bw_sel == NULL) {
		return -EINVAL;
	}

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK1);
	if (rc != 0) {
		return rc;
	}

	rc = icm42688_read_reg(dev, ICM42688_REG_GYRO_CONFIG_STATIC9, &val);
	if (rc == 0) {
		*bw_sel = (val & ICM42688_GYRO_NF_BW_SEL_MASK) >> ICM42688_GYRO_NF_BW_SEL_SHIFT;
	}

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}
