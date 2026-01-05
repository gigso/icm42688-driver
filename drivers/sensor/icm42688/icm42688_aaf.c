/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "icm42688.h"
#include "icm42688_reg.h"
#include "icm42688_aaf_table.h"

LOG_MODULE_REGISTER(ICM42688_AAF, CONFIG_SENSOR_LOG_LEVEL);

/**
 * @brief Find AAF table entry closest to target frequency
 *
 * @param freq_hz Target frequency in Hz (42-3979 Hz)
 * @param entry Pointer to store found entry
 * @return 0 on success, -EINVAL if out of range
 */
static int find_aaf_entry(uint16_t freq_hz, const struct icm42688_aaf_entry **entry)
{
	if (freq_hz < 42 || freq_hz > 3979) {
		return -EINVAL;
	}

	/* Linear search for closest frequency */
	int closest_idx = 0;
	uint16_t min_diff = UINT16_MAX;

	for (int i = 0; i < ICM42688_AAF_TABLE_SIZE; i++) {
		uint16_t diff = (freq_hz > icm42688_aaf_table[i].freq_hz) ?
				(freq_hz - icm42688_aaf_table[i].freq_hz) :
				(icm42688_aaf_table[i].freq_hz - freq_hz);
		if (diff < min_diff) {
			min_diff = diff;
			closest_idx = i;
		}
	}

	*entry = &icm42688_aaf_table[closest_idx];
	return 0;
}

/**
 * @brief Enable or disable Gyroscope Anti-Alias Filter
 *
 * AAF range: 42 Hz to 3979 Hz.
 * IMPORTANT: AAF only works in Low Noise (LN) mode (datasheet section 12.4).
 *
 * @param dev ICM42688 device
 * @param enable true to enable, false to disable
 * @return 0 on success, negative errno on failure
 */
int icm42688_gyro_aaf_enable(const struct device *dev, bool enable)
{
	int rc;
	uint8_t mask = enable ? 0 : ICM42688_GYRO_AAF_DIS;

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK1);
	if (rc != 0) {
		return rc;
	}

	rc = icm42688_reg_update8(dev, ICM42688_REG_GYRO_CONFIG_STATIC2,
				  ICM42688_GYRO_AAF_DIS, mask);

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}

/**
 * @brief Set Gyroscope AAF bandwidth
 *
 * Programs DELT, DELTSQR, BITSHIFT registers based on lookup table.
 *
 * @param dev ICM42688 device
 * @param freq_hz Target 3dB bandwidth in Hz (42-3979 Hz)
 * @return 0 on success, negative errno on failure
 */
int icm42688_gyro_aaf_set_freq(const struct device *dev, uint16_t freq_hz)
{
	int rc;
	const struct icm42688_aaf_entry *entry;

	rc = find_aaf_entry(freq_hz, &entry);
	if (rc != 0) {
		LOG_ERR("AAF frequency %u Hz out of range (42-3979 Hz)", freq_hz);
		return rc;
	}

	LOG_DBG("Gyro AAF: %u Hz -> DELT=%u, DELTSQR=%u, BITSHIFT=%u",
		entry->freq_hz, entry->delt, entry->deltsqr, entry->bitshift);

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK1);
	if (rc != 0) {
		return rc;
	}

	/* Write DELT (6 bits in GYRO_CONFIG_STATIC3) */
	rc = icm42688_write_reg(dev, ICM42688_REG_GYRO_CONFIG_STATIC3, entry->delt);
	if (rc != 0) {
		goto exit;
	}

	/* Write DELTSQR[7:0] (GYRO_CONFIG_STATIC4) */
	rc = icm42688_write_reg(dev, ICM42688_REG_GYRO_CONFIG_STATIC4,
				(uint8_t)(entry->deltsqr & 0xFFu));
	if (rc != 0) {
		goto exit;
	}

	/* Write BITSHIFT[7:4] and DELTSQR[11:8] (GYRO_CONFIG_STATIC5) */
	uint8_t static5 = (uint8_t)((entry->bitshift << ICM42688_GYRO_AAF_BITSHIFT_SHIFT) |
				    ((entry->deltsqr >> 8) & 0x0Fu));
	rc = icm42688_write_reg(dev, ICM42688_REG_GYRO_CONFIG_STATIC5, static5);

exit:
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}

/**
 * @brief Enable or disable Accelerometer Anti-Alias Filter
 *
 * AAF range: 42 Hz to 3979 Hz.
 * IMPORTANT: AAF only works in Low Noise (LN) mode (datasheet section 12.4).
 *
 * @param dev ICM42688 device
 * @param enable true to enable, false to disable
 * @return 0 on success, negative errno on failure
 */
int icm42688_accel_aaf_enable(const struct device *dev, bool enable)
{
	int rc;
	uint8_t mask = enable ? 0 : ICM42688_ACCEL_AAF_DIS;

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK2);
	if (rc != 0) {
		return rc;
	}

	rc = icm42688_reg_update8(dev, ICM42688_REG_ACCEL_CONFIG_STATIC2,
				  ICM42688_ACCEL_AAF_DIS, mask);

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}

/**
 * @brief Set Accelerometer AAF bandwidth
 *
 * Programs DELT, DELTSQR, BITSHIFT registers based on lookup table.
 *
 * @param dev ICM42688 device
 * @param freq_hz Target 3dB bandwidth in Hz (42-3979 Hz)
 * @return 0 on success, negative errno on failure
 */
int icm42688_accel_aaf_set_freq(const struct device *dev, uint16_t freq_hz)
{
	int rc;
	const struct icm42688_aaf_entry *entry;

	rc = find_aaf_entry(freq_hz, &entry);
	if (rc != 0) {
		LOG_ERR("AAF frequency %u Hz out of range (42-3979 Hz)", freq_hz);
		return rc;
	}

	LOG_DBG("Accel AAF: %u Hz -> DELT=%u, DELTSQR=%u, BITSHIFT=%u",
		entry->freq_hz, entry->delt, entry->deltsqr, entry->bitshift);

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK2);
	if (rc != 0) {
		return rc;
	}

	/* Write DELT[6:1] and DIS[0] in ACCEL_CONFIG_STATIC2 */
	uint8_t static2 = (uint8_t)(entry->delt << ICM42688_ACCEL_AAF_DELT_SHIFT);
	/* Preserve current DIS bit */
	rc = icm42688_reg_update8(dev, ICM42688_REG_ACCEL_CONFIG_STATIC2,
				  ICM42688_ACCEL_AAF_DELT_MASK, static2);
	if (rc != 0) {
		goto exit;
	}

	/* Write DELTSQR[7:0] (ACCEL_CONFIG_STATIC3) */
	rc = icm42688_write_reg(dev, ICM42688_REG_ACCEL_CONFIG_STATIC3,
				(uint8_t)(entry->deltsqr & 0xFFu));
	if (rc != 0) {
		goto exit;
	}

	/* Write BITSHIFT[7:4] and DELTSQR[11:8] (ACCEL_CONFIG_STATIC4) */
	uint8_t static4 = (uint8_t)((entry->bitshift << ICM42688_ACCEL_AAF_BITSHIFT_SHIFT) |
				    ((entry->deltsqr >> 8) & 0x0Fu));
	rc = icm42688_write_reg(dev, ICM42688_REG_ACCEL_CONFIG_STATIC4, static4);

exit:
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}

/**
 * @brief Get current Gyroscope AAF configuration
 *
 * Reads DELT, DELTSQR, BITSHIFT and finds matching table entry.
 *
 * @param dev ICM42688 device
 * @param freq_hz Pointer to store current frequency in Hz
 * @return 0 on success, negative errno on failure
 */
int icm42688_gyro_aaf_get_freq(const struct device *dev, uint16_t *freq_hz)
{
	int rc;
	uint8_t delt, deltsqr_l, static5;
	uint16_t deltsqr;
	uint8_t bitshift;

	if (freq_hz == NULL) {
		return -EINVAL;
	}

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK1);
	if (rc != 0) {
		return rc;
	}

	rc = icm42688_read_reg(dev, ICM42688_REG_GYRO_CONFIG_STATIC3, &delt);
	if (rc != 0) {
		goto exit;
	}

	rc = icm42688_read_reg(dev, ICM42688_REG_GYRO_CONFIG_STATIC4, &deltsqr_l);
	if (rc != 0) {
		goto exit;
	}

	rc = icm42688_read_reg(dev, ICM42688_REG_GYRO_CONFIG_STATIC5, &static5);
	if (rc != 0) {
		goto exit;
	}

	deltsqr = (uint16_t)deltsqr_l | (uint16_t)((static5 & 0x0Fu) << 8);
	bitshift = (static5 & ICM42688_GYRO_AAF_BITSHIFT_MASK) >> ICM42688_GYRO_AAF_BITSHIFT_SHIFT;

	/* Find matching entry in table */
	delt &= ICM42688_GYRO_AAF_DELT_MASK;
	for (int i = 0; i < ICM42688_AAF_TABLE_SIZE; i++) {
		if (icm42688_aaf_table[i].delt == delt &&
		    icm42688_aaf_table[i].deltsqr == deltsqr &&
		    icm42688_aaf_table[i].bitshift == bitshift) {
			*freq_hz = icm42688_aaf_table[i].freq_hz;
			goto exit;
		}
	}

	/* No exact match found */
	rc = -ENOENT;

exit:
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}

/**
 * @brief Get current Accelerometer AAF configuration
 *
 * Reads DELT, DELTSQR, BITSHIFT and finds matching table entry.
 *
 * @param dev ICM42688 device
 * @param freq_hz Pointer to store current frequency in Hz
 * @return 0 on success, negative errno on failure
 */
int icm42688_accel_aaf_get_freq(const struct device *dev, uint16_t *freq_hz)
{
	int rc;
	uint8_t static2, deltsqr_l, static4;
	uint8_t delt;
	uint16_t deltsqr;
	uint8_t bitshift;

	if (freq_hz == NULL) {
		return -EINVAL;
	}

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK2);
	if (rc != 0) {
		return rc;
	}

	rc = icm42688_read_reg(dev, ICM42688_REG_ACCEL_CONFIG_STATIC2, &static2);
	if (rc != 0) {
		goto exit;
	}

	rc = icm42688_read_reg(dev, ICM42688_REG_ACCEL_CONFIG_STATIC3, &deltsqr_l);
	if (rc != 0) {
		goto exit;
	}

	rc = icm42688_read_reg(dev, ICM42688_REG_ACCEL_CONFIG_STATIC4, &static4);
	if (rc != 0) {
		goto exit;
	}

	delt = (static2 & ICM42688_ACCEL_AAF_DELT_MASK) >> ICM42688_ACCEL_AAF_DELT_SHIFT;
	deltsqr = (uint16_t)deltsqr_l | (uint16_t)((static4 & 0x0Fu) << 8);
	bitshift = (static4 & ICM42688_ACCEL_AAF_BITSHIFT_MASK) >>
		   ICM42688_ACCEL_AAF_BITSHIFT_SHIFT;

	/* Find matching entry in table */
	for (int i = 0; i < ICM42688_AAF_TABLE_SIZE; i++) {
		if (icm42688_aaf_table[i].delt == delt &&
		    icm42688_aaf_table[i].deltsqr == deltsqr &&
		    icm42688_aaf_table[i].bitshift == bitshift) {
			*freq_hz = icm42688_aaf_table[i].freq_hz;
			goto exit;
		}
	}

	/* No exact match found */
	rc = -ENOENT;

exit:
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}
