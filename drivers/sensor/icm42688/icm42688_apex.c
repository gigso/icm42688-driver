/* SPDX-License-Identifier: Apache-2.0 */
#include "icm42688.h"
#include "icm42688_reg.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_DECLARE(icm42688, CONFIG_SENSOR_LOG_LEVEL);

#ifdef CONFIG_ICM42688_APEX

static int apex_write_config0(const struct device *dev,
			      bool dmp_ps,
			      uint8_t dmp_odr_sel,
			      bool tap_en, bool ped_en, bool tilt_en, bool r2w_en)
{
	uint8_t v = 0;

	if (dmp_ps)  { v |= ICM42688_APEX_DMP_POWER_SAVE; }
	if (tap_en)  { v |= ICM42688_APEX_TAP_ENABLE; }
	if (ped_en)  { v |= ICM42688_APEX_PED_ENABLE; }
	if (tilt_en) { v |= ICM42688_APEX_TILT_ENABLE; }
	if (r2w_en)  { v |= ICM42688_APEX_R2W_ENABLE; }

	v |= (dmp_odr_sel & ICM42688_APEX_DMP_ODR_MASK);
	return icm42688_write_reg(dev, ICM42688_REG_APEX_CONFIG0, v);
}

static int apex_set_accel_odr_via_sensor_api(const struct device *dev, uint32_t hz)
{
	struct sensor_value v = { .val1 = (int32_t)hz, .val2 = 0 };
	return sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
}

static int apex_route_interrupts(const struct device *dev, uint8_t feature_mask, bool use_int2)
{
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK4);

	uint8_t reg = use_int2 ? ICM42688_REG_INT_SOURCE7 : ICM42688_REG_INT_SOURCE6;
	uint8_t m = 0;

	if (feature_mask & ICM42688_APEX_FEAT_PEDOMETER) { m |= ICM42688_INT_SRC_STEP_DET_EN; }
	if (feature_mask & ICM42688_APEX_FEAT_TILT)      { m |= ICM42688_INT_SRC_TILT_DET_EN; }
	if (feature_mask & ICM42688_APEX_FEAT_R2W)       { m |= (ICM42688_INT_SRC_WAKE_DET_EN | ICM42688_INT_SRC_SLEEP_DET_EN); }
	if (feature_mask & ICM42688_APEX_FEAT_TAP)       { m |= ICM42688_INT_SRC_TAP_DET_EN; }

	return icm42688_reg_update8(dev, reg, m, m);
}

static int apex_program_bank4_defaults(const struct device *dev, const struct icm42688_config *cfg)
{
	int rc;
	uint8_t bank_sel_rd = 0, cfg1_rd = 0, cfg4_rd = 0;

	/* Switch to Bank4 */
	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	LOG_INF("Bank select to Bank4, rc=%d", rc);

	/* Verify bank switch by reading REG_BANK_SEL */
	(void)icm42688_read_reg(dev, ICM42688_REG_REG_BANK_SEL, &bank_sel_rd);
	LOG_INF("REG_BANK_SEL readback=0x%02x (expect 0x04 for Bank4)", bank_sel_rd);

	LOG_INF("Programming Bank4 APEX configs...");

	/* Write CONFIG1 and verify */
	rc = icm42688_write_reg(dev, ICM42688_REG_APEX_CONFIG1, 0x10);
	LOG_INF("Write APEX_CONFIG1=0x10, rc=%d", rc);
	if (rc < 0) { LOG_ERR("CONFIG1 write failed: %d", rc); return rc; }
	
	(void)icm42688_read_reg(dev, ICM42688_REG_APEX_CONFIG1, &cfg1_rd);
	LOG_INF("Readback APEX_CONFIG1=0x%02x (expect 0x10)", cfg1_rd);

	rc = icm42688_write_reg(dev, ICM42688_REG_APEX_CONFIG2, 0x08);
	if (rc < 0) { LOG_ERR("CONFIG2 write failed: %d", rc); return rc; }
	rc = icm42688_write_reg(dev, ICM42688_REG_APEX_CONFIG3, 0x20);
	if (rc < 0) { LOG_ERR("CONFIG3 write failed: %d", rc); return rc; }

	{
		uint8_t v = 0;

		v &= ~ICM42688_APEX_CFG4_TILT_WAIT_TIME_MASK;
		v |= (uint8_t)((cfg->apex_tilt_wait_time_sel & 0x3u) << ICM42688_APEX_CFG4_TILT_WAIT_TIME_SHIFT);

		v &= ~ICM42688_APEX_CFG4_SLEEP_TIMEOUT_MASK;
		v |= (uint8_t)((cfg->apex_sleep_timeout_sel & 0x7u) << ICM42688_APEX_CFG4_SLEEP_TIMEOUT_SHIFT);

		/* Enable both single and double tap detection */
		v |= ICM42688_APEX_CFG4_TAP_SINGLE_EN | ICM42688_APEX_CFG4_TAP_DOUBLE_EN;

		LOG_INF("Writing APEX_CONFIG4=0x%02x (single=%d double=%d)", v,
			(v >> 1) & 1, (v >> 2) & 1);

		rc = icm42688_write_reg(dev, ICM42688_REG_APEX_CONFIG4, v);
		if (rc < 0) { return rc; }
		
		(void)icm42688_read_reg(dev, ICM42688_REG_APEX_CONFIG4, &cfg4_rd);
		LOG_INF("Readback APEX_CONFIG4=0x%02x", cfg4_rd);
	}

	rc = icm42688_reg_update8(dev, ICM42688_REG_APEX_CONFIG5,
			ICM42688_APEX_CFG5_MOUNTING_MATRIX_MASK,
			(uint8_t)(cfg->apex_mounting_matrix & 0x7u));
	if (rc < 0) { return rc; }

	rc = icm42688_reg_update8(dev, ICM42688_REG_APEX_CONFIG6,
			ICM42688_APEX_CFG6_SLEEP_GESTURE_DELAY_MASK,
			(uint8_t)(cfg->apex_sleep_gesture_delay & 0x7u));
	if (rc < 0) { return rc; }

	rc = icm42688_reg_update8(dev, ICM42688_REG_APEX_CONFIG9,
			ICM42688_APEX_CFG9_SENSITIVITY_MODE_MASK,
			(uint8_t)(cfg->apex_sensitivity_mode & 0x1u));
	if (rc < 0) { return rc; }

	return 0;
}

int icm42688_apex_init(const struct device *dev, uint8_t feature_mask)
{
	struct icm42688_data *data = dev->data;
	const struct icm42688_config *cfg = dev->config;

	if (!cfg->apex_enable) {
		return -ENOTSUP;
	}

	const bool ped  = (feature_mask & ICM42688_APEX_FEAT_PEDOMETER) != 0;
	const bool tilt = (feature_mask & ICM42688_APEX_FEAT_TILT) != 0;
	const bool r2w  = (feature_mask & ICM42688_APEX_FEAT_R2W) != 0;
	const bool tap  = (feature_mask & ICM42688_APEX_FEAT_TAP) != 0;

	const uint8_t dmp_odr_sel = cfg->apex_dmp_odr_sel;
	/* For tap reliability use >=500 Hz; otherwise use configured 25/50 Hz */
	const uint32_t accel_odr = tap ? 1000u : ((dmp_odr_sel == ICM42688_APEX_DMP_ODR_25HZ) ? 25u : 50u);

	LOG_INF("APEX init: tap=%d accel_odr=%u", tap, accel_odr);

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	/* Set accel ODR first - critical for tap */
	int rc = apex_set_accel_odr_via_sensor_api(dev, accel_odr);
	LOG_INF("Set accel ODR to %uHz, rc=%d", accel_odr, rc);

	/* Step 1: Disable all APEX features before DMP reset */
	rc = apex_write_config0(dev, false, dmp_odr_sel, false, false, false, false);
	if (rc < 0) {
		return rc;
	}
	k_sleep(K_MSEC(1));

	/* Step 2: DMP memory reset */
	rc = icm42688_write_reg(dev, ICM42688_REG_SIGNAL_PATH_RESET, ICM42688_SPR_DMP_MEM_RESET_EN);
	if (rc < 0) {
		return rc;
	}
	k_sleep(K_MSEC(1));

	/* Step 3: DMP initialization (BEFORE Bank4 config per datasheet) */
	rc = icm42688_write_reg(dev, ICM42688_REG_SIGNAL_PATH_RESET, ICM42688_SPR_DMP_INIT_EN);
	if (rc < 0) {
		return rc;
	}
	k_sleep(K_MSEC(50));

	/* Step 4: Program Bank4 APEX configuration AFTER DMP init */
	rc = apex_program_bank4_defaults(dev, cfg);
	if (rc < 0) {
		LOG_ERR("Bank4 programming failed: %d", rc);
		return rc;
	}

	/* Verify Bank4 was written */
	uint8_t cfg1_rd = 0, cfg4_rd = 0;
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	(void)icm42688_read_reg(dev, ICM42688_REG_APEX_CONFIG1, &cfg1_rd);
	(void)icm42688_read_reg(dev, ICM42688_REG_APEX_CONFIG4, &cfg4_rd);
	LOG_INF("Verify: APEX_CONFIG1=0x%02x CONFIG4=0x%02x", cfg1_rd, cfg4_rd);

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	
	/* Step 5: Route interrupts */
	rc = apex_route_interrupts(dev, feature_mask, cfg->apex_route_int2);
	if (rc < 0) {
		return rc;
	}

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	
	/* Step 6: FINAL - Enable APEX features AFTER DMP init */
	rc = apex_write_config0(dev, cfg->apex_dmp_power_save, dmp_odr_sel,
				tap, ped, tilt, r2w);
	if (rc < 0) {
		return rc;
	}

	data->apex_enabled = (feature_mask != 0);
	data->apex_features_enabled = feature_mask;

	LOG_INF("APEX enabled: mask=0x%02x DMP_ODR=%s accel_odr=%uHz int=%s",
		feature_mask,
		(dmp_odr_sel == ICM42688_APEX_DMP_ODR_25HZ) ? "25Hz" : "50Hz",
		(unsigned)accel_odr,
		cfg->apex_route_int2 ? "INT2" : "INT1");

	return 0;
}

int icm42688_apex_enable_feature(const struct device *dev, uint8_t feature_mask, bool enable)
{
	struct icm42688_data *data = dev->data;
	const struct icm42688_config *cfg = dev->config;

	if (!cfg->apex_enable) {
		return -ENOTSUP;
	}

	uint8_t new_mask = data->apex_features_enabled;
	if (enable) {
		new_mask |= feature_mask;
	} else {
		new_mask &= (uint8_t)~feature_mask;
	}

	bool ped  = (new_mask & ICM42688_APEX_FEAT_PEDOMETER) != 0;
	bool tilt = (new_mask & ICM42688_APEX_FEAT_TILT) != 0;
	bool r2w  = (new_mask & ICM42688_APEX_FEAT_R2W) != 0;
	bool tap  = (new_mask & ICM42688_APEX_FEAT_TAP) != 0;

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	int rc = apex_write_config0(dev, cfg->apex_dmp_power_save, cfg->apex_dmp_odr_sel,
				    tap, ped, tilt, r2w);
	if (rc < 0) {
		return rc;
	}

	rc = apex_route_interrupts(dev, new_mask, cfg->apex_route_int2);
	if (rc < 0) {
		return rc;
	}

	data->apex_features_enabled = new_mask;
	data->apex_enabled = (new_mask != 0);

	return 0;
}

int icm42688_apex_read_step_count_u16(const struct device *dev, uint16_t *steps)
{
	if (!steps) {
		return -EINVAL;
	}

	uint8_t b[2];

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	int rc = icm42688_read_burst(dev, ICM42688_REG_APEX_DATA0, b, sizeof(b));
	if (rc < 0) {
		return rc;
	}

	*steps = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
	return 0;
}

int icm42688_apex_read_step_cadence_u8(const struct device *dev, uint8_t *cad)
{
	if (!cad) {
		return -EINVAL;
	}

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return icm42688_read_reg(dev, ICM42688_REG_APEX_DATA2, cad);
}

int icm42688_apex_read_activity_class_u8(const struct device *dev, uint8_t *cls)
{
	if (!cls) {
		return -EINVAL;
	}

	uint8_t v = 0;

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	int rc = icm42688_read_reg(dev, ICM42688_REG_APEX_DATA3, &v);
	if (rc < 0) {
		return rc;
	}

	*cls = (uint8_t)(v & 0x03u);
	return 0;
}

int icm42688_apex_read_tap(const struct device *dev,
			   uint8_t *tap_num, uint8_t *tap_axis, uint8_t *tap_dir,
			   uint8_t *double_tap_timing)
{
	uint8_t d4 = 0, d5 = 0;

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	int rc = icm42688_read_reg(dev, ICM42688_REG_APEX_DATA4, &d4);
	if (rc < 0) {
		return rc;
	}
	rc = icm42688_read_reg(dev, ICM42688_REG_APEX_DATA5, &d5);
	if (rc < 0) {
		return rc;
	}

	if (tap_num) {
		*tap_num = (uint8_t)((d4 & ICM42688_APEX_DATA4_TAP_NUM_MASK) >> ICM42688_APEX_DATA4_TAP_NUM_SHIFT);
	}
	if (tap_axis) {
		*tap_axis = (uint8_t)((d4 & ICM42688_APEX_DATA4_TAP_AXIS_MASK) >> ICM42688_APEX_DATA4_TAP_AXIS_SHIFT);
	}
	if (tap_dir) {
		*tap_dir = (uint8_t)((d4 & ICM42688_APEX_DATA4_TAP_DIR_MASK) ? 1u : 0u);
	}
	if (double_tap_timing) {
		*double_tap_timing = (uint8_t)(d5 & 0x3Fu);
	}

	struct icm42688_data *data = dev->data;
	data->last_tap_num = (uint8_t)((d4 & ICM42688_APEX_DATA4_TAP_NUM_MASK) >> ICM42688_APEX_DATA4_TAP_NUM_SHIFT);
	data->last_tap_axis = (uint8_t)((d4 & ICM42688_APEX_DATA4_TAP_AXIS_MASK) >> ICM42688_APEX_DATA4_TAP_AXIS_SHIFT);
	data->last_tap_dir = (uint8_t)((d4 & ICM42688_APEX_DATA4_TAP_DIR_MASK) ? 1u : 0u);
	data->last_double_tap_timing = (uint8_t)(d5 & 0x3Fu);

	return 0;
}

/* Route APEX events to INT2 */
int icm42688_apex_route_to_int2(const struct device *dev, uint8_t feature_mask)
{
	int rc;

	LOG_INF("APEX route to INT2: feature_mask=0x%02x", feature_mask);

	/* Switch to Bank4 and update INT_SOURCE7 for INT2 routing */
	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) {
		return rc;
	}

	uint8_t m = 0;

	if (feature_mask & ICM42688_APEX_FEAT_PEDOMETER) { m |= ICM42688_INT_SRC_STEP_DET_EN; }
	if (feature_mask & ICM42688_APEX_FEAT_TILT)      { m |= ICM42688_INT_SRC_TILT_DET_EN; }
	if (feature_mask & ICM42688_APEX_FEAT_R2W)       { m |= (ICM42688_INT_SRC_WAKE_DET_EN | ICM42688_INT_SRC_SLEEP_DET_EN); }
	if (feature_mask & ICM42688_APEX_FEAT_TAP)       { m |= ICM42688_INT_SRC_TAP_DET_EN; }

	/* Write to INT_SOURCE7 for INT2 */
	rc = icm42688_reg_update8(dev, ICM42688_REG_INT_SOURCE7, m, m);
	if (rc < 0) {
		LOG_ERR("INT2 routing write failed: %d", rc);
		return rc;
	}

	LOG_INF("APEX events routed to INT2 (INT_SOURCE7=0x%02x)", m);

	return 0;
}

/* Remove APEX events from INT2 routing */
int icm42688_apex_route_from_int2(const struct device *dev, uint8_t feature_mask)
{
	int rc;

	LOG_INF("APEX unroute from INT2: feature_mask=0x%02x", feature_mask);

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) {
		return rc;
	}

	uint8_t m = 0;

	if (feature_mask & ICM42688_APEX_FEAT_PEDOMETER) { m |= ICM42688_INT_SRC_STEP_DET_EN; }
	if (feature_mask & ICM42688_APEX_FEAT_TILT)      { m |= ICM42688_INT_SRC_TILT_DET_EN; }
	if (feature_mask & ICM42688_APEX_FEAT_R2W)       { m |= (ICM42688_INT_SRC_WAKE_DET_EN | ICM42688_INT_SRC_SLEEP_DET_EN); }
	if (feature_mask & ICM42688_APEX_FEAT_TAP)       { m |= ICM42688_INT_SRC_TAP_DET_EN; }

	/* Clear from INT_SOURCE7 for INT2 */
	rc = icm42688_reg_update8(dev, ICM42688_REG_INT_SOURCE7, m, 0);
	if (rc < 0) {
		LOG_ERR("INT2 unrouting write failed: %d", rc);
		return rc;
	}

	LOG_INF("APEX events unrouted from INT2");

	return 0;
}

#endif /* CONFIG_ICM42688_APEX */
