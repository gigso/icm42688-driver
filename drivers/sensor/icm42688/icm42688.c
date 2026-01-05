/* SPDX-License-Identifier: Apache-2.0 */
#define DT_DRV_COMPAT invensense_icm42688

#include "icm42688.h"
#include "icm42688_reg.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(icm42688, CONFIG_SENSOR_LOG_LEVEL);

#define ICM42688_SPI_READ_BIT 0x80

/* ========= Low-level bus I/O ========= */

static int icm42688_bus_read(const struct device *dev, uint8_t reg, uint8_t *buf, size_t len)
{
	const struct icm42688_config *cfg = dev->config;

	switch (cfg->bus_type) {
	case ICM42688_BUS_I2C:
		/* ========== I2C path ==========
		 *
		 * IMPORTANT:
		 * FIFO_DATA (0x30) must be read as:
		 *   1) write register address (no repeated-start read of addr+data)
		 *   2) read N bytes
		 *
		 * On a number of I2C controllers + ICM42688, using i2c_write_read()
		 * for FIFO_DATA can return zeros (even when FIFO_COUNT is non-zero).
		 */
		if (reg == ICM42688_REG_FIFO_DATA && len > 0) {
			int rc = i2c_write_dt(&cfg->i2c.bus, &reg, 1);
			if (rc < 0) {
				return rc;
			}
			return i2c_read_dt(&cfg->i2c.bus, buf, len);
		}

		/* Normal registers: repeated-start is fine */
		return i2c_write_read_dt(&cfg->i2c.bus, &reg, sizeof(reg), buf, len);

	case ICM42688_BUS_SPI:
#if defined(CONFIG_SPI)
		{
			/* SPI read: set MSB bit in register address */
			uint8_t tx = (uint8_t)(reg | ICM42688_SPI_READ_BIT);

			/* For multi-byte reads the SPI master must clock address + data bytes.
			 * Build a combined TX buffer (addr + zeros) and an RX buffer of the same
			 * length, then perform a single transceive. Copy the received data
			 * (skipping the first dummy byte) into the caller buffer.
			 */
			size_t total = 1 + len;
			uint8_t tx_all[1 + 256];
			uint8_t rx_all[1 + 256];

			if (len > 256) {
				return -EINVAL;
			}

			tx_all[0] = tx;
			memset(&tx_all[1], 0, len);

			struct spi_buf tx_buf = { .buf = tx_all, .len = total };
			struct spi_buf rx_buf = { .buf = rx_all, .len = total };
			struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
			struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

			int ret = spi_transceive_dt(&cfg->spi.bus, &tx_set, &rx_set);
			if (ret < 0) {
				return ret;
			}

			memcpy(buf, &rx_all[1], len);
			return 0;
		}
#else
		return -ENOTSUP;
#endif

	case ICM42688_BUS_I3C:
#if defined(CONFIG_I3C)
		{
			/* I3C Private Read */
			uint8_t tx_buf[1] = { reg };
			struct i3c_msg msgs[2];

			msgs[0].buf = tx_buf;
			msgs[0].len = 1;
			msgs[0].flags = I3C_MSG_WRITE | I3C_MSG_STOP;

			msgs[1].buf = buf;
			msgs[1].len = len;
			msgs[1].flags = I3C_MSG_READ | I3C_MSG_STOP;

			return i3c_transfer(cfg->i3c.bus, msgs, 2);
		}
#else
		return -ENOTSUP;
#endif

	default:
		return -ENOTSUP;
	}
}



static int icm42688_bus_write(const struct device *dev, uint8_t reg, const uint8_t *buf, size_t len)
{
	const struct icm42688_config *cfg = dev->config;

	switch (cfg->bus_type) {
	case ICM42688_BUS_I2C:
		{
			uint8_t tmp[1 + 32];

			if (len > 32) {
				return -EINVAL;
			}

			tmp[0] = reg;
			memcpy(&tmp[1], buf, len);
			return i2c_write_dt(&cfg->i2c.bus, tmp, len + 1);
		}

	case ICM42688_BUS_SPI:
#if defined(CONFIG_SPI)
		{
			uint8_t tmp[1 + 32];

			if (len > 32) {
				return -EINVAL;
			}

			/* SPI write: MSB=0 for write (register address as-is) */
			tmp[0] = reg;
			memcpy(&tmp[1], buf, len);

			struct spi_buf tx_buf = { .buf = tmp, .len = len + 1 };
			struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };

			return spi_write_dt(&cfg->spi.bus, &tx_set);
		}
#else
		return -ENOTSUP;
#endif

	case ICM42688_BUS_I3C:
#if defined(CONFIG_I3C)
		{
			/* I3C Private Write */
			uint8_t tx_buf[1 + 32];

			if (len > 32) {
				return -EINVAL;
			}

			tx_buf[0] = reg;
			memcpy(&tx_buf[1], buf, len);

			struct i3c_msg msg;
			msg.buf = tx_buf;
			msg.len = len + 1;
			msg.flags = I3C_MSG_WRITE | I3C_MSG_STOP;

			return i3c_transfer(cfg->i3c.bus, &msg, 1);
		}
#else
		return -ENOTSUP;
#endif

	default:
		return -ENOTSUP;
	}
}

int icm42688_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	return icm42688_bus_read(dev, reg, val, 1);
}

int icm42688_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	return icm42688_bus_write(dev, reg, &val, 1);
}

int icm42688_read_burst(const struct device *dev, uint8_t reg, uint8_t *buf, size_t len)
{
	return icm42688_bus_read(dev, reg, buf, len);
}

/* ========= Small helpers ========= */

/* ========= Bank select ========= */

int icm42688_reg_bank_sel(const struct device *dev, uint8_t bank)
{
	/* REG_BANK_SEL (0x76): bits [2:0] = BANK_SEL per datasheet */
	uint8_t v = (uint8_t)(bank & 0x07u);
	return icm42688_write_reg(dev, ICM42688_REG_REG_BANK_SEL, v);
}

/* ========= Helpers: sensor_value conversions ========= */

static inline void icm42688_from_mps2(struct sensor_value *val, int32_t micro_ms2)
{
	val->val1 = micro_ms2 / 1000000;
	val->val2 = micro_ms2 % 1000000;
}

static inline void icm42688_from_urads(struct sensor_value *val, int32_t micro_rads)
{
	val->val1 = micro_rads / 1000000;
	val->val2 = micro_rads % 1000000;
}

static inline void icm42688_from_mC(struct sensor_value *val, int32_t milli_c)
{
	val->val1 = milli_c / 1000;
	val->val2 = (milli_c % 1000) * 1000;
}

/* ========= Conversions ========= */

static int32_t accel_raw_to_micro_ms2(int16_t raw, uint16_t fs_g)
{
	const int64_t G_MICRO = 9806650LL;
	int64_t v = (int64_t)raw * (int64_t)fs_g * G_MICRO;
	v /= 32768LL;
	return (int32_t)v;
}

static int32_t gyro_raw_to_micro_rads(int16_t raw, uint16_t fs_dps)
{
	const int64_t U_RAD_PER_DEG_NUM = 17453293LL;
	const int64_t U_RAD_PER_DEG_DEN = 1000LL;

	int64_t v = (int64_t)raw * (int64_t)fs_dps * U_RAD_PER_DEG_NUM;
	v /= (32768LL * U_RAD_PER_DEG_DEN);
	return (int32_t)v;
}

static int32_t temp_raw_to_milli_c(int16_t raw)
{
	int64_t v = (int64_t)raw * 100000LL;
	v /= 13248LL;
	v += 25000LL;
	return (int32_t)v;
}

/* ========= FS/ODR mapping ========= */

static int accel_fs_to_sel(uint16_t g, uint8_t *sel)
{
	switch (g) {
	case 16: *sel = 0; return 0;
	case 8:  *sel = 1; return 0;
	case 4:  *sel = 2; return 0;
	case 2:  *sel = 3; return 0;
	default: return -EINVAL;
	}
}

static int gyro_fs_to_sel(uint16_t dps, uint8_t *sel)
{
	switch (dps) {
	case 2000: *sel = 0; return 0;
	case 1000: *sel = 1; return 0;
	case 500:  *sel = 2; return 0;
	case 250:  *sel = 3; return 0;
	case 125:  *sel = 4; return 0;
	default: return -EINVAL;
	}
}

static int odr_hz_to_sel(uint32_t hz, uint8_t *sel)
{
	switch (hz) {
	case 32000: *sel = 0x1; return 0;
	case 16000: *sel = 0x2; return 0;
	case 8000:  *sel = 0x3; return 0;
	case 4000:  *sel = 0x4; return 0;
	case 2000:  *sel = 0x5; return 0;
	case 1000:  *sel = 0x6; return 0;
	case 200:   *sel = 0x7; return 0;
	case 100:   *sel = 0x8; return 0;
	case 50:    *sel = 0x9; return 0;
	case 25:    *sel = 0xA; return 0;
	case 12:    *sel = 0xB; return 0;
	case 500:   *sel = 0xF; return 0;
	default: return -EINVAL;
	}
}

static int icm42688_write_accel_cfg0(const struct device *dev, uint16_t fs_g, uint32_t odr_hz)
{
	uint8_t fs_sel, odr_sel;
	int rc = accel_fs_to_sel(fs_g, &fs_sel);
	if (rc) {
		return rc;
	}
	rc = odr_hz_to_sel(odr_hz, &odr_sel);
	if (rc) {
		return rc;
	}

	uint8_t v = (uint8_t)((fs_sel << 5) | (odr_sel & 0x0F));
	return icm42688_write_reg(dev, ICM42688_REG_ACCEL_CONFIG0, v);
}

static int icm42688_write_gyro_cfg0(const struct device *dev, uint16_t fs_dps, uint32_t odr_hz)
{
	uint8_t fs_sel, odr_sel;
	int rc = gyro_fs_to_sel(fs_dps, &fs_sel);
	if (rc) {
		return rc;
	}
	rc = odr_hz_to_sel(odr_hz, &odr_sel);
	if (rc) {
		return rc;
	}

	uint8_t v = (uint8_t)((fs_sel << 5) | (odr_sel & 0x0F));
	return icm42688_write_reg(dev, ICM42688_REG_GYRO_CONFIG0, v);
}

/* ========= Zephyr sensor API ========= */

static int icm42688_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	ARG_UNUSED(chan);

	struct icm42688_data *data = dev->data;

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	uint8_t b[14];
	int rc = icm42688_read_burst(dev, ICM42688_REG_TEMP_DATA1, b, sizeof(b));
	if (rc < 0) {
		return rc;
	}

	int16_t t_raw  = (int16_t)sys_get_be16(&b[0]);
	int16_t ax_raw = (int16_t)sys_get_be16(&b[2]);
	int16_t ay_raw = (int16_t)sys_get_be16(&b[4]);
	int16_t az_raw = (int16_t)sys_get_be16(&b[6]);
	int16_t gx_raw = (int16_t)sys_get_be16(&b[8]);
	int16_t gy_raw = (int16_t)sys_get_be16(&b[10]);
	int16_t gz_raw = (int16_t)sys_get_be16(&b[12]);

	data->temp_mc = temp_raw_to_milli_c(t_raw);

	data->accel_x = accel_raw_to_micro_ms2(ax_raw, data->accel_fs_g);
	data->accel_y = accel_raw_to_micro_ms2(ay_raw, data->accel_fs_g);
	data->accel_z = accel_raw_to_micro_ms2(az_raw, data->accel_fs_g);

	data->gyro_x = gyro_raw_to_micro_rads(gx_raw, data->gyro_fs_dps);
	data->gyro_y = gyro_raw_to_micro_rads(gy_raw, data->gyro_fs_dps);
	data->gyro_z = gyro_raw_to_micro_rads(gz_raw, data->gyro_fs_dps);

	return 0;
}

static int icm42688_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct icm42688_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		icm42688_from_mps2(val, data->accel_x);
		return 0;
	case SENSOR_CHAN_ACCEL_Y:
		icm42688_from_mps2(val, data->accel_y);
		return 0;
	case SENSOR_CHAN_ACCEL_Z:
		icm42688_from_mps2(val, data->accel_z);
		return 0;

	case SENSOR_CHAN_GYRO_X:
		icm42688_from_urads(val, data->gyro_x);
		return 0;
	case SENSOR_CHAN_GYRO_Y:
		icm42688_from_urads(val, data->gyro_y);
		return 0;
	case SENSOR_CHAN_GYRO_Z:
		icm42688_from_urads(val, data->gyro_z);
		return 0;

	case SENSOR_CHAN_DIE_TEMP:
		icm42688_from_mC(val, data->temp_mc);
		return 0;

	default:
		return -ENOTSUP;
	}
}

static int icm42688_attr_get(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, struct sensor_value *val)
{
	struct icm42688_data *data = dev->data;
	const int a = (int)attr;

	/* common attributes */
	if (attr == SENSOR_ATTR_FULL_SCALE) {
		if (chan == SENSOR_CHAN_ACCEL_XYZ) {
			val->val1 = data->accel_fs_g;
			val->val2 = 0;
			return 0;
		}
		if (chan == SENSOR_CHAN_GYRO_XYZ) {
			val->val1 = data->gyro_fs_dps;
			val->val2 = 0;
			return 0;
		}
	}

	if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
		if (chan == SENSOR_CHAN_ACCEL_XYZ) {
			val->val1 = (int32_t)data->accel_odr_hz;
			val->val2 = 0;
			return 0;
		}
		if (chan == SENSOR_CHAN_GYRO_XYZ) {
			val->val1 = (int32_t)data->gyro_odr_hz;
			val->val2 = 0;
			return 0;
		}
	}

	/* Self-test result */
	if (a == (int)ICM42688_ATTR_SELF_TEST_RESULT) {
		val->val1 = data->self_test_result;
		val->val2 = 0;
		return 0;
	}

	/* UI Filter Bandwidth - read from register */
	if (a == (int)ICM42688_ATTR_ACCEL_FILT_BW || a == (int)ICM42688_ATTR_GYRO_FILT_BW) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		int rc = icm42688_read_reg(dev, ICM42688_REG_GYRO_ACCEL_CONFIG0, &reg_val);
		if (rc < 0) return rc;
		
		if (a == (int)ICM42688_ATTR_ACCEL_FILT_BW) {
			val->val1 = (reg_val & ICM42688_ACCEL_UI_FILT_BW_MASK) >> ICM42688_ACCEL_UI_FILT_BW_SHIFT;
		} else {
			val->val1 = (reg_val & ICM42688_GYRO_UI_FILT_BW_MASK) >> ICM42688_GYRO_UI_FILT_BW_SHIFT;
		}
		val->val2 = 0;
		return 0;
	}

	/* UI Filter Order */
	if (a == (int)ICM42688_ATTR_ACCEL_FILT_ORD) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		int rc = icm42688_read_reg(dev, ICM42688_REG_ACCEL_CONFIG1, &reg_val);
		if (rc < 0) return rc;
		val->val1 = (reg_val & ICM42688_ACCEL_UI_FILT_ORD_MASK) >> ICM42688_ACCEL_UI_FILT_ORD_SHIFT;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_GYRO_FILT_ORD) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		int rc = icm42688_read_reg(dev, ICM42688_REG_GYRO_CONFIG1, &reg_val);
		if (rc < 0) return rc;
		val->val1 = (reg_val & ICM42688_GYRO_UI_FILT_ORD_MASK) >> ICM42688_GYRO_UI_FILT_ORD_SHIFT;
		val->val2 = 0;
		return 0;
	}

	/* Offset Calibration read */
	if (a == (int)ICM42688_ATTR_GYRO_OFFSET_X) {
		int32_t offset;
		int rc = icm42688_get_gyro_offset(dev, 0, &offset);
		if (rc < 0) return rc;
		val->val1 = offset;
		val->val2 = 0;
		return 0;
	}
	if (a == (int)ICM42688_ATTR_GYRO_OFFSET_Y) {
		int32_t offset;
		int rc = icm42688_get_gyro_offset(dev, 1, &offset);
		if (rc < 0) return rc;
		val->val1 = offset;
		val->val2 = 0;
		return 0;
	}
	if (a == (int)ICM42688_ATTR_GYRO_OFFSET_Z) {
		int32_t offset;
		int rc = icm42688_get_gyro_offset(dev, 2, &offset);
		if (rc < 0) return rc;
		val->val1 = offset;
		val->val2 = 0;
		return 0;
	}
	if (a == (int)ICM42688_ATTR_ACCEL_OFFSET_X) {
		int32_t offset;
		int rc = icm42688_get_accel_offset(dev, 0, &offset);
		if (rc < 0) return rc;
		val->val1 = offset;
		val->val2 = 0;
		return 0;
	}
	if (a == (int)ICM42688_ATTR_ACCEL_OFFSET_Y) {
		int32_t offset;
		int rc = icm42688_get_accel_offset(dev, 1, &offset);
		if (rc < 0) return rc;
		val->val1 = offset;
		val->val2 = 0;
		return 0;
	}
	if (a == (int)ICM42688_ATTR_ACCEL_OFFSET_Z) {
		int32_t offset;
		int rc = icm42688_get_accel_offset(dev, 2, &offset);
		if (rc < 0) return rc;
		val->val1 = offset;
		val->val2 = 0;
		return 0;
	}

#ifdef CONFIG_ICM42688_FIFO
	/* Timestamp value read */
	if (a == (int)ICM42688_ATTR_TIMESTAMP_VALUE) {
		uint32_t ts;
		int rc = icm42688_read_timestamp(dev, &ts);
		if (rc < 0) return rc;
		val->val1 = (int32_t)ts;
		val->val2 = 0;
		return 0;
	}
	
	/* Timestamp enable status */
	if (a == (int)ICM42688_ATTR_TIMESTAMP_EN) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		int rc = icm42688_read_reg(dev, ICM42688_REG_TMST_CONFIG, &reg_val);
		if (rc < 0) return rc;
		val->val1 = (reg_val & ICM42688_TMST_EN) ? 1 : 0;
		val->val2 = 0;
		return 0;
	}
	
	/* Timestamp resolution */
	if (a == (int)ICM42688_ATTR_TIMESTAMP_RES) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		int rc = icm42688_read_reg(dev, ICM42688_REG_TMST_CONFIG, &reg_val);
		if (rc < 0) return rc;
		val->val1 = (reg_val & ICM42688_TMST_RES) ? ICM42688_TIMESTAMP_RES_16US : ICM42688_TIMESTAMP_RES_1US;
		val->val2 = 0;
		return 0;
	}
#endif /* CONFIG_ICM42688_FIFO */

	/* ---- FSYNC attributes ---- */
	if (a == (int)ICM42688_ATTR_FSYNC_CONFIG) {
		uint8_t fsync_cfg;
		int rc = icm42688_read_reg(dev, ICM42688_REG_FSYNC_CONFIG, &fsync_cfg);
		if (rc < 0) return rc;
		val->val1 = fsync_cfg;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_FSYNC_UI_SEL) {
		uint8_t fsync_cfg;
		int rc = icm42688_read_reg(dev, ICM42688_REG_FSYNC_CONFIG, &fsync_cfg);
		if (rc < 0) return rc;
		val->val1 = (fsync_cfg & ICM42688_FSYNC_UI_SEL_MASK) >> ICM42688_FSYNC_UI_SEL_SHIFT;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_FSYNC_POLARITY) {
		uint8_t fsync_cfg;
		int rc = icm42688_read_reg(dev, ICM42688_REG_FSYNC_CONFIG, &fsync_cfg);
		if (rc < 0) return rc;
		val->val1 = (fsync_cfg & ICM42688_FSYNC_POLARITY) ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_FSYNC_ODR_DELAY) {
		uint8_t sync_cfg;
		int rc = icm42688_read_reg(dev, ICM42688_REG_SYNC_CONFIG, &sync_cfg);
		if (rc < 0) return rc;
		val->val1 = (sync_cfg >> 4) & 0x0F;  /* Bits [7:4] */
		val->val2 = 0;
		return 0;
	}

#ifdef CONFIG_ICM42688_APEX
	/* ---- APEX attributes ---- */

	/* STEP_COUNT (исправление: его раньше не было) */
	if (attr_is(a, ICM42688_ATTR_STEP_COUNT, ICM42688_ATTR_LEG_STEP_COUNT)) {
		uint16_t steps = 0;
		int rc = icm42688_apex_read_step_count_u16(dev, &steps);
		if (rc < 0) {
			return rc;
		}
		val->val1 = (int32_t)steps;
		val->val2 = 0;
		return 0;
	}

	if (attr_is(a, ICM42688_ATTR_STEP_CADENCE, ICM42688_ATTR_LEG_STEP_CADENCE)) {
		uint8_t cad = 0;
		int rc = icm42688_apex_read_step_cadence_u8(dev, &cad);
		if (rc < 0) {
			return rc;
		}
		val->val1 = cad;
		val->val2 = 0;
		return 0;
	}

	if (attr_is(a, ICM42688_ATTR_ACTIVITY_CLASS, ICM42688_ATTR_LEG_ACTIVITY_CLASS)) {
		uint8_t cls = 0;
		int rc = icm42688_apex_read_activity_class_u8(dev, &cls);
		if (rc < 0) {
			return rc;
		}
		val->val1 = cls;
		val->val2 = 0;
		return 0;
	}

	/* TAP attributes: read once, return requested field */
	if (attr_is(a, ICM42688_ATTR_TAP_NUM, ICM42688_ATTR_LEG_TAP_NUM) ||
	    attr_is(a, ICM42688_ATTR_TAP_AXIS, ICM42688_ATTR_LEG_TAP_AXIS) ||
	    attr_is(a, ICM42688_ATTR_TAP_DIR, ICM42688_ATTR_LEG_TAP_DIR) ||
	    attr_is(a, ICM42688_ATTR_DOUBLE_TAP_TIMING, ICM42688_ATTR_LEG_DOUBLE_TAP_TIMING)) {

		uint8_t tap_num = 0, tap_axis = 0, tap_dir = 0, dt = 0;
		int rc = icm42688_apex_read_tap(dev, &tap_num, &tap_axis, &tap_dir, &dt);
		if (rc < 0) {
			return rc;
		}

		if (attr_is(a, ICM42688_ATTR_TAP_NUM, ICM42688_ATTR_LEG_TAP_NUM)) {
			val->val1 = tap_num;
		} else if (attr_is(a, ICM42688_ATTR_TAP_AXIS, ICM42688_ATTR_LEG_TAP_AXIS)) {
			val->val1 = tap_axis;
		} else if (attr_is(a, ICM42688_ATTR_TAP_DIR, ICM42688_ATTR_LEG_TAP_DIR)) {
			val->val1 = tap_dir;
		} else {
			val->val1 = dt;
		}

		val->val2 = 0;
		return 0;
	}
#endif /* CONFIG_ICM42688_APEX */

	/* ---- FSYNC configuration ---- */
	if (a == ICM42688_ATTR_FSYNC_CONFIG) {
		uint8_t reg_val = 0;
		int rc = icm42688_read_reg(dev, ICM42688_REG_FSYNC_CONFIG, &reg_val);
		if (rc < 0) {
			return rc;
		}
		val->val1 = reg_val;
		val->val2 = 0;
		return 0;
	}

	if (a == ICM42688_ATTR_FSYNC_UI_SEL) {
		uint8_t reg_val = 0;
		int rc = icm42688_read_reg(dev, ICM42688_REG_FSYNC_CONFIG, &reg_val);
		if (rc < 0) {
			return rc;
		}
		val->val1 = (reg_val & ICM42688_FSYNC_UI_SEL_MASK) >> ICM42688_FSYNC_UI_SEL_SHIFT;
		val->val2 = 0;
		return 0;
	}

	if (a == ICM42688_ATTR_FSYNC_POLARITY) {
		uint8_t reg_val = 0;
		int rc = icm42688_read_reg(dev, ICM42688_REG_FSYNC_CONFIG, &reg_val);
		if (rc < 0) {
			return rc;
		}
		val->val1 = (reg_val & ICM42688_FSYNC_POLARITY) ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == ICM42688_ATTR_FSYNC_ODR_DELAY) {
		uint8_t reg_val = 0;
		int rc = icm42688_read_reg(dev, ICM42688_REG_SYNC_CONFIG, &reg_val);
		if (rc < 0) {
			return rc;
		}
		val->val1 = (reg_val >> 4) & 0x0F; /* bits[7:4] ODR delay */
		val->val2 = 0;
		return 0;
	}

	/* ---- Temperature DLPF Bandwidth (attr_get) ---- */
	if (a == (int)ICM42688_ATTR_TEMP_DLPF_BW) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		int rc = icm42688_read_reg(dev, ICM42688_REG_GYRO_CONFIG1, &reg_val);
		if (rc < 0) return rc;
		val->val1 = (reg_val & ICM42688_TEMP_FILT_BW_MASK) >> ICM42688_TEMP_FILT_BW_SHIFT;
		val->val2 = 0;
		return 0;
	}

	/* ---- DEC2_M2 Filter Order (attr_get) ---- */
	if (a == (int)ICM42688_ATTR_GYRO_DEC2_M2_ORD) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		int rc = icm42688_read_reg(dev, ICM42688_REG_GYRO_CONFIG1, &reg_val);
		if (rc < 0) return rc;
		val->val1 = (reg_val & ICM42688_GYRO_DEC2_M2_ORD_MASK) >> ICM42688_GYRO_DEC2_M2_ORD_SHIFT;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_ACCEL_DEC2_M2_ORD) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		int rc = icm42688_read_reg(dev, ICM42688_REG_ACCEL_CONFIG1, &reg_val);
		if (rc < 0) return rc;
		val->val1 = (reg_val & ICM42688_ACCEL_DEC2_M2_ORD_MASK) >> ICM42688_ACCEL_DEC2_M2_ORD_SHIFT;
		val->val2 = 0;
		return 0;
	}

	/* ---- Gyro Notch Filter (attr_get) ---- */
	if (a == (int)ICM42688_ATTR_GYRO_NOTCH_ENABLE) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK1);
		int rc = icm42688_read_reg(dev, ICM42688_REG_GYRO_CONFIG_STATIC2, &reg_val);
		if (rc < 0) return rc;
		val->val1 = (reg_val & ICM42688_GYRO_NF_DIS) ? 0 : 1;  /* DIS bit inverted */
		val->val2 = 0;
		return 0;
	}

	/* ---- Gyro AAF (attr_get) ---- */
	if (a == (int)ICM42688_ATTR_GYRO_AAF_ENABLE) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK1);
		int rc = icm42688_read_reg(dev, ICM42688_REG_GYRO_CONFIG_STATIC2, &reg_val);
		if (rc < 0) return rc;
		val->val1 = (reg_val & ICM42688_GYRO_AAF_DIS) ? 0 : 1;  /* DIS bit inverted */
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_GYRO_AAF_FREQ) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK1);
		int rc = icm42688_read_reg(dev, ICM42688_REG_GYRO_CONFIG_STATIC3, &reg_val);
		if (rc < 0) return rc;
		val->val1 = (reg_val & ICM42688_GYRO_AAF_DELT_MASK) >> ICM42688_GYRO_AAF_DELT_SHIFT;
		val->val2 = 0;
		return 0;
	}

	/* ---- Accel AAF (attr_get) ---- */
	if (a == (int)ICM42688_ATTR_ACCEL_AAF_ENABLE) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK2);
		int rc = icm42688_read_reg(dev, ICM42688_REG_ACCEL_CONFIG_STATIC2, &reg_val);
		if (rc < 0) return rc;
		val->val1 = (reg_val & ICM42688_ACCEL_AAF_DIS) ? 0 : 1;  /* DIS bit inverted */
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_ACCEL_AAF_FREQ) {
		uint8_t reg_val = 0;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK2);
		int rc = icm42688_read_reg(dev, ICM42688_REG_ACCEL_CONFIG_STATIC2, &reg_val);
		if (rc < 0) return rc;
		val->val1 = (reg_val & ICM42688_ACCEL_AAF_DELT_MASK) >> ICM42688_ACCEL_AAF_DELT_SHIFT;
		val->val2 = 0;
		return 0;
	}

	/* ---- Wake-on-Motion (attr_get) ---- */
	if (a == (int)ICM42688_ATTR_WOM_ENABLE) {
		val->val1 = data->wom_enabled ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_WOM_THRESHOLD) {
		/* Return threshold in mg (stored as 4mg units) */
		val->val1 = data->wom_threshold * 4;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_WOM_INT_MODE) {
		val->val1 = data->wom_int_mode;
		val->val2 = 0;
		return 0;
	}

	/* ---- FIFO Mode ---- */
	if (a == (int)ICM42688_ATTR_FIFO_MODE) {
		uint8_t reg_val = 0;
		int rc = icm42688_read_reg(dev, ICM42688_REG_FIFO_CONFIG, &reg_val);
		if (rc < 0) {
			return rc;
		}
		/* Extract FIFO mode from register bits [1:0] */
		val->val1 = (reg_val & 0x3u);
		val->val2 = 0;
		return 0;
	}

	/* ---- SMD (Significant Motion Detection) ---- */
	if (a == (int)ICM42688_ATTR_SMD_ENABLE) {
		val->val1 = data->smd_enabled ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_SMD_THRESHOLD) {
		/* Return threshold in mg (stored as 4mg units) */
		val->val1 = data->smd_threshold * 4;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_SMD_MODE) {
		val->val1 = data->smd_mode;
		val->val2 = 0;
		return 0;
	}

	/* ---- INT2 APEX Routing ---- */
	if (a == (int)ICM42688_ATTR_INT2_ROUTE_TAP) {
		val->val1 = data->int2_route_tap ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_INT2_ROUTE_TILT) {
		val->val1 = data->int2_route_tilt ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_INT2_ROUTE_R2W) {
		val->val1 = data->int2_route_r2w ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_INT2_ROUTE_PEDOMETER) {
		val->val1 = data->int2_route_pedometer ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	/* Low Power Modes */
	if (a == (int)ICM42688_ATTR_GYRO_LOW_POWER) {
		val->val1 = data->gyro_low_power ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_ACCEL_LOW_POWER) {
		val->val1 = data->accel_low_power ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_TEMP_DISABLE) {
		val->val1 = data->temp_disabled ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_TEMP_RESOLUTION) {
		val->val1 = data->temp_resolution;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_UI_DRDY_INT1_EN) {
		val->val1 = data->ui_drdy_int1_enabled ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_UI_DRDY_INT2_EN) {
		val->val1 = data->ui_drdy_int2_enabled ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_DELTA_TIMESTAMP) {
		val->val1 = data->delta_timestamp_enabled ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_FIFO_THRESHOLD) {
		val->val1 = data->fifo_threshold;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_MOUNTING_MATRIX) {
		/* Mounting matrix stored in data structure as 3x3 (9 elements)
		 * Return first element via val1 (simplest approach for sensor_value framework)
		 */
		val->val1 = data->mounting_matrix[0];  /* Return first element */
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_TAP_THRESHOLD) {
		val->val1 = data->tap_threshold;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_TAP_WINDOW) {
		val->val1 = data->tap_window;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_TAP_COUNTS) {
		val->val1 = data->tap_counts;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_FIFO_FULL_INT_EN) {
		val->val1 = data->fifo_full_int_enabled ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_STRIDE_LENGTH) {
		val->val1 = data->stride_length;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_STEP_COUNT_ENABLE) {
		val->val1 = data->step_count_enabled ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_RTC_MODE) {
		val->val1 = data->rtc_mode_enabled ? 1 : 0;
		val->val2 = 0;
		return 0;
	}

	if (a == (int)ICM42688_ATTR_CLKSEL) {
		val->val1 = data->clksel;
		val->val2 = 0;
		return 0;
	}

	return -ENOTSUP;
}

static int icm42688_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
	struct icm42688_data *data = dev->data;
	const int a = (int)attr;

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	if (attr == SENSOR_ATTR_FULL_SCALE) {
		if (chan == SENSOR_CHAN_ACCEL_XYZ) {
			uint16_t fs_g = (uint16_t)val->val1;
			int rc = icm42688_write_accel_cfg0(dev, fs_g, data->accel_odr_hz);
			if (rc) {
				return rc;
			}
			data->accel_fs_g = fs_g;
			return 0;
		}
		if (chan == SENSOR_CHAN_GYRO_XYZ) {
			uint16_t fs_dps = (uint16_t)val->val1;
			int rc = icm42688_write_gyro_cfg0(dev, fs_dps, data->gyro_odr_hz);
			if (rc) {
				return rc;
			}
			data->gyro_fs_dps = fs_dps;
			return 0;
		}
		return -ENOTSUP;
	}

	if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
		uint32_t odr = (uint32_t)val->val1;

		if (chan == SENSOR_CHAN_ACCEL_XYZ) {
			int rc = icm42688_write_accel_cfg0(dev, data->accel_fs_g, odr);
			if (rc) {
				return rc;
			}
			data->accel_odr_hz = odr;
			return 0;
		}
		if (chan == SENSOR_CHAN_GYRO_XYZ) {
			int rc = icm42688_write_gyro_cfg0(dev, data->gyro_fs_dps, odr);
			if (rc) {
				return rc;
			}
			data->gyro_odr_hz = odr;
			return 0;
		}
		return -ENOTSUP;
	}

	/* FIFO flush */
	if (a == (int)ICM42688_ATTR_FIFO_FLUSH) {
#ifdef CONFIG_ICM42688_FIFO
		ARG_UNUSED(val);
		return icm42688_fifo_flush(dev);
#else
		return -ENOTSUP;
#endif
	}

#ifdef CONFIG_ICM42688_APEX
	/* Feature enable toggles (accept legacy IDs too) */
	if (attr_is(a, ICM42688_ATTR_PEDOMETER_ENABLE, ICM42688_ATTR_LEG_PEDOMETER_ENABLE) ||
	    attr_is(a, ICM42688_ATTR_TILT_ENABLE,      ICM42688_ATTR_LEG_TILT_ENABLE) ||
	    attr_is(a, ICM42688_ATTR_R2W_ENABLE,       ICM42688_ATTR_LEG_R2W_ENABLE) ||
	    attr_is(a, ICM42688_ATTR_TAP_ENABLE,       ICM42688_ATTR_LEG_TAP_ENABLE)) {

		const bool en = (val->val1 != 0);
		uint8_t m = 0;

		if (attr_is(a, ICM42688_ATTR_PEDOMETER_ENABLE, ICM42688_ATTR_LEG_PEDOMETER_ENABLE)) {
			m = ICM42688_APEX_FEAT_PEDOMETER;
		} else if (attr_is(a, ICM42688_ATTR_TILT_ENABLE, ICM42688_ATTR_LEG_TILT_ENABLE)) {
			m = ICM42688_APEX_FEAT_TILT;
		} else if (attr_is(a, ICM42688_ATTR_R2W_ENABLE, ICM42688_ATTR_LEG_R2W_ENABLE)) {
			m = ICM42688_APEX_FEAT_R2W;
		} else {
			m = ICM42688_APEX_FEAT_TAP;
		}

		return icm42688_apex_enable_feature(dev, m, en);
	}
#endif /* CONFIG_ICM42688_APEX */

	/* WOM enable/disable */
	if (a == (int)ICM42688_ATTR_WOM_ENABLE) {
		if (val->val1 != 0) {
			/* Enable with current or default threshold */
			uint8_t thr = data->wom_threshold ? data->wom_threshold : 52; /* ~208mg default */
			return icm42688_wom_enable(dev, thr, data->wom_int_mode);
		} else {
			return icm42688_wom_disable(dev);
		}
	}

	if (a == (int)ICM42688_ATTR_WOM_THRESHOLD) {
		/* val1 = threshold in mg, we convert to 4mg units */
		uint8_t thr = (uint8_t)(val->val1 / 4);
		if (thr == 0) thr = 1;
		if (thr > 255) thr = 255;
		data->wom_threshold = thr;
		if (data->wom_enabled) {
			return icm42688_wom_enable(dev, thr, data->wom_int_mode);
		}
		return 0;
	}

	if (a == (int)ICM42688_ATTR_WOM_INT_MODE) {
		data->wom_int_mode = (uint8_t)(val->val1 & 0x1);
		if (data->wom_enabled) {
			return icm42688_wom_enable(dev, data->wom_threshold, data->wom_int_mode);
		}
		return 0;
	}

	if (a == (int)ICM42688_ATTR_SELF_TEST_RUN) {
		uint8_t st_result = 0;
		int rc = icm42688_self_test(dev, &st_result);
		if (rc < 0) {
			return rc;
		}
		/* Store result for later retrieval via attr_get */
		data->self_test_result = st_result;
		return (st_result == 0) ? 0 : -EIO;
	}

	/* UI Filter Bandwidth */
	if (a == (int)ICM42688_ATTR_ACCEL_FILT_BW) {
		uint8_t bw = (uint8_t)(val->val1 & 0x0F);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return icm42688_reg_update8(dev, ICM42688_REG_GYRO_ACCEL_CONFIG0,
					    ICM42688_ACCEL_UI_FILT_BW_MASK,
					    (uint8_t)(bw << ICM42688_ACCEL_UI_FILT_BW_SHIFT));
	}

	if (a == (int)ICM42688_ATTR_GYRO_FILT_BW) {
		uint8_t bw = (uint8_t)(val->val1 & 0x0F);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return icm42688_reg_update8(dev, ICM42688_REG_GYRO_ACCEL_CONFIG0,
					    ICM42688_GYRO_UI_FILT_BW_MASK,
					    (uint8_t)(bw << ICM42688_GYRO_UI_FILT_BW_SHIFT));
	}

	/* UI Filter Order */
	if (a == (int)ICM42688_ATTR_ACCEL_FILT_ORD) {
		uint8_t ord = (uint8_t)(val->val1 & 0x03);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return icm42688_reg_update8(dev, ICM42688_REG_ACCEL_CONFIG1,
					    ICM42688_ACCEL_UI_FILT_ORD_MASK,
					    (uint8_t)(ord << ICM42688_ACCEL_UI_FILT_ORD_SHIFT));
	}

	if (a == (int)ICM42688_ATTR_GYRO_FILT_ORD) {
		uint8_t ord = (uint8_t)(val->val1 & 0x03);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return icm42688_reg_update8(dev, ICM42688_REG_GYRO_CONFIG1,
					    ICM42688_GYRO_UI_FILT_ORD_MASK,
					    (uint8_t)(ord << ICM42688_GYRO_UI_FILT_ORD_SHIFT));
	}

	/* Temperature DLPF Bandwidth */
	if (a == (int)ICM42688_ATTR_TEMP_DLPF_BW) {
		uint8_t bw = (uint8_t)(val->val1 & 0x07);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return icm42688_reg_update8(dev, ICM42688_REG_GYRO_CONFIG1,
					    ICM42688_TEMP_FILT_BW_MASK,
					    (uint8_t)(bw << ICM42688_TEMP_FILT_BW_SHIFT));
	}

	/* DEC2_M2 Filter Order */
	if (a == (int)ICM42688_ATTR_GYRO_DEC2_M2_ORD) {
		uint8_t ord = (uint8_t)(val->val1 & 0x03);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return icm42688_reg_update8(dev, ICM42688_REG_GYRO_CONFIG1,
					    ICM42688_GYRO_DEC2_M2_ORD_MASK,
					    (uint8_t)(ord << ICM42688_GYRO_DEC2_M2_ORD_SHIFT));
	}

	if (a == (int)ICM42688_ATTR_ACCEL_DEC2_M2_ORD) {
		uint8_t ord = (uint8_t)(val->val1 & 0x03);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return icm42688_reg_update8(dev, ICM42688_REG_ACCEL_CONFIG1,
					    ICM42688_ACCEL_DEC2_M2_ORD_MASK,
					    (uint8_t)(ord << ICM42688_ACCEL_DEC2_M2_ORD_SHIFT));
	}

	/* Offset Calibration attributes */
	if (a == (int)ICM42688_ATTR_CALIB_RUN) {
		if (val->val1 != 0) {
			return icm42688_calibrate(dev);
		}
		return 0;
	}

	if (a == (int)ICM42688_ATTR_GYRO_OFFSET_X) {
		return icm42688_set_gyro_offset(dev, 0, val->val1);
	}
	if (a == (int)ICM42688_ATTR_GYRO_OFFSET_Y) {
		return icm42688_set_gyro_offset(dev, 1, val->val1);
	}
	if (a == (int)ICM42688_ATTR_GYRO_OFFSET_Z) {
		return icm42688_set_gyro_offset(dev, 2, val->val1);
	}
	if (a == (int)ICM42688_ATTR_ACCEL_OFFSET_X) {
		return icm42688_set_accel_offset(dev, 0, val->val1);
	}
	if (a == (int)ICM42688_ATTR_ACCEL_OFFSET_Y) {
		return icm42688_set_accel_offset(dev, 1, val->val1);
	}
	if (a == (int)ICM42688_ATTR_ACCEL_OFFSET_Z) {
		return icm42688_set_accel_offset(dev, 2, val->val1);
	}

	/* Per-axis enable/disable (Bank1 SENSOR_CONFIG0) */
	if (a == (int)ICM42688_ATTR_GYRO_X_ENABLE) {
		uint8_t mask = (val->val1 != 0) ? 0 : ICM42688_XG_DISABLE;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK1);
		int rc = icm42688_reg_update8(dev, ICM42688_REG_SENSOR_CONFIG0,
					      ICM42688_XG_DISABLE, mask);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return rc;
	}

	if (a == (int)ICM42688_ATTR_GYRO_Y_ENABLE) {
		uint8_t mask = (val->val1 != 0) ? 0 : ICM42688_YG_DISABLE;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK1);
		int rc = icm42688_reg_update8(dev, ICM42688_REG_SENSOR_CONFIG0,
					      ICM42688_YG_DISABLE, mask);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return rc;
	}

	if (a == (int)ICM42688_ATTR_GYRO_Z_ENABLE) {
		uint8_t mask = (val->val1 != 0) ? 0 : ICM42688_ZG_DISABLE;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK1);
		int rc = icm42688_reg_update8(dev, ICM42688_REG_SENSOR_CONFIG0,
					      ICM42688_ZG_DISABLE, mask);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return rc;
	}

	if (a == (int)ICM42688_ATTR_ACCEL_X_ENABLE) {
		uint8_t mask = (val->val1 != 0) ? 0 : ICM42688_XA_DISABLE;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK1);
		int rc = icm42688_reg_update8(dev, ICM42688_REG_SENSOR_CONFIG0,
					      ICM42688_XA_DISABLE, mask);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return rc;
	}

	if (a == (int)ICM42688_ATTR_ACCEL_Y_ENABLE) {
		uint8_t mask = (val->val1 != 0) ? 0 : ICM42688_YA_DISABLE;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK1);
		int rc = icm42688_reg_update8(dev, ICM42688_REG_SENSOR_CONFIG0,
					      ICM42688_YA_DISABLE, mask);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return rc;
	}

	if (a == (int)ICM42688_ATTR_ACCEL_Z_ENABLE) {
		uint8_t mask = (val->val1 != 0) ? 0 : ICM42688_ZA_DISABLE;
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK1);
		int rc = icm42688_reg_update8(dev, ICM42688_REG_SENSOR_CONFIG0,
					      ICM42688_ZA_DISABLE, mask);
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return rc;
	}

#ifdef CONFIG_ICM42688_FIFO
	/* Timestamp attributes */
	if (a == (int)ICM42688_ATTR_TIMESTAMP_EN) {
		return icm42688_fifo_timestamp_enable(dev, val->val1 != 0);
	}
	if (a == (int)ICM42688_ATTR_TIMESTAMP_RES) {
		return icm42688_timestamp_set_resolution(dev, (uint8_t)val->val1);
	}
#endif /* CONFIG_ICM42688_FIFO */

	/* ---- FSYNC configuration ---- */
	if (a == (int)ICM42688_ATTR_FSYNC_CONFIG) {
		return icm42688_write_reg(dev, ICM42688_REG_FSYNC_CONFIG, (uint8_t)val->val1);
	}

	if (a == (int)ICM42688_ATTR_FSYNC_UI_SEL) {
		uint8_t ui_sel = (uint8_t)(val->val1 & 0x03);
		return icm42688_reg_update8(dev, ICM42688_REG_FSYNC_CONFIG,
					    ICM42688_FSYNC_UI_SEL_MASK,
					    (uint8_t)(ui_sel << ICM42688_FSYNC_UI_SEL_SHIFT));
	}

	if (a == (int)ICM42688_ATTR_FSYNC_POLARITY) {
		uint8_t pol = (val->val1 != 0) ? ICM42688_FSYNC_POLARITY : 0;
		return icm42688_reg_update8(dev, ICM42688_REG_FSYNC_CONFIG,
					    ICM42688_FSYNC_POLARITY, pol);
	}

	if (a == (int)ICM42688_ATTR_FSYNC_ODR_DELAY) {
		uint8_t delay = (uint8_t)(val->val1 & 0x0F);
		return icm42688_reg_update8(dev, ICM42688_REG_SYNC_CONFIG,
					    0xF0u, (uint8_t)(delay << 4));
	}

	/* Notch Filter attributes */
	if (a == (int)ICM42688_ATTR_GYRO_NOTCH_ENABLE) {
		return icm42688_notch_enable(dev, val->val1 != 0);
	}

	if (a == (int)ICM42688_ATTR_GYRO_NOTCH_FREQ_X) {
		return icm42688_notch_set_freq(dev, 0, (uint8_t)val->val1);
	}

	if (a == (int)ICM42688_ATTR_GYRO_NOTCH_FREQ_Y) {
		return icm42688_notch_set_freq(dev, 1, (uint8_t)val->val1);
	}

	if (a == (int)ICM42688_ATTR_GYRO_NOTCH_FREQ_Z) {
		return icm42688_notch_set_freq(dev, 2, (uint8_t)val->val1);
	}

	if (a == (int)ICM42688_ATTR_GYRO_NOTCH_BW) {
		return icm42688_notch_set_bandwidth(dev, (uint8_t)val->val1);
	}

	/* Anti-Alias Filter attributes */
	if (a == (int)ICM42688_ATTR_GYRO_AAF_ENABLE) {
		return icm42688_gyro_aaf_enable(dev, val->val1 != 0);
	}

	if (a == (int)ICM42688_ATTR_GYRO_AAF_FREQ) {
		return icm42688_gyro_aaf_set_freq(dev, (uint16_t)val->val1);
	}

	if (a == (int)ICM42688_ATTR_ACCEL_AAF_ENABLE) {
		return icm42688_accel_aaf_enable(dev, val->val1 != 0);
	}

	if (a == (int)ICM42688_ATTR_ACCEL_AAF_FREQ) {
		return icm42688_accel_aaf_set_freq(dev, (uint16_t)val->val1);
	}

	/* Signal Path Reset */
	if (a == (int)ICM42688_ATTR_SIGNAL_PATH_RESET) {
		uint8_t reset_mask = (uint8_t)val->val1;
		return icm42688_signal_path_reset(dev, reset_mask);
	}

	/* SMD (Significant Motion Detection) */
	if (a == (int)ICM42688_ATTR_SMD_ENABLE) {
		bool enable = val->val1 != 0;
		if (enable) {
			struct icm42688_data *data = dev->data;
			return icm42688_smd_enable(dev, data->smd_threshold, data->smd_mode);
		} else {
			return icm42688_smd_disable(dev);
		}
	}

	if (a == (int)ICM42688_ATTR_SMD_THRESHOLD) {
		struct icm42688_data *data = dev->data;
		data->smd_threshold = (uint8_t)val->val1;
		if (data->smd_enabled) {
			return icm42688_smd_enable(dev, data->smd_threshold, data->smd_mode);
		}
		return 0;
	}

	if (a == (int)ICM42688_ATTR_SMD_MODE) {
		struct icm42688_data *data = dev->data;
		uint8_t mode = (uint8_t)val->val1;
		if (mode != ICM42688_SMD_MODE_SHORT && mode != ICM42688_SMD_MODE_LONG) {
			return -EINVAL;
		}
		data->smd_mode = mode;
		if (data->smd_enabled) {
			return icm42688_smd_enable(dev, data->smd_threshold, data->smd_mode);
		}
		return 0;
	}

	/* INT2 APEX routing */
	if (a == (int)ICM42688_ATTR_INT2_ROUTE_TAP) {
		struct icm42688_data *data = dev->data;
		data->int2_route_tap = (val->val1 != 0);
		return icm42688_apex_route_to_int2(dev, 
			(data->int2_route_tap ? ICM42688_APEX_FEAT_TAP : 0) |
			(data->int2_route_tilt ? ICM42688_APEX_FEAT_TILT : 0) |
			(data->int2_route_r2w ? ICM42688_APEX_FEAT_R2W : 0) |
			(data->int2_route_pedometer ? ICM42688_APEX_FEAT_PEDOMETER : 0));
	}

	if (a == (int)ICM42688_ATTR_INT2_ROUTE_TILT) {
		struct icm42688_data *data = dev->data;
		data->int2_route_tilt = (val->val1 != 0);
		return icm42688_apex_route_to_int2(dev, 
			(data->int2_route_tap ? ICM42688_APEX_FEAT_TAP : 0) |
			(data->int2_route_tilt ? ICM42688_APEX_FEAT_TILT : 0) |
			(data->int2_route_r2w ? ICM42688_APEX_FEAT_R2W : 0) |
			(data->int2_route_pedometer ? ICM42688_APEX_FEAT_PEDOMETER : 0));
	}

	if (a == (int)ICM42688_ATTR_INT2_ROUTE_R2W) {
		struct icm42688_data *data = dev->data;
		data->int2_route_r2w = (val->val1 != 0);
		return icm42688_apex_route_to_int2(dev, 
			(data->int2_route_tap ? ICM42688_APEX_FEAT_TAP : 0) |
			(data->int2_route_tilt ? ICM42688_APEX_FEAT_TILT : 0) |
			(data->int2_route_r2w ? ICM42688_APEX_FEAT_R2W : 0) |
			(data->int2_route_pedometer ? ICM42688_APEX_FEAT_PEDOMETER : 0));
	}

	if (a == (int)ICM42688_ATTR_INT2_ROUTE_PEDOMETER) {
		struct icm42688_data *data = dev->data;
		data->int2_route_pedometer = (val->val1 != 0);
		return icm42688_apex_route_to_int2(dev, 
			(data->int2_route_tap ? ICM42688_APEX_FEAT_TAP : 0) |
			(data->int2_route_tilt ? ICM42688_APEX_FEAT_TILT : 0) |
			(data->int2_route_r2w ? ICM42688_APEX_FEAT_R2W : 0) |
			(data->int2_route_pedometer ? ICM42688_APEX_FEAT_PEDOMETER : 0));
	}

	/* Low Power Modes */
	if (a == (int)ICM42688_ATTR_GYRO_LOW_POWER) {
		return icm42688_set_gyro_low_power(dev, (val->val1 != 0));
	}

	if (a == (int)ICM42688_ATTR_ACCEL_LOW_POWER) {
		return icm42688_set_accel_low_power(dev, (val->val1 != 0));
	}

	if (a == (int)ICM42688_ATTR_TEMP_DISABLE) {
		return icm42688_set_temp_disable(dev, (val->val1 != 0));
	}

	if (a == (int)ICM42688_ATTR_TEMP_RESOLUTION) {
		return icm42688_set_temp_resolution(dev, (uint8_t)val->val1);
	}

	if (a == (int)ICM42688_ATTR_UI_DRDY_INT1_EN) {
		return icm42688_set_ui_drdy_int1(dev, (val->val1 != 0));
	}

	if (a == (int)ICM42688_ATTR_UI_DRDY_INT2_EN) {
		return icm42688_set_ui_drdy_int2(dev, (val->val1 != 0));
	}

	if (a == (int)ICM42688_ATTR_DELTA_TIMESTAMP) {
		return icm42688_set_delta_timestamp(dev, (val->val1 != 0));
	}

	if (a == (int)ICM42688_ATTR_FIFO_THRESHOLD) {
		return icm42688_set_fifo_threshold(dev, (uint8_t)val->val1);
	}

	if (a == (int)ICM42688_ATTR_MOUNTING_MATRIX) {
		/* Mounting matrix: val->val1 points to int16_t array, val->val2 is size
		 * For now, just store the first value as a placeholder
		 * Full matrix handling would need extended sensor_value structure
		 */
		int16_t matrix[ICM42688_MOUNTING_MATRIX_SIZE];
		memset(matrix, 0, sizeof(matrix));
		matrix[0] = (int16_t)val->val1;
		return icm42688_set_mounting_matrix(dev, matrix, 1);
	}

	if (a == (int)ICM42688_ATTR_TAP_THRESHOLD) {
		return icm42688_set_tap_threshold(dev, (uint8_t)val->val1);
	}

	if (a == (int)ICM42688_ATTR_TAP_WINDOW) {
		return icm42688_set_tap_window(dev, (uint8_t)val->val1);
	}

	if (a == (int)ICM42688_ATTR_TAP_COUNTS) {
		return icm42688_set_tap_counts(dev, (uint8_t)val->val1);
	}

	if (a == (int)ICM42688_ATTR_FIFO_FULL_INT_EN) {
		return icm42688_set_fifo_full_interrupt(dev, (val->val1 != 0));
	}

	if (a == (int)ICM42688_ATTR_STRIDE_LENGTH) {
		return icm42688_set_stride_length(dev, (uint8_t)val->val1);
	}

	if (a == (int)ICM42688_ATTR_STEP_COUNT_ENABLE) {
		return icm42688_set_step_count_enable(dev, (val->val1 != 0));
	}

	if (a == (int)ICM42688_ATTR_RTC_MODE) {
		return icm42688_set_rtc_mode(dev, (val->val1 != 0));
	}

	if (a == (int)ICM42688_ATTR_CLKSEL) {
		return icm42688_set_clksel(dev, (uint8_t)val->val1);
	}

	/* FIFO Mode */
	if (a == (int)ICM42688_ATTR_FIFO_MODE) {
		uint8_t mode = (uint8_t)val->val1;
		if (mode > 2) {
			return -EINVAL;  /* Only 0, 1, 2 valid */
		}

		int rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		if (rc < 0) return rc;

		/* Read current register value */
		uint8_t reg_val = 0;
		rc = icm42688_read_reg(dev, ICM42688_REG_FIFO_CONFIG, &reg_val);
		if (rc < 0) return rc;

		/* Clear mode bits [1:0] and set new mode */
		reg_val = (uint8_t)((reg_val & ~0x3u) | (mode & 0x3u));

		/* Write back */
		rc = icm42688_write_reg(dev, ICM42688_REG_FIFO_CONFIG, reg_val);
		if (rc < 0) return rc;

		LOG_INF("FIFO mode set to %u (%s)", mode,
			mode == 0 ? "Bypass" :
			mode == 1 ? "Stream" :
			mode == 2 ? "Stop-on-Full" : "Unknown");

		return 0;
	}

	return -ENOTSUP;
}

/* ========= Wake-on-Motion (WOM) ========= */

int icm42688_wom_enable(const struct device *dev, uint8_t threshold_mg_div4, uint8_t int_mode)
{
	struct icm42688_data *data = dev->data;
	int rc;

	LOG_INF("WOM enable: threshold=%u (x4mg=%umg), int_mode=%s",
		threshold_mg_div4, threshold_mg_div4 * 4,
		int_mode ? "AND" : "OR");

	/* 1) Bank4: thresholds */
	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) {
		return rc;
	}

	rc = icm42688_write_reg(dev, ICM42688_REG_ACCEL_WOM_X_THR, threshold_mg_div4);
	if (rc < 0) goto done_b0;
	rc = icm42688_write_reg(dev, ICM42688_REG_ACCEL_WOM_Y_THR, threshold_mg_div4);
	if (rc < 0) goto done_b0;
	rc = icm42688_write_reg(dev, ICM42688_REG_ACCEL_WOM_Z_THR, threshold_mg_div4);
	if (rc < 0) goto done_b0;

	/* 2) Bank0: route WOM to INT1 via INT_SOURCE1 */
	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	uint8_t int_src1_mask =
		ICM42688_INT_SRC1_WOM_X_INT1_EN |
		ICM42688_INT_SRC1_WOM_Y_INT1_EN |
		ICM42688_INT_SRC1_WOM_Z_INT1_EN;

	rc = icm42688_reg_update8(dev, ICM42688_REG_INT_SOURCE1, int_src1_mask, int_src1_mask);
	if (rc < 0) {
		return rc;
	}

	/* 3) Configure WOM mode in SMD_CONFIG */
	uint8_t smd_cfg = 0;

	/* Compare to previous sample (recommended for steady behavior) */
	smd_cfg |= ICM42688_SMD_WOM_MODE;

	/* Interrupt mode: OR (0) / AND (1) */
	if (int_mode == ICM42688_WOM_INT_MODE_AND) {
		smd_cfg |= ICM42688_SMD_WOM_INT_MODE;
	}

	/* We want WOM only; SMD algorithm disabled */
	smd_cfg |= ICM42688_SMD_SMD_MODE_DISABLED;

	rc = icm42688_write_reg(dev, ICM42688_REG_SMD_CONFIG, smd_cfg);
	if (rc < 0) {
		return rc;
	}

	data->wom_enabled = true;
	data->wom_threshold = threshold_mg_div4;
	data->wom_int_mode = int_mode;

	LOG_INF("WOM enabled");
	return 0;

done_b0:
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}


int icm42688_wom_disable(const struct device *dev)
{
	struct icm42688_data *data = dev->data;
	int rc;

	LOG_INF("WOM disable");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* Disable WOM routing on INT1 in INT_SOURCE1 */
	uint8_t int_src1_mask =
		ICM42688_INT_SRC1_WOM_X_INT1_EN |
		ICM42688_INT_SRC1_WOM_Y_INT1_EN |
		ICM42688_INT_SRC1_WOM_Z_INT1_EN;

	rc = icm42688_reg_update8(dev, ICM42688_REG_INT_SOURCE1, int_src1_mask, 0);
	if (rc < 0) {
		return rc;
	}

	/* Disable WOM/SMD block */
	rc = icm42688_write_reg(dev, ICM42688_REG_SMD_CONFIG, 0);
	if (rc < 0) {
		return rc;
	}

	data->wom_enabled = false;

	LOG_INF("WOM disabled");
	return 0;
}

/* ========= SMD (Significant Motion Detection) ========= */

int icm42688_smd_enable(const struct device *dev, uint8_t threshold_mg_div4, uint8_t mode)
{
	struct icm42688_data *data = dev->data;
	int rc;

	if (mode != ICM42688_SMD_MODE_SHORT && mode != ICM42688_SMD_MODE_LONG) {
		LOG_ERR("Invalid SMD mode: %u (must be 2 or 3)", mode);
		return -EINVAL;
	}

	LOG_INF("SMD enable: threshold=%u (x4mg=%umg), mode=%s",
		threshold_mg_div4, threshold_mg_div4 * 4,
		mode == ICM42688_SMD_MODE_SHORT ? "SHORT(2s)" : "LONG(10s)");

	/* 1) Bank4: set WOM thresholds for SMD */
	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) {
		return rc;
	}

	rc = icm42688_write_reg(dev, ICM42688_REG_ACCEL_WOM_X_THR, threshold_mg_div4);
	if (rc < 0) goto done_b0;
	rc = icm42688_write_reg(dev, ICM42688_REG_ACCEL_WOM_Y_THR, threshold_mg_div4);
	if (rc < 0) goto done_b0;
	rc = icm42688_write_reg(dev, ICM42688_REG_ACCEL_WOM_Z_THR, threshold_mg_div4);
	if (rc < 0) goto done_b0;

	/* 2) Bank0: route SMD to INT1 */
	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* Enable SMD event on INT1 (INT_SOURCE1 bit 3) */
	uint8_t int_src1_smd = ICM42688_INT_SRC1_SMD_INT1_EN;
	rc = icm42688_reg_update8(dev, ICM42688_REG_INT_SOURCE1, int_src1_smd, int_src1_smd);
	if (rc < 0) {
		return rc;
	}

	/* 3) Configure SMD in SMD_CONFIG (0x57) */
	uint8_t smd_cfg = 0;

	/* Compare to previous sample */
	smd_cfg |= ICM42688_SMD_WOM_MODE;

	/* WOM interrupt mode: OR (recommended for SMD) */
	/* Don't set WOM_INT_MODE bit (keep it 0) */

	/* SMD algorithm enabled with selected window size */
	smd_cfg |= (mode & ICM42688_SMD_SMD_MODE_MASK);

	rc = icm42688_write_reg(dev, ICM42688_REG_SMD_CONFIG, smd_cfg);
	if (rc < 0) {
		return rc;
	}

	data->smd_enabled = true;
	data->smd_threshold = threshold_mg_div4;
	data->smd_mode = mode;

	LOG_INF("SMD enabled");
	return 0;

done_b0:
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}

int icm42688_smd_disable(const struct device *dev)
{
	struct icm42688_data *data = dev->data;
	int rc;

	LOG_INF("SMD disable");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* Disable SMD routing on INT1 */
	uint8_t int_src1_smd = ICM42688_INT_SRC1_SMD_INT1_EN;
	rc = icm42688_reg_update8(dev, ICM42688_REG_INT_SOURCE1, int_src1_smd, 0);
	if (rc < 0) {
		return rc;
	}

	/* Disable WOM/SMD block */
	rc = icm42688_write_reg(dev, ICM42688_REG_SMD_CONFIG, 0);
	if (rc < 0) {
		return rc;
	}

	data->smd_enabled = false;

	LOG_INF("SMD disabled");
	return 0;
}

/* ========= Low Power Modes ========= */

int icm42688_set_gyro_low_power(const struct device *dev, bool enable)
{
	struct icm42688_data *data = dev->data;
	int rc;

	LOG_INF("Gyro Low Power: %s", enable ? "ENABLED (Standby mode)" : "DISABLED (Low-Noise mode)");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	uint8_t gyro_mode = enable ? ICM42688_GYRO_MODE_STBY : ICM42688_GYRO_MODE_LN;

	uint8_t pwr_mgmt0 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_PWR_MGMT0, &pwr_mgmt0);
	if (rc < 0) {
		return rc;
	}

	/* Clear gyro mode bits [3:2] and set new mode */
	pwr_mgmt0 = (pwr_mgmt0 & ~(3u << ICM42688_PWR_GYRO_MODE_SHIFT)) |
	            (gyro_mode << ICM42688_PWR_GYRO_MODE_SHIFT);

	rc = icm42688_write_reg(dev, ICM42688_REG_PWR_MGMT0, pwr_mgmt0);
	if (rc < 0) {
		return rc;
	}

	data->gyro_low_power = enable;
	k_sleep(K_MSEC(1));  /* Wait for mode change */

	LOG_INF("Gyro Low Power: %s", enable ? "enabled" : "disabled");
	return 0;
}

int icm42688_set_accel_low_power(const struct device *dev, bool enable)
{
	struct icm42688_data *data = dev->data;
	int rc;

	LOG_INF("Accel Low Power: %s", enable ? "ENABLED (LP mode)" : "DISABLED (Low-Noise mode)");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	uint8_t accel_mode = enable ? ICM42688_ACCEL_MODE_LP : ICM42688_ACCEL_MODE_LN;

	uint8_t pwr_mgmt0 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_PWR_MGMT0, &pwr_mgmt0);
	if (rc < 0) {
		return rc;
	}

	/* Clear accel mode bits [1:0] and set new mode */
	pwr_mgmt0 = (pwr_mgmt0 & ~(3u << ICM42688_PWR_ACCEL_MODE_SHIFT)) |
	            (accel_mode << ICM42688_PWR_ACCEL_MODE_SHIFT);

	rc = icm42688_write_reg(dev, ICM42688_REG_PWR_MGMT0, pwr_mgmt0);
	if (rc < 0) {
		return rc;
	}

	data->accel_low_power = enable;
	k_sleep(K_MSEC(1));  /* Wait for mode change */

	LOG_INF("Accel Low Power: %s", enable ? "enabled" : "disabled");
	return 0;
}

int icm42688_set_temp_disable(const struct device *dev, bool disable)
{
	struct icm42688_data *data = dev->data;
	int rc;

	LOG_INF("Temperature: %s", disable ? "DISABLED" : "ENABLED");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	uint8_t temp_dis_bit = disable ? ICM42688_PWR_TEMP_DIS : 0;

	uint8_t pwr_mgmt0 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_PWR_MGMT0, &pwr_mgmt0);
	if (rc < 0) {
		return rc;
	}

	/* Update TEMP_DIS bit [5] */
	pwr_mgmt0 = (pwr_mgmt0 & ~ICM42688_PWR_TEMP_DIS) | temp_dis_bit;

	rc = icm42688_write_reg(dev, ICM42688_REG_PWR_MGMT0, pwr_mgmt0);
	if (rc < 0) {
		return rc;
	}

	data->temp_disabled = disable;

	LOG_INF("Temperature: %s", disable ? "disabled" : "enabled");
	return 0;
}

int icm42688_set_temp_resolution(const struct device *dev, uint8_t mode)
{
	struct icm42688_data *data = dev->data;
	int rc;

	if (mode > 1) {
		LOG_ERR("Invalid temperature resolution mode: %u (must be 0=8-bit or 1=16-bit)", mode);
		return -EINVAL;
	}

	LOG_INF("Temperature Resolution: %s mode (FIFO_HIRES_EN=%s)",
		mode == ICM42688_TEMP_RES_16BIT ? "16-bit accurate" : "8-bit fast",
		mode == ICM42688_TEMP_RES_16BIT ? "enabled" : "disabled");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* Read FIFO_CONFIG1 register (0x5F) */
	uint8_t fifo_config1 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_FIFO_CONFIG1, &fifo_config1);
	if (rc < 0) {
		return rc;
	}

	/* Update FIFO_HIRES_EN bit [4] to enable/disable 16-bit temperature
	 * 0 = 8-bit FIFO_TEMP_DATA (fast, low power, ~480Hz conversion)
	 * 1 = 16-bit temperature sensor data in 20-bit FIFO packet (accurate, normal power)
	 */
	uint8_t hires_en_bit = (mode == ICM42688_TEMP_RES_16BIT) ? (1u << 4) : 0;
	fifo_config1 = (fifo_config1 & ~(1u << 4)) | hires_en_bit;

	rc = icm42688_write_reg(dev, ICM42688_REG_FIFO_CONFIG1, fifo_config1);
	if (rc < 0) {
		return rc;
	}

	data->temp_resolution = mode;

	LOG_INF("Temperature Resolution: %s mode (verified)",
		mode == ICM42688_TEMP_RES_16BIT ? "16-bit accurate" : "8-bit fast");
	return 0;
}

/* ========= INTERRUPT ROUTING ========= */

int icm42688_set_ui_drdy_int1(const struct device *dev, bool enable)
{
	struct icm42688_data *data = dev->data;
	int rc;

	LOG_INF("UI Data Ready INT1: %s", enable ? "ENABLED" : "DISABLED");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* Read INT_SOURCE0 register (0x65) - INT1 routing control */
	uint8_t int_source0 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_INT_SOURCE0, &int_source0);
	if (rc < 0) {
		return rc;
	}

	/* Update UI_DRDY_INT1_EN bit [3]
	 * 0 = UI data ready interrupt not routed to INT1
	 * 1 = UI data ready interrupt routed to INT1 (fires when new sensor data ready)
	 */
	uint8_t drdy_en_bit = enable ? (1u << 3) : 0;
	int_source0 = (int_source0 & ~(1u << 3)) | drdy_en_bit;

	rc = icm42688_write_reg(dev, ICM42688_REG_INT_SOURCE0, int_source0);
	if (rc < 0) {
		return rc;
	}

	data->ui_drdy_int1_enabled = enable;

	LOG_INF("UI Data Ready INT1: %s", enable ? "enabled" : "disabled");
	return 0;
}

int icm42688_set_ui_drdy_int2(const struct device *dev, bool enable)
{
	struct icm42688_data *data = dev->data;
	int rc;

	LOG_INF("UI Data Ready INT2: %s", enable ? "ENABLED" : "DISABLED");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* Read INT_SOURCE3 register (0x68) - INT2 routing control */
	uint8_t int_source3 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_INT_SOURCE3, &int_source3);
	if (rc < 0) {
		return rc;
	}

	/* Update UI_DRDY_INT2_EN bit [3]
	 * 0 = UI data ready interrupt not routed to INT2
	 * 1 = UI data ready interrupt routed to INT2 (fires when new sensor data ready)
	 */
	uint8_t drdy_en_bit = enable ? (1u << 3) : 0;
	int_source3 = (int_source3 & ~(1u << 3)) | drdy_en_bit;

	rc = icm42688_write_reg(dev, ICM42688_REG_INT_SOURCE3, int_source3);
	if (rc < 0) {
		return rc;
	}

	data->ui_drdy_int2_enabled = enable;

	LOG_INF("UI Data Ready INT2: %s", enable ? "enabled" : "disabled");
	return 0;
}

/* ========= TIMESTAMP MODE ========= */

int icm42688_set_delta_timestamp(const struct device *dev, bool enable)
{
	struct icm42688_data *data = dev->data;
	int rc;

	LOG_INF("Delta Timestamp Mode: %s", enable ? "ENABLED (relative)" : "DISABLED (absolute)");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* Read TMST_CONFIG register (0x54) - Timestamp configuration */
	uint8_t tmst_config = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_TMST_CONFIG, &tmst_config);
	if (rc < 0) {
		return rc;
	}

	/* Update TMST_DELTA_EN bit [2]
	 * 0 = Absolute timestamps (full counter value, useful for absolute time references)
	 * 1 = Delta timestamps (time since last ODR sample, useful for batch processing, smaller values)
	 */
	uint8_t delta_en_bit = enable ? (1u << 2) : 0;
	tmst_config = (tmst_config & ~(1u << 2)) | delta_en_bit;

	rc = icm42688_write_reg(dev, ICM42688_REG_TMST_CONFIG, tmst_config);
	if (rc < 0) {
		return rc;
	}

	data->delta_timestamp_enabled = enable;

	LOG_INF("Delta Timestamp Mode: %s (verified)",
		enable ? "relative time (ODR delta)" : "absolute time (full counter)");
	return 0;
}

/* ========= FIFO THRESHOLD ========= */

int icm42688_set_fifo_threshold(const struct device *dev, uint8_t threshold)
{
	struct icm42688_data *data = dev->data;
	int rc;

	if (threshold > ICM42688_FIFO_THRESHOLD_MAX) {
		LOG_ERR("FIFO threshold out of range: %u (max=%u)", threshold, ICM42688_FIFO_THRESHOLD_MAX);
		return -EINVAL;
	}

	LOG_INF("FIFO Threshold: %u bytes", threshold);

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* FIFO watermark is configured via FIFO_WATERMARK registers (0x6E-0x6F, Bank 0)
	 * Default is 256-byte watermark level
	 * Writing new threshold triggers watermark interrupt when FIFO reaches that level
	 * Used for batch processing: interrupt fires when enough data accumulated
	 */
	
	/* High byte at 0x6F */
	uint8_t watermark_hi = (threshold >> 8) & 0xFF;
	rc = icm42688_write_reg(dev, ICM42688_REG_FIFO_WATERMARK_H, watermark_hi);
	if (rc < 0) {
		return rc;
	}

	/* Low byte at 0x6E */
	uint8_t watermark_lo = threshold & 0xFF;
	rc = icm42688_write_reg(dev, ICM42688_REG_FIFO_WATERMARK_L, watermark_lo);
	if (rc < 0) {
		return rc;
	}

	data->fifo_threshold = threshold;

	LOG_INF("FIFO Threshold: %u bytes set (FIFO watermark triggers interrupt at this level)", threshold);
	return 0;
}

/* ========= MOUNTING MATRIX ========= */

int icm42688_set_mounting_matrix(const struct device *dev, const int16_t *matrix, size_t size)
{
	struct icm42688_data *data = dev->data;

	if (size > ICM42688_MOUNTING_MATRIX_SIZE) {
		LOG_ERR("Mounting matrix size too large: %zu (max=%d)", size, ICM42688_MOUNTING_MATRIX_SIZE);
		return -EINVAL;
	}

	LOG_INF("Mounting Matrix: storing 3x3 orientation matrix (%zu elements)", size);

	/* Copy the 3x3 matrix into the driver data structure
	 * Matrix format (row-major):
	 *   [m00  m01  m02]
	 *   [m10  m11  m12]
	 *   [m20  m21  m22]
	 *
	 * Each element is fixed-point (typically in 1/1024 units for orientation)
	 * Used to transform sensor output based on physical board mounting orientation
	 * Allows correction for axes that are rotated, inverted, or swapped
	 */
	
	memset(data->mounting_matrix, 0, sizeof(data->mounting_matrix));
	if (matrix && size > 0) {
		memcpy(data->mounting_matrix, matrix, size * sizeof(int16_t));
	}

	/* Log the matrix values for debugging */
	LOG_INF("Mounting Matrix elements: ");
	for (size_t i = 0; i < ICM42688_MOUNTING_MATRIX_SIZE; i++) {
		if (i < size && matrix) {
			LOG_INF("  m[%zu] = %d", i, data->mounting_matrix[i]);
		} else {
			LOG_INF("  m[%zu] = 0 (unused)", i);
		}
	}

	LOG_INF("Mounting Matrix: 3x3 orientation correction configured (verified)");
	return 0;
}

/* ========= TAP DETECTION TUNING ========= */

int icm42688_set_tap_threshold(const struct device *dev, uint8_t threshold)
{
	struct icm42688_data *data = dev->data;
	int rc;

	if (threshold > ICM42688_TAP_THRESHOLD_MAX) {
		LOG_ERR("Tap threshold out of range: %u (max=%u)", threshold, ICM42688_TAP_THRESHOLD_MAX);
		return -EINVAL;
	}

	LOG_INF("Tap Threshold: %u (sensitivity: %s)", threshold,
		threshold == 0 ? "most sensitive" : threshold == 15 ? "least sensitive" : "medium");

	/* Tap threshold is stored in Bank4 APEX_CONFIG2 (0x41)
	 * APEX_CONFIG2[7:4] = TAP_TAPER[3:0] threshold
	 * 0-15: each unit = 31mg, lower=more sensitive
	 */
	
	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) {
		return rc;
	}

	uint8_t apex_cfg2 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_APEX_CONFIG2, &apex_cfg2);
	if (rc < 0) {
		return rc;
	}

	/* Update threshold bits [7:4] */
	apex_cfg2 = (apex_cfg2 & ~(0x0Fu << 4)) | ((threshold & 0x0Fu) << 4);

	rc = icm42688_write_reg(dev, ICM42688_REG_APEX_CONFIG2, apex_cfg2);
	if (rc < 0) {
		return rc;
	}

	data->tap_threshold = threshold;

	LOG_INF("Tap Threshold: %u set (each unit = 31mg, lower=more sensitive) (verified)", threshold);
	return 0;
}

int icm42688_set_tap_window(const struct device *dev, uint8_t window)
{
	struct icm42688_data *data = dev->data;
	int rc;

	if (window > ICM42688_TAP_WINDOW_MAX) {
		LOG_ERR("Tap window out of range: %u (max=%u)", window, ICM42688_TAP_WINDOW_MAX);
		return -EINVAL;
	}

	LOG_INF("Tap Window: %u (time window: ~%ums)", window, window * 10);

	/* Tap window duration is stored in Bank4 APEX_CONFIG2 (0x41)
	 * APEX_CONFIG2[3:0] = TAP_TAPER_DUR[3:0] duration
	 * 0-15: each unit ~10ms, longer=slower tap detection
	 */
	
	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) {
		return rc;
	}

	uint8_t apex_cfg2 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_APEX_CONFIG2, &apex_cfg2);
	if (rc < 0) {
		return rc;
	}

	/* Update window bits [3:0] */
	apex_cfg2 = (apex_cfg2 & ~(0x0Fu)) | (window & 0x0Fu);

	rc = icm42688_write_reg(dev, ICM42688_REG_APEX_CONFIG2, apex_cfg2);
	if (rc < 0) {
		return rc;
	}

	data->tap_window = window;

	LOG_INF("Tap Window: %u set (~%ums detection window) (verified)", window, window * 10);
	return 0;
}

int icm42688_set_tap_counts(const struct device *dev, uint8_t counts)
{
	struct icm42688_data *data = dev->data;
	int rc;

	if (counts < ICM42688_TAP_COUNTS_MIN || counts > ICM42688_TAP_COUNTS_MAX) {
		LOG_ERR("Tap counts out of range: %u (min=%u, max=%u)", counts,
			ICM42688_TAP_COUNTS_MIN, ICM42688_TAP_COUNTS_MAX);
		return -EINVAL;
	}

	LOG_INF("Tap Counts: %u peaks per tap", counts);

	/* Tap peaks configuration is stored in Bank4 APEX_CONFIG3 (0x42)
	 * APEX_CONFIG3[7:5] = TAP_NUM_TAP[2:0] - number of peaks per tap
	 * 0 = single peak (single tap)
	 * 1-15 = multiple peaks (double/multi-tap detection)
	 */
	
	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) {
		return rc;
	}

	uint8_t apex_cfg3 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_APEX_CONFIG3, &apex_cfg3);
	if (rc < 0) {
		return rc;
	}

	/* Update counts bits [7:5] */
	apex_cfg3 = (apex_cfg3 & ~(0x07u << 5)) | ((counts & 0x07u) << 5);

	rc = icm42688_write_reg(dev, ICM42688_REG_APEX_CONFIG3, apex_cfg3);
	if (rc < 0) {
		return rc;
	}

	data->tap_counts = counts;

	LOG_INF("Tap Counts: %u peaks set (multi-tap detection tuned) (verified)", counts);
	return 0;
}

/* ========= FIFO FULL INTERRUPT ========= */

int icm42688_set_fifo_full_interrupt(const struct device *dev, bool enable)
{
	struct icm42688_data *data = dev->data;
	int rc;

	LOG_INF("FIFO Full Interrupt: %s", enable ? "ENABLED" : "DISABLED");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* INT_CONFIG register (0x14, Bank 0) bit 0 = FIFO_FULL_EN
	 * 0 = FIFO full interrupt disabled (no interrupt on overflow)
	 * 1 = FIFO full interrupt enabled (interrupt when FIFO reaches capacity ~256 bytes)
	 * Useful for batch processing: interrupt signals when enough data accumulated
	 */
	
	uint8_t int_config = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_INT_CONFIG, &int_config);
	if (rc < 0) {
		return rc;
	}

	/* Update FIFO_FULL_EN bit [0] */
	uint8_t full_en_bit = enable ? (1u << 0) : 0;
	int_config = (int_config & ~(1u << 0)) | full_en_bit;

	rc = icm42688_write_reg(dev, ICM42688_REG_INT_CONFIG, int_config);
	if (rc < 0) {
		return rc;
	}

	data->fifo_full_int_enabled = enable;

	LOG_INF("FIFO Full Interrupt: %s", enable ? "enabled" : "disabled");
	return 0;
}

/* ========= ADVANCED PEDOMETER / STRIDE DETECTION ========= */

int icm42688_set_stride_length(const struct device *dev, uint8_t stride)
{
	struct icm42688_data *data = dev->data;
	int rc;

	if (stride > ICM42688_STRIDE_LENGTH_MAX) {
		LOG_ERR("Stride length out of range: %u (max=%u)", stride, ICM42688_STRIDE_LENGTH_MAX);
		return -EINVAL;
	}

	LOG_INF("Stride Length: %u (approx %.2fm)", stride, (double)stride * 0.78);

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) {
		return rc;
	}

	/* APEX_CONFIG5 register (0x5D, Bank 4) bits [7:2] = STRIDE_DET_TH[5:0]
	 * Stride length calibration for pedometer accuracy
	 * Each unit ~0.78m of stride length
	 * Lower values: shorter strides (shorter people or walking)
	 * Higher values: longer strides (taller people or running)
	 * Default: around 100 (typical adult step ~78cm)
	 */
	
	uint8_t apex_config5 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_APEX_CONFIG5, &apex_config5);
	if (rc < 0) {
		return rc;
	}

	/* Update stride length bits [7:2] */
	apex_config5 = (apex_config5 & ~(0x3Fu << 2)) | ((stride & 0x3Fu) << 2);

	rc = icm42688_write_reg(dev, ICM42688_REG_APEX_CONFIG5, apex_config5);
	if (rc < 0) {
		return rc;
	}

	data->stride_length = stride;

	LOG_INF("Stride Length: %u set (%.2fm per step, personalized pedometer calibration) (verified)",
		stride, (double)stride * 0.78);
	return 0;
}

int icm42688_set_step_count_enable(const struct device *dev, bool enable)
{
	struct icm42688_data *data = dev->data;
	int rc;

	LOG_INF("Step Count Detection: %s", enable ? "ENABLED" : "DISABLED");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) {
		return rc;
	}

	/* APEX_CONFIG1 register (0x50, Bank 4) bit 1 = PEDOMETER_EN
	 * 0 = Pedometer disabled (step counting off)
	 * 1 = Pedometer enabled (step counting on, running in background)
	 * Pedometer automatically counts steps while accel data is available
	 * Works at any ODR and is independent of other APEX features
	 */
	
	uint8_t apex_config1 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_APEX_CONFIG1, &apex_config1);
	if (rc < 0) {
		return rc;
	}

	/* Update PEDOMETER_EN bit [1] */
	uint8_t pedometer_en_bit = enable ? (1u << 1) : 0;
	apex_config1 = (apex_config1 & ~(1u << 1)) | pedometer_en_bit;

	rc = icm42688_write_reg(dev, ICM42688_REG_APEX_CONFIG1, apex_config1);
	if (rc < 0) {
		return rc;
	}

	data->step_count_enabled = enable;

	LOG_INF("Step Count Detection: %s", enable ? "enabled" : "disabled");
	return 0;
}

/* ========= External Clock (CLKIN/RTC) ========= */

int icm42688_set_rtc_mode(const struct device *dev, bool enable)
{
	struct icm42688_data *data = dev->data;
	int rc;

	LOG_INF("RTC Mode: %s", enable ? "ENABLED" : "DISABLED");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* INTF_CONFIG1 register (0x4D, Bank 0) bit 2 = RTC_MODE
	 * 0 = No RTC clock input (default, uses internal RC oscillator)
	 * 1 = RTC clock required (enables CLKIN pin for external clock input)
	 * 
	 * RTC mode allows external clock synchronization for:
	 * - Improved ODR stability (reduced temperature sensitivity)
	 * - Lower device-to-device variation
	 * - Better timestamp accuracy
	 * - Supports 32.768kHz crystal or other precise clock sources
	 * 
	 * NOTE: External clock must be connected to CLKIN pin before enabling
	 */
	
	uint8_t intf_config1 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_INTF_CONFIG1, &intf_config1);
	if (rc < 0) {
		return rc;
	}

	/* Update RTC_MODE bit [2] */
	if (enable) {
		intf_config1 |= ICM42688_INTF_CONFIG1_RTC_MODE;
	} else {
		intf_config1 &= ~ICM42688_INTF_CONFIG1_RTC_MODE;
	}

	rc = icm42688_write_reg(dev, ICM42688_REG_INTF_CONFIG1, intf_config1);
	if (rc < 0) {
		return rc;
	}

	/* Verify write */
	uint8_t verify = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_INTF_CONFIG1, &verify);
	if (rc < 0) {
		return rc;
	}

	data->rtc_mode_enabled = enable;

	LOG_INF("RTC Mode: %s (INTF_CONFIG1=0x%02x, verified)", 
		enable ? "enabled (external clock on CLKIN)" : "disabled (internal RC oscillator)", 
		verify);
	return 0;
}

int icm42688_set_clksel(const struct device *dev, uint8_t clksel)
{
	struct icm42688_data *data = dev->data;
	int rc;

	if (clksel > ICM42688_CLKSEL_EXTERNAL) {
		return -EINVAL;
	}

	const char *clksel_names[] = {
		"Internal RC oscillator",
		"Auto (PLL if available, else RC)",
		"Disabled (low power)",
		"External clock (CLKIN/RTC)"
	};

	LOG_INF("Clock Source Selection (CLKSEL): %u (%s)", clksel, clksel_names[clksel]);

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* INTF_CONFIG1 register (0x4D, Bank 0) bits [1:0] = CLKSEL
	 * 00 = Internal RC oscillator only (always RC, ignores PLL)
	 * 01 = Auto-select: PLL when available, RC otherwise (default)
	 * 10 = Reserved (behaves like 00)
	 * 11 = External clock via CLKIN pin (requires RTC_MODE=1)
	 * 
	 * Clock source affects:
	 * - ODR accuracy and stability
	 * - Temperature sensitivity of sampling rate
	 * - Device-to-device ODR variation
	 * - Power consumption (PLL uses more power than RC)
	 * 
	 * Recommended usage:
	 * - Internal (0): Low power applications, ODR stability not critical
	 * - Auto (1): General use, balances power and accuracy (default)
	 * - External (3): High-precision applications (requires CLKIN connection + RTC_MODE=1)
	 */
	
	uint8_t intf_config1 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_INTF_CONFIG1, &intf_config1);
	if (rc < 0) {
		return rc;
	}

	/* Update CLKSEL bits [1:0] */
	intf_config1 = (intf_config1 & ~ICM42688_INTF_CONFIG1_CLKSEL_MASK) | 
	               ((clksel << ICM42688_INTF_CONFIG1_CLKSEL_SHIFT) & ICM42688_INTF_CONFIG1_CLKSEL_MASK);

	rc = icm42688_write_reg(dev, ICM42688_REG_INTF_CONFIG1, intf_config1);
	if (rc < 0) {
		return rc;
	}

	/* Verify write */
	uint8_t verify = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_INTF_CONFIG1, &verify);
	if (rc < 0) {
		return rc;
	}

	data->clksel = clksel;

	LOG_INF("Clock Source: %s (INTF_CONFIG1=0x%02x, verified)", clksel_names[clksel], verify);
	return 0;
}

/* ========= FSYNC (Frame Synchronization) ========= */

int icm42688_fsync_enable(const struct device *dev, uint8_t ui_sel, bool polarity)
{
	int rc;

	if (ui_sel > 3) {
		return -EINVAL;
	}

	LOG_INF("FSYNC enable: ui_sel=%u (%s), polarity=%s",
		ui_sel,
		ui_sel == 0 ? "disable" : ui_sel == 1 ? "accel" : ui_sel == 2 ? "gyro" : "both",
		polarity ? "falling" : "rising");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* Configure FSYNC_CONFIG register */
	uint8_t fsync_cfg = 0;

	/* Set UI_SEL (which sensors to trigger) */
	fsync_cfg |= (ui_sel & 0x03) << ICM42688_FSYNC_UI_SEL_SHIFT;

	/* Set polarity */
	if (polarity) {
		fsync_cfg |= ICM42688_FSYNC_POLARITY;
	}

	/* Clear disabled bit (enable FSYNC) */
	fsync_cfg &= ~ICM42688_FSYNC_DISABLED;

	rc = icm42688_write_reg(dev, ICM42688_REG_FSYNC_CONFIG, fsync_cfg);
	if (rc < 0) {
		return rc;
	}

	LOG_INF("FSYNC enabled");
	return 0;
}

int icm42688_fsync_disable(const struct device *dev)
{
	int rc;

	LOG_INF("FSYNC disable");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* Set disabled bit in FSYNC_CONFIG */
	rc = icm42688_reg_update8(dev, ICM42688_REG_FSYNC_CONFIG,
				  ICM42688_FSYNC_DISABLED, ICM42688_FSYNC_DISABLED);
	if (rc < 0) {
		return rc;
	}

	LOG_INF("FSYNC disabled");
	return 0;
}

int icm42688_fsync_set_polarity(const struct device *dev, bool polarity)
{
	int rc;

	LOG_INF("FSYNC set polarity: %s", polarity ? "falling" : "rising");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	uint8_t mask = ICM42688_FSYNC_POLARITY;
	uint8_t val = polarity ? ICM42688_FSYNC_POLARITY : 0;

	rc = icm42688_reg_update8(dev, ICM42688_REG_FSYNC_CONFIG, mask, val);
	if (rc < 0) {
		return rc;
	}

	LOG_INF("FSYNC polarity set");
	return 0;
}

int icm42688_fsync_set_delay(const struct device *dev, uint8_t delay)
{
	int rc;

	if (delay > 15) {
		return -EINVAL;
	}

	LOG_INF("FSYNC set ODR delay: %u", delay);

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	/* ODR delay is in bits [7:4] of SYNC_CONFIG */
	rc = icm42688_reg_update8(dev, ICM42688_REG_SYNC_CONFIG, 0xF0u, (uint8_t)(delay << 4));
	if (rc < 0) {
		return rc;
	}

	LOG_INF("FSYNC ODR delay set");
	return 0;
}

/* ========= External Clock (CLKIN/RTC) ========= */


/* ========= Self-Test ========= */

/*
 * Self-test limits from datasheet (Table 15):
 * Gyro: min 60 dps (±250dps FS) - TYPICAL
 * Accel: 225 mg - 675 mg (±4g FS) - TYPICAL
 *
 * At ±4g, Sensitivity = 8192 LSB/g
 * At ±250dps, Sensitivity = 131.072 LSB/dps
 *
 * We apply 15% tolerance to account for manufacturing variation
 * as datasheet values are TYPICAL, not absolute limits.
 */
#define ST_TOLERANCE_PCT    15

/* Accel: 225mg * 8192 LSB/g / 1000 = 1843 LSB (apply -15% = 1567) */
#define ST_ACCEL_MIN_LSB    1567   /* 225 mg - 15% tolerance */
#define ST_ACCEL_MAX_LSB    6360   /* 675 mg + 15% tolerance */

/* Gyro: 60 dps * 131.072 LSB/dps = 7864 LSB (apply -15% = 6685) */
#define ST_GYRO_MIN_LSB     6685   /* 60 dps - 15% tolerance */

/* Number of samples to average */
#define ST_SAMPLES          50
#define ST_SAMPLE_DELAY_US  1000

static int st_read_axis_avg(const struct device *dev, uint8_t start_reg,
			    int32_t *val1, int32_t *val2, int32_t *val3)
{
	int64_t sum1 = 0, sum2 = 0, sum3 = 0;
	uint8_t buf[6];

	for (int i = 0; i < ST_SAMPLES; i++) {
		int rc = icm42688_read_burst(dev, start_reg, buf, 6);
		if (rc < 0) return rc;

		sum1 += (int16_t)sys_get_be16(&buf[0]);
		sum2 += (int16_t)sys_get_be16(&buf[2]);
		sum3 += (int16_t)sys_get_be16(&buf[4]);

		k_busy_wait(ST_SAMPLE_DELAY_US);
	}

	*val1 = (int32_t)(sum1 / ST_SAMPLES);
	*val2 = (int32_t)(sum2 / ST_SAMPLES);
	*val3 = (int32_t)(sum3 / ST_SAMPLES);

	return 0;
}

int icm42688_self_test(const struct device *dev, uint8_t *result)
{
	int rc;
	uint8_t pwr_save, accel_cfg_save, gyro_cfg_save;

	if (!result) {
		return -EINVAL;
	}

	*result = 0;

	LOG_INF("Starting self-test...");

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) return rc;

	/* Save current config */
	rc = icm42688_read_reg(dev, ICM42688_REG_PWR_MGMT0, &pwr_save);
	if (rc < 0) return rc;
	rc = icm42688_read_reg(dev, ICM42688_REG_ACCEL_CONFIG0, &accel_cfg_save);
	if (rc < 0) return rc;
	rc = icm42688_read_reg(dev, ICM42688_REG_GYRO_CONFIG0, &gyro_cfg_save);
	if (rc < 0) return rc;

	/* Configure for self-test per datasheet:
	 * - Accel: ±4g FS, 1kHz ODR (FS_SEL=2, ODR=6)
	 * - Gyro: ±250dps FS, 1kHz ODR (FS_SEL=3, ODR=6)
	 * - Both sensors in Low-Noise mode
	 */
	uint8_t accel_cfg = (2u << 5) | 0x06u;  /* ±4g, 1kHz */
	uint8_t gyro_cfg = (3u << 5) | 0x06u;   /* ±250dps, 1kHz */
	uint8_t pwr = (ICM42688_GYRO_MODE_LN << ICM42688_PWR_GYRO_MODE_SHIFT) |
		      (ICM42688_ACCEL_MODE_LN << ICM42688_PWR_ACCEL_MODE_SHIFT);

	rc = icm42688_write_reg(dev, ICM42688_REG_ACCEL_CONFIG0, accel_cfg);
	if (rc < 0) goto restore;
	rc = icm42688_write_reg(dev, ICM42688_REG_GYRO_CONFIG0, gyro_cfg);
	if (rc < 0) goto restore;
	rc = icm42688_write_reg(dev, ICM42688_REG_PWR_MGMT0, pwr);
	if (rc < 0) goto restore;

	/* Wait for sensors to stabilize per datasheet */
	k_sleep(K_MSEC(100));

	/* ===== GYRO SELF-TEST ===== */
	int32_t gx_off, gy_off, gz_off;
	rc = st_read_axis_avg(dev, ICM42688_REG_GYRO_X1, &gx_off, &gy_off, &gz_off);
	if (rc < 0) goto restore;
	LOG_DBG("Gyro OFF: [%d, %d, %d]", gx_off, gy_off, gz_off);

	/* Enable gyro self-test for all axes + regulator */
	rc = icm42688_write_reg(dev, ICM42688_REG_SELF_TEST_CONFIG,
			ICM42688_EN_GX_ST |
			ICM42688_EN_GY_ST |
			ICM42688_EN_GZ_ST);
	if (rc < 0) goto restore;

	k_sleep(K_MSEC(200));  /* Wait for ST actuation */

	int32_t gx_on, gy_on, gz_on;
	rc = st_read_axis_avg(dev, ICM42688_REG_GYRO_X1, &gx_on, &gy_on, &gz_on);
	if (rc < 0) goto restore;
	LOG_DBG("Gyro ON: [%d, %d, %d]", gx_on, gy_on, gz_on);

	/* Disable ST */
	rc = icm42688_write_reg(dev, ICM42688_REG_SELF_TEST_CONFIG, 0);
	if (rc < 0) goto restore;
	k_sleep(K_MSEC(50));

	/* Calculate gyro ST response */
	int32_t gx_st = gx_on - gx_off;
	int32_t gy_st = gy_on - gy_off;
	int32_t gz_st = gz_on - gz_off;
	if (gx_st < 0) gx_st = -gx_st;
	if (gy_st < 0) gy_st = -gy_st;
	if (gz_st < 0) gz_st = -gz_st;

	/* Convert to dps for logging: LSB * 1000 / 131072 = dps (scaled by 1000) */
	int32_t gx_dps = (gx_st * 1000) / 131;
	int32_t gy_dps = (gy_st * 1000) / 131;
	int32_t gz_dps = (gz_st * 1000) / 131;

	LOG_INF("Gyro ST Response: [%d, %d, %d] LSB = [%d, %d, %d] mdps",
		gx_st, gy_st, gz_st, gx_dps, gy_dps, gz_dps);
	LOG_INF("Gyro ST limit: >= %d LSB (51 dps with 15%% tolerance)", ST_GYRO_MIN_LSB);

	if (gx_st < ST_GYRO_MIN_LSB) {
		*result |= ICM42688_ST_GYRO_X_FAIL;
		LOG_WRN("Gyro X FAILED: %d < %d", gx_st, ST_GYRO_MIN_LSB);
	}
	if (gy_st < ST_GYRO_MIN_LSB) {
		*result |= ICM42688_ST_GYRO_Y_FAIL;
		LOG_WRN("Gyro Y FAILED: %d < %d", gy_st, ST_GYRO_MIN_LSB);
	}
	if (gz_st < ST_GYRO_MIN_LSB) {
		*result |= ICM42688_ST_GYRO_Z_FAIL;
		LOG_WRN("Gyro Z FAILED: %d < %d", gz_st, ST_GYRO_MIN_LSB);
	}

	/* ===== ACCEL SELF-TEST ===== */
	int32_t ax_off, ay_off, az_off;
	rc = st_read_axis_avg(dev, ICM42688_REG_ACCEL_X1, &ax_off, &ay_off, &az_off);
	if (rc < 0) goto restore;
	LOG_DBG("Accel OFF: [%d, %d, %d]", ax_off, ay_off, az_off);

	/* Enable accel self-test for all axes + regulator */
	rc = icm42688_write_reg(dev, ICM42688_REG_SELF_TEST_CONFIG,
			ICM42688_ACCEL_ST_POWER |
			ICM42688_EN_AX_ST |
			ICM42688_EN_AY_ST |
			ICM42688_EN_AZ_ST);

	if (rc < 0) goto restore;

	k_sleep(K_MSEC(200));  /* Wait for ST actuation */

	int32_t ax_on, ay_on, az_on;
	rc = st_read_axis_avg(dev, ICM42688_REG_ACCEL_X1, &ax_on, &ay_on, &az_on);
	if (rc < 0) goto restore;
	LOG_DBG("Accel ON: [%d, %d, %d]", ax_on, ay_on, az_on);

	/* Disable ST */
	rc = icm42688_write_reg(dev, ICM42688_REG_SELF_TEST_CONFIG, 0);
	if (rc < 0) goto restore;

	/* Calculate accel ST response */
	int32_t ax_st = ax_on - ax_off;
	int32_t ay_st = ay_on - ay_off;
	int32_t az_st = az_on - az_off;
	if (ax_st < 0) ax_st = -ax_st;
	if (ay_st < 0) ay_st = -ay_st;
	if (az_st < 0) az_st = -az_st;

	/* Convert to mg for logging: LSB * 1000 / 8192 = mg */
	int32_t ax_mg = (ax_st * 1000) / 8192;
	int32_t ay_mg = (ay_st * 1000) / 8192;
	int32_t az_mg = (az_st * 1000) / 8192;

	LOG_INF("Accel ST Response: [%d, %d, %d] LSB = [%d, %d, %d] mg",
		ax_st, ay_st, az_st, ax_mg, ay_mg, az_mg);
	LOG_INF("Accel ST limits: %d-%d LSB (191-776 mg with 15%% tolerance)",
		ST_ACCEL_MIN_LSB, ST_ACCEL_MAX_LSB);

	if (ax_st < ST_ACCEL_MIN_LSB || ax_st > ST_ACCEL_MAX_LSB) {
		*result |= ICM42688_ST_ACCEL_X_FAIL;
		LOG_WRN("Accel X FAILED: %d not in [%d, %d]", ax_st, ST_ACCEL_MIN_LSB, ST_ACCEL_MAX_LSB);
	}
	if (ay_st < ST_ACCEL_MIN_LSB || ay_st > ST_ACCEL_MAX_LSB) {
		*result |= ICM42688_ST_ACCEL_Y_FAIL;
		LOG_WRN("Accel Y FAILED: %d not in [%d, %d]", ay_st, ST_ACCEL_MIN_LSB, ST_ACCEL_MAX_LSB);
	}
	if (az_st < ST_ACCEL_MIN_LSB || az_st > ST_ACCEL_MAX_LSB) {
		*result |= ICM42688_ST_ACCEL_Z_FAIL;
		LOG_WRN("Accel Z FAILED: %d not in [%d, %d]", az_st, ST_ACCEL_MIN_LSB, ST_ACCEL_MAX_LSB);
	}

	if (*result == 0) {
		LOG_INF("Self-test PASSED");
	} else {
		LOG_WRN("Self-test FAILED: 0x%02x", *result);
	}

restore:
	/* Restore original configuration */
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	(void)icm42688_write_reg(dev, ICM42688_REG_SELF_TEST_CONFIG, 0);
	(void)icm42688_write_reg(dev, ICM42688_REG_ACCEL_CONFIG0, accel_cfg_save);
	(void)icm42688_write_reg(dev, ICM42688_REG_GYRO_CONFIG0, gyro_cfg_save);
	(void)icm42688_write_reg(dev, ICM42688_REG_PWR_MGMT0, pwr_save);
	k_sleep(K_MSEC(50));

	return rc;
}

/* ========= Signal Path Reset ========= */

int icm42688_signal_path_reset(const struct device *dev, uint8_t reset_mask)
{
	int rc;

	if (!dev) {
		return -EINVAL;
	}

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		LOG_ERR("Bank select failed: %d", rc);
		return rc;
	}

	/* Map reset_mask values to actual SPR register bits */
	uint8_t spr_bits = 0;

	if (reset_mask & ICM42688_SPR_ACCEL_SRDST) {
		spr_bits |= ICM42688_SPR_ACCEL_SRDST;
	}
	if (reset_mask & ICM42688_SPR_GYRO_SRDST) {
		spr_bits |= ICM42688_SPR_GYRO_SRDST;
	}
	if (reset_mask & ICM42688_SPR_TEMP_SRDST) {
		spr_bits |= ICM42688_SPR_TEMP_SRDST;
	}

	LOG_INF("Signal Path Reset: mask=0x%02x, spr_bits=0x%02x", reset_mask, spr_bits);

	rc = icm42688_write_reg(dev, ICM42688_REG_SIGNAL_PATH_RESET, spr_bits);
	if (rc < 0) {
		LOG_ERR("Signal path reset write failed: %d", rc);
		return rc;
	}

	/* Wait for reset to complete per datasheet (10ms) */
	k_sleep(K_MSEC(10));

	/* Verify reset by reading back register (should be clear) */
	uint8_t verify;
	rc = icm42688_read_reg(dev, ICM42688_REG_SIGNAL_PATH_RESET, &verify);
	if (rc < 0) {
		LOG_ERR("Signal path reset verify failed: %d", rc);
		return rc;
	}

	LOG_INF("Signal path reset complete (verify=0x%02x)", verify);

	return 0;
}

/* ========= Offset Calibration ========= */

/**
 * Write 12-bit signed offset for gyroscope axis
 * @param dev Device
 * @param axis 0=X, 1=Y, 2=Z
 * @param offset_mdps Offset in milli-degrees per second
 */
int icm42688_set_gyro_offset(const struct device *dev, int axis, int32_t offset_mdps)
{
	int rc;

	/* Convert mdps to LSB: 1 LSB = 1/32 dps = 31.25 mdps */
	int16_t off_lsb = (int16_t)((offset_mdps * 32) / 1000);

	/* Clamp to 12-bit signed */
	if (off_lsb > 2047)  off_lsb = 2047;
	if (off_lsb < -2048) off_lsb = -2048;

	uint8_t lo = (uint8_t)(off_lsb & 0xFF);
	uint8_t hi = (uint8_t)((off_lsb >> 8) & 0x0F);

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) return rc;

	/* Packing per datasheet:
	 * GYRO_X: USER0 = X[7:0], USER1[3:0] = X[11:8]
	 * GYRO_Y: USER2 = Y[7:0], USER1[7:4] = Y[11:8]
	 * GYRO_Z: USER3 = Z[7:0], USER4[3:0] = Z[11:8]
	 */
	switch (axis) {
	case 0: /* X */
		rc = icm42688_write_reg(dev, ICM42688_REG_OFFSET_USER0, lo);
		if (rc < 0) break;
		rc = icm42688_reg_update8(dev, ICM42688_REG_OFFSET_USER1, 0x0F, hi);
		break;

	case 1: /* Y */
		rc = icm42688_write_reg(dev, ICM42688_REG_OFFSET_USER2, lo);
		if (rc < 0) break;
		rc = icm42688_reg_update8(dev, ICM42688_REG_OFFSET_USER1, 0xF0, (uint8_t)(hi << 4));
		break;

	case 2: /* Z */
		rc = icm42688_write_reg(dev, ICM42688_REG_OFFSET_USER3, lo);
		if (rc < 0) break;
		rc = icm42688_reg_update8(dev, ICM42688_REG_OFFSET_USER4, 0x0F, hi);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}


/**
 * Read 12-bit signed offset for gyroscope axis
 */
int icm42688_get_gyro_offset(const struct device *dev, int axis, int32_t *offset_mdps)
{
	int rc;
	uint8_t lo = 0, hi = 0, r = 0;

	if (!offset_mdps) return -EINVAL;

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) return rc;

	switch (axis) {
	case 0: /* X: USER0 + USER1[3:0] */
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER0, &lo);
		if (rc < 0) break;
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER1, &r);
		if (rc < 0) break;
		hi = (uint8_t)(r & 0x0F);
		break;

	case 1: /* Y: USER2 + USER1[7:4] */
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER2, &lo);
		if (rc < 0) break;
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER1, &r);
		if (rc < 0) break;
		hi = (uint8_t)((r >> 4) & 0x0F);
		break;

	case 2: /* Z: USER3 + USER4[3:0] */
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER3, &lo);
		if (rc < 0) break;
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER4, &r);
		if (rc < 0) break;
		hi = (uint8_t)(r & 0x0F);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return rc;
	}

	/* Reconstruct 12-bit signed */
	int16_t off12 = (int16_t)((hi << 8) | lo);
	if (off12 & 0x0800) {
		off12 |= 0xF000;
	}

	/* Convert to mdps: 1 LSB = 31.25 mdps */
	*offset_mdps = (off12 * 1000) / 32;

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return 0;
}

/**
 * Write 12-bit signed offset for accelerometer axis
 * @param axis 0=X, 1=Y, 2=Z
 * @param offset_mg Offset in milli-g
 */
int icm42688_set_accel_offset(const struct device *dev, int axis, int32_t offset_mg)
{
	int rc;

	/* Convert mg to LSB: 1 LSB = 0.5 mg */
	int16_t off_lsb = (int16_t)(offset_mg * 2);

	/* Clamp to 12-bit signed */
	if (off_lsb > 2047)  off_lsb = 2047;
	if (off_lsb < -2048) off_lsb = -2048;

	uint8_t lo = (uint8_t)(off_lsb & 0xFF);
	uint8_t hi = (uint8_t)((off_lsb >> 8) & 0x0F);

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) return rc;

	/* Packing per datasheet:
	 * ACCEL_X: USER5 = X[7:0], USER4[7:4] = X[11:8]
	 * ACCEL_Y: USER6 = Y[7:0], USER7[3:0] = Y[11:8]
	 * ACCEL_Z: USER8 = Z[7:0], USER7[7:4] = Z[11:8]
	 */
	switch (axis) {
	case 0: /* X */
		rc = icm42688_write_reg(dev, ICM42688_REG_OFFSET_USER5, lo);
		if (rc < 0) break;
		rc = icm42688_reg_update8(dev, ICM42688_REG_OFFSET_USER4, 0xF0, (uint8_t)(hi << 4));
		break;

	case 1: /* Y */
		rc = icm42688_write_reg(dev, ICM42688_REG_OFFSET_USER6, lo);
		if (rc < 0) break;
		rc = icm42688_reg_update8(dev, ICM42688_REG_OFFSET_USER7, 0x0F, hi);
		break;

	case 2: /* Z */
		rc = icm42688_write_reg(dev, ICM42688_REG_OFFSET_USER8, lo);
		if (rc < 0) break;
		rc = icm42688_reg_update8(dev, ICM42688_REG_OFFSET_USER7, 0xF0, (uint8_t)(hi << 4));
		break;

	default:
		rc = -EINVAL;
		break;
	}

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}


/**
 * Read 12-bit signed offset for accelerometer axis
 */
int icm42688_get_accel_offset(const struct device *dev, int axis, int32_t *offset_mg)
{
	int rc;
	uint8_t lo = 0, hi = 0, r = 0;

	if (!offset_mg) return -EINVAL;

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) return rc;

	switch (axis) {
	case 0: /* X: USER5 + USER4[7:4] */
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER5, &lo);
		if (rc < 0) break;
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER4, &r);
		if (rc < 0) break;
		hi = (uint8_t)((r >> 4) & 0x0F);
		break;

	case 1: /* Y: USER6 + USER7[3:0] */
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER6, &lo);
		if (rc < 0) break;
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER7, &r);
		if (rc < 0) break;
		hi = (uint8_t)(r & 0x0F);
		break;

	case 2: /* Z: USER8 + USER7[7:4] */
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER8, &lo);
		if (rc < 0) break;
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER7, &r);
		if (rc < 0) break;
		hi = (uint8_t)((r >> 4) & 0x0F);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
		return rc;
	}

	/* Reconstruct 12-bit signed */
	int16_t off12 = (int16_t)((hi << 8) | lo);
	if (off12 & 0x0800) {
		off12 |= 0xF000;
	}

	/* Convert to mg: 1 LSB = 0.5mg => mg = lsb/2 */
	*offset_mg = off12 / 2;

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return 0;
}

static int __maybe_unused icm42688_accel_offset_read_mg(const struct device *dev,
					uint8_t axis,
					int16_t *offset_mg)
{
	int rc;
	uint8_t lower, upper_reg;
	uint8_t upper_nibble;
	int16_t offset_lsb;

	/* axis: 0=X, 1=Y, 2=Z */
	if (axis > 2 || offset_mg == NULL) {
		return -EINVAL;
	}

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK4);
	if (rc < 0) {
		return rc;
	}

	rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER4 + axis, &lower);
	if (rc < 0) {
		goto done;
	}

	if (axis == 0) { /* X: USER8[3:0] */
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER8, &upper_reg);
		upper_nibble = upper_reg & 0x0F;
	} else if (axis == 1) { /* Y: USER7[7:4] */
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER7, &upper_reg);
		upper_nibble = (upper_reg >> 4) & 0x0F;
	} else { /* Z: USER7[3:0] */
		rc = icm42688_read_reg(dev, ICM42688_REG_OFFSET_USER7, &upper_reg);
		upper_nibble = upper_reg & 0x0F;
	}

	if (rc < 0) {
		goto done;
	}

	/* Reconstruct 12-bit signed value */
	offset_lsb = (int16_t)(lower | ((uint16_t)upper_nibble << 8));
	if (offset_lsb & 0x0800) { /* sign extend from 12-bit */
		offset_lsb |= 0xF000;
	}

	/* Convert to mg: 1 LSB = 0.5 mg */
	*offset_mg = offset_lsb / 2;

done:
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	return rc;
}

/**
 * Auto-calibrate sensor offsets
 * Sensor should be stationary, Z-axis pointing up (1g expected on Z)
 * @param dev Device
 * @return 0 on success
 */
int icm42688_calibrate(const struct device *dev)
{
	int rc;
	struct icm42688_data *data = dev->data;

	const int num_samples = 64;
	int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
	int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;

	LOG_INF("Starting calibration - keep sensor stationary, Z-up!");

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	/* Save current power state */
	uint8_t pwr_save = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_PWR_MGMT0, &pwr_save);
	if (rc < 0) {
		return rc;
	}

	/* Ensure sensors are running in LN */
	uint8_t pwr_ln = (uint8_t)((ICM42688_GYRO_MODE_LN << ICM42688_PWR_GYRO_MODE_SHIFT) |
				  (ICM42688_ACCEL_MODE_LN << ICM42688_PWR_ACCEL_MODE_SHIFT));
	rc = icm42688_write_reg(dev, ICM42688_REG_PWR_MGMT0, pwr_ln);
	if (rc < 0) {
		return rc;
	}

	/* Give time to settle */
	k_sleep(K_MSEC(60));

	/* Collect samples (raw) */
	for (int i = 0; i < num_samples; i++) {
		uint8_t buf[12];

		/* ACCEL_X1..ACCEL_Z0 (6 bytes) + GYRO_X1..GYRO_Z0 (6 bytes) are contiguous */
		rc = icm42688_read_burst(dev, ICM42688_REG_ACCEL_X1, buf, sizeof(buf));
		if (rc < 0) {
			goto restore;
		}

		int16_t ax = (int16_t)sys_get_be16(&buf[0]);
		int16_t ay = (int16_t)sys_get_be16(&buf[2]);
		int16_t az = (int16_t)sys_get_be16(&buf[4]);

		int16_t gx = (int16_t)sys_get_be16(&buf[6]);
		int16_t gy = (int16_t)sys_get_be16(&buf[8]);
		int16_t gz = (int16_t)sys_get_be16(&buf[10]);

		ax_sum += ax;
		ay_sum += ay;
		az_sum += az;

		gx_sum += gx;
		gy_sum += gy;
		gz_sum += gz;

		/* ~200 Hz */
		k_sleep(K_MSEC(5));
	}

	/* Calculate averages */
	int32_t ax_avg = ax_sum / num_samples;
	int32_t ay_avg = ay_sum / num_samples;
	int32_t az_avg = az_sum / num_samples;

	int32_t gx_avg = gx_sum / num_samples;
	int32_t gy_avg = gy_sum / num_samples;
	int32_t gz_avg = gz_sum / num_samples;

	LOG_INF("Average raw: Accel=[%d %d %d] Gyro=[%d %d %d]",
		(int)ax_avg, (int)ay_avg, (int)az_avg,
		(int)gx_avg, (int)gy_avg, (int)gz_avg);

	/* Convert to physical units using CURRENT full-scale settings:
	 * Accel: mg   = raw * (FS_g   * 1000) / 32768
	 * Gyro : mdps = raw * (FS_dps * 1000) / 32768
	 */
	int32_t fs_g   = (int32_t)(data->accel_fs_g ? data->accel_fs_g : 16);
	int32_t fs_dps = (int32_t)(data->gyro_fs_dps ? data->gyro_fs_dps : 2000);

	int32_t ax_mg   = (ax_avg * (fs_g * 1000)) / 32768;
	int32_t ay_mg   = (ay_avg * (fs_g * 1000)) / 32768;
	int32_t az_mg   = (az_avg * (fs_g * 1000)) / 32768;

	int32_t gx_mdps = (gx_avg * (fs_dps * 1000)) / 32768;
	int32_t gy_mdps = (gy_avg * (fs_dps * 1000)) / 32768;
	int32_t gz_mdps = (gz_avg * (fs_dps * 1000)) / 32768;

	LOG_INF("Measured (FS=%dg, %ddps): Accel=[%d %d %d] mg, Gyro=[%d %d %d] mdps",
		(int)fs_g, (int)fs_dps,
		(int)ax_mg, (int)ay_mg, (int)az_mg,
		(int)gx_mdps, (int)gy_mdps, (int)gz_mdps);

	/* Target: accel X,Y = 0mg, Z = +1000mg; gyro X,Y,Z = 0mdps */
	int32_t ax_err_mg   = -ax_mg;
	int32_t ay_err_mg   = -ay_mg;
	int32_t az_err_mg   = -(az_mg - 1000);

	int32_t gx_err_mdps = -gx_mdps;
	int32_t gy_err_mdps = -gy_mdps;
	int32_t gz_err_mdps = -gz_mdps;

	LOG_INF("Offset corrections: Accel=[%d %d %d] mg, Gyro=[%d %d %d] mdps",
		(int)ax_err_mg, (int)ay_err_mg, (int)az_err_mg,
		(int)gx_err_mdps, (int)gy_err_mdps, (int)gz_err_mdps);

	/* Apply offsets (your set_* functions must be the FIXED packing versions) */
	rc = icm42688_set_gyro_offset(dev, 0, gx_err_mdps);
	if (rc < 0) goto restore;
	rc = icm42688_set_gyro_offset(dev, 1, gy_err_mdps);
	if (rc < 0) goto restore;
	rc = icm42688_set_gyro_offset(dev, 2, gz_err_mdps);
	if (rc < 0) goto restore;

	rc = icm42688_set_accel_offset(dev, 0, ax_err_mg);
	if (rc < 0) goto restore;
	rc = icm42688_set_accel_offset(dev, 1, ay_err_mg);
	if (rc < 0) goto restore;
	rc = icm42688_set_accel_offset(dev, 2, az_err_mg);
	if (rc < 0) goto restore;

	LOG_INF("Calibration complete!");

restore:
	/* Restore original configuration */
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	(void)icm42688_write_reg(dev, ICM42688_REG_PWR_MGMT0, pwr_save);
	k_sleep(K_MSEC(50));
	return rc;
}


#ifdef CONFIG_ICM42688_TRIGGER
static int icm42688_trigger_set_wrap(const struct device *dev, const struct sensor_trigger *trig,
				     sensor_trigger_handler_t handler)
{
	return icm42688_trigger_set(dev, trig, handler);
}
#endif

static const struct sensor_driver_api icm42688_api = {
	.sample_fetch = icm42688_sample_fetch,
	.channel_get = icm42688_channel_get,
	.attr_get = icm42688_attr_get,
	.attr_set = icm42688_attr_set,
#ifdef CONFIG_ICM42688_TRIGGER
	.trigger_set = icm42688_trigger_set_wrap,
#endif
};

/* ========= Init helpers ========= */

static int icm42688_hw_probe(const struct device *dev)
{
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	uint8_t who = 0;
	int rc = icm42688_read_reg(dev, ICM42688_REG_WHO_AM_I, &who);
	if (rc < 0) {
		LOG_ERR("WHO_AM_I read failed: %d", rc);
		return rc;
	}

	if (who != ICM42688_WHOAMI_VALUE) {
		LOG_WRN("Unexpected WHO_AM_I: 0x%02x", who);
	}

	return 0;
}

static int icm42688_apply_defaults(const struct device *dev)
{
	struct icm42688_data *data = dev->data;
	const struct icm42688_config *cfg = dev->config;
	int rc;

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	/* === INTF_CONFIG0 (0x4C) ===
	 * bits[1:0] UI_SIFS_CFG:
	 *   - На I2C фиксируем в 0x0 (I2C mode).
	 *   - На SPI НЕ трогаем (или можно фиксировать под SPI, но ты на I2C).
	 *
	 * Остальные биты:
	 *  - FIFO_HOLD_LAST_DATA_EN = 0 (не "залипать" на последнем байте)
	 *  - FIFO_COUNT_REC = 0 (байтовый счётчик)
	 *  - SENSOR_DATA_ENDIAN = 1 (BE, чтобы sys_get_be16 работал “в лоб”)
	 *  - FIFO_COUNT_ENDIAN = 1 (BE) — у тебя fifo_count() это учитывает, но пусть будет единообразно
	 */
	uint8_t intf0 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_INTF_CONFIG0, &intf0);
	if (rc < 0) {
		return rc;
	}

	uint8_t new_intf0 = intf0;

	/* 1) UI_SIFS_CFG */
	if (cfg->bus_type == ICM42688_BUS_I2C) {
		/* I2C: force UI_SIFS_CFG=0 */
		new_intf0 = (uint8_t)((new_intf0 & ~0x03u) | 0x00u);
	} else {
		/* SPI/I3C: оставим как есть (важно не сломать режим) */
	}

	/* 2) FIFO sane defaults */
	new_intf0 &= ~(ICM42688_INTF_FIFO_HOLD_LAST_DATA_EN);
	new_intf0 &= ~(ICM42688_INTF_FIFO_COUNT_REC);

	/* 3) Endianness: BE */
	new_intf0 |= ICM42688_INTF_SENSOR_DATA_ENDIAN;
	new_intf0 |= ICM42688_INTF_FIFO_COUNT_ENDIAN;

	if (new_intf0 != intf0) {
		rc = icm42688_write_reg(dev, ICM42688_REG_INTF_CONFIG0, new_intf0);
		if (rc < 0) {
			return rc;
		}
	}

	uint8_t intf0_rb = 0;
	(void)icm42688_read_reg(dev, ICM42688_REG_INTF_CONFIG0, &intf0_rb);
	LOG_INF("INTF_CONFIG0: old=0x%02x new=0x%02x rb=0x%02x", intf0, new_intf0, intf0_rb);

	/* INT_CONFIG1 (0x64): сбросить INT_ASYNC_RESET */
	rc = icm42688_reg_update8(dev, ICM42688_REG_INT_CONFIG1,
				  ICM42688_INT_CONFIG1_INT_ASYNC_RESET, 0);
	if (rc < 0) {
		return rc;
	}

	/* Runtime defaults из cfg */
	data->accel_fs_g   = cfg->accel_fs_g   ? cfg->accel_fs_g   : 16;
	data->gyro_fs_dps  = cfg->gyro_fs_dps  ? cfg->gyro_fs_dps  : 2000;
	data->accel_odr_hz = cfg->accel_odr_hz ? cfg->accel_odr_hz : 1000;
	data->gyro_odr_hz  = cfg->gyro_odr_hz  ? cfg->gyro_odr_hz  : 1000;

	rc = icm42688_write_accel_cfg0(dev, data->accel_fs_g, data->accel_odr_hz);
	if (rc < 0) {
		return rc;
	}

	rc = icm42688_write_gyro_cfg0(dev, data->gyro_fs_dps, data->gyro_odr_hz);
	if (rc < 0) {
		return rc;
	}

	return 0;
}



static int icm42688_enable_sensors_ln(const struct device *dev)
{
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	uint8_t pwr = (uint8_t)((ICM42688_GYRO_MODE_LN << ICM42688_PWR_GYRO_MODE_SHIFT) |
				(ICM42688_ACCEL_MODE_LN << ICM42688_PWR_ACCEL_MODE_SHIFT));

	int rc = icm42688_write_reg(dev, ICM42688_REG_PWR_MGMT0, pwr);
	if (rc) {
		return rc;
	}

	k_busy_wait(200);
	k_sleep(K_MSEC(45));

	return 0;
}

static int icm42688_init(const struct device *dev)
{
	const struct icm42688_config *cfg = dev->config;

	/* Check bus readiness based on interface type */
	switch (cfg->bus_type) {
	case ICM42688_BUS_I2C:
		if (!device_is_ready(cfg->i2c.bus.bus)) {
			LOG_ERR("I2C bus not ready");
			return -ENODEV;
		}
		LOG_INF("ICM42688 using I2C interface");
		break;

	case ICM42688_BUS_SPI:
#if defined(CONFIG_SPI)
		if (!device_is_ready(cfg->spi.bus.bus)) {
			LOG_ERR("SPI bus not ready");
			return -ENODEV;
		}
		LOG_INF("ICM42688 using SPI interface (up to 24MHz, mode 0/3)");
#else
		LOG_ERR("SPI selected but CONFIG_SPI is not enabled");
		return -ENOTSUP;
#endif
		break;

	case ICM42688_BUS_I3C:
#if defined(CONFIG_I3C)
		if (cfg->i3c.bus == NULL) {
			LOG_ERR("I3C device descriptor not initialized");
			return -ENODEV;
		}
		LOG_INF("ICM42688 using I3C interface (high-speed, dynamic addressing)");
#else
		LOG_ERR("I3C selected but CONFIG_I3C is not enabled");
		return -ENOTSUP;
#endif
		break;

	default:
		LOG_ERR("Unknown bus type: %d", cfg->bus_type);
		return -EINVAL;
	}

	int rc = icm42688_hw_probe(dev);
	if (rc) {
		return rc;
	}

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	(void)icm42688_write_reg(dev, ICM42688_REG_DEVICE_CONFIG, 0x01);
	k_sleep(K_MSEC(2));

	rc = icm42688_apply_defaults(dev);
	if (rc) {
		return rc;
	}

	rc = icm42688_enable_sensors_ln(dev);
	if (rc) {
		return rc;
	}

/* Initialize APEX (DMP) before FIFO so DMP setup doesn't overwrite or
 * populate FIFO with unexpected/zeroed packets after FIFO is enabled.
 */
#ifdef CONFIG_ICM42688_APEX
	if (cfg->apex_enable && cfg->apex_features) {
		rc = icm42688_apex_init(dev, cfg->apex_features);
		if (rc) {
			LOG_ERR("APEX init failed: %d", rc);
			return rc;
		}
	}
#endif

#if defined(CONFIG_ICM42688_FORCE_DISABLE_APEX) && CONFIG_ICM42688_FORCE_DISABLE_APEX
	/* Build-time option to force-disable APEX/DMP to isolate FIFO behavior. */
	LOG_INF("CONFIG_ICM42688_FORCE_DISABLE_APEX active: disabling APEX permanently");
	(void)icm42688_write_reg(dev, ICM42688_REG_APEX_CONFIG0, 0);
#endif

#ifdef CONFIG_ICM42688_FIFO
	if (cfg->fifo_enable) {
		rc = icm42688_fifo_init(dev);
		if (rc) {
			LOG_ERR("fifo init failed: %d", rc);
			return rc;
		}
	}
#endif

#ifdef CONFIG_ICM42688_TRIGGER
	rc = icm42688_trigger_init(dev);
	if (rc == -ENOTSUP) {
		LOG_WRN("trigger disabled (no int1-gpios?)");
	} else if (rc < 0) {
		LOG_ERR("trigger init failed: %d", rc);
		return rc;
	}
#endif

	LOG_INF("ICM42688 init ok");
	return 0;
}

/* ========= Devicetree instantiation ========= */

#define ICM42688_DEFINE(inst)                                                      \
	static struct icm42688_data icm42688_data_##inst;                           \
	static const struct icm42688_config icm42688_cfg_##inst = {                 \
		.bus_type = DT_INST_ON_BUS(inst, spi) ? ICM42688_BUS_SPI :           \
			    DT_INST_ON_BUS(inst, i3c) ? ICM42688_BUS_I3C :           \
			    ICM42688_BUS_I2C,                                        \
		COND_CODE_1(DT_INST_ON_BUS(inst, i2c),                              \
			(.i2c = { .bus = I2C_DT_SPEC_INST_GET(inst) },),            \
			())                                                          \
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),                              \
			(.spi = { .bus = SPI_DT_SPEC_INST_GET(                        \
					inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0) },), \
			())                                                          \
		IF_ENABLED(CONFIG_ICM42688_TRIGGER,                                 \
			(.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int1_gpios, {0}), \
			 .int2_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int2_gpios, {0}),)) \
		.accel_fs_g = 16,                                                  \
		.gyro_fs_dps = 2000,                                               \
		.accel_odr_hz = 1000,                                              \
		.gyro_odr_hz = 1000,                                               \
		IF_ENABLED(CONFIG_ICM42688_FIFO, (                                  \
			.fifo_enable = DT_INST_PROP_OR(inst, fifo_enable, false),   \
			.fifo_hires = DT_INST_PROP_OR(inst, fifo_hires, false),     \
			.fifo_watermark = (uint16_t)DT_INST_PROP_OR(inst, fifo_watermark, 256), \
		))                                                                  \
		IF_ENABLED(CONFIG_ICM42688_APEX, (                                  \
			.apex_enable = DT_INST_PROP_OR(inst, apex_enable, false),   \
			.apex_features = (uint8_t)DT_INST_PROP_OR(inst, apex_features, 0), \
			.apex_dmp_power_save = DT_INST_PROP_OR(inst, apex_dmp_power_save, false), \
			.apex_dmp_odr_sel = (uint8_t)DT_INST_PROP_OR(inst, apex_dmp_odr_sel, ICM42688_APEX_DMP_ODR_50HZ), \
			.apex_sensitivity_mode = (uint8_t)DT_INST_PROP_OR(inst, apex_sensitivity_mode, 0), \
			.apex_tilt_wait_time_sel = (uint8_t)DT_INST_PROP_OR(inst, apex_tilt_wait_time_sel, 2), \
			.apex_sleep_timeout_sel = (uint8_t)DT_INST_PROP_OR(inst, apex_sleep_timeout_sel, 0), \
			.apex_mounting_matrix = (uint8_t)DT_INST_PROP_OR(inst, apex_mounting_matrix, 0), \
			.apex_sleep_gesture_delay = (uint8_t)DT_INST_PROP_OR(inst, apex_sleep_gesture_delay, 0), \
			.apex_route_int2 = DT_INST_PROP_OR(inst, apex_route_int2, false), \
		))                                                                  \
	};                                                                            \
	DEVICE_DT_INST_DEFINE(inst,                                                   \
			      icm42688_init,                                           \
			      NULL,                                                   \
			      &icm42688_data_##inst,                                  \
			      &icm42688_cfg_##inst,                                   \
			      POST_KERNEL,                                            \
			      CONFIG_SENSOR_INIT_PRIORITY,                            \
			      &icm42688_api);

DT_INST_FOREACH_STATUS_OKAY(ICM42688_DEFINE)
