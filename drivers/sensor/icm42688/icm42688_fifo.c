/* SPDX-License-Identifier: Apache-2.0 */
#include "icm42688.h"
#include "icm42688_reg.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_DECLARE(icm42688, CONFIG_SENSOR_LOG_LEVEL);

#ifdef CONFIG_ICM42688_FIFO

/* ============================================================================
 *  IMPORTANT (driver runtime fields this file expects in struct icm42688_data)
 *  --------------------------------------------------------------------------
 *  uint8_t  fifo_tail[32];
 *  uint8_t  fifo_tail_len;
 *  bool     fifo_overflowed;
 *  uint32_t fifo_tmst_accum;   // expanded 32-bit timestamp accumulator
 *  uint16_t fifo_tmst_prev16;  // last seen 16-bit timestamp
 *  bool     fifo_tmst_have_prev;
 *  bool     tmst_delta_en;     // optional: if you ever enable delta mode
 * ============================================================================
 */

/* Forward decls */
static int icm42688_fifo_count(const struct device *dev, uint16_t *count);
static int icm42688_fifo_read_bytes(const struct device *dev, uint8_t *dst, size_t len);

/* ============================================================================
 * FIFO header bits (datasheet FIFO packet header)
 * ============================================================================ */
#define FIFO_HDR_MSG        (1u << 7)
#define FIFO_HDR_ACCEL      (1u << 6)
#define FIFO_HDR_GYRO       (1u << 5)
#define FIFO_HDR_20BIT      (1u << 4)
#define FIFO_HDR_TMST_ODR   (1u << 3)
#define FIFO_HDR_TMST_FSYNC (1u << 2)

/* Packet sizes */
#define FIFO_PACKET_HEADER_ONLY     1
#define FIFO_PACKET_ACCEL_ONLY      8
#define FIFO_PACKET_GYRO_ONLY       8
#define FIFO_PACKET_ACCEL_GYRO      16
#define FIFO_PACKET_ACCEL_GYRO_HR   20

/* FIFO_CONFIG (0x16): FIFO_MODE bits [7:6] */
#define ICM42688_FIFO_MODE_SHIFT 6
#define ICM42688_FIFO_MODE_MASK  (0x3u << ICM42688_FIFO_MODE_SHIFT)

#define ICM42688_FIFO_MODE_BYPASS   0u
#define ICM42688_FIFO_MODE_STREAM   1u
#define ICM42688_FIFO_MODE_STOPFULL 2u

/* FIFO_CONFIG1 bits */
#define ICM42688_FIFO_ACCEL_EN            (1u << 0)
#define ICM42688_FIFO_GYRO_EN             (1u << 1)
#define ICM42688_FIFO_TEMP_EN             (1u << 2)
#define ICM42688_FIFO_TMST_FSYNC_EN       (1u << 3) /* timestamp in packets */
#define ICM42688_FIFO_HIRES_EN            (1u << 4)
#define ICM42688_FIFO_WM_GT_TH            (1u << 5)
#define ICM42688_FIFO_RESUME_PARTIAL_RD   (1u << 6)

/* ============================================================================
 * Helpers
 * ============================================================================ */

static inline int32_t sign_extend_20(int32_t v20)
{
	v20 &= 0xFFFFF;
	if (v20 & (1 << 19)) {
		v20 |= ~0xFFFFF;
	}
	return v20;
}

static int fifo_get_packet_size(uint8_t header)
{
	if (header & FIFO_HDR_MSG) {
		return FIFO_PACKET_HEADER_ONLY;
	}

	const bool has_accel = (header & FIFO_HDR_ACCEL) != 0;
	const bool has_gyro  = (header & FIFO_HDR_GYRO)  != 0;
	const bool is_20bit  = (header & FIFO_HDR_20BIT) != 0;

	if (has_accel && has_gyro) {
		return is_20bit ? FIFO_PACKET_ACCEL_GYRO_HR : FIFO_PACKET_ACCEL_GYRO;
	}

	if (has_accel || has_gyro) {
		return 8;
	}

	return FIFO_PACKET_HEADER_ONLY;
}

static uint32_t fifo_expand_timestamp(struct icm42688_data *data, uint16_t ts16)
{
	if (!data->tmst_delta_en) {
		/* Normal mode: FIFO gives low16 of a running counter, detect rollover */
		if (!data->fifo_tmst_have_prev) {
			data->fifo_tmst_have_prev = true;
			data->fifo_tmst_prev16 = ts16;
			data->fifo_tmst_accum = (data->fifo_tmst_accum & 0xFFFF0000u) | ts16;
			return data->fifo_tmst_accum;
		}

		if (ts16 < data->fifo_tmst_prev16) {
			data->fifo_tmst_accum += 0x10000u;
		}

		data->fifo_tmst_prev16 = ts16;
		data->fifo_tmst_accum = (data->fifo_tmst_accum & 0xFFFF0000u) | ts16;
		return data->fifo_tmst_accum;
	} else {
		/* Delta timestamp mode: ts16 - это дельта */
		if (!data->fifo_tmst_have_prev) {
			data->fifo_tmst_have_prev = true;
			data->fifo_tmst_prev16 = 0;
			data->fifo_tmst_accum = ts16;
			return data->fifo_tmst_accum;
		}
		data->fifo_tmst_accum += ts16;
		data->fifo_tmst_prev16 = ts16;
		return data->fifo_tmst_accum;
	}
}

static int fifo_set_mode(const struct device *dev, uint8_t mode)
{
	uint8_t cur = 0;
	int rc = icm42688_read_reg(dev, ICM42688_REG_FIFO_CONFIG, &cur);
	if (rc < 0) {
		return rc;
	}

	uint8_t nv = (cur & ~ICM42688_FIFO_MODE_MASK) |
		     ((mode & 0x3u) << ICM42688_FIFO_MODE_SHIFT);

	return icm42688_write_reg(dev, ICM42688_REG_FIFO_CONFIG, nv);
}

static int fifo_set_watermark(const struct device *dev, uint16_t wm)
{
	/* FIFO_CONFIG2 = WM[7:0], FIFO_CONFIG3 = WM[11:8] */
	uint8_t lo = (uint8_t)(wm & 0xFFu);
	uint8_t hi = (uint8_t)((wm >> 8) & 0x0Fu);

	int rc = icm42688_write_reg(dev, ICM42688_REG_FIFO_CONFIG2, lo);
	if (rc < 0) {
		return rc;
	}
	return icm42688_write_reg(dev, ICM42688_REG_FIFO_CONFIG3, hi);
}

static int icm42688_fifo_int_clear_policy(const struct device *dev)
{
	/* Clear FIFO ints on INT_STATUS read */
	uint8_t mask = ICM42688_INT_CONFIG0_FIFO_THS_INT_CLEAR_MASK |
		       ICM42688_INT_CONFIG0_FIFO_FULL_INT_CLEAR_MASK;

	uint8_t val =
		(ICM42688_FIFO_INT_CLEAR_ON_STATUS_READ << ICM42688_INT_CONFIG0_FIFO_THS_INT_CLEAR_SHIFT) |
		(ICM42688_FIFO_INT_CLEAR_ON_STATUS_READ << ICM42688_INT_CONFIG0_FIFO_FULL_INT_CLEAR_SHIFT);

	return icm42688_reg_update8(dev, ICM42688_REG_INT_CONFIG0, mask, val);
}

static int icm42688_fifo_route_irqs_int1(const struct device *dev, bool wm, bool full)
{
	uint8_t mask = ICM42688_INT_SRC_FIFO_THS_INT1_EN |
		       ICM42688_INT_SRC_FIFO_FULL_INT1_EN;

	uint8_t val = 0;
	if (wm) {
		val |= ICM42688_INT_SRC_FIFO_THS_INT1_EN;
	}
	if (full) {
		val |= ICM42688_INT_SRC_FIFO_FULL_INT1_EN;
	}

	return icm42688_reg_update8(dev, ICM42688_REG_INT_SOURCE0, mask, val);
}

/* ============================================================================
 * Public: FIFO flush
 * ============================================================================ */
int icm42688_fifo_flush(const struct device *dev)
{
	struct icm42688_data *data = dev->data;

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	/* SIGNAL_PATH_RESET.FIFO_FLUSH */
	int rc = icm42688_write_reg(dev, ICM42688_REG_SIGNAL_PATH_RESET,
				    ICM42688_SPR_FIFO_FLUSH);
	if (rc < 0) {
		return rc;
	}

	/* write-on-clear */
	k_busy_wait(50);

	/* Reset SW parser/timestamp state */
	data->fifo_tail_len = 0;
	data->fifo_overflowed = false;

	data->fifo_tmst_accum = 0;
	data->fifo_tmst_prev16 = 0;
	data->fifo_tmst_have_prev = false;

	/* Purge packet queue too */
	if (data->fifo_msgq) {
		k_msgq_purge(data->fifo_msgq);
	}

	return 0;
}


/* ============================================================================
 * Public: FIFO init
 * ============================================================================ */
int icm42688_fifo_init(const struct device *dev)
{
	const struct icm42688_config *cfg = dev->config;
	struct icm42688_data *data = dev->data;
	int rc;

	if (!cfg->fifo_enable) {
		return 0;
	}

	/* Init FIFO packet queue before any IRQ/work starts draining */
	icm42688_fifo_msgq_init_once(dev);
	(void)icm42688_fifo_queue_reset(dev);

	/* 1) INTF_CONFIG0: keep interface bits, set stable FIFO options + BE */
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	uint8_t intf0 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_INTF_CONFIG0, &intf0);
	if (rc < 0) {
		return rc;
	}

	uint8_t new_intf0 = intf0;
	new_intf0 &= ~ICM42688_INTF_FIFO_HOLD_LAST_DATA_EN;
	new_intf0 &= ~ICM42688_INTF_FIFO_COUNT_REC;  // Исправлено: было ICM42688_INTF_FIFO_COUNT_ENC

	/* Keep BE (matches sys_get_be16 usage) */
	new_intf0 |= ICM42688_INTF_SENSOR_DATA_ENDIAN;
	new_intf0 |= ICM42688_INTF_FIFO_COUNT_ENDIAN;

	if (new_intf0 != intf0) {
		rc = icm42688_write_reg(dev, ICM42688_REG_INTF_CONFIG0, new_intf0);
		if (rc < 0) {
			return rc;
		}
	}

	/* 2) Timestamp resolution (0 = 1us) */
	rc = icm42688_timestamp_set_resolution(dev, ICM42688_TIMESTAMP_RES_1US);
	if (rc < 0) {
		return rc;
	}

	/* 3) Watermark: make it multiple of packet size (16 or 20) */
	uint16_t wm = cfg->fifo_watermark ? cfg->fifo_watermark : 256;
	const uint16_t pkt_sz = cfg->fifo_hires ? 20u : 16u;

	if ((wm % pkt_sz) != 0u) {
		uint16_t adj = (uint16_t)((wm / pkt_sz) * pkt_sz);
		if (adj < pkt_sz) {
			adj = pkt_sz;
		}
		LOG_WRN("FIFO watermark not multiple of packet size, adjusting");
		wm = adj;
	}

	/* 4) BYPASS while configuring */
	rc = fifo_set_mode(dev, ICM42688_FIFO_MODE_BYPASS);
	if (rc < 0) {
		return rc;
	}

	/* 5) Set watermark */
	rc = fifo_set_watermark(dev, wm);
	if (rc < 0) {
		return rc;
	}

	/* 6) FIFO_CONFIG1: accel+gyro+temp + timestamp, hires optional */
	uint8_t f1 = 0;
	f1 |= ICM42688_FIFO_ACCEL_EN;
	f1 |= ICM42688_FIFO_GYRO_EN;
	f1 |= ICM42688_FIFO_TEMP_EN;
	f1 |= ICM42688_FIFO_TMST_FSYNC_EN;     /* include timestamp */
	f1 |= ICM42688_FIFO_RESUME_PARTIAL_RD; /* safe chunk reads */
	f1 |= ICM42688_FIFO_WM_GT_TH;          /* IRQ when COUNT > WM */

	if (cfg->fifo_hires) {
		f1 |= ICM42688_FIFO_HIRES_EN;
	}

	rc = icm42688_write_reg(dev, ICM42688_REG_FIFO_CONFIG1, f1);
	if (rc < 0) {
		return rc;
	}

	/* 7) STREAM mode */
	rc = fifo_set_mode(dev, ICM42688_FIFO_MODE_STREAM);
	if (rc < 0) {
		return rc;
	}

	/* 8) Enable timestamp counter */
	(void)icm42688_reg_update8(dev, ICM42688_REG_TMST_CONFIG,
				   ICM42688_TMST_EN | ICM42688_TMST_TO_REGS_EN,
				   ICM42688_TMST_EN | ICM42688_TMST_TO_REGS_EN);

	/* 9) FIFO IRQ routing + clear policy (INT1: watermark+full) */
	rc = icm42688_fifo_int_clear_policy(dev);
	if (rc < 0) {
		return rc;
	}
	rc = icm42688_fifo_route_irqs_int1(dev, true, true);
	if (rc < 0) {
		return rc;
	}

	/* 10) Flush to start clean */
	rc = icm42688_fifo_flush(dev);
	if (rc < 0) {
		return rc;
	}

	/* Reset SW state explicitly */
	data->fifo_overflowed = false;

	LOG_INF("FIFO init completed");
	return 0;
}

/* ============================================================================
 * Timestamp controls
 * ============================================================================ */
int icm42688_timestamp_set_resolution(const struct device *dev, uint8_t res)
{
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	if (res == ICM42688_TIMESTAMP_RES_16US) {
		return icm42688_reg_update8(dev, ICM42688_REG_TMST_CONFIG,
					    ICM42688_TMST_RES, ICM42688_TMST_RES);
	}

	return icm42688_reg_update8(dev, ICM42688_REG_TMST_CONFIG,
				    ICM42688_TMST_RES, 0);
}

int icm42688_fifo_timestamp_enable(const struct device *dev, bool enable)
{
	struct icm42688_data *data = dev->data;
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	/* Reset SW accumulator any time we toggle */
	data->fifo_tmst_accum = 0;
	data->fifo_tmst_prev16 = 0;
	data->fifo_tmst_have_prev = false;

	if (enable) {
		int rc = icm42688_reg_update8(dev, ICM42688_REG_TMST_CONFIG,
					      ICM42688_TMST_EN | ICM42688_TMST_TO_REGS_EN,
					      ICM42688_TMST_EN | ICM42688_TMST_TO_REGS_EN);
		if (rc < 0) {
			return rc;
		}

		rc = icm42688_reg_update8(dev, ICM42688_REG_FIFO_CONFIG1,
					  ICM42688_FIFO_TMST_FSYNC_EN,
					  ICM42688_FIFO_TMST_FSYNC_EN);
		return rc;
	}

	int rc = icm42688_reg_update8(dev, ICM42688_REG_FIFO_CONFIG1,
				      ICM42688_FIFO_TMST_FSYNC_EN, 0);
	if (rc < 0) {
		return rc;
	}

	return icm42688_reg_update8(dev, ICM42688_REG_TMST_CONFIG,
				    ICM42688_TMST_EN | ICM42688_TMST_TO_REGS_EN, 0);
}

/* Timestamp value from TMSTVALx (Bank1) */
int icm42688_read_timestamp(const struct device *dev, uint32_t *timestamp)
{
	uint8_t b0, b1, b2;
	int rc;

	if (!timestamp) {
		return -EINVAL;
	}

	rc = icm42688_reg_bank_sel(dev, ICM42688_BANK1);
	if (rc < 0) {
		return rc;
	}

	/* TMSTVAL0 read latches */
	rc = icm42688_read_reg(dev, ICM42688_REG_TMSTVAL0, &b0);
	if (rc < 0) {
		goto out;
	}
	rc = icm42688_read_reg(dev, ICM42688_REG_TMSTVAL1, &b1);
	if (rc < 0) {
		goto out;
	}
	rc = icm42688_read_reg(dev, ICM42688_REG_TMSTVAL2, &b2);

out:
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);
	if (rc < 0) {
		return rc;
	}

	*timestamp = ((uint32_t)b0) | ((uint32_t)b1 << 8) | ((uint32_t)(b2 & 0x0F) << 16);
	return 0;
}

/* ============================================================================
 * FIFO count & raw read
 * ============================================================================ */
static int icm42688_fifo_count(const struct device *dev, uint16_t *count)
{
	uint8_t b[2];

	if (!count) {
		return -EINVAL;
	}

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	int rc = icm42688_read_burst(dev, ICM42688_REG_FIFO_COUNTH, b, 2);
	if (rc < 0) {
		return rc;
	}

	/* INTF_CONFIG0.FIFO_COUNT_ENDIAN (1=BE, 0=LE) */
	uint8_t intf0 = 0;
	rc = icm42688_read_reg(dev, ICM42688_REG_INTF_CONFIG0, &intf0);
	if (rc < 0) {
		return rc;
	}

	if (intf0 & ICM42688_INTF_FIFO_COUNT_ENDIAN) {
		*count = sys_get_be16(b);
	} else {
		*count = sys_get_le16(b);
	}

	return 0;
}

static int icm42688_fifo_read_bytes(const struct device *dev, uint8_t *dst, size_t len)
{
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	size_t remaining = len;
	size_t off = 0;

	while (remaining > 0) {
		size_t chunk = MIN(remaining, (size_t)128);

		int rc = icm42688_read_burst(dev, ICM42688_REG_FIFO_DATA, &dst[off], chunk);
		if (rc < 0) {
			return rc;
		}

		off += chunk;
		remaining -= chunk;
	}

	return 0;
}

int icm42688_fifo_read_all(const struct device *dev, uint8_t *buf, size_t cap, size_t *out_len)
{
	if (!buf || !out_len) {
		return -EINVAL;
	}

	uint16_t cnt = 0;
	int rc = icm42688_fifo_count(dev, &cnt);
	if (rc < 0) {
		return rc;
	}

	if (cnt == 0) {
		*out_len = 0;
		return 0;
	}

	size_t to_read = (size_t)cnt;
	if (to_read > cap) {
		to_read = cap;
	}

	rc = icm42688_fifo_read_bytes(dev, buf, to_read);
	if (rc < 0) {
		return rc;
	}

	*out_len = to_read;
	return 0;
}

/* ============================================================================
 * FIFO packet parser (signature kept as in your header)
 *   - 16-byte: timestamp is bytes [14..15] MSB-first
 *   - 20-byte: timestamp is bytes [15..16] MSB-first
 * ============================================================================ */
int icm42688_fifo_parse_packet(const uint8_t *buf, size_t len,
			       struct icm42688_fifo_data *out)
{
	if (!buf || !out || len < 1) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));

	const uint8_t h = buf[0];
	out->header = h;

	if (h & FIFO_HDR_MSG) {
		out->valid = false;
		return FIFO_PACKET_HEADER_ONLY;
	}

	const int pkt_size = fifo_get_packet_size(h);
	if ((size_t)pkt_size > len) {
		return -ENODATA;
	}

	out->valid = true;
	out->has_accel = (h & FIFO_HDR_ACCEL) != 0;
	out->has_gyro  = (h & FIFO_HDR_GYRO)  != 0;
	out->is_20bit  = (h & FIFO_HDR_20BIT) != 0;

	const uint8_t tmst_mode = (h >> 2) & 0x3u;
	out->has_timestamp = (tmst_mode == 0x2u || tmst_mode == 0x3u);

	/* 8-byte accel-only / gyro-only:
	 * [0] hdr
	 * [1..6] XYZ BE16
	 * [7] temp8
	 */
	if (pkt_size == 8) {
		out->temp_raw8  = (int8_t)buf[7];
		out->temp_raw16 = 0;
		out->has_timestamp = false;
		out->timestamp16 = 0;

		if (out->has_accel) {
			out->accel_x = (int32_t)(int16_t)sys_get_be16(&buf[1]);
			out->accel_y = (int32_t)(int16_t)sys_get_be16(&buf[3]);
			out->accel_z = (int32_t)(int16_t)sys_get_be16(&buf[5]);
		} else if (out->has_gyro) {
			out->gyro_x = (int32_t)(int16_t)sys_get_be16(&buf[1]);
			out->gyro_y = (int32_t)(int16_t)sys_get_be16(&buf[3]);
			out->gyro_z = (int32_t)(int16_t)sys_get_be16(&buf[5]);
		}

		return pkt_size;
	}

	/* 16-byte accel+gyro:
	 * [0] hdr
	 * [1..6] accel
	 * [7..12] gyro
	 * [13] temp8
	 * [14..15] timestamp16 (MSB-first)
	 */
	if (pkt_size == 16) {
		out->accel_x = (int32_t)(int16_t)sys_get_be16(&buf[1]);
		out->accel_y = (int32_t)(int16_t)sys_get_be16(&buf[3]);
		out->accel_z = (int32_t)(int16_t)sys_get_be16(&buf[5]);

		out->gyro_x  = (int32_t)(int16_t)sys_get_be16(&buf[7]);
		out->gyro_y  = (int32_t)(int16_t)sys_get_be16(&buf[9]);
		out->gyro_z  = (int32_t)(int16_t)sys_get_be16(&buf[11]);

		out->temp_raw8  = (int8_t)buf[13];
		out->temp_raw16 = 0;

		out->timestamp16 = out->has_timestamp ? sys_get_be16(&buf[14]) : 0;
		return pkt_size;
	}

	/* 20-byte hi-res accel+gyro:
	 * [0] hdr
	 * [1..12] base words
	 * [13..14] temp16
	 * [15..16] timestamp16 (MSB-first)
	 * [17..19] ext nibbles (AX/GX, AY/GY, AZ/GZ)
	 */
	if (pkt_size == 20) {
		const int16_t ax_base = (int16_t)sys_get_be16(&buf[1]);
		const int16_t ay_base = (int16_t)sys_get_be16(&buf[3]);
		const int16_t az_base = (int16_t)sys_get_be16(&buf[5]);

		const int16_t gx_base = (int16_t)sys_get_be16(&buf[7]);
		const int16_t gy_base = (int16_t)sys_get_be16(&buf[9]);
		const int16_t gz_base = (int16_t)sys_get_be16(&buf[11]);

		const uint8_t ext0 = buf[17];
		const uint8_t ext1 = buf[18];
		const uint8_t ext2 = buf[19];

		const uint8_t ax_lo4 = (ext0 >> 4) & 0x0F;
		const uint8_t gx_lo4 = (ext0 >> 0) & 0x0F;

		const uint8_t ay_lo4 = (ext1 >> 4) & 0x0F;
		const uint8_t gy_lo4 = (ext1 >> 0) & 0x0F;

		const uint8_t az_lo4 = (ext2 >> 4) & 0x0F;
		const uint8_t gz_lo4 = (ext2 >> 0) & 0x0F;

		out->accel_x = sign_extend_20(((int32_t)ax_base << 4) | ax_lo4);
		out->accel_y = sign_extend_20(((int32_t)ay_base << 4) | ay_lo4);
		out->accel_z = sign_extend_20(((int32_t)az_base << 4) | az_lo4);

		out->gyro_x  = sign_extend_20(((int32_t)gx_base << 4) | gx_lo4);
		out->gyro_y  = sign_extend_20(((int32_t)gy_base << 4) | gy_lo4);
		out->gyro_z  = sign_extend_20(((int32_t)gz_base << 4) | gz_lo4);

		out->temp_raw16 = (int16_t)sys_get_be16(&buf[13]);
		out->temp_raw8  = (int8_t)(out->temp_raw16 & 0xFF);

		out->timestamp16 = out->has_timestamp ? sys_get_be16(&buf[15]) : 0;
		return pkt_size;
	}

	out->valid = false;
	return FIFO_PACKET_HEADER_ONLY;
}

/* ============================================================================
 * Robust draining: tail + read bytes -> parse complete packets
 * ============================================================================ */
static int icm42688_fifo_handle_overflow(const struct device *dev)
{
	struct icm42688_data *data = dev->data;

	LOG_DBG("FIFO overflow: resetting parser state");
	data->fifo_tail_len = 0;
	data->fifo_overflowed = false;

	int rc = icm42688_fifo_flush(dev);
	if (rc < 0) {
		LOG_ERR("fifo_flush failed after overflow");
		return rc;
	}

	return 0;
}

int icm42688_fifo_drain_and_parse(const struct device *dev,
				 struct icm42688_fifo_data *out_pkts,
				 size_t max_pkts,
				 size_t *out_n,
				 size_t max_bytes)
{
	struct icm42688_data *data = dev->data;

	if (!out_pkts || !out_n || max_pkts == 0) {
		return -EINVAL;
	}

	*out_n = 0;

	if (data->fifo_overflowed) {
		int rc = icm42688_fifo_handle_overflow(dev);
		if (rc < 0) {
			return rc;
		}
	}

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	/* Check FIFO count first - FIFO_FULL flag can be stale */
	uint16_t fifo_cnt = 0;
	int rc = icm42688_fifo_count(dev, &fifo_cnt);
	if (rc < 0) {
		return rc;
	}
	if (fifo_cnt == 0) {
		return 0;
	}

	/* Limit read size to prevent I2C bus saturation */
	size_t to_read = (size_t)fifo_cnt;
	if (to_read > 512) {
		to_read = 512;  /* Hard limit regardless of max_bytes */
	}
	if (max_bytes > 0 && to_read > max_bytes) {
		to_read = max_bytes;
	}

	uint8_t raw[512];  /* Increased buffer size */
	if (to_read > sizeof(raw)) {
		to_read = sizeof(raw);
	}

	rc = icm42688_fifo_read_bytes(dev, raw, to_read);
	if (rc < 0) {
		return rc;
	}

	uint8_t parse_buf[32 + 512];  /* Match raw buffer size */
	size_t parse_len = 0;

	if (data->fifo_tail_len > 0) {
		memcpy(parse_buf, data->fifo_tail, data->fifo_tail_len);
		parse_len = data->fifo_tail_len;
	}

	memcpy(&parse_buf[parse_len], raw, to_read);
	parse_len += to_read;

	size_t off = 0;
	while (off < parse_len && *out_n < max_pkts) {

		if ((parse_len - off) < 1) {
			break;
		}

		uint8_t hdr = parse_buf[off];
		int expected = fifo_get_packet_size(hdr);
		if (expected <= 0) {
			data->fifo_overflowed = true;
			return icm42688_fifo_handle_overflow(dev);
		}

		if ((parse_len - off) < (size_t)expected) {
			break; /* tail */
		}

		struct icm42688_fifo_data pkt;
		int used = icm42688_fifo_parse_packet(&parse_buf[off], parse_len - off, &pkt);
		if (used == -ENODATA) {
			break;
		}
		if (used < 0) {
			/* desync: shift by 1 byte */
			off += 1;
			continue;
		}

		off += (size_t)used;

		if (!pkt.valid) {
			continue;
		}

		/* Expand timestamp into SW accumulator (kept in data),
		 * but packet struct keeps only timestamp16.
		 */
		if (pkt.has_timestamp) {
			(void)fifo_expand_timestamp(data, pkt.timestamp16);
		}

		out_pkts[*out_n] = pkt;
		(*out_n)++;
	}

	/* Save tail */
	size_t rem = parse_len - off;
	if (rem > sizeof(data->fifo_tail)) {
		data->fifo_overflowed = true;
		return icm42688_fifo_handle_overflow(dev);
	}

	if (rem > 0) {
		memcpy(data->fifo_tail, &parse_buf[off], rem);
		data->fifo_tail_len = (uint8_t)rem;
	} else {
		data->fifo_tail_len = 0;
	}

	return 0;
}

/* ============================================================================
 * Simple "read once" API: read up to 256 bytes, parse into packets[]
 * ============================================================================ */
int icm42688_fifo_read_packets(const struct device *dev,
			       struct icm42688_fifo_data *packets,
			       size_t max_packets,
			       size_t *num_packets)
{
	if (!packets || !num_packets || max_packets == 0) {
		return -EINVAL;
	}

	*num_packets = 0;

	uint16_t cnt = 0;
	int rc = icm42688_fifo_count(dev, &cnt);
	if (rc < 0) {
		return rc;
	}
	if (cnt == 0) {
		return 0;
	}

	uint8_t buf[256];
	size_t to_read = MIN((size_t)cnt, sizeof(buf));

	rc = icm42688_fifo_read_bytes(dev, buf, to_read);
	if (rc < 0) {
		return rc;
	}

	size_t off = 0;
	size_t n = 0;

	while (off < to_read && n < max_packets) {
		struct icm42688_fifo_data pkt;

		int used = icm42688_fifo_parse_packet(&buf[off], to_read - off, &pkt);
		if (used == -ENODATA) {
			break;
		}
		if (used < 0) {
			/* desync: shift by 1 byte */
			off += 1;
			continue;
		}

		off += (size_t)used;

		if (!pkt.valid) {
			continue;
		}

		packets[n++] = pkt;
	}

	*num_packets = n;
	return 0;
}

int icm42688_fifo_get_latest_timestamp(const struct device *dev, uint16_t *timestamp)
{
	if (!timestamp) {
		return -EINVAL;
	}

	*timestamp = 0;

	uint8_t buf[128];
	size_t read = 0;
	int rc = icm42688_fifo_read_all(dev, buf, sizeof(buf), &read);
	if (rc < 0) {
		return rc;
	}

	size_t off = 0;
	while (off < read) {
		struct icm42688_fifo_data pkt;
		int used = icm42688_fifo_parse_packet(&buf[off], read - off, &pkt);
		if (used < 0) {
			break;
		}
		off += (size_t)used;

		if (pkt.valid && pkt.has_timestamp) {
			*timestamp = pkt.timestamp16;
		}
	}

	return 0;
}

#endif /* CONFIG_ICM42688_FIFO */
