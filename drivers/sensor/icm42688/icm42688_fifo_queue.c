/* SPDX-License-Identifier: Apache-2.0 */
#include "icm42688.h"
#include "icm42688_reg.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <string.h>

LOG_MODULE_DECLARE(icm42688, CONFIG_SENSOR_LOG_LEVEL);

#ifdef CONFIG_ICM42688_FIFO

/* ====== Tunables ====== */
#ifndef CONFIG_ICM42688_FIFO_MSGQ_LEN
#define CONFIG_ICM42688_FIFO_MSGQ_LEN 64
#endif

/* Each msg is one parsed packet struct */
#define FIFO_MSG_SIZE (sizeof(struct icm42688_fifo_data))

/* Forward: uses your robust tail-aware drain_and_parse() */
static int fifo_read_and_enqueue_some(const struct device *dev);

/* =====================================================================================
 * Global msgq (simplified - single instance support)
 * ===================================================================================== */
K_MSGQ_DEFINE(icm42688_fifo_msgq_global, FIFO_MSG_SIZE, CONFIG_ICM42688_FIFO_MSGQ_LEN, 4);

/* =====================================================================================
 * Init-once called from fifo_init()
 * ===================================================================================== */
void icm42688_fifo_msgq_init_once(const struct device *dev)
{
	struct icm42688_data *data = dev->data;

	if (data->fifo_msgq_inited) {
		return;
	}

	/* Bind to global msgq */
	data->fifo_msgq = &icm42688_fifo_msgq_global;
	data->fifo_msgq_inited = true;
	
	LOG_INF("FIFO msgq initialized successfully");
}

/* =====================================================================================
 * Reset queue contents
 * ===================================================================================== */
int icm42688_fifo_queue_reset(const struct device *dev)
{
	struct icm42688_data *data = dev->data;

	if (!data->fifo_msgq_inited || data->fifo_msgq == NULL) {
		return -ENOTSUP;
	}

	k_msgq_purge(data->fifo_msgq);
	return 0;
}

/* =====================================================================================
 * Public pop API
 * ===================================================================================== */
int icm42688_fifo_pop(const struct device *dev,
		      struct icm42688_fifo_data *out,
		      k_timeout_t timeout)
{
	struct icm42688_data *data = dev->data;

	if (!out) {
		return -EINVAL;
	}

	if (!data->fifo_msgq_inited || data->fifo_msgq == NULL) {
		return -ENOTSUP;
	}

	int rc = k_msgq_get(data->fifo_msgq, out, timeout);
	if (rc == 0) {
		return 0;
	}

	/* Zephyr returns -EAGAIN on timeout for msgq_get */
	return -EAGAIN;
}

/* =====================================================================================
 * Internal helper: enqueue one packet, drop oldest if full (lossy but real-time safe)
 * ===================================================================================== */
static void msgq_put_drop_oldest(struct k_msgq *q, const struct icm42688_fifo_data *pkt)
{
	/* try put, if full -> drop oldest -> put again */
	int rc = k_msgq_put(q, pkt, K_NO_WAIT);
	if (rc == 0) {
		return;
	}

	/* drop one */
	struct icm42688_fifo_data tmp;
	(void)k_msgq_get(q, &tmp, K_NO_WAIT);
	(void)k_msgq_put(q, pkt, K_NO_WAIT);
}

/* =====================================================================================
 * Drain FIFO -> parse -> push packets into msgq
 * ===================================================================================== */
int icm42688_fifo_drain_to_queue(const struct device *dev)
{
	struct icm42688_data *data = dev->data;

	if (!data->fifo_msgq_inited || data->fifo_msgq == NULL) {
		return -ENOTSUP;
	}

	/* If we previously flagged overflow, flush + reset parsing */
	if (data->fifo_overflowed) {
		(void)icm42688_fifo_flush(dev);
		(void)icm42688_fifo_queue_reset(dev);
		data->fifo_overflowed = false;
	}

	/* Read + parse in chunks until FIFO count becomes small/zero.
	 * We do bounded work per call to keep IRQ work handler short.
	 */
	return fifo_read_and_enqueue_some(dev);
}

/* =====================================================================================
 * Read a bounded chunk (<=256 bytes), parse packets, enqueue.
 * Uses your robust drain_and_parse().
 * ===================================================================================== */
static int fifo_read_and_enqueue_some(const struct device *dev)
{
	struct icm42688_data *data = dev->data;

	/* Parse into local packet array */
	struct icm42688_fifo_data pkts[32];  /* Increased from 16 */
	size_t n = 0;

	/* max_bytes=512 to read more data per call */
	int rc = icm42688_fifo_drain_and_parse(dev, pkts, ARRAY_SIZE(pkts), &n, 512);
	if (rc < 0) {
		if (rc != -EOVERFLOW) {
			LOG_ERR("drain_and_parse failed: %d", rc);
		}
		/* if parse says overflowed, mark and flush next time */
		if (rc == -EOVERFLOW) {
			data->fifo_overflowed = true;
		}
		return rc;
	}

	if (n == 0) {
		return 0;
	}
	
	LOG_DBG("Parsed %zu packets from FIFO", n);

	int enqueued = 0;
	for (size_t i = 0; i < n; i++) {
		if (!pkts[i].valid) {
			continue;
		}
		msgq_put_drop_oldest(data->fifo_msgq, &pkts[i]);
		enqueued++;
	}

	return enqueued;
}

#endif /* CONFIG_ICM42688_FIFO */
