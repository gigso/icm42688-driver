/* SPDX-License-Identifier: Apache-2.0 */
#include "icm42688.h"
#include "icm42688_reg.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(icm42688_trigger, CONFIG_SENSOR_LOG_LEVEL);

#ifdef CONFIG_ICM42688_TRIGGER

static int icm42688_route_drdy_int1(const struct device *dev, bool en)
{
	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	uint8_t mask = ICM42688_INT_SRC_UI_DRDY_INT1_EN;
	uint8_t val  = en ? ICM42688_INT_SRC_UI_DRDY_INT1_EN : 0;

	return icm42688_reg_update8(dev, ICM42688_REG_INT_SOURCE0, mask, val);
}


static void icm42688_work_handler(struct k_work *work)
{
	struct icm42688_data *data =
		CONTAINER_OF(work, struct icm42688_data, work);
	const struct device *dev = data->dev;

	(void)icm42688_reg_bank_sel(dev, ICM42688_BANK0);

	/* INT_STATUS is R/C (read clears flags) */
	uint8_t st0 = 0;
	if (icm42688_read_reg(dev, ICM42688_REG_INT_STATUS, &st0) < 0) {
		return;
	}

	/* INT_STATUS2 holds WOM flags */
	uint8_t st2 = 0;
	(void)icm42688_read_reg(dev, ICM42688_REG_INT_STATUS2, &st2);

#if defined(CONFIG_ICM42688_FIFO)
	/* ===== FIFO FULL = overflow ===== */
	if (st0 & ICM42688_INT_STATUS_FIFO_FULL_INT) {

		LOG_WRN("FIFO FULL interrupt");

		data->fifo_overflowed = true;
		data->fifo_tail_len = 0;

		(void)icm42688_fifo_flush(dev);
		(void)icm42688_fifo_queue_reset(dev);

		if (data->fifo_full_handler) {
			data->fifo_full_handler(dev, &data->fifo_full_trig);
		}

		/* После FIFO_FULL ничего больше не обрабатываем */
		return;
	}

	/* ===== FIFO WATERMARK ===== */
	if (st0 & ICM42688_INT_STATUS_FIFO_THS_INT) {

		/* Drain FIFO -> push packets to queue */
		(void)icm42688_fifo_drain_to_queue(dev);

		if (data->fifo_wm_handler) {
			data->fifo_wm_handler(dev, &data->fifo_wm_trig);
		}
	}
#endif /* CONFIG_ICM42688_FIFO */


	/* ===== DATA READY =====
	 * Обычно не используется при FIFO, но оставим для совместимости
	 */
	if ((st0 & ICM42688_INT_STATUS_DATA_RDY_INT) &&
	    data->drdy_handler) {

#if !defined(CONFIG_ICM42688_FIFO)
		data->drdy_handler(dev, &data->drdy_trig);
#endif
	}

	/* ===== Wake-on-Motion ===== */
	uint8_t wom_mask =
		ICM42688_INT_STATUS2_WOM_X_INT |
		ICM42688_INT_STATUS2_WOM_Y_INT |
		ICM42688_INT_STATUS2_WOM_Z_INT;

	if ((st2 & wom_mask) && data->wom_handler) {
		data->wom_handler(dev, &data->wom_trig);
	}
}

static void icm42688_gpio_cb(const struct device *port,
			    struct gpio_callback *cb,
			    uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct icm42688_data *data =
		CONTAINER_OF(cb, struct icm42688_data, int_cb);

	k_work_submit(&data->work);
}

int icm42688_trigger_init(const struct device *dev)
{
	struct icm42688_data *data = dev->data;
	const struct icm42688_config *cfg = dev->config;

	data->dev = dev;
	k_work_init(&data->work, icm42688_work_handler);

	if (cfg->int_gpio.port == NULL) {
		return -ENOTSUP;
	}

	if (!device_is_ready(cfg->int_gpio.port)) {
		return -ENOTSUP;
	}

	int rc = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (rc < 0) {
		return rc;
	}

	rc = gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
					    GPIO_INT_EDGE_TO_ACTIVE);
	if (rc < 0) {
		return rc;
	}

	gpio_init_callback(&data->int_cb,
			   icm42688_gpio_cb,
			   BIT(cfg->int_gpio.pin));

	rc = gpio_add_callback(cfg->int_gpio.port, &data->int_cb);
	if (rc < 0) {
		return rc;
	}

	/* Optional: Initialize INT2 if present */
	if (cfg->int2_gpio.port != NULL) {
		if (!device_is_ready(cfg->int2_gpio.port)) {
			LOG_WRN("INT2 GPIO port not ready, continuing without INT2");
			return 0;
		}

		rc = gpio_pin_configure_dt(&cfg->int2_gpio, GPIO_INPUT);
		if (rc < 0) {
			LOG_WRN("Failed to configure INT2 pin: %d", rc);
			return 0;
		}

		rc = gpio_pin_interrupt_configure_dt(&cfg->int2_gpio,
						    GPIO_INT_EDGE_TO_ACTIVE);
		if (rc < 0) {
			LOG_WRN("Failed to configure INT2 interrupt: %d", rc);
			return 0;
		}

		gpio_init_callback(&data->int2_cb,
				   icm42688_gpio_cb,
				   BIT(cfg->int2_gpio.pin));

		rc = gpio_add_callback(cfg->int2_gpio.port, &data->int2_cb);
		if (rc < 0) {
			LOG_WRN("Failed to add INT2 callback: %d", rc);
			return 0;
		}

		LOG_INF("INT2 GPIO initialized on pin %d", cfg->int2_gpio.pin);
	}

	return 0;
}

int icm42688_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	struct icm42688_data *data = dev->data;
	int rc = 0;

	switch (trig->type) {

	case SENSOR_TRIG_DATA_READY:
		data->drdy_handler = handler;
		if (handler) {
			data->drdy_trig = *trig;
			/* Реально включаем DRDY -> INT1 */
			rc = icm42688_route_drdy_int1(dev, true);
		} else {
			/* Выключаем DRDY routing */
			rc = icm42688_route_drdy_int1(dev, false);
		}
		return rc;

	case SENSOR_TRIG_MOTION:
		data->wom_handler = handler;
		if (handler) {
			data->wom_trig = *trig;
		}
		/* WOM routing/enable делается через attr_set(WOM_ENABLE) -> wom_enable(),
		 * поэтому здесь железо не трогаем.
		 */
		return 0;

#if defined(CONFIG_ICM42688_FIFO)
	case SENSOR_TRIG_FIFO_WATERMARK:
		data->fifo_wm_handler = handler;
		if (handler) {
			data->fifo_wm_trig = *trig;
		}
		/* FIFO routing делается в fifo_init() */
		return 0;

	case SENSOR_TRIG_FIFO_FULL:
		data->fifo_full_handler = handler;
		if (handler) {
			data->fifo_full_trig = *trig;
		}
		/* FIFO routing делается в fifo_init() */
		return 0;
#endif

	default:
		return -ENOTSUP;
	}
}


#endif /* CONFIG_ICM42688_TRIGGER */
