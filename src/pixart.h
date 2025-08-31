#pragma once

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/* device data structure */
struct pixart_data {
    const struct device          *dev;
    bool                         sw_smart_flag; // for pmw3610 smart algorithm

    struct gpio_callback         irq_gpio_cb; // motion pin irq callback
    struct k_work                trigger_work; // realtrigger job

    struct k_work_delayable      init_work; // the work structure for delayable init steps
    int                          async_init_step;
    int                          init_retry_count; // current retry count
    int                          init_retry_attempts; // remaining retry attempts

    bool                         ready; // whether init is finished successfully
    int                          err; // error code during async init

    bool                         data_ready;
    uint8_t                      data_index;

#if IS_ENABLED(CONFIG_PMW3610_IGNORE_AFTER_REST) || IS_ENABLED(CONFIG_PMW3610_ANTI_WARP)
    uint64_t                     last_data;
#endif
};

// device config data structure
struct pixart_config {
    uint8_t id;
	struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    struct gpio_dt_spec rst_gpio;
    uint16_t cpi;
    uint8_t evt_type;
    uint8_t x_input_code;
    uint8_t y_input_code;
    bool xy_swap;
    bool x_invert;
    bool y_invert;
    bool force_awake;
    bool force_high_performance;
    bool enable_pm_support;
    uint8_t init_retry_count;
    uint16_t init_retry_interval;
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
