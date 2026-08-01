/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610_efogtech

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zmk/keymap.h>
#include <zmk/events/activity_state_changed.h>
#include "pmw3610.h"

#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
#include <zmk_adaptive_feedback/adaptive_feedback.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_LOG_LEVEL);

#if IS_ENABLED(CONFIG_PMW3610_SQUAL_LOG)
static struct k_work_delayable squal_log_work;
static void pmw3610_log_squal_work(struct k_work *work);
#endif

#if IS_ENABLED(CONFIG_SHELL)
#include <zephyr/shell/shell.h>

struct pmw3610_squal_accum {
    bool active;
    uint16_t min;
    uint16_t max;
    uint32_t sum;
    uint32_t count;
};
static void pmw3610_squal_accum_sample(uint8_t id, uint8_t raw);
#endif

#if IS_ENABLED(CONFIG_SHELL)
#include <stdlib.h>
#endif

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // reset cs line and assert power-up reset
    ASYNC_INIT_STEP_CLEAR_OB1, // clear observation1 register for self-test check
    ASYNC_INIT_STEP_CHECK_OB1, // check the value of observation1 register after self-test check
    ASYNC_INIT_STEP_CONFIGURE, // set other registes like cpi and donwshift time (run, rest1, rest2)
                               // and clear motion registers

    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10 + CONFIG_PMW3610_INIT_POWER_UP_EXTRA_DELAY_MS, // >10ms needed
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200, // 150 us required, test shows too short,
                                       // also power-up reset is added in this step, thus using 50 ms
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,  // 10 ms required in spec,
                                       // test shows too short,
                                       // especially when integrated with display,
                                       // > 50ms is needed
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Function definitions //////////

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value, const size_t len) {
	const struct pixart_config *cfg = dev->config;
	const struct spi_buf tx_buf = { .buf = &addr, .len = sizeof(addr) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf rx_buf[] = {
		{ .buf = NULL, .len = sizeof(addr), },
		{ .buf = value, .len = len, },
	};
	const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };
	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int pmw3610_read_reg(const struct device *dev, const uint8_t addr, uint8_t *value) {
	return pmw3610_read(dev, addr, value, 1);
}

static int pmw3610_write_reg(const struct device *dev, const uint8_t addr, const uint8_t value) {
	const struct pixart_config *cfg = dev->config;
	uint8_t write_buf[] = {addr | SPI_WRITE_BIT, value};
	const struct spi_buf tx_buf = { .buf = write_buf, .len = sizeof(write_buf), };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1, };
	return spi_write_dt(&cfg->spi, &tx);
}

static int pmw3610_write(const struct device *dev, const uint8_t reg, const uint8_t val) {
	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    const int err = pmw3610_write_reg(dev, reg, val);
    if (unlikely(err != 0)) {
        return err;
    }
    
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    return 0;
}

#if IS_ENABLED(CONFIG_SHELL)
static int pmw3610_frame_capture(const struct device *dev, uint8_t *buf) {
    int err = pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (err) {
        return err;
    }

    k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    err = pmw3610_write_reg(dev, PMW3610_REG_TEST_CLOCK, PMW3610_TEST_CLOCK_CMD_ON);
    if (!err) {
        err = pmw3610_write_reg(dev, PMW3610_REG_FRAME_GRAB, PMW3610_FRAME_GRAB_CMD);
    }
    if (err) {
        return err;
    }

    k_sleep(K_MSEC(PMW3610_FRAME_GRAB_DELAY_MS));
    return pmw3610_read(dev, PMW3610_REG_MOTION_BURST, buf, PMW3610_FRAME_SIZE);
}
#endif

static int pmw3610_set_cpi(const struct device *dev, const uint32_t cpi) {
    /* Set resolution with CPI step of 200 cpi
     * 0x1: 200 cpi (minimum cpi)
     * 0x2: 400 cpi
     * 0x3: 600 cpi
     * :
     */

    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    // Convert CPI to register value
    const struct pixart_config *config = dev->config;
    uint8_t value = (cpi / 200);
    LOG_DBG("Setting CPI to %u (reg value 0x%x)", cpi, value);

    if (config->xy_swap) {
        value |= (1 << 7);
    }

    if (config->x_invert) {
        value |= (1 << 6);
    }

    if (config->y_invert) {
        value |= (1 << 5);
    }

    /* set the cpi */
    const uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    const uint8_t data[] = {0xFF, value, 0x00};

	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    /* Write data */
    int err = 0;
    for (size_t i = 0; i < sizeof(data); i++) {
        err = pmw3610_write_reg(dev, addr[i], data[i]);
        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            break;
        }
    }
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

    if (err) {
        LOG_ERR("Failed to set CPI");
        return err;
    }

    return 0;
}

/* Set sampling rate in each mode (in ms) */
static int pmw3610_set_sample_time(const struct device *dev, const uint8_t reg_addr, const uint32_t sample_time) {
    const uint32_t maxtime = 2550;
    const uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
        return -EINVAL;
    }

    const uint8_t value = sample_time / mintime;
    LOG_DBG("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

    /* The sample time is (reg_value * mintime ) ms. 0x00 is rounded to 0x1 */
    const int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }

    return err;
}

/* Set downshift time in ms. */
// NOTE: The unit of run-mode downshift is related to pos mode rate, which is hard coded to be 4 ms
// The pos-mode rate is configured in pmw3610_async_init_configure
static int pmw3610_set_downshift_time(const struct device *dev, const uint8_t reg_addr, const uint32_t time) {
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        /*
         * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                      * 8 * pos-rate (fixed to 4ms)
         */
        maxtime = 8160; // 32 * 255;
        mintime = 32; // hard-coded in pmw3610_async_init_configure
        break;

    case PMW3610_REG_REST1_DOWNSHIFT:
        /*
         * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                        * 16 * Rest1_sample_period (default 40 ms)
         */
        maxtime = 255 * 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        break;

    case PMW3610_REG_REST2_DOWNSHIFT:
        /*
         * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT
         *                        * 128 * Rest2 rate (default 100 ms)
         */
        maxtime = 255 * 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        break;

    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range (%u - %u)", time, mintime, maxtime);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    /* Convert time to register value */
    const uint8_t value = time / mintime;

    LOG_DBG("Set downshift time to %u ms (reg value 0x%x)", time, value);

    const int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

static int pmw3610_set_performance(const struct device *dev, const bool enabled) {
    const struct pixart_config *config = dev->config;
    int err = 0;

    if (config->force_awake) {
        uint8_t value;
        err = pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &value);
        if (err) {
            LOG_ERR("Can't read ref-performance %d", err);
            return err;
        }
        LOG_DBG("Get performance register (reg value 0x%x)", value);

        // Set prefered RUN RATE        
        //   BIT 3:   VEL_RUNRATE    0x0: 8ms; 0x1 4ms;
        //   BIT 2:   POSHI_RUN_RATE 0x0: 8ms; 0x1 4ms;
        //   BIT 1-0: POSLO_RUN_RATE 0x0: 8ms; 0x1 4ms; 0x2 2ms; 0x4 Reserved
        uint8_t perf = 0;
        if (config->force_high_performance) {
            perf = enabled
                ? 0x0e // RUN RATE @ 4/2ms
                : 0x00;// RUN RATE @ 8ms
        } else {
            perf = 0;
        }

        if (perf != value) {
            struct pixart_data *data = dev->data;
            data->data_index = 0;
            data->data_ready = false;

            err = pmw3610_write(dev, PMW3610_REG_PERFORMANCE, perf);
            if (err) {
                LOG_ERR("Can't write performance register %d", err);
                return err;
            }
            LOG_DBG("Set performance register (reg value 0x%x)", perf);
        }
        LOG_DBG("%s performance mode", enabled ? "enable" : "disable");
    }

    return err;
}

static int pmw3610_set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    const int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio, en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
    return ret;
}

static int pmw3610_async_init_power_up(const struct device *dev) {
	const int ret = pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    return pmw3610_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    uint8_t value;
    int err = pmw3610_read_reg(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Can't do self-test");
        return err;
    }

    if ((value & 0x0F) != 0x0F) {
        LOG_ERR("Failed self-test (0x%x)", value);

        const struct pixart_config *config = dev->config;
        LOG_ERR("Sensor ID#%d failed self-test", config->id);
        return -EINVAL;
    }

    uint8_t product_id = 0x01;
    err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);

        const struct pixart_config *config = dev->config;
        LOG_ERR("Sensor ID#%d doesn't seem to be connected", config->id);
        return -EIO;
    }

    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    int err = 0;
    const struct pixart_config *config = dev->config;

    // clear motion registers first (required in datasheet)
    for (uint8_t reg = 0x02; (reg <= 0x05) && !err; reg++) {
        uint8_t buf[1];
        err = pmw3610_read_reg(dev, reg, buf);
    }

    if (!err) {
        err = pmw3610_set_performance(dev, true);
    }

    if (!err) {
        err = pmw3610_set_cpi(dev, config->cpi);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                         CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                         CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                         CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE,
                                      CONFIG_PMW3610_REST1_SAMPLE_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE,
                                      CONFIG_PMW3610_REST2_SAMPLE_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE,
                                      CONFIG_PMW3610_REST3_SAMPLE_TIME_MS);
    }

    if (err) {
        LOG_ERR("Config the sensor failed");
        return err;
    }

#if IS_ENABLED(CONFIG_PMW3610_SQUAL_LOG)
    k_work_init_delayable(&squal_log_work, pmw3610_log_squal_work);
    k_work_schedule(&squal_log_work, K_MSEC(500 + CONFIG_PMW3610_SQUAL_LOG_INTERVAL));
#endif

    return 0;
}

static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;
    const struct pixart_config *config = dev->config;

    LOG_DBG("PMW3610 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("PMW3610 initialization failed in step %d", data->async_init_step);
        
        if (data->init_retry_attempts > 0) {
            data->init_retry_attempts--;
            data->init_retry_count++;
            LOG_WRN("PMW3610#%d retrying initialization (attempt %d/%d)", config->id, data->init_retry_count, config->init_retry_count);
            
#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
            // ReSharper disable once CppRedundantBooleanExpressionArgument
            if (data->init_retry_count >= CONFIG_PMW3610_INIT_FAILURE_THRESHOLD && CONFIG_PMW3610_INIT_FAILURE_THRESHOLD > 0 && !data->error_triggered) {
                zaf_error_trigger(config->id);
                data->error_triggered = true;
            }
#endif

            data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
            k_work_schedule(&data->init_work, K_MSEC(config->init_retry_interval));
        } else {
#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
            if (!data->error_triggered) {
                zaf_error_trigger(config->id);
                data->error_triggered = true;
            }
#endif

            LOG_ERR("PMW3610#%d initialization failed after %d attempts", config->id, config->init_retry_count);
        }
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true; 

#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
            zaf_error_clear(config->id);
#endif

            LOG_INF("PMW3610 initialized successfully");
            if (data->init_retry_count > 0) {
                LOG_INF("PMW3610 initialization succeeded after %d retries", data->init_retry_count);
            }
            pmw3610_set_interrupt(dev, true);
        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

#if IS_ENABLED(CONFIG_PMW3610_IGNORE_AFTER_REST) || IS_ENABLED(CONFIG_PMW3610_ANTI_WARP)
    const uint32_t now = k_uptime_get_32();
    const uint32_t passed = now - data->last_data;
#endif

#if IS_ENABLED(CONFIG_SHELL)
    if (unlikely(data->streaming)) {
        return 0;
    }
#endif

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    uint8_t buf[PMW3610_BURST_SIZE];
	const int err = pmw3610_read(dev, PMW3610_REG_MOTION_BURST, buf, PMW3610_BURST_SIZE);
    if (err) {
        return err;
    }
    // LOG_HEXDUMP_DBG(buf, PMW3610_BURST_SIZE, "buf");

#if IS_ENABLED(CONFIG_SHELL)
    pmw3610_squal_accum_sample(config->id, buf[4]);
#endif

    // 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

    int16_t x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
    int16_t y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);
	LOG_DBG("(sensor=%d) X: %4d, Y: %4d", config->id, x, y);

#if IS_ENABLED(CONFIG_PMW3610_SWAP_XY)
    int16_t a = x;
    x = y;
    y = a;
#endif
#if IS_ENABLED(CONFIG_PMW3610_INVERT_X)
    x = -x;
#endif
#if IS_ENABLED(CONFIG_PMW3610_INVERT_Y)
    y = -y;
#endif

#if IS_ENABLED(CONFIG_PMW3610_IGNORE_AFTER_REST)
    if (passed > CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS + CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS + CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS) {
        data->data_index = 0;
        data->data_ready = false;
    }
#endif

#if IS_ENABLED(CONFIG_PMW3610_ANTI_WARP)
    if (passed > CONFIG_PMW3610_ANTI_WARP_INACTIVITY_MS &&
        (x > CONFIG_PMW3610_ANTI_WARP_THRES || y > CONFIG_PMW3610_ANTI_WARP_THRES) &&
        data->data_ready) {
        data->last_data = now;
        data->data_index = CONFIG_PMW3610_IGNORE_FIRST_N / 2;
        data->data_ready = false;
        LOG_WRN("Discarded large movement after inactivity, likely warping");
    }
#endif

#ifdef CONFIG_PMW3610_SMART_ALGORITHM
    const int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8)
                    + buf[PMW3610_SHUTTER_L_POS];
    if (data->sw_smart_flag && shutter < 45) {
        pmw3610_write(dev, 0x32, 0x00);
        data->sw_smart_flag = false;
    }
    if (!data->sw_smart_flag && shutter > 45) {
        pmw3610_write(dev, 0x32, 0x80);
        data->sw_smart_flag = true;
    }
#endif

#if IS_ENABLED(CONFIG_PMW3610_IGNORE_AFTER_REST) || IS_ENABLED(CONFIG_PMW3610_ANTI_WARP)
    data->last_data = now;
#endif

    if (!data->data_ready) {
        if (++data->data_index >= CONFIG_PMW3610_IGNORE_FIRST_N) {
            data->data_ready = true;
        }
        return 0;
    }

    // accumulate delta until report in next iteration
    data->dx += x;
    data->dy += y;

    // fetch report value
    const int16_t rx = (int16_t)CLAMP(data->dx, INT16_MIN, INT16_MAX);
    const int16_t ry = (int16_t)CLAMP(data->dy, INT16_MIN, INT16_MAX);
    const bool have_x = rx != 0;
    const bool have_y = ry != 0;

    if (have_x || have_y) {
        data->dx = 0;
        data->dy = 0;
        if (have_x) {
            input_report(dev, config->evt_type, config->x_input_code, rx, !have_y, K_NO_WAIT);
        }
        if (have_y) {
            input_report(dev, config->evt_type, config->y_input_code, ry, true, K_NO_WAIT);
        }
    }

    return err;
}

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    pmw3610_set_interrupt(dev, false);
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    const struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;
    pmw3610_report_data(dev);
    pmw3610_set_interrupt(dev, true);
}

static int pmw3610_init_irq(const struct device *dev) {
    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    // check readiness of irq gpio pin
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    // init the irq pin
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    // setup and add the irq callback associated
    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    return err;
}

static int pmw3610_init(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("%s is not ready", config->spi.bus->name);
		return -ENODEV;
	}

    data->dev = dev;
    data->sw_smart_flag = false;

    data->init_retry_count = 0;
    data->init_retry_attempts = config->init_retry_count;
    data->dx = data->dy = 0;

    // init trigger handler work
    k_work_init(&data->trigger_work, pmw3610_work_callback);

    // init irq routine
    const int err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    if (config->rst_gpio.port != NULL) {
        if (gpio_pin_configure_dt(&config->rst_gpio, GPIO_OUTPUT) != 0) {
            LOG_ERR("Failed to configure RST GPIO");
        } else {
            gpio_pin_set_dt(&config->rst_gpio, 0);
        }
    }

    // Setup delayable and non-blocking init jobs, including following steps:
    // 1. power reset
    // 2. upload initial settings
    // 3. other configs like cpi, downshift time, sample time etc.
    // The sensor is ready to work (i.e., data->ready=true after the above steps are finished)
    k_work_init_delayable(&data->init_work, pmw3610_async_init);
    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

#if IS_ENABLED(CONFIG_PM_DEVICE)
    if (pm_device_wakeup_is_capable(dev)) {
        pm_device_wakeup_enable(dev, true);
    }
#endif

    return err;
}

static int pmw3610_attr_set(const struct device *dev, const enum sensor_channel chan,
                            const enum sensor_attribute attr, const struct sensor_value *val) {
    const struct pixart_data *data = dev->data;
    int err;

    if (unlikely(chan != SENSOR_CHAN_ALL)) {
        return -ENOTSUP;
    }

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    switch ((uint32_t)attr) {
    case PMW3610_ATTR_CPI:
        err = pmw3610_set_cpi(dev, PMW3610_SVALUE_TO_CPI(*val));
        break;

    case PMW3610_ATTR_RUN_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST1_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST2_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST1_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST2_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ATTR_REST3_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }

    return err;
}

static const struct sensor_driver_api pmw3610_driver_api = {
    .attr_set = pmw3610_attr_set,
};

#if IS_ENABLED(CONFIG_PM_DEVICE)
static int pmw3610_shutdown(const struct device *dev);

static int pmw3610_pm_action(const struct device *dev, const enum pm_device_action action) {
    const struct pixart_config *config = dev->config;
    struct pixart_data *data = dev->data;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND: {
        if (pm_device_wakeup_is_enabled(dev)) {
            return 0;
        }
        const int ret = pmw3610_set_interrupt(dev, false);
        if (ret < 0) {
            return ret;
        }
        data->ready = false;
        return pmw3610_shutdown(dev);
    }
    case PM_DEVICE_ACTION_RESUME:
        if (config->rst_gpio.port) {
            gpio_pin_set_dt(&config->rst_gpio, 1);
            k_sleep(K_MSEC(1));
            gpio_pin_set_dt(&config->rst_gpio, 0);
        }
        data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
        data->init_retry_count = 0;
        data->init_retry_attempts = config->init_retry_count;
        return k_work_schedule(&data->init_work,
                               K_MSEC(async_init_delay[ASYNC_INIT_STEP_POWER_UP])) < 0 ? -EIO : 0;
    default:
        return -ENOTSUP;
    }
}
#endif

#define PMW3610_SPI_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | \
                        SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define PMW3610_DEFINE(n)                                                                               \
    static struct pixart_data data##n;                                                                  \
    static const struct pixart_config config##n = {                                                     \
        .id = n,                                                                                        \
		.spi = SPI_DT_SPEC_INST_GET(n, PMW3610_SPI_MODE, 0),		                                    \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                                \
        .rst_gpio = GPIO_DT_SPEC_INST_GET_OR(n, rst_gpios, { .port = NULL }),                           \
        .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                                            \
        .evt_type = DT_PROP(DT_DRV_INST(n), evt_type),                                                  \
        .x_input_code = DT_PROP(DT_DRV_INST(n), x_input_code),                                          \
        .y_input_code = DT_PROP(DT_DRV_INST(n), y_input_code),                                          \
        .xy_swap = DT_PROP(DT_DRV_INST(n), xy_swap),                                                    \
        .x_invert = DT_PROP(DT_DRV_INST(n), x_invert),                                                  \
        .y_invert = DT_PROP(DT_DRV_INST(n), y_invert),                                                  \
        .force_awake = DT_PROP(DT_DRV_INST(n), force_awake),                                            \
        .force_high_performance = DT_PROP(DT_DRV_INST(n), force_high_performance),                      \
        .init_retry_count = DT_PROP(DT_DRV_INST(n), init_retry_count),                                  \
        .init_retry_interval = DT_PROP(DT_DRV_INST(n), init_retry_interval),                            \
    };                                                                                                  \
    PM_DEVICE_DT_INST_DEFINE(n, pmw3610_pm_action);                                                     \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, PM_DEVICE_DT_INST_GET(n), &data##n, &config##n, POST_KERNEL, \
                          CONFIG_INPUT_PMW3610_INIT_PRIORITY, &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)

#define GET_PMW3610_DEV(node_id) DEVICE_DT_GET(node_id),

static const struct device *pmw3610_devs[] = {
    DT_FOREACH_STATUS_OKAY(pixart_pmw3610_efogtech, GET_PMW3610_DEV)
};

static int pmw3610_shutdown(const struct device *dev) {
    return pmw3610_write_reg(dev, PMW3610_REG_SHUTDOWN, PMW3610_REG_SHUTDOWN_CMD);
}

static uint8_t prev_state = 0;
static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);

    if (!state_ev) {
        LOG_WRN("NO EVENT, leaving early");
        return 0;
    }

    LOG_DBG("PM: %d → %d", prev_state, state_ev->state);

    const bool enable = state_ev->state == ZMK_ACTIVITY_ACTIVE;
    for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
        const struct device *dev = pmw3610_devs[i];
        const struct pixart_config *config = dev->config;
        struct pixart_data *data = dev->data;

#if IS_ENABLED(CONFIG_SHELL)
        if (data->streaming) {
            continue;
        }
#endif

        bool wakeup = false;
#if IS_ENABLED(CONFIG_PM_DEVICE)
        wakeup = pm_device_wakeup_is_enabled(dev);
#endif

        pmw3610_set_performance(dev, enable);
        if (data->ready) {
            pmw3610_set_interrupt(dev, enable || wakeup);

            if (!enable && !wakeup) {
                LOG_WRN("Powering down sensor ID%d", config->id);
                if (pmw3610_shutdown(dev) != 0) {
                    LOG_ERR("Failed to power down sensor ID%d", config->id);
                }
            } else if (enable && prev_state != ZMK_ACTIVITY_ACTIVE && !wakeup) {
                LOG_WRN("Powering up sensor ID%d", config->id);
                data->async_init_step = 0;
                k_work_schedule(&data->init_work, K_MSEC(async_init_delay[0]));
                prev_state = state_ev->state;
                return 0;
            }
        }

        if (!enable) {
            data->data_ready = false;
            data->data_index = 0;
        }
    }

    prev_state = state_ev->state;
    return 0;
}

#if IS_ENABLED(CONFIG_PMW3610_SQUAL_LOG)
static void pmw3610_log_squal_work(struct k_work *work) {
    for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
        const struct device *dev = pmw3610_devs[i];
        const struct pixart_data *data = dev->data;

        if (data->ready) {
            uint8_t squal_value;
            const int err = pmw3610_read_reg(dev, PMW3610_REG_SQUAL, &squal_value);
            const uint16_t corrected_squal = ((uint16_t) squal_value) << 1;

            if (err == 0) {
                if (corrected_squal != 0) {
                    LOG_DBG("ID#%d surface_quality: %d/361", i, corrected_squal);
                }

                if (corrected_squal < 50) {
                    LOG_ERR("ID#%d no surface detected", i);
                } else if (corrected_squal < 210) {
                    LOG_WRN("ID#%d check sensor: bad surface quality (%d/361), expect warping", i, corrected_squal);
                } else if (corrected_squal < 230) {
                    LOG_WRN("ID#%d surface quality is sub-optimal (%d/361), warping is possible", i, corrected_squal);
                }
            } else {
                LOG_ERR("ID#%d failed to read SQUAL register: %d", i, err);
            }
        }
    }
    
    k_work_schedule(&squal_log_work, K_MSEC(CONFIG_PMW3610_SQUAL_LOG_INTERVAL));
}
#endif

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);

#if IS_ENABLED(CONFIG_SHELL)
static struct pmw3610_squal_accum squal_accum[ARRAY_SIZE(pmw3610_devs)];
static struct k_spinlock squal_accum_lock;

static void pmw3610_squal_accum_sample(const uint8_t id, const uint8_t raw) {
    if (id >= ARRAY_SIZE(squal_accum)) {
        return;
    }
    struct pmw3610_squal_accum *a = &squal_accum[id];
    if (!a->active) {
        return;
    }
    const uint16_t corrected = ((uint16_t) raw) << 1;
    const k_spinlock_key_t key = k_spin_lock(&squal_accum_lock);
    if (a->active) {
        if (a->count == 0 || corrected < a->min) {
            a->min = corrected;
        }
        if (a->count == 0 || corrected > a->max) {
            a->max = corrected;
        }
        a->sum += corrected;
        a->count++;
    }
    k_spin_unlock(&squal_accum_lock, key);
}

static int cmd_sensor_surface(const struct shell *sh, const size_t argc, char **argv) {
    if (argc == 1) {
        for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
            const struct device *dev = pmw3610_devs[i];
            const struct pixart_config *config = dev->config;
            const struct pixart_data *data = dev->data;
            if (!data->ready) {
                shell_print(sh, "Sensor #%u: not ready", config->id);
                continue;
            }
            uint8_t raw;
            const int err = pmw3610_read_reg(dev, PMW3610_REG_SQUAL, &raw);
            if (err) {
                shell_error(sh, "Sensor #%u: read failed (error %d)", config->id, err);
                continue;
            }
            const uint16_t corrected = ((uint16_t) raw) << 1;
            shell_print(sh, "Sensor #%u: surface quality = %u/361", config->id, corrected);
        }
        return 0;
    }

    if (argc == 3 && strcmp(argv[1], "--accum-ms") == 0) {
        char *end;
        const unsigned long ms = strtoul(argv[2], &end, 10);
        if (*end != '\0' || ms == 0 || ms > 60000) {
            shell_error(sh, "Invalid duration: %s (1..60000 ms)", argv[2]);
            return -EINVAL;
        }

        k_spinlock_key_t key = k_spin_lock(&squal_accum_lock);
        for (size_t i = 0; i < ARRAY_SIZE(squal_accum); i++) {
            squal_accum[i] = (struct pmw3610_squal_accum){ .active = true };
        }
        k_spin_unlock(&squal_accum_lock, key);
        k_msleep((int32_t) ms);

        struct pmw3610_squal_accum snapshot[ARRAY_SIZE(squal_accum)];
        key = k_spin_lock(&squal_accum_lock);
        for (size_t i = 0; i < ARRAY_SIZE(squal_accum); i++) {
            squal_accum[i].active = false;
            snapshot[i] = squal_accum[i];
        }
        k_spin_unlock(&squal_accum_lock, key);

        for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
            const struct pixart_config *config = pmw3610_devs[i]->config;
            const struct pmw3610_squal_accum *a = &snapshot[i];
            if (a->count == 0) {
                shell_print(sh, "Sensor #%u: no samples (move pointer during the test)", config->id);
                continue;
            }
            const uint32_t avg = a->sum / a->count;
            shell_print(sh, "Sensor #%u: %u/%u/%u of 361 (min/max/avg) over %u samples",
                        config->id, a->min, a->max, avg, a->count);
        }
        return 0;
    }

    shell_error(sh, "usage: sensor surface [--accum-ms N]");
    return -EINVAL;
}

#define PMW3610_STREAM_TEXT_SIZE (PMW3610_FRAME_DIM * (PMW3610_FRAME_DIM * 2 + 1) + 32)

static K_THREAD_STACK_DEFINE(stream_stack, CONFIG_PMW3610_STREAM_STACK_SIZE);
static struct k_thread stream_thread;
static const struct shell *stream_sh;
static uint8_t *stream_buf; // frame bytes followed by the text buffer, allocated on stream start
static volatile bool stream_active;

static void pmw3610_stream_emit(const uint8_t id, const uint32_t seq, const uint8_t *frame,
                                char *text) {
    static const char hex[] = "0123456789ABCDEF";
    char *p = text + snprintk(text, 32, "F %u %08u\n", id, seq);

    for (size_t row = 0; row < PMW3610_FRAME_DIM; row++) {
        const uint8_t *pixels = &frame[row * PMW3610_FRAME_DIM];
        for (size_t col = 0; col < PMW3610_FRAME_DIM; col++) {
            *p++ = hex[pixels[col] >> 4];
            *p++ = hex[pixels[col] & 0x0F];
        }
        *p++ = '\n';
    }
    *p = '\0';

    shell_fprintf(stream_sh, SHELL_NORMAL, "%sEND\n", text);
}

static void pmw3610_stream_thread_fn(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    uint8_t *frame = stream_buf;
    char *text = (char *)(stream_buf + PMW3610_FRAME_SIZE);
    uint32_t seq = 0;

    while (stream_active) {
        for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs) && stream_active; i++) {
            const struct device *dev = pmw3610_devs[i];
            const struct pixart_config *config = dev->config;
            const struct pixart_data *data = dev->data;

            if (!data->streaming) {
                continue;
            }

            const int err = pmw3610_frame_capture(dev, frame);
            if (err) {
                shell_fprintf(stream_sh, SHELL_ERROR, "E %u %d\n", config->id, err);
                k_msleep(10);
                continue;
            }

            pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
            pmw3610_stream_emit(config->id, ++seq, frame, text);
        }
    }
}

static int pmw3610_stream_start(const struct shell *sh) {
    if (stream_active) {
        return -EALREADY;
    }

    stream_buf = malloc(PMW3610_FRAME_SIZE + PMW3610_STREAM_TEXT_SIZE);
    if (stream_buf == NULL) {
        LOG_ERR("Cannot allocate buffer!");
        return -ENOMEM;
    }

    size_t count = 0;
    for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
        const struct device *dev = pmw3610_devs[i];
        const struct pixart_config *config = dev->config;
        struct pixart_data *data = dev->data;

        if (!data->ready) {
            continue;
        }

        pmw3610_set_interrupt(dev, false);
        pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
        pmw3610_write_reg(dev, PMW3610_REG_PERFORMANCE, PMW3610_PERF_FRAME_CAPTURE);
        pmw3610_write_reg(dev, PMW3610_REG_TEST_CLOCK, PMW3610_TEST_CLOCK_CMD_ON);
        pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
        data->streaming = true;
        data->ready = false;

        struct k_work_sync sync;
        k_work_flush(&data->trigger_work, &sync);
        count++;
    }

    if (count == 0) {
        free(stream_buf);
        stream_buf = NULL;
        return -ENODEV;
    }

    stream_sh = sh;
    stream_active = true;
    k_thread_create(&stream_thread, stream_stack, K_THREAD_STACK_SIZEOF(stream_stack),
        pmw3610_stream_thread_fn, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&stream_thread, "pmw3610_stream");
    return 0;
}

static int pmw3610_stream_stop(const struct shell *sh) {
    if (!stream_active) {
        shell_error(sh, "Stream is not active");
        return -EALREADY;
    }

    stream_active = false;
    k_thread_join(&stream_thread, K_FOREVER);
    free(stream_buf);
    stream_buf = NULL;
    stream_sh = NULL;

    for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
        const struct device *dev = pmw3610_devs[i];
        const struct pixart_config *config = dev->config;
        struct pixart_data *data = dev->data;

        if (!data->streaming) {
            continue;
        }

        data->streaming = false;
        data->data_ready = false;
        data->data_index = 0;
        data->dx = data->dy = 0;

        if (config->rst_gpio.port) {
            gpio_pin_set_dt(&config->rst_gpio, 1);
            k_sleep(K_MSEC(1));
            gpio_pin_set_dt(&config->rst_gpio, 0);
        }

        data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
        data->init_retry_count = 0;
        data->init_retry_attempts = config->init_retry_count;
        k_work_schedule(&data->init_work, K_MSEC(async_init_delay[ASYNC_INIT_STEP_POWER_UP]));
        shell_print(sh, "Sensor #%u: reinitializing", config->id);
    }

    return 0;
}

static int cmd_sensor_stream(const struct shell *sh, const size_t argc, char **argv) {
    if (argc == 1) {
        shell_error(sh, "usage: sensor stream [--on|--off]");
        return -EINVAL;
    }

    if (strcmp(argv[1], "--on") == 0) {
        return pmw3610_stream_start(sh);
    }

    if (strcmp(argv[1], "--off") == 0) {
        return pmw3610_stream_stop(sh);
    }

    shell_error(sh, "usage: sensor stream [--on|--off]");
    return -EINVAL;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sensor,
    SHELL_CMD_ARG(surface, NULL, "Read surface quality. Usage: surface [--accum-ms N]", cmd_sensor_surface, 1, 2),
    SHELL_CMD_ARG(stream, NULL, "Raw frame capture. Usage: stream [--on|--off]", cmd_sensor_stream, 1, 1),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(sensor, &sub_sensor, "Sensor diagnostics", NULL);

#endif
