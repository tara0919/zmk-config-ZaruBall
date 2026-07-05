/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_inertial_scroll

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>

#include <drivers/input_processor.h>
#include <zmk/endpoints.h>
#include <zmk/hid.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define VELOCITY_SCALE 100

struct inertial_scroll_config {
    uint8_t type;
    size_t codes_len;
    uint16_t interval_ms;
    uint16_t gain_percent;
    int16_t start_threshold;
    int16_t burst_threshold;
    uint16_t burst_timeout_ms;
    uint16_t burst_window_ms;
    uint16_t release_ms;
    uint8_t decay_percent;
    int16_t stop_threshold;
    int16_t max_step;
    uint16_t codes[];
};

struct inertial_scroll_data {
    const struct device *dev;
    struct k_work_delayable work;
    int32_t velocity;
    int32_t remainder;
    int32_t burst_accum;
    int64_t last_input_ms;
    int64_t burst_start_ms;
    int8_t burst_dir;
    uint16_t code;
};

static bool handles_code(const struct inertial_scroll_config *cfg, uint16_t code) {
    for (size_t i = 0; i < cfg->codes_len; i++) {
        if (cfg->codes[i] == code) {
            return true;
        }
    }

    return false;
}

static int32_t abs32(int32_t value) {
    return value < 0 ? -value : value;
}

static int8_t sign32(int32_t value) {
    if (value > 0) {
        return 1;
    }

    if (value < 0) {
        return -1;
    }

    return 0;
}

static void stop_inertia(struct inertial_scroll_data *data) {
    data->velocity = 0;
    data->remainder = 0;
}

static int32_t input_to_velocity(const struct inertial_scroll_config *cfg, int8_t dir,
                                 int32_t amount) {
    return (dir * amount * VELOCITY_SCALE * cfg->gain_percent) / 100;
}

static int16_t limit_step(const struct inertial_scroll_config *cfg, int32_t step) {
    if (cfg->max_step <= 0) {
        return step;
    }

    if (step > cfg->max_step) {
        return cfg->max_step;
    }

    if (step < -cfg->max_step) {
        return -cfg->max_step;
    }

    return step;
}

static int send_scroll_report(uint16_t code, int16_t step) {
    switch (code) {
    case INPUT_REL_WHEEL:
        zmk_hid_mouse_scroll_set(0, step);
        break;
    case INPUT_REL_HWHEEL:
        zmk_hid_mouse_scroll_set(step, 0);
        break;
    default:
        return -ENOTSUP;
    }

    zmk_endpoints_send_mouse_report();
    zmk_hid_mouse_scroll_set(0, 0);

    return 0;
}

static void inertial_scroll_work_handler(struct k_work *work) {
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct inertial_scroll_data *data =
        CONTAINER_OF(delayable, struct inertial_scroll_data, work);
    const struct device *dev = data->dev;
    const struct inertial_scroll_config *cfg = dev->config;

    if (data->velocity == 0) {
        if (data->burst_accum < cfg->burst_threshold) {
            return;
        }

        data->velocity = input_to_velocity(cfg, data->burst_dir, data->burst_accum);
        data->remainder = 0;
        data->burst_accum = 0;
    }

    data->velocity = (data->velocity * cfg->decay_percent) / 100;
    if (abs32(data->velocity) < cfg->stop_threshold) {
        stop_inertia(data);
        return;
    }

    data->remainder += data->velocity;

    int32_t raw_step = data->remainder / VELOCITY_SCALE;

    if (raw_step == 0) {
        k_work_reschedule(&data->work, K_MSEC(cfg->interval_ms));
        return;
    }

    int16_t step = limit_step(cfg, raw_step);
    data->remainder -= step * VELOCITY_SCALE;

    int err = send_scroll_report(data->code, step);
    if (err < 0) {
        LOG_WRN("Failed to send inertial scroll: %d", err);
        stop_inertia(data);
        return;
    }

    k_work_reschedule(&data->work, K_MSEC(cfg->interval_ms));
}

static int inertial_scroll_handle_event(const struct device *dev, struct input_event *event,
                                        uint32_t param1, uint32_t param2,
                                        struct zmk_input_processor_state *state) {
    struct inertial_scroll_data *data = dev->data;
    const struct inertial_scroll_config *cfg = dev->config;

    if (event->type != cfg->type || !handles_code(cfg, event->code)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->value == 0) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    const int8_t input_dir = sign32(event->value);
    const int8_t inertia_dir = sign32(data->velocity);
    const int64_t now = k_uptime_get();

    if (data->velocity != 0) {
        if (input_dir == inertia_dir) {
            data->velocity = input_to_velocity(cfg, input_dir, cfg->burst_threshold);
            data->code = event->code;
            data->burst_accum = 0;
            data->burst_dir = input_dir;
            data->burst_start_ms = now;
            data->last_input_ms = now;
            k_work_reschedule(&data->work, K_MSEC(cfg->interval_ms));
            return ZMK_INPUT_PROC_CONTINUE;
        } else {
            stop_inertia(data);
            data->burst_accum = 0;
            data->burst_dir = input_dir;
            data->burst_start_ms = now;
            data->last_input_ms = now;
            data->code = event->code;
            return ZMK_INPUT_PROC_STOP;
        }
    }

    if (abs32(event->value) < cfg->start_threshold) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (data->burst_dir != input_dir || data->code != event->code ||
        now - data->last_input_ms > cfg->burst_timeout_ms) {
        data->burst_accum = 0;
        data->burst_dir = input_dir;
        data->burst_start_ms = now;
    } else if (now - data->burst_start_ms > cfg->burst_window_ms) {
        data->burst_accum = 0;
        data->burst_start_ms = now;
    }

    data->last_input_ms = now;
    data->code = event->code;
    data->burst_accum += abs32(event->value);

    k_work_reschedule(&data->work, K_MSEC(cfg->release_ms));

    return ZMK_INPUT_PROC_CONTINUE;
}

static int inertial_scroll_init(const struct device *dev) {
    struct inertial_scroll_data *data = dev->data;

    data->dev = dev;
    k_work_init_delayable(&data->work, inertial_scroll_work_handler);

    return 0;
}

static struct zmk_input_processor_driver_api inertial_scroll_driver_api = {
    .handle_event = inertial_scroll_handle_event,
};

#define INERTIAL_SCROLL_INST(n)                                                                    \
    static const struct inertial_scroll_config inertial_scroll_config_##n = {                      \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                            \
        .codes_len = DT_INST_PROP_LEN(n, codes),                                                   \
        .interval_ms = DT_INST_PROP_OR(n, interval_ms, 16),                                        \
        .gain_percent = DT_INST_PROP_OR(n, gain_percent, 100),                                     \
        .start_threshold = DT_INST_PROP_OR(n, start_threshold, 1),                                  \
        .burst_threshold = DT_INST_PROP_OR(n, burst_threshold, 1),                                  \
        .burst_timeout_ms = DT_INST_PROP_OR(n, burst_timeout_ms, 120),                              \
        .burst_window_ms = DT_INST_PROP_OR(n, burst_window_ms, 80),                                  \
        .release_ms = DT_INST_PROP_OR(n, release_ms, 40),                                           \
        .decay_percent = DT_INST_PROP_OR(n, decay_percent, 78),                                    \
        .stop_threshold = DT_INST_PROP_OR(n, stop_threshold, 35),                                  \
        .max_step = DT_INST_PROP_OR(n, max_step, 4),                                               \
        .codes = DT_INST_PROP(n, codes),                                                           \
    };                                                                                             \
    static struct inertial_scroll_data inertial_scroll_data_##n = {};                              \
    BUILD_ASSERT(DT_INST_PROP_OR(n, gain_percent, 100) > 0,                                        \
                 "gain-percent must be greater than 0");                                          \
    BUILD_ASSERT(DT_INST_PROP_OR(n, start_threshold, 1) > 0,                                       \
                 "start-threshold must be greater than 0");                                       \
    BUILD_ASSERT(DT_INST_PROP_OR(n, burst_threshold, 1) > 0,                                       \
                 "burst-threshold must be greater than 0");                                       \
    BUILD_ASSERT(DT_INST_PROP_OR(n, burst_timeout_ms, 120) > 0,                                    \
                 "burst-timeout-ms must be greater than 0");                                      \
    BUILD_ASSERT(DT_INST_PROP_OR(n, burst_window_ms, 80) > 0,                                      \
                 "burst-window-ms must be greater than 0");                                       \
    BUILD_ASSERT(DT_INST_PROP_OR(n, release_ms, 40) > 0,                                           \
                 "release-ms must be greater than 0");                                            \
    BUILD_ASSERT(DT_INST_PROP_OR(n, decay_percent, 78) < 100,                                     \
                 "decay-percent must be less than 100");                                          \
    DEVICE_DT_INST_DEFINE(n, inertial_scroll_init, NULL, &inertial_scroll_data_##n,                \
                          &inertial_scroll_config_##n, POST_KERNEL,                                \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &inertial_scroll_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INERTIAL_SCROLL_INST)
