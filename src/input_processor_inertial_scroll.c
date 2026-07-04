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
    uint8_t decay_percent;
    int16_t stop_threshold;
    int16_t max_step;
    uint16_t codes[];
};

struct inertial_scroll_data {
    const struct device *dev;
    struct k_work_delayable work;
    int32_t velocity;
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
        return;
    }

    data->velocity = (data->velocity * cfg->decay_percent) / 100;
    int32_t abs_velocity = data->velocity < 0 ? -data->velocity : data->velocity;
    if (abs_velocity < cfg->stop_threshold) {
        data->velocity = 0;
        return;
    }

    int32_t step = data->velocity / VELOCITY_SCALE;
    if (step == 0) {
        step = data->velocity > 0 ? 1 : -1;
    }

    int err = send_scroll_report(data->code, limit_step(cfg, step));
    if (err < 0) {
        LOG_WRN("Failed to send inertial scroll: %d", err);
        data->velocity = 0;
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

    data->code = event->code;
    data->velocity = event->value * VELOCITY_SCALE;

    k_work_reschedule(&data->work, K_MSEC(cfg->interval_ms));

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
        .decay_percent = DT_INST_PROP_OR(n, decay_percent, 78),                                    \
        .stop_threshold = DT_INST_PROP_OR(n, stop_threshold, 35),                                  \
        .max_step = DT_INST_PROP_OR(n, max_step, 4),                                               \
        .codes = DT_INST_PROP(n, codes),                                                           \
    };                                                                                             \
    static struct inertial_scroll_data inertial_scroll_data_##n = {};                              \
    BUILD_ASSERT(DT_INST_PROP_OR(n, decay_percent, 78) < 100,                                     \
                 "decay-percent must be less than 100");                                          \
    DEVICE_DT_INST_DEFINE(n, inertial_scroll_init, NULL, &inertial_scroll_data_##n,                \
                          &inertial_scroll_config_##n, POST_KERNEL,                                \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &inertial_scroll_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INERTIAL_SCROLL_INST)
