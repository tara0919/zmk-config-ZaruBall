/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_axis_keys

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <drivers/input_processor.h>
#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/keymap.h>
#include <zmk/virtual_key_position.h>

#define AXIS_KEY_BINDINGS 2
#define NEGATIVE_BINDING 0
#define POSITIVE_BINDING 1

struct axis_keys_config {
    uint8_t index;
    uint8_t type;
    uint16_t code;
    int16_t threshold;
    uint16_t min_interval_ms;
    uint16_t tap_ms;
    struct zmk_behavior_binding bindings[AXIS_KEY_BINDINGS];
};

struct axis_keys_data {
    int32_t remainder;
    int64_t last_tap_at;
    bool has_tapped;
};

static int queue_tap(const struct zmk_behavior_binding *binding,
                     const struct zmk_behavior_binding_event *event, uint16_t tap_ms) {
    int ret = zmk_behavior_queue_add(event, *binding, true, tap_ms);
    if (ret < 0) {
        return ret;
    }

    return zmk_behavior_queue_add(event, *binding, false, 0);
}

static int axis_keys_handle_event(const struct device *dev, struct input_event *event,
                                  uint32_t param1, uint32_t param2,
                                  struct zmk_input_processor_state *state) {
    const struct axis_keys_config *cfg = dev->config;
    struct axis_keys_data *data = dev->data;

    if (event->type != cfg->type || event->code != cfg->code) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    data->remainder =
        CLAMP(data->remainder + event->value, -cfg->threshold, cfg->threshold);
    event->value = 0;

    const int64_t now = k_uptime_get();
    const bool negative = data->remainder <= -cfg->threshold;
    const bool positive = data->remainder >= cfg->threshold;

    if (!negative && !positive) {
        return ZMK_INPUT_PROC_STOP;
    }

    if (data->has_tapped && now - data->last_tap_at < cfg->min_interval_ms) {
        return ZMK_INPUT_PROC_STOP;
    }

    struct zmk_behavior_binding_event behavior_event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(
            state->input_device_index, cfg->index),
        .timestamp = now,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    data->remainder = 0;
    data->last_tap_at = now;
    data->has_tapped = true;

    int ret = queue_tap(&cfg->bindings[negative ? NEGATIVE_BINDING : POSITIVE_BINDING],
                        &behavior_event, cfg->tap_ms);

    return ret < 0 ? ret : ZMK_INPUT_PROC_STOP;
}

static int axis_keys_init(const struct device *dev) { return 0; }

static struct zmk_input_processor_driver_api axis_keys_driver_api = {
    .handle_event = axis_keys_handle_event,
};

#define AXIS_KEYS_INST(n)                                                                          \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, bindings) == AXIS_KEY_BINDINGS,                              \
                 "axis-keys requires exactly two bindings");                                      \
    BUILD_ASSERT(DT_INST_PROP(n, threshold) > 0, "axis-keys threshold must be greater than zero"); \
    BUILD_ASSERT(DT_INST_PROP(n, min_interval_ms) > 0,                                            \
                 "axis-keys min-interval-ms must be greater than zero");                           \
    BUILD_ASSERT(DT_INST_PROP(n, tap_ms) > 0,                                                     \
                 "axis-keys tap-ms must be greater than zero");                                   \
    static const struct axis_keys_config axis_keys_config_##n = {                                  \
        .index = n,                                                                                \
        .type = DT_INST_PROP(n, type),                                                             \
        .code = DT_INST_PROP(n, code),                                                             \
        .threshold = DT_INST_PROP(n, threshold),                                                   \
        .min_interval_ms = DT_INST_PROP(n, min_interval_ms),                                       \
        .tap_ms = DT_INST_PROP(n, tap_ms),                                                         \
        .bindings = {ZMK_KEYMAP_EXTRACT_BINDING(0, DT_DRV_INST(n)),                               \
                     ZMK_KEYMAP_EXTRACT_BINDING(1, DT_DRV_INST(n))},                               \
    };                                                                                             \
    static struct axis_keys_data axis_keys_data_##n = {};                                          \
    DEVICE_DT_INST_DEFINE(n, axis_keys_init, NULL, &axis_keys_data_##n, &axis_keys_config_##n,     \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                        \
                          &axis_keys_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AXIS_KEYS_INST)
