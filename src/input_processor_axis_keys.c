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
    struct zmk_behavior_binding bindings[AXIS_KEY_BINDINGS];
};

struct axis_keys_data {
    int32_t remainder;
};

static int tap_binding(const struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    int ret = zmk_behavior_invoke_binding(binding, event, true);
    if (ret < 0) {
        return ret;
    }

    event.timestamp = k_uptime_get();
    return zmk_behavior_invoke_binding(binding, event, false);
}

static int axis_keys_handle_event(const struct device *dev, struct input_event *event,
                                  uint32_t param1, uint32_t param2,
                                  struct zmk_input_processor_state *state) {
    const struct axis_keys_config *cfg = dev->config;
    struct axis_keys_data *data = dev->data;

    if (event->type != cfg->type || event->code != cfg->code) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    data->remainder += event->value;
    event->value = 0;

    struct zmk_behavior_binding_event behavior_event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(
            state->input_device_index, cfg->index),
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    while (data->remainder <= -cfg->threshold) {
        int ret = tap_binding(&cfg->bindings[NEGATIVE_BINDING], behavior_event);
        if (ret < 0) {
            return ret;
        }
        data->remainder += cfg->threshold;
    }

    while (data->remainder >= cfg->threshold) {
        int ret = tap_binding(&cfg->bindings[POSITIVE_BINDING], behavior_event);
        if (ret < 0) {
            return ret;
        }
        data->remainder -= cfg->threshold;
    }

    return ZMK_INPUT_PROC_STOP;
}

static int axis_keys_init(const struct device *dev) { return 0; }

static struct zmk_input_processor_driver_api axis_keys_driver_api = {
    .handle_event = axis_keys_handle_event,
};

#define AXIS_KEYS_INST(n)                                                                          \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, bindings) == AXIS_KEY_BINDINGS,                              \
                 "axis-keys requires exactly two bindings");                                      \
    BUILD_ASSERT(DT_INST_PROP(n, threshold) > 0, "axis-keys threshold must be greater than zero"); \
    static const struct axis_keys_config axis_keys_config_##n = {                                  \
        .index = n,                                                                                \
        .type = DT_INST_PROP(n, type),                                                             \
        .code = DT_INST_PROP(n, code),                                                             \
        .threshold = DT_INST_PROP(n, threshold),                                                   \
        .bindings = {ZMK_KEYMAP_EXTRACT_BINDING(0, DT_DRV_INST(n)),                               \
                     ZMK_KEYMAP_EXTRACT_BINDING(1, DT_DRV_INST(n))},                               \
    };                                                                                             \
    static struct axis_keys_data axis_keys_data_##n = {};                                          \
    DEVICE_DT_INST_DEFINE(n, axis_keys_init, NULL, &axis_keys_data_##n, &axis_keys_config_##n,     \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                        \
                          &axis_keys_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AXIS_KEYS_INST)
