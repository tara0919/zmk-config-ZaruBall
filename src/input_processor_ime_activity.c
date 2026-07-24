/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_ime_activity

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>

#include <drivers/input_processor.h>
#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/keymap.h>
#include <zmk/virtual_key_position.h>

#define KEY_WAIT_MS 10

struct ime_activity_config {
    uint8_t index;
    uint8_t layer;
    uint32_t keycode;
};

struct ime_activity_data {
    bool armed;
};

static int ime_activity_handle_event(const struct device *dev, struct input_event *event,
                                     uint32_t param1, uint32_t param2,
                                     struct zmk_input_processor_state *state) {
    const struct ime_activity_config *config = dev->config;
    struct ime_activity_data *data = dev->data;

    if (event->type != INPUT_EV_REL || event->value == 0 || !data->armed) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    data->armed = false;

    const struct zmk_behavior_binding binding = {
        .behavior_dev = DEVICE_DT_NAME(DT_NODELABEL(kp)),
        .param1 = config->keycode,
    };
    const struct zmk_behavior_binding_event behavior_event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(
            state->input_device_index, config->index),
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    int ret = zmk_behavior_queue_add(&behavior_event, binding, true, KEY_WAIT_MS);
    if (ret < 0) {
        return ret;
    }

    ret = zmk_behavior_queue_add(&behavior_event, binding, false, KEY_WAIT_MS);
    return ret < 0 ? ret : ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api ime_activity_driver_api = {
    .handle_event = ime_activity_handle_event,
};

static int ime_activity_init(const struct device *dev) { return 0; }

#define IME_ACTIVITY_INST(n)                                                        \
    static struct ime_activity_data ime_activity_data_##n;                         \
    static const struct ime_activity_config ime_activity_config_##n = {             \
        .index = n,                                                                 \
        .layer = DT_INST_PROP(n, layer),                                             \
        .keycode = DT_INST_PROP(n, keycode),                                         \
    };                                                                              \
    DEVICE_DT_INST_DEFINE(n, ime_activity_init, NULL, &ime_activity_data_##n,        \
                          &ime_activity_config_##n, POST_KERNEL,                     \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &ime_activity_driver_api);

DT_INST_FOREACH_STATUS_OKAY(IME_ACTIVITY_INST)

#define IME_ACTIVITY_SET_ARMED(n)                                                   \
    if (ime_activity_config_##n.layer == event->layer) {                            \
        ime_activity_data_##n.armed = event->state;                                 \
    }

static int ime_activity_layer_listener(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *event = as_zmk_layer_state_changed(eh);

    if (event == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    DT_INST_FOREACH_STATUS_OKAY(IME_ACTIVITY_SET_ARMED)
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(ime_activity, ime_activity_layer_listener);
ZMK_SUBSCRIPTION(ime_activity, zmk_layer_state_changed);
