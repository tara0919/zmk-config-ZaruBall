/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_ime_activity

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <string.h>

#include <drivers/input_processor.h>
#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/virtual_key_position.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define KEY_WAIT_MS 10
#define MAX_TRACKED_POSITIONS 128
#define POSITION_WORD_BITS 32
#define POSITION_WORDS (MAX_TRACKED_POSITIONS / POSITION_WORD_BITS)

struct ime_activity_config {
    uint8_t index;
    uint8_t layer;
    uint32_t enter_keycode;
    uint32_t exit_keycode;
    uint32_t restore_delay_ms;
    const uint32_t *ignored_positions;
    size_t ignored_positions_count;
};

struct ime_activity_data {
    const struct ime_activity_config *config;
    struct k_work_delayable restore_work;
    bool active;
    bool used;
    bool restore_pending;
    uint8_t pressed_count;
    uint32_t pressed_positions[POSITION_WORDS];
};

static int queue_keycode(const struct ime_activity_config *config, uint32_t keycode) {
    const struct zmk_behavior_binding binding = {
        .behavior_dev = DEVICE_DT_NAME(DT_NODELABEL(kp)),
        .param1 = keycode,
    };
    const struct zmk_behavior_binding_event behavior_event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(0, config->index),
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    int ret = zmk_behavior_queue_add(&behavior_event, binding, true, KEY_WAIT_MS);
    if (ret < 0) {
        return ret;
    }

    return zmk_behavior_queue_add(&behavior_event, binding, false, KEY_WAIT_MS);
}

static bool position_is_ignored(const struct ime_activity_config *config, uint32_t position) {
    for (size_t i = 0; i < config->ignored_positions_count; i++) {
        if (config->ignored_positions[i] == position) {
            return true;
        }
    }

    return false;
}

static bool set_position_pressed(struct ime_activity_data *data, uint32_t position, bool pressed) {
    if (position >= MAX_TRACKED_POSITIONS) {
        return false;
    }

    uint32_t word = position / POSITION_WORD_BITS;
    uint32_t mask = BIT(position % POSITION_WORD_BITS);
    bool was_pressed = (data->pressed_positions[word] & mask) != 0;

    if (pressed == was_pressed) {
        return false;
    }

    if (pressed) {
        data->pressed_positions[word] |= mask;
        data->pressed_count++;
    } else {
        data->pressed_positions[word] &= ~mask;
        data->pressed_count--;
    }

    return true;
}

static void restore_ime_work_handler(struct k_work *work) {
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct ime_activity_data *data =
        CONTAINER_OF(delayable, struct ime_activity_data, restore_work);

    if (!data->restore_pending || data->active || data->pressed_count != 0) {
        return;
    }

    data->restore_pending = false;
    int ret = queue_keycode(data->config, data->config->exit_keycode);
    if (ret < 0) {
        LOG_WRN("Failed to queue IME exit keycode: %d", ret);
    }
}

static void schedule_ime_restore(struct ime_activity_data *data) {
    if (data->restore_pending && !data->active && data->pressed_count == 0) {
        k_work_reschedule(&data->restore_work, K_MSEC(data->config->restore_delay_ms));
    }
}

static int ime_activity_handle_event(const struct device *dev, struct input_event *event,
                                     uint32_t param1, uint32_t param2,
                                     struct zmk_input_processor_state *state) {
    struct ime_activity_data *data = dev->data;

    if (data->active && event->type == INPUT_EV_REL && event->value != 0) {
        data->used = true;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api ime_activity_driver_api = {
    .handle_event = ime_activity_handle_event,
};

static int ime_activity_init(const struct device *dev) {
    struct ime_activity_data *data = dev->data;

    data->config = dev->config;
    k_work_init_delayable(&data->restore_work, restore_ime_work_handler);
    return 0;
}

#define IME_ACTIVITY_INST(n)                                                        \
    static const uint32_t ime_activity_ignored_positions_##n[] =                    \
        DT_INST_PROP(n, ignored_positions);                                         \
    static struct ime_activity_data ime_activity_data_##n;                         \
    static const struct ime_activity_config ime_activity_config_##n = {             \
        .index = n,                                                                 \
        .layer = DT_INST_PROP(n, layer),                                             \
        .enter_keycode = DT_INST_PROP(n, enter_keycode),                            \
        .exit_keycode = DT_INST_PROP(n, exit_keycode),                              \
        .restore_delay_ms = DT_INST_PROP(n, restore_delay_ms),                       \
        .ignored_positions = ime_activity_ignored_positions_##n,                    \
        .ignored_positions_count = ARRAY_SIZE(ime_activity_ignored_positions_##n),  \
    };                                                                              \
    DEVICE_DT_INST_DEFINE(n, ime_activity_init, NULL, &ime_activity_data_##n,        \
                          &ime_activity_config_##n, POST_KERNEL,                     \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &ime_activity_driver_api);

DT_INST_FOREACH_STATUS_OKAY(IME_ACTIVITY_INST)

#define IME_ACTIVITY_LAYER_EVENT(n)                                                 \
    if (ime_activity_config_##n.layer == layer_event->layer) {                      \
        if (layer_event->state) {                                                   \
            k_work_cancel_delayable(&ime_activity_data_##n.restore_work);           \
            ime_activity_data_##n.active = true;                                    \
            ime_activity_data_##n.used = false;                                     \
            ime_activity_data_##n.restore_pending = false;                          \
            ime_activity_data_##n.pressed_count = 0;                                \
            memset(ime_activity_data_##n.pressed_positions, 0,                      \
                   sizeof(ime_activity_data_##n.pressed_positions));                \
            int ret = queue_keycode(&ime_activity_config_##n,                       \
                                    ime_activity_config_##n.enter_keycode);          \
            if (ret < 0) {                                                          \
                LOG_WRN("Failed to queue IME enter keycode: %d", ret);              \
            }                                                                       \
        } else {                                                                    \
            ime_activity_data_##n.restore_pending = ime_activity_data_##n.active && \
                                                        ime_activity_data_##n.used;  \
            ime_activity_data_##n.active = false;                                   \
            ime_activity_data_##n.used = false;                                     \
            schedule_ime_restore(&ime_activity_data_##n);                           \
        }                                                                           \
    }

#define IME_ACTIVITY_POSITION_EVENT(n)                                              \
    if (!position_is_ignored(&ime_activity_config_##n, position_event->position)) { \
        if (ime_activity_data_##n.active && position_event->state) {                 \
            ime_activity_data_##n.used = true;                                      \
            set_position_pressed(&ime_activity_data_##n, position_event->position,  \
                                 true);                                              \
        } else if (!position_event->state &&                                        \
                   set_position_pressed(&ime_activity_data_##n,                     \
                                        position_event->position, false)) {          \
            schedule_ime_restore(&ime_activity_data_##n);                           \
        }                                                                           \
    }

static int ime_activity_event_listener(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *layer_event = as_zmk_layer_state_changed(eh);

    if (layer_event != NULL) {
        DT_INST_FOREACH_STATUS_OKAY(IME_ACTIVITY_LAYER_EVENT)
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_position_state_changed *position_event =
        as_zmk_position_state_changed(eh);
    if (position_event != NULL) {
        DT_INST_FOREACH_STATUS_OKAY(IME_ACTIVITY_POSITION_EVENT)
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(ime_activity, ime_activity_event_listener);
ZMK_SUBSCRIPTION(ime_activity, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(ime_activity, zmk_position_state_changed);
