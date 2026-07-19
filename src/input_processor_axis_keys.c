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
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <drivers/input_processor.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>
#include <zmk/virtual_key_position.h>

#define AXIS_KEY_BINDINGS 2
#define AXIS_KEY_QUEUE_DEPTH 1
#define NEGATIVE_BINDING 0
#define POSITIVE_BINDING 1

struct axis_key_tap {
    uint8_t binding_index;
    struct zmk_behavior_binding_event event;
};

struct axis_keys_config {
    uint8_t index;
    uint8_t type;
    uint16_t code;
    int16_t threshold;
    uint16_t min_interval_ms;
    uint16_t tap_ms;
    const uint16_t *consume_codes;
    size_t consume_codes_len;
    struct zmk_behavior_binding bindings[AXIS_KEY_BINDINGS];
    k_thread_stack_t *thread_stack;
    size_t thread_stack_size;
};

struct axis_keys_data {
    int32_t remainder;
    atomic_t busy;
    struct k_msgq tap_queue;
    char tap_queue_buffer[sizeof(struct axis_key_tap) * AXIS_KEY_QUEUE_DEPTH] __aligned(4);
    struct k_thread thread;
};

static void axis_keys_thread(void *dev_ptr, void *unused1, void *unused2) {
    const struct device *dev = dev_ptr;
    const struct axis_keys_config *cfg = dev->config;
    struct axis_keys_data *data = dev->data;
    struct axis_key_tap tap;

    while (true) {
        k_msgq_get(&data->tap_queue, &tap, K_FOREVER);

        const struct zmk_behavior_binding *binding = &cfg->bindings[tap.binding_index];
        tap.event.timestamp = k_uptime_get();
        zmk_behavior_invoke_binding(binding, tap.event, true);

        k_sleep(K_MSEC(cfg->tap_ms));

        tap.event.timestamp = k_uptime_get();
        zmk_behavior_invoke_binding(binding, tap.event, false);

        if (cfg->min_interval_ms > cfg->tap_ms) {
            k_sleep(K_MSEC(cfg->min_interval_ms - cfg->tap_ms));
        }

        atomic_clear(&data->busy);
    }
}

static bool should_consume_code(const struct axis_keys_config *cfg, uint16_t code) {
    for (size_t i = 0; i < cfg->consume_codes_len; i++) {
        if (cfg->consume_codes[i] == code) {
            return true;
        }
    }

    return false;
}

static int axis_keys_handle_event(const struct device *dev, struct input_event *event,
                                  uint32_t param1, uint32_t param2,
                                  struct zmk_input_processor_state *state) {
    const struct axis_keys_config *cfg = dev->config;
    struct axis_keys_data *data = dev->data;

    if (event->type != cfg->type || !should_consume_code(cfg, event->code)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code != cfg->code) {
        event->value = 0;
        return ZMK_INPUT_PROC_STOP;
    }

    data->remainder =
        CLAMP(data->remainder + event->value, -cfg->threshold, cfg->threshold);
    event->value = 0;

    const bool negative = data->remainder <= -cfg->threshold;
    const bool positive = data->remainder >= cfg->threshold;

    if (!negative && !positive) {
        return ZMK_INPUT_PROC_STOP;
    }

    data->remainder = 0;

    if (!atomic_cas(&data->busy, 0, 1)) {
        return ZMK_INPUT_PROC_STOP;
    }

    struct axis_key_tap tap = {
        .binding_index = negative ? NEGATIVE_BINDING : POSITIVE_BINDING,
        .event =
            {
                .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(
                    state->input_device_index, cfg->index),
                .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
                .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
            },
    };

    if (k_msgq_put(&data->tap_queue, &tap, K_NO_WAIT) < 0) {
        atomic_clear(&data->busy);
    }

    return ZMK_INPUT_PROC_STOP;
}

static int axis_keys_init(const struct device *dev) {
    const struct axis_keys_config *cfg = dev->config;
    struct axis_keys_data *data = dev->data;

    k_msgq_init(&data->tap_queue, data->tap_queue_buffer, sizeof(struct axis_key_tap),
                AXIS_KEY_QUEUE_DEPTH);

    k_tid_t thread_id =
        k_thread_create(&data->thread, cfg->thread_stack, cfg->thread_stack_size, axis_keys_thread,
                        (void *)dev, NULL, NULL,
                        K_PRIO_PREEMPT(CONFIG_ZARUBALL_AXIS_KEYS_THREAD_PRIORITY), 0, K_NO_WAIT);
    k_thread_name_set(thread_id, "axis_keys");

    return 0;
}

static struct zmk_input_processor_driver_api axis_keys_driver_api = {
    .handle_event = axis_keys_handle_event,
};

#define AXIS_KEYS_INST(n)                                                                          \
    static const uint16_t axis_keys_consume_codes_##n[] = DT_INST_PROP(n, consume_codes);          \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, bindings) == AXIS_KEY_BINDINGS,                              \
                 "axis-keys requires exactly two bindings");                                      \
    BUILD_ASSERT(DT_INST_PROP(n, threshold) > 0, "axis-keys threshold must be greater than zero"); \
    BUILD_ASSERT(DT_INST_PROP(n, min_interval_ms) > 0,                                            \
                 "axis-keys min-interval-ms must be greater than zero");                           \
    BUILD_ASSERT(DT_INST_PROP(n, tap_ms) > 0,                                                     \
                 "axis-keys tap-ms must be greater than zero");                                   \
    K_THREAD_STACK_DEFINE(axis_keys_thread_stack_##n,                                              \
                          CONFIG_ZARUBALL_AXIS_KEYS_THREAD_STACK_SIZE);                            \
    static const struct axis_keys_config axis_keys_config_##n = {                                  \
        .index = n,                                                                                \
        .type = DT_INST_PROP(n, type),                                                             \
        .code = DT_INST_PROP(n, code),                                                             \
        .threshold = DT_INST_PROP(n, threshold),                                                   \
        .min_interval_ms = DT_INST_PROP(n, min_interval_ms),                                       \
        .tap_ms = DT_INST_PROP(n, tap_ms),                                                         \
        .consume_codes = axis_keys_consume_codes_##n,                                              \
        .consume_codes_len = ARRAY_SIZE(axis_keys_consume_codes_##n),                              \
        .bindings = {ZMK_KEYMAP_EXTRACT_BINDING(0, DT_DRV_INST(n)),                               \
                     ZMK_KEYMAP_EXTRACT_BINDING(1, DT_DRV_INST(n))},                               \
        .thread_stack = axis_keys_thread_stack_##n,                                                \
        .thread_stack_size = K_THREAD_STACK_SIZEOF(axis_keys_thread_stack_##n),                    \
    };                                                                                             \
    static struct axis_keys_data axis_keys_data_##n = {};                                          \
    DEVICE_DT_INST_DEFINE(n, axis_keys_init, NULL, &axis_keys_data_##n, &axis_keys_config_##n,     \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                        \
                          &axis_keys_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AXIS_KEYS_INST)
