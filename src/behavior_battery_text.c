/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_battery_text

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <drivers/behavior.h>
#include <dt-bindings/zmk/keys.h>
#include <zmk/battery.h>
#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>

#define KEY_WAIT_MS 10
#define PERIPHERAL_SOURCE 0

static atomic_t peripheral_connected = ATOMIC_INIT(0);
static atomic_t peripheral_level_known = ATOMIC_INIT(0);
static atomic_t peripheral_level = ATOMIC_INIT(0);

static bool is_split_peripheral_connection(struct bt_conn *conn) {
    struct bt_conn_info info;

    return bt_conn_get_info(conn, &info) == 0 && info.role == BT_CONN_ROLE_CENTRAL;
}

static void battery_text_connected(struct bt_conn *conn, uint8_t err) {
    if (err == 0 && is_split_peripheral_connection(conn)) {
        atomic_set(&peripheral_connected, 1);
        atomic_clear(&peripheral_level_known);
    }
}

static void battery_text_disconnected(struct bt_conn *conn, uint8_t reason) {
    if (is_split_peripheral_connection(conn)) {
        atomic_clear(&peripheral_connected);
        atomic_clear(&peripheral_level_known);
    }
}

BT_CONN_CB_DEFINE(battery_text_conn_callbacks) = {
    .connected = battery_text_connected,
    .disconnected = battery_text_disconnected,
};

static int peripheral_battery_listener(const zmk_event_t *event) {
    const struct zmk_peripheral_battery_state_changed *battery_event =
        as_zmk_peripheral_battery_state_changed(event);

    if (battery_event == NULL || battery_event->source != PERIPHERAL_SOURCE) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (atomic_get(&peripheral_connected)) {
        atomic_set(&peripheral_level, MIN(battery_event->state_of_charge, 100));
        atomic_set(&peripheral_level_known, 1);
    } else {
        atomic_clear(&peripheral_level_known);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(peripheral_battery_text, peripheral_battery_listener);
ZMK_SUBSCRIPTION(peripheral_battery_text, zmk_peripheral_battery_state_changed);

static int queue_keycode(const struct zmk_behavior_binding_event *event, uint32_t keycode) {
    const struct zmk_behavior_binding binding = {
        .behavior_dev = DEVICE_DT_NAME(DT_NODELABEL(kp)),
        .param1 = keycode,
    };

    int err = zmk_behavior_queue_add(event, binding, true, KEY_WAIT_MS);
    if (err < 0) {
        return err;
    }

    return zmk_behavior_queue_add(event, binding, false, KEY_WAIT_MS);
}

static int queue_text_keycode(const struct zmk_behavior_binding_event *event, uint32_t keycode,
                              uint8_t *typed_chars) {
    int err = queue_keycode(event, keycode);

    if (err == 0) {
        (*typed_chars)++;
    }

    return err;
}

static int queue_level(const struct zmk_behavior_binding_event *event, uint8_t level,
                       uint8_t *typed_chars) {
    static const uint32_t digits[] = {N0, N1, N2, N3, N4, N5, N6, N7, N8, N9};
    int err;

    level = MIN(level, 100);

    if (level == 100) {
        err = queue_text_keycode(event, N1, typed_chars);
        if (err < 0) {
            return err;
        }

        err = queue_text_keycode(event, N0, typed_chars);
        if (err < 0) {
            return err;
        }

        return queue_text_keycode(event, N0, typed_chars);
    }

    if (level >= 10) {
        err = queue_text_keycode(event, digits[level / 10], typed_chars);
        if (err < 0) {
            return err;
        }
    }

    return queue_text_keycode(event, digits[level % 10], typed_chars);
}

static int queue_select_previous_chars(const struct zmk_behavior_binding_event *event,
                                       uint8_t typed_chars) {
    for (uint8_t i = 0; i < typed_chars; i++) {
        int err = queue_keycode(event, LS(LEFT));

        if (err < 0) {
            return err;
        }
    }

    return 0;
}

static int queue_battery_text(const struct zmk_behavior_binding_event *event) {
    const uint8_t central_level = MIN(zmk_battery_state_of_charge(), 100);
    const bool peripheral_available = atomic_get(&peripheral_level_known);
    const uint8_t cached_peripheral_level = atomic_get(&peripheral_level);
    uint8_t typed_chars = 0;

    int err = queue_keycode(event, LANGUAGE_2);
    if (err < 0) {
        return err;
    }

    err = queue_text_keycode(event, LS(L), &typed_chars);
    if (err < 0) {
        return err;
    }

    // The apostrophe HID usage produces ':' on the configured Japanese layout.
    err = queue_text_keycode(event, SQT, &typed_chars);
    if (err < 0) {
        return err;
    }

    err = queue_level(event, central_level, &typed_chars);
    if (err < 0) {
        return err;
    }

    err = queue_text_keycode(event, PERCENT, &typed_chars);
    if (err < 0) {
        return err;
    }

    err = queue_text_keycode(event, SPACE, &typed_chars);
    if (err < 0) {
        return err;
    }

    err = queue_text_keycode(event, LS(R), &typed_chars);
    if (err < 0) {
        return err;
    }

    err = queue_text_keycode(event, SQT, &typed_chars);
    if (err < 0) {
        return err;
    }

    if (peripheral_available) {
        err = queue_level(event, cached_peripheral_level, &typed_chars);
        if (err < 0) {
            return err;
        }

        err = queue_text_keycode(event, PERCENT, &typed_chars);
        if (err < 0) {
            return err;
        }

        return queue_select_previous_chars(event, typed_chars);
    }

    err = queue_text_keycode(event, MINUS, &typed_chars);
    if (err < 0) {
        return err;
    }

    err = queue_text_keycode(event, MINUS, &typed_chars);
    if (err < 0) {
        return err;
    }

    return queue_select_previous_chars(event, typed_chars);
}

static int on_battery_text_pressed(struct zmk_behavior_binding *binding,
                                   struct zmk_behavior_binding_event event) {
    return queue_battery_text(&event);
}

static int on_battery_text_released(struct zmk_behavior_binding *binding,
                                    struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_battery_text_driver_api = {
    .binding_pressed = on_battery_text_pressed,
    .binding_released = on_battery_text_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_battery_text_driver_api);
