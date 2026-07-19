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
#include <zephyr/sys/util.h>

#include <drivers/input_processor.h>
#include <zmk/endpoints.h>
#include <zmk/hid.h>
#include <zmk/keymap.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zaruball_inertia, LOG_LEVEL_WRN);

#define VELOCITY_SCALE 100

struct inertial_scroll_config {
    uint8_t type;
    size_t codes_len;
    bool axis_lock;
    int16_t axis_lock_threshold;
    uint16_t axis_lock_ratio_percent;
    int16_t axis_lock_max_pending;
    uint16_t axis_lock_release_ms;
    uint16_t interval_ms;
    uint16_t gain_percent;
    uint8_t velocity_percent;
    int16_t min_velocity;
    int16_t start_threshold;
    int16_t burst_threshold;
    int16_t burst_peak_threshold;
    uint8_t burst_peak_percent;
    uint16_t burst_timeout_ms;
    uint16_t burst_window_ms;
    uint16_t release_ms;
    uint16_t candidate_tail_base_ms;
    uint16_t candidate_tail_per_peak_ms;
    uint16_t candidate_tail_max_ms;
    uint8_t decay_percent;
    uint8_t tail_decay_percent;
    int16_t tail_threshold;
    int16_t stop_threshold;
    int16_t max_step;
    int16_t cancel_layer;
    uint16_t output_code;
    uint16_t codes[];
};

enum scroll_axis {
    SCROLL_AXIS_NONE,
    SCROLL_AXIS_X,
    SCROLL_AXIS_Y,
};

struct inertial_scroll_data {
    const struct device *dev;
    struct k_work_delayable work;
    int32_t velocity;
    int32_t velocity_remainder;
    int32_t remainder;
    int32_t burst_accum;
    int32_t burst_peak;
    int32_t best_burst_accum;
    int32_t best_burst_peak;
    int64_t last_input_ms;
    int64_t burst_start_ms;
    int64_t best_burst_ms;
    int8_t burst_dir;
    int8_t best_burst_dir;
    uint16_t input_code;
    uint16_t code;
    enum scroll_axis locked_axis;
    int32_t pending_x;
    int32_t pending_y;
    int32_t pending_abs_x;
    int32_t pending_abs_y;
    int64_t last_axis_input_ms;
#if IS_ENABLED(CONFIG_ZARUBALL_INERTIAL_SCROLL_DEBUG)
    int32_t debug_pos_accum;
    int32_t debug_neg_accum;
    int32_t debug_pos_peak;
    int32_t debug_neg_peak;
    uint16_t debug_pos_count;
    uint16_t debug_neg_count;
    int64_t debug_start_ms;
    int64_t debug_last_ms;
    uint16_t debug_code;
    bool debug_active;
#endif
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

#if IS_ENABLED(CONFIG_ZARUBALL_INERTIAL_SCROLL_DEBUG)
static void debug_reset(struct inertial_scroll_data *data) {
    data->debug_pos_accum = 0;
    data->debug_neg_accum = 0;
    data->debug_pos_peak = 0;
    data->debug_neg_peak = 0;
    data->debug_pos_count = 0;
    data->debug_neg_count = 0;
    data->debug_start_ms = 0;
    data->debug_last_ms = 0;
    data->debug_code = 0;
    data->debug_active = false;
}

static void debug_record_input(const struct inertial_scroll_config *cfg,
                               struct inertial_scroll_data *data, uint16_t code, int8_t dir,
                               int32_t amount, int64_t now) {
    (void)cfg;

    if (!data->debug_active || data->debug_code != code) {
        debug_reset(data);
        data->debug_active = true;
        data->debug_code = code;
        data->debug_start_ms = now;
    }

    data->debug_last_ms = now;
    if (dir > 0) {
        data->debug_pos_accum += amount;
        data->debug_pos_count++;
        if (amount > data->debug_pos_peak) {
            data->debug_pos_peak = amount;
        }
    } else {
        data->debug_neg_accum += amount;
        data->debug_neg_count++;
        if (amount > data->debug_neg_peak) {
            data->debug_neg_peak = amount;
        }
    }
}

static void debug_log_window(const struct inertial_scroll_config *cfg,
                             const struct inertial_scroll_data *data, const char *state,
                             int8_t chosen_dir, int32_t chosen_accum, int32_t chosen_peak,
                             int32_t velocity, int64_t now) {
    if (!data->debug_active) {
        return;
    }

    int64_t tail_limit_ms =
        cfg->candidate_tail_base_ms +
        (int64_t)chosen_peak * cfg->candidate_tail_per_peak_ms;
    if (cfg->candidate_tail_max_ms > 0 &&
        tail_limit_ms > cfg->candidate_tail_max_ms) {
        tail_limit_ms = cfg->candidate_tail_max_ms;
    }

    LOG_WRN("inertia_dbg %s in=%u out=%u age=%lld gap=%lld total=%ld/%ld peak=%ld/%ld "
            "chosen=%d/%ld/%ld vel=%ld",
            state, data->debug_code, cfg->output_code,
            (long long)(now - data->debug_start_ms), (long long)(now - data->debug_last_ms),
            (long)data->debug_pos_accum, (long)data->debug_neg_accum,
            (long)data->debug_pos_peak, (long)data->debug_neg_peak, chosen_dir,
            (long)chosen_accum, (long)chosen_peak, (long)velocity);
    LOG_WRN("inertia_dbg profile samples=%u/%u best_at=%lld tail=%lld limit=%lld",
            data->debug_pos_count, data->debug_neg_count,
            (long long)(data->best_burst_ms - data->debug_start_ms),
            (long long)(now - data->best_burst_ms), (long long)tail_limit_ms);
}

static void debug_log_touch_stop(uint16_t code, int8_t dir, int32_t amount, int32_t velocity) {
    LOG_WRN("inertia_dbg touch_stop in=%u dir=%d amount=%ld old_vel=%ld", code, dir,
            (long)amount, (long)velocity);
}
#else
static void debug_reset(struct inertial_scroll_data *data) { (void)data; }
static void debug_record_input(const struct inertial_scroll_config *cfg,
                               struct inertial_scroll_data *data, uint16_t code, int8_t dir,
                               int32_t amount, int64_t now) {
    (void)cfg;
    (void)data;
    (void)code;
    (void)dir;
    (void)amount;
    (void)now;
}
static void debug_log_window(const struct inertial_scroll_config *cfg,
                             const struct inertial_scroll_data *data, const char *state,
                             int8_t chosen_dir, int32_t chosen_accum, int32_t chosen_peak,
                             int32_t velocity, int64_t now) {
    (void)cfg;
    (void)data;
    (void)state;
    (void)chosen_dir;
    (void)chosen_accum;
    (void)chosen_peak;
    (void)velocity;
    (void)now;
}
static void debug_log_touch_stop(uint16_t code, int8_t dir, int32_t amount, int32_t velocity) {
    (void)code;
    (void)dir;
    (void)amount;
    (void)velocity;
}
#endif

static void stop_inertia(struct inertial_scroll_data *data) {
    data->velocity = 0;
    data->velocity_remainder = 0;
    data->remainder = 0;
}

static void reset_burst_window(struct inertial_scroll_data *data) {
    data->burst_accum = 0;
    data->burst_peak = 0;
}

static void reset_burst_tracking(struct inertial_scroll_data *data) {
    reset_burst_window(data);
    data->best_burst_accum = 0;
    data->best_burst_peak = 0;
    data->best_burst_dir = 0;
    data->best_burst_ms = 0;
}

static bool burst_is_sharp_enough(const struct inertial_scroll_config *cfg, int32_t accum,
                                  int32_t peak) {
    if (accum < cfg->burst_threshold || peak < cfg->burst_peak_threshold) {
        return false;
    }

    return cfg->burst_peak_percent == 0 ||
           peak * 100 >= accum * cfg->burst_peak_percent;
}

static void update_best_burst(const struct inertial_scroll_config *cfg,
                              struct inertial_scroll_data *data, int64_t now) {
    bool current_is_valid =
        burst_is_sharp_enough(cfg, data->burst_accum, data->burst_peak);
    bool best_is_valid =
        burst_is_sharp_enough(cfg, data->best_burst_accum, data->best_burst_peak);

    if ((current_is_valid && !best_is_valid) ||
        (current_is_valid == best_is_valid &&
         (data->burst_peak > data->best_burst_peak ||
          (data->burst_peak == data->best_burst_peak &&
           data->burst_accum > data->best_burst_accum)))) {
        data->best_burst_accum = data->burst_accum;
        data->best_burst_peak = data->burst_peak;
        data->best_burst_dir = data->burst_dir;
        data->best_burst_ms = now;
    }
}

static void clear_pending_scroll(struct inertial_scroll_data *data) {
    stop_inertia(data);
    reset_burst_tracking(data);
    debug_reset(data);
}

static void clear_axis_lock(struct inertial_scroll_data *data) {
    data->locked_axis = SCROLL_AXIS_NONE;
    data->pending_x = 0;
    data->pending_y = 0;
    data->pending_abs_x = 0;
    data->pending_abs_y = 0;
    data->last_axis_input_ms = 0;
}

static void prime_first_step(struct inertial_scroll_data *data) {
    data->remainder = sign32(data->velocity) * (VELOCITY_SCALE - 1);
}

static bool layer_cancels_inertia(const struct inertial_scroll_config *cfg) {
    return cfg->cancel_layer >= 0 && zmk_keymap_layer_active(cfg->cancel_layer);
}

static enum scroll_axis axis_for_code(uint16_t code) {
    switch (code) {
    case INPUT_REL_X:
        return SCROLL_AXIS_X;
    case INPUT_REL_Y:
        return SCROLL_AXIS_Y;
    default:
        return SCROLL_AXIS_NONE;
    }
}

static uint16_t output_code_for_input(const struct inertial_scroll_config *cfg,
                                      uint16_t input_code) {
    if (!cfg->axis_lock) {
        return cfg->output_code;
    }

    return input_code == INPUT_REL_X ? INPUT_REL_HWHEEL : INPUT_REL_WHEEL;
}

static enum scroll_axis choose_axis(const struct inertial_scroll_config *cfg,
                                    const struct inertial_scroll_data *data) {
    int32_t x = data->pending_abs_x;
    int32_t y = data->pending_abs_y;

    if (x >= cfg->axis_lock_threshold &&
        (int64_t)x * 100 >= (int64_t)y * cfg->axis_lock_ratio_percent) {
        return SCROLL_AXIS_X;
    }

    if (y >= cfg->axis_lock_threshold &&
        (int64_t)y * 100 >= (int64_t)x * cfg->axis_lock_ratio_percent) {
        return SCROLL_AXIS_Y;
    }

    if (x + y < cfg->axis_lock_max_pending) {
        return SCROLL_AXIS_NONE;
    }

    // Prefer vertical scrolling when the direction is ambiguous.
    return x > y ? SCROLL_AXIS_X : SCROLL_AXIS_Y;
}

static bool apply_axis_lock(const struct inertial_scroll_config *cfg,
                            struct inertial_scroll_data *data, struct input_event *event,
                            int64_t now) {
    enum scroll_axis event_axis = axis_for_code(event->code);

    if (!cfg->axis_lock || event_axis == SCROLL_AXIS_NONE) {
        return true;
    }

    if (data->last_axis_input_ms > 0 &&
        now - data->last_axis_input_ms >= cfg->axis_lock_release_ms) {
        clear_axis_lock(data);
        clear_pending_scroll(data);
    }

    if (data->locked_axis != SCROLL_AXIS_NONE) {
        if (event_axis != data->locked_axis) {
            if (event->value != 0) {
                stop_inertia(data);
            }
            event->value = 0;
            return false;
        }

        if (event->value != 0) {
            data->last_axis_input_ms = now;
        }
        return true;
    }

    if (event_axis == SCROLL_AXIS_X) {
        data->pending_x += event->value;
        data->pending_abs_x += abs32(event->value);
    } else {
        data->pending_y += event->value;
        data->pending_abs_y += abs32(event->value);
    }

    if (event->value != 0) {
        data->last_axis_input_ms = now;
    }
    event->value = 0;

    // Choose once per synchronized X/Y report so the first event cannot win prematurely.
    if (!event->sync) {
        return false;
    }

    data->locked_axis = choose_axis(cfg, data);
    if (data->locked_axis == SCROLL_AXIS_NONE) {
        return false;
    }

    event->code = data->locked_axis == SCROLL_AXIS_X ? INPUT_REL_X : INPUT_REL_Y;
    event->value = data->locked_axis == SCROLL_AXIS_X ? data->pending_x : data->pending_y;
    clear_axis_lock(data);
    data->locked_axis = axis_for_code(event->code);
    data->last_axis_input_ms = now;

    return event->value != 0;
}

static bool candidate_is_fresh(const struct inertial_scroll_config *cfg,
                               const struct inertial_scroll_data *data, int64_t now) {
    if (cfg->candidate_tail_base_ms == 0 && cfg->candidate_tail_per_peak_ms == 0) {
        return true;
    }

    int64_t allowed_tail_ms =
        cfg->candidate_tail_base_ms +
        (int64_t)data->best_burst_peak * cfg->candidate_tail_per_peak_ms;

    if (cfg->candidate_tail_max_ms > 0 &&
        allowed_tail_ms > cfg->candidate_tail_max_ms) {
        allowed_tail_ms = cfg->candidate_tail_max_ms;
    }

    return data->best_burst_ms > 0 && now - data->best_burst_ms <= allowed_tail_ms;
}

static int32_t input_to_velocity(const struct inertial_scroll_config *cfg, int8_t dir,
                                 int32_t amount) {
    int64_t velocity = (int64_t)dir * amount * VELOCITY_SCALE * cfg->gain_percent *
                       cfg->velocity_percent;
    int32_t scaled_velocity = velocity / 10000;

    if (cfg->min_velocity > 0 && abs32(scaled_velocity) < cfg->min_velocity) {
        scaled_velocity = dir * cfg->min_velocity;
    }

    if (abs32(scaled_velocity) > VELOCITY_SCALE) {
        scaled_velocity = dir * VELOCITY_SCALE;
    }

    return scaled_velocity;
}

static void decay_velocity(struct inertial_scroll_data *data,
                           const struct inertial_scroll_config *cfg) {
    uint8_t decay_percent = abs32(data->velocity) <= cfg->tail_threshold
                                ? cfg->tail_decay_percent
                                : cfg->decay_percent;
    int64_t scaled = (int64_t)data->velocity * decay_percent + data->velocity_remainder;
    int32_t next_velocity = scaled / 100;

    if (next_velocity == 0 && data->velocity != 0 && scaled != 0) {
        next_velocity = sign32(data->velocity);
    }

    data->velocity = next_velocity;
    data->velocity_remainder = scaled - (data->velocity * 100);
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
    const int64_t now = k_uptime_get();

    if (layer_cancels_inertia(cfg)) {
        debug_log_window(cfg, data, "mouse_cancel", data->best_burst_dir,
                         data->best_burst_accum, data->best_burst_peak, 0, now);
        clear_pending_scroll(data);
        clear_axis_lock(data);
        return;
    }

    if (data->velocity == 0) {
        if (!burst_is_sharp_enough(cfg, data->best_burst_accum, data->best_burst_peak)) {
            debug_log_window(cfg, data, "no_start", data->best_burst_dir,
                             data->best_burst_accum, data->best_burst_peak, 0, now);
            reset_burst_tracking(data);
            debug_reset(data);
            return;
        }

        if (!candidate_is_fresh(cfg, data, now)) {
            debug_log_window(cfg, data, "stale", data->best_burst_dir,
                             data->best_burst_accum, data->best_burst_peak, 0, now);
            reset_burst_tracking(data);
            debug_reset(data);
            return;
        }

        int32_t velocity =
            input_to_velocity(cfg, data->best_burst_dir, data->best_burst_peak);
        debug_log_window(cfg, data, "start", data->best_burst_dir, data->best_burst_accum,
                         data->best_burst_peak, velocity, now);
        debug_reset(data);
        data->velocity = velocity;
        data->velocity_remainder = 0;
        prime_first_step(data);
        reset_burst_tracking(data);
    }

    decay_velocity(data, cfg);
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

    const int64_t now = k_uptime_get();

    if (!apply_axis_lock(cfg, data, event, now)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->value == 0) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    const int8_t input_dir = sign32(event->value);
    const int32_t input_amount = abs32(event->value);

    if (data->velocity != 0) {
        debug_log_touch_stop(event->code, input_dir, input_amount, data->velocity);
        k_work_cancel_delayable(&data->work);
        stop_inertia(data);
        reset_burst_tracking(data);
        debug_reset(data);
        data->burst_dir = input_dir;
        data->burst_start_ms = now;
        data->last_input_ms = 0;
    }

    if (input_amount < cfg->start_threshold) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    debug_record_input(cfg, data, event->code, input_dir, input_amount, now);

    if (data->burst_dir != input_dir || data->input_code != event->code ||
        now - data->last_input_ms > cfg->burst_timeout_ms) {
        reset_burst_window(data);
        data->burst_dir = input_dir;
        data->burst_start_ms = now;
    } else if (now - data->burst_start_ms > cfg->burst_window_ms) {
        reset_burst_window(data);
        data->burst_start_ms = now;
    }

    data->last_input_ms = now;
    data->input_code = event->code;
    data->code = output_code_for_input(cfg, event->code);
    data->burst_accum += input_amount;
    if (input_amount > data->burst_peak) {
        data->burst_peak = input_amount;
    }
    update_best_burst(cfg, data, now);

    k_work_reschedule(&data->work, K_MSEC(cfg->release_ms));

    return ZMK_INPUT_PROC_CONTINUE;
}

static int inertial_scroll_init(const struct device *dev) {
    struct inertial_scroll_data *data = dev->data;

    data->dev = dev;
    k_work_init_delayable(&data->work, inertial_scroll_work_handler);

#if IS_ENABLED(CONFIG_ZARUBALL_INERTIAL_SCROLL_DEBUG)
    const struct inertial_scroll_config *cfg = dev->config;
    LOG_WRN("inertia_dbg ready in=%u out=%u cancel_layer=%d", cfg->codes[0],
            cfg->output_code, cfg->cancel_layer);
#endif

    return 0;
}

static struct zmk_input_processor_driver_api inertial_scroll_driver_api = {
    .handle_event = inertial_scroll_handle_event,
};

#define INERTIAL_SCROLL_INST(n)                                                                    \
    static const struct inertial_scroll_config inertial_scroll_config_##n = {                      \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                            \
        .codes_len = DT_INST_PROP_LEN(n, codes),                                                   \
        .axis_lock = DT_INST_PROP_OR(n, axis_lock, false),                                         \
        .axis_lock_threshold = DT_INST_PROP_OR(n, axis_lock_threshold, 4),                         \
        .axis_lock_ratio_percent = DT_INST_PROP_OR(n, axis_lock_ratio_percent, 150),               \
        .axis_lock_max_pending = DT_INST_PROP_OR(n, axis_lock_max_pending, 12),                    \
        .axis_lock_release_ms = DT_INST_PROP_OR(n, axis_lock_release_ms, 80),                      \
        .interval_ms = DT_INST_PROP_OR(n, interval_ms, 16),                                        \
        .gain_percent = DT_INST_PROP_OR(n, gain_percent, 100),                                     \
        .velocity_percent = DT_INST_PROP_OR(n, velocity_percent, 100),                             \
        .min_velocity = DT_INST_PROP_OR(n, min_velocity, 0),                                       \
        .start_threshold = DT_INST_PROP_OR(n, start_threshold, 1),                                  \
        .burst_threshold = DT_INST_PROP_OR(n, burst_threshold, 1),                                  \
        .burst_peak_threshold = DT_INST_PROP_OR(n, burst_peak_threshold, 1),                        \
        .burst_peak_percent = DT_INST_PROP_OR(n, burst_peak_percent, 0),                            \
        .burst_timeout_ms = DT_INST_PROP_OR(n, burst_timeout_ms, 120),                              \
        .burst_window_ms = DT_INST_PROP_OR(n, burst_window_ms, 80),                                  \
        .release_ms = DT_INST_PROP_OR(n, release_ms, 40),                                           \
        .candidate_tail_base_ms = DT_INST_PROP_OR(n, candidate_tail_base_ms, 0),                    \
        .candidate_tail_per_peak_ms = DT_INST_PROP_OR(n, candidate_tail_per_peak_ms, 0),            \
        .candidate_tail_max_ms = DT_INST_PROP_OR(n, candidate_tail_max_ms, 0),                      \
        .decay_percent = DT_INST_PROP_OR(n, decay_percent, 78),                                    \
        .tail_decay_percent = DT_INST_PROP_OR(n, tail_decay_percent,                                \
                                              DT_INST_PROP_OR(n, decay_percent, 78)),               \
        .tail_threshold = DT_INST_PROP_OR(n, tail_threshold, 0),                                    \
        .stop_threshold = DT_INST_PROP_OR(n, stop_threshold, 35),                                  \
        .max_step = DT_INST_PROP_OR(n, max_step, 4),                                               \
        .cancel_layer = DT_INST_PROP_OR(n, cancel_layer, -1),                                      \
        .output_code = DT_INST_PROP_OR(n, output_code, INPUT_REL_WHEEL),                           \
        .codes = DT_INST_PROP(n, codes),                                                           \
    };                                                                                             \
    static struct inertial_scroll_data inertial_scroll_data_##n = {};                              \
    BUILD_ASSERT(DT_INST_PROP_OR(n, gain_percent, 100) > 0,                                        \
                 "gain-percent must be greater than 0");                                          \
    BUILD_ASSERT(DT_INST_PROP_OR(n, velocity_percent, 100) > 0,                                    \
                 "velocity-percent must be greater than 0");                                      \
    BUILD_ASSERT(DT_INST_PROP_OR(n, axis_lock_threshold, 4) > 0,                                  \
                 "axis-lock-threshold must be greater than 0");                                   \
    BUILD_ASSERT(DT_INST_PROP_OR(n, axis_lock_ratio_percent, 150) >= 100,                          \
                 "axis-lock-ratio-percent must be at least 100");                                 \
    BUILD_ASSERT(DT_INST_PROP_OR(n, axis_lock_max_pending, 12) > 0,                               \
                 "axis-lock-max-pending must be greater than 0");                                 \
    BUILD_ASSERT(DT_INST_PROP_OR(n, axis_lock_release_ms, 80) > 0,                                \
                 "axis-lock-release-ms must be greater than 0");                                  \
    BUILD_ASSERT(DT_INST_PROP_OR(n, start_threshold, 1) > 0,                                       \
                 "start-threshold must be greater than 0");                                       \
    BUILD_ASSERT(DT_INST_PROP_OR(n, burst_threshold, 1) > 0,                                       \
                 "burst-threshold must be greater than 0");                                       \
    BUILD_ASSERT(DT_INST_PROP_OR(n, burst_peak_threshold, 1) > 0,                                  \
                 "burst-peak-threshold must be greater than 0");                                  \
    BUILD_ASSERT(DT_INST_PROP_OR(n, burst_timeout_ms, 120) > 0,                                    \
                 "burst-timeout-ms must be greater than 0");                                      \
    BUILD_ASSERT(DT_INST_PROP_OR(n, burst_window_ms, 80) > 0,                                      \
                 "burst-window-ms must be greater than 0");                                       \
    BUILD_ASSERT(DT_INST_PROP_OR(n, release_ms, 40) > 0,                                           \
                 "release-ms must be greater than 0");                                            \
    BUILD_ASSERT(DT_INST_PROP_OR(n, decay_percent, 78) < 100,                                     \
                 "decay-percent must be less than 100");                                          \
    BUILD_ASSERT(DT_INST_PROP_OR(n, tail_decay_percent, DT_INST_PROP_OR(n, decay_percent, 78)) <   \
                     100,                                                                          \
                 "tail-decay-percent must be less than 100");                                     \
    DEVICE_DT_INST_DEFINE(n, inertial_scroll_init, NULL, &inertial_scroll_data_##n,                \
                          &inertial_scroll_config_##n, POST_KERNEL,                                \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &inertial_scroll_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INERTIAL_SCROLL_INST)
