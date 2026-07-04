# Copyright (c) 2026 The ZMK Contributors
# SPDX-License-Identifier: MIT

target_sources_ifdef(CONFIG_ZARUBALL_BATTERY_TEXT app PRIVATE src/behavior_battery_text.c)
