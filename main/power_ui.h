#pragma once
#include "lvgl.h"

/* Build the power gauge on `parent` (pass lv_screen_active()). */
void power_ui_init(lv_obj_t *parent);

/* Point the needle at `power_kw` and update the central kW / % readouts.
 * Regen (negative kW) is supported; the needle pins at the 7 o'clock start
 * below the gauge minimum and at 6 o'clock above the over-range limit. */
void power_ui_set_power(float power_kw);
