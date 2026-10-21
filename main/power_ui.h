#pragma once
#include "lvgl.h"

/* Build the power gauge on `parent` (pass lv_screen_active()). */
void power_ui_init(lv_obj_t *parent);

/* Point the needle at `power_kw` and update the central kW / % readouts.
 * Regen (negative kW) is supported; the needle pins at the 7 o'clock start
 * below the gauge minimum and at 6 o'clock above the over-range limit. */
void power_ui_set_power(float power_kw);

/* Update the max-throttle button's label with the authoritative value from
 * speed_test_driver's telemetry (the current throtmax ceiling, %). The
 * displayed value always mirrors this — tapping the button only requests a
 * change, it doesn't set the label directly. */
void power_ui_set_max_throttle_pct(float pct);

/* Update the car's opmode (car_mode_t, from telemetry_t.opmode). The
 * max-throttle button only accepts taps while the car is off/precharge/etc —
 * it's disabled (dimmed, non-interactive) whenever the car is in CAR_MODE_RUN. */
void power_ui_set_car_mode(uint8_t opmode);

/* Update the Wh/mile efficiency readout — three meters, cycled by tapping
 * the label itself. Same behavior as waveform_tr_display/main/gauge_ui.c:
 *   T = since trip reset      C = since last charge      M = last 10 miles
 * Pass the (wh, mi) pair for all three every time; the label renders
 * whichever is currently selected and stays in sync across calls. */
void power_ui_set_whmi(double trip_wh, double trip_mi,
                       double charge_wh, double charge_mi,
                       double roll_wh, double roll_mi);

/* Update the trip-mileage readout (lower right, mirrors the Wh/mi label). */
void power_ui_set_trip_mi(double trip_mi);

/* Register a callback invoked (from LVGL task context) when the trip-mileage
 * readout is tapped, to reset the trip counters — this board had no way to
 * do that before. Set a flag in the callback; do not call LVGL or blocking
 * functions directly inside it. */
void power_ui_set_trip_cb(void (*cb)(void));
