#include <stdio.h>
#include <math.h>
#include "power_ui.h"
#include "triumph_logo.h"
#include "rs485_rx.h"

LV_FONT_DECLARE(lv_font_montserrat_bold_28)
LV_FONT_DECLARE(lv_font_montserrat_bold_36)
LV_FONT_DECLARE(lv_font_montserrat_bold_44)

/* ── Display geometry (800 × 800) ─────────────────────────────────────────── */
#define CX  400
#define CY  400

/* ── Power scale — ported from qualia_tr_display/code.py ───────────────────────
 * -20 % sits at 7 o'clock, 100 % at 5 o'clock, 300° total sweep.
 * 100 % == PWR_KW_AT_100 kW. Regen (negative) pins the needle at the 7 o'clock
 * start; the needle otherwise clamps at the 100 % (5 o'clock) end.
 *
 * lv_scale rotation: 0° = 3 o'clock, CW. 7 o'clock = 120° from 3 o'clock —
 * identical mapping to the waveform speedometer, so the ring sits in the same
 * place. Minor tick every 5 %, numeric label every 20 %.  */
#define PWR_KW_AT_100    110.0f
#define PWR_MIN_PCT      (-20)
#define PWR_MAX_PCT      (100)
#define SWEEP_TOTAL_DEG  300
#define SCALE_ROTATION   120
#define NUM_TICKS        25      /* -20..100 step 5 → 25 ticks              */
#define MAJOR_EVERY      4       /* every 4th tick (= every 20 %) is major  */

#define SCALE_DIAM       760
#define NEEDLE_LEN       320   /* matches waveform_tr_display/main/gauge_ui.c */
#define NEEDLE_WIDTH     12    /* matches waveform_tr_display/main/gauge_ui.c */
#define TICK_MINOR_LEN   13
#define TICK_MAJOR_LEN   22
#define HUB_R            20

/* Central readout placement, below centre so it clears the hub/needle base */
#define KW_Y   (CY + 90)
#define PCT_Y  (CY + 160)

/* Max-throttle preset button.
 *
 * This display is round: the panel's visible area is a circle, not the full
 * 800x800 framebuffer square. The gauge sweeps 300 degrees (-20% at 7
 * o'clock to 100% at 5 o'clock) and covers every angle EXCEPT a 60-degree gap
 * at the bottom (centred on 6 o'clock, between the 5 and 7 o'clock
 * endpoints) — no ticks, no labels there at any radius.
 *
 * Two corner placements were tried and both failed for opposite reasons:
 *   (110, 90) ~425px from centre — inside the reach of the major tick labels
 *             (44pt font on top of the 380px ring radius + 22px tick length
 *             pushes their outer edge to ~450-470px) — it overlapped the
 *             ~20% tick.
 *   (100, 20) ~484px from centre — far enough to clear the ticks, but past
 *             the round panel's visible edge, so it went dark entirely.
 * There isn't much radial room between those two failure zones anywhere the
 * gauge actually sweeps, so this uses the tick-free bottom gap instead,
 * centred under CX like the kW/% readouts — their y-ranges don't overlap
 * (readouts end ~577, this starts ~653), so sharing the x-centre is fine. */
#define MAXPWR_CX   CX
#define MAXPWR_CY   670
#define MAXPWR_W    200

/* Wh/mi efficiency readout — lower left, inside the tick-free bottom gap
 * (radius from centre is ~300 px, well clear of the ~380 px tick ring, so
 * unlike the corner placements above this isn't fighting the rim at all).
 * Tap to cycle Trip -> Charge -> last 10 mi -> Trip, same behavior and
 * "T 123 / Wh/mi" format as waveform_tr_display/main/gauge_ui.c. */
#define WHMI_CX   280
#define WHMI_CY   620
#define WHMI_W    200

/* Trip-mileage readout — mirrors the Wh/mi label on the lower right, same
 * radius from centre. Tap to reset the trip counters — the only way to do
 * that on this board, since it has no ODO/TRIP plaque like tr_display. */
#define TRIP_CX   520
#define TRIP_CY   620
#define TRIP_W    200

/* Colours — red gauge on the dark background used by the current display.
 * (Change COL_BG to COL_WHITE for the qualia's white-background look.) */
#define COL_RED     lv_color_hex(0xFF0000)
#define COL_BG      lv_color_hex(0x111111)

/* Widget handles */
static lv_obj_t *g_scale;
static lv_obj_t *g_needle;
static lv_obj_t *g_kw_lbl;
static lv_obj_t *g_pct_lbl;

/* Persistent styles */
static lv_style_t s_scale_bg;
static lv_style_t s_ticks_minor;
static lv_style_t s_ticks_major;
static lv_style_t s_needle_line;

/* Create a filled circle whose centre is at (cx, cy). */
static lv_obj_t *circle(lv_obj_t *parent, int cx, int cy, int r, lv_color_t col)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_set_size(o, r * 2, r * 2);
    lv_obj_set_pos(o, cx - r, cy - r);
    lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(o, col, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    return o;
}

/* Fixed-width centred label — text updates need only lv_label_set_text().
 * `lines` sizes the label for that many rows (needed for the two-line
 * Wh/mi readout: "T 123" over "Wh/mi"). */
static lv_obj_t *fixed_label(lv_obj_t *parent, int cx, int cy, int w, int lines,
                             const char *txt, lv_color_t col, const lv_font_t *font)
{
    int h = font->line_height * lines + 4 * (lines - 1);
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, col, 0);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_bg_opa(l, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(l, w);
    lv_obj_set_height(l, LV_SIZE_CONTENT);
    lv_obj_set_pos(l, cx - w / 2, cy - h / 2);
    return l;
}

/* Max-throttle presets, cycled by tapping the on-screen button. The button
 * only ever *requests* a preset — it does not set the label. The label is
 * driven solely by power_ui_set_max_throttle_pct(), called from the latest
 * telemetry frame, so it always shows speed_test_driver's authoritative
 * value (itself read from the ZombieVerter's throtmax at boot, or the last
 * command this display sent). speed_test_driver does not yet forward the
 * value on to the ZombieVerter over CAN. */
static const float MAXPWR_PRESETS[] = { 50.0f, 75.0f, 100.0f };
#define MAXPWR_PRESET_COUNT  (int)(sizeof(MAXPWR_PRESETS) / sizeof(MAXPWR_PRESETS[0]))

static lv_obj_t *g_maxpwr_lbl;
static float     g_last_max_throttle_pct = NAN;   /* last value seen in telemetry */
static car_mode_t g_car_mode = CAR_MODE_OFF;      /* last opmode seen in telemetry */

#define COL_DIM     lv_color_hex(0x802020)   /* dimmed red — button disabled in Run */
#define COL_WHITE   lv_color_hex(0xFFFFFF)

/* Wh/mile readout — three meters, cycled by tapping the label:
 *   T = since trip reset      C = since last charge      M = last 10 miles
 * Ported from waveform_tr_display/main/gauge_ui.c (same enum, same render
 * format), minus that display's separate "big readout" promotion step —
 * here the lower-left label is the only Wh/mi widget, so tapping it cycles
 * the mode directly instead of first promoting to a big centre readout. */
typedef enum {
    WHMI_TRIP = 0,
    WHMI_CHARGE,
    WHMI_ROLL,
    WHMI_MODE_COUNT,
} whmi_mode_t;

static lv_obj_t   *g_whmi_lbl;
static double      s_whmi_trip_wh = 0.0, s_whmi_trip_mi = 0.0;
static double      s_whmi_chg_wh  = 0.0, s_whmi_chg_mi  = 0.0;
static double      s_whmi_roll_wh = 0.0, s_whmi_roll_mi = 0.0;
static whmi_mode_t s_whmi_mode = WHMI_TRIP;

/* The (wh, mi) pair and tag letter for the currently selected meter. */
static void whmi_selected(double *wh, double *mi, char *tag)
{
    switch (s_whmi_mode) {
        case WHMI_CHARGE: *wh = s_whmi_chg_wh;  *mi = s_whmi_chg_mi;  *tag = 'C'; break;
        case WHMI_ROLL:   *wh = s_whmi_roll_wh; *mi = s_whmi_roll_mi; *tag = 'M'; break;
        default:          *wh = s_whmi_trip_wh; *mi = s_whmi_trip_mi; *tag = 'T'; break;
    }
}

/* Below ~0.1 mi the ratio is noise-dominated, so hold at 0. fabs() makes the
 * reading polarity-independent (the energy counter's sign depends on wiring). */
static int whmi_value(double wh, double mi)
{
    if (mi < 0.1) return 0;
    double v = fabs(wh) / mi;
    if (v > 999.0) v = 999.0;   /* keep it to 3 digits */
    return (int)(v + 0.5);
}

static void whmi_render(void)
{
    char buf[24];
    double wh, mi;
    char   tag;
    whmi_selected(&wh, &mi, &tag);
    snprintf(buf, sizeof(buf), "%c %d\nWh/mi", tag, whmi_value(wh, mi));
    lv_label_set_text(g_whmi_lbl, buf);
}

static void whmi_click_cb(lv_event_t *e)
{
    (void)e;
    s_whmi_mode = (whmi_mode_t)((s_whmi_mode + 1) % WHMI_MODE_COUNT);
    whmi_render();
}

/* Trip-mileage readout, tap to reset trip */
static lv_obj_t *g_trip_lbl;
static void (*s_trip_cb)(void) = NULL;

static void trip_click_cb(lv_event_t *e)
{
    (void)e;
    if (s_trip_cb) s_trip_cb();
}

/* Index of the preset closest to `pct` (ties favour the lower index). */
static int maxpwr_nearest_preset(float pct)
{
    int   best     = 0;
    float bestDist = fabsf(pct - MAXPWR_PRESETS[0]);
    for (int i = 1; i < MAXPWR_PRESET_COUNT; i++) {
        float d = fabsf(pct - MAXPWR_PRESETS[i]);
        if (d < bestDist) { bestDist = d; best = i; }
    }
    return best;
}

static void maxpwr_click_cb(lv_event_t *e)
{
    (void)e;
    /* Changing max throttle while the car is actually moving under power is
     * unsafe, so the button is ignored (and shown dimmed) whenever the car
     * is in Run. */
    if (g_car_mode == CAR_MODE_RUN) {
        return;
    }
    /* Advance from whatever's currently shown (or the first preset, if no
     * telemetry has arrived yet) to the next preset in the cycle. Only sends
     * the request — the label updates once speed_test_driver echoes the new
     * value back in telemetry. */
    float base = isnan(g_last_max_throttle_pct) ? MAXPWR_PRESETS[0] : g_last_max_throttle_pct;
    int   idx  = (maxpwr_nearest_preset(base) + 1) % MAXPWR_PRESET_COUNT;
    rs485_send_max_throttle_pct(MAXPWR_PRESETS[idx]);
}

void power_ui_init(lv_obj_t *parent)
{
    lv_obj_set_style_bg_color(parent, COL_BG, 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);

    /* Background logo — drawn first so the gauge layers on top */
    lv_obj_t *bg_img = lv_image_create(parent);
    lv_image_set_src(bg_img, &triumph_logo);
    lv_obj_center(bg_img);

    /* Scale background (transparent) */
    lv_style_init(&s_scale_bg);
    lv_style_set_bg_opa(&s_scale_bg, LV_OPA_TRANSP);
    lv_style_set_border_width(&s_scale_bg, 0);
    lv_style_set_pad_all(&s_scale_bg, 0);

    /* Minor tick style (every 5 %) */
    lv_style_init(&s_ticks_minor);
    lv_style_set_line_color(&s_ticks_minor, COL_RED);
    lv_style_set_line_width(&s_ticks_minor, 2);
    lv_style_set_length(&s_ticks_minor, TICK_MINOR_LEN);

    /* Major tick + numeric label style (every 20 %) */
    lv_style_init(&s_ticks_major);
    lv_style_set_line_color(&s_ticks_major, COL_RED);
    lv_style_set_line_width(&s_ticks_major, 3);
    lv_style_set_length(&s_ticks_major, TICK_MAJOR_LEN);
    lv_style_set_text_color(&s_ticks_major, COL_WHITE);
    lv_style_set_text_font(&s_ticks_major, &lv_font_montserrat_bold_44);

    /* Scale widget */
    g_scale = lv_scale_create(parent);
    lv_obj_set_size(g_scale, SCALE_DIAM, SCALE_DIAM);
    lv_obj_center(g_scale);
    lv_scale_set_mode(g_scale, LV_SCALE_MODE_ROUND_INNER);
    lv_scale_set_range(g_scale, PWR_MIN_PCT, PWR_MAX_PCT);
    lv_scale_set_total_tick_count(g_scale, NUM_TICKS);
    lv_scale_set_major_tick_every(g_scale, MAJOR_EVERY);
    lv_scale_set_angle_range(g_scale, SWEEP_TOTAL_DEG);
    lv_scale_set_rotation(g_scale, SCALE_ROTATION);
    lv_obj_add_style(g_scale, &s_scale_bg,    0);
    lv_obj_add_style(g_scale, &s_ticks_minor, LV_PART_ITEMS);
    lv_obj_add_style(g_scale, &s_ticks_major, LV_PART_INDICATOR);

    /* Needle (child of scale) */
    lv_style_init(&s_needle_line);
    lv_style_set_line_color(&s_needle_line, COL_RED);
    lv_style_set_line_width(&s_needle_line, NEEDLE_WIDTH);
    lv_style_set_line_rounded(&s_needle_line, true);

    g_needle = lv_line_create(g_scale);
    lv_obj_add_style(g_needle, &s_needle_line, 0);
    lv_scale_set_line_needle_value(g_scale, g_needle, NEEDLE_LEN, PWR_MIN_PCT);

    /* Hub dot on top of the needle base */
    circle(parent, CX, CY, HUB_R, COL_RED);

    /* Central readouts: kW (large) and % (below) */
    g_kw_lbl  = fixed_label(parent, CX, KW_Y,  300, 1, "0 kW", COL_WHITE, &lv_font_montserrat_bold_44);
    g_pct_lbl = fixed_label(parent, CX, PCT_Y, 300, 1, "0 %",  COL_WHITE, &lv_font_montserrat_bold_28);

    /* Max-throttle preset button (tap to cycle). Label starts blank — it's
     * filled in by the first telemetry frame's max_throttle_pct, not assumed
     * locally; see power_ui_set_max_throttle_pct(). */
    g_maxpwr_lbl = fixed_label(parent, MAXPWR_CX, MAXPWR_CY, MAXPWR_W, 1, "MAX -- %",
                               COL_WHITE, &lv_font_montserrat_bold_28);
    lv_obj_add_flag(g_maxpwr_lbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_maxpwr_lbl, maxpwr_click_cb, LV_EVENT_CLICKED, NULL);

    /* Wh/mi efficiency readout (tap to cycle Trip -> Charge -> last 10 mi),
     * same look and behavior as waveform_tr_display's small Wh/mi label. */
    g_whmi_lbl = fixed_label(parent, WHMI_CX, WHMI_CY, WHMI_W, 2, "T 0\nWh/mi",
                             COL_WHITE, &lv_font_montserrat_bold_36);
    lv_obj_add_flag(g_whmi_lbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_whmi_lbl, whmi_click_cb, LV_EVENT_CLICKED, NULL);

    /* Trip-mileage readout (tap to reset trip) */
    g_trip_lbl = fixed_label(parent, TRIP_CX, TRIP_CY, TRIP_W, 2, "TRIP\n0.0 mi",
                             COL_WHITE, &lv_font_montserrat_bold_36);
    lv_obj_add_flag(g_trip_lbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_trip_lbl, trip_click_cb, LV_EVENT_CLICKED, NULL);
}

void power_ui_set_max_throttle_pct(float pct)
{
    g_last_max_throttle_pct = pct;
    char buf[24];
    snprintf(buf, sizeof(buf), "MAX %.0f %%", pct);
    lv_label_set_text(g_maxpwr_lbl, buf);
}

void power_ui_set_car_mode(uint8_t opmode)
{
    car_mode_t mode = (opmode < CAR_MODE_LAST) ? (car_mode_t)opmode : CAR_MODE_OFF;
    if (mode == g_car_mode) {
        return;
    }
    g_car_mode = mode;
    lv_obj_set_style_text_color(g_maxpwr_lbl, mode == CAR_MODE_RUN ? COL_DIM : COL_WHITE, 0);
}

void power_ui_set_power(float power_kw)
{
    char buf[24];

    float pct = power_kw / PWR_KW_AT_100 * 100.0f;
    int   npct = (int)lroundf(pct);

    /* Needle clamps to the gauge range: regen below -20 % pins at the 7 o'clock
     * start, above 100 % pins at the 5 o'clock end. */
    int nv = npct < PWR_MIN_PCT ? PWR_MIN_PCT
           : npct > PWR_MAX_PCT ? PWR_MAX_PCT : npct;
    lv_scale_set_line_needle_value(g_scale, g_needle, NEEDLE_LEN, nv);

    /* Readouts show the true (unclamped) values, like the qualia. */
    snprintf(buf, sizeof(buf), "%d kW", (int)lroundf(power_kw));
    lv_label_set_text(g_kw_lbl, buf);
    snprintf(buf, sizeof(buf), "%d %%", npct);
    lv_label_set_text(g_pct_lbl, buf);
}

void power_ui_set_whmi(double trip_wh, double trip_mi,
                       double charge_wh, double charge_mi,
                       double roll_wh, double roll_mi)
{
    s_whmi_trip_wh = trip_wh;   s_whmi_trip_mi = trip_mi;
    s_whmi_chg_wh  = charge_wh; s_whmi_chg_mi  = charge_mi;
    s_whmi_roll_wh = roll_wh;   s_whmi_roll_mi = roll_mi;
    whmi_render();   /* honours the current T/C/M selection */
}

void power_ui_set_trip_mi(double trip_mi)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "TRIP\n%.1f mi", trip_mi);
    lv_label_set_text(g_trip_lbl, buf);
}

void power_ui_set_trip_cb(void (*cb)(void))
{
    s_trip_cb = cb;
}
