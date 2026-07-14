#include <stdio.h>
#include <math.h>
#include "power_ui.h"
#include "triumph_logo.h"

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
#define NEEDLE_LEN       330
#define TICK_MINOR_LEN   13
#define TICK_MAJOR_LEN   22
#define HUB_R            20

/* Central readout placement, below centre so it clears the hub/needle base */
#define KW_Y   (CY + 90)
#define PCT_Y  (CY + 160)

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

/* Fixed-width centred label — text updates need only lv_label_set_text(). */
static lv_obj_t *fixed_label(lv_obj_t *parent, int cx, int cy, int w,
                             const char *txt, lv_color_t col, const lv_font_t *font)
{
    int h = font->line_height;
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
    lv_style_set_text_color(&s_ticks_major, COL_RED);
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
    lv_style_set_line_width(&s_needle_line, 5);
    lv_style_set_line_rounded(&s_needle_line, true);

    g_needle = lv_line_create(g_scale);
    lv_obj_add_style(g_needle, &s_needle_line, 0);
    lv_scale_set_line_needle_value(g_scale, g_needle, NEEDLE_LEN, PWR_MIN_PCT);

    /* Hub dot on top of the needle base */
    circle(parent, CX, CY, HUB_R, COL_RED);

    /* Central readouts: kW (large) and % (below) */
    g_kw_lbl  = fixed_label(parent, CX, KW_Y,  300, "0 kW", COL_RED, &lv_font_montserrat_bold_44);
    g_pct_lbl = fixed_label(parent, CX, PCT_Y, 300, "0 %",  COL_RED, &lv_font_montserrat_bold_28);
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
