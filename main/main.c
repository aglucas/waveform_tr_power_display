#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "power_ui.h"
#include "rs485_rx.h"

static const char *TAG = "power_display";

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Mileage/energy persistence — feeds the Wh/mi readout (power_ui_set_whmi)   */
/*                                                                              */
/*  This board keeps its OWN odometer.bin, independent of waveform_tr_display: */
/*  its own SD card, its own accumulation from the same broadcast telemetry.   */
/*  There is no trip-reset control on this screen (unlike tr_display's ODO/    */
/*  TRIP plaque), so trip_mi/trip_wh just accumulate since the last reset that */
/*  happened to be on disk when this board booted. The on-disk struct/magic    */
/*  match tr_display's so a card is at least readable across boards.           */
/* ═══════════════════════════════════════════════════════════════════════════ */
#define MILEAGE_SAVE_MI   0.1        /* save every N miles                    */
#define MILEAGE_TICK_MS   100        /* accumulation period                   */

/* Mirrors g_cur_speed staleness handling in waveform_tr_display/main/main.c —
 * only integrate distance while the last telemetry frame is fresh. */
#define SPEED_STALE_MS    500

#define MILEAGE_FILE       BSP_SD_MOUNT_POINT "/odometer.bin"
#define MILEAGE_MAGIC      0xA55A1234U
#define MILEAGE_V4_BYTES   60        /* pre-save-flag layout, see tr_display  */

/* Reject an implausible frame-to-frame jump in the IVT energy counter (a shunt
 * reset, a dropped/garbled frame) — same threshold as tr_display. */
#define ENERGY_MAX_STEP_WH 5000.0

typedef struct __attribute__((packed)) {
    uint32_t magic;
    double   total_mi;
    double   trip_mi;
    double   charge_mi;
    double   total_wh;
    double   trip_wh;
    double   charge_wh;
    double   last_soc;
    uint8_t  save_is_pfail;   /* always 0 — this board has no PLP button */
} mileage_file_t;

static bool s_sd_mounted = false;

static double g_total_mi = 0.0, g_trip_mi = 0.0, g_charge_mi = 0.0;
static double g_mi_last_save = 0.0;
static float  g_cur_speed = 0.0f;
static volatile TickType_t g_last_frame_tick = 0;

/* Energy counters — integrated from the IVT cumulative counter, same as
 * tr_display, so deltas (not the raw counter) survive a shunt reset. */
static double g_total_wh = 0.0, g_trip_wh = 0.0, g_charge_wh = 0.0;
static double g_energy_prev      = 0.0;
static bool   g_energy_have_prev = false;
static bool   s_was_charge_mode  = false;

static double g_last_soc     = 0.0;
static bool   s_was_bus_live = false;

static volatile bool g_save_dirty  = false;
static volatile bool g_trip_reset  = false;

static void on_trip_reset(void) { g_trip_reset = true; }

/* ── Rolling efficiency window ────────────────────────────────────────────────
 * Wh/mile over the last ROLL_WINDOW_MI of TRIP distance — ported verbatim from
 * waveform_tr_display/main/main.c so the "M" meter behaves identically.
 * RAM-only: a reboot restarts the window even though trip_mi/trip_wh persist. */
#define ROLL_WINDOW_MI  10.0
#define ROLL_BUCKET_MI  0.1
#define ROLL_N          101

typedef struct { double mi, wh; } roll_sample_t;
static roll_sample_t g_roll[ROLL_N];
static int    g_roll_head    = 0;
static int    g_roll_count   = 0;
static double g_roll_last_mi = 0.0;

static void roll_reset(void)
{
    g_roll[0].mi   = g_trip_mi;
    g_roll[0].wh   = g_trip_wh;
    g_roll_head    = 1 % ROLL_N;
    g_roll_count   = 1;
    g_roll_last_mi = g_trip_mi;
}

static void roll_update(void)
{
    if (g_roll_count == 0)                      { roll_reset(); return; }
    if (g_trip_mi - g_roll_last_mi < ROLL_BUCKET_MI) return;

    g_roll[g_roll_head].mi = g_trip_mi;
    g_roll[g_roll_head].wh = g_trip_wh;
    g_roll_head = (g_roll_head + 1) % ROLL_N;
    if (g_roll_count < ROLL_N) g_roll_count++;
    g_roll_last_mi = g_trip_mi;
}

static void roll_window(double *wh, double *mi)
{
    if (g_roll_count == 0) { *wh = 0.0; *mi = 0.0; return; }
    int oldest = (g_roll_head - g_roll_count + ROLL_N) % ROLL_N;
    *mi = g_trip_mi - g_roll[oldest].mi;
    *wh = g_trip_wh - g_roll[oldest].wh;
}

/* Dump every persisted counter, plus the derived Wh/mi and the raw IVT
 * counter we're differencing — ported from waveform_tr_display/main/main.c's
 * log_counters() (trimmed: this board doesn't track trip_moving_s/implied
 * mph, having no odometer display of its own to cross-check against). Full
 * float precision, not cast to int — a cast made a 12.97 mi reading look
 * like data loss in the older tr_display logs. */
static void log_counters(const char *what)
{
    double trip_whmi = (g_trip_mi   >= 0.1) ? fabs(g_trip_wh)   / g_trip_mi   : 0.0;
    double chg_whmi  = (g_charge_mi >= 0.1) ? fabs(g_charge_wh) / g_charge_mi : 0.0;

    ESP_LOGI(TAG, "%s | MI  total=%.4f trip=%.4f charge=%.4f", what,
             g_total_mi, g_trip_mi, g_charge_mi);
    ESP_LOGI(TAG, "%s | WH  total=%.2f trip=%.2f charge=%.2f | last_soc=%.1f%%", what,
             g_total_wh, g_trip_wh, g_charge_wh, g_last_soc);

    double rwh, rmi;
    roll_window(&rwh, &rmi);
    double roll_whmi = (rmi >= 0.1) ? fabs(rwh) / rmi : 0.0;
    ESP_LOGI(TAG, "%s | EFF roll(M)=%.1f Wh/mi over %.2f mi (%d/%d buckets)",
             what, roll_whmi, rmi, g_roll_count, ROLL_N);

    uint32_t age_ms = (uint32_t)((xTaskGetTickCount() - g_last_frame_tick) * portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "%s | EFF trip=%.1f Wh/mi  charge=%.1f Wh/mi | IVT raw=%.2f Wh (have_prev=%d) "
                  "speed=%.2f mph (frame age=%lu ms%s)",
             what, trip_whmi, chg_whmi, g_energy_prev, (int)g_energy_have_prev,
             (double)g_cur_speed, (unsigned long)age_ms,
             (age_ms >= SPEED_STALE_MS) ? ", STALE - odo frozen" : "");
}

static void load_mileage(void)
{
    if (!s_sd_mounted) {
        ESP_LOGW(TAG, "SD: not mounted — counters start at 0");
        return;
    }
    FILE *fp = fopen(MILEAGE_FILE, "rb");
    if (!fp) {
        ESP_LOGW(TAG, "SD: no odometer.bin yet — starting all counters at 0");
        return;
    }

    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    mileage_file_t f;
    memset(&f, 0, sizeof(f));
    const char *layout = "unknown";
    bool read_ok = false;

    if (sz >= (long)sizeof(f)) {
        layout  = "current(61)";
        read_ok = (fread(&f, sizeof(f), 1, fp) == 1);
    } else if (sz == MILEAGE_V4_BYTES) {
        layout  = "v4(60)";
        read_ok = (fread(&f, MILEAGE_V4_BYTES, 1, fp) == 1);
    }
    fclose(fp);

    bool magic_ok = read_ok && (f.magic == MILEAGE_MAGIC);
    bool vals_ok  = magic_ok
                    && isfinite(f.total_mi)  && f.total_mi  >= 0.0
                    && isfinite(f.trip_mi)   && f.trip_mi   >= 0.0
                    && isfinite(f.charge_mi) && f.charge_mi >= 0.0
                    && isfinite(f.total_wh)  && isfinite(f.trip_wh)
                    && isfinite(f.charge_wh)
                    && isfinite(f.last_soc)  && f.last_soc >= 0.0
                                             && f.last_soc <= 100.0;

    ESP_LOGI(TAG, "SD: odometer.bin %ld bytes, layout=%s, read=%d magic=%d(0x%08lX) vals=%d",
             sz, layout, (int)read_ok, (int)magic_ok, (unsigned long)f.magic, (int)vals_ok);

    if (!vals_ok) {
        ESP_LOGE(TAG, "SD: odometer.bin REJECTED (%ld bytes) — counters start at 0 "
                      "and the next save will OVERWRITE the file.", sz);
        return;
    }

    g_total_mi  = f.total_mi;
    g_trip_mi   = f.trip_mi;
    g_charge_mi = f.charge_mi;
    g_total_wh  = f.total_wh;
    g_trip_wh   = f.trip_wh;
    g_charge_wh = f.charge_wh;
    g_last_soc  = f.last_soc;
    log_counters("LOAD");
}

static void save_mileage(void)
{
    if (!s_sd_mounted) {
        ESP_LOGW(TAG, "SAVE skipped — no SD card mounted");
        return;
    }
    mileage_file_t f = { MILEAGE_MAGIC, g_total_mi, g_trip_mi, g_charge_mi,
                         g_total_wh, g_trip_wh, g_charge_wh, g_last_soc, 0 };
    FILE *fp = fopen(MILEAGE_FILE, "wb");
    if (!fp) {
        ESP_LOGW(TAG, "SD: failed to write odometer.bin");
        return;
    }
    size_t n = fwrite(&f, 1, sizeof(f), fp);
    fclose(fp);
    ESP_LOGI(TAG, "SAVE wrote %u/%u bytes", (unsigned)n, (unsigned)sizeof(f));
    log_counters("SAVE");
    g_mi_last_save = g_total_mi;
}

/* Runs in the Timer Service task — keep this minimal, same discipline as
 * tr_display's mileage_timer_cb. */
static void mileage_timer_cb(TimerHandle_t xTimer)
{
    bool fresh = (xTaskGetTickCount() - g_last_frame_tick) < pdMS_TO_TICKS(SPEED_STALE_MS);

    if (fresh && g_cur_speed > 0.0f) {
        double delta = g_cur_speed * (MILEAGE_TICK_MS / 1000.0 / 3600.0);
        g_total_mi  += delta;
        g_trip_mi   += delta;
        g_charge_mi += delta;
    }

    if (g_total_mi - g_mi_last_save >= MILEAGE_SAVE_MI)
        g_save_dirty = true;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Task watchdog — LVGL task                                                   */
/*                                                                              */
/*  Same gap that caused waveform_tr_display's screen-freeze lockup exists      */
/*  here: CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0/1 only watches IDLE, so if    */
/*  the LVGL task blocks inside the touch driver (this board still reads the     */
/*  GT911 over I2C — only telemetry moved to RS-485) IDLE keeps running and      */
/*  keeps feeding the watchdog while the screen sits frozen.                     */
/*                                                                              */
/*  The LVGL task is created by the BSP so app_main has no handle for it, and    */
/*  esp_task_wdt_reset() only ever resets the CALLING task's entry — so an       */
/*  lv_timer is used to do both: its callback runs inside lv_timer_handler(),    */
/*  i.e. in the LVGL task's own context, letting it subscribe and feed itself.   */
/* ═══════════════════════════════════════════════════════════════════════════ */
#define LVGL_WDT_FEED_MS  1000   /* well under CONFIG_ESP_TASK_WDT_TIMEOUT_S (5 s) */

static void lvgl_wdt_cb(lv_timer_t *timer)
{
    static bool subscribed = false;

    if (!subscribed) {
        esp_err_t err = esp_task_wdt_add(NULL);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "LVGL task TWDT subscribe failed: %s", esp_err_to_name(err));
            return;   /* don't reset an entry we don't own */
        }
        subscribed = true;
        ESP_LOGI(TAG, "LVGL task subscribed to TWDT (feeding every %d ms)", LVGL_WDT_FEED_MS);
    }

    esp_task_wdt_reset();
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Power receive task — the sender broadcasts ONE telemetry frame to every     */
/*  display on the RS-485 pair; this gauge just takes the power_kw field and    */
/*  drives the needle. One wire format, one transport, shared with the speedo.  */
/* ═══════════════════════════════════════════════════════════════════════════ */
static void power_task(void *arg)
{
    telemetry_t t;
    float prev_speed = 0.0f;

    /* Watched by the TWDT — bounded per loop (bsp_display_lock's 200 ms timeout
     * + a 20 ms delay), so a stuck LVGL lock trips this well inside the 5 s
     * TWDT timeout. The mileage FATFS write in save_mileage() runs in this
     * loop too, same as tr_display's telemetry_task. */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (1) {
        esp_task_wdt_reset();

        if (rs485_rx_get(&t)) {
            float new_speed = (t.speed_mph < 0) ? 0 :
                              (t.speed_mph > 100) ? 100 : t.speed_mph;

            /* Save when speed drops below 1 mph (car just stopped) — same
             * trigger as tr_display. */
            if (new_speed < 1.0f && prev_speed >= 1.0f)
                g_save_dirty = true;

            bool is_run_mode    = (t.opmode == CAR_MODE_RUN);
            bool is_charge_mode = (t.opmode == CAR_MODE_CHARGE);

            /* Integrate the IVT cumulative energy counter, same rejection
             * threshold as tr_display for a shunt reset / bad frame. Energy put
             * IN while charging is kept out of every meter: the baseline still
             * follows the counter each frame, so driving picks up from the
             * post-charge value — same as tr_display. */
            double e = (double)t.energy_wh;
            if (g_energy_have_prev && !is_charge_mode) {
                double d = e - g_energy_prev;
                if (fabs(d) < ENERGY_MAX_STEP_WH) {
                    g_total_wh  += d;
                    g_trip_wh   += d;
                    g_charge_wh += d;
                }
            } else if (!g_energy_have_prev) {
                ESP_LOGI(TAG, "IVT: baseline latched at %.2f Wh", e);
            }
            g_energy_prev = e;
            g_energy_have_prev = true;

            /* Reset the "since last charge" meter on the rising edge into
             * Charge. */
            if (is_charge_mode && !s_was_charge_mode) {
                g_charge_wh = 0.0;
                g_charge_mi = 0.0;
                g_save_dirty = true;
            }
            s_was_charge_mode = is_charge_mode;

            /* SOC is only real with the contactors closed — remember the
             * last real reading and persist it on the falling edge, same as
             * tr_display. */
            bool bus_live = is_run_mode || is_charge_mode;
            if (bus_live) {
                g_last_soc = t.soc_pct;
            } else {
                if (s_was_bus_live) g_save_dirty = true;
            }
            s_was_bus_live = bus_live;

            g_cur_speed = new_speed;
            g_last_frame_tick = xTaskGetTickCount();
            prev_speed = new_speed;

            /* Advance the rolling-efficiency window (no-op until the trip
             * crosses the next 0.1 mi bucket). */
            roll_update();

            /* Periodic snapshot so miles vs Wh can be watched while driving,
             * same cadence as tr_display's telemetry_task. */
            static TickType_t last_dbg = 0;
            TickType_t now_dbg = xTaskGetTickCount();
            if (now_dbg - last_dbg >= pdMS_TO_TICKS(5000)) {
                last_dbg = now_dbg;
                log_counters("TICK");
            }

            /* 200 ms timeout rides out the adapter's lv_timer_handler() render. */
            if (bsp_display_lock(200) == ESP_OK) {
                power_ui_set_power(t.power_kw);
                power_ui_set_max_throttle_pct(t.max_throttle_pct);
                power_ui_set_car_mode(t.opmode);
                double rwh, rmi;
                roll_window(&rwh, &rmi);
                power_ui_set_whmi(g_trip_wh, g_trip_mi, g_charge_wh, g_charge_mi, rwh, rmi);
                power_ui_set_trip_mi(g_trip_mi);
                bsp_display_unlock();
            }
            ESP_LOGD(TAG, "power: %.1f kW", t.power_kw);
        }

        if (g_trip_reset) {
            g_trip_reset = false;
            g_trip_mi = 0.0;
            g_trip_wh = 0.0;
            roll_reset();   /* M is trip-based, so it resets with the trip */
            ESP_LOGI(TAG, "TRIP RESET");
            if (bsp_display_lock(200) == ESP_OK) {
                double rwh, rmi;
                roll_window(&rwh, &rmi);
                power_ui_set_whmi(g_trip_wh, g_trip_mi, g_charge_wh, g_charge_mi, rwh, rmi);
                power_ui_set_trip_mi(g_trip_mi);
                bsp_display_unlock();
            }
        }

        if (g_save_dirty) {
            g_save_dirty = false;
            save_mileage();
        }

        vTaskDelay(pdMS_TO_TICKS(20));   /* cap UI updates at ~50 Hz */
    }
}

void app_main(void)
{
    /* Mount SD card before loading mileage, same order as tr_display. */
    if (bsp_sdcard_mount() == ESP_OK) {
        s_sd_mounted = true;
        ESP_LOGI(TAG, "SD card mounted");
    } else {
        ESP_LOGW(TAG, "SD card not available — mileage/energy won't persist");
    }

    load_mileage();
    g_mi_last_save = g_total_mi;
    /* Anchor the rolling window at the restored trip point, same as
     * tr_display — the ring itself isn't persisted, so M measures from here
     * until it refills. */
    roll_reset();

    /* Display + LVGL */
    bsp_display_cfg_t cfg = {
        .lv_adapter_cfg   = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation         = ESP_LV_ADAPTER_ROTATE_0,
        .tear_avoid_mode  = ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL,
        .touch_flags      = {.swap_xy = 0, .mirror_x = 0, .mirror_y = 0},
    };
    bsp_display_start_with_config(&cfg);
    bsp_display_backlight_on();

    /* Build the power gauge */
    bsp_display_lock(-1);
    power_ui_init(lv_screen_active());
    power_ui_set_power(0.0f);
    {
        double rwh, rmi;
        roll_window(&rwh, &rmi);
        power_ui_set_whmi(g_trip_wh, g_trip_mi, g_charge_wh, g_charge_mi, rwh, rmi);
        power_ui_set_trip_mi(g_trip_mi);
    }
    power_ui_set_trip_cb(on_trip_reset);

    /* Runs in the LVGL task, so it both subscribes that task to the TWDT and
     * feeds it. If the touch read ever blocks on a wedged I2C bus, this timer
     * stops firing and the watchdog reboots us instead of leaving a dead dash. */
    lv_timer_create(lvgl_wdt_cb, LVGL_WDT_FEED_MS, NULL);

    bsp_display_unlock();

    /* RS-485 telemetry receiver — its own UART, nothing shared with touch. */
    ESP_ERROR_CHECK(rs485_rx_init());

    TimerHandle_t t_mile = xTimerCreate("mile", pdMS_TO_TICKS(MILEAGE_TICK_MS),
                                        pdTRUE, NULL, mileage_timer_cb);
    xTimerStart(t_mile, 0);

    /* 8192, not 4096: save_mileage() does fopen/fwrite through FATFS, and
     * newlib's full float formatting in the log calls burns real stack —
     * same sizing rationale as tr_display's telemetry_task. */
    xTaskCreate(power_task, "power", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "Power display ready (RS-485 telemetry)");
}
