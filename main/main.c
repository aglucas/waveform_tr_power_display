#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "power_ui.h"
#include "rs485_rx.h"

static const char *TAG = "power_display";

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Power receive task — the sender broadcasts ONE telemetry frame to every     */
/*  display on the RS-485 pair; this gauge just takes the power_kw field and    */
/*  drives the needle. (The old swappable ESP-NOW/I2C/CAN power_source backends */
/*  are gone — one wire format, one transport, shared with the speedo.)         */
/* ═══════════════════════════════════════════════════════════════════════════ */
static void power_task(void *arg)
{
    telemetry_t t;
    while (1) {
        if (rs485_rx_get(&t)) {
            /* 200 ms timeout rides out the adapter's lv_timer_handler() render. */
            if (bsp_display_lock(200) == ESP_OK) {
                power_ui_set_power(t.power_kw);
                bsp_display_unlock();
            }
            ESP_LOGD(TAG, "power: %.1f kW", t.power_kw);
        }
        vTaskDelay(pdMS_TO_TICKS(20));   /* cap UI updates at ~50 Hz */
    }
}

void app_main(void)
{
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
    bsp_display_unlock();

    /* RS-485 telemetry receiver — its own UART, nothing shared with touch. */
    ESP_ERROR_CHECK(rs485_rx_init());

    xTaskCreate(power_task, "power", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Power display ready (RS-485 telemetry)");
}
