#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "power_ui.h"
#include "power_source.h"

static const char *TAG = "power_display";

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Power receive task — pull the latest DC-bus power from whichever transport  */
/*  is compiled in (see power_source.h) and drive the needle.                   */
/* ═══════════════════════════════════════════════════════════════════════════ */
static void power_task(void *arg)
{
    float kw;
    while (1) {
        if (power_source_get(&kw)) {
            /* 200 ms timeout rides out the adapter's lv_timer_handler() render. */
            if (bsp_display_lock(200) == ESP_OK) {
                power_ui_set_power(kw);
                bsp_display_unlock();
            }
            ESP_LOGD(TAG, "power: %.1f kW", kw);
        }
        vTaskDelay(pdMS_TO_TICKS(20));   /* cap UI updates at ~50 Hz */
    }
}

void app_main(void)
{
    /* NVS + default event loop — required by the WiFi/ESP-NOW power backend,
     * harmless for the I2C/CAN backends. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_event_loop_create_default());

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

    /* Bring up the selected power transport (after the display so the I2C
     * backend can borrow the BSP's I2C bus handle). */
    ESP_ERROR_CHECK(power_source_init());

    xTaskCreate(power_task, "power", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Power display ready");
}
