#include "power_source.h"
#if POWER_SOURCE == POWER_SOURCE_ESPNOW

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"

/* ─────────────────────────────────────────────────────────────────────────────
 *  ESP-NOW power backend — drop-in replacement for the CircuitPython qualia.
 *
 *  The sender (speed_test_driver) unicasts a 5-byte packet to the qualia's MAC
 *  at ~5 Hz on channel 1:
 *      struct __attribute__((packed)) { float power_kw; uint8_t ready; }
 *  with ready == 0xA5 when the value is valid.
 *
 *  For unicast delivery to reach us, THIS board must present the same MAC the
 *  sender targets (receiverMAC in speed_test_driver/main/main.cpp). That is the
 *  qualia's factory STA MAC, set below — so the sender needs no changes.
 *
 *  NOTE: the P4 does WiFi through its ESP32-C6 co-processor (esp-hosted), whose
 *  ESP-NOW build may not exactly match the plain-ESP32 sender. If this link
 *  proves flaky, switch POWER_SOURCE to _I2C or _CAN in power_source.h — the
 *  gauge does not change.
 * ──────────────────────────────────────────────────────────────────────────── */

/* Qualia STA MAC the sender unicasts power to. Must equal receiverMAC[] in the
 * sender firmware. Update BOTH together if you re-pair to a different board. */
static const uint8_t RECEIVER_MAC[6] = {0x20, 0x6E, 0xF1, 0xB8, 0xE3, 0x2C};

#define POWER_PACKET_LEN  5   /* sizeof(float) + sizeof(uint8_t), packed */
#define POWER_READY_MAGIC 0xA5

static const char *TAG = "pwr_espnow";
static QueueHandle_t s_queue;

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len != POWER_PACKET_LEN) return;

    float   power_kw;
    uint8_t ready;
    memcpy(&power_kw, data, sizeof(power_kw));       /* little-endian float */
    ready = data[sizeof(power_kw)];
    if (ready != POWER_READY_MAGIC) return;

    xQueueOverwrite(s_queue, &power_kw);
}

esp_err_t power_source_init(void)
{
    s_queue = xQueueCreate(1, sizeof(float));
    if (!s_queue) return ESP_ERR_NO_MEM;

    /* Event loop + NVS are initialised by app_main before this call. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, RECEIVER_MAC));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Sender locks to channel 1 (CircuitPython default when not joined). */
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));

    ESP_LOGI(TAG, "ESP-NOW power receiver ready (MAC %02x:%02x:%02x:%02x:%02x:%02x, ch1)",
             RECEIVER_MAC[0], RECEIVER_MAC[1], RECEIVER_MAC[2],
             RECEIVER_MAC[3], RECEIVER_MAC[4], RECEIVER_MAC[5]);
    return ESP_OK;
}

bool power_source_get(float *power_kw)
{
    return xQueueReceive(s_queue, power_kw, 0) == pdTRUE;
}

#endif /* POWER_SOURCE == POWER_SOURCE_ESPNOW */
