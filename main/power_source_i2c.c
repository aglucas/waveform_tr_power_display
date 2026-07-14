#include "power_source.h"
#if POWER_SOURCE == POWER_SOURCE_I2C

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "bsp/esp-bsp.h"

/* ─────────────────────────────────────────────────────────────────────────────
 *  I2C power backend — this board acts as I2C master and reads a framed power
 *  packet from the sender's I2C slave, reusing the waveform display's proven
 *  reframe-and-CRC approach (torn/underrun reads are dropped, never shown).
 *
 *  SENDER-SIDE CONTRACT (must be added to speed_test_driver — it does NOT emit
 *  this yet; the existing 0x42 slave streams the 30-byte TelemetryPacket, which
 *  carries no power field):
 *
 *      Expose a framed power snapshot the master can resync to:
 *          [0xAA][0x55][ float power_kw ][ CRC8 over the 4 payload bytes ]
 *      CRC8 = Dallas/Maxim poly 0x8C, LSB-first (sender's ow_crc8()).
 *
 *  Until the sender implements that, this backend links and runs but simply
 *  never reports a fresh value. Point PWR_I2C_ADDR at whichever slave address
 *  the sender uses for the power frame.
 * ──────────────────────────────────────────────────────────────────────────── */

#define PWR_I2C_ADDR     0x43   /* TODO: sender's power-frame slave address */
#define PWR_POLL_MS      20
#define I2C_TIMEOUT_MS   10

#define I2C_SYNC0        0xAA
#define I2C_SYNC1        0x55
#define PWR_PAYLOAD_LEN  ((int)sizeof(float))                 /* 4 bytes */
#define I2C_FRAME_LEN    (2 + PWR_PAYLOAD_LEN + 1)            /* sync(2)+payload(4)+crc(1)=7 */
#define I2C_READ_LEN     (I2C_FRAME_LEN * 2)                  /* window to resync on */

#define I2C_RECOVER_AFTER_ERRS  5
#define I2C_RECOVER_MIN_GAP_MS  1000

static const char *TAG = "pwr_i2c";
static QueueHandle_t s_queue;
static i2c_master_bus_handle_t s_bus;

/* Dallas/Maxim CRC8 (poly 0x8C, LSB-first) — must match the sender's ow_crc8(). */
static uint8_t crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (int b = 0; b < 8; b++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

static void poll_task(void *arg)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)arg;
    uint8_t buf[I2C_READ_LEN];
    uint32_t consec_err = 0;
    TickType_t last_reset = 0;

    while (1) {
        esp_err_t err = i2c_master_receive(dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
        if (err == ESP_OK) {
            consec_err = 0;
            /* Scan the window newest-first for a CRC-valid frame. */
            for (int i = I2C_READ_LEN - I2C_FRAME_LEN; i >= 0; i--) {
                if (buf[i] != I2C_SYNC0 || buf[i + 1] != I2C_SYNC1) continue;
                const uint8_t *payload = &buf[i + 2];
                if (crc8(payload, PWR_PAYLOAD_LEN) != payload[PWR_PAYLOAD_LEN]) continue;
                float power_kw;
                memcpy(&power_kw, payload, sizeof(power_kw));
                if (isfinite(power_kw)) {
                    xQueueOverwrite(s_queue, &power_kw);
                    break;
                }
            }
        } else if (++consec_err >= I2C_RECOVER_AFTER_ERRS &&
                   xTaskGetTickCount() - last_reset >= pdMS_TO_TICKS(I2C_RECOVER_MIN_GAP_MS)) {
            i2c_master_bus_reset(s_bus);
            ESP_LOGW(TAG, "I2C bus wedged (%s) — reset", esp_err_to_name(err));
            last_reset = xTaskGetTickCount();
            consec_err = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(PWR_POLL_MS));
    }
}

esp_err_t power_source_init(void)
{
    s_queue = xQueueCreate(1, sizeof(float));
    if (!s_queue) return ESP_ERR_NO_MEM;

    /* Reuse the BSP's I2C bus (GPIO7=SDA, GPIO8=SCL, shared with touch/codec). */
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) {
        ESP_LOGE(TAG, "BSP I2C bus not ready — call after bsp_display_start");
        return ESP_ERR_INVALID_STATE;
    }
    s_bus = bus;

    /* 100 kHz for noise margin on a long in-vehicle run (see waveform i2c_rx.c). */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = PWR_I2C_ADDR,
        .scl_speed_hz    = 100000,
    };
    i2c_master_dev_handle_t dev;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));

    xTaskCreate(poll_task, "pwr_i2c", 4096, (void *)dev, 5, NULL);
    ESP_LOGI(TAG, "I2C power receiver ready (addr=0x%02x, SDA=GPIO7, SCL=GPIO8)", PWR_I2C_ADDR);
    return ESP_OK;
}

bool power_source_get(float *power_kw)
{
    return xQueueReceive(s_queue, power_kw, 0) == pdTRUE;
}

#endif /* POWER_SOURCE == POWER_SOURCE_I2C */
