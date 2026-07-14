#include "power_source.h"
#if POWER_SOURCE == POWER_SOURCE_CAN

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

/* ─────────────────────────────────────────────────────────────────────────────
 *  CAN power backend — self-contained: this board joins the CAN bus and reads
 *  DC-bus power straight from the ZombieVerter over CANopen SDO, exactly the way
 *  speed_test_driver does. No dependency on the speedo sender at all.
 *
 *  HARDWARE: the P4 board needs a CAN transceiver on the TX/RX pins below.
 *  Defaults match the speedo sender (TX=GPIO19, RX=GPIO18); change to your
 *  wiring before flashing.
 * ──────────────────────────────────────────────────────────────────────────── */

#define CAN_TX_PIN        GPIO_NUM_19   /* TODO: confirm against P4 board wiring */
#define CAN_RX_PIN        GPIO_NUM_18   /* TODO: confirm against P4 board wiring */

/* CANopen SDO — mirrors speed_test_driver/main/main.cpp */
#define ZOMBIE_NODE_ID    3
#define SDO_REQ_ID        (0x600U + ZOMBIE_NODE_ID)   /* 0x603 master→slave */
#define SDO_REP_ID        (0x580U + ZOMBIE_NODE_ID)   /* 0x583 slave→master */
#define SDO_INDEX_BASE    0x2100U
#define SDO_READ          0x40
#define SDO_ABORT         0x80

#define POWER_ID          2011   /* DC bus power, kW (udc*idc already computed) */
#define POWER_POLL_MS     200    /* 5 Hz, same cadence the sender used */

static const char *TAG = "pwr_can";
static QueueHandle_t s_queue;

static inline uint64_t millis(void) { return (uint64_t)(esp_timer_get_time() / 1000ULL); }

static void request_sdo(int id)
{
    uint16_t index  = SDO_INDEX_BASE | (uint16_t)(id >> 8);
    uint8_t  sub    = (uint8_t)(id & 0xFF);

    twai_message_t msg = {0};
    msg.identifier       = SDO_REQ_ID;
    msg.data_length_code = 8;
    msg.data[0] = SDO_READ;
    msg.data[1] = (uint8_t)(index & 0xFF);
    msg.data[2] = (uint8_t)(index >> 8);
    msg.data[3] = sub;
    twai_transmit(&msg, pdMS_TO_TICKS(10));
}

/* Returns the parameter value, or NAN on timeout/abort. ZombieVerter SDO values
 * are fixed-point with a /32 scale (raw / 32.0), identical to the sender. */
static float get_sdo_value(int id)
{
    uint16_t index = SDO_INDEX_BASE | (uint16_t)(id >> 8);
    uint8_t  sub   = (uint8_t)(id & 0xFF);

    request_sdo(id);

    uint64_t deadline = millis() + 50;
    while (millis() < deadline) {
        twai_message_t rx = {0};
        if (twai_receive(&rx, pdMS_TO_TICKS(10)) == ESP_OK) {
            if (rx.identifier       == SDO_REP_ID &&
                rx.data_length_code == 8          &&
                rx.data[0]          != SDO_ABORT  &&
                rx.data[1]          == (uint8_t)(index & 0xFF) &&
                rx.data[2]          == (uint8_t)(index >> 8)   &&
                rx.data[3]          == sub) {
                int32_t raw;
                memcpy(&raw, &rx.data[4], 4);
                return (float)(raw / 32.0);
            }
        }
        taskYIELD();
    }
    return NAN;
}

/* Keep the TWAI controller alive — it can latch bus-off on noise and never
 * recovers on its own. Same restart sequence the sender runs. */
static void check_twai_health(void)
{
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) return;
    if (st.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGE(TAG, "TWAI bus-off — initiating recovery");
        twai_initiate_recovery();
    } else if (st.state == TWAI_STATE_STOPPED) {
        ESP_LOGW(TAG, "TWAI stopped — restarting");
        twai_start();
    }
}

static void poll_task(void *arg)
{
    while (1) {
        check_twai_health();
        float v = get_sdo_value(POWER_ID);
        if (!isnan(v))
            xQueueOverwrite(s_queue, &v);
        else
            ESP_LOGW(TAG, "SDO timeout for POWER_ID %d", POWER_ID);
        vTaskDelay(pdMS_TO_TICKS(POWER_POLL_MS));
    }
}

esp_err_t power_source_init(void)
{
    s_queue = xQueueCreate(1, sizeof(float));
    if (!s_queue) return ESP_ERR_NO_MEM;

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g, &t, &f);
    if (err != ESP_OK) { ESP_LOGE(TAG, "TWAI install failed: %s", esp_err_to_name(err)); return err; }
    err = twai_start();
    if (err != ESP_OK) { ESP_LOGE(TAG, "TWAI start failed: %s", esp_err_to_name(err)); return err; }

    ESP_LOGI(TAG, "CAN power source ready (500 kbps, TX=%d RX=%d, node %d, POWER_ID %d)",
             CAN_TX_PIN, CAN_RX_PIN, ZOMBIE_NODE_ID, POWER_ID);
    xTaskCreate(poll_task, "pwr_can", 4096, NULL, 5, NULL);
    return ESP_OK;
}

bool power_source_get(float *power_kw)
{
    return xQueueReceive(s_queue, power_kw, 0) == pdTRUE;
}

#endif /* POWER_SOURCE == POWER_SOURCE_CAN */
