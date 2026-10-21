#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "rs485_rx.h"

/* ── RS-485 telemetry receiver + command transmitter ─────────────────────────
 * Listens to the sender's 25 Hz telemetry broadcast on a dedicated UART. This
 * replaces the I2C link that shared a bus with the touch controller — a fault
 * on this wire can corrupt a frame (which the CRC rejects) but can never
 * wedge a bus, block the touch read, or freeze the UI.
 *
 * The board was originally receive-only (transceiver DE tied low, 10k
 * pulldown) but has since been rewired with DE under GPIO control, so this
 * node can now also transmit: the on-screen max-throttle button sends a
 * command frame back to speed_test_driver over the same pair (see
 * power_cmd_t below and cmd_tx_task). If the sender is off, the fail-safe
 * receiver idles the line and no telemetry frames arrive — telemetry_task's staleness guard
 * (SPEED_STALE_MS) then freezes the odometer rather than integrating a stale
 * speed.
 *
 * Telemetry frame format (must match speed_test_driver/main/main.cpp):
 *     [0xAA][0x55][ telemetry_t payload ][CRC8 over payload]
 * Command frame format (this node -> speed_test_driver):
 *     [0xC3][0x3C][ power_cmd_t payload ][CRC8 over payload]
 * Distinct sync bytes keep the two frame kinds from ever being confused. A
 * UART byte stream has no message boundaries, so we accumulate bytes, scan
 * for a sync marker, and accept a payload only when the CRC checks out. */

#define RS485_UART        UART_NUM_1
#define RS485_BAUD        115200     /* must match the sender */

/* Command TX (max-throttle). speed_test_driver's pollPowerCommand() parses
 * this frame, applies it locally, and forwards it to the ZombieVerter over
 * CAN via setSdoValue(THROTMAX_PARAM_ID, ...) — see
 * speed_test_driver/main/main.cpp. */
#define RS485_TX_ENABLED  1

/* 40-pin-header GPIOs wired to the transceiver. Clear of BSP assignments
 * (I2C=7/8, I2S=9-13, LCD=23/26/27, SD=39-44, amp=53). TX/RX/RTS route
 * through the GPIO matrix, so any output-capable GPIO would work for any of
 * the three — but which physical pin the transceiver's TXD/RXD land on is
 * fixed by the wiring harness, not free to choose.
 *
 * GPIO20/21 are swapped from what you'd guess: the transceiver's TXD/RXD are
 * straight-matched to these header pins instead of crossed (same harness bug
 * waveform_tr_display hit), so the ESP32's RX must be GPIO20 (wired to the
 * transceiver's TXD, its recovered bus data) and the ESP32's TX must be
 * GPIO21 (wired to the transceiver's RXD, its data input). */
#define RS485_TX_PIN      GPIO_NUM_21   /* -> transceiver DI */
#define RS485_RX_PIN      GPIO_NUM_20   /* <- transceiver RO */
#define RS485_RTS_PIN     GPIO_NUM_2    /* -> transceiver DE/RE, auto-driven by UART HW */

#define TELEM_SYNC0       0xAA
#define TELEM_SYNC1       0x55
#define TELEM_FRAME_LEN   (2 + (int)sizeof(telemetry_t) + 1)   /* sync(2)+payload+crc(1) */

#define PWRCMD_SYNC0      0xC3
#define PWRCMD_SYNC1      0x3C
#define PWRCMD_FRAME_LEN  (2 + (int)sizeof(power_cmd_t) + 1)   /* sync(2)+payload+crc(1) */

/* Gap between repeats of a command burst — see rs485_send_max_throttle_pct(). */
#define PWRCMD_BURST_GAP_MS   15
#define PWRCMD_BURST_COUNT    3

/* Accumulator sized for a few frames: at 25 Hz the scan drains it every pass,
 * so this only needs headroom for a burst after a stall, not sustained input. */
#define RX_ACC_LEN        (TELEM_FRAME_LEN * 6)

static const char *TAG = "rs485_rx";
static QueueHandle_t s_queue;
#if RS485_TX_ENABLED
static QueueHandle_t s_cmd_queue;   /* holds pending float kw values to send */
#endif

/* Dallas/Maxim CRC8 (poly 0x8C, LSB-first) — must match sender's ow_crc8(). */
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

/* Range checks: the CRC proves the bytes arrived intact; this proves the sender
 * itself wasn't fed garbage (SDO glitch, uninitialised field). */
static bool telemetry_valid(const telemetry_t *t)
{
    if (t->ready != TELEMETRY_READY_MAGIC) return false;
    if (!isfinite(t->speed_mph)  || t->speed_mph  <    0.0f || t->speed_mph  >  250.0f) return false;
    if (!isfinite(t->temp_bat_c) || t->temp_bat_c <  -40.0f || t->temp_bat_c >  150.0f) return false;
    if (!isfinite(t->temp_mot_c) || t->temp_mot_c <  -40.0f || t->temp_mot_c >  300.0f) return false;
    if (!isfinite(t->temp_inv_c) || t->temp_inv_c <  -40.0f || t->temp_inv_c >  200.0f) return false;
    if (!isfinite(t->temp_isa_c) || t->temp_isa_c <  -40.0f || t->temp_isa_c >  200.0f) return false;
    if (!isfinite(t->soc_pct)    || t->soc_pct    <    0.0f || t->soc_pct    >  100.0f) return false;
    if (!isfinite(t->udc_v)      || t->udc_v      <    0.0f || t->udc_v      > 1000.0f) return false;
    /* Cumulative energy in Wh; ±100,000 kWh brackets any plausible counter value. */
    if (!isfinite(t->energy_wh)  || t->energy_wh  < -1e8f  || t->energy_wh  >  1e8f)   return false;
    if (!isfinite(t->power_kw)   || t->power_kw   < -300.0f || t->power_kw  >  300.0f) return false;
    if (!isfinite(t->max_throttle_pct) || t->max_throttle_pct < 0.0f || t->max_throttle_pct > 100.0f) return false;
    if (t->opmode >= CAR_MODE_LAST) return false;
    return true;
}

static void rx_task(void *arg)
{
    static uint8_t acc[RX_ACC_LEN];
    int fill = 0;

    /* Watched by the TWDT for symmetry with telemetry_task. uart_read_bytes()
     * always returns within its 100 ms timeout, so a trip here means the UART
     * driver itself is broken — reboot is the right answer. */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    /* Log link health once a second, only while it's bad. */
    uint32_t   ok_count = 0, rej_count = 0;
    TickType_t last_report = xTaskGetTickCount();

    while (1) {
        esp_task_wdt_reset();

        int n = uart_read_bytes(RS485_UART, acc + fill, RX_ACC_LEN - fill,
                                pdMS_TO_TICKS(100));
        if (n > 0) fill += n;

        /* Scan for complete frames; keep whatever tail remains for next pass. */
        int i = 0;
        while (fill - i >= TELEM_FRAME_LEN) {
            if (acc[i] != TELEM_SYNC0 || acc[i + 1] != TELEM_SYNC1) { i++; continue; }
            const uint8_t *payload = &acc[i + 2];
            if (crc8(payload, sizeof(telemetry_t)) != payload[sizeof(telemetry_t)]) {
                /* Sync bytes can occur inside a payload; a CRC miss here is not
                 * necessarily corruption. Step one byte and keep scanning. */
                i++;
                rej_count++;
                continue;
            }
            telemetry_t t;
            memcpy(&t, payload, sizeof(t));
            if (telemetry_valid(&t)) {
                xQueueOverwrite(s_queue, &t);
                ok_count++;
            } else {
                rej_count++;
            }
            i += TELEM_FRAME_LEN;
        }
        if (i > 0) {
            memmove(acc, acc + i, fill - i);
            fill -= i;
        }
        /* A full accumulator with no frame found means sustained garbage
         * (wrong baud, floating line). Start over rather than wedging. */
        if (fill >= RX_ACC_LEN) fill = 0;

        TickType_t now = xTaskGetTickCount();
        if (now - last_report >= pdMS_TO_TICKS(1000)) {
            /* Silence here used to mean "healthy", but it's indistinguishable
             * from "nothing is arriving at all" (dead sender, unplugged pair,
             * wedged UART) — the exact case that makes the display look hung
             * with no clue why. Report every second: a WARN when zero valid
             * frames landed (link is down), an INFO otherwise so a hang is
             * never silent. */
            if (ok_count == 0)
                ESP_LOGW(TAG, "RS-485: NO valid frames in last 1s (%lu rejected) — link down?",
                         (unsigned long)rej_count);
            else if (rej_count)
                ESP_LOGW(TAG, "RS-485: %lu ok / %lu bad in last 1s (noise or baud mismatch?)",
                         (unsigned long)ok_count, (unsigned long)rej_count);
            last_report = now;
            ok_count = rej_count = 0;
        }
    }
}

#if RS485_TX_ENABLED
/* Sends each queued max-throttle command as a burst of PWRCMD_BURST_COUNT
 * frames, PWRCMD_BURST_GAP_MS apart. The bus has no arbitration — a burst sent
 * while speed_test_driver is mid-broadcast corrupts both frames, but the CRC
 * makes that self-evident (dropped, never misapplied), and repeating a few
 * times against the sender's 40 ms telemetry period makes it likely at least
 * one copy lands in a gap. This is a manual, user-initiated action, so
 * there's no ack/retry: if every copy in a burst is missed, the user just
 * taps again. Runs in its own task so a UI tap never blocks on UART timing,
 * and so rx_task's receive loop is never delayed by a send. */
static void cmd_tx_task(void *arg)
{
    float pct;
    while (1) {
        if (xQueueReceive(s_cmd_queue, &pct, portMAX_DELAY) != pdTRUE) continue;

        power_cmd_t cmd = { .max_throttle_pct = pct, .ready = PWRCMD_READY_MAGIC };
        uint8_t frame[PWRCMD_FRAME_LEN];
        frame[0] = PWRCMD_SYNC0;
        frame[1] = PWRCMD_SYNC1;
        memcpy(&frame[2], &cmd, sizeof(cmd));
        frame[2 + sizeof(cmd)] = crc8((const uint8_t *)&cmd, sizeof(cmd));

        for (int i = 0; i < PWRCMD_BURST_COUNT; i++) {
            uart_write_bytes(RS485_UART, frame, sizeof(frame));
            if (i + 1 < PWRCMD_BURST_COUNT) vTaskDelay(pdMS_TO_TICKS(PWRCMD_BURST_GAP_MS));
        }
        ESP_LOGI(TAG, "sent max-throttle command: %.0f%% (x%d burst)", pct, PWRCMD_BURST_COUNT);
    }
}
#endif /* RS485_TX_ENABLED */

esp_err_t rs485_rx_init(void)
{
    s_queue = xQueueCreate(1, sizeof(telemetry_t));
    if (!s_queue) return ESP_ERR_NO_MEM;
#if RS485_TX_ENABLED
    s_cmd_queue = xQueueCreate(4, sizeof(float));
    if (!s_cmd_queue) return ESP_ERR_NO_MEM;
#endif

    uart_config_t cfg = {
        .baud_rate  = RS485_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /* TX buffer size 0 == blocking TX (writes straight to the HW FIFO, no ring
     * buffer). uart_driver_install() requires a nonzero TX buffer to exceed the
     * HW FIFO length (128 on this UART) or it fails with "uart tx buffer length
     * error" — PWRCMD_FRAME_LEN*4 (32 bytes) was neither 0 nor big enough. Since
     * cmd_tx_task only ever writes an 8-byte frame from its own task, blocking
     * is fine and simpler than sizing a ring buffer we don't need. */
    ESP_ERROR_CHECK(uart_driver_install(RS485_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART, &cfg));
#if RS485_TX_ENABLED
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART, RS485_TX_PIN, RS485_RX_PIN,
                                 RS485_RTS_PIN, UART_PIN_NO_CHANGE));
    /* Hardware half duplex: UART peripheral drives RTS (-> DE/RE) itself,
     * asserted only while actually transmitting — same approach as
     * speed_test_driver's rs485Init(). No manual GPIO toggling, so there's no
     * software race between "byte queued" and "byte on the wire". */
    ESP_ERROR_CHECK(uart_set_mode(RS485_UART, UART_MODE_RS485_HALF_DUPLEX));
#else
    /* TX disabled — leave TX/RTS untouched (UART_PIN_NO_CHANGE) and stay in
     * plain UART mode. DE/RE on the transceiver is never asserted, so this
     * node is electrically receive-only regardless of what's wired to it. */
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART, UART_PIN_NO_CHANGE, RS485_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif

    xTaskCreate(rx_task, "rs485_rx", 4096, NULL, 5, NULL);
#if RS485_TX_ENABLED
    xTaskCreate(cmd_tx_task, "rs485_cmd_tx", 3072, NULL, 5, NULL);
#endif

#if RS485_TX_ENABLED
    ESP_LOGI(TAG, "RS-485 ready (TX GPIO %d, RX GPIO %d, DE/RE GPIO %d, %d baud, "
                  "telemetry %d-byte / command %d-byte frames)",
             RS485_TX_PIN, RS485_RX_PIN, RS485_RTS_PIN, RS485_BAUD,
             TELEM_FRAME_LEN, PWRCMD_FRAME_LEN);
#else
    ESP_LOGI(TAG, "RS-485 ready, receive-only (RX GPIO %d, %d baud, %d-byte frames) "
                  "— command TX disabled, see RS485_TX_ENABLED",
             RS485_RX_PIN, RS485_BAUD, TELEM_FRAME_LEN);
#endif
    return ESP_OK;
}

bool rs485_rx_get(telemetry_t *out)
{
    return xQueueReceive(s_queue, out, 0) == pdTRUE;
}

void rs485_send_max_throttle_pct(float pct)
{
#if RS485_TX_ENABLED
    xQueueSend(s_cmd_queue, &pct, 0);
#else
    ESP_LOGW(TAG, "RS-485 command TX disabled — dropping max-throttle request (%.0f%%)", pct);
#endif
}
