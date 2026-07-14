# waveform_power_display

ESP-IDF application for the ESP32-P4 Waveshare LCD board (`esp32p4`) that renders
a **DC-bus power gauge** — the LVGL replacement for the CircuitPython Qualia
power display (`qualia_tr_display`).

It is built on the same foundation as `waveform_tr_display` (the speed/dashboard
display): same board, BSP, LVGL adapter, fonts, and Triumph logo background. The
speedometer and its dashboard are replaced by a single power needle + ring with a
central `kW` / `%` readout, matching the Qualia power gauge:

- `-20 %` at 7 o'clock, `100 %` at 5 o'clock, 300° sweep
- `100 % == 110 kW` (`PWR_KW_AT_100` in `main/power_ui.c`)
- minor tick every 5 %, numeric label every 20 %
- regen (negative kW) pins the needle at the 7 o'clock start

## Swappable power source

The power value is fed through a transport-agnostic interface
(`main/power_source.h`):

```c
esp_err_t power_source_init(void);
bool      power_source_get(float *power_kw);
```

Three backends implement it; exactly one is compiled, chosen by a single line in
`main/power_source.h`:

```c
#define POWER_SOURCE  POWER_SOURCE_ESPNOW   // or _I2C, or _CAN
```

| Backend        | File                      | Status  | Notes |
|----------------|---------------------------|---------|-------|
| `POWER_SOURCE_ESPNOW` | `power_source_espnow.c` | **active default** | Exact drop-in for the Qualia: receives the sender's 5-byte `<fB` (power_kw, ready=0xA5) packet on channel 1. This board adopts the Qualia's MAC (`20:6E:F1:B8:E3:2C`) so the sender needs no changes. |
| `POWER_SOURCE_CAN`    | `power_source_can.c`    | implemented | Self-contained: joins the CAN bus and reads `POWER_ID` (2011) from the ZombieVerter (node 3) over CANopen SDO, like `speed_test_driver`. Needs a CAN transceiver on `TX=GPIO19 / RX=GPIO18` (edit the `#define`s to match your wiring). |
| `POWER_SOURCE_I2C`    | `power_source_i2c.c`    | scaffold | Master read of a framed power packet from the sender's I2C slave, reusing the waveform display's reframe+CRC. **Requires a sender-side change** — see the contract documented at the top of the file — because the current I2C telemetry packet carries no power field. |

> The ESP-NOW link is the default per request, but note the P4 does WiFi through
> its ESP32-C6 co-processor (esp-hosted), whose ESP-NOW build may not exactly
> match the plain-ESP32 sender. If it proves flaky, switch to `_CAN` (fully
> independent) or `_I2C` — the gauge code does not change.

## Build

```bash
idf.py set-target esp32p4
idf.py build
```

## Flash

```bash
idf.py -p <PORT> flash monitor
```

## Notes

- `sdkconfig` is git-ignored; configuration comes from `sdkconfig.defaults`.
- `managed_components/` is reconstructed from `dependencies.lock` on first build.
- The local BSP component lives in `components/`.
