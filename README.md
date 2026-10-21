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

## Power source

The power value arrives over RS-485 (`main/rs485_rx.c` / `rs485_rx.h`), as one
field (`power_kw`) in the shared `telemetry_t` frame broadcast to every display
on the bus — the same wire format and transport used by `waveform_tr_display`.
This display also sends max-throttle commands back over the same pair.

The older swappable ESP-NOW / I2C / CAN `power_source` backends have been
removed; RS-485 is now the only transport.

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
