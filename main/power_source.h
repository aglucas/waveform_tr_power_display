#pragma once
#include <stdbool.h>
#include "esp_err.h"

/* ─────────────────────────────────────────────────────────────────────────────
 *  power_source — transport-agnostic DC-bus power feed
 *
 *  The gauge UI (power_ui) and app_main only ever see these two calls. The
 *  actual transport is chosen at compile time by POWER_SOURCE below; each
 *  backend lives in its own file and compiles to nothing unless selected:
 *
 *      POWER_SOURCE_ESPNOW → power_source_espnow.c   (qualia's 5-byte link)
 *      POWER_SOURCE_I2C    → power_source_i2c.c      (read from sender's slave)
 *      POWER_SOURCE_CAN    → power_source_can.c      (SDO read off the CAN bus)
 *
 *  To switch transports, change the one line below (or pass -DPOWER_SOURCE=…
 *  from the build). Nothing else in the project needs to change.
 * ──────────────────────────────────────────────────────────────────────────── */

#define POWER_SOURCE_ESPNOW  0
#define POWER_SOURCE_I2C     1
#define POWER_SOURCE_CAN     2

#ifndef POWER_SOURCE
#define POWER_SOURCE  POWER_SOURCE_ESPNOW   /* ← change this one line to switch */
#endif

/* Bring up the selected transport (radio / I2C master / TWAI + its task). */
esp_err_t power_source_init(void);

/* Non-blocking. Returns true and writes the newest DC-bus power (kW) into
 * *power_kw if a fresh reading has arrived since the last call; false if
 * nothing new. Regen is negative. */
bool power_source_get(float *power_kw);
