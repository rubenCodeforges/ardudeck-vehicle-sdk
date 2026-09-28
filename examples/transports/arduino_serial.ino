/*
 * Arduino. Serial, and nothing else.
 *
 * The shortest path from nothing to a vehicle on a map. Works on an ESP32 or a board with
 * a spare hardware serial; on an Uno, use Serial and flash the sketch over ICSP, because
 * MAVLink and the bootloader cannot share one port.
 *
 * Put the SDK in libraries/ardudeck/ and this compiles as it stands.
 */

extern "C" {
#include "ardudeck.h"
}

/* 57600 rather than 115200: it is what most telemetry radios default to. */
static const long BAUD = 57600;

static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  Serial1.write(buf, len);
}

static uint32_t now_ms(void) { return millis(); }

/* ─── the smallest vehicle that shows up ───────────────────────────────────── */

static const ad_mode_t MODES[] = {
  {0, "Manual", 0},
};

static const ad_capability_t CAPS = {
  .vendor = "workshop",
  .model = "test rig",
  .firmware = "0.1.0",
  .frame = AD_FRAME_ROVER,
  .features = 0,            /* rung 0 only: a dot on the map and a horizon */
  .modes = MODES,
  .mode_count = 1,
};

static ad_config_t CONFIG = {
  .caps = &CAPS,
  .send = sink,
  .now_ms = now_ms,
};

void setup() {
  Serial1.begin(BAUD);
  ardudeck_begin(&CONFIG);
}

void loop() {
  while (Serial1.available()) {
    uint8_t byte = (uint8_t)Serial1.read();
    ardudeck_receive(&byte, 1);
  }

  /* Your real position goes here. Fix 0 means "do not draw me", which is the honest
     answer before a GPS has locked, and better than a marker at 0,0. */
  ardudeck_position(52.5163, 13.3777, 0.0f, 90.0f, 3, 9);
  ardudeck_status(0, 0, 12.4f, false);

  ardudeck_tick(now_ms());
  delay(20);
}
