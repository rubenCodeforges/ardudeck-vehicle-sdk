# Recipes

Each one is lifted from a file in [`examples/`](examples) that compiles in CI. Copy it,
change the names, ship it.

Working from nothing? [Build a rover](build-a-rover.md) puts all of these together in
order, on a vehicle you can run.

---

## Declare what the vehicle is

The manifest is what hides screens you cannot serve. Undeclared means hidden, never
broken.

```c src=examples/quad/ardudeck_link.c
/* Your own mode numbers, whatever they already are. */
enum { MODE_STABILIZE, MODE_ALTHOLD, MODE_LOITER, MODE_AUTO, MODE_RTL, MODE_LAND,
       MODE_FAILSAFE };

static const uint16_t MISSION_CMDS[] = {
  AD_NAV_WAYPOINT, AD_NAV_TAKEOFF, AD_NAV_LAND, AD_NAV_LOITER_TIME,
  AD_NAV_RETURN_TO_LAUNCH,
};

static const ad_mode_t MODES[] = {
  {MODE_STABILIZE, "Stabilize", 0},
  {MODE_LOITER,    "Loiter",    0},
  {MODE_AUTO,      "Auto",      AD_MODE_MISSION},   /* the mode that flies a plan */
  {MODE_LAND,      "Land",      AD_MODE_ARMED_ONLY},
  {MODE_FAILSAFE,  "Failsafe",  AD_MODE_TERMINAL},  /* reached, never chosen */
};

static const ad_capability_t CAPS = {
  .vendor = "acme",
  .model = "X500 quad",
  .firmware = "2.1.0",
  .frame = AD_FRAME_MULTIROTOR,
  .features = AD_FEAT_PARAMS | AD_FEAT_MISSION | AD_FEAT_COMMANDS |
              AD_FEAT_CALIBRATION | AD_FEAT_HOME,
  .mission_cmds = MISSION_CMDS,
  .mission_cmd_count = AD_COUNT(MISSION_CMDS),
  .mission_capacity = MAX_WAYPOINTS,
  .modes = MODES,
  .mode_count = AD_COUNT(MODES),
};
```

Full file: [`quad/ardudeck_link.c`](examples/quad/ardudeck_link.c)

---

## Parameters, including a dropdown

A number is one line. A dropdown needs a getter and a setter, because what you store is
your own enum and what crosses the wire is the position in the list.

```c src=examples/quad/ardudeck_link.c
static const char *const RTL_BEHAVIOUR[] = {"Land", "Hover and wait", "Return to pilot"};

static uint8_t rtl_behaviour = 0;

static bool rtl_get(const ad_param_t *p, float *out) {
  (void)p;
  *out = (float)rtl_behaviour;
  return true;
}

static bool rtl_set(const ad_param_t *p, float value) {
  (void)p;
  if (value < 0.0f || value > 2.0f) return false;
  rtl_behaviour = (uint8_t)value;
  return true;
}

static const ad_param_t PARAMS[] = {
  AD_F32("RATE_P",    &cfg.rate_p,       0.0f, 0.5f,   "",     "Rate loop gain"),
  AD_F32("RTL_ALT",   &cfg.rtl_alt_m,    5.0f, 120.0f, "m",    "Height it returns at"),
  AD_U16("WP_SPEED",  &cfg.wp_speed_cms, 100,  1500,   "cm/s", "Speed between waypoints"),
  AD_ENUM("RTL_ACTION", rtl_get, rtl_set, RTL_BEHAVIOUR, 3,
          "What it does once it arrives home"),
};
```

Full file: [`quad/ardudeck_link.c`](examples/quad/ardudeck_link.c) · reference:
[Parameters](api.md#parameters)

---

## Take a mission

`seq` is the index in the array. There is no home item at position 0.

```c src=examples/quad/ardudeck_link.c
static bool on_mission(const ad_wp_t *items, uint16_t count, char *why, size_t why_len,
                       void *user) {
  (void)user;
  if (count > MAX_WAYPOINTS) {
    snprintf(why, why_len, "%u waypoints, this aircraft holds %d", count, MAX_WAYPOINTS);
    return false;   /* the operator reads this sentence */
  }
  return fc_load_mission(items, count) == 0;
}
```

If your own app already loads missions, call that function here rather than writing a
second set of rules about what a valid plan is.

Full file: [`boat/ardudeck_link.c`](examples/boat/ardudeck_link.c)

---

## Commands, and refusing one with a reason

```c src=examples/quad/ardudeck_link.c
static bool on_command(uint16_t cmd, const float args[7], char *why, size_t why_len,
                       void *user) {
  (void)user;
  switch (cmd) {
    case AD_CMD_ARM: {
      const bool arm = args[0] != 0.0f;
      const bool forced = args[1] == 21196.0f;   /* the operator chose to force it */
      if (arm && !forced && prearm_failed(why, (unsigned)why_len)) return false;
      return fc_arm(arm, forced) == 0;
    }
    case AD_CMD_TAKEOFF:
      /* MAVLink puts the altitude in param7; the SDK hands it to you in a[0]. */
      return fc_takeoff(args[0]) == 0;
    case AD_CMD_SET_MODE:
      /* a[0] is one of YOUR mode ids. An id you never declared never reaches here. */
      return fc_set_mode((int)args[0]) == 0;
    default:
      snprintf(why, why_len, "This aircraft does not do that");
      return false;
  }
}
```

Full file: [`quad/ardudeck_link.c`](examples/quad/ardudeck_link.c) · reference:
[Commands](api.md#commands)

---

## A calibration with poses

Six orientations, each held until the operator accepts it.

```c src=examples/quad/ardudeck_link.c
static const ad_cal_pose_t ACCEL_POSES[] = {
  {"Level",       0.0f,   0.0f},
  {"Left side",  -90.0f,  0.0f},
  {"Right side",  90.0f,  0.0f},
  {"Nose down",   0.0f, -90.0f},
  {"Nose up",     0.0f,  90.0f},
  {"On its back", 180.0f, 0.0f},
};

static const ad_calibration_t CALS[] = {
  {
    .id = "accel",
    .name = "Accelerometer",
    .kind = AD_CAL_POSITIONAL,
    .requirements = AD_CAL_REQ_DISARMED | AD_CAL_REQ_STATIONARY,
    .warning = "Hold the frame in each position until it is accepted.",
    .poses = ACCEL_POSES,
    .pose_count = AD_COUNT(ACCEL_POSES),
  },
};
```

Then say which pose you are on, and finish:

```c
ardudeck_cal_progress("accel", &(ad_cal_progress_t){
  .step = pose_index, .step_done = captured, .percent = pct,
});

ardudeck_cal_done("accel", true, 0.96f, "Offsets stored");
ardudeck_param_changed("INS_ACCOFFS_X");   /* every value the routine wrote */
```

Full file: [`quad/ardudeck_link.c`](examples/quad/ardudeck_link.c) · reference:
[Calibration](calibration.md)

---

## A link failsafe that is actually fed

The SDK transmits whether or not anybody is listening, so its own sending tells you
nothing. Ask how long the ground station has been quiet.

```c
if (ardudeck_silent_for(now_ms()) > 3000 && is_flying()) fc_set_mode(MODE_RTL);
```

A ground station watching a mission sends little besides its heartbeat, which is exactly
when a failsafe built on anything else fires by mistake.

---

## Your link

The only board specific part. None of these knows anything about your vehicle.

**ESP-IDF**, UDP on the AP the board already runs. Answers whoever spoke to it, so no
address is ever typed in:

```c src=examples/transports/esp32_udp.c
static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  if (!have_peer || sock < 0) return;
  sendto(sock, buf, len, 0, (struct sockaddr *)&peer, sizeof peer);
}
```

**STM32**, DMA into a ring, handed over in at most two pieces:

```c src=examples/transports/stm32_uart.c
/* Call from your main loop, as often as you can. */
void ardudeck_link_poll(void) {
  const uint16_t write = (uint16_t)(RX_RING - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
  if (write > rx_read) {
    ardudeck_receive(&rx[rx_read], (size_t)(write - rx_read));
  } else if (write < rx_read) {
    ardudeck_receive(&rx[rx_read], (size_t)(RX_RING - rx_read));
    if (write > 0) ardudeck_receive(&rx[0], (size_t)write);
  }
  rx_read = write;
  ardudeck_tick(now_ms());
}
```

Full files: [ESP-IDF](examples/transports/esp32_udp.c) ·
[Arduino](examples/transports/arduino_serial.ino) ·
[STM32](examples/transports/stm32_uart.c) ·
[Zephyr](examples/transports/zephyr_uart.c) ·
[Linux](examples/transports/linux_udp.c)

---

---

[Getting started](getting-started.md) · [Examples](examples.md) · [Build a rover](build-a-rover.md) · [The contract](contract.md) · [API reference](api.md) · [Calibration](calibration.md) · [Links](transports.md) · [ardudeck-conform](conformance.md) · [When it is not working](troubleshooting.md)
