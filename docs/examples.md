# Examples

Every snippet below is lifted from a file in this repository that compiles in CI. Copy
one, change the names, ship it.

| File | Lines | Use it for |
|---|---|---|
| [`examples/transports/arduino_serial.ino`](examples/transports/arduino_serial.ino) | 65 | The smallest thing that appears on a map |
| [`examples/rover/`](examples/rover) | 4 files | **A firmware you can run**, and the integration bolted on beside it |
| [`examples/boat/ardudeck_link.c`](examples/boat/ardudeck_link.c) | 208 | A surface vehicle, rungs 0 to 4, coverage calibration |
| [`examples/quad/ardudeck_link.c`](examples/quad/ardudeck_link.c) | 285 | A multirotor: attitude, altitude, takeoff, poses, dropdowns |
| [`examples/transports/esp32_udp.c`](examples/transports/esp32_udp.c) | 75 | ESP-IDF, UDP on the board's own AP |
| [`examples/transports/stm32_uart.c`](examples/transports/stm32_uart.c) | 59 | STM32 HAL, UART with DMA receive |
| [`examples/transports/zephyr_uart.c`](examples/transports/zephyr_uart.c) | 62 | Zephyr, interrupt driven UART |
| [`examples/transports/linux_udp.c`](examples/transports/linux_udp.c) | 73 | Linux or a companion computer, and SITL |

---

## Hello, vehicle

Everything you need to be a dot on a map with a live horizon. This is the whole sketch,
not an excerpt.

```c
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
  Serial1.begin(57600);
  ardudeck_begin(&CONFIG);
}

void loop() {
  while (Serial1.available()) {
    uint8_t byte = (uint8_t)Serial1.read();
    ardudeck_receive(&byte, 1);
  }

  ardudeck_position(lat, lon, speed_ms, heading_deg, fix, sats);
  ardudeck_status(mode, active_item, volts, armed);

  ardudeck_tick(now_ms());
  delay(20);
}
```

Full file: [`arduino_serial.ino`](examples/transports/arduino_serial.ino)

---

## A whole firmware, and the seam

Every snippet above shows the SDK half against `extern` stubs. `examples/rover` is the
other thing: a firmware that actually runs, with the integration beside it, so you can see
exactly where one ends and the other begins.

```
examples/rover/
  vehicle.h        what the firmware already offers
  vehicle.c        the firmware. Drives to waypoints. Never mentions ArduDeck
  ardudeck_link.c  the integration. Talks to the firmware only through vehicle.h
  main.c           a loop and a UDP socket
```

**The firmware first.** This is the whole of what it exposes, and it is the kind of thing
any vehicle already has:

```c
rover_params_t *rover_params(void);
double        rover_lat(void);
double        rover_lon(void);
float         rover_heading_deg(void);
rover_mode_t  rover_mode(void);
bool          rover_arm(bool arm, const char **why);
bool          rover_set_mode(rover_mode_t mode, const char **why);
bool          rover_load_mission(const rover_wp_t *items, uint16_t count, const char **why);
```

**Then the integration**, which is translation and nothing else. Parameters point straight
at the firmware's own struct, so a value written from ArduDeck is the value the control
loop reads on its next pass:

```c
rover_params_t *p = rover_params();
table[0] = (ad_param_t)AD_F32("CRUISE_SPD", &p->cruise_speed_ms, 0.2f, 4.0f, "m/s",
                              "Speed held between waypoints");
```

Commands hand back the firmware's own refusal, so the operator reads the real reason
rather than a generic one:

```c
case AD_CMD_ARM:
  if (rover_arm(args[0] != 0.0f, &reason)) return true;
  break;
...
snprintf(why, why_len, "%s", reason);   /* "Battery too low to arm" */
return false;
```

And the main loop shows the split in four lines:

```c
rover_step(0.02f);        /* the firmware, doing what it always did */
ardudeck_link_tick();     /* the integration, telling ArduDeck about it */
```

`vehicle.c` did not change by one line to get a map, an instrument panel, a parameter
editor, mission upload and a command bar.

### Run it

```sh
cd examples/rover
make
./rover                      # then connect ArduDeck to UDP 14550
```

It drives to waypoints you upload, refuses to arm on a flat battery, and its steering gain
is a parameter you can turn up and watch the turns tighten. Point the conformance tool at
the same binary:

```sh
ardudeck-conform --udp 14550 --allow-mission-write
```

```
PASS  rung 0  position      heartbeat 1.0 Hz, position x40, GPS fix 3
PASS  rung 1  identity      acme rover kit fw 1.0.0, 4 modes named, 1 mission commands
WARN  rung 2  parameters    4 parameters, 4 with metadata
WARN  rung 3  missions      3 items round-tripped, capacity 16, 1 commands declared
PASS  rung 4  commands      unknown command refused in 0 ms
PASS  extra   link          10 heartbeats, 80 telemetry in 10 s with nothing requested
```

---

## Declare what the vehicle is

The manifest is what hides screens you cannot serve. Undeclared means hidden, never
broken.

```c
/* Your own mode numbers, whatever they already are. */
enum { MODE_STABILIZE, MODE_LOITER, MODE_AUTO, MODE_LAND, MODE_FAILSAFE };

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

```c
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

```c
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

```c
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

```c
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

```c
static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  if (!have_peer || sock < 0) return;
  sendto(sock, buf, len, 0, (struct sockaddr *)&peer, sizeof peer);
}
```

**STM32**, DMA into a ring, handed over in at most two pieces:

```c
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

## Run one without hardware

```sh
cd examples/transports
cc -o vehicle linux_udp.c your_vehicle.c ../../src/ad_*.c ../../src/ardudeck.c \
   -I../../include
./vehicle
```

Connect ArduDeck to UDP 14550. The same binary is what you point
[ardudeck-conform](conformance.md) at, so you can pass the conformance table before you
have soldered anything.

---

## Not here yet

Rover, fixed wing, submarine and helicopter. A rover is close enough to the boat to copy
it and change the frame; the others have shapes of their own.

---

[Getting started](getting-started.md) · [The contract](contract.md) · [API reference](api.md) · [Calibration](calibration.md) · [Links](transports.md) · [ardudeck-conform](conformance.md) · [When it is not working](troubleshooting.md)
