# Getting started

By the end of this page your vehicle is on ArduDeck's map with a live instrument panel.
Budget about twenty minutes. You do not need to read anything else first.

---

## What you are building

```mermaid
flowchart TB
    STATE["<b>State you already have</b><br/>lat, lon, attitude, battery"]
    OUT["builds frames"]
    W(["<b>Your link</b><br/>USB · UDP · radio"])
    GCS["<b>ArduDeck</b><br/>map, instruments,<br/>parameters, missions"]
    IN["parses frames"]
    CB["<b>Callbacks you write</b><br/>on_param_set · on_mission<br/>on_command · on_calibrate"]

    STATE -->|"ardudeck_position()<br/>ardudeck_attitude()"| OUT
    OUT -->|"send()"| W
    W --> GCS
    GCS -->|"the operator<br/>pressed something"| W
    W -->|"ardudeck_receive()"| IN
    IN -->|"calls you back"| CB

    classDef yours fill:#0b6e78,stroke:#2dd4bf,color:#ffffff
    classDef sdk fill:#1f2937,stroke:#6b7280,color:#e5e7eb
    classDef gcs fill:#1e3a5f,stroke:#60a5fa,color:#dbeafe
    class STATE,CB yours
    class OUT,IN,W sdk
    class GCS gcs
    linkStyle 0,1,2 stroke:#2dd4bf
```

Teal is your code, grey is the SDK, blue is ArduDeck.

**You give the SDK numbers. It gives you bytes.** Where those bytes go is entirely your
choice, and the SDK never finds out. It never touches an actuator: everything inbound
arrives as a callback your own code can refuse.

---

## Step 1: add the library

Pick whichever matches your project.

Get it first:

```sh
git clone https://github.com/rubenCodeforges/ardudeck-vehicle-sdk
```

| Your build | What to do |
|---|---|
| **ESP-IDF** | Copy the repo into `components/ardudeck/`, or add it to `idf_component.yml`, then add `ardudeck` to `REQUIRES` in your own component's `CMakeLists.txt`. |
| **PlatformIO** | `lib_deps = https://github.com/rubenCodeforges/ardudeck-vehicle-sdk` |
| **Arduino** | Drop the folder into `libraries/`. |
| **CMake** | `add_subdirectory(ardudeck-vehicle-sdk)` then link `ardudeck::sdk`. |
| **Makefile, anything else** | Compile `src/*.c` and put `include/` on your include path. That is the whole integration. |

### Working examples

Complete files, all of which compile in CI, so nothing here is pseudocode.

| Example | What it shows |
|---|---|
| [`examples/boat`](examples/boat/ardudeck_link.c) | A real surface vehicle: rungs 0 to 4, a coverage compass routine, and a second control surface beside the phone app it already had |
| [`examples/quad`](examples/quad/ardudeck_link.c) | A multirotor: attitude and altitude, takeoff, a six point accelerometer routine, and a dropdown parameter |

The link layer is the only part that is board specific, and it is small:

| Board | File |
|---|---|
| ESP-IDF | [`transports/esp32_udp.c`](examples/transports/esp32_udp.c), UDP on the board's own AP |
| Arduino | [`transports/arduino_serial.ino`](examples/transports/arduino_serial.ino), the shortest thing that appears on a map |
| STM32 HAL | [`transports/stm32_uart.c`](examples/transports/stm32_uart.c), UART with DMA receive |
| Zephyr | [`transports/zephyr_uart.c`](examples/transports/zephyr_uart.c), interrupt driven UART |
| Linux | [`transports/linux_udp.c`](examples/transports/linux_udp.c), a companion computer, or your desktop against SITL |

Start with the Linux one if you have no hardware yet: it builds and runs where you are
reading this, so you can point ArduDeck at your logic before a board exists.

---

On ESP-IDF the `REQUIRES` line is the step people miss. Without it the component builds
but your own file cannot see `ardudeck.h`:

```cmake
idf_component_register(
    SRCS "main.c" "mavlink_out.c"
    INCLUDE_DIRS "."
    REQUIRES ardudeck          # <- without this, ardudeck.h is not on the include path
)
```

It is C99 and needs no configuration to build. If it does not compile, that is a bug in
the SDK and worth reporting, not something for you to work around.

---

## Step 2: the smallest thing that works

Create one file. This is the complete text, not an excerpt.

```c
#include "ardudeck.h"

/* 1. What this vehicle is. */
static const ad_capability_t CAPS = {
  .vendor   = "acme",
  .model    = "quad v2",
  .firmware = "1.4.0",
  .frame    = AD_FRAME_MULTIROTOR,
  .features = 0,                  /* telemetry only for now */
};

/* 2. Where the bytes go. Replace link_write with whatever you have. */
static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  link_write(buf, len);
}

/* 3. Start it once. */
void my_setup(void) {
  static const ad_config_t cfg = {
    .caps   = &CAPS,
    .send   = sink,
    .now_ms = millis,             /* your millisecond clock */
  };
  ardudeck_begin(&cfg);
}

/* 4. Feed it, from a loop you already have. Anything above 20 Hz is plenty. */
void my_loop(void) {
  ardudeck_position(lat, lon, speed_ms, heading_deg, fix, sats);
  ardudeck_attitude(roll_rad, pitch_rad, yaw_rad);
  ardudeck_status(mode_id, 0, battery_volts, armed);
  ardudeck_tick(millis());
}
```

And wherever bytes arrive from the ground station:

```c
ardudeck_receive(buf, len);
```

That is it. Four calls and a struct.

> ### If your vehicle has a link failsafe, read this
>
> The SDK transmits whether or not anybody is listening, so its own sending tells you
> nothing. Ask it how long the ground station has been quiet:
>
> ```c
> if (ardudeck_silent_for(millis()) > 3000) enter_failsafe();
> ```
>
> Returns `AD_NEVER_HEARD` until something speaks. **Do not build a failsafe on
> `ardudeck_linked()`**, which is only a convenience over the same number.
>
> This matters most on a vehicle that already has a failsafe for a different link. A boat
> flown from a phone, then given a mission from a laptop, will otherwise trigger its
> phone-link failsafe mid-mission.

> **Your clock may wrap and that is fine.** `now_ms` is expected to be a plain
> millisecond counter. It rolls over every 49 days and the SDK handles it. Do not reset
> it, and do not try to be clever.

---

## Step 3: get the bytes onto a wire

### Over USB, which is the easiest way to start

Write the SDK's bytes to your serial port, and feed anything that arrives back in.

```c
static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  uart_write_bytes(UART_NUM_0, buf, len);
}
```

> ### ⚠ The one that catches everyone
>
> **On most boards, UART0 is also where your logs go.** Your MAVLink frames and your
> `printf` output end up interleaved on the same wire. ArduDeck will recover, because the
> parser resynchronises, but you will lose frames and spend an hour blaming the SDK.
>
> Either turn the console off on that port, or put MAVLink on a different UART. On
> ESP-IDF that is `Component config → ESP System Settings → Channel for console output`.

### Over WiFi, once USB works

Broadcast UDP to port 14550. Nothing needs configuring at either end, because ArduDeck
listens and answers whoever it hears.

See [Links](transports.md) for radio modems, rate limits and the rest.

---

## Step 4: look at it

If you do not have ArduDeck yet, it is a free download for Windows, macOS and Linux:
**[ardudeck.com](https://ardudeck.com/#download)**. The SDK is the library your firmware
compiles against; ArduDeck is the application on your laptop. You need both.

**Use 0.1.2 or newer.** Reading the profile, which is what turns a declared parameter table
into an editor and a declared calibration into a screen, arrived in 0.1.2. On an earlier
build your vehicle still appears on the map and the instruments still move, because that is
ordinary MAVLink, but everything above rung 0 stays hidden with nothing to say why.

Open it, connect to your serial port or to UDP 14550, and you should see your vehicle on
the map with a moving instrument panel.

**If nothing appears**, do not guess. Run the conformance tool, which will tell you what
is wrong in one line:

```
make -C conformance
./conformance/ardudeck-conform --serial /dev/ttyUSB0:57600
```

See [When it is not working](troubleshooting.md) if you would rather read symptoms.

---

## Step 5: climb, or stop

You are now on rung 0. Everything above it is optional, independent, and can be done in
any order or not at all.

```mermaid
flowchart LR
    R0["<b>0 · Position</b><br/><i>✓ you are here</i><br/>map, instruments, HUD"]
    R1["<b>1 · Identity</b><br/><i>one struct</i><br/>modes by name"]

    R2["<b>2 · Parameters</b><br/><i>one table</i><br/>the editor"]
    R3["<b>3 · Missions</b><br/><i>one callback</i><br/>planning and survey"]
    R4["<b>4 · Commands</b><br/><i>one callback</i><br/>arm, mode, start"]
    CAL["<b>✦ Calibration</b><br/><i>a description</i><br/>your routine, our screen"]

    R0 ==> R1
    R1 --> R2
    R1 --> R3
    R1 --> R4
    R1 --> CAL

    classDef yours fill:#0b6e78,stroke:#2dd4bf,color:#ffffff
    classDef sdk fill:#1f2937,stroke:#6b7280,color:#e5e7eb
    classDef gcs fill:#1e3a5f,stroke:#60a5fa,color:#dbeafe
    classDef warn fill:#78350f,stroke:#f59e0b,color:#fde68a
    class R0 yours
    class R1 gcs
    class R2,R3,R4,CAL sdk
```

Everything after rung 1 is **independent**. Take parameters without missions, or commands
without either, in whatever order suits you.

**Rung 1 is the one that matters**, and it is the shortest. Until you declare what your
vehicle is, ArduDeck assumes ArduPilot: your mode picker shows modes you do not have, and
pressing one sends a number that means something else on your firmware.

### Rung 1, in full

```c
static const ad_mode_t MODES[] = {
  /*  id, name shown to the pilot, flags */
  {  0, "Idle",      0 },
  {  1, "Running",   AD_MODE_MISSION },      /* the one that flies a plan */
  {  4, "Returning", 0 },
  {  6, "Manual",    AD_MODE_LOCAL_ONLY },   /* exists, but not ours to command */
};

static const uint16_t MISSION_CMDS[] = { AD_NAV_WAYPOINT, AD_NAV_RETURN_TO_LAUNCH };

static const ad_capability_t CAPS = {
  .vendor = "acme", .model = "quad v2", .firmware = "1.4.0",
  .frame  = AD_FRAME_MULTIROTOR,
  .features = AD_FEAT_PARAMS | AD_FEAT_MISSION | AD_FEAT_COMMANDS,

  /* AD_COUNT keeps these from drifting when you add a mode later. */
  .modes = MODES, .mode_count = AD_COUNT(MODES),
  .mission_cmds = MISSION_CMDS, .mission_cmd_count = AD_COUNT(MISSION_CMDS),
  .mission_capacity = 64,
};
```

> ### Expect to add a few accessors to your own code
>
> The SDK reads your state through pointers and calls into your firmware through
> callbacks, so anything it needs has to be reachable from outside the file that owns it.
>
> Most firmware is not written that way, and this is normal work rather than a problem:
>
> - Parameters kept `static` in a config module need something returning a writable
>   pointer, or a getter and setter pair per value.
> - Declaring `AD_FEAT_MISSION_READ` needs a way to read a stored plan back out, which
>   plenty of mission stores have never needed.
>
> Neither is hard, but both touch files that had no other reason to change. Budget for it.

> ### The rule the whole thing runs on
>
> **A capability you do not declare is a screen ArduDeck hides.**
>
> Declare less than you support and you lose a screen until you add it. Declare more and
> your operator gets a mission planner that uploads plans your vehicle refuses, and a
> button that does nothing.
>
> **Under-declare, then grow.** It is always the safer direction.

---

## Where to go next

| You want to | Read |
|---|---|
| Add parameters, missions or commands | [The contract](contract.md) |
| Look up a function or struct field | [API reference](api.md) |
| Put your compass routine on ArduDeck's screen | [Calibration](calibration.md) |
| Find out why something is not working | [When it is not working](troubleshooting.md) |
| Choose or configure a link | [Links](transports.md) |
| Prove it before you ship | [ardudeck-conform](conformance.md) |

---

**Getting started** · [Contract](contract.md) · [API](api.md) · [Calibration](calibration.md) · [Links](transports.md) · [Conformance](conformance.md) · [Troubleshooting](troubleshooting.md)
