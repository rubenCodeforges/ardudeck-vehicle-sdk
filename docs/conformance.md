# ardudeck-conform

A small program that pretends to be a ground station and finds out whether your vehicle
does what it says it does.

"Supports MAVLink" is a claim nobody can check. This turns it into a table that is either
green or not.

```
$ ardudeck-conform --udp 14550

acme test rover  fw 1.0.0  profile 1

PASS  rung 0  position      heartbeat 1.0 Hz, 40 position, 43 attitude, fix 3
PASS  rung 1  identity      acme test rover fw 1.0.0, 4 modes named, 2 mission commands
FAIL  rung 2  parameters    5 parameters, 5 with metadata
                            -> 3 parameters have no description -> PITCH_KP, PITCH_KI
PASS  rung 3  missions      3 items round-tripped, capacity 16, 2 commands declared
PASS  rung 4  commands      unknown command refused in 12 ms
PASS  extra   calibration   3 declared

5 of 6 tested, 1 failing.
ArduDeck will enable: position, identity, missions, commands, calibration
parameters stays hidden until it passes.
```

Every finding names the thing that is wrong. There is no "conformance error 7".

---

## Getting it

```bash
git clone https://github.com/rubenCodeforges/ardudeck-vehicle-sdk
make -C ardudeck-vehicle-sdk/conformance
```

One binary, no dependencies, no runtime. It builds on macOS and Linux with any C99
compiler.

## Running it

```bash
ardudeck-conform --udp 14550                  # listen, answer whoever is heard
ardudeck-conform --udp 192.168.4.1:14550      # a vehicle at a known address
ardudeck-conform --serial /dev/ttyUSB0:57600  # over a radio or USB
ardudeck-conform --udp 14550 --json           # for CI
```

A whole run takes about twenty seconds. The exit code is 0 when every capability the
vehicle declared passed, and 1 otherwise, so it drops straight into a build.

---

## It is safe to run on a real aircraft

These rules are not settings, they are how the tool is built:

- **It never arms, never disarms and never changes flight mode.** The only command it
  sends is one that does not exist, to see whether you refuse it.
- **It never starts a calibration that declares `AD_CAL_REQ_MOTORS_LIVE`**, or one marked
  local only. It starts a harmless one and cancels it a moment later, because a cancel
  that is not honoured is a real defect.
- **It puts your mission back.** The stored plan is downloaded before anything is
  written and re-uploaded afterwards. When a vehicle cannot read its plan back, the
  mission tests are **skipped** rather than silently wiping a plan somebody drove out to
  load. `--allow-mission-write` overrides that, and says so on the line.

Run it on the bench with the propellers off the first time anyway. Not because the tool
will spin them, but because the first run is when you find out what your firmware does
with messages it has never seen.

---

## What each rung checks

### rung 0, position

Is anything there, and is it honest about what it does not know.

- A heartbeat arrives, at roughly 1 Hz
- It reports `MAV_AUTOPILOT_GENERIC`. **Claiming ArduPilot or PX4 fails**, because a
  ground station will then apply that firmware's conventions to a vehicle that does not
  share them
- Position and attitude arrive
- **No position is sent while the fix says there is none.** Absent is not zero
- **Latitude 0, longitude 0 fails.** That is a real place in the Gulf of Guinea, and a
  marker there sends somebody walking toward the sea

### rung 1, identity

Does the vehicle say what it is, and does the description hold together.

- A manifest arrives on its own, and again when asked
- Vendor and model are not empty
- Every declared mode actually arrives, **each with a name**. A mode with no name leaves
  the picker showing a number
- No two modes share an id
- The claims are consistent: declaring missions but listing no mission commands fails,
  as does declaring missions with a capacity of zero, or calibration with no
  calibrations

### rung 2, parameters

Is the parameter screen usable, or a list of numbers.

- Every parameter promised by the manifest arrives
- Every one has metadata, **a description** and a range where the minimum is below the
  maximum
- Every enum parameter sends all of its option labels
- A missing **unit** is a warning rather than a failure, because a pure gain genuinely
  has none
- One value is written back to itself and must be answered. This changes nothing and
  proves the write path replies, which is how the editor knows whether a change took

### rung 3, missions

Does planning work, and does it refuse what it should.

- A plan uploads and is acknowledged
- **One item is deliberately answered out of order.** A lost frame is ordinary, and the
  vehicle must re-request the item it is still missing rather than abandoning the
  transfer
- A plan containing a command the vehicle never declared is **refused**
- A plan one item over the declared capacity is refused **before** any items transfer,
  not after the operator has waited through the upload
- The stored plan is restored afterwards

### rung 4, commands

Does pressing a button do something, including saying no.

- A command that does not exist is **answered**, not ignored. Silence is the worst
  outcome, because the operator sees a button that does nothing and presses it again
- That answer is a refusal, not an acceptance
- The answer arrives within 500 ms. Slower reads as ignored

### extra, calibration

- Every declared calibration arrives, with a title
- A positional one declares poses, a coverage one declares tracks, and all of them
  actually arrive
- No more than three tracks
- A safe calibration is started and immediately cancelled. **The cancel must be
  honoured**

---

## In CI

```yaml
- run: make -C conformance
- run: ./my_firmware_host_build --port 14550 &
- run: ./conformance/ardudeck-conform --udp 14550 --json > conform.json
```

You do not need hardware. Build your integration for the host, point the tool at it, and
the run fails the build the day somebody removes a parameter description.

The SDK does this to itself. `test/conform.sh` starts a vehicle built on this SDK on a
real UDP socket, runs the tool against it, and then **breaks the vehicle on purpose** and
insists the matching check goes red:

```
ok   a correct vehicle passes missions (missions is PASS)
ok   a mode with no name fails (identity is FAIL)
ok   parameters with no help fail (parameters is FAIL)
ok   a position at 0,0 fails (position is FAIL)
ok   accepting any command fails (commands is FAIL)
ok   declaring missions with no commands fails (missions is FAIL)
```

A check nobody has watched fail is not a check.

---

## Some checks cannot fail if you use the SDK

The mission command filter, the range clamp on a parameter write, the acknowledgement of
an unknown command: the SDK does all of these for you, so a vehicle built on it passes
them without the vendor thinking about it.

They are still in the tool, because the profile is a contract and not a library. Firmware
that implements it directly has to get them right too.

---

## Reading a result

**FAIL** means a declared capability does not work. ArduDeck hides that screen.

**WARN** means something is legal but probably not what you meant. A parameter with no
unit is right for a gain and wrong for a speed, and only you know which.

**SKIP** means the capability was never declared, so there was nothing to test. A skip is
not a failure: a vehicle that does not do missions is not broken, it just does not do
missions.

---

[Getting started](getting-started.md) · [Contract](contract.md) · [API](api.md) · [Calibration](calibration.md) · [Links](transports.md) · **Conformance** · [Troubleshooting](troubleshooting.md)
