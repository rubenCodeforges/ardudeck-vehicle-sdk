# Status

This SDK is **experimental**, and the honest reason is worth stating plainly.

Its twelve messages occupy ids **43000 to 43011**. MAVLink message ids are one global
namespace, and nothing has reserved that range for us. We picked it because it looked
empty, which is not the same as owning it. That matters: an earlier draft of this profile
sat at 42000, which turned out to belong to ICAROUS, and the symptom was frames being
silently dropped with nothing in any log to say why.

**Reserving the block means a pull request to the [MAVLink project](https://github.com/mavlink/mavlink),
and we are not sending it yet.** A reservation is a promise that the layouts are stable.
Proposing one for a contract still being shaped by the first firmwares built on it would
be asking other people to carry our mistakes. We would rather find those mistakes now,
while moving a field costs us a rebuild instead of costing you a fleet.

So the order is: build vehicles on it, let the rough edges show, and propose the block once
the shape has stopped changing.

### What that means for you

| | |
|---|---|
| **Building, experimenting, teaching** | Go ahead. That is what it is for, and it is how the ids get earned |
| **Shipping a product** | Not yet. If the block moves, deployed vehicles stop being seen, and they stop quietly |
| **Following along** | The ids and layouts live in [`profile/ardudeck.xml`](../profile/ardudeck.xml), and any change to them will be called out, not slipped in |

None of this touches rung 0. Position, attitude and battery are ordinary MAVLink common
messages with ids that have been settled for years, so a vehicle that only does telemetry
is on solid ground today. It is the profile on top, the part that describes what your
vehicle *is*, that is still provisional.

---

## Which ArduDeck you need

**0.1.2 or newer.**

Rung 0 works with anything, because position, attitude and battery are ordinary MAVLink
common messages that every ground station has read for years. Everything above it, the
capability manifest, the parameter metadata, the mission command list and the calibration
declarations, is carried in the profile, and reading the profile arrived in 0.1.2.

The failure on an older build is quiet in the way this project keeps warning about: the
vehicle appears on the map, the instruments move, and the parameter and calibration screens
simply are not there. Nothing is logged, because from ArduDeck's side nothing went wrong.
It is the first thing to check when a firmware that passes the conformance table still
looks half connected.

---

## When the shape stops moving

Two ESP32 firmwares now run on this contract, a rover and a quad, neither written by us.
Every one of them has found something: a missing airspeed input, a link failsafe fed by the
wrong thing, a calibration check that verified nothing. That is the process working, and it
is also why the block is not reserved yet.

When new firmware stops finding holes in the contract, we propose the block. If you are
building on it, you are part of how that happens, and the conformance tool is how we find
out together rather than after the fact.

---

[Getting started](getting-started.md) · [Examples](examples.md) · [The contract](contract.md) · [API reference](api.md) · [Calibration](calibration.md) · [Links](transports.md) · [ardudeck-conform](conformance.md) · [When it is not working](troubleshooting.md)
