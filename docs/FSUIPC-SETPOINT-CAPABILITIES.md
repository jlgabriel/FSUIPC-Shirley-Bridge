# FSUIPC Setpoint Capabilities for MSFS 2024

**What a Shirley bridge built on FSUIPC7 can actually *set* in Microsoft Flight Simulator 2024.**

Prepared for the Airplane Team. Mapped field-by-field against
`schemas/set_simdata_schemas_xplane.ts` from [Airplane-Team/sim-interface](https://github.com/Airplane-Team/sim-interface).

| | |
|---|---|
| **Status** | Verified against a running sim, August 2026 |
| **Sim** | MSFS 2024 (Steam). C172SP G1000, King Air 350i and Citation CJ4, parked; CJ4 also airborne for the landing-gear test |
| **Stack** | FSUIPC7 v7.5.7 · WebSocket Server v1.1.4 · WASM variable service active |
| **Method** | Every claim exercised by [`tools/verify_msfs2024.py`](../tools/verify_msfs2024.py) |

> **Terminology.** In the simulation community *MSFS* on its own means MSFS 2020. Everything
> here was tested exclusively on **MSFS 2024** and is written as such; nothing in this document
> is a claim about MSFS 2020. Where a statement is common to both titles — the calculator-code
> language, the `K:` event family — it is called out explicitly.

> **Revision note.** The first version of this document was derived from FSUIPC's offset-status
> PDF alone and got several things wrong — it labelled flaps, the autopilot bugs and the radios
> "read-only", which they are not. Live testing corrected those and, more importantly, replaced
> the reasoning behind the central recommendation. The recommendation itself survived.
>
> **Second revision.** Re-tested on a King Air 350i and a Citation CJ4 to reach the systems a
> C172 does not have. Spoilers, landing gear and prop de-ice — all three previously recorded as
> failures — turned out to write correctly by offset; they had been tested on an aircraft that
> could not respond. That prompted a correction to the verification method itself, described
> below, and surfaced a second silent-failure mode that matters more than the first.

## Summary

**Yes — FSUIPC can cover essentially all of `SetSimData` except weather.** Excluding the weather
group, 36 of the remaining 42 schema fields are reachable.

**Prefer control events; use an offset write only where it is verified.** This is the practical
conclusion, and the reason matters more than the rule — see below. Seven offsets are now proven
to write through, and for prop de-ice the offset is the only path there is, so the rule is a
default rather than a prohibition.

| Not reachable | Why |
|---|---|
| The whole `environment` weather group | Verified: writes are accepted and echoed back, but never applied. Only `zuluTimeHours` and `dayOfYear` are settable. |
| `failures.*` | Engine-failure offsets are read-only; no failure-set events exist. |
| `simulation.isCrashed`, `shouldResetFlight` | No corresponding control event. |
| `levers.propBetaEnabled` | `PROP BETA:n` is read-only; no beta-set event. |

## The findings that should drive the design

### 1. FSUIPC echoes back writes that never reach the simulator

Write to an offset, read it back, and you get exactly the value you wrote — whether or not
SimConnect applied it. FSUIPC maintains its own offset buffer and serves reads from it. There is
no error, no status flag, and no way to tell success from failure by reading the offset you just
wrote.

This is detectable only by writing an offset and watching a *mirror* — a second offset backed by
the same SimVar, or the physical surface-position indicator, which only the sim moves:

| Write | Offset reads back | Mirror | Applied? |
|---|---|---|---|
| Flaps `0x0BDC` = 10922 (C172) | 10922 | `0x0BE0` animated 0 → 1619 → 10922 | **yes** |
| Flaps index `0x0BFC` = 2 (King Air) | 2 | `0x0BE0` moved 8192 → 16383 | **yes** |
| COM1 standby `0x311A` | `0x2185` | `0x05CC` moved 124.85 → 121.85 MHz | **yes** |
| Spoilers `0x0BD0` = 16383 (CJ4) | 16384 | `0x0BD4` animated 0 → 14746 → 16384 | **yes** |
| Gear `0x0BE8` = 0 (CJ4, airborne) | 0 | `0x0BEC`/`0x0BF0`/`0x0BF4` ran full travel to 0 | **yes** |
| Prop de-ice `0x2440` = 1 (King Air) | 1 | `0x337C` moved 0 → 3 | **yes** |
| OAT `0x0E8C` = 20 °C | 20 °C | `0x34A8` stayed 21.99 °C | **no** |
| Wind `0x0E90` = 15 kt | 15 | `0x3488` stayed 0.0 | **no** |

Of the offsets exercised, **7** are now proven to reach the sim, **2** are proven not to, **1**
(`0x0EC6`) is rejected outright, **1** (`0x07D4`) is applied but misinterprets the value, and
**13** have no mirror available and cannot be verified either way.

### 2. Writing a read-only offset silently corrupts every later read of it

This is the more dangerous of the two, and it was not visible until a write was aimed at an
offset the sim ignores. The poisoned value **survives the connection**: it is still served after
disconnecting, reconnecting and re-declaring the group.

```
after writing 0x0E8C = 20 °C and 0x0E90 = 15 kt, on a brand-new connection:
  oat        5120  ->  20.0 °C          m_oatDbl   21.99 °C   <- the sim's real value
  windSpeed    15  ->  15 kt            m_windDbl   0.0       <- the sim's real value
```

FSUIPC keeps what was written in its buffer and serves it as if it were telemetry. Nothing clears
it. For a bridge this inverts the cost of a mistake: an echo merely misleads you about a write,
but this corrupts a *reading* that Shirley then consumes as flight data. One stray write to
`0x0E8C` leaves the bridge reporting an invented outside air temperature for the rest of the
session.

The design consequence is concrete: **a bridge must never probe an offset to find out whether it
is writable.** The set of writable offsets has to be known in advance and the rest never touched.

### 3. A still mirror has two causes, not one

The mirror test says "the write did not take effect". It does not say *why*, and there are two
reasons: FSUIPC echoed it, or the sim received the command and refused it for a legitimate
physical reason.

The first round of testing read every still mirror as an echo, and was wrong three times. On the
ground, MSFS 2024 will not retract the landing gear with weight on the wheels — so `0x0BE8`
looked exactly like an echo. Flown at 1900 ft, the same write ran the gear through its full
travel in about nine seconds. Spoilers and prop de-ice were the same story on an airframe that
had neither.

So a verdict of "echo" is only meaningful once the aircraft, and its current state, could have
honoured the command. Testing a setpoint matrix on a single simple aircraft systematically
understates what works.

### What this means

The rule is not "the offset is read-only". It is: **an offset write fails silently and
indistinguishably from success, a bad one poisons subsequent reads, and a control event does
neither.** Prefer events. Use a direct offset write only where the table below marks it verified,
and never anywhere else.

## How writing works

**Control event** — the recommended path. Three interchangeable transports, all over the existing
connection:

```json
{"command":"vars.calc","name":"setFlaps","code":"10922 (>K:FLAPS_SET)"}
```

- `vars.calc` — runs MSFS calculator code (RPN). Most legible; needs the WASM module.
- offset `0x3110` — 32-bit control number + 32-bit parameter. Works in *unregistered* FSUIPC with
  no WASM module: the most portable option.
- offset `0x7C50` — send by name (`P:` preset, `L:` lvar, `H:` hvar, `I:` input event), with the
  parameter in `0x7C90`.

**Direct offset write** — for the few cases where it is verified, or where no event exists.

```json
{"command":"offsets.write","name":"flightData",
 "offsets":[{"name":"gearHandle","value":16383}]}
```

The offset must belong to a previously declared group and is referenced **by name, not address**.

## The matrix

**Offset column** — ✅ verified applied (mirror moved) · ◐ accepted, no mirror available, cannot be
verified · ✗ echo: accepted but proven not applied · ✕ rejected outright · — not applicable

**Event column** — ⚡ verified working · — none needed or none exists

### levers

| Field | Offset | Event | Notes |
|---|---|---|---|
| `flapsHandlePercentDown` | ✅ `0x0BDC` | ⚡ `FLAPS_SET` | `0…16383`, snapped to the aircraft's detents. On coarse-detent aircraft prefer the index — see below. |
| — *(detent index)* | ✅ `0x0BFC` | — | Addresses detents directly, one byte, `0`-based. The reliable path on aircraft with few positions. |
| `speedBrakesHandlePercentDeployed` | ✅ `0x0BD0` | ⚡ `SPOILERS_SET` | Verified on a CJ4, both ways. Full deflection normalises to **16384**, not 16383. |
| `landingGearHandlePercentDown` | ✅ `0x0BE8` | ⚡ `GEAR_SET` | Verified airborne on a CJ4, both ways. On the ground the sim refuses to retract and both paths look dead. |
| `carburetorHeatLeverPercentHot` | ◐ `0x08B2` | ⚡ `ANTI_ICE_SET_ENG1` | Binary `0`/`1`, not a percentage — MSFS 2024 models carb heat as the engine anti-ice switch. |
| `propBetaEnabled` | ✕ | — | `PROP BETA:n` read-only, no beta-set event. |

### autopilot

Every field here has a working event. Offset writes are accepted but unverifiable.

| Field | Offset | Event |
|---|---|---|
| `isAutopilotEngaged` | ◐ `0x07BC` | ⚡ `AUTOPILOT_ON` / `AUTOPILOT_OFF` (`AP_MASTER` toggles) |
| `isHeadingSelectEnabled` | ◐ `0x07C8` | ⚡ `AP_PANEL_HEADING_HOLD` |
| `magneticHeadingBugDeg` | ◐ `0x07CC` | ⚡ `HEADING_BUG_SET` (degrees) |
| `altitudeBugFt` | ✗ `0x07D4` | ⚡ `AP_ALT_VAR_SET_ENGLISH` (feet) |
| `altitudeMode` = `altitudeHold` | ◐ `0x07D0` | ⚡ `AP_PANEL_ALTITUDE_HOLD` |
| `altitudeMode` = `verticalSpeed` | ◐ `0x07EC` | ⚡ `AP_PANEL_VS_HOLD` |
| `targetVerticalSpeedUpFpm` | ◐ `0x07F2` | ⚡ `AP_VS_VAR_SET_ENGLISH` (fpm) |
| `isFlightDirectorEngaged` | — | ⚡ `TOGGLE_FLIGHT_DIRECTOR` — toggle only |
| `shouldLevelWings` | — | ⚡ `AP_WING_LEVELER` |

**`0x07D4` deserves a warning.** It does not accept a value: each write *increments* the altitude
bug by exactly 1000 ft, whatever you write. Three consecutive writes of the same value produced
+1000, +1000, +1000. Use the event.

`altitudeMode`'s other values — `pitch`, `terrain`, `VNAV`, `TOGA`, `flightPathAngle`,
`VNAVSpeed` — have no generic MSFS 2024 event. `levelChange` maps to `FLIGHT_LEVEL_CHANGE` and
`glideSlope` to `AP_APR_HOLD`; neither was exercised on this aircraft.

### radiosNavigation

Frequency parameters are 4-digit **BCD with the leading 1 assumed** — 123.45 MHz is `0x2345`.

| Field | Offset | Event |
|---|---|---|
| `standbyFrequencyHz.com1` | ✅ `0x311A` | ⚡ `COM_STBY_RADIO_SET` |
| `standbyFrequencyHz.com2` | ✅ `0x311C` | ⚡ `COM2_STBY_RADIO_SET` |
| `standbyFrequencyHz.nav1` | ◐ `0x311E` | ⚡ `NAV1_STBY_SET` |
| `transponderCode` | ◐ `0x0354` | ⚡ `XPNDR_SET` — BCD, squawk 1200 is `0x1200` |
| `comShouldSwapFrequencies` | — | `COM_STBY_RADIO_SWAP`, `COM2_STBY_RADIO_SWAP` — not exercised |

### lights

`0x0D0C` is a bitmask; all four were driven by event and verified.

| Field | Event |
|---|---|
| `navigationLightsSwitchOn` | ⚡ `NAV_LIGHTS_SET` |
| `landingLightsSwitchOn` | ⚡ `LANDING_LIGHTS_SET` |
| `taxiLightsSwitchOn` | ⚡ `TAXI_LIGHTS_SET` |
| `strobeLightsSwitchOn` | ⚡ `STROBES_SET` |

### systems · indicators

| Field | Offset | Event | Notes |
|---|---|---|---|
| `parkingBrakeOn` | ◐ `0x0BC8` | ⚡ `PARKING_BRAKE_SET` | Offset is `0` / `32767` |
| `pitotHeatSwitchOn` | ◐ `0x029C` | ⚡ `PITOT_HEAT_SET` | |
| `batteryOn.main` | ◐ `0x281C` | ⚡ `TOGGLE_MASTER_BATTERY` | Event toggles |
| `altimeterSettingInchesMercury` | ◐ `0x0330` | ⚡ `KOHLSMAN_SET` | millibars × 16 |
| `propHeatSwitchOn` | ✅ `0x2440` | — | Verified on a King Air 350i: mirror `0x337C` moved 0 → 3. `TOGGLE_STRUCTURAL_DEICE` is *not* the event for this — it drives airframe de-ice, and did nothing here. No prop-specific set event ships in the preset catalogue, so the offset is the only path. |
| `governorSwitchOn` | — | `HELICOPTER_ENGINE_n_GOVERNOR_SWITCH_SET` | Helicopters only, not exercised |
| `totalEnergyAudioSwitchOn` | — | `TOGGLE_VARIOMETER_SWITCH` | Toggle only, not exercised |

### environment

**Weather cannot be set — verified, not inferred.** Writes to ambient temperature and wind are
accepted and echoed back while the sim's own value never moves; sea-level pressure is rejected
outright. That rules out all 20 cloud-layer, wind-layer, visibility, pressure, temperature,
precipitation, runway-friction, thermal and weather-evolution fields.

These are also the offsets that demonstrated read poisoning. A bridge that reads
`groundTemperatureDegC` from `0x0E8C` must never write to it — one attempt and that field reports
a fabricated value for the rest of the session.

| Field | Event |
|---|---|
| `zuluTimeHours` | `ZULU_HOURS_SET`, `ZULU_MINUTES_SET` |
| `dayOfYear` | `ZULU_DAY_SET` |

### position · attitude · simulation · freezes

| Field | Mechanism | Notes |
|---|---|---|
| `latitudeDeg` / `longitudeDeg` / `mslAltitudeFt` | offset `0x0560` / `0x0568` / `0x0570` | Teleports; needs slew (`0x05DC`, writable) or a freeze. Not exercised. |
| `indicatedAirspeedKts` | offset `0x02BC` | knots × 128. Not exercised. |
| `pitchAngleDegUp` / `trueHeadingDeg` | offset `0x0578` / `0x0580` | Sign inverts on pitch: FSUIPC is negative-for-nose-up. Not exercised. |
| `positionFreezeEnabled` | ⚡ `FREEZE_LATITUDE_LONGITUDE_TOGGLE` | Verified. Toggle only — read `0x3540` first to stay idempotent. |
| `isPaused` | `PAUSE_ON` / `PAUSE_OFF` | Not exercised. |
| `simSpeedRatio` | `SIM_RATE_INCR` / `SIM_RATE_DECR` | Stepwise only; `0x0C1A` is read-only. |
| `isCrashed`, `shouldResetFlight`, `failures.*` | — | No mechanism. |

## Protocol notes

Three behaviours of the WebSocket server that cost time to discover:

**`offsets.read` responds on change, not on request.** If nothing in the group changed since the
last response, the server sends nothing at all — not an empty payload, no response. A one-shot
read after a quiet moment hangs forever. Subscribe once with `interval` and merge the partial
payloads into a running state.

**A malformed `offsets.write` is discarded silently.** Sending the undocumented
`{"values":[{"address":…}]}` shape produces no error response whatsoever.

**A successful `offsets.write` replies with an `offsets.read` response.** You only see an
`offsets.write` response when it failed.

**The server drops client connections intermittently.** Observed five times across two sessions,
always the same way: a `vars.calc` gets no reply, and the socket turns out to have been closed
with no close frame. FSUIPC7, the WebSocket server and the sim all stay alive and responsive, and
the identical call succeeds immediately on a fresh connection — so it is not the command, the
aircraft or load. A bridge that holds one long-lived connection will simply go quiet the first
time this happens. **Automatic reconnection, with re-declaration of the offset groups and
re-subscription, is a requirement, not a refinement.**

## Input Events (`B:` vars, MSFS 2024 only)

These are the modern per-aircraft cockpit controls, and they are the natural home for the
`altitudeMode` values with no generic event. Two practical limits:

- **They are not discoverable over the WebSocket.** `vars.list` returned 314 lvars and 0 hvars on
  the C172, with no `I:`/`B:` entries at all. You must already know the name.
- Reaching them means offset `0x7C50` with an `I:` prefix, or mapping them to offsets via an
  `[InputEventOffsets]` section in `FSUIPC7.ini` — which requires each user to hand-edit a config
  file, so it is poorly suited to a distributable bridge.

## Notes for the Shirley side

**The endpoint is already compatible.** The airplane.team SimConnect bridge listens on
`ws://localhost:2992/api/v1`, which is exactly what this FSUIPC bridge serves. An FSUIPC-backed
bridge is a drop-in substitute: stop one, start the other, and `?msfs2024` connects unchanged.

**Toggle-only fields need a read first.** `isFlightDirectorEngaged`, `totalEnergyAudioSwitchOn`
and `positionFreezeEnabled` have no absolute setter in MSFS 2024. A bridge can make them idempotent by
reading current state before toggling, though that is a race in principle — `Writability.AfterRead`
fits them naturally.

**Percent versus detent.** `flapsHandlePercentDown` is continuous `0…16383`, which MSFS 2024 snaps to
the aircraft's detents; offset `0x3BFA` gives the increment per detent — 5461 on the C172 (four
positions), 8191 on both the King Air 350i and the CJ4 (three). Reading back gives the snapped
value, so a naive write-then-verify reports a mismatch.

On a three-detent aircraft the snapping is coarse enough to swallow a command whole: asking for
10922 lands on 8191, which on those two airframes was where the flaps already were, so nothing
moved and nothing indicated why. A bridge should convert the requested percentage to a detent
index and write `0x0BFC`, rather than pass the percentage through and hope.

**Carb heat is binary** in MSFS 2024, though the schema types it as a percentage.

## What has not been exercised

Everything above marked "not exercised", plus `governorSwitchOn`, which needs a helicopter. The
position and attitude writes are untested because they teleport the aircraft.

Spoilers, landing gear and prop de-ice are now closed, on a CJ4 and a King Air 350i.

Re-run [`tools/verify_msfs2024.py --write`](../tools/verify_msfs2024.py) on a different airframe
to close what remains; the script prints the same matrix. Two cautions learned the hard way:
a field can only be judged on an aircraft that has the system and is in a state where the command
is legal, and the run should be treated as leaving the aircraft dirty — a write to a read-only
offset poisons that offset's reads until FSUIPC restarts.
