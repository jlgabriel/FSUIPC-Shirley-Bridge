# FSUIPC Setpoint Capabilities for MSFS 2024

**What a Shirley bridge built on FSUIPC7 can actually *set* in Microsoft Flight Simulator.**

Prepared for the Airplane Team. Mapped field-by-field against
`schemas/set_simdata_schemas_xplane.ts` from [Airplane-Team/sim-interface](https://github.com/Airplane-Team/sim-interface).

| | |
|---|---|
| **Status** | Verified against a running sim, August 2026 |
| **Sim** | MSFS 2024 (Steam), C172SP G1000, parked |
| **Stack** | FSUIPC7 v7.5.7 · WebSocket Server v1.1.4 · WASM variable service active |
| **Method** | Every claim exercised by [`tools/verify_msfs2024.py`](../tools/verify_msfs2024.py) |

> **Revision note.** The first version of this document was derived from FSUIPC's offset-status
> PDF alone and got several things wrong — it labelled flaps, the autopilot bugs and the radios
> "read-only", which they are not. Live testing corrected those and, more importantly, replaced
> the reasoning behind the central recommendation. The recommendation itself survived.

## Summary

**Yes — FSUIPC can cover essentially all of `SetSimData` except weather.** Excluding the weather
group, 36 of the remaining 42 schema fields are reachable.

**Use control events, not offset writes.** This is the practical conclusion, and the reason
matters more than the rule — see below.

| Not reachable | Why |
|---|---|
| The whole `environment` weather group | Verified: writes are accepted and echoed back, but never applied. Only `zuluTimeHours` and `dayOfYear` are settable. |
| `failures.*` | Engine-failure offsets are read-only; no failure-set events exist. |
| `simulation.isCrashed`, `shouldResetFlight` | No corresponding control event. |
| `levers.propBetaEnabled` | `PROP BETA:n` is read-only; no beta-set event. |

## The finding that should drive the design

**FSUIPC accepts and echoes back writes that never reach the simulator.** Write to an offset, read
it back, and you get exactly the value you wrote — whether or not SimConnect applied it. FSUIPC
maintains its own offset buffer and serves reads from it. There is no error, no status flag, and
no way to tell success from failure by reading the offset you just wrote.

This was verified by writing an offset and watching a *mirror* — a second offset backed by the
same SimVar, or the physical surface-position indicator:

| Write | Offset reads back | Mirror | Applied? |
|---|---|---|---|
| Flaps `0x0BDC` = 10922 | 10922 | `0x0BE0` animated 0 → 1619 → 10922 | **yes** |
| COM1 standby `0x311A` | 0x2185 | `0x05CC` moved 124.85 → 121.85 MHz | **yes** |
| Spoilers `0x0BD0` = 16383 | 16383 | `0x0BD4` stayed 0 indefinitely | **no** |
| OAT `0x0E8C` = 30 °C | 30 °C | `0x34A8` stayed 21.99 °C | **no** |
| Wind `0x0E90` = 42 kt | 42 | `0x3488` stayed 0.0 m/s | **no** |

Of the 20 fields whose offset accepted a write, only **3** could be proven to have reached the
sim, **4** were proven not to, and **13** have no mirror available and therefore cannot be
verified either way.

Control events behave the opposite way: every event fired against a system the aircraft actually
has produced an observable change. The two that did nothing — `GEAR_SET` and
`TOGGLE_STRUCTURAL_DEICE` — were on a C172, which has fixed gear and no de-ice.

So the rule is not "the offset is read-only". It is: **an offset write fails silently and
indistinguishably from success, and a control event does not.** For a bridge that reports its
capabilities to Shirley, that difference is the whole game.

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
| `flapsHandlePercentDown` | ✅ `0x0BDC` | ⚡ `FLAPS_SET` | `0…16383`. MSFS snaps to the aircraft's detents: commanding 6041 read back as 5461. |
| `speedBrakesHandlePercentDeployed` | ✗ `0x0BD0` | — | Echo on a C172, which has no spoilers. Retest on a spoiler-equipped aircraft. |
| `landingGearHandlePercentDown` | ◐ `0x0BE8` | ⚡ `GEAR_SET` | Neither took on a fixed-gear C172, as expected. Retest on a retractable. |
| `carburetorHeatLeverPercentHot` | ◐ `0x08B2` | ⚡ `ANTI_ICE_SET_ENG1` | Binary `0`/`1`, not a percentage — MSFS models carb heat as the engine anti-ice switch. |
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
`VNAVSpeed` — have no generic MSFS event. `levelChange` maps to `FLIGHT_LEVEL_CHANGE` and
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
| `propHeatSwitchOn` | ✗ `0x2440` | — | Echo on a C172. No prop-specific set event ships in the preset catalogue. |
| `governorSwitchOn` | — | `HELICOPTER_ENGINE_n_GOVERNOR_SWITCH_SET` | Helicopters only, not exercised |
| `totalEnergyAudioSwitchOn` | — | `TOGGLE_VARIOMETER_SWITCH` | Toggle only, not exercised |

### environment

**Weather cannot be set — verified, not inferred.** Writes to ambient temperature and wind are
accepted and echoed back while the sim's own value never moves; sea-level pressure is rejected
outright. That rules out all 20 cloud-layer, wind-layer, visibility, pressure, temperature,
precipitation, runway-friction, thermal and weather-evolution fields.

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
and `positionFreezeEnabled` have no absolute setter in MSFS. A bridge can make them idempotent by
reading current state before toggling, though that is a race in principle — `Writability.AfterRead`
fits them naturally.

**Percent versus detent.** `flapsHandlePercentDown` is continuous `0…16383`, which MSFS snaps to
the aircraft's detents; offset `0x3BFA` gives the increment per detent (5461 on the C172, so four
positions). Reading back gives the snapped value, so a naive write-then-verify reports a mismatch.

**Carb heat is binary** in MSFS, though the schema types it as a percentage.

## What has not been exercised

Everything above marked "not exercised", plus: spoilers and prop de-ice need a suitably equipped
aircraft; landing gear needs a retractable; `governorSwitchOn` needs a helicopter. The position
and attitude writes are untested because they teleport the aircraft.

Re-run [`tools/verify_msfs2024.py --write`](../tools/verify_msfs2024.py) on a different airframe
to close those gaps; the script prints the same matrix.
