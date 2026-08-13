# FSUIPC Setpoint Capabilities for MSFS

**What a Shirley bridge built on FSUIPC7 can actually *set* in Microsoft Flight Simulator.**

Prepared for the Airplane Team, August 2026. Mapped field-by-field against
`schemas/set_simdata_schemas_xplane.ts` from [Airplane-Team/sim-interface](https://github.com/Airplane-Team/sim-interface).

- **Sim:** MSFS 2024 (Steam). MSFS 2020 not available for testing.
- **FSUIPC7 v7.5.6**, licensed, with the bundled WebSocket Server and the WASM module active.
- **Sources:** `FSUIPC7 Offsets Status.pdf` v0.8.4 (Jan 2026) — John Dowson's per-offset record of
  which MSFS SimVars respond to reads and to writes — plus the ~23 000 MobiFlight/HubHop presets
  shipped in `events.txt`, and `FSUIPC7 for Advanced Users.pdf`.

## Summary

**Yes — FSUIPC can cover essentially all of `SetSimData` except weather.** Set the weather group
aside and **36 of the remaining 42 schema fields are reachable**, most of them trivially. The gaps
are few and specific:

| Not reachable | Why |
|---|---|
| The whole `environment` weather group | Every ambient/pressure/visibility offset is **read-only**. The MSFS SDK exposes no weather write path to FSUIPC. Only `zuluTimeHours` and `dayOfYear` are settable. |
| `failures.*` | `GENERAL ENG FAILED:n` offsets are read-only and there are no failure-set events. |
| `simulation.isCrashed`, `simulation.shouldResetFlight` | No corresponding control event. |
| `levers.propBetaEnabled` | `PROP BETA:n` is read-only; no beta-set event. |

Everything else — flaps, gear, spoilers, all autopilot modes and bugs, lights, radios,
transponder, altimeter, parking brake, battery, pitot heat, position and attitude — works.

## How writing works

The important structural point: **MSFS makes many legacy offsets read-only**, because the
underlying SimVar is not settable. Where that happens you send a *control event* instead. So a
bridge needs two code paths, not one.

The good news is that both paths travel over the same transport. `offsets.write` is enough for
everything; you never strictly need a second WebSocket command.

**1. Direct offset write** — for offsets whose SimVar is settable.

```json
{"command":"offsets.write","name":"flightData",
 "offsets":[{"name":"gearHandle","value":16383}]}
```

Note the shape: the offset must belong to a group you previously declared, and is referenced **by
name, not address**. A successful write replies with an `offsets.read` response; you only get an
`offsets.write` response back when it failed.

**2. Control event via offset `0x3110`** — the universal escape hatch, and it works in
*unregistered* FSUIPC with no WASM module. Write 8 bytes: a 32-bit control number followed by a
32-bit parameter. FSUIPC fires the control the moment `0x3110` is written.

**3. Named preset via offset `0x7C50`** — write the parameter to `0x7C90` (32-bit), then the
preset name prefixed with `P:` to `0x7C50` (max 64 chars). The same offset also takes `L:` to set
an LVar, `H:` to fire an HVar and `I:` for Input Events. Requires the WASM module and that the
name is known to FSUIPC.

There is also `vars.calc`, a dedicated WebSocket command that executes raw MSFS calculator code
(RPN). It is the most expressive option and the easiest to read, so the tables below name events
in that form:

```json
{"command":"vars.calc","name":"setFlaps","code":"16383 (>K:FLAPS_SET)"}
```

Any `⚡` row below can be done through mechanism 2, 3 or `vars.calc` interchangeably. Mechanism 2
is the most portable, since it needs neither the WASM module nor a licence.

**Legend** — ✅ direct offset write · ⚡ control event · ⚠️ works with a caveat · ❌ not available

---

## position

Writing position teleports the aircraft. In practice these need slew mode (`0x05DC`, writable) or
a freeze, otherwise the flight model fights the write.

| Field | | Target | Conversion |
|---|---|---|---|
| `latitudeDeg` | ✅ | offset `0x0560` (8 bytes) | `deg × (10001750 × 65536²) / 90` |
| `longitudeDeg` | ✅ | offset `0x0568` (8 bytes) | `deg × 65536⁴ / 360` |
| `mslAltitudeFt` | ✅ | offset `0x0570` (8 bytes) | metres × 65536² |
| `aglAltitudeFt` | ⚠️ | derive | No settable AGL. Read ground altitude from `0x0020` (metres × 256, read-only) and write MSL = AGL + ground. |
| `indicatedAirspeedKts` | ✅ | offset `0x02BC` (4 bytes) | knots × 128 |

## attitude

| Field | | Target | Conversion |
|---|---|---|---|
| `pitchAngleDegUp` | ✅ | offset `0x0578` (4 bytes) | `−deg × 65536² / 360` — FSUIPC is negative-for-nose-up, so the sign inverts |
| `trueHeadingDeg` | ✅ | offset `0x0580` (4 bytes) | `deg × 65536² / 360` |

## radiosNavigation

All radio offsets are read-only in MSFS; everything here is an event. Frequency parameters are
4-digit **BCD with the leading 1 assumed** — 123.45 MHz is `0x2345`.

| Field | | Target | Notes |
|---|---|---|---|
| `standbyFrequencyHz.com1` | ⚡ | `COM_STBY_RADIO_SET` | BCD |
| `standbyFrequencyHz.com2` | ⚡ | `COM2_STBY_RADIO_SET` | BCD. Not exercised by any shipped preset — worth a live check. |
| `standbyFrequencyHz.nav1` | ⚡ | `NAV1_STBY_SET` | BCD. Same caveat. |
| `comShouldSwapFrequencies.com1` | ⚡ | `COM_STBY_RADIO_SWAP` | no parameter |
| `comShouldSwapFrequencies.com2` | ⚡ | `COM2_STBY_RADIO_SWAP` | no parameter |
| `transponderCode` | ⚡ | `XPNDR_SET` | BCD: squawk 1200 is `0x1200` |

## lights

Offset `0x0D0C` is a read-only bitmask in MSFS. Each light takes `0` or `1`.

| Field | | Target |
|---|---|---|
| `landingLightsSwitchOn` | ⚡ | `LANDING_LIGHTS_SET` |
| `taxiLightsSwitchOn` | ⚡ | `TAXI_LIGHTS_SET` |
| `navigationLightsSwitchOn` | ⚡ | `NAV_LIGHTS_SET` |
| `strobeLightsSwitchOn` | ⚡ | `STROBES_SET` |

## indicators

| Field | | Target | Conversion |
|---|---|---|---|
| `altimeterSettingInchesMercury` | ✅ | offset `0x0330` (2 bytes) | millibars × 16, so `inHg / 0.02953 × 16`. `KOHLSMAN_SET` also works. |

## levers

| Field | | Target | Notes |
|---|---|---|---|
| `flapsHandlePercentDown` | ⚡ | `FLAPS_SET` | Parameter `0…16383`. **Offset `0x0BDC` is read-only** — this is the one place where the obvious approach fails. |
| | ✅ | offset `0x0BFC` (1 byte) | Alternative: flaps handle *index* (detent number, 0 = up) rather than a percentage. |
| `speedBrakesHandlePercentDeployed` | ✅ | offset `0x0BD0` (4 bytes) | `0…16383`. 4800 means "armed". |
| `landingGearHandlePercentDown` | ✅ | offset `0x0BE8` (4 bytes) | `0` = up, `16383` = down. Effectively binary. |
| `carburetorHeatLeverPercentHot` | ⚡ | `ANTI_ICE_SET_ENG1` | Binary `0`/`1`, not a percentage — MSFS models carb heat as the engine anti-ice switch. |
| `propBetaEnabled` | ❌ | — | `PROP BETA:n` (`0x2418`) is read-only and there is no beta-set event. |

## autopilot

Every autopilot offset is read-only in MSFS. All of this is events.

| Field | | Target | Notes |
|---|---|---|---|
| `isAutopilotEngaged` | ⚡ | `AUTOPILOT_ON` / `AUTOPILOT_OFF` | `AP_MASTER` toggles instead |
| `isFlightDirectorEngaged` | ⚠️ | `TOGGLE_FLIGHT_DIRECTOR` | Toggle only — read the current state first to stay idempotent |
| `isHeadingSelectEnabled` | ⚡ | `AP_HDG_HOLD_ON` / `AP_HDG_HOLD_OFF` | |
| `magneticHeadingBugDeg` | ⚡ | `HEADING_BUG_SET` | degrees |
| `altitudeBugFt` | ⚡ | `AP_ALT_VAR_SET_ENGLISH` | feet |
| `targetVerticalSpeedUpFpm` | ⚡ | `AP_VS_VAR_SET_ENGLISH` | fpm. FSUIPC notes writes to the VS value only take effect *after* an AP VS SET control has been sent once — so send the event, not the offset. |
| `shouldLevelWings` | ⚡ | `AP_WING_LEVELER` | |
| `altitudeMode` | ⚠️ | partial | see below |

`altitudeMode` maps cleanly for four of its eleven values:

| Value | Event |
|---|---|
| `altitudeHold` | `AP_ALT_HOLD_ON` (or `AP_PANEL_ALTITUDE_HOLD`) |
| `verticalSpeed` | `AP_VS_HOLD` (or `AP_PANEL_VS_HOLD`) |
| `levelChange` | `FLIGHT_LEVEL_CHANGE` |
| `glideSlope` | `AP_APR_HOLD` |
| `disabled` | the matching `*_OFF` event |
| `pitch`, `terrain`, `VNAV`, `TOGA`, `flightPathAngle`, `VNAVSpeed` | no generic MSFS event — these are avionics-specific and would need per-aircraft LVars/HVars |

## systems

| Field | | Target | Notes |
|---|---|---|---|
| `batteryOn.main` | ✅ | offset `0x281C` (4 bytes) | `0`/`1`. `TOGGLE_MASTER_BATTERY` also exists but toggles. |
| `parkingBrakeOn` | ✅ | offset `0x0BC8` (2 bytes) | `0` = off, `32767` = on. `PARKING_BRAKE_SET` also works. |
| `pitotHeatSwitchOn` | ⚡ | `PITOT_HEAT_SET` | `0`/`1`. Offset `0x029C` is read-only. |
| `governorSwitchOn` | ⚡ | `HELICOPTER_ENGINE_1_GOVERNOR_SWITCH_SET` | helicopters only; `_2_` for the second engine |
| `totalEnergyAudioSwitchOn` | ⚠️ | `TOGGLE_VARIOMETER_SWITCH` | Toggle only |
| `propHeatSwitchOn` | ⚠️ | `TOGGLE_STRUCTURAL_DEICE` | `PROP DEICE SWITCH:n` (`0x2440`) is read-only and no prop-specific set event ships in the preset file. The structural de-ice toggle is the closest generic equivalent — needs a live check. |

## environment

**Weather cannot be set.** Every relevant offset — `AMBIENT TEMPERATURE`, `AMBIENT WIND
VELOCITY`/`DIRECTION`, `AMBIENT VISIBILITY`, `AMBIENT PRESSURE`, `SEA LEVEL PRESSURE`, precipitation
— is marked read-only, and FSUIPC's own release notes state the MSFS SDK gives it no weather
write access. That rules out every cloud layer, wind layer, visibility, pressure, temperature,
precipitation, runway friction, thermal and weather-evolution field in `SetEnvironment`.

Two fields survive:

| Field | | Target |
|---|---|---|
| `zuluTimeHours` | ⚡ | `ZULU_HOURS_SET` (plus `ZULU_MINUTES_SET`) |
| `dayOfYear` | ⚡ | `ZULU_DAY_SET` |

## simulation

| Field | | Target | Notes |
|---|---|---|---|
| `isPaused` | ⚡ | `PAUSE_ON` / `PAUSE_OFF` | |
| `simSpeedRatio` | ⚠️ | `SIM_RATE_INCR` / `SIM_RATE_DECR` | Stepwise only — no absolute set. `SIMULATION RATE` (`0x0C1A`) is read-only, so you must read it and step toward the target. |
| `isCrashed` | ❌ | — | |
| `shouldResetFlight` | ❌ | — | Only `SITUATION_SAVE` exists; there is no reset/reload event. |

## freezes

| Field | | Target | Notes |
|---|---|---|---|
| `positionFreezeEnabled` | ⚠️ | `FREEZE_LATITUDE_LONGITUDE_TOGGLE` | Toggle only. Read the current state from `0x3540` (`IS LATITUDE LONGITUDE FREEZE ON`) to make it idempotent. `FREEZE_ALTITUDE_TOGGLE` and `FREEZE_ATTITUDE_TOGGLE` also exist; their status offsets `0x081C`/`0x081D` are documented as *not currently populated correctly*. Slew mode (`0x05DC`, writable) is the sturdier alternative. |

---

## Notes for the Shirley side

**`SimPlatform` has no MSFS 2024.** The enum in `data_descriptor.ts` defines `xplane12`,
`msfs2020` and `generic`. MSFS 2024 is a separate product with its own SimVar behaviour; if
`msfs2020` is meant as a catch-all for MSFS, that is fine, but the naming will get confusing.

**Toggle-only fields need read-back.** `isFlightDirectorEngaged`, `totalEnergyAudioSwitchOn` and
`positionFreezeEnabled` have no absolute setter in MSFS. A bridge can make them idempotent by
reading current state first, but that is a race in principle. If `Writability.AfterRead` already
carries that meaning on the Shirley side, these fit it naturally.

**Percent vs. detent.** `flapsHandlePercentDown` maps to a continuous `0…16383`, which MSFS then
snaps to the aircraft's detents. Reading it back gives the snapped value, not the commanded one,
so a naive write-then-verify will report a mismatch.

**Carb heat is binary.** `carburetorHeatLeverPercentHot` is a percentage in the schema but a
switch in MSFS.

## Verification status

Everything above is derived from FSUIPC7's own MSFS offset-status record and its shipped preset
catalogue — that is, from what John Dowson has measured against MSFS, not from the FSX-era offset
lists that circulate on the web. It has **not** yet been exercised against a running sim. The
read/write determination comes from the status document's per-offset SDK response columns.

Live confirmation is the obvious next step, and the fields worth testing first are the ones where
the documentation leaves room for doubt: `COM2_STBY_RADIO_SET`, `NAV1_STBY_SET`, `propHeatSwitchOn`
and the flaps percent-to-detent round-trip.
