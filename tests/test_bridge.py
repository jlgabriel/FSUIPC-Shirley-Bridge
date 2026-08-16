"""
End-to-end tests for the two paths that carry real traffic.

The transform tests cover pure functions. These cover the parts where the
defects actually lived: a fake FSUIPC payload fed through
``FSUIPCWSClient._handle_incoming``, asserted on ``SimData.get_snapshot()``,
and a fake Shirley ``SetSimData`` message asserted on what reaches the wire.
"""

import json

import pytest

from fsuipc_shirley_bridge import (
    FSUIPCWSClient,
    ShirleyWebSocketServer,
    SimData,
    SET_FIELDS,
    WRITABLE_OFFSETS,
    _flatten_set_simdata,
)


# ===================== helpers =====================

class FakeWS:
    """Stands in for the FSUIPC server socket.

    Records what the bridge sends and answers command responses the way the
    real server does: ``vars.calc`` always replies, a successful
    ``offsets.write`` replies with nothing at all.
    """

    def __init__(self, calc_ok=True):
        self.sent = []
        self.calc_ok = calc_ok
        self.closed = False
        self._client = None

    def attach(self, client):
        self._client = client
        return self

    async def send(self, raw):
        import asyncio
        msg = json.loads(raw)
        self.sent.append(msg)
        if msg.get("command") == "vars.calc":
            reply = {"command": "vars.calc", "name": msg.get("name"),
                     "success": self.calc_ok, "errorCode": None, "errorMessage": None}
            # Deliver after the caller has registered its waiter.
            asyncio.get_running_loop().call_soon(self._client._resolve_waiter, reply)

    async def close(self):
        self.closed = True

    def codes(self):
        return [m["code"] for m in self.sent if m.get("command") == "vars.calc"]

    def writes(self):
        return [m for m in self.sent if m.get("command") == "offsets.write"]


def make_client(calc_ok=True):
    sim = SimData()
    client = FSUIPCWSClient(sim)
    ws = FakeWS(calc_ok=calc_ok).attach(client)
    client.ws = ws
    client.WRITE_ERROR_WINDOW_S = 0.01   # a successful write never answers
    client.CALC_TIMEOUT_S = 1.0
    return sim, client, ws


async def feed(client, payload):
    """Push one FSUIPC offsets.read response through the bridge."""
    await client._handle_incoming(json.dumps({
        "command": "offsets.read", "name": "flightData",
        "success": True, "data": payload,
    }))


def raw_alt_bug_ft(feet):
    """0x07D4 is metres * 65536."""
    return int(feet / 3.28084 * 65536)


def raw_hdg_bug_deg(degrees):
    """0x07CC is degrees * 65536 / 360."""
    return int(degrees * 65536 / 360)


# ===================== read path =====================

@pytest.mark.asyncio
class TestReadPath:

    async def test_position_and_attitude(self):
        sim, client, _ = make_client()
        await feed(client, {
            "LatitudeDeg": -34.6, "LongitudeDeg": -58.4,
            "AltitudeM": 1000.0, "GroundSpeedKts": 120 * 65536 / 1.943844,
            "IASraw_U32": 110 * 128,
            "HeadingTrueRaw": int(90 * 65536 * 65536 / 360),
        })
        snap = await sim.get_snapshot()

        assert snap["position"]["latitudeDeg"] == pytest.approx(-34.6, abs=1e-4)
        assert snap["position"]["mslAltitudeFt"] == pytest.approx(3280.84, abs=1.0)
        assert snap["position"]["indicatedAirspeedKts"] == pytest.approx(110.0, abs=0.1)
        assert snap["attitude"]["trueHeadingDeg"] == pytest.approx(90.0, abs=0.1)

    async def test_roll_and_pitch_signs(self):
        """FSUIPC is positive-nose-down and positive-left-bank; Shirley is not."""
        sim, client, _ = make_client()
        quarter = 65536 * 65536 / 360
        await feed(client, {
            "PitchRaw": int(-5 * quarter),    # FSUIPC negative = nose up
            "BankRaw": int(-20 * quarter),    # FSUIPC negative = right bank
        })
        snap = await sim.get_snapshot()

        assert snap["attitude"]["pitchAngleDegUp"] == pytest.approx(5.0, abs=0.1)
        assert snap["attitude"]["rollAngleDegRight"] == pytest.approx(20.0, abs=0.1)

    async def test_autopilot_bugs_are_real_numbers(self):
        """The regression that made every bug come out as 1.0.

        Two dicts named _AUTOPILOT_SINK_TO_SHIRLEY existed, get_snapshot walked
        the surviving one twice, and the second pass coerced numeric fields to
        bool because its type test was case-sensitive against camelCase names.
        """
        sim, client, _ = make_client()
        await feed(client, {
            "AP_ALT_BUG": raw_alt_bug_ft(5000),
            "AP_HDG_BUG": raw_hdg_bug_deg(270),
            "AP_VS_TARGET": 500,
            "AP_MASTER": 1,
        })
        ap = (await sim.get_snapshot())["autopilot"]

        assert ap["altitudeBugFt"] == pytest.approx(5000.0, abs=1.0)
        assert ap["magneticHeadingBugDeg"] == pytest.approx(270.0, abs=0.1)
        assert ap["targetVerticalSpeedUpFpm"] == pytest.approx(500.0)
        assert ap["isAutopilotEngaged"] is True

    async def test_altitude_mode_absent_without_data(self):
        sim, client, _ = make_client()
        await feed(client, {"AP_MASTER": 1})
        assert "altitudeMode" not in (await sim.get_snapshot())["autopilot"]

        await feed(client, {"AP_ALT_HOLD": 1})
        assert (await sim.get_snapshot())["autopilot"]["altitudeMode"] == "altitudeHold"

    async def test_partial_payloads_accumulate(self):
        """FSUIPC answers with changes only, never the full group."""
        sim, client, _ = make_client()
        await feed(client, {"LatitudeDeg": 10.0, "LongitudeDeg": 20.0})
        await feed(client, {"LatitudeDeg": 11.0})
        pos = (await sim.get_snapshot())["position"]

        assert pos["latitudeDeg"] == pytest.approx(11.0)
        assert pos["longitudeDeg"] == pytest.approx(20.0)

    async def test_out_of_range_temperature_is_omitted(self):
        """Never publish a fabricated constant as if it were telemetry."""
        sim, client, _ = make_client()
        await feed(client, {"OUTSIDE_TEMP": 20 * 256})
        assert (await sim.get_snapshot())["environment"]["groundTemperatureDegC"] == pytest.approx(20.0)

        sim2, client2, _ = make_client()
        await feed(client2, {"OUTSIDE_TEMP": 300 * 256})
        assert "groundTemperatureDegC" not in (await sim2.get_snapshot()).get("environment", {})

    async def test_barometer_prefers_first_altimeter(self):
        sim, client, _ = make_client()
        await feed(client, {"BARO_0330_U32": int(1013.25 * 16), "BARO_0332_U32": int(1000.0 * 16)})
        snap = await sim.get_snapshot()

        assert snap["indicators"]["altimeterSettingInchesMercury"] == pytest.approx(29.92, abs=0.02)

    async def test_barometer_falls_back_to_second(self):
        sim, client, _ = make_client()
        await feed(client, {"BARO_0330_U32": 0, "BARO_0332_U32": int(1013.25 * 16)})
        snap = await sim.get_snapshot()

        assert snap["indicators"]["altimeterSettingInchesMercury"] == pytest.approx(29.92, abs=0.02)

    async def test_lights_bitmask(self):
        sim, client, _ = make_client()
        await feed(client, {"LIGHTS_BITS32": 0b11001})   # nav, taxi, strobe
        lights = (await sim.get_snapshot())["lights"]

        assert lights["navigationLightsSwitchOn"] is True
        assert lights["taxiLightsSwitchOn"] is True
        assert lights["strobeLightsSwitchOn"] is True
        assert lights["landingLightsSwitchOn"] is False

    async def test_aircraft_name_reaches_the_snapshot(self):
        """It used to be written into a kwargs dict after the task was created."""
        sim, client, _ = make_client()
        await feed(client, {"aircraftNameStr": "Cessna Skyhawk G1000"})
        assert (await sim.get_snapshot())["simulation"]["aircraftName"] == "Cessna Skyhawk G1000"

    async def test_parking_brake_reaches_the_snapshot(self):
        sim, client, _ = make_client()
        await feed(client, {"parkingBrakeU": 32767})
        assert (await sim.get_snapshot())["systems"]["parkingBrakeOn"] is True

    async def test_command_error_response_is_not_parsed_as_data(self):
        sim, client, _ = make_client()
        await client._handle_incoming(json.dumps({
            "command": "offsets.declare", "name": "flightData",
            "success": False, "errorCode": "UnknownName", "errorMessage": "nope",
        }))
        assert await sim.get_snapshot() == {}


# ===================== write path =====================

@pytest.mark.asyncio
class TestWritePath:

    async def test_gear_goes_out_as_an_event(self):
        _, client, ws = make_client()
        results = await client.apply_set_simdata({"levers": {"landingGearHandlePercentDown": 100}})

        assert results == [{"field": "levers.landingGearHandlePercentDown", "ok": True}]
        assert ws.codes() == ["1 (>K:GEAR_SET)"]

    async def test_heading_bug_encodes_degrees(self):
        _, client, ws = make_client()
        await client.apply_set_simdata({"autopilot": {"magneticHeadingBugDeg": 270}})
        assert ws.codes() == ["270 (>K:2:HEADING_BUG_SET)"]

    async def test_altitude_bug_never_uses_the_offset(self):
        """0x07D4 ignores the value and adds 1000 ft per write."""
        _, client, ws = make_client()
        await client.apply_set_simdata({"autopilot": {"altitudeBugFt": 5000}})

        assert ws.codes() == ["5000 (>K:2:AP_ALT_VAR_SET_ENGLISH)"]
        assert ws.writes() == []

    async def test_com_frequency_encodes_bcd(self):
        _, client, ws = make_client()
        await client.apply_set_simdata(
            {"radiosNavigation": {"standbyFrequencyHz": {"com1": 121850}}})

        assert ws.codes() == [f"{0x2185} (>K:COM_STBY_RADIO_SET)"]

    async def test_offsets_write_uses_group_and_name(self):
        """The old shape — {"values": [{"address": ...}]} — was discarded in silence."""
        _, client, ws = make_client()
        client.raw_state.update({"FLAPS_DETENT_INC": 8191, "FLAPS_NUM_POS": 2})
        await client.apply_set_simdata({"levers": {"flapsHandlePercentDown": 100}})

        assert ws.writes() == [{
            "command": "offsets.write", "name": "flightData",
            "offsets": [{"name": "FLAPS_INDEX", "value": 2}],
        }]

    async def test_flaps_fall_back_to_the_event_without_detent_data(self):
        _, client, ws = make_client()
        await client.apply_set_simdata({"levers": {"flapsHandlePercentDown": 50}})

        assert ws.writes() == []
        assert ws.codes() == ["8192 (>K:FLAPS_SET)"]     # 50% of 16383

    async def test_flaps_snap_to_the_nearest_detent(self):
        _, client, ws = make_client()
        client.raw_state.update({"FLAPS_DETENT_INC": 5461, "FLAPS_NUM_POS": 3})
        await client.apply_set_simdata({"levers": {"flapsHandlePercentDown": 33}})

        assert ws.writes()[0]["offsets"] == [{"name": "FLAPS_INDEX", "value": 1}]

    async def test_toggle_is_idempotent(self):
        sim, client, ws = make_client()
        await feed(client, {"BATTERY_MAIN": 1})

        await client.apply_set_simdata({"systems": {"batteryOn": {"main": True}}})
        assert ws.codes() == []                      # already on, nothing sent

        await client.apply_set_simdata({"systems": {"batteryOn": {"main": False}}})
        assert ws.codes() == ["(>K:TOGGLE_MASTER_BATTERY)"]

    async def test_toggle_refuses_without_known_state(self):
        _, client, ws = make_client()
        results = await client.apply_set_simdata({"systems": {"batteryOn": {"main": True}}})

        assert results[0]["ok"] is False
        assert ws.codes() == []

    async def test_unwritable_offset_is_blocked(self):
        """Writing a read-only offset poisons its reads for the whole session."""
        _, client, ws = make_client()
        assert await client.write_offset("OUTSIDE_TEMP", 20) is False
        assert ws.writes() == []
        assert "OUTSIDE_TEMP" not in WRITABLE_OFFSETS

    async def test_weather_is_rejected_with_a_reason(self):
        _, client, ws = make_client()
        results = await client.apply_set_simdata({"environment": {"groundTemperatureDegC": 20}})

        assert results[0]["ok"] is False
        assert "clima" in results[0]["error"]
        assert ws.sent == []           # nothing was even attempted

    async def test_failed_calc_is_reported(self):
        _, client, _ = make_client(calc_ok=False)
        results = await client.apply_set_simdata({"lights": {"strobeLightsSwitchOn": True}})

        assert results[0]["ok"] is False

    async def test_multiple_fields_in_one_message(self):
        _, client, ws = make_client()
        results = await client.apply_set_simdata({
            "lights": {"navigationLightsSwitchOn": True, "strobeLightsSwitchOn": False},
        })

        assert [r["ok"] for r in results] == [True, True]
        assert ws.codes() == ["1 (>K:NAV_LIGHTS_SET)", "0 (>K:STROBES_SET)"]


# ===================== message recognition =====================

class TestSetSimDataRecognition:
    """Shirley sends the bare nested object, with no 'type' wrapper."""

    def test_bare_nested_object_is_recognised(self):
        body = ShirleyWebSocketServer._as_set_simdata(
            {"levers": {"flapsHandlePercentDown": 50}})
        assert body == {"levers": {"flapsHandlePercentDown": 50}}

    def test_wrapped_form_still_works(self):
        body = ShirleyWebSocketServer._as_set_simdata(
            {"type": "SetSimData", "data": {"lights": {"taxiLightsSwitchOn": True}}})
        assert body == {"lights": {"taxiLightsSwitchOn": True}}

    def test_unrelated_message_is_ignored(self):
        assert ShirleyWebSocketServer._as_set_simdata({"hello": "world"}) is None

    def test_flatten_walks_to_the_leaves(self):
        paths = dict(_flatten_set_simdata({
            "systems": {"batteryOn": {"main": True}},
            "autopilot": {"altitudeBugFt": 5000, "isAutopilotEngaged": None},
        }))
        assert paths == {"systems.batteryOn.main": True, "autopilot.altitudeBugFt": 5000}

    def test_every_registered_path_has_a_mechanism(self):
        for path, spec in SET_FIELDS.items():
            assert spec["kind"] in ("event", "toggle", "offset", "custom"), path
            if spec["kind"] == "event" and "{v}" in spec["code"]:
                assert callable(spec.get("encode")), path
            if spec["kind"] == "offset":
                assert spec["offset"] in WRITABLE_OFFSETS, path
            if spec["kind"] == "toggle":
                assert isinstance(spec.get("state"), tuple), path
