import asyncio
import json
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any, Set

import websockets
import websockets.exceptions

# ===================== Configuration =====================
FSUIPC_WS_URL = "ws://localhost:2048/fsuipc/"  # FSUIPC WebSocket Server (Paul Henty)
WS_HOST = "localhost"
WS_PORT = 2992
WS_PATH = "/api/v1"                     # for logging only
SEND_INTERVAL = 0.25                    # 4 Hz (every 250 ms)
DEBUG_FSUIPC_MESSAGES = False
FIRST_PAYLOAD = False

# ===================== Declarative writes =====================
WRITE_COMMANDS = {
    "GEAR_HANDLE": {  # 0=retracted, 1=down
        "type": "offset",
        "address": 0x0BE8, "size": 4, "dtype": "int",
        "encode": lambda v: 16383 if int(float(v)) else 0,
    },
    "throttle": {     # accepts -1..1 or raw value [-16384..16384]
        "type": "offset",
        "address": 0x088C, "size": 2, "dtype": "short",
        "encode": lambda v: (
            max(0, min(16384, round((float(v)+1.0)*0.5*16384.0))) if -1.0 <= float(v) <= 1.0
            else max(-16384, min(16384, int(float(v))))
        ),
    },
}

def compute_capabilities_writes():
    return sorted(WRITE_COMMANDS.keys())

# ===================== Constants / Helpers =====================
METERS_TO_FEET = 3.28084
MPS_TO_KTS = 1.943844

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def iso_utc_ms() -> str:
    t = time.time()
    whole = int(t)
    ms = int((t - whole) * 1000.0)
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(whole)) + f".{ms:03d}Z"

# ---- FSUIPC raw -> human conversions ----
def fs_lat_to_deg(raw: int) -> float:
    # 64-bit FS units -> degrees
    return (raw * 90.0) / (10001750.0 * 65536.0 * 65536.0)

def fs_lon_to_deg(raw: int) -> float:
    # 64-bit FS units -> degrees (2^64 = 65536^4)
    return (raw * 360.0) / (65536.0 * 65536.0 * 65536.0 * 65536.0)

def fs_alt_to_m(raw: int) -> float:
    # meters * 65536 -> meters
    return raw / 65536.0

def fs_heading_true_deg(raw: int) -> float:
    # fraction of turn * 2^32 -> degrees
    return (raw * 360.0) / (65536.0 * 65536.0)

def fs_ground_speed_mps(raw: int) -> float:
    # 65536 * m/s -> m/s
    return raw / 65536.0

def fs_angle_deg(raw: int) -> float:
    # For pitch/bank (same factor as heading)
    return (raw * 360.0) / (65536.0 * 65536.0)

# ===================== FSUIPC signals to read (declarative) =====================
# Note: we use 'lat'/'lon' to receive degrees directly; Easy altitude in meters (0x6020 float).
READ_SIGNALS = {
    # --- Position ---
    "LatitudeDeg":   {"address": 0x0560, "type": "lat",   "size": 8, "sink": ("gps", "latitude")},      # deg
    "LongitudeDeg":  {"address": 0x0568, "type": "lon",   "size": 8, "sink": ("gps", "longitude")},     # deg
    "AltitudeM":     {"address": 0x6020, "type": "float", "size": 8, "sink": ("gps", "alt_msl_meters")},# m

    "GroundSpeedKts":{"address": 0x02B4, "type": "uint",  "size": 4, "transform": "knots128_to_kts", "sink": ("gps", "ground_speed_kts")},

    # --- Airspeeds / VS ---
    "IASraw":   {"address": 0x02BC, "type": "uint", "size": 4, "transform": "knots128_to_kts", "sink": ("gps", "ias_kts")},
    "VSraw":    {"address": 0x02C8, "type": "int",  "size": 4, "transform": "vs_raw_to_fpm",   "sink": ("gps", "vs_fpm_raw")},

    # --- AGL via ground altitude ---
    "GroundAltRaw": {"address": 0x0020, "type": "int", "size": 4, "transform": "meters256_to_m", "sink": ("gps", "ground_alt_m")},

    # --- Attitude ---
    "HeadingTrueRaw":{"address": 0x0580, "type": "uint",  "size": 4, "transform": "raw_hdg_to_deg", "sink": ("att", "heading_deg")},
    "PitchRaw":      {"address": 0x0578, "type": "int",   "size": 4, "transform": "raw_ang_to_deg_pitch", "sink": ("att", "pitch_deg")},
    "BankRaw":       {"address": 0x057C, "type": "int",   "size": 4, "transform": "raw_ang_to_deg", "sink": ("att", "roll_deg")},

    # --- Magnetic variation (for magnetic heading) ---
    "MagVarRaw": {"address": 0x02A0, "type": "short", "size": 2, "transform": "magvar_raw_to_deg", "sink": ("att", "mag_var_deg")},

    # --- Lights (bitfield from 0x0D0C) ---
    "LIGHT_NAV":     {"address": 0x0D0C, "type": "ushort", "size": 2, "transform": "bit0_to_bool", "sink": ("lights", "nav_on")},
    "LIGHT_BEACON":  {"address": 0x0D0C, "type": "ushort", "size": 2, "transform": "bit1_to_bool", "sink": ("lights", "beacon_on")},
    "LIGHT_LANDING": {"address": 0x0D0C, "type": "ushort", "size": 2, "transform": "bit2_to_bool", "sink": ("lights", "landing_on")},
    "LIGHT_STROBE":  {"address": 0x0D0C, "type": "ushort", "size": 2, "transform": "bit4_to_bool", "sink": ("lights", "strobe_on")},
    "LIGHT_TAXI":    {"address": 0x0D0C, "type": "ushort", "size": 2, "transform": "bit2_to_bool", "sink": ("lights", "taxi_on")},
}

# Transforms to units expected by SimData.update_*_partial()
def raw_ang_to_deg(raw):
    return fs_angle_deg(raw) if raw is not None else None

def raw_ang_to_deg_pitch(raw):
    v = fs_angle_deg(raw) if raw is not None else None
    return -v if v is not None else None  # 👈 we invert sign for positive 'Up'
def raw_hdg_to_deg(raw):    return (fs_heading_true_deg(raw) % 360.0) if raw is not None else None
def mps_to_mps(raw):        return fs_ground_speed_mps(raw) if raw is not None else None

# === transforms (add alongside existing ones) ===

TRANSFORMS = {
    "raw_ang_to_deg": raw_ang_to_deg,
    "raw_ang_to_deg_pitch": raw_ang_to_deg_pitch,
    "raw_hdg_to_deg": raw_hdg_to_deg,
    "mps_to_mps":     mps_to_mps,
}

# --- New transforms ---
def knots128_to_kts(raw):
    try: return float(raw) / 128.0
    except: return None

def vs_raw_to_fpm(raw):
    # raw = 256 * m/s  ->  ft/min
    try: return float(raw) * 60.0 * 3.28084 / 256.0
    except: return None

def meters256_to_m(raw):
    # ground altitude in meters *256
    try: return float(raw) / 256.0
    except: return None

def magvar_raw_to_deg(raw):
    # 0x02A0: signed word; deg = raw * 360 / 65536, East positive (-ve = West in old docs)
    try:
        # interpret as int16
        if isinstance(raw, str) and raw.startswith("0x"):
            val = int(raw, 16)
            if val >= 0x8000: val -= 0x10000
        else:
            val = int(raw)
            if val >= 32768: val -= 65536
        return (val * 360.0) / 65536.0
    except:
        return None

def bit0_to_bool(raw):
    """Extract bit 0 from raw value"""
    try: return bool(int(raw) & 0x01)
    except: return None

def bit1_to_bool(raw):
    """Extract bit 1 from raw value"""
    try: return bool(int(raw) & 0x02)
    except: return None

def bit2_to_bool(raw):
    """Extract bit 2 from raw value"""
    try: return bool(int(raw) & 0x04)
    except: return None

def bit4_to_bool(raw):
    """Extract bit 4 from raw value"""
    try: return bool(int(raw) & 0x10)
    except: return None

TRANSFORMS.update({
    "knots128_to_kts": knots128_to_kts,
    "vs_raw_to_fpm":   vs_raw_to_fpm,
    "meters256_to_m":  meters256_to_m,
    "magvar_raw_to_deg": magvar_raw_to_deg,
    # Bitfield transforms for lights
    "bit0_to_bool": bit0_to_bool,
    "bit1_to_bool": bit1_to_bool,
    "bit2_to_bool": bit2_to_bool,
    "bit4_to_bool": bit4_to_bool,
})

# ===================== Mapping sinks -> Shirley keys =====================
_GPS_SINK_TO_SHIRLEY = {
    "latitude":           "position.latitudeDeg",
    "longitude":          "position.longitudeDeg",
    "alt_msl_meters":     "position.mslAltitudeFt",
    "ground_speed_kts":   "position.gpsGroundSpeedKts",
    # "track_deg":        "position.trueGroundTrackDeg",  # when you publish it
    "ias_kts":         "position.indicatedAirspeedKts",
    "vs_fpm_raw":      "position.verticalSpeedUpFpm",  # we'll use raw if it arrives
    "ground_alt_m":    "position.aglAltitudeFt",       # calculated in snapshot
}

_ATT_SINK_TO_SHIRLEY = {
    "heading_deg":        "attitude.trueHeadingDeg",
    "pitch_deg":          "attitude.pitchAngleDegUp",
    "roll_deg":           "attitude.rollAngleDegRight",
    # the published value will be 'attitude.magneticHeadingDeg'
    # (calculated in snapshot), but we keep mag_var_deg as input.
    "mag_var_deg": None,
}

# ===================== Mapping sinks -> Shirley keys (NUEVOS GRUPOS) =====================

_LIGHTS_SINK_TO_SHIRLEY = {
    "nav_on": "lights.navigationLightsSwitchOn",
    "beacon_on": "lights.beaconLightsSwitchOn",
    "landing_on": "lights.landingLightsSwitchOn",
    "taxi_on": "lights.taxiLightsSwitchOn",
    "strobe_on": "lights.strobeLightsSwitchOn",
}

_SYSTEMS_SINK_TO_SHIRLEY = {
    "pitot_heat_on": "systems.pitotHeatSwitchOn",
    "brakes_on": "systems.brakesOn",
    "battery_main_on": "systems.batteryOn.main",
}

_AUTOPILOT_SINK_TO_SHIRLEY = {
    "ap_master_on": "autopilot.isAutopilotEngaged",
    "hdg_select_on": "autopilot.isHeadingSelectEnabled",
    "hdg_bug_deg": "autopilot.magneticHeadingBugDeg",
    "alt_bug_ft": "autopilot.altitudeBugFt",
    "vs_target_fpm": "autopilot.targetVerticalSpeedFpm",
}

_LEVERS_SINK_TO_SHIRLEY = {
    "flaps_pct": "levers.flapsHandlePercentDown",
    "gear_pct": "levers.landingGearHandlePercentDown",
    "throttle1_pct": "levers.throttlePercentOpen.engine1",
}

_INDICATORS_SINK_TO_SHIRLEY = {
    "altimeter_inhg": "indicators.altimeterSettingInchesMercury",
    "stall_warning_on": "indicators.stallWarningOn",
}

def compute_capabilities_reads():
    """Generates the list of 'reads' fields from all sink mappings."""
    out = set()

    # Existing groups
    for _, cfg in READ_SIGNALS.items():
        sink_group, sink_field = cfg["sink"]
        if sink_group == "gps":
            key = _GPS_SINK_TO_SHIRLEY.get(sink_field)
            if key: out.add(key)
        elif sink_group == "att":
            key = _ATT_SINK_TO_SHIRLEY.get(sink_field)
            if key: out.add(key)

    # New groups - add all possible keys from sink mappings
    for shirley_key in _LIGHTS_SINK_TO_SHIRLEY.values():
        out.add(shirley_key)
    for shirley_key in _SYSTEMS_SINK_TO_SHIRLEY.values():
        out.add(shirley_key)
    for shirley_key in _AUTOPILOT_SINK_TO_SHIRLEY.values():
        out.add(shirley_key)
    for shirley_key in _LEVERS_SINK_TO_SHIRLEY.values():
        out.add(shirley_key)
    for shirley_key in _INDICATORS_SINK_TO_SHIRLEY.values():
        out.add(shirley_key)

    # Software-derived fields
    out.add("position.verticalSpeedUpFpm")
    out.add("attitude.magneticHeadingDeg")
    out.add("attitude.trueGroundTrackDeg")

    return sorted(out)

# ===================== ForeFlight-style input data =====================
@dataclass
class XGPSData:
    sim_name: str
    longitude: Optional[float]
    latitude: Optional[float]
    alt_msl_meters: Optional[float]
    track_deg: float
    ground_speed_kts: float

@dataclass
class XATTData:
    sim_name: str
    heading_deg: float
    pitch_deg: float
    roll_deg: float  # positive = roll to the right

def validate_position_data(lat: float = None, lon: float = None, alt_ft: float = None) -> bool:
    """Validate basic position data ranges"""
    try:
        if lat is not None and not (-90.0 <= lat <= 90.0):
            return False
        if lon is not None and not (-180.0 <= lon <= 180.0):
            return False
        if alt_ft is not None and not (-1000.0 <= alt_ft <= 100000.0):  # reasonable flight envelope
            return False
        return True
    except (TypeError, ValueError):
        return False

# ===================== SimData model (Shirley format) =====================
class SimData:
    """
    Maintains the last XGPS/XATT and builds the JSON that Shirley consumes:
    {
      "position": {
        "latitudeDeg", "longitudeDeg", "mslAltitudeFt",
        "gpsGroundSpeedKts", "verticalSpeedUpFpm"
      },
      "attitude": {
        "rollAngleDegRight", "pitchAngleDegUp", "trueHeadingDeg",
        "trueGroundTrackDeg"
      }
    }
    """
    def __init__(self):
        self.xgps: Optional[XGPSData] = None
        self.xatt: Optional[XATTData] = None
        self._lock = asyncio.Lock()
        self.last_timestamp: Optional[str] = None

        # Vertical Speed (software derived)
        self._last_alt_ft = None
        self._last_vs_ts = None
        self._vs_fpm = None

        # New fields
        self._ias_kts = None
        self._vs_fpm_raw = None
        self._ground_alt_m = None
        self._mag_var_deg = None

        # Ground track calculation (bearing between consecutive positions)
        self._last_lat = None
        self._last_lon = None
        self._track_deg = None

        # New data groups
        self._lights_data = {}      # nav_on, landing_on, taxi_on, strobe_on
        self._systems_data = {}     # pitot_heat_on, brakes_on, battery_main_on
        self._autopilot_data = {}   # ap_master_on, hdg_select_on, hdg_bug_deg, alt_bug_ft, vs_target_fpm
        self._levers_data = {}      # flaps_pct, gear_pct, throttle1_pct
        self._indicators_data = {}  # altimeter_inhg, stall_warning_on

    async def update_from_xgps(self, xgps: XGPSData):
        async with self._lock:
            self.xgps = xgps
            self.last_timestamp = iso_utc_ms()

    async def update_from_xatt(self, xatt: XATTData):
        async with self._lock:
            self.xatt = xatt
            self.last_timestamp = iso_utc_ms()

    async def update_gps_partial(self, **kwargs):
        async with self._lock:
            curr = self.xgps or XGPSData(
                sim_name="MSFS-FSUIPC",
                longitude=None, latitude=None,
                alt_msl_meters=None, track_deg=0.0, ground_speed_kts=0.0
            )
            self.xgps = XGPSData(
                sim_name="MSFS-FSUIPC",
                longitude=kwargs.get("longitude") if kwargs.get("longitude") is not None else curr.longitude,
                latitude=kwargs.get("latitude") if kwargs.get("latitude") is not None else curr.latitude,
                alt_msl_meters=kwargs.get("alt_msl_meters") if kwargs.get("alt_msl_meters") is not None else curr.alt_msl_meters,
                track_deg=kwargs.get("track_deg") if kwargs.get("track_deg") is not None else curr.track_deg,
                ground_speed_kts=kwargs.get("ground_speed_kts") if kwargs.get("ground_speed_kts") is not None else curr.ground_speed_kts
            )
            self.last_timestamp = iso_utc_ms()

            # New fields
            if "ias_kts" in kwargs and kwargs["ias_kts"] is not None:
                self._ias_kts = float(kwargs["ias_kts"])
            if "vs_fpm_raw" in kwargs and kwargs["vs_fpm_raw"] is not None:
                self._vs_fpm_raw = float(kwargs["vs_fpm_raw"])
            if "ground_alt_m" in kwargs and kwargs["ground_alt_m"] is not None:
                self._ground_alt_m = float(kwargs["ground_alt_m"])

            # VS derived: Δalt_ft / Δmin
            now = time.time()
            alt_ft = None
            if self.xgps and self.xgps.alt_msl_meters is not None:
                alt_ft = self.xgps.alt_msl_meters * METERS_TO_FEET

            if alt_ft is not None:
                if self._last_alt_ft is not None and self._last_vs_ts is not None:
                    dt_min = max(1e-6, (now - self._last_vs_ts) / 60.0)
                    self._vs_fpm = (alt_ft - self._last_alt_ft) / dt_min
                self._last_alt_ft = alt_ft
                self._last_vs_ts = now

            # Calculate ground track from position changes
            if self.xgps and self.xgps.latitude is not None and self.xgps.longitude is not None:
                lat, lon = self.xgps.latitude, self.xgps.longitude

                # Only calculate if we have previous position and position actually changed
                if (self._last_lat is not None and self._last_lon is not None and
                    (abs(lat - self._last_lat) > 1e-7 or abs(lon - self._last_lon) > 1e-7)):
                    self._track_deg = self._bearing_deg(self._last_lat, self._last_lon, lat, lon)

                # Update last position
                self._last_lat, self._last_lon = lat, lon

    async def update_att_partial(self, **kwargs):
        async with self._lock:
            curr = self.xatt or XATTData(
                sim_name="MSFS-FSUIPC",
                heading_deg=0.0, pitch_deg=0.0, roll_deg=0.0
            )
            self.xatt = XATTData(
                sim_name="MSFS-FSUIPC",
                heading_deg=kwargs.get("heading_deg") if kwargs.get("heading_deg") is not None else curr.heading_deg,
                pitch_deg=kwargs.get("pitch_deg") if kwargs.get("pitch_deg") is not None else curr.pitch_deg,
                roll_deg=kwargs.get("roll_deg") if kwargs.get("roll_deg") is not None else curr.roll_deg
            )
            self.last_timestamp = iso_utc_ms()

            # New fields
            if "mag_var_deg" in kwargs and kwargs["mag_var_deg"] is not None:
                self._mag_var_deg = float(kwargs["mag_var_deg"])

    async def update_lights_partial(self, **kwargs):
        async with self._lock:
            for key, value in kwargs.items():
                if value is not None:
                    self._lights_data[key] = value
            self.last_timestamp = iso_utc_ms()

    async def update_systems_partial(self, **kwargs):
        async with self._lock:
            for key, value in kwargs.items():
                if value is not None:
                    self._systems_data[key] = value
            self.last_timestamp = iso_utc_ms()

    async def update_autopilot_partial(self, **kwargs):
        async with self._lock:
            for key, value in kwargs.items():
                if value is not None:
                    self._autopilot_data[key] = value
            self.last_timestamp = iso_utc_ms()

    async def update_levers_partial(self, **kwargs):
        async with self._lock:
            for key, value in kwargs.items():
                if value is not None:
                    self._levers_data[key] = value
            self.last_timestamp = iso_utc_ms()

    async def update_indicators_partial(self, **kwargs):
        async with self._lock:
            for key, value in kwargs.items():
                if value is not None:
                    self._indicators_data[key] = value
            self.last_timestamp = iso_utc_ms()

    async def get_snapshot(self) -> Dict[str, Any]:
        async with self._lock:
            pos = {}
            att = {}

            if self.xgps:
                if self.xgps.latitude  is not None:  pos["latitudeDeg"]  = round(clamp(self.xgps.latitude,  -90.0,  90.0), 6)
                if self.xgps.longitude is not None:  pos["longitudeDeg"] = round(clamp(self.xgps.longitude, -180.0, 180.0), 6)
                if self.xgps.alt_msl_meters is not None:
                    pos["mslAltitudeFt"] = self.xgps.alt_msl_meters * METERS_TO_FEET
                if self.xgps.ground_speed_kts is not None:
                    pos["gpsGroundSpeedKts"] = self.xgps.ground_speed_kts

            # Direct IAS if available
            if self._ias_kts is not None:
                pos["indicatedAirspeedKts"] = round(self._ias_kts, 1)

            # VS: prioritize raw VS; if not available, use derived VS
            if self._vs_fpm_raw is not None:
                pos["verticalSpeedUpFpm"] = round(self._vs_fpm_raw, 0)
            elif self._vs_fpm is not None:
                pos["verticalSpeedUpFpm"] = round(self._vs_fpm, 0)

            # AGL if we have MSL altitude and ground altitude
            if self.xgps and self.xgps.alt_msl_meters is not None and self._ground_alt_m is not None:
                agl_ft = (self.xgps.alt_msl_meters - self._ground_alt_m) * METERS_TO_FEET
                pos["aglAltitudeFt"] = max(0.0, round(agl_ft, 1))

            if self.xatt:
                att["trueHeadingDeg"]    = self._norm360(self.xatt.heading_deg)
                att["pitchAngleDegUp"]   = self._nz(self.xatt.pitch_deg)
                att["rollAngleDegRight"] = self._nz(self.xatt.roll_deg)

                # Magnetic heading if we have magnetic variation
                if "trueHeadingDeg" in att and self._mag_var_deg is not None:
                    mag = (att["trueHeadingDeg"] - self._mag_var_deg) % 360.0
                    att["magneticHeadingDeg"] = mag

                # Ground track (derived from position changes)
                if self._track_deg is not None:
                    att["trueGroundTrackDeg"] = self._norm360(self._track_deg)

            # New data groups
            lights = {}
            systems = {}
            autopilot = {}
            levers = {}
            indicators = {}

            # Build lights group
            for sink_key, shirley_key in _LIGHTS_SINK_TO_SHIRLEY.items():
                if sink_key in self._lights_data:
                    # Parse nested keys like "lights.navigationLightsSwitchOn"
                    parts = shirley_key.split('.')
                    if len(parts) == 2 and parts[0] == "lights":
                        lights[parts[1]] = bool(self._lights_data[sink_key])

            # Build systems group
            for sink_key, shirley_key in _SYSTEMS_SINK_TO_SHIRLEY.items():
                if sink_key in self._systems_data:
                    parts = shirley_key.split('.')
                    if len(parts) == 2 and parts[0] == "systems":
                        systems[parts[1]] = bool(self._systems_data[sink_key])
                    elif len(parts) == 3 and parts[0] == "systems":  # batteryOn.main
                        if parts[1] not in systems:
                            systems[parts[1]] = {}
                        systems[parts[1]][parts[2]] = bool(self._systems_data[sink_key])

            # Build autopilot group
            for sink_key, shirley_key in _AUTOPILOT_SINK_TO_SHIRLEY.items():
                if sink_key in self._autopilot_data:
                    parts = shirley_key.split('.')
                    if len(parts) == 2 and parts[0] == "autopilot":
                        value = self._autopilot_data[sink_key]
                        if "deg" in parts[1] or "ft" in parts[1] or "fpm" in parts[1]:
                            autopilot[parts[1]] = float(value)
                        else:
                            autopilot[parts[1]] = bool(value)

            # Build levers group
            for sink_key, shirley_key in _LEVERS_SINK_TO_SHIRLEY.items():
                if sink_key in self._levers_data:
                    parts = shirley_key.split('.')
                    if len(parts) == 2 and parts[0] == "levers":
                        levers[parts[1]] = float(self._levers_data[sink_key])
                    elif len(parts) == 3 and parts[0] == "levers":  # throttlePercentOpen.engine1
                        if parts[1] not in levers:
                            levers[parts[1]] = {}
                        levers[parts[1]][parts[2]] = float(self._levers_data[sink_key])

            # Build indicators group
            for sink_key, shirley_key in _INDICATORS_SINK_TO_SHIRLEY.items():
                if sink_key in self._indicators_data:
                    parts = shirley_key.split('.')
                    if len(parts) == 2 and parts[0] == "indicators":
                        value = self._indicators_data[sink_key]
                        if "warning" in parts[1].lower() or "on" in parts[1].lower():
                            indicators[parts[1]] = bool(value)
                        else:
                            indicators[parts[1]] = float(value)

            # Add non-empty groups to output
            if lights: out["lights"] = lights
            if systems: out["systems"] = systems
            if autopilot: out["autopilot"] = autopilot
            if levers: out["levers"] = levers
            if indicators: out["indicators"] = indicators

            # Validar datos críticos antes de enviar
            if pos.get("latitudeDeg") is not None:
                if not validate_position_data(pos.get("latitudeDeg"), pos.get("longitudeDeg"), pos.get("mslAltitudeFt")):
                    print(f"[WARNING] Invalid position data detected: lat={pos.get('latitudeDeg')}, lon={pos.get('longitudeDeg')}")

            out = {}
            if pos: out["position"] = pos
            if att: out["attitude"] = att
            return out

    def _bearing_deg(self, lat1, lon1, lat2, lon2):
        """Calculate true bearing between two lat/lon points (great circle)"""
        import math
        try:
            φ1, φ2 = math.radians(lat1), math.radians(lat2)
            Δλ = math.radians(lon2 - lon1)

            y = math.sin(Δλ) * math.cos(φ2)
            x = math.cos(φ1) * math.sin(φ2) - math.sin(φ1) * math.cos(φ2) * math.cos(Δλ)

            brng = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
            return brng
        except (ValueError, ZeroDivisionError):
            return None

    # Auxiliary functions for normalization
    def _norm360(self, x):
        """Normalize angle to range [0, 360)"""
        if x is None:
            return None
        return (x % 360.0 + 360.0) % 360.0

    def _nz(self, x, eps=1e-6):
        """Avoid values close to zero that become '-0'"""
        if x is None:
            return None
        return 0.0 if abs(x) < eps else x

# ===================== Cliente FSUIPC WebSocket =====================
class FSUIPCWSClient:
    """
    WebSocket client to FSUIPC WebSocket Server.
    Subscribes to offsets and feeds SimData with XGPS/XATT "synthetic" data.
    """
    def __init__(self, sim_data: SimData, url: str = FSUIPC_WS_URL):
        self.sim_data = sim_data
        self.url = url
        self.ws: Optional[Any] = None  # WebSocket client connection
        self.lastDataReceivedTime: Optional[float] = None  # useful for UI/logging

    async def run(self):
        while True:
            try:
                print(f"[FSUIPCWS] Connecting {self.url} …")
                async with websockets.connect(
                    self.url,
                    max_size=None,
                    subprotocols=["fsuipc"],
                    open_timeout=4,
                    ping_interval=None
                ) as ws:
                    self.ws = ws
                    print(f"[FSUIPCWS] Connected {self.url} (subprotocol={ws.subprotocol})")

                    # Build dynamic declare from READ_SIGNALS
                    declare_msg = {
                        "command": "offsets.declare",
                        "name": "flightData",
                        "offsets": [
                            {"name": key, "address": cfg["address"], "type": cfg["type"], "size": cfg["size"]}
                            for key, cfg in READ_SIGNALS.items()
                        ],
                    }
                    await ws.send(json.dumps(declare_msg))
                    print("[FSUIPCWS] Offsets declared")

                    # Start continuous reading from FSUIPC with fixed interval (ms)
                    read_msg = {
                        "command": "offsets.read",
                        "name": "flightData",
                        "interval": int(SEND_INTERVAL * 1000)  # 250 ms if SEND_INTERVAL=0.25
                    }
                    await ws.send(json.dumps(read_msg))
                    print(f"[FSUIPCWS] Started reading offsets every {int(SEND_INTERVAL*1000)} ms")

                    async for msg in ws:
                        # Convert bytes to str if necessary
                        if isinstance(msg, bytes):
                            try:
                                msg = msg.decode('utf-8', 'ignore')
                            except Exception:
                                continue
                        if isinstance(msg, str):
                            self._handle_incoming(msg)

            except Exception as e:
                print(f"[FSUIPCWS] Error: {e!r}. Reconnecting in 2s…")
                await asyncio.sleep(2)

    def _handle_incoming(self, msg: str):
        global FIRST_PAYLOAD
        try:
            data = json.loads(msg)
        except json.JSONDecodeError:
            return

        # Debug log
        if DEBUG_FSUIPC_MESSAGES or FIRST_PAYLOAD:
            print(f"[FSUIPCWS] Received: {data}")
            FIRST_PAYLOAD = False

        # before detecting payload:
        if "command" in data and "success" in data and not any(k in data for k in ("data","values","offsets")):
            if not data.get("success"):
                print(f"[FSUIPCWS] Command error: {data.get('errorMessage')}")
            return

        # Generic parser using table and partial updates
        payload = data.get("data") or data.get("values") or data

        # Log data quality
        if isinstance(payload, dict) and payload:
            valid_keys = [k for k, v in payload.items() if v is not None]
            if len(valid_keys) > 0:
                print(f"[FSUIPCWS] Valid data fields: {len(valid_keys)}/{len(payload)}")

        # some builds return 'values' as a list of {name, value}
        if isinstance(payload, list):
            try:
                payload = {it["name"]: it.get("value") for it in payload if isinstance(it, dict) and "name" in it}
            except Exception:
                payload = {}

        if not isinstance(payload, dict):
            return

        gps_kwargs = {}   # latitude, longitude, alt_msl_meters, track_deg, ground_speed_kts
        att_kwargs = {}   # heading_deg, pitch_deg, roll_deg
        lights_kwargs = {} # nav_on, beacon_on, landing_on, taxi_on, strobe_on

        for key, cfg in READ_SIGNALS.items():
            if key not in payload:
                continue
            val = payload[key]
            # apply transform if exists
            tf = cfg.get("transform")
            if tf:
                val = TRANSFORMS[tf](val)
            # dispatch to SimData according to 'sink' (prefer the first one that arrives)
            sink_group, sink_field = cfg["sink"]
            if sink_group == "gps":
                if val is not None and sink_field not in gps_kwargs:
                    gps_kwargs[sink_field] = val
            elif sink_group == "att":
                if val is not None and sink_field not in att_kwargs:
                    att_kwargs[sink_field] = val
            elif sink_group == "lights":
                if val is not None and sink_field not in lights_kwargs:
                    lights_kwargs[sink_field] = val

        # Partial updates (already exist in SimData)
        if gps_kwargs:
            asyncio.create_task(self.sim_data.update_gps_partial(**gps_kwargs))
        if att_kwargs:
            asyncio.create_task(self.sim_data.update_att_partial(**att_kwargs))
        if lights_kwargs:
            asyncio.create_task(self.sim_data.update_lights_partial(**lights_kwargs))

        self.lastDataReceivedTime = time.time()

    async def write_offset(self, address: int, value: int, *, size: int, dtype: str = "int") -> bool:
        if not self.ws:
            print("[FSUIPCWS] Write failed: not connected")
            return False
        msg = {
            "command": "offsets.write",
            "values": [
                {"address": address, "type": dtype, "size": size, "value": int(value)}
            ]
        }
        try:
            await self.ws.send(json.dumps(msg))
            # Optional: wait for ACK if your server responds with success
            return True
        except Exception as e:
            print(f"[FSUIPCWS] Write error: {e!r}")
            return False

# ===================== WebSocket Server for Shirley =====================
class ShirleyWebSocketServer:
    """
    - Accepts clients (including Shirley) at ws://host:port/api/v1
    - Broadcasts SimData snapshot every SEND_INTERVAL (4 Hz)
    - Receives SetSimData and forwards to FSUIPC (gear/throttle MVP)
    """
    def __init__(self, sim_data: SimData, fsuipc: FSUIPCWSClient,
                 host=WS_HOST, port=WS_PORT, path=WS_PATH, send_interval=SEND_INTERVAL):
        self.sim_data = sim_data
        self.fsuipc = fsuipc
        self.host = host
        self.port = port
        self.path = path
        self.send_interval = send_interval
        self.connections: Set[Any] = set()  # WebSocket server connections
        self.server = None

    async def handler(self, websocket, path=None):
        client_info = getattr(websocket, "remote_address", "Unknown")
        request_path = path if path is not None else getattr(websocket, "path", "/")
        print(f"[ShirleyWS] Client connected {client_info} path={request_path}")

        # --- Allow both /api/v1 and / (and variations with/without slash) ---
        def _norm(p: str) -> str:
            return (p or "/").rstrip("/") or "/"

        wanted = _norm(self.path)           # e.g. "/api/v1"
        got    = _norm(request_path)        # e.g. "/"

        allowed = {wanted, "/"}             # accepts "/api/v1" and "/"
        if self.path and got not in allowed:
            try:
                await websocket.close(code=1008, reason="Invalid path")
            except Exception:
                pass
            print(f"[ShirleyWS] Rejected client {client_info} wrong path={request_path}")
            return

        self.connections.add(websocket)

        # Send capabilities on connection (dynamic)
        capabilities = {
            "type": "Capabilities",
            "reads": compute_capabilities_reads(),
            "writes": compute_capabilities_writes()
        }
        try:
            await websocket.send(json.dumps(capabilities))
        except websockets.exceptions.ConnectionClosed:
            pass

        try:
            async for raw in websocket:
                # Wait for SetSimData messages (single or array of commands)
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    continue

                if data.get("type") == "SetSimData":
                    results = []
                    commands = data.get("commands", [])
                    if not isinstance(commands, list):
                        # Legacy format: single command
                        commands = [{"name": data.get("control"), "value": data.get("value")}]

                    for cmd in commands:
                        if isinstance(cmd, dict):
                            ok = await self._handle_command(cmd)
                            results.append({"name": cmd.get("name"), "ok": ok})

                    ack = {"type": "SetSimDataAck", "results": results}
                    try:
                        await websocket.send(json.dumps(ack))
                    except websockets.exceptions.ConnectionClosed:
                        pass
                    continue

        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            if websocket in self.connections:
                self.connections.remove(websocket)
            print(f"[ShirleyWS] Client disconnected {client_info}")

    async def _handle_command(self, cmd: dict) -> bool:
        name = (cmd.get("name") or cmd.get("control") or "").strip()
        value = cmd.get("value", 0)
        spec = WRITE_COMMANDS.get(name)
        if not spec:
            print(f"[CMD] Unknown command: {cmd}")
            return False
        try:
            raw = spec["encode"](value) if callable(spec.get("encode")) else value
            if spec["type"] == "offset":
                ok = await self.fsuipc.write_offset(spec["address"], int(raw), size=spec["size"], dtype=spec["dtype"])
                print(f"[CMD] {name} -> {value} (raw={raw}) ok={ok}")
                return ok
            print(f"[CMD] Unsupported write type for {name}: {spec['type']}")
            return False
        except Exception as e:
            print(f"[CMD] Error handling {cmd}: {e!r}")
            return False

    async def broadcast_loop(self):
        try:
            while True:
                snapshot = await self.sim_data.get_snapshot()
                msg = json.dumps(snapshot)
                stale = []
                for ws in list(self.connections):
                    try:
                        await ws.send(msg)
                    except websockets.exceptions.ConnectionClosed:
                        stale.append(ws)
                    except Exception as e:
                        print(f"[ShirleyWS] Send error: {e}")
                        stale.append(ws)
                for ws in stale:
                    if ws in self.connections:
                        self.connections.remove(ws)
                await asyncio.sleep(self.send_interval)
        except asyncio.CancelledError:
            print("[ShirleyWS] Broadcast stopped")

    async def run(self):
        # Start server and broadcast loop
        self.server = await websockets.serve(self.handler, self.host, self.port)
        print(f"[ShirleyWS] Serving at ws://{self.host}:{self.port}{self.path}")
        broadcast_task = asyncio.create_task(self.broadcast_loop())

        try:
            # Keep the server running indefinitely
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            print("[ShirleyWS] Server stopping...")
        finally:
            broadcast_task.cancel()
            if self.server:
                self.server.close()
                await self.server.wait_closed()

# ===================== Orchestrator =====================
async def main():
    sim_data = SimData()
    fsuipc = FSUIPCWSClient(sim_data, url=FSUIPC_WS_URL)
    shirley_ws = ShirleyWebSocketServer(sim_data, fsuipc)

    await asyncio.gather(
        fsuipc.run(),        # downstream (FSUIPC)
        shirley_ws.run(),    # upstream (Shirley)
    )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nBridge shutting down.")