import asyncio
import json
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any, Set

import websockets
import websockets.exceptions

# ===================== CONFIGURATION =====================
FSUIPC_WS_URL = "ws://localhost:2048/fsuipc/"  # FSUIPC WebSocket Server (Paul Henty)
WS_HOST = "localhost"
WS_PORT = 2992
WS_PATH = "/api/v1"                     # for logging only
SEND_INTERVAL = 0.25                    # 4 Hz (every 250 ms)
DEBUG_FSUIPC_MESSAGES = False
# Set to True to enable detailed debugging of FSUIPC messages, JSON output, and broadcast info
FIRST_PAYLOAD = False

# ===================== FSUIPC CONSTANTS =====================
# Conversion factors
METERS_TO_FEET = 3.28084
MPS_TO_KTS = 1.943844

# FSUIPC scaling factors
FSUIPC_SCALE_FACTOR_65536 = 65536.0
FSUIPC_SCALE_FACTOR_16383 = 16383
FSUIPC_SCALE_FACTOR_32768 = 32768
FSUIPC_SCALE_FACTOR_256 = 256.0
FSUIPC_SCALE_FACTOR_128 = 128.0
FSUIPC_SCALE_FACTOR_16 = 16.0

# Angular conversion factors
FSUIPC_TURN_FRACTION_TO_DEG = 360.0
FSUIPC_LAT_SCALE = 10001750.0 * 65536.0 * 65536.0
FSUIPC_LON_SCALE = 65536.0 * 65536.0 * 65536.0 * 65536.0

# Thresholds
BRAKE_PEDAL_THRESHOLD = 200
PARKING_BRAKE_THRESHOLD = 1000
ZERO_THRESHOLD_EPSILON = 1e-6
POSITION_CHANGE_EPSILON = 1e-7

# Barometric pressure validation ranges (raw values)
BARO_RAW_MIN = 12800  # ~800 mb
BARO_RAW_MAX = 17600  # ~1100 mb

# Time and frequency constants
MILLISECONDS_PER_SECOND = 1000
SECONDS_PER_MINUTE = 60.0

# Pressure conversion constants
MB_TO_INHG_FACTOR = 0.02953  # millibar to inches of mercury conversion factor

# FSUIPC bit masks
FSUIPC_SIGNED_16BIT_MASK = 0xFFFF
FSUIPC_SIGNED_16BIT_OFFSET = 0x10000

# Throttle max value
FSUIPC_THROTTLE_MAX = 16384

# --- Config de frenado ---
USE_BRAKES_ON_INCLUDES_PARKING = True  # True: brakesOn = pedales OR parking

# ===================== Declarative writes =====================
WRITE_COMMANDS = {
    "GEAR_HANDLE": {  # 0=retracted, 1=down
        "type": "offset",
        "address": 0x0BE8, "size": 4, "dtype": "int",
        "encode": lambda v: FSUIPC_SCALE_FACTOR_16383 if int(float(v)) else 0,
    },
    "throttle": {     # accepts -1..1 or raw value [-16384..16384]
        "type": "offset",
        "address": 0x088C, "size": 2, "dtype": "short",
        "encode": lambda v: (
            max(0, min(FSUIPC_THROTTLE_MAX, round((float(v)+1.0)*0.5*FSUIPC_THROTTLE_MAX))) if -1.0 <= float(v) <= 1.0
            else max(-FSUIPC_THROTTLE_MAX, min(FSUIPC_THROTTLE_MAX, int(float(v))))
        ),
    },
}

# ===================== CAPABILITIES FUNCTIONS =====================
def compute_capabilities_writes():
    """
    Get list of available write commands for capabilities reporting.

    Returns:
        Sorted list of command names that can be written to the simulator

    Example:
        >>> commands = compute_capabilities_writes()
        >>> len(commands) > 0
        True
        >>> 'GEAR_HANDLE' in commands
        True
    """
    return sorted(WRITE_COMMANDS.keys())

def compute_capabilities_reads():
    """
    Get list of available read signals for capabilities reporting.

    Returns:
        List of dictionaries containing read capabilities with format:
        [{"key": signal_name, "group": data_group, "field": field_name}, ...]

    Example:
        >>> reads = compute_capabilities_reads()
        >>> len(reads) > 0
        True
        >>> all('key' in r and 'group' in r and 'field' in r for r in reads)
        True
    """
    reads = []
    for _, cfg in READ_SIGNALS.items():
        sink = cfg.get("sink")
        if not sink:
            continue
        if isinstance(sink, tuple) and len(sink) == 2:
            g, f = sink
            reads.append({"key": cfg.get("name",""), "group": g, "field": f})
    return reads

# ===================== UTILITY FUNCTIONS =====================

def clamp(v, lo, hi):
    """
    Clamp a value between minimum and maximum bounds.

    Args:
        v: Value to clamp
        lo: Lower bound (inclusive)
        hi: Upper bound (inclusive)

    Returns:
        Value clamped to [lo, hi] range

    Example:
        >>> clamp(15, 0, 10)
        10
        >>> clamp(-5, 0, 10)
        0
    """
    return max(lo, min(hi, v))

def iso_utc_ms() -> str:
    """
    Generate ISO 8601 UTC timestamp with millisecond precision.

    Returns:
        ISO 8601 formatted timestamp string (e.g., "2023-12-25T10:30:45.123Z")

    Example:
        >>> result = iso_utc_ms()
        >>> len(result) >= 20  # Minimum length for ISO format
        True
        >>> result.endswith('Z')  # Should end with Z for UTC
        True
    """
    t = time.time()
    whole = int(t)
    ms = int((t - whole) * MILLISECONDS_PER_SECOND)
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(whole)) + f".{ms:03d}Z"

# ===================== FSUIPC RAW DATA CONVERSIONS =====================
def fs_lat_to_deg(raw: int) -> float:
    """
    Convert FSUIPC 64-bit latitude units to degrees.

    Args:
        raw: Raw 64-bit latitude value from FSUIPC

    Returns:
        Latitude in decimal degrees (-90 to +90)

    Example:
        >>> fs_lat_to_deg(0)
        0.0
        >>> abs(fs_lat_to_deg(2**63)) <= 90  # Max value should be <= 90
        True
    """
    return (raw * 90.0) / FSUIPC_LAT_SCALE

def fs_lon_to_deg(raw: int) -> float:
    """
    Convert FSUIPC 64-bit longitude units to degrees.

    Args:
        raw: Raw 64-bit longitude value from FSUIPC

    Returns:
        Longitude in decimal degrees (-180 to +180)

    Example:
        >>> fs_lon_to_deg(0)
        0.0
        >>> abs(fs_lon_to_deg(2**63)) <= 180  # Max value should be <= 180
        True
    """
    return (raw * FSUIPC_TURN_FRACTION_TO_DEG) / FSUIPC_LON_SCALE

def fs_alt_to_m(raw: int) -> float:
    # meters * 65536 -> meters
    return raw / FSUIPC_SCALE_FACTOR_65536

def fs_heading_true_deg(raw: int) -> float:
    """
    Convert FSUIPC raw heading units to true heading in degrees.

    Args:
        raw: Raw heading value from FSUIPC (fraction of full turn)

    Returns:
        True heading in degrees (0-360)

    Example:
        >>> fs_heading_true_deg(0)
        0.0
        >>> 0 <= fs_heading_true_deg(2**32//4) <= 360  # Quarter turn = 90 degrees
        True
    """
    return (raw * FSUIPC_TURN_FRACTION_TO_DEG) / (FSUIPC_SCALE_FACTOR_65536 * FSUIPC_SCALE_FACTOR_65536)

def fs_ground_speed_mps(raw: int) -> float:
    # 65536 * m/s -> m/s
    return raw / FSUIPC_SCALE_FACTOR_65536

def fs_angle_deg(raw: int) -> float:
    # For pitch/bank (same factor as heading)
    return (raw * FSUIPC_TURN_FRACTION_TO_DEG) / (FSUIPC_SCALE_FACTOR_65536 * FSUIPC_SCALE_FACTOR_65536)


# ===================== FSUIPC SIGNAL DEFINITIONS =====================
READ_SIGNALS = {
    # --- Position ---
    "LatitudeDeg":   {"address": 0x0560, "type": "lat",   "size": 8, "sink": ("gps", "latitude")},      # deg
    "LongitudeDeg":  {"address": 0x0568, "type": "lon",   "size": 8, "sink": ("gps", "longitude")},     # deg
    "AltitudeM":     {"address": 0x6020, "type": "float", "size": 8, "sink": ("gps", "alt_msl_meters")},# m

    "GroundSpeedKts":{"address": 0x02B4, "type": "uint",  "size": 4, "transform": "gs_u32_to_kts", "sink": ("gps", "ground_speed_kts")},

    # --- Airspeeds / VS ---
    "IASraw_U32":   {"address": 0x02BC, "type": "uint", "size": 4, "transform": "knots128_to_kts", "sink": ("gps", "ias_kts")},
    "VSraw":        {"address": 0x02C8, "type": "int",  "size": 4, "transform": "vs_raw_to_fpm",   "sink": ("gps", "vs_fpm_raw")},

    # --- AGL via ground altitude ---
    "GroundAltRaw": {"address": 0x0020, "type": "int", "size": 4, "transform": "meters256_to_m", "sink": ("gps", "ground_alt_m")},

    # --- Attitude ---
    "HeadingTrueRaw":{"address": 0x0580, "type": "uint",  "size": 4, "transform": "raw_hdg_to_deg", "sink": ("att", "heading_deg")},
    "PitchRaw":      {"address": 0x0578, "type": "int",   "size": 4, "transform": "raw_ang_to_deg_pitch", "sink": ("att", "pitch_deg")},
    "BankRaw":       {"address": 0x057C, "type": "int",   "size": 4, "transform": "raw_ang_to_deg", "sink": ("att", "roll_deg")},

    # --- Magnetic variation (U32 variant) ---
    "MagVar_U32": {"address": 0x02A0, "type": "uint", "size": 4, "transform": "u32_signed16_to_magdeg", "sink": ("att", "mag_var_deg")},

    # --- Luces (U32 bitmask confirmed working) ---
    "LIGHTS_BITS32": {"address": 0x0D0C, "type": "uint", "size": 4, "sink": ("lights", "bitmask")},

    # --- Sistemas (U32 variants confirmed) ---
    "BATTERY_MAIN":   {"address": 0x281C, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("systems", "battery_main_on")},
    "PITOT_HEAT_U32": {"address": 0x029C, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("systems", "pitot_heat_on")},

    # --- BARO (G1000 → primario, U32 confirmed) ---
    "BARO_0332_U32": {"address": 0x0332, "type": "uint", "size": 4, "transform": "u32_baro_to_inhg", "sink": ("environment", "baro_0332")},
    "BARO_0330_U32": {"address": 0x0330, "type": "uint", "size": 4, "transform": "u32_baro_to_inhg", "sink": ("environment", "baro_0330")},

    # --- Frenos/parking (U32 confirmed) ---
    "brakeLeftU":    {"address": 0x0BC4, "type": "uint", "size": 4, "sink": None},
    "brakeRightU":   {"address": 0x0BC6, "type": "uint", "size": 4, "sink": None},
    "parkingBrakeU": {"address": 0x0BC8, "type": "uint", "size": 4, "sink": None},

    # --- Controles (para flaps/gear percentage) ---
    "flapsHandle":   {"address": 0x0BDC, "type": "uint", "size": 4, "transform": "u32_to_pct_16383", "sink": ("levers", "flaps_pct")},
    "gearHandle":    {"address": 0x0BE8, "type": "uint", "size": 4, "transform": "u32_to_pct_16383", "sink": ("levers", "gear_pct")},

    # --- Nombre aeronave ---
    "aircraftNameStr": {"address": 0x3D00, "type": "string", "size": 256, "sink": ("aircraft", "name")},
}

# Normaliza: si alguna señal no define 'sink', déjalo en None
for _k, _cfg in READ_SIGNALS.items():
    _cfg.setdefault("sink", None)

# ===================== DATA TRANSFORM FUNCTIONS =====================
def raw_ang_to_deg(raw):
    return fs_angle_deg(raw) if raw is not None else None

def raw_ang_to_deg_pitch(raw):
    v = fs_angle_deg(raw) if raw is not None else None
    return -v if v is not None else None  # 👈 we invert sign for positive 'Up'
def raw_hdg_to_deg(raw):    return (fs_heading_true_deg(raw) % 360.0) if raw is not None else None
def mps_to_mps(raw):        return fs_ground_speed_mps(raw) if raw is not None else None

# ===================== TRANSFORM REGISTRY =====================

TRANSFORMS = {
    "raw_ang_to_deg": raw_ang_to_deg,
    "raw_ang_to_deg_pitch": raw_ang_to_deg_pitch,
    "raw_hdg_to_deg": raw_hdg_to_deg,
    "mps_to_mps":     mps_to_mps,
}

# --- New transforms ---
def knots128_to_kts(raw):
    try: return float(raw) / FSUIPC_SCALE_FACTOR_128
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] knots128_to_kts failed for {raw}: {e}")
        return None

def vs_raw_to_fpm(raw):
    # raw = 256 * m/s  ->  ft/min
    try: return float(raw) * SECONDS_PER_MINUTE * METERS_TO_FEET / FSUIPC_SCALE_FACTOR_256
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] vs_raw_to_fpm failed for {raw}: {e}")
        return None

def meters256_to_m(raw):
    # ground altitude in meters *256
    try: return float(raw) / FSUIPC_SCALE_FACTOR_256
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] meters256_to_m failed for {raw}: {e}")
        return None

def magvar_raw_to_deg(raw):
    # 0x02A0: signed word; deg = raw * 360 / 65536, East positive (-ve = West in old docs)
    try:
        # interpret as int16
        if isinstance(raw, str) and raw.startswith("0x"):
            val = int(raw, 16)
            if val >= 0x8000: val -= FSUIPC_SIGNED_16BIT_OFFSET
        else:
            val = int(raw)
            if val >= FSUIPC_SCALE_FACTOR_32768: val -= FSUIPC_SCALE_FACTOR_65536
        return (val * FSUIPC_TURN_FRACTION_TO_DEG) / FSUIPC_SCALE_FACTOR_65536
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] magvar_raw_to_deg failed for {raw}: {e}")
        return None

def bits_to_bool_0(raw):
    """Extract bit 0 from FSUIPC bits object"""
    try:
        if isinstance(raw, dict) and '0' in raw:
            return bool(raw['0'])
        return None
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] bits_to_bool_0 failed for {raw}: {e}")
        return None

def bits_to_bool_1(raw):
    """Extract bit 1 from FSUIPC bits object"""
    try:
        if isinstance(raw, dict) and '1' in raw:
            return bool(raw['1'])
        return None
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] bits_to_bool_1 failed for {raw}: {e}")
        return None

def bits_to_bool_2(raw):
    """Extract bit 2 from FSUIPC bits object"""
    try:
        if isinstance(raw, dict) and '2' in raw:
            return bool(raw['2'])
        return None
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] bits_to_bool_2 failed for {raw}: {e}")
        return None

def bits_to_bool_3(raw):
    """Extract bit 3 from FSUIPC bits object"""
    try:
        if isinstance(raw, dict) and '3' in raw:
            return bool(raw['3'])
        return None
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] bits_to_bool_3 failed for {raw}: {e}")
        return None

def bits_to_bool_4(raw):
    """Extract bit 4 from FSUIPC bits object"""
    try:
        if isinstance(raw, dict) and '4' in raw:
            return bool(raw['4'])
        return None
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] bits_to_bool_4 failed for {raw}: {e}")
        return None

def nonzero_to_bool(raw):
    """Convert non-zero values to True, zero to False"""
    try: return bool(int(raw))
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] nonzero_to_bool failed for {raw}: {e}")
        return None



def baro_to_inhg(raw):
    """Convert barometric pressure from millibars*16 to inches of mercury"""
    try:
        mb = float(raw) / FSUIPC_SCALE_FACTOR_16  # Convert to millibars
        return mb * MB_TO_INHG_FACTOR     # Convert mb to inHg
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] baro_to_inhg failed for {raw}: {e}")
        return None

# === U32 → lower16 helpers (from probe findings) ===
def lower16(u):
    try: return int(u) & FSUIPC_SIGNED_16BIT_MASK
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] lower16 failed for {u}: {e}")
        return None

def u32_baro_to_inhg(u):
    v = lower16(u)
    if v is None: return None
    mb = v / FSUIPC_SCALE_FACTOR_16
    return mb * MB_TO_INHG_FACTOR  # 16212→1013.25mb→29.92 inHg

def u32_to_pct_16383(u):
    v = lower16(u)
    if v is None: return None
    return max(0.0, min(100.0, (v / FSUIPC_SCALE_FACTOR_16383) * 100.0))

def u32_to_bool_parking(u):
    v = lower16(u)
    if v is None: return None
    return v >= PARKING_BRAKE_THRESHOLD   # tolerante (0/32767 típico)

def u32_signed16_to_magdeg(u):
    v = lower16(u)
    if v is None: return None
    if v >= FSUIPC_SCALE_FACTOR_32768: v -= FSUIPC_SCALE_FACTOR_65536
    return (v * FSUIPC_TURN_FRACTION_TO_DEG) / FSUIPC_SCALE_FACTOR_65536

def gs_u32_to_kts(raw):
    try:
        # 0x02B4 = ground speed en (m/s) * 65536
        return (float(raw) / FSUIPC_SCALE_FACTOR_65536) * MPS_TO_KTS  # m/s → kts
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            print(f"[TRANSFORM] gs_u32_to_kts failed for {raw}: {e}")
        return None

TRANSFORMS.update({
    "knots128_to_kts": knots128_to_kts,
    "vs_raw_to_fpm":   vs_raw_to_fpm,
    "meters256_to_m":  meters256_to_m,
    "magvar_raw_to_deg": magvar_raw_to_deg,
    # Bitfield transforms for lights (updated for bits object processing)
    "bits_to_bool_0": bits_to_bool_0,
    "bits_to_bool_1": bits_to_bool_1,
    "bits_to_bool_2": bits_to_bool_2,
    "bits_to_bool_3": bits_to_bool_3,
    "bits_to_bool_4": bits_to_bool_4,
    # Boolean transforms for systems
    "nonzero_to_bool": nonzero_to_bool,
    # Weather transforms for environment
    "baro_to_inhg": baro_to_inhg,
    # U32 transforms (from probe findings)
    "lower16": lower16,
    "u32_baro_to_inhg": u32_baro_to_inhg,
    "u32_to_pct_16383": u32_to_pct_16383,
    "u32_to_bool_parking": u32_to_bool_parking,
    "u32_signed16_to_magdeg": u32_signed16_to_magdeg,
    "gs_u32_to_kts": gs_u32_to_kts
})

# ===================== SINK TO SHIRLEY MAPPINGS =====================
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
    "landing_on": "lights.landingLightsSwitchOn",
    "taxi_on": "lights.taxiLightsSwitchOn",
    "strobe_on": "lights.strobeLightsSwitchOn",
}

_SYSTEMS_SINK_TO_SHIRLEY = {
    "pitot_heat_on": "systems.pitotHeatSwitchOn",
    # "brakes_on": "systems.brakesOn",
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

_ENVIRONMENT_SINK_TO_SHIRLEY = {
    "pressure_inhg": "environment.seaLevelPressureInchesMercury",
}

_SIMULATION_SINK_TO_SHIRLEY = {
    "aircraft_name": "simulation.aircraftName",
}

# ===================== DATA MODEL CLASSES =====================
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

# ===================== SIMDATA CLASS =====================
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
        self._environment_data = {} # pressure_inhg (only working field in MSFS)

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
                    dt_min = max(ZERO_THRESHOLD_EPSILON, (now - self._last_vs_ts) / SECONDS_PER_MINUTE)
                    self._vs_fpm = (alt_ft - self._last_alt_ft) / dt_min
                self._last_alt_ft = alt_ft
                self._last_vs_ts = now

            # Calculate ground track from position changes
            if self.xgps and self.xgps.latitude is not None and self.xgps.longitude is not None:
                lat, lon = self.xgps.latitude, self.xgps.longitude

                # Only calculate if we have previous position and position actually changed
                if (self._last_lat is not None and self._last_lon is not None and
                    (abs(lat - self._last_lat) > POSITION_CHANGE_EPSILON or abs(lon - self._last_lon) > POSITION_CHANGE_EPSILON)):
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

    async def update_environment_partial(self, **kwargs):
        async with self._lock:
            for key, value in kwargs.items():
                if value is not None:
                    self._environment_data[key] = value
        self.last_timestamp = iso_utc_ms()

    async def get_snapshot(self) -> Dict[str, Any]:
        async with self._lock:
            pos = {}
            att = {}
            out = {}

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

            # DEBUG: Check pos and att construction
            if DEBUG_FSUIPC_MESSAGES:
                print(f"[DEBUG] pos dict: {pos}")
                print(f"[DEBUG] att dict: {att}")
                print(f"[DEBUG] self.xgps exists: {self.xgps is not None}")
                print(f"[DEBUG] self.xatt exists: {self.xatt is not None}")
                if self.xgps:
                    print(f"[DEBUG] xgps latitude: {self.xgps.latitude}")
                    print(f"[DEBUG] xgps longitude: {self.xgps.longitude}")
                    print(f"[DEBUG] xgps alt_msl_meters: {self.xgps.alt_msl_meters}")
                    print(f"[DEBUG] xgps ground_speed_kts: {self.xgps.ground_speed_kts}")
                if self.xatt:
                    print(f"[DEBUG] xatt heading_deg: {self.xatt.heading_deg}")
                    print(f"[DEBUG] xatt pitch_deg: {self.xatt.pitch_deg}")
                    print(f"[DEBUG] xatt roll_deg: {self.xatt.roll_deg}")

            # New data groups
            lights = {}
            systems = {}
            autopilot = {}
            levers = {}
            indicators = {}
            environment = {}

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

            # Build environment group
            for sink_key, shirley_key in _ENVIRONMENT_SINK_TO_SHIRLEY.items():
                if sink_key in self._environment_data:
                    parts = shirley_key.split('.')
                    if len(parts) == 2 and parts[0] == "environment":
                        environment[parts[1]] = float(self._environment_data[sink_key])

            # Build simulation group
            simulation = {}
            for sink_key, shirley_key in _SIMULATION_SINK_TO_SHIRLEY.items():
                if sink_key in self._systems_data:  # aircraft_name está en systems_data
                    parts = shirley_key.split('.')
                    if len(parts) == 2 and parts[0] == "simulation":
                        simulation[parts[1]] = str(self._systems_data[sink_key])

            # CRITICAL: Ensure pos and att are added to output
            if pos:
                out["position"] = pos
                if DEBUG_FSUIPC_MESSAGES:
                    print(f"[DEBUG] Added position to output: {len(pos)} fields")
            else:
                if DEBUG_FSUIPC_MESSAGES:
                    print(f"[DEBUG] WARNING: pos dict is empty!")

            if att:
                out["attitude"] = att
                if DEBUG_FSUIPC_MESSAGES:
                    print(f"[DEBUG] Added attitude to output: {len(att)} fields")
            else:
                if DEBUG_FSUIPC_MESSAGES:
                    print(f"[DEBUG] WARNING: att dict is empty!")

            # Add non-empty groups to output
            if lights: out["lights"] = lights
            if systems: out["systems"] = systems
            if autopilot: out["autopilot"] = autopilot
            if levers: out["levers"] = levers
            if indicators: out["indicators"] = indicators
            if environment: out["environment"] = environment
            if simulation: out["simulation"] = simulation

            # Validar datos críticos antes de enviar
            if pos.get("latitudeDeg") is not None:
                if not validate_position_data(pos.get("latitudeDeg"), pos.get("longitudeDeg"), pos.get("mslAltitudeFt")):
                    print(f"[WARNING] Invalid position data detected: lat={pos.get('latitudeDeg')}, lon={pos.get('longitudeDeg')}")

            # Official Debug: Show complete JSON when debug enabled
            if DEBUG_FSUIPC_MESSAGES:
                print(f"[DEBUG] Complete JSON to Shirley:")
                print(json.dumps(out, indent=2))
                print(f"[DEBUG] JSON groups: {list(out.keys())}")
                if out:
                    total_fields = sum(len(group) if isinstance(group, dict) else 1 for group in out.values())
                    print(f"[DEBUG] Total fields: {total_fields}")

            # Return the complete snapshot with all groups
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

    def _nz(self, x, eps=ZERO_THRESHOLD_EPSILON):
        """Avoid values close to zero that become '-0'"""
        if x is None:
            return None
        return 0.0 if abs(x) < eps else x

# ===================== FSUIPC WEBSOCKET CLIENT =====================
class FSUIPCWSClient:
    """
    WebSocket client to FSUIPC WebSocket Server.
    Subscribes to offsets and feeds SimData with XGPS/XATT "synthetic" data.
    """
    def __init__(self, sim_data: SimData, url: str = FSUIPC_WS_URL):
        self.sim_data = sim_data
        self.url = url
        self.ws: Optional[Any] = None  # WebSocket client connection
        self.last_data_received_time: Optional[float] = None  # useful for UI/logging

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

        # some builds return 'values' as a list of {name, value}
        if isinstance(payload, list):
            try:
                payload = {it["name"]: it.get("value") for it in payload if isinstance(it, dict) and "name" in it}
            except Exception:
                payload = {}

        if not isinstance(payload, dict):
            return

        # === MAPEO DIRECTO A SIMDATA_SCHEMAS (sin sobreescribir con null) ===
        # Construir snapshot actual para no sobreescribir con None
        current_snapshot = asyncio.create_task(self.sim_data.get_snapshot())
        
        # Indicadores principales
        if "IASraw_U32" in payload:
            ias = knots128_to_kts(payload["IASraw_U32"])
            if ias is not None:
                asyncio.create_task(self.sim_data.update_gps_partial(ias_kts=ias))
        
        if "VSraw" in payload:
            vs = vs_raw_to_fpm(payload["VSraw"])
            if vs is not None:
                asyncio.create_task(self.sim_data.update_gps_partial(vs_fpm_raw=vs))
        
        # GroundSpeedKts: NO procesar manualmente - ya está declarado con transform automático
        # El sistema automático se encarga de: raw → knots128_to_kts → sink("gps", "ground_speed_kts")
        
        if "MagVar_U32" in payload:
            magvar = u32_signed16_to_magdeg(payload["MagVar_U32"])
            if magvar is not None:
                asyncio.create_task(self.sim_data.update_att_partial(mag_var_deg=magvar))

        # BARO (prefiere 0332; validar rango si usas 0330 como fallback)
        baro_inhg = None
        if "BARO_0332_U32" in payload:
            baro_inhg = u32_baro_to_inhg(payload["BARO_0332_U32"])
        if baro_inhg is None and "BARO_0330_U32" in payload:
            raw16 = lower16(payload["BARO_0330_U32"])
            if raw16 is not None and BARO_RAW_MIN <= raw16 <= BARO_RAW_MAX:  # rango razonable: 800–1100 mb
                baro_inhg = u32_baro_to_inhg(payload["BARO_0330_U32"])
        if baro_inhg is not None:
            asyncio.create_task(self.sim_data.update_environment_partial(pressure_inhg=baro_inhg))
            # También publicar en indicators para clientes que esperan ese campo
            asyncio.create_task(self.sim_data.update_indicators_partial(altimeter_inhg=baro_inhg))

        # Luces bitmask (uint32)
        if "LIGHTS_BITS32" in payload:
            m = int(payload["LIGHTS_BITS32"])
            lights_kwargs = {
                "nav_on": bool(m & (1<<0)),
                "landing_on": bool(m & (1<<2)),
                "taxi_on": bool(m & (1<<3)),
                "strobe_on": bool(m & (1<<4)),
            }
            asyncio.create_task(self.sim_data.update_lights_partial(**lights_kwargs))

        # Sistemas
        systems_kwargs = {}
        if "BATTERY_MAIN" in payload:
            systems_kwargs["battery_main_on"] = bool(payload["BATTERY_MAIN"])
        if "PITOT_HEAT_U32" in payload:
            systems_kwargs["pitot_heat_on"] = bool(payload["PITOT_HEAT_U32"])

        # --- Derivado: brakes_on desde offsets U32 ---
        pb = u32_to_bool_parking(payload.get("parkingBrakeU")) if "parkingBrakeU" in payload else None

        bl = lower16(payload.get("brakeLeftU"))  if "brakeLeftU"  in payload else None
        br = lower16(payload.get("brakeRightU")) if "brakeRightU" in payload else None

        brakes_on = None

        # Pedales (0..16383)
        if bl is not None or br is not None:
            L = bl or 0
            R = br or 0
            threshold = BRAKE_PEDAL_THRESHOLD
            brakes_on = (L > threshold) or (R > threshold)

        # Parking según flag
        if USE_BRAKES_ON_INCLUDES_PARKING:
            if pb is not None:
                brakes_on = bool((brakes_on or False) or pb) if brakes_on is not None else bool(pb)

        # Publicar SOLO la clave soportada por el schema
        if brakes_on is not None:
            systems_kwargs["brakes_on"] = brakes_on

        if systems_kwargs:
            asyncio.create_task(self.sim_data.update_systems_partial(**systems_kwargs))

        # Flaps/Gear en %
        levers_kwargs = {}
        if "flapsHandle" in payload:
            levers_kwargs["flaps_pct"] = u32_to_pct_16383(payload["flapsHandle"])
        if "gearHandle" in payload:
            levers_kwargs["gear_pct"] = u32_to_pct_16383(payload["gearHandle"])
        
        if levers_kwargs:
            asyncio.create_task(self.sim_data.update_levers_partial(**levers_kwargs))

        # Posición/actitud (mantener el mapeo automático existente)
        gps_kwargs = {}
        att_kwargs = {}
        
        if "LatitudeDeg" in payload:
            gps_kwargs["latitude"] = payload["LatitudeDeg"]
        if "LongitudeDeg" in payload:
            gps_kwargs["longitude"] = payload["LongitudeDeg"]
        if "AltitudeM" in payload:
            gps_kwargs["alt_msl_meters"] = payload["AltitudeM"]
        if "GroundAltRaw" in payload:
            gps_kwargs["ground_alt_m"] = meters256_to_m(payload["GroundAltRaw"])
        
        if "BankRaw" in payload:
            att_kwargs["roll_deg"] = -raw_ang_to_deg(payload["BankRaw"])
        if "PitchRaw" in payload:
            att_kwargs["pitch_deg"] = raw_ang_to_deg_pitch(payload["PitchRaw"])
        if "HeadingTrueRaw" in payload:
            att_kwargs["heading_deg"] = raw_hdg_to_deg(payload["HeadingTrueRaw"])

        # Nombre aeronave - almacenar en systems_data temporalmente
        if "aircraftNameStr" in payload:
            systems_kwargs["aircraft_name"] = str(payload["aircraftNameStr"])

        if gps_kwargs:
            asyncio.create_task(self.sim_data.update_gps_partial(**gps_kwargs))
        if att_kwargs:
            asyncio.create_task(self.sim_data.update_att_partial(**att_kwargs))

        # === SISTEMA AUTOMÁTICO PARA OFFSETS NO PROCESADOS MANUALMENTE ===
        # Procesar READ_SIGNALS que no fueron manejados manualmente arriba
        auto_gps_kwargs = {}
        auto_att_kwargs = {}
        auto_lights_kwargs = {}
        auto_systems_kwargs = {}
        auto_environment_kwargs = {}

        for key, cfg in READ_SIGNALS.items():
            if key not in payload:
                continue
            
            # Skip offsets ya procesados manualmente
            if key in ["IASraw_U32", "VSraw", "MagVar_U32", "BARO_0332_U32", "BARO_0330_U32", 
                      "LIGHTS_BITS32", "BATTERY_MAIN", "PITOT_HEAT_U32", "brakeLeftU", 
                      "brakeRightU", "parkingBrakeU", "flapsHandle", "gearHandle", 
                      "aircraftNameStr", "LatitudeDeg", "LongitudeDeg", "AltitudeM", "GroundAltRaw",
                      "BankRaw", "PitchRaw", "HeadingTrueRaw"]:
                continue
                
            val = payload[key]
            # Aplicar transform si existe
            tf = cfg.get("transform")
            if tf and tf in TRANSFORMS:
                raw_val = val
                val = TRANSFORMS[tf](val)
                # Debug temporal para GroundSpeedKts
                if key == "GroundSpeedKts" and not hasattr(self, "_gs_auto_dbg"):
                    self._gs_auto_dbg = 0
                if key == "GroundSpeedKts" and self._gs_auto_dbg < 5:
                    self._gs_auto_dbg += 1
            
            # Dispatch según sink
            sink_group, sink_field = cfg["sink"]
            if sink_group == "gps" and val is not None and sink_field not in auto_gps_kwargs:
                auto_gps_kwargs[sink_field] = val
            elif sink_group == "att" and val is not None and sink_field not in auto_att_kwargs:
                auto_att_kwargs[sink_field] = val
            elif sink_group == "lights" and val is not None and sink_field not in auto_lights_kwargs:
                auto_lights_kwargs[sink_field] = val
            elif sink_group == "systems" and val is not None and sink_field not in auto_systems_kwargs:
                auto_systems_kwargs[sink_field] = val
            elif sink_group == "environment" and val is not None and sink_field not in auto_environment_kwargs:
                auto_environment_kwargs[sink_field] = val

        # --- Derivado: brakes_on (de U32) ---
        pb = None
        if "parkingBrakeU" in payload:
            pb = u32_to_bool_parking(payload["parkingBrakeU"])

        bl = lower16(payload.get("brakeLeftU"))  if "brakeLeftU"  in payload else None
        br = lower16(payload.get("brakeRightU")) if "brakeRightU" in payload else None

        brakes_on = None
        if bl is not None or br is not None:
            L = bl or 0
            R = br or 0
            threshold = BRAKE_PEDAL_THRESHOLD           # 0..16383
            brakes_on = (L > threshold) or (R > threshold)

        # OR con parking
        if pb is not None:
            brakes_on = bool((brakes_on or False) or pb) if brakes_on is not None else bool(pb)

        # Publica SOLO lo que entiende el schema
        # if brakes_on is not None:
            # systems_kwargs["brakes_on"] = brakes_on REVISAR FRENADO


        # Aplicar updates automáticos
        if auto_gps_kwargs:
            asyncio.create_task(self.sim_data.update_gps_partial(**auto_gps_kwargs))
        if auto_att_kwargs:
            asyncio.create_task(self.sim_data.update_att_partial(**auto_att_kwargs))
        if auto_lights_kwargs:
            asyncio.create_task(self.sim_data.update_lights_partial(**auto_lights_kwargs))
        if auto_systems_kwargs:
            asyncio.create_task(self.sim_data.update_systems_partial(**auto_systems_kwargs))
        if auto_environment_kwargs:
            asyncio.create_task(self.sim_data.update_environment_partial(**auto_environment_kwargs))

        self.last_data_received_time = time.time()

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

# ===================== SHIRLEY WEBSOCKET SERVER =====================
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

                # Official Debug: Show broadcast info
                if DEBUG_FSUIPC_MESSAGES:
                    print(f"[DEBUG] Broadcasting to {len(self.connections)} clients")
                    if not snapshot:
                        print(f"[DEBUG] WARNING: Empty snapshot!")

                # DEBUG: Verificar que no hay keys prohibidas
                if any(key in snapshot for key in ["type", "reads", "writes"]):
                    print(f"[ERROR] Snapshot contains prohibited keys: {list(snapshot.keys())}")

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

# ===================== MAIN ORCHESTRATOR =====================
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