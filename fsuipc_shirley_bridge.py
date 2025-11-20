import asyncio
import json
import logging
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any, Set

import websockets
import websockets.exceptions

# Optional: Load environment variables from .env file
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    # python-dotenv not installed, will use system environment variables only
    pass

# ===================== LOGGING CONFIGURATION =====================
def setup_logging():
    """Configure logging system with appropriate handlers and formatters."""
    log_level_str = os.getenv("LOG_LEVEL", "INFO").upper()
    log_level = getattr(logging, log_level_str, logging.INFO)

    # Create formatter
    formatter = logging.Formatter(
        fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)

    # File handler (optional, only if LOG_FILE is set)
    handlers = [console_handler]
    log_file = os.getenv("LOG_FILE")
    if log_file:
        try:
            file_handler = logging.FileHandler(log_file)
            file_handler.setFormatter(formatter)
            handlers.append(file_handler)
        except Exception as e:
            print(f"Warning: Could not create log file {log_file}: {e}", file=sys.stderr)

    # Configure root logger
    logging.basicConfig(
        level=log_level,
        handlers=handlers,
        force=True
    )

    # Create module logger
    logger = logging.getLogger("fsuipc_shirley_bridge")
    logger.setLevel(log_level)

    return logger

# Initialize logging
logger = setup_logging()

# ===================== CONFIGURATION =====================
# Configuration values can be overridden via environment variables
# Example: export FSUIPC_WS_URL="ws://192.168.1.100:2048/fsuipc/"

FSUIPC_WS_URL = os.getenv("FSUIPC_WS_URL", "ws://localhost:2048/fsuipc/")
WS_HOST = os.getenv("WS_HOST", "localhost")
WS_PORT = int(os.getenv("WS_PORT", "2992"))
WS_PATH = os.getenv("WS_PATH", "/api/v1")
SEND_INTERVAL = float(os.getenv("SEND_INTERVAL", "0.25"))  # 4 Hz (every 250 ms)
DEBUG_FSUIPC_MESSAGES = os.getenv("DEBUG_FSUIPC_MESSAGES", "false").lower() in ("true", "1", "yes")

# Internal state (not configurable via environment)
FIRST_PAYLOAD = False

# Log configuration on startup
logger.info("=" * 60)
logger.info("FSUIPC-Shirley-Bridge Configuration")
logger.info("=" * 60)
logger.info(f"FSUIPC WebSocket URL: {FSUIPC_WS_URL}")
logger.info(f"Shirley WebSocket: ws://{WS_HOST}:{WS_PORT}{WS_PATH}")
logger.info(f"Send Interval: {SEND_INTERVAL}s ({1/SEND_INTERVAL:.1f} Hz)")
logger.info(f"Debug FSUIPC Messages: {DEBUG_FSUIPC_MESSAGES}")
logger.info(f"Log Level: {logging.getLevelName(logger.level)}")
logger.info("=" * 60)

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

    # === RADIOS/NAVIGATION (CORREGIDOS) ===
    "COM1_FREQ":      {"address": 0x034E, "type": "uint", "size": 2, "transform": "bcd_to_freq_com_official", "sink": ("radios", "com1_active_khz")},
    "COM1_STANDBY":   {"address": 0x311A, "type": "uint", "size": 2, "transform": "bcd_to_freq_com_official", "sink": ("radios", "com1_standby_khz")},
    "COM2_FREQ":      {"address": 0x3118, "type": "uint", "size": 2, "transform": "bcd_to_freq_com_official", "sink": ("radios", "com2_active_khz")},
    "COM2_STANDBY":   {"address": 0x311C, "type": "uint", "size": 2, "transform": "bcd_to_freq_com_official", "sink": ("radios", "com2_standby_khz")},
    "NAV1_FREQ":      {"address": 0x0350, "type": "uint", "size": 2, "transform": "bcd_to_freq_nav_official", "sink": ("radios", "nav1_active_khz")},
    "NAV1_STANDBY":   {"address": 0x311E, "type": "uint", "size": 2, "transform": "bcd_to_freq_nav_official", "sink": ("radios", "nav1_standby_khz")},
    "TRANSPONDER":    {"address": 0x0354, "type": "uint", "size": 2, "transform": "bcd_to_xpdr_official", "sink": ("radios", "transponder_code")},

    # === INDICATORS (NUEVOS) ===
    "ENGINE1_RPM":    {"address": 0x0898, "type": "uint", "size": 4, "transform": "rpm_raw_to_rpm", "sink": ("indicators", "engine1_rpm")},
    "ENGINE2_RPM":    {"address": 0x0930, "type": "uint", "size": 4, "transform": "rpm_raw_to_rpm", "sink": ("indicators", "engine2_rpm")},
    "PROP1_RPM":      {"address": 0x089C, "type": "uint", "size": 4, "transform": "rpm_raw_to_rpm", "sink": ("indicators", "prop1_rpm")},
    "PROP2_RPM":      {"address": 0x0934, "type": "uint", "size": 4, "transform": "rpm_raw_to_rpm", "sink": ("indicators", "prop2_rpm")},
    "MANIFOLD_PRESSURE": {"address": 0x08A0, "type": "uint", "size": 4, "transform": "manifold_to_inhg", "sink": ("indicators", "manifold_pressure")},
    "ENGINE1_N1":     {"address": 0x2010, "type": "float", "size": 8, "transform": None, "sink": ("indicators", "engine1_n1_pct")},
    "ENGINE1_EGT":    {"address": 0x08B8, "type": "uint", "size": 2, "transform": "egt_to_celsius", "sink": ("indicators", "engine1_egt_c")},
    "ENGINE1_CHT":    {"address": 0x08BA, "type": "uint", "size": 2, "transform": "temp_to_celsius", "sink": ("indicators", "engine1_cht_c")},
    # === ESTOS CAMPOS NO EXISTEN EN SCHEMA - REMOVER ===
    # "FUEL_QTY_LEFT":  {"address": 0x0B74, "type": "uint", "size": 4, "transform": "fuel_to_gallons", "sink": ("systems", "fuel_left_gal")},
    # "FUEL_QTY_RIGHT": {"address": 0x0B7C, "type": "uint", "size": 4, "transform": "fuel_to_gallons", "sink": ("systems", "fuel_right_gal")},
    # "OIL_TEMP":       {"address": 0x08B0, "type": "uint", "size": 2, "transform": "temp_to_celsius", "sink": ("systems", "oil_temp_c")},
    # "OIL_PRESSURE":   {"address": 0x08B4, "type": "uint", "size": 2, "transform": "oil_pressure_to_psi", "sink": ("systems", "oil_pressure_psi")},

    # === LEVERS ADICIONALES ===
    "THROTTLE1_POS":  {"address": 0x088C, "type": "int", "size": 2, "transform": "throttle_to_percent", "sink": ("levers", "throttle1_pct")},
    "THROTTLE2_POS":  {"address": 0x0924, "type": "int", "size": 2, "transform": "throttle_to_percent", "sink": ("levers", "throttle2_pct")},
    "MIXTURE1_POS":   {"address": 0x08A4, "type": "int", "size": 2, "transform": "mixture_to_percent", "sink": ("levers", "mixture1_pct")},
    "MIXTURE2_POS":   {"address": 0x093C, "type": "int", "size": 2, "transform": "mixture_to_percent", "sink": ("levers", "mixture2_pct")},
    "PROP1_POS":      {"address": 0x08A8, "type": "int", "size": 2, "transform": "prop_to_percent", "sink": ("levers", "prop1_pct")},
    "PROP2_POS":      {"address": 0x0940, "type": "int", "size": 2, "transform": "prop_to_percent", "sink": ("levers", "prop2_pct")},
    "SPEEDBRAKE_POS": {"address": 0x0BD0, "type": "uint", "size": 4, "transform": "u32_to_pct_16383", "sink": ("levers", "speedbrake_pct")},

    # === AUTOPILOT ===
    "AP_MASTER":      {"address": 0x07BC, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("autopilot", "master_on")},
    "AP_HDG_HOLD":    {"address": 0x07C8, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("autopilot", "hdg_select_on")},
    "AP_ALT_HOLD":    {"address": 0x07D0, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("autopilot", "alt_hold_on")},
    "AP_HDG_BUG":     {"address": 0x07CC, "type": "uint", "size": 2, "transform": "heading_bug_to_deg", "sink": ("autopilot", "hdg_bug_deg")},
    "AP_ALT_BUG":     {"address": 0x07D4, "type": "uint", "size": 4, "transform": "alt_bug_to_feet", "sink": ("autopilot", "alt_bug_ft")},
    "AP_VS_HOLD":     {"address": 0x07EC, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("autopilot", "vs_hold_on")},
    "AP_VS_TARGET":   {"address": 0x07F2, "type": "int", "size": 2, "transform": "vs_target_to_fpm", "sink": ("autopilot", "vs_target_fpm")},

    # === ENVIRONMENT ADICIONAL ===
    "WIND_SPEED":     {"address": 0x0E90, "type": "uint", "size": 2, "transform": "wind_to_kts", "sink": ("environment", "wind_speed_kts")},
    "WIND_DIR":       {"address": 0x0E92, "type": "uint", "size": 2, "transform": "wind_dir_to_deg", "sink": ("environment", "wind_dir_deg")},
    "OUTSIDE_TEMP":   {"address": 0x0E8C, "type": "int", "size": 2, "transform": "temp_to_celsius", "sink": ("environment", "outside_temp_c")},
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
            logger.debug(f"Transform knots128_to_kts failed for {raw}: {e}")
        return None

def vs_raw_to_fpm(raw):
    # raw = 256 * m/s  ->  ft/min
    try: return float(raw) * SECONDS_PER_MINUTE * METERS_TO_FEET / FSUIPC_SCALE_FACTOR_256
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform vs_raw_to_fpm failed for {raw}: {e}")
        return None

def meters256_to_m(raw):
    # ground altitude in meters *256
    try: return float(raw) / FSUIPC_SCALE_FACTOR_256
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform meters256_to_m failed for {raw}: {e}")
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
            logger.debug(f"Transform magvar_raw_to_deg failed for {raw}: {e}")
        return None

def bits_to_bool_0(raw):
    """Extract bit 0 from FSUIPC bits object"""
    try:
        if isinstance(raw, dict) and '0' in raw:
            return bool(raw['0'])
        return None
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform bits_to_bool_0 failed for {raw}: {e}")
        return None

def bits_to_bool_1(raw):
    """Extract bit 1 from FSUIPC bits object"""
    try:
        if isinstance(raw, dict) and '1' in raw:
            return bool(raw['1'])
        return None
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform bits_to_bool_1 failed for {raw}: {e}")
        return None

def bits_to_bool_2(raw):
    """Extract bit 2 from FSUIPC bits object"""
    try:
        if isinstance(raw, dict) and '2' in raw:
            return bool(raw['2'])
        return None
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform bits_to_bool_2 failed for {raw}: {e}")
        return None

def bits_to_bool_3(raw):
    """Extract bit 3 from FSUIPC bits object"""
    try:
        if isinstance(raw, dict) and '3' in raw:
            return bool(raw['3'])
        return None
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform bits_to_bool_3 failed for {raw}: {e}")
        return None

def bits_to_bool_4(raw):
    """Extract bit 4 from FSUIPC bits object"""
    try:
        if isinstance(raw, dict) and '4' in raw:
            return bool(raw['4'])
        return None
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform bits_to_bool_4 failed for {raw}: {e}")
        return None

def nonzero_to_bool(raw):
    """Convert non-zero values to True, zero to False"""
    try: return bool(int(raw))
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform nonzero_to_bool failed for {raw}: {e}")
        return None



def baro_to_inhg(raw):
    """Convert barometric pressure from millibars*16 to inches of mercury"""
    try:
        mb = float(raw) / FSUIPC_SCALE_FACTOR_16  # Convert to millibars
        return mb * MB_TO_INHG_FACTOR     # Convert mb to inHg
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform baro_to_inhg failed for {raw}: {e}")
        return None

# === U32 → lower16 helpers (from probe findings) ===
def lower16(u):
    try: return int(u) & FSUIPC_SIGNED_16BIT_MASK
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform lower16 failed for {u}: {e}")
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
            logger.debug(f"Transform gs_u32_to_kts failed for {raw}: {e}")
        return None

# ===================== NUEVAS TRANSFORMACIONES PARA SCHEMA =====================

def bcd_to_freq_com(raw):
    """Convert BCD COM frequency correctly"""
    try:
        val = int(raw)
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"COM_DEBUG: Raw COM frequency: {val} (hex: 0x{val:08X})")

        # FSUIPC COM frequencies are stored as packed BCD
        # Format: 0x0001XXYY where XX.YY is the frequency
        # Example: 127.850 MHz stored as 0x00012785

        # Extract the frequency part (lower 16 bits typically)
        freq_bcd = val & 0xFFFF

        # Convert BCD to frequency
        # Each nibble represents a decimal digit
        mhz_hundreds = (freq_bcd >> 12) & 0xF  # 1 (from 127.85)
        mhz_tens = (freq_bcd >> 8) & 0xF       # 2 (from 127.85)
        mhz_units = (freq_bcd >> 4) & 0xF      # 7 (from 127.85)
        khz_hundreds = freq_bcd & 0xF          # 8 (from 127.85, .850)

        # Additional digits may be in upper bits
        if val > 0xFFFF:
            khz_tens = (val >> 20) & 0xF
            khz_units = (val >> 16) & 0xF
        else:
            khz_tens = 5  # Default assumption
            khz_units = 0

        # Construct frequency in kHz
        frequency_khz = (mhz_hundreds * 100 + mhz_tens * 10 + mhz_units) * 1000 + \
                        khz_hundreds * 100 + khz_tens * 10 + khz_units

        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"COM_DEBUG: BCD conversion: {mhz_hundreds}{mhz_tens}{mhz_units}.{khz_hundreds}{khz_tens}{khz_units} = {frequency_khz} kHz")

        # Validate COM range (118.000 - 136.975 MHz)
        if frequency_khz < 118000 or frequency_khz > 136975:
            if DEBUG_FSUIPC_MESSAGES:
                logger.debug(f"COM_DEBUG: Frequency {frequency_khz} out of range, using default 122750")
            return 122750

        return frequency_khz

    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform bcd_to_freq_com failed for {raw}: {e}")
        return 122750  # Default frequency

def bcd_to_freq_com_official(raw):
    """Convert COM frequency according to FSUIPC official documentation"""
    try:
        val = int(raw)
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"COM_OFFICIAL: Raw COM value: {val} (hex: 0x{val:04X})")

        # According to FSUIPC doc: 4 digits in BCD, leading 1 assumed
        # Example: 123.45 MHz -> 0x2345 (2345 decimal)
        # Format: 0xXXYY -> 1XX.YY MHz

        # Extract BCD digits
        tens_mhz = (val >> 12) & 0xF      # First BCD digit (tens of MHz after 1)
        units_mhz = (val >> 8) & 0xF      # Second BCD digit (units of MHz)
        tens_khz = (val >> 4) & 0xF       # Third BCD digit (tenths of MHz)
        units_khz = val & 0xF             # Fourth BCD digit (hundredths of MHz)

        # Construct frequency: 1XX.YY MHz
        # Leading 1 is assumed, so we get 1 + tens_mhz + units_mhz . tens_khz + units_khz
        frequency_mhz = 100 + (tens_mhz * 10) + units_mhz + (tens_khz * 0.1) + (units_khz * 0.01)
        frequency_khz = int(frequency_mhz * 1000)

        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"COM_OFFICIAL: BCD digits: {tens_mhz}{units_mhz}.{tens_khz}{units_khz}")
            logger.debug(f"COM_OFFICIAL: Frequency: 1{tens_mhz}{units_mhz}.{tens_khz}{units_khz} MHz = {frequency_khz} kHz")

        # Validate range (118000-136975 kHz)
        if frequency_khz < 118000 or frequency_khz > 136975:
            if DEBUG_FSUIPC_MESSAGES:
                logger.debug(f"COM_OFFICIAL: Frequency {frequency_khz} out of COM range, using default")
            return 122750

        return frequency_khz

    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"COM_OFFICIAL: Transform failed for {raw}: {e}")
        return 122750

def bcd_to_freq_nav_official(raw):
    """Convert NAV frequency according to FSUIPC official documentation"""
    try:
        val = int(raw)
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"NAV_OFFICIAL: Raw NAV value: {val} (hex: 0x{val:04X})")

        # According to FSUIPC doc: 4 digits in BCD, leading 1 assumed
        # Example: 113.45 MHz -> 0x1345
        # Format: 0xXXYY -> 1XX.YY MHz (same as COM)

        tens_mhz = (val >> 12) & 0xF
        units_mhz = (val >> 8) & 0xF
        tens_khz = (val >> 4) & 0xF
        units_khz = val & 0xF

        # Construct frequency: 1XX.YY MHz
        frequency_mhz = 100 + (tens_mhz * 10) + units_mhz + (tens_khz * 0.1) + (units_khz * 0.01)
        frequency_khz = int(frequency_mhz * 1000)

        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"NAV_OFFICIAL: BCD digits: {tens_mhz}{units_mhz}.{tens_khz}{units_khz}")
            logger.debug(f"NAV_OFFICIAL: Frequency: 1{tens_mhz}{units_mhz}.{tens_khz}{units_khz} MHz = {frequency_khz} kHz")

        # Validate NAV range (108000-117950 kHz)
        if frequency_khz < 108000 or frequency_khz > 117950:
            if DEBUG_FSUIPC_MESSAGES:
                logger.debug(f"NAV_OFFICIAL: Frequency {frequency_khz} out of NAV range, using default")
            return 110000

        return frequency_khz

    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"NAV_OFFICIAL: Transform failed for {raw}: {e}")
        return 110000

def bcd_to_xpdr_official(raw):
    """Convert transponder according to FSUIPC official documentation"""
    try:
        val = int(raw)
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"XPDR_OFFICIAL: Raw transponder value: {val} (hex: 0x{val:04X})")

        # According to FSUIPC doc: 4 digits in BCD format
        # Example: 0x1200 means 1200 on the dials
        # This is straightforward BCD to decimal conversion

        thousands = (val >> 12) & 0xF
        hundreds = (val >> 8) & 0xF
        tens = (val >> 4) & 0xF
        units = val & 0xF

        result = thousands * 1000 + hundreds * 100 + tens * 10 + units

        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"XPDR_OFFICIAL: BCD digits: {thousands}{hundreds}{tens}{units} = {result}")

        # Validate transponder range (0000-7777)
        if result > 7777:
            if DEBUG_FSUIPC_MESSAGES:
                logger.debug(f"XPDR_OFFICIAL: Invalid transponder {result}, using 1200")
            return 1200

        return result

    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"XPDR_OFFICIAL: Transform failed for {raw}: {e}")
        return 1200

def bcd_to_freq_nav(raw):
    """Convert BCD NAV frequency correctly"""
    try:
        val = int(raw)
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"NAV_DEBUG: Raw NAV frequency: {val} (hex: 0x{val:08X})")

        # Similar to COM but different valid range
        # ... (same BCD parsing logic as COM)

        # For now, use simple approach
        if 108000 <= val <= 117950:
            return val
        elif 108 <= val <= 118:
            return val * 1000
        else:
            return 110000  # Default NAV frequency

    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform bcd_to_freq_nav failed for {raw}: {e}")
        return 110000

def bcd_to_freq_com_simple(raw):
    """Simplified COM frequency conversion with debugging"""
    try:
        val = int(raw)
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"COM_SIMPLE: Raw value: {val}")

        # Si el valor parece razonable, usarlo directamente
        if 118000 <= val <= 136975:
            return val

        # Si está en MHz*1000 format
        if 118 <= val <= 137:
            return val * 1000

        # Si es un valor BCD simple, convertir dígito por dígito
        if val > 0:
            # Extract as string and reinterpret
            str_val = f"{val:08d}"
            try:
                # Try to extract meaningful frequency parts
                if len(str_val) >= 4:
                    mhz = int(str_val[:3])  # First 3 digits as MHz
                    khz = int(str_val[3:6]) if len(str_val) >= 6 else 0  # Next 3 as kHz
                    frequency = mhz * 1000 + khz

                    if 118000 <= frequency <= 136975:
                        if DEBUG_FSUIPC_MESSAGES:
                            logger.debug(f"COM_SIMPLE: Parsed {str_val} as {frequency} kHz")
                        return frequency
            except:
                pass

        # Fallback
        return 122750

    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"COM_SIMPLE: Failed for {raw}: {e}")
        return 122750

def bcd_to_xpdr(raw):
    """Convert BCD transponder code correctly"""
    try:
        val = int(raw)
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"XPDR_DEBUG: Raw transponder value: {val} (hex: 0x{val:04X})")

        # FSUIPC transponder is stored as BCD in a 16-bit word
        # Each digit occupies 4 bits (nibble)
        digit1 = (val >> 12) & 0xF  # Thousands
        digit2 = (val >> 8) & 0xF   # Hundreds
        digit3 = (val >> 4) & 0xF   # Tens
        digit4 = val & 0xF          # Units

        # Convert BCD digits to decimal
        result = digit1 * 1000 + digit2 * 100 + digit3 * 10 + digit4

        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"XPDR_DEBUG: BCD digits: {digit1}{digit2}{digit3}{digit4} = {result}")

        # Validate range (0000-7777 for transponder)
        if result > 7777:
            if DEBUG_FSUIPC_MESSAGES:
                logger.debug(f"XPDR_DEBUG: Invalid transponder code {result}, using 1200")
            return 1200

        return result

    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform bcd_to_xpdr failed for {raw}: {e}")
        return 1200  # Default squawk code

def rpm_raw_to_rpm(raw):
    """Convert raw RPM to actual RPM"""
    try:
        return float(raw)  # Direct conversion for most aircraft
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform rpm_raw_to_rpm failed for {raw}: {e}")
        return None

def manifold_to_inhg(raw):
    """Convert manifold pressure to inches of mercury"""
    try:
        return float(raw) / 1024.0  # Typical FSUIPC scaling
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform manifold_to_inhg failed for {raw}: {e}")
        return None

def egt_to_celsius(raw):
    """Convert EGT to Celsius"""
    try:
        return (float(raw) * 850.0 / 16384.0) - 273.15  # Convert from Rankine
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform egt_to_celsius failed for {raw}: {e}")
        return None

def temp_to_celsius(raw):
    """Convert temperature to Celsius (corregida para valores reales)"""
    try:
        val = float(raw)
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"TEMP_DEBUG: Raw temp value: {val}")

        # Si el valor es muy negativo, podría ser Kelvin*256 mal interpretado
        if val < -200:
            # Probablemente el valor necesita interpretación diferente
            # Try direct conversion assuming it's already in reasonable range
            if DEBUG_FSUIPC_MESSAGES:
                logger.debug(f"TEMP_DEBUG: Temperature out of range, using default")
            return 15.0  # Temperature default for debugging

        # Conversión normal: raw/256 - 273.15 (from Kelvin*256)
        celsius = (val / 256.0) - 273.15

        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"TEMP_DEBUG: Converted temp: {celsius}°C")

        # Sanity check: Si está fuera de rango razonable (-50°C a +50°C)
        if celsius < -50 or celsius > 50:
            if DEBUG_FSUIPC_MESSAGES:
                logger.debug(f"TEMP_DEBUG: Temperature out of range, using default")
            return 15.0  # Temperatura ambiente default

        return celsius

    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"TEMP_ERROR: Temperature conversion failed: {raw} -> {e}")
        return 15.0  # Temperatura ambiente default

def temp_to_celsius_alt(raw):
    """Alternative temperature conversion"""
    try:
        val = float(raw)
        # Different FSUIPC scaling - try direct Fahrenheit to Celsius
        fahrenheit = val / 256.0
        celsius = (fahrenheit - 32) * 5.0 / 9.0

        # Sanity check
        if celsius < -50 or celsius > 50:
            return 15.0

        return celsius
    except:
        return 15.0

def fuel_to_gallons(raw):
    """Convert fuel quantity to gallons"""
    try:
        return float(raw) * 128.0 / (65536.0 * 256.0)
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform fuel_to_gallons failed for {raw}: {e}")
        return None

def oil_pressure_to_psi(raw):
    """Convert oil pressure to PSI"""
    try:
        return float(raw) / 16384.0 * 55.0  # Typical max 55 PSI
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform oil_pressure_to_psi failed for {raw}: {e}")
        return None

def throttle_to_percent(raw):
    """Convert throttle position to percentage"""
    try:
        val = int(raw)
        if val < 0: val += 65536  # Handle signed
        return (val / 16384.0) * 100.0
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform throttle_to_percent failed for {raw}: {e}")
        return None

def mixture_to_percent(raw):
    """Convert mixture position to percentage"""
    try:
        val = int(raw)
        if val < 0: val += 65536
        return (val / 16384.0) * 100.0
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform mixture_to_percent failed for {raw}: {e}")
        return None

def prop_to_percent(raw):
    """Convert prop position to percentage"""
    try:
        val = int(raw)
        if val < 0: val += 65536
        return (val / 16384.0) * 100.0
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform prop_to_percent failed for {raw}: {e}")
        return None

def heading_bug_to_deg(raw):
    """Convert heading bug to degrees (always return number)"""
    try:
        val = float(raw)
        result = (val * 360.0) / 65536.0 if val != 0 else 0.0
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform heading_bug_to_deg: {raw} → {result}")
        return result
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform heading_bug_to_deg failed for {raw}: {e}")
        return 0.0  # Always return a number

def alt_bug_to_feet(raw):
    """Convert altitude bug to feet (always return number)"""
    try:
        val = float(raw)
        result = val if val != 0 else 0.0
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform alt_bug_to_feet: {raw} → {result}")
        return result
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform alt_bug_to_feet failed for {raw}: {e}")
        return 0.0  # Always return a number

def vs_target_to_fpm(raw):
    """Convert VS target to feet per minute"""
    try:
        return float(raw)
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform vs_target_to_fpm failed for {raw}: {e}")
        return None

def wind_to_kts(raw):
    """Convert wind speed to knots"""
    try:
        return float(raw)
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform wind_to_kts failed for {raw}: {e}")
        return None

def wind_dir_to_deg(raw):
    """Convert wind direction to degrees"""
    try:
        return (float(raw) * 360.0) / 65536.0
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform wind_dir_to_deg failed for {raw}: {e}")
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
    "gs_u32_to_kts": gs_u32_to_kts,

    # New transforms for schema variables
    "bcd_to_freq_com": bcd_to_freq_com,
    "bcd_to_freq_nav": bcd_to_freq_nav,
    "bcd_to_xpdr": bcd_to_xpdr,
    "rpm_raw_to_rpm": rpm_raw_to_rpm,
    "manifold_to_inhg": manifold_to_inhg,
    "egt_to_celsius": egt_to_celsius,
    "temp_to_celsius": temp_to_celsius,
    "temp_to_celsius_alt": temp_to_celsius_alt,  # Alternative temperature conversion
    "fuel_to_gallons": fuel_to_gallons,
    "oil_pressure_to_psi": oil_pressure_to_psi,
    "throttle_to_percent": throttle_to_percent,
    "mixture_to_percent": mixture_to_percent,
    "prop_to_percent": prop_to_percent,
    "heading_bug_to_deg": heading_bug_to_deg,
    "alt_bug_to_feet": alt_bug_to_feet,
    "vs_target_to_fpm": vs_target_to_fpm,
    "wind_to_kts": wind_to_kts,
    "wind_dir_to_deg": wind_dir_to_deg,

    # Official FSUIPC documentation transforms
    "bcd_to_freq_com_official": bcd_to_freq_com_official,
    "bcd_to_freq_nav_official": bcd_to_freq_nav_official,
    "bcd_to_xpdr_official": bcd_to_xpdr_official,
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
    # REMOVIDAS: fuel_left_gal, fuel_right_gal, oil_temp_c, oil_pressure_psi (NO EXISTEN EN SCHEMA)
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

# ===================== NUEVOS SINK MAPPINGS =====================

_RADIOS_SINK_TO_SHIRLEY = {
    "com1_active_khz": "radiosNavigation.frequencyHz.com1",
    "com1_standby_khz": "radiosNavigation.standbyFrequencyHz.com1",
    "com2_active_khz": "radiosNavigation.frequencyHz.com2",
    "com2_standby_khz": "radiosNavigation.standbyFrequencyHz.com2",
    "nav1_active_khz": "radiosNavigation.frequencyHz.nav1",
    "nav1_standby_khz": "radiosNavigation.standbyFrequencyHz.nav1",
    "transponder_code": "radiosNavigation.transponderCode",
}

_INDICATORS_ADDITIONAL_SINK_TO_SHIRLEY = {
    "engine1_rpm": "indicators.engineRpm.engine1",
    "engine2_rpm": "indicators.engineRpm.engine2",
    "prop1_rpm": "indicators.propellerRpm.prop1",
    "prop2_rpm": "indicators.propellerRpm.prop2",
    "manifold_pressure": "indicators.manifoldPressureInchesMercury.engine1",
    "engine1_n1_pct": "indicators.engineN1Percent.engine1",
    "engine1_egt_c": "indicators.exhaustGasDegC.engine1",
    "engine1_cht_c": "indicators.turbineGasTemperatureDegC.engine1",  # CHT -> TGT que sí existe
    # REMOVIDAS: fuel_left_gal, fuel_right_gal, oil_temp_c, oil_pressure_psi (no existen en schema)
}

_LEVERS_ADDITIONAL_SINK_TO_SHIRLEY = {
    "throttle1_pct": "levers.throttlePercentOpen.engine1",
    "throttle2_pct": "levers.throttlePercentOpen.engine2",
    "mixture1_pct": "levers.mixtureLeverPercentRich.engine1",
    "mixture2_pct": "levers.mixtureLeverPercentRich.engine2",
    "prop1_pct": "levers.propellerLeverPercentCoarse.prop1",
    "prop2_pct": "levers.propellerLeverPercentCoarse.prop2",
    "speedbrake_pct": "levers.speedBrakesHandlePercentDeployed",
}

_AUTOPILOT_SINK_TO_SHIRLEY = {
    "master_on": "autopilot.isAutopilotEngaged",
    "hdg_select_on": "autopilot.isHeadingSelectEnabled",
    "hdg_bug_deg": "autopilot.magneticHeadingBugDeg",
    "alt_bug_ft": "autopilot.altitudeBugFt",
    # REMOVIDO: "vs_target_fpm": "autopilot.targetVerticalSpeedFpm",  # NO EXISTE EN SCHEMA
    # REMOVIDAS: alt_hold_on, vs_hold_on (no se mapean directamente)
}

_ENVIRONMENT_ADDITIONAL_SINK_TO_SHIRLEY = {
    "wind_speed_kts": "environment.aircraftWindSpeedKts",
    "wind_dir_deg": "environment.aircraftWindHeadingDeg",
    "outside_temp_c": "environment.groundTemperatureDegC",
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

        # Nuevos grupos de datos
        self._radios_data = {}       # COM/NAV frequencies, transponder
        self._indicators_data = {}   # RPMs, temperatures, pressures, fuel
        self._autopilot_data = {}    # AP status, bugs, targets

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

    async def update_radios_partial(self, **kwargs):
        async with self._lock:
            for key, value in kwargs.items():
                if value is not None:
                    self._radios_data[key] = value
            self.last_timestamp = iso_utc_ms()

    async def update_indicators_partial(self, **kwargs):
        async with self._lock:
            for key, value in kwargs.items():
                if value is not None:
                    self._indicators_data[key] = value
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
                logger.debug(f"pos dict: {pos}")
                logger.debug(f"att dict: {att}")
                logger.debug(f"self.xgps exists: {self.xgps is not None}")
                logger.debug(f"self.xatt exists: {self.xatt is not None}")
                if self.xgps:
                    logger.debug(f"xgps latitude: {self.xgps.latitude}")
                    logger.debug(f"xgps longitude: {self.xgps.longitude}")
                    logger.debug(f"xgps alt_msl_meters: {self.xgps.alt_msl_meters}")
                    logger.debug(f"xgps ground_speed_kts: {self.xgps.ground_speed_kts}")
                if self.xatt:
                    logger.debug(f"xatt heading_deg: {self.xatt.heading_deg}")
                    logger.debug(f"xatt pitch_deg: {self.xatt.pitch_deg}")
                    logger.debug(f"xatt roll_deg: {self.xatt.roll_deg}")

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
                        # Forzar tipos específicos para campos problemáticos
                        if sink_key in ["hdg_bug_deg", "alt_bug_ft"]:
                            autopilot[parts[1]] = float(value)  # Explicitly float
                            if DEBUG_FSUIPC_MESSAGES:
                                logger.debug(f"AUTOPILOT_SNAPSHOT {sink_key}: {value} → {float(value)}")
                        elif "deg" in parts[1] or "ft" in parts[1] or "fpm" in parts[1]:
                            autopilot[parts[1]] = float(value)
                        else:
                            autopilot[parts[1]] = bool(value)

            # Debug: Mostrar grupo autopilot completo si hay datos
            if DEBUG_FSUIPC_MESSAGES and autopilot:
                logger.debug(f"Autopilot group being sent: {autopilot}")

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

            # Build nuevos grupos
            radios = {}
            indicators_additional = {}
            levers_additional = {}
            autopilot_additional = {}
            environment_additional = {}

            # Build radios group
            for sink_key, shirley_key in _RADIOS_SINK_TO_SHIRLEY.items():
                if sink_key in self._radios_data:
                    parts = shirley_key.split('.')
                    if len(parts) == 3:  # frequencyHz.com1
                        if parts[1] not in radios:
                            radios[parts[1]] = {}
                        radios[parts[1]][parts[2]] = self._radios_data[sink_key]
                    elif len(parts) == 2:  # transponderCode
                        radios[parts[1]] = self._radios_data[sink_key]

            # Build indicators additional group
            for sink_key, shirley_key in _INDICATORS_ADDITIONAL_SINK_TO_SHIRLEY.items():
                if sink_key in self._indicators_data:
                    parts = shirley_key.split('.')
                    if len(parts) == 3:  # engineRpm.engine1
                        if parts[1] not in indicators_additional:
                            indicators_additional[parts[1]] = {}
                        indicators_additional[parts[1]][parts[2]] = self._indicators_data[sink_key]

            # Build levers additional group
            for sink_key, shirley_key in _LEVERS_ADDITIONAL_SINK_TO_SHIRLEY.items():
                if sink_key in self._levers_data:
                    parts = shirley_key.split('.')
                    if len(parts) == 3:  # throttlePercentOpen.engine1
                        if parts[1] not in levers_additional:
                            levers_additional[parts[1]] = {}
                        levers_additional[parts[1]][parts[2]] = self._levers_data[sink_key]
                    elif len(parts) == 2:
                        levers_additional[parts[1]] = self._levers_data[sink_key]

            # Build autopilot additional group
            for sink_key, shirley_key in _AUTOPILOT_SINK_TO_SHIRLEY.items():
                if sink_key in self._autopilot_data:
                    parts = shirley_key.split('.')
                    if len(parts) == 2:
                        value = self._autopilot_data[sink_key]
                        if "deg" in parts[1] or "ft" in parts[1] or "fpm" in parts[1]:
                            autopilot[parts[1]] = float(value)
                        else:
                            autopilot[parts[1]] = bool(value)

            # Handle altitudeMode separately (enum logic)
            if "alt_hold_on" in self._autopilot_data and self._autopilot_data["alt_hold_on"]:
                autopilot["altitudeMode"] = "altitudeHold"
            elif "vs_hold_on" in self._autopilot_data and self._autopilot_data["vs_hold_on"]:
                autopilot["altitudeMode"] = "verticalSpeed"
            else:
                autopilot["altitudeMode"] = "disabled"

            # Build environment additional group
            for sink_key, shirley_key in _ENVIRONMENT_ADDITIONAL_SINK_TO_SHIRLEY.items():
                if sink_key in self._environment_data:
                    parts = shirley_key.split('.')
                    if len(parts) == 2:
                        environment_additional[parts[1]] = self._environment_data[sink_key]

            # CRITICAL: Ensure pos and att are added to output
            if pos:
                out["position"] = pos
                if DEBUG_FSUIPC_MESSAGES:
                    logger.debug(f"Added position to output: {len(pos)} fields")
            else:
                if DEBUG_FSUIPC_MESSAGES:
                    logger.warning("Position dict is empty!")

            if att:
                out["attitude"] = att
                if DEBUG_FSUIPC_MESSAGES:
                    logger.debug(f"Added attitude to output: {len(att)} fields")
            else:
                if DEBUG_FSUIPC_MESSAGES:
                    logger.warning("Attitude dict is empty!")

            # ALTERNATIVA FINAL: Forzar tipos correctos para campos problemáticos
            if autopilot:
                # Force correct types for problematic fields
                if "magneticHeadingBugDeg" in autopilot:
                    autopilot["magneticHeadingBugDeg"] = float(autopilot["magneticHeadingBugDeg"])
                if "altitudeBugFt" in autopilot:
                    autopilot["altitudeBugFt"] = float(autopilot["altitudeBugFt"])

                if DEBUG_FSUIPC_MESSAGES:
                    logger.debug(f"Autopilot after type forcing: {autopilot}")

            # Add non-empty groups to output
            if lights: out["lights"] = lights
            if systems: out["systems"] = systems
            if autopilot: out["autopilot"] = autopilot
            if levers: out["levers"] = levers
            if indicators: out["indicators"] = indicators
            if environment: out["environment"] = environment
            if simulation: out["simulation"] = simulation

            # Add nuevos grupos al output
            if radios: out["radiosNavigation"] = radios
            if indicators_additional:
                if "indicators" not in out:
                    out["indicators"] = {}
                out["indicators"].update(indicators_additional)
            if levers_additional:
                if "levers" not in out:
                    out["levers"] = {}
                out["levers"].update(levers_additional)
            if autopilot_additional:
                if "autopilot" not in out:
                    out["autopilot"] = {}
                out["autopilot"].update(autopilot_additional)
            if environment_additional:
                if "environment" not in out:
                    out["environment"] = {}
                out["environment"].update(environment_additional)

            # Validar datos críticos antes de enviar
            if pos.get("latitudeDeg") is not None:
                if not validate_position_data(pos.get("latitudeDeg"), pos.get("longitudeDeg"), pos.get("mslAltitudeFt")):
                    logger.warning(f"Invalid position data detected: lat={pos.get('latitudeDeg')}, lon={pos.get('longitudeDeg')}")

            # Official Debug: Show complete JSON when debug enabled
            if DEBUG_FSUIPC_MESSAGES:
                logger.debug("Complete JSON to Shirley:")
                logger.debug(json.dumps(out, indent=2))
                logger.debug(f"JSON groups: {list(out.keys())}")
                if out:
                    total_fields = sum(len(group) if isinstance(group, dict) else 1 for group in out.values())
                    logger.debug(f"Total fields: {total_fields}")

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
                logger.info(f"Connecting to FSUIPC at {self.url}")
                async with websockets.connect(
                    self.url,
                    max_size=None,
                    subprotocols=["fsuipc"],
                    open_timeout=4,
                    ping_interval=None
                ) as ws:
                    self.ws = ws
                    logger.info(f"Connected to FSUIPC (subprotocol={ws.subprotocol})")

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
                    logger.info(f"Declared {len(READ_SIGNALS)} FSUIPC offsets")

                    # Start continuous reading from FSUIPC with fixed interval (ms)
                    read_msg = {
                        "command": "offsets.read",
                        "name": "flightData",
                        "interval": int(SEND_INTERVAL * 1000)  # 250 ms if SEND_INTERVAL=0.25
                    }
                    await ws.send(json.dumps(read_msg))
                    logger.info(f"Started reading FSUIPC offsets every {int(SEND_INTERVAL*1000)} ms")

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
                logger.error(f"FSUIPC connection error: {e!r}. Reconnecting in 2s...")
                await asyncio.sleep(2)

    def _handle_incoming(self, msg: str):
        global FIRST_PAYLOAD
        try:
            data = json.loads(msg)
        except json.JSONDecodeError:
            return

        # Debug log
        if DEBUG_FSUIPC_MESSAGES or FIRST_PAYLOAD:
            logger.debug(f"FSUIPC received: {data}")
            FIRST_PAYLOAD = False

        # before detecting payload:
        if "command" in data and "success" in data and not any(k in data for k in ("data","values","offsets")):
            if not data.get("success"):
                logger.error(f"FSUIPC command error: {data.get('errorMessage')}")
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
        auto_radios_kwargs = {}
        auto_indicators_kwargs = {}
        auto_autopilot_kwargs = {}

        for key, cfg in READ_SIGNALS.items():
            if key not in payload:
                continue
            
            # Skip offsets ya procesados manualmente
            if key in ["IASraw_U32", "VSraw", "MagVar_U32", "BARO_0332_U32", "BARO_0330_U32",
                      "LIGHTS_BITS32", "BATTERY_MAIN", "PITOT_HEAT_U32", "brakeLeftU",
                      "brakeRightU", "parkingBrakeU", "flapsHandle", "gearHandle",
                      "aircraftNameStr", "LatitudeDeg", "LongitudeDeg", "AltitudeM", "GroundAltRaw",
                      "BankRaw", "PitchRaw", "HeadingTrueRaw", "AP_HDG_BUG", "AP_ALT_BUG"]:
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
            elif sink_group == "radios" and val is not None and sink_field not in auto_radios_kwargs:
                auto_radios_kwargs[sink_field] = val
            elif sink_group == "indicators" and val is not None and sink_field not in auto_indicators_kwargs:
                auto_indicators_kwargs[sink_field] = val
            elif sink_group == "autopilot" and val is not None and sink_field not in auto_autopilot_kwargs:
                auto_autopilot_kwargs[sink_field] = val

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

        # === AUTOPILOT BUGS (procesamiento manual) ===
        autopilot_manual_kwargs = {}

        if "AP_HDG_BUG" in payload:
            hdg_bug = heading_bug_to_deg(payload["AP_HDG_BUG"])
            if hdg_bug is not None:
                autopilot_manual_kwargs["hdg_bug_deg"] = hdg_bug
                if DEBUG_FSUIPC_MESSAGES:
                    logger.debug(f"AUTOPILOT HDG Bug: {payload['AP_HDG_BUG']} → {hdg_bug}")

        if "AP_ALT_BUG" in payload:
            alt_bug = alt_bug_to_feet(payload["AP_ALT_BUG"])
            if alt_bug is not None:
                autopilot_manual_kwargs["alt_bug_ft"] = alt_bug
                if DEBUG_FSUIPC_MESSAGES:
                    logger.debug(f"AUTOPILOT ALT Bug: {payload['AP_ALT_BUG']} → {alt_bug}")

        if autopilot_manual_kwargs:
            asyncio.create_task(self.sim_data.update_autopilot_partial(**autopilot_manual_kwargs))
            if DEBUG_FSUIPC_MESSAGES:
                logger.debug(f"Autopilot manual kwargs: {autopilot_manual_kwargs}")

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
        if auto_radios_kwargs:
            asyncio.create_task(self.sim_data.update_radios_partial(**auto_radios_kwargs))
        if auto_indicators_kwargs:
            asyncio.create_task(self.sim_data.update_indicators_partial(**auto_indicators_kwargs))
        if auto_autopilot_kwargs:
            asyncio.create_task(self.sim_data.update_autopilot_partial(**auto_autopilot_kwargs))

        self.last_data_received_time = time.time()

    async def write_offset(self, address: int, value: int, *, size: int, dtype: str = "int") -> bool:
        if not self.ws:
            logger.warning("FSUIPC write failed: not connected")
            return False
        msg = {
            "command": "offsets.write",
            "values": [
                {"address": address, "type": dtype, "size": size, "value": int(value)}
            ]
        }
        try:
            await self.ws.send(json.dumps(msg))
            logger.debug(f"Wrote to FSUIPC offset 0x{address:04X}: {value}")
            return True
        except Exception as e:
            logger.error(f"FSUIPC write error: {e!r}")
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
        logger.info(f"Shirley client connected: {client_info} (path={request_path})")

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
            logger.warning(f"Rejected Shirley client {client_info}: invalid path {request_path}")
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
            logger.info(f"Shirley client disconnected: {client_info}")

    async def _handle_command(self, cmd: dict) -> bool:
        name = (cmd.get("name") or cmd.get("control") or "").strip()
        value = cmd.get("value", 0)
        spec = WRITE_COMMANDS.get(name)
        if not spec:
            logger.warning(f"Unknown command received: {cmd}")
            return False
        try:
            raw = spec["encode"](value) if callable(spec.get("encode")) else value
            if spec["type"] == "offset":
                ok = await self.fsuipc.write_offset(spec["address"], int(raw), size=spec["size"], dtype=spec["dtype"])
                logger.info(f"Command: {name} = {value} (raw={raw}) {'succeeded' if ok else 'failed'}")
                return ok
            logger.error(f"Unsupported write type for command {name}: {spec['type']}")
            return False
        except Exception as e:
            logger.error(f"Error handling command {cmd}: {e!r}")
            return False

    async def broadcast_loop(self):
        try:
            while True:
                snapshot = await self.sim_data.get_snapshot()

                # Official Debug: Show broadcast info
                if DEBUG_FSUIPC_MESSAGES:
                    logger.debug(f"Broadcasting to {len(self.connections)} clients")
                    if not snapshot:
                        logger.warning("Empty snapshot detected!")

                # DEBUG: Verificar que no hay keys prohibidas
                if any(key in snapshot for key in ["type", "reads", "writes"]):
                    logger.error(f"Snapshot contains prohibited keys: {list(snapshot.keys())}")

                msg = json.dumps(snapshot)
                stale = []
                for ws in list(self.connections):
                    try:
                        await ws.send(msg)
                    except websockets.exceptions.ConnectionClosed:
                        stale.append(ws)
                    except Exception as e:
                        logger.error(f"Shirley broadcast send error: {e}")
                        stale.append(ws)
                for ws in stale:
                    if ws in self.connections:
                        self.connections.remove(ws)
                await asyncio.sleep(self.send_interval)
        except asyncio.CancelledError:
            logger.info("Shirley broadcast stopped")

    async def run(self):
        # Start server and broadcast loop
        self.server = await websockets.serve(self.handler, self.host, self.port)
        logger.info(f"Shirley WebSocket server listening on ws://{self.host}:{self.port}{self.path}")
        broadcast_task = asyncio.create_task(self.broadcast_loop())

        try:
            # Keep the server running indefinitely
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            logger.info("Shirley server stopping...")
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
        logger.info("\nBridge shutting down.")