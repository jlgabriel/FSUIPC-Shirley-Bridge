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

# Publicar o no las frecuencias COM/NAV.
#
# Apagado por defecto, y no por gusto: la Shirley en uso las rechaza con
# "radiosNavigation: Unrecognized key(s) in object: 'frequencyHz',
# 'standbyFrequencyHz'", aunque el esquema publicado del repo sim-interface las
# define así en v2.12 y en v2.13 — o sea que la Shirley desplegada va con un
# esquema anterior al publicado. Como cada grupo es .strict(), mandarlas marca
# todo el feed como inválido y se pierde también el transponder, que sí acepta.
#
# Ponerlo en true cuando Shirley actualice; las frecuencias se leen igual y se
# pueden escribir, sólo no se publican.
PUBLISH_RADIO_FREQUENCIES = os.getenv("PUBLISH_RADIO_FREQUENCIES", "false").lower() in ("true", "1", "yes")

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
PARKING_BRAKE_THRESHOLD = 1000
ZERO_THRESHOLD_EPSILON = 1e-6
POSITION_CHANGE_EPSILON = 1e-7

# Barometric pressure validation ranges (raw values)
BARO_RAW_MIN = 12800  # ~800 mb
BARO_RAW_MAX = 17600  # ~1100 mb

# Time and frequency constants
MILLISECONDS_PER_SECOND = 1000
SECONDS_PER_MINUTE = 60.0
MINUTES_PER_HOUR = 60.0

# Pressure conversion constants
MB_TO_INHG_FACTOR = 0.02953  # millibar to inches of mercury conversion factor

# FSUIPC bit masks
FSUIPC_SIGNED_16BIT_MASK = 0xFFFF
FSUIPC_SIGNED_16BIT_OFFSET = 0x10000

# Throttle max value
FSUIPC_THROTTLE_MAX = 16384

# ===================== DATA VALIDATORS =====================
"""
Validation functions for ensuring data integrity.
All validators return True if valid, False otherwise.
They accept None and return False for None values.
"""

def validate_in_range(value: Optional[float], min_val: float, max_val: float,
                      allow_none: bool = True) -> bool:
    """
    Validate that a numeric value is within a specified range.

    Args:
        value: The value to validate
        min_val: Minimum acceptable value (inclusive)
        max_val: Maximum acceptable value (inclusive)
        allow_none: If True, None values are considered valid

    Returns:
        True if value is valid, False otherwise

    Examples:
        >>> validate_in_range(50.0, 0.0, 100.0)
        True
        >>> validate_in_range(150.0, 0.0, 100.0)
        False
        >>> validate_in_range(None, 0.0, 100.0, allow_none=True)
        True
        >>> validate_in_range(None, 0.0, 100.0, allow_none=False)
        False
    """
    if value is None:
        return allow_none
    try:
        val = float(value)
        return min_val <= val <= max_val
    except (TypeError, ValueError):
        return False


def validate_latitude(lat: Optional[float]) -> bool:
    """Validate latitude (-90 to +90 degrees)."""
    return validate_in_range(lat, -90.0, 90.0, allow_none=False)


def validate_longitude(lon: Optional[float]) -> bool:
    """Validate longitude (-180 to +180 degrees)."""
    return validate_in_range(lon, -180.0, 180.0, allow_none=False)


def validate_altitude(alt_ft: Optional[float]) -> bool:
    """
    Validate altitude in feet.

    Accepts values from -1500 ft (Dead Sea, Death Valley) to 60000 ft (typical max).
    """
    return validate_in_range(alt_ft, -1500.0, 60000.0, allow_none=True)


def validate_speed(speed_kts: Optional[float]) -> bool:
    """
    Validate speed in knots.

    Accepts values from 0 to 600 kts (covers most GA and commercial aircraft).
    """
    return validate_in_range(speed_kts, 0.0, 600.0, allow_none=True)


def validate_vertical_speed(vs_fpm: Optional[float]) -> bool:
    """
    Validate vertical speed in feet per minute.

    Typical range: -6000 to +6000 fpm for most aircraft.
    """
    return validate_in_range(vs_fpm, -6000.0, 6000.0, allow_none=True)


def validate_heading(heading_deg: Optional[float]) -> bool:
    """Validate heading in degrees (0 to 360)."""
    return validate_in_range(heading_deg, 0.0, 360.0, allow_none=True)


def validate_pitch(pitch_deg: Optional[float]) -> bool:
    """Validate pitch angle (-90 to +90 degrees)."""
    return validate_in_range(pitch_deg, -90.0, 90.0, allow_none=True)


def validate_roll(roll_deg: Optional[float]) -> bool:
    """Validate roll angle (-180 to +180 degrees)."""
    return validate_in_range(roll_deg, -180.0, 180.0, allow_none=True)


def validate_temperature(temp_celsius: Optional[float]) -> bool:
    """
    Validate temperature in Celsius.

    Accepts -60°C to +60°C (covers atmospheric conditions at cruise altitudes).
    """
    return validate_in_range(temp_celsius, -60.0, 60.0, allow_none=True)


def validate_pressure(pressure_inhg: Optional[float]) -> bool:
    """
    Validate barometric pressure in inches of mercury.

    Normal range: 28.00 to 31.00 inHg
    Extended range for extreme conditions: 27.00 to 32.00 inHg
    """
    return validate_in_range(pressure_inhg, 27.0, 32.0, allow_none=True)


def validate_rpm(rpm: Optional[float]) -> bool:
    """
    Validate engine RPM.

    Accepts 0 to 10000 RPM (covers piston and turboprop engines).
    """
    return validate_in_range(rpm, 0.0, 10000.0, allow_none=True)


def validate_n1_percent(n1: Optional[float]) -> bool:
    """Validate N1 percentage (0 to 110% - allows slight over-range)."""
    return validate_in_range(n1, 0.0, 110.0, allow_none=True)


def validate_percentage(percent: Optional[float]) -> bool:
    """Validate generic percentage (0 to 100)."""
    return validate_in_range(percent, 0.0, 100.0, allow_none=True)


def validate_com_frequency(freq_khz: Optional[int]) -> bool:
    """
    Validate COM radio frequency in kHz.

    Aviation COM range: 118.000 to 136.975 MHz (118000 to 136975 kHz).
    """
    if freq_khz is None:
        return True
    try:
        freq = int(freq_khz)
        return 118000 <= freq <= 136975
    except (TypeError, ValueError):
        return False


def validate_nav_frequency(freq_khz: Optional[int]) -> bool:
    """
    Validate NAV radio frequency in kHz.

    Aviation NAV range: 108.000 to 117.950 MHz (108000 to 117950 kHz).
    """
    if freq_khz is None:
        return True
    try:
        freq = int(freq_khz)
        return 108000 <= freq <= 117950
    except (TypeError, ValueError):
        return False


def validate_transponder_code(code: Optional[int]) -> bool:
    """
    Validate transponder squawk code.

    Valid range: 0000 to 7777 (octal digits only).
    """
    if code is None:
        return True
    try:
        c = int(code)
        if c < 0 or c > 7777:
            return False
        # Check that all digits are octal (0-7)
        str_code = str(c).zfill(4)
        return all(d in '01234567' for d in str_code)
    except (TypeError, ValueError):
        return False


def validate_throttle_command(value: Optional[float]) -> bool:
    """
    Validate throttle command value.

    Accepts:
    - Normalized range: -1.0 to +1.0 (fractional values for percentage)
    - Raw range: Integer values from -16384 to +16384 (FSUIPC raw format)

    Values between 1.0 and 16384 that are not close to integers are rejected
    to avoid ambiguity between normalized and raw formats.
    """
    if value is None:
        return False
    try:
        val = float(value)

        # Normalized range (fractional values -1.0 to 1.0)
        if -1.0 <= val <= 1.0:
            return True

        # Raw range (must be integer-like values outside normalized range)
        # Allow values > 1.0 or < -1.0 only if they're close to integers
        if abs(val - round(val)) < 0.01:  # Close enough to an integer
            if -FSUIPC_THROTTLE_MAX <= val <= FSUIPC_THROTTLE_MAX:
                return True

        return False
    except (TypeError, ValueError):
        return False


def validate_gear_command(value: Optional[int]) -> bool:
    """
    Validate gear handle command.

    Accepts only 0 (retracted) or 1 (down).
    Values must be exactly 0 or 1 (or floats very close to them like 0.0, 1.0).
    """
    if value is None:
        return False
    try:
        val = float(value)
        # Check if value is very close to 0 or 1
        if abs(val - 0.0) < 0.01:
            return True
        if abs(val - 1.0) < 0.01:
            return True
        return False
    except (TypeError, ValueError):
        return False


def sanitize_float(value: Any, default: float = 0.0) -> float:
    """
    Safely convert a value to float, returning default if conversion fails.

    Args:
        value: Value to convert
        default: Default value to return on failure

    Returns:
        Float value or default

    Examples:
        >>> sanitize_float("123.45")
        123.45
        >>> sanitize_float("invalid", 0.0)
        0.0
        >>> sanitize_float(None, 10.0)
        10.0
    """
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def sanitize_int(value: Any, default: int = 0) -> int:
    """
    Safely convert a value to int, returning default if conversion fails.

    Args:
        value: Value to convert
        default: Default value to return on failure

    Returns:
        Integer value or default
    """
    if value is None:
        return default
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def sanitize_bool(value: Any, default: bool = False) -> bool:
    """
    Safely convert a value to bool, returning default if conversion fails.

    Args:
        value: Value to convert
        default: Default value to return on failure

    Returns:
        Boolean value or default
    """
    if value is None:
        return default
    try:
        return bool(value)
    except (TypeError, ValueError):
        return default


# ===================== ESCRITURA HACIA EL SIMULADOR =====================
# Nombre del grupo de offsets declarado ante FSUIPC. offsets.write exige un
# grupo ya declarado y referencia los offsets por nombre, nunca por dirección.
FSUIPC_GROUP = "flightData"

# Único conjunto de offsets que este puente escribe, cerrado a propósito.
#
# Escribir un offset que MSFS 2024 no aplica no devuelve error: FSUIPC guarda
# el valor en su propio buffer y después lo sirve como si fuera telemetría,
# incluso después de desconectar, reconectar y volver a declarar el grupo. Un
# solo intento contra 0x0E8C deja al puente informando una temperatura exterior
# inventada durante el resto de la sesión. Por eso el puente nunca escribe un
# offset "a ver si anda": si no está acá, va por evento o no va.
#
# Los dos que están acá se verificaron en vivo con espejo sobre un King Air
# 350i (0x0BFC movió 0x0BE0; 0x2440 movió 0x337C).
WRITABLE_OFFSETS = frozenset({"FLAPS_INDEX", "PROP_DEICE"})


# --- Codificadores de valor -> parámetro del evento ---

def _enc_pct_16383(v):
    """Porcentaje 0..100 -> 0..16383."""
    return int(round(clamp(float(v), 0.0, 100.0) / 100.0 * FSUIPC_SCALE_FACTOR_16383))

def _enc_pct_flag(v):
    """Porcentaje -> 0/1. Para mandos que MSFS 2024 modela como binarios."""
    return 1 if float(v) >= 50.0 else 0

def _enc_flag(v):
    return 1 if bool(v) else 0

def _enc_deg(v):
    return int(round(float(v))) % 360

def _enc_int(v):
    return int(round(float(v)))

def _enc_freq_bcd(khz):
    """Frecuencia en kHz -> BCD de 4 dígitos con el 1 inicial sobreentendido.

    Los campos frequencyHz del schema están acotados a [108000, 136975] pese al
    nombre: son kHz. 121.850 MHz llega como 121850 y sale como 0x2185.
    """
    digits = int(round(float(khz) / 10.0))     # 121850 -> 12185
    return int(f"{digits:05d}"[-4:], 16)       # "12185" -> "2185" -> 0x2185

def _enc_xpdr_bcd(code):
    """Código de transponder -> BCD. 1200 se manda como 0x1200."""
    return int(f"{int(code):04d}", 16)

def _enc_kohlsman(inhg):
    """Pulgadas de mercurio -> milibares * 16, que es lo que toma KOHLSMAN_SET."""
    return int(round(float(inhg) / MB_TO_INHG_FACTOR * FSUIPC_SCALE_FACTOR_16))


# --- Tabla declarativa de escritura ---
#
# Mecanismos, en orden de preferencia:
#
#   "event"  -> vars.calc con código de calculadora RPN. Es el camino
#               recomendado: no toca el buffer de offsets de FSUIPC, así que un
#               comando que el avión no puede honrar no deja nada sucio.
#   "toggle" -> el evento sólo conmuta. Se lee el estado actual desde
#               READ_SIGNALS y se dispara únicamente si hace falta, para que la
#               operación siga siendo idempotente.
#   "offset" -> escritura directa por nombre. Sólo para WRITABLE_OFFSETS.
#   "custom" -> necesita lógica propia (flaps por detente, enums, hora zulú).
#
# La columna 'verified' dice si el mecanismo se ejercitó contra MSFS 2024 con
# un espejo que confirmara que el simulador lo aplicó de verdad.
SET_FIELDS = {
    # --- levers ---
    "levers.flapsHandlePercentDown": {
        "kind": "custom", "handler": "flaps", "verified": True,
        "note": "convierte el porcentaje a índice de detente cuando el avión informa sus detentes",
    },
    "levers.speedBrakesHandlePercentDeployed": {
        "kind": "event", "code": "{v} (>K:SPOILERS_SET)", "encode": _enc_pct_16383, "verified": True,
    },
    "levers.landingGearHandlePercentDown": {
        "kind": "event", "code": "{v} (>K:GEAR_SET)", "encode": _enc_pct_flag, "verified": True,
        "note": "en tierra MSFS 2024 se niega a replegar con peso sobre ruedas; no es una falla del puente",
    },
    "levers.carburetorHeatLeverPercentHot.engine1": {
        "kind": "event", "code": "{v} (>K:ANTI_ICE_SET_ENG1)", "encode": _enc_pct_flag, "verified": True,
        "note": "MSFS 2024 modela el aire caliente al carburador como el antihielo del motor: binario",
    },
    "levers.carburetorHeatLeverPercentHot.engine2": {
        "kind": "event", "code": "{v} (>K:ANTI_ICE_SET_ENG2)", "encode": _enc_pct_flag, "verified": False,
    },

    # --- autopilot ---
    "autopilot.isAutopilotEngaged": {
        "kind": "event", "code": "(>K:AUTOPILOT_ON)", "code_off": "(>K:AUTOPILOT_OFF)", "verified": True,
    },
    "autopilot.isHeadingSelectEnabled": {
        "kind": "toggle", "code": "1 (>K:AP_PANEL_HEADING_HOLD)",
        "state": ("autopilot", "hdg_select_on"), "verified": True,
    },
    "autopilot.magneticHeadingBugDeg": {
        "kind": "event", "code": "{v} (>K:2:HEADING_BUG_SET)", "encode": _enc_deg, "verified": True,
    },
    "autopilot.altitudeBugFt": {
        "kind": "custom", "handler": "altitude_bug", "verified": True,
        "note": "el evento no fija el valor, mueve el preselector 1000 ft hacia el objetivo; "
                "hay que converger",
    },
    "autopilot.targetVerticalSpeedUpFpm": {
        "kind": "event", "code": "{v} (>K:2:AP_VS_VAR_SET_ENGLISH)", "encode": _enc_int, "verified": True,
    },
    "autopilot.altitudeMode": {
        "kind": "custom", "handler": "altitude_mode", "verified": True,
    },
    "autopilot.isFlightDirectorEngaged": {
        "kind": "toggle", "code": "(>K:TOGGLE_FLIGHT_DIRECTOR)",
        "state": ("autopilot", "flight_director_on"), "verified": True,
        "note": "con el piloto automático puesto el avión lo fuerza encendido y no deja apagarlo; "
                "no es una falla del puente",
    },
    "autopilot.shouldLevelWings": {
        "kind": "toggle", "code": "(>K:AP_WING_LEVELER)",
        "state": ("autopilot", "wing_leveler_on"), "verified": False,
    },

    # --- radiosNavigation ---
    "radiosNavigation.standbyFrequencyHz.com1": {
        "kind": "event", "code": "{v} (>K:COM_STBY_RADIO_SET)", "encode": _enc_freq_bcd, "verified": True,
    },
    "radiosNavigation.standbyFrequencyHz.com2": {
        "kind": "event", "code": "{v} (>K:COM2_STBY_RADIO_SET)", "encode": _enc_freq_bcd, "verified": True,
    },
    "radiosNavigation.standbyFrequencyHz.nav1": {
        "kind": "event", "code": "{v} (>K:NAV1_STBY_SET)", "encode": _enc_freq_bcd, "verified": True,
    },
    "radiosNavigation.transponderCode": {
        "kind": "event", "code": "{v} (>K:XPNDR_SET)", "encode": _enc_xpdr_bcd, "verified": True,
    },
    "radiosNavigation.comShouldSwapFrequencies.com1": {
        "kind": "event", "code": "(>K:COM_STBY_RADIO_SWAP)", "when_true_only": True, "verified": False,
    },
    "radiosNavigation.comShouldSwapFrequencies.com2": {
        "kind": "event", "code": "(>K:COM2_STBY_RADIO_SWAP)", "when_true_only": True, "verified": False,
    },

    # --- lights ---
    "lights.navigationLightsSwitchOn": {
        "kind": "event", "code": "{v} (>K:NAV_LIGHTS_SET)", "encode": _enc_flag, "verified": True,
    },
    "lights.landingLightsSwitchOn": {
        "kind": "event", "code": "{v} (>K:LANDING_LIGHTS_SET)", "encode": _enc_flag, "verified": True,
    },
    "lights.taxiLightsSwitchOn": {
        "kind": "event", "code": "{v} (>K:TAXI_LIGHTS_SET)", "encode": _enc_flag, "verified": True,
    },
    "lights.strobeLightsSwitchOn": {
        "kind": "event", "code": "{v} (>K:STROBES_SET)", "encode": _enc_flag, "verified": True,
    },

    # --- systems ---
    "systems.parkingBrakeOn": {
        "kind": "event", "code": "{v} (>K:PARKING_BRAKE_SET)", "encode": _enc_flag, "verified": True,
    },
    "systems.pitotHeatSwitchOn": {
        "kind": "event", "code": "{v} (>K:PITOT_HEAT_SET)", "encode": _enc_flag, "verified": True,
    },
    "systems.batteryOn.main": {
        "kind": "toggle", "code": "(>K:TOGGLE_MASTER_BATTERY)",
        "state": ("systems", "battery_main_on"), "verified": True,
    },
    "systems.propHeatSwitchOn": {
        "kind": "offset", "offset": "PROP_DEICE", "encode": _enc_flag, "verified": True,
        "note": "único camino: no hay evento de deshielo de hélice en el catálogo de presets, y "
                "TOGGLE_STRUCTURAL_DEICE mueve el deshielo de célula, no éste",
    },

    # --- indicators ---
    "indicators.altimeterSettingInchesMercury": {
        "kind": "event", "code": "{v} (>K:KOHLSMAN_SET)", "encode": _enc_kohlsman, "verified": True,
    },

    # --- environment (sólo tiempo: el clima no se puede fijar, ver docs) ---
    "environment.zuluTimeHours": {
        "kind": "custom", "handler": "zulu_time", "verified": False,
    },
    "environment.dayOfYear": {
        "kind": "event", "code": "{v} (>K:ZULU_DAY_SET)", "encode": _enc_int, "verified": False,
    },

    # --- simulation / freezes ---
    "simulation.isPaused": {
        "kind": "event", "code": "(>K:PAUSE_ON)", "code_off": "(>K:PAUSE_OFF)", "verified": False,
    },
    "freezes.positionFreezeEnabled": {
        "kind": "toggle", "code": "(>K:FREEZE_LATITUDE_LONGITUDE_TOGGLE)",
        "state": ("raw", "LL_FREEZE"), "verified": True,
    },
}

# Campos del schema que existen pero que MSFS 2024 no permite fijar. Se
# rechazan con el motivo, en vez de aceptarlos en silencio y no hacer nada.
SET_FIELDS_UNSUPPORTED = {
    "levers.propBetaEnabled": "PROP BETA es de sólo lectura y no hay evento para fijarla",
    "systems.governorSwitchOn": "sólo helicópteros; sin verificar",
    "systems.totalEnergyAudioSwitchOn": "TOGGLE_VARIOMETER_SWITCH sólo conmuta y no hay offset para leer el estado",
    "simulation.isCrashed": "sin mecanismo en MSFS 2024",
    "simulation.shouldResetFlight": "sin mecanismo en MSFS 2024",
    "simulation.simSpeedRatio": "SIM_RATE_INCR/DECR sólo va por pasos; 0x0C1A es de sólo lectura",
}

# Grupos de primer nivel de SetSimData. Sirven para reconocer un mensaje
# entrante: Shirley manda el objeto anidado pelado, sin ninguna envoltura.
SET_SIMDATA_GROUPS = frozenset(
    [p.split(".")[0] for p in SET_FIELDS] +
    [p.split(".")[0] for p in SET_FIELDS_UNSUPPORTED] +
    ["position", "attitude", "failures", "environment", "freezes"]
)


def _flatten_set_simdata(body: Any, prefix: str = ""):
    """Recorre el objeto anidado de SetSimData y produce (ruta, valor) por hoja."""
    if not isinstance(body, dict):
        return
    for key, value in body.items():
        path = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            yield from _flatten_set_simdata(value, path)
        elif value is not None:
            yield path, value


def _unsupported_reason(path: str) -> str:
    """Por qué el puente no escribe este campo. Se responde el motivo en vez de
    aceptar el campo en silencio y no hacer nada."""
    if path in SET_FIELDS_UNSUPPORTED:
        return SET_FIELDS_UNSUPPORTED[path]
    if path.startswith("failures."):
        return "los offsets de fallas son de sólo lectura y no hay evento para provocarlas"
    if path.startswith("environment."):
        return ("el clima no se puede fijar en MSFS 2024: las escrituras se aceptan y se "
                "devuelven en el eco, pero el simulador nunca las aplica")
    if path.startswith(("position.", "attitude.")):
        return "teletransportar la aeronave requiere slew o freeze; no implementado"
    return "campo no soportado por el puente"


def fmt_ft(value: Any) -> str:
    try:
        return f"{float(value):.0f} ft"
    except (TypeError, ValueError):
        return repr(value)


def _calc_tag(path: str) -> str:
    """Etiqueta corta y estable para correlacionar la respuesta de vars.calc."""
    return "set_" + path.replace(".", "_")

# ===================== CAPABILITIES FUNCTIONS =====================
def compute_capabilities_writes():
    """
    Rutas de SetSimData que el puente sabe escribir.

    Returns:
        Lista ordenada de rutas con puntos, tal como llegan de Shirley.

    Example:
        >>> writes = compute_capabilities_writes()
        >>> 'levers.landingGearHandlePercentDown' in writes
        True
    """
    return sorted(SET_FIELDS.keys())

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
    for key, cfg in READ_SIGNALS.items():
        sink = cfg.get("sink")
        if isinstance(sink, tuple) and len(sink) == 2:
            g, f = sink
            reads.append({"key": key, "group": g, "field": f})
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
# Cada entrada declara un offset FSUIPC, la transformación que lo lleva a las
# unidades de Shirley y el 'sink' (grupo interno, campo) donde se deposita.
#
# Direcciones, tamaños y SimVars verificados contra
# "FSUIPC7 Offsets Status.pdf" v0.8.4 para MSFS 2024. El SimVar real va en el
# comentario porque varios offsets de la era FSX apuntan hoy a otra cosa
# distinta de la que sugiere su nombre histórico: 0x08A0 no es la presión de
# admisión sino el flujo de combustible, 0x08B8 no es la EGT sino la
# temperatura de aceite, y 0x0898 no son RPM de pistón sino N1 de turbina.
READ_SIGNALS = {
    # --- Position ---
    "LatitudeDeg":   {"address": 0x0560, "type": "lat",   "size": 8, "sink": ("gps", "latitude")},       # PLANE LATITUDE
    "LongitudeDeg":  {"address": 0x0568, "type": "lon",   "size": 8, "sink": ("gps", "longitude")},      # PLANE LONGITUDE
    "AltitudeM":     {"address": 0x6020, "type": "float", "size": 8, "sink": ("gps", "alt_msl_meters")}, # GPS POSITION ALT, m

    "GroundSpeedKts":{"address": 0x02B4, "type": "uint",  "size": 4, "transform": "gs_u32_to_kts", "sink": ("gps", "ground_speed_kts")},  # GROUND VELOCITY, m/s*65536

    # --- Airspeeds / VS ---
    "IASraw_U32":   {"address": 0x02BC, "type": "uint", "size": 4, "transform": "knots128_to_kts", "sink": ("gps", "ias_kts")},  # AIRSPEED INDICATED, kt*128
    "VSraw":        {"address": 0x02C8, "type": "int",  "size": 4, "transform": "vs_raw_to_fpm",   "sink": ("gps", "vs_fpm_raw")},  # VERTICAL SPEED, m/s*256

    # --- AGL via ground altitude ---
    "GroundAltRaw": {"address": 0x0020, "type": "int", "size": 4, "transform": "meters256_to_m", "sink": ("gps", "ground_alt_m")},  # GROUND ALTITUDE, m*256

    # --- Attitude ---
    "HeadingTrueRaw":{"address": 0x0580, "type": "uint",  "size": 4, "transform": "raw_hdg_to_deg", "sink": ("att", "heading_deg")},      # PLANE HEADING DEGREES TRUE
    "PitchRaw":      {"address": 0x0578, "type": "int",   "size": 4, "transform": "raw_ang_to_deg_pitch", "sink": ("att", "pitch_deg")},  # PLANE PITCH DEGREES (negativo = morro arriba)
    "BankRaw":       {"address": 0x057C, "type": "int",   "size": 4, "transform": "raw_ang_to_deg_roll", "sink": ("att", "roll_deg")},    # PLANE BANK DEGREES (positivo = alabeo a izquierda)

    # --- Magnetic variation ---
    "MagVar_U32": {"address": 0x02A0, "type": "uint", "size": 2, "transform": "u32_signed16_to_magdeg", "sink": ("att", "mag_var_deg")},  # MAGVAR, int16

    # --- Luces (bitmask de 2 bytes) ---
    "LIGHTS_BITS32": {"address": 0x0D0C, "type": "uint", "size": 2, "sink": None},  # LIGHT NAV/BEACON/LANDING/TAXI/STROBE/...

    # --- Sistemas ---
    "BATTERY_MAIN":   {"address": 0x281C, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("systems", "battery_main_on")},  # ELECTRICAL MASTER BATTERY
    "PITOT_HEAT_U32": {"address": 0x029C, "type": "uint", "size": 1, "transform": "nonzero_to_bool", "sink": ("systems", "pitot_heat_on")},    # PITOT HEAT
    "PROP_DEICE":     {"address": 0x2440, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("systems", "prop_heat_on")},     # PROP DEICE SWITCH:1

    # --- BARO ---
    # 0x0330 es el altímetro 1 y es el que corresponde por defecto; 0x0332 es
    # el segundo altímetro (paneles con dos, tipo G1000) y sólo se usa como
    # respaldo. La preferencia se resuelve en _derive_environment().
    "BARO_0330_U32": {"address": 0x0330, "type": "uint", "size": 2, "transform": "u32_baro_to_inhg", "sink": None},  # KOHLSMAN SETTING MB, mb*16
    "BARO_0332_U32": {"address": 0x0332, "type": "uint", "size": 2, "transform": "u32_baro_to_inhg", "sink": None},  # KOHLSMAN SETTING MB:2

    # --- Freno de estacionamiento ---
    # El schema de Shirley no tiene un campo para los pedales de freno, así que
    # 0x0BC4/0x0BC6 ya no se declaran: sólo agregaban tráfico.
    "parkingBrakeU": {"address": 0x0BC8, "type": "uint", "size": 2, "transform": "u32_to_bool_parking", "sink": ("systems", "parking_brake_on")},  # BRAKE PARKING POSITION, 0/32767

    # --- Controles (flaps/gear en %) ---
    "flapsHandle":   {"address": 0x0BDC, "type": "uint", "size": 4, "transform": "u32_to_pct_16383", "sink": ("levers", "flaps_pct")},  # FLAPS HANDLE PERCENT
    "gearHandle":    {"address": 0x0BE8, "type": "uint", "size": 4, "transform": "u32_to_pct_16383", "sink": ("levers", "gear_pct")},   # GEAR HANDLE POSITION

    # Auxiliares del write path: permiten traducir un porcentaje de flaps al
    # índice de detente del avión en vuelo (ver _encode_flaps).
    "FLAPS_INDEX":     {"address": 0x0BFC, "type": "uint", "size": 1, "sink": None},  # FLAPS HANDLE INDEX
    "FLAPS_NUM_POS":   {"address": 0x3BF8, "type": "uint", "size": 2, "sink": None},  # FLAPS NUM HANDLE POSITIONS (sin contar recogido)
    "FLAPS_DETENT_INC":{"address": 0x3BFA, "type": "uint", "size": 2, "sink": None},  # incremento de 0x0BDC por detente
    "LL_FREEZE":       {"address": 0x3540, "type": "uint", "size": 1, "sink": None},  # IS LATITUDE LONGITUDE FREEZE ON

    # --- Nombre aeronave ---
    "aircraftNameStr": {"address": 0x3D00, "type": "string", "size": 256, "sink": ("simulation", "aircraft_name")},  # TITLE

    # === RADIOS/NAVIGATION ===
    "COM1_FREQ":      {"address": 0x034E, "type": "uint", "size": 2, "transform": "bcd_to_freq_com_official", "sink": ("radios", "com1_active_khz")},
    "COM1_STANDBY":   {"address": 0x311A, "type": "uint", "size": 2, "transform": "bcd_to_freq_com_official", "sink": ("radios", "com1_standby_khz")},
    "COM2_FREQ":      {"address": 0x3118, "type": "uint", "size": 2, "transform": "bcd_to_freq_com_official", "sink": ("radios", "com2_active_khz")},
    "COM2_STANDBY":   {"address": 0x311C, "type": "uint", "size": 2, "transform": "bcd_to_freq_com_official", "sink": ("radios", "com2_standby_khz")},
    "NAV1_FREQ":      {"address": 0x0350, "type": "uint", "size": 2, "transform": "bcd_to_freq_nav_official", "sink": ("radios", "nav1_active_khz")},
    "NAV1_STANDBY":   {"address": 0x311E, "type": "uint", "size": 2, "transform": "bcd_to_freq_nav_official", "sink": ("radios", "nav1_standby_khz")},
    "TRANSPONDER":    {"address": 0x0354, "type": "uint", "size": 2, "transform": "bcd_to_xpdr_official", "sink": ("radios", "transponder_code")},

    # === INDICATORS ===
    # No hay RPM de pistón utilizable por offset legacy en MSFS 2024: 0x0898 y
    # 0x0930 son N1 de turbina, y 0x089C/0x0934 no están documentados. Para un
    # avión de pistón hay que mapear GENERAL ENG RPM:n con MyOffsets.
    "ENGINE1_N1":     {"address": 0x2010, "type": "float", "size": 8, "sink": ("indicators", "engine1_n1_pct")},  # TURB ENG CORRECTED N1:1, %
    "ENGINE2_N1":     {"address": 0x2110, "type": "float", "size": 8, "sink": ("indicators", "engine2_n1_pct")},  # TURB ENG CORRECTED N1:2, %
    "ENGINE1_MANIFOLD": {"address": 0x08C0, "type": "uint", "size": 2, "transform": "manifold_to_inhg", "sink": ("indicators", "engine1_manifold_inhg")},  # RECIP ENG MANIFOLD PRESSURE:1, inHg*1024
    "ENGINE2_MANIFOLD": {"address": 0x0958, "type": "uint", "size": 2, "transform": "manifold_to_inhg", "sink": ("indicators", "engine2_manifold_inhg")},  # RECIP ENG MANIFOLD PRESSURE:2
    "ENGINE1_EGT":    {"address": 0x08BE, "type": "uint", "size": 2, "transform": "egt_to_celsius", "sink": ("indicators", "engine1_egt_c")},  # GENERAL ENG EXHAUST GAS TEMPERATURE:1, 16384 = 860 °C
    "ENGINE2_EGT":    {"address": 0x0956, "type": "uint", "size": 2, "transform": "egt_to_celsius", "sink": ("indicators", "engine2_egt_c")},  # GENERAL ENG EXHAUST GAS TEMPERATURE:2
    "STALL_WARNING":  {"address": 0x036C, "type": "uint", "size": 1, "transform": "nonzero_to_bool", "sink": ("indicators", "stall_warning_on")},  # STALL WARNING

    # === LEVERS ===
    "THROTTLE1_POS":  {"address": 0x088C, "type": "int", "size": 2, "transform": "throttle_to_percent", "sink": ("levers", "throttle1_pct")},  # GENERAL ENG THROTTLE LEVER POSITION:1, -4096..16384
    "THROTTLE2_POS":  {"address": 0x0924, "type": "int", "size": 2, "transform": "throttle_to_percent", "sink": ("levers", "throttle2_pct")},  # ...:2
    "PROP1_POS":      {"address": 0x088E, "type": "int", "size": 2, "transform": "prop_to_percent", "sink": ("levers", "prop1_pct")},          # GENERAL ENG PROPELLER LEVER POSITION:1
    "PROP2_POS":      {"address": 0x0926, "type": "int", "size": 2, "transform": "prop_to_percent", "sink": ("levers", "prop2_pct")},          # ...:2
    "MIXTURE1_POS":   {"address": 0x0890, "type": "int", "size": 2, "transform": "mixture_to_percent", "sink": ("levers", "mixture1_pct")},    # GENERAL ENG MIXTURE LEVER POSITION:1, 0..16384
    "MIXTURE2_POS":   {"address": 0x0928, "type": "int", "size": 2, "transform": "mixture_to_percent", "sink": ("levers", "mixture2_pct")},    # ...:2
    "CARB_HEAT1":     {"address": 0x08B2, "type": "uint", "size": 2, "transform": "carb_heat_to_percent", "sink": ("levers", "carb_heat1_pct")},  # GENERAL ENG ANTI ICE POSITION:1 (binario en MSFS 2024)
    "SPEEDBRAKE_POS": {"address": 0x0BD0, "type": "uint", "size": 4, "transform": "u32_to_pct_16383", "sink": ("levers", "speedbrake_pct")},   # SPOILERS HANDLE POSITION

    # === AUTOPILOT ===
    "AP_MASTER":      {"address": 0x07BC, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("autopilot", "master_on")},        # AUTOPILOT MASTER
    "AP_HDG_HOLD":    {"address": 0x07C8, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("autopilot", "hdg_select_on")},    # AUTOPILOT HEADING LOCK
    "AP_ALT_HOLD":    {"address": 0x07D0, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("autopilot", "alt_hold_on")},      # AUTOPILOT ALTITUDE LOCK
    "AP_WING_LEVELER":{"address": 0x07C0, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("autopilot", "wing_leveler_on")},  # AUTOPILOT WING LEVELER
    "AP_FLIGHT_DIR":  {"address": 0x2EE0, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("autopilot", "flight_director_on")},  # AUTOPILOT FLIGHT DIRECTOR ACTIVE
    "AP_HDG_BUG":     {"address": 0x07CC, "type": "uint", "size": 2, "transform": "heading_bug_to_deg", "sink": ("autopilot", "hdg_bug_deg")},   # AUTOPILOT HEADING LOCK DIR, deg*65536/360
    "AP_ALT_BUG":     {"address": 0x07D4, "type": "uint", "size": 4, "transform": "alt_bug_to_feet", "sink": ("autopilot", "alt_bug_ft")},       # AUTOPILOT ALTITUDE LOCK VAR, m*65536
    "AP_VS_HOLD":     {"address": 0x07EC, "type": "uint", "size": 4, "transform": "nonzero_to_bool", "sink": ("autopilot", "vs_hold_on")},       # AUTOPILOT VERTICAL HOLD
    "AP_VS_TARGET":   {"address": 0x07F2, "type": "int", "size": 2, "transform": "vs_target_to_fpm", "sink": ("autopilot", "vs_target_fpm")},    # AUTOPILOT VERTICAL HOLD VAR, ft/min

    # === ENVIRONMENT ===
    "WIND_SPEED":     {"address": 0x0E90, "type": "uint", "size": 2, "transform": "wind_to_kts", "sink": ("environment", "wind_speed_kts")},   # AMBIENT WIND VELOCITY, kt
    "WIND_DIR":       {"address": 0x0E92, "type": "uint", "size": 2, "transform": "wind_dir_to_deg", "sink": ("environment", "wind_dir_deg")}, # AMBIENT WIND DIRECTION, deg*65536/360
    "OUTSIDE_TEMP":   {"address": 0x0E8C, "type": "int", "size": 2, "transform": "temp_to_celsius", "sink": ("environment", "outside_temp_c")},# AMBIENT TEMPERATURE, °C*256
}

# Normaliza: si alguna señal no define 'sink', déjalo en None
for _k, _cfg in READ_SIGNALS.items():
    _cfg.setdefault("sink", None)

# ===================== DATA TRANSFORM FUNCTIONS =====================
def raw_ang_to_deg(raw):
    return fs_angle_deg(raw) if raw is not None else None

def raw_ang_to_deg_pitch(raw):
    # FSUIPC da el cabeceo positivo hacia abajo; Shirley lo quiere positivo
    # hacia arriba (pitchAngleDegUp).
    v = fs_angle_deg(raw) if raw is not None else None
    return -v if v is not None else None

def raw_ang_to_deg_roll(raw):
    # FSUIPC da el alabeo positivo hacia la izquierda; Shirley lo quiere
    # positivo hacia la derecha (rollAngleDegRight).
    v = fs_angle_deg(raw) if raw is not None else None
    return -v if v is not None else None

def raw_hdg_to_deg(raw):    return (fs_heading_true_deg(raw) % 360.0) if raw is not None else None
def mps_to_mps(raw):        return fs_ground_speed_mps(raw) if raw is not None else None

# ===================== TRANSFORM REGISTRY =====================

TRANSFORMS = {
    "raw_ang_to_deg": raw_ang_to_deg,
    "raw_ang_to_deg_pitch": raw_ang_to_deg_pitch,
    "raw_ang_to_deg_roll": raw_ang_to_deg_roll,
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

def manifold_to_inhg(raw):
    """0x08C0 RECIP ENG MANIFOLD PRESSURE: pulgadas de mercurio * 1024."""
    try:
        return float(raw) / 1024.0
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform manifold_to_inhg failed for {raw}: {e}")
        return None

def egt_to_celsius(raw):
    """0x08BE GENERAL ENG EXHAUST GAS TEMPERATURE: 16384 = 860 °C.

    No es Rankine ni Kelvin: es una escala lineal directa a grados Celsius.
    """
    try:
        return float(raw) * 860.0 / 16384.0
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform egt_to_celsius failed for {raw}: {e}")
        return None

def temp_to_celsius(raw):
    """0x0E8C AMBIENT TEMPERATURE: grados Celsius * 256, con signo.

    No es Kelvin*256 — esa escala ni siquiera entra en el int16 declarado.
    Fuera de rango devuelve None para que el campo se omita del snapshot; antes
    devolvía 15.0, con lo cual el puente publicaba una temperatura inventada
    como si fuera telemetría.
    """
    try:
        celsius = float(raw) / FSUIPC_SCALE_FACTOR_256
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform temp_to_celsius failed for {raw}: {e}")
        return None

    if not validate_temperature(celsius):
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform temp_to_celsius out of range: {raw} -> {celsius}")
        return None
    return celsius

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

def _lever_to_percent(raw, lo_pct, name):
    """Palancas FSUIPC: 16384 = 100 %. El signo se respeta.

    Las palancas de gases y paso de hélice llegan hasta -4096 (reversa / beta),
    que es -25 %. Sumarles 65536 para 'arreglar el signo' convertía la reversa
    en ~300 % de potencia, que es lo que hacía la versión anterior.
    """
    try:
        pct = (int(raw) / float(FSUIPC_THROTTLE_MAX)) * 100.0
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform {name} failed for {raw}: {e}")
        return None
    return clamp(pct, lo_pct, 100.0)

def throttle_to_percent(raw):
    """0x088C GENERAL ENG THROTTLE LEVER POSITION: -4096..16384."""
    return _lever_to_percent(raw, -25.0, "throttle_to_percent")

def mixture_to_percent(raw):
    """0x0890 GENERAL ENG MIXTURE LEVER POSITION: 0..16384."""
    return _lever_to_percent(raw, 0.0, "mixture_to_percent")

def prop_to_percent(raw):
    """0x088E GENERAL ENG PROPELLER LEVER POSITION: -4096..16384."""
    return _lever_to_percent(raw, -25.0, "prop_to_percent")

def carb_heat_to_percent(raw):
    """0x08B2 GENERAL ENG ANTI ICE POSITION: binario en MSFS 2024.

    El schema de Shirley lo tipa como porcentaje, pero MSFS 2024 modela el aire
    caliente al carburador como el interruptor de antihielo del motor, que sólo
    vale 0 o 1.
    """
    try:
        return 100.0 if int(raw) else 0.0
    except (TypeError, ValueError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform carb_heat_to_percent failed for {raw}: {e}")
        return None

def heading_bug_to_deg(raw):
    """0x07CC AUTOPILOT HEADING LOCK DIR: grados * 65536 / 360."""
    try:
        return ((float(raw) * FSUIPC_TURN_FRACTION_TO_DEG) / FSUIPC_SCALE_FACTOR_65536) % 360.0
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform heading_bug_to_deg failed for {raw}: {e}")
        return None

def alt_bug_to_feet(raw):
    """0x07D4 AUTOPILOT ALTITUDE LOCK VAR: metros * 65536, no pies.

    Se leía en crudo, así que el selector de altitud del piloto automático se
    publicaba con un valor 65536/3.28 veces mayor que el real.
    """
    try:
        return (float(raw) / FSUIPC_SCALE_FACTOR_65536) * METERS_TO_FEET
    except (TypeError, ValueError, ZeroDivisionError) as e:
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Transform alt_bug_to_feet failed for {raw}: {e}")
        return None

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
    "manifold_to_inhg": manifold_to_inhg,
    "egt_to_celsius": egt_to_celsius,
    "temp_to_celsius": temp_to_celsius,
    "fuel_to_gallons": fuel_to_gallons,
    "oil_pressure_to_psi": oil_pressure_to_psi,
    "throttle_to_percent": throttle_to_percent,
    "mixture_to_percent": mixture_to_percent,
    "prop_to_percent": prop_to_percent,
    "carb_heat_to_percent": carb_heat_to_percent,
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
# Los dos diccionarios que siguen son documentación, no configuración: nada los
# lee. Los campos de gps y att necesitan cálculos derivados (AGL a partir de la
# altitud del terreno, rumbo magnético a partir de la declinación, derrota a
# partir de posiciones sucesivas) y por eso get_snapshot() los arma a mano.
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

# ===================== Mapping sinks -> claves de Shirley =====================
# (grupo interno, campo) -> (ruta en el schema de Shirley, tipo)
#
# El tipo va explícito a propósito. La versión anterior lo deducía buscando
# "deg"/"ft"/"fpm" como subcadena del nombre del campo; la prueba distinguía
# mayúsculas y los nombres del schema son camelCase, así que nunca acertaba y
# los selectores numéricos del piloto automático terminaban convertidos a bool
# y publicados como 1.0.
#
# Los grupos "gps" y "att" no aparecen acá: sus campos necesitan cálculos
# derivados (AGL, rumbo magnético, derrota) y se arman a mano en get_snapshot().
SINK_TO_SHIRLEY = {
    # --- lights ---
    ("lights", "nav_on"):            ("lights.navigationLightsSwitchOn", "bool"),
    ("lights", "landing_on"):        ("lights.landingLightsSwitchOn", "bool"),
    ("lights", "taxi_on"):           ("lights.taxiLightsSwitchOn", "bool"),
    ("lights", "strobe_on"):         ("lights.strobeLightsSwitchOn", "bool"),

    # --- systems ---
    ("systems", "pitot_heat_on"):    ("systems.pitotHeatSwitchOn", "bool"),
    ("systems", "battery_main_on"):  ("systems.batteryOn.main", "bool"),
    ("systems", "parking_brake_on"): ("systems.parkingBrakeOn", "bool"),
    ("systems", "prop_heat_on"):     ("systems.propHeatSwitchOn", "bool"),

    # --- autopilot ---
    # alt_hold_on y vs_hold_on no se mapean directamente: alimentan el enum
    # altitudeMode, que se resuelve en get_snapshot().
    ("autopilot", "master_on"):           ("autopilot.isAutopilotEngaged", "bool"),
    ("autopilot", "hdg_select_on"):       ("autopilot.isHeadingSelectEnabled", "bool"),
    ("autopilot", "flight_director_on"):  ("autopilot.isFlightDirectorEngaged", "bool"),
    ("autopilot", "wing_leveler_on"):     ("autopilot.shouldLevelWings", "bool"),
    ("autopilot", "hdg_bug_deg"):         ("autopilot.magneticHeadingBugDeg", "float"),
    ("autopilot", "alt_bug_ft"):          ("autopilot.altitudeBugFt", "float"),
    ("autopilot", "vs_target_fpm"):       ("autopilot.targetVerticalSpeedUpFpm", "float"),

    # --- levers ---
    ("levers", "flaps_pct"):         ("levers.flapsHandlePercentDown", "float"),
    ("levers", "gear_pct"):          ("levers.landingGearHandlePercentDown", "float"),
    ("levers", "speedbrake_pct"):    ("levers.speedBrakesHandlePercentDeployed", "float"),
    ("levers", "throttle1_pct"):     ("levers.throttlePercentOpen.engine1", "float"),
    ("levers", "throttle2_pct"):     ("levers.throttlePercentOpen.engine2", "float"),
    ("levers", "mixture1_pct"):      ("levers.mixtureLeverPercentRich.engine1", "float"),
    ("levers", "mixture2_pct"):      ("levers.mixtureLeverPercentRich.engine2", "float"),
    ("levers", "prop1_pct"):         ("levers.propellerLeverPercentCoarse.prop1", "float"),
    ("levers", "prop2_pct"):         ("levers.propellerLeverPercentCoarse.prop2", "float"),
    ("levers", "carb_heat1_pct"):    ("levers.carburetorHeatLeverPercentHot.engine1", "float"),

    # --- indicators ---
    ("indicators", "altimeter_inhg"):        ("indicators.altimeterSettingInchesMercury", "float"),
    ("indicators", "stall_warning_on"):      ("indicators.stallWarningOn", "bool"),
    ("indicators", "engine1_n1_pct"):        ("indicators.engineN1Percent.engine1", "float"),
    ("indicators", "engine2_n1_pct"):        ("indicators.engineN1Percent.engine2", "float"),
    ("indicators", "engine1_manifold_inhg"): ("indicators.manifoldPressureInchesMercury.engine1", "float"),
    ("indicators", "engine2_manifold_inhg"): ("indicators.manifoldPressureInchesMercury.engine2", "float"),
    ("indicators", "engine1_egt_c"):         ("indicators.exhaustGasDegC.engine1", "float"),
    ("indicators", "engine2_egt_c"):         ("indicators.exhaustGasDegC.engine2", "float"),

    # --- environment ---
    ("environment", "pressure_inhg"):   ("environment.seaLevelPressureInchesMercury", "float"),
    ("environment", "wind_speed_kts"):  ("environment.aircraftWindSpeedKts", "float"),
    ("environment", "wind_dir_deg"):    ("environment.aircraftWindHeadingDeg", "float"),
    ("environment", "outside_temp_c"):  ("environment.groundTemperatureDegC", "float"),

    # --- radiosNavigation ---
    ("radios", "com1_active_khz"):   ("radiosNavigation.frequencyHz.com1", "int"),
    ("radios", "com1_standby_khz"):  ("radiosNavigation.standbyFrequencyHz.com1", "int"),
    ("radios", "com2_active_khz"):   ("radiosNavigation.frequencyHz.com2", "int"),
    ("radios", "com2_standby_khz"):  ("radiosNavigation.standbyFrequencyHz.com2", "int"),
    ("radios", "nav1_active_khz"):   ("radiosNavigation.frequencyHz.nav1", "int"),
    ("radios", "nav1_standby_khz"):  ("radiosNavigation.standbyFrequencyHz.nav1", "int"),
    ("radios", "transponder_code"):  ("radiosNavigation.transponderCode", "int"),

    # --- simulation ---
    ("simulation", "aircraft_name"): ("simulation.aircraftName", "str"),
}

# Campos que se leen pero no se publican. Un campo que Shirley no reconoce
# invalida el grupo entero, así que es preferible omitirlo antes que perder el
# grupo completo por su culpa.
SUPPRESSED_PATHS = frozenset() if PUBLISH_RADIO_FREQUENCIES else frozenset({
    "radiosNavigation.frequencyHz.com1",
    "radiosNavigation.frequencyHz.com2",
    "radiosNavigation.frequencyHz.nav1",
    "radiosNavigation.standbyFrequencyHz.com1",
    "radiosNavigation.standbyFrequencyHz.com2",
    "radiosNavigation.standbyFrequencyHz.nav1",
})

_SINK_COERCERS = {
    "bool":  bool,
    "float": float,
    "int":   int,
    "str":   str,
}

def _assign_path(out: Dict[str, Any], path: str, value: Any, kind: str) -> None:
    """Deposita value en out siguiendo una ruta con puntos, coercionando el tipo.

    Si el valor no se puede convertir al tipo declarado se descarta: es
    preferible omitir un campo a mandarle a Shirley uno con el tipo equivocado,
    porque cada grupo del schema es .strict() y un solo campo mal tipado
    invalida el grupo entero.
    """
    coerce = _SINK_COERCERS.get(kind, float)
    try:
        typed = coerce(value)
    except (TypeError, ValueError):
        if DEBUG_FSUIPC_MESSAGES:
            logger.debug(f"Snapshot: descarto {path}={value!r}, no es {kind}")
        return

    parts = path.split(".")
    node = out
    for part in parts[:-1]:       # crea el grupo y los objetos intermedios
        node = node.setdefault(part, {})
    node[parts[-1]] = typed

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

        # Grupos de datos. Las claves de cada uno son los 'sink field' de
        # READ_SIGNALS; SINK_TO_SHIRLEY las traduce a rutas del schema.
        self._lights_data = {}      # nav_on, landing_on, taxi_on, strobe_on
        self._systems_data = {}     # pitot_heat_on, battery_main_on, parking_brake_on, prop_heat_on
        self._autopilot_data = {}   # master_on, hdg_select_on, hdg_bug_deg, alt_bug_ft, vs_target_fpm...
        self._levers_data = {}      # flaps_pct, gear_pct, throttle1_pct, mixture1_pct...
        self._indicators_data = {}  # altimeter_inhg, stall_warning_on, engine1_n1_pct...
        self._environment_data = {} # pressure_inhg, wind_speed_kts, outside_temp_c...
        self._radios_data = {}      # frecuencias COM/NAV, transponder
        self._simulation_data = {}  # aircraft_name

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

    async def update_environment_partial(self, **kwargs):
        async with self._lock:
            for key, value in kwargs.items():
                if value is not None:
                    self._environment_data[key] = value
            self.last_timestamp = iso_utc_ms()

    async def update_simulation_partial(self, **kwargs):
        async with self._lock:
            for key, value in kwargs.items():
                if value is not None:
                    self._simulation_data[key] = value
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

            if pos:
                out["position"] = pos
            elif DEBUG_FSUIPC_MESSAGES:
                logger.debug("Snapshot sin grupo position")

            if att:
                out["attitude"] = att
            elif DEBUG_FSUIPC_MESSAGES:
                logger.debug("Snapshot sin grupo attitude")

            # Resto de los grupos: una sola pasada sobre SINK_TO_SHIRLEY, con el
            # tipo declarado en la tabla. Antes había dos pasadas sobre dos
            # diccionarios homónimos, la segunda de las cuales deshacía la
            # primera.
            sources = {
                "lights":      self._lights_data,
                "systems":     self._systems_data,
                "autopilot":   self._autopilot_data,
                "levers":      self._levers_data,
                "indicators":  self._indicators_data,
                "environment": self._environment_data,
                "radios":      self._radios_data,
                "simulation":  self._simulation_data,
            }
            for (group, field), (path, kind) in SINK_TO_SHIRLEY.items():
                if path in SUPPRESSED_PATHS:
                    continue
                data = sources.get(group)
                if data is None or field not in data or data[field] is None:
                    continue
                _assign_path(out, path, data[field], kind)

            # altitudeMode es un enum y sale de dos banderas distintas. Sólo se
            # publica si hay dato real de al menos una: mandar "disabled" sin
            # haber leído nada afirma algo que no se sabe.
            alt_hold = self._autopilot_data.get("alt_hold_on")
            vs_hold = self._autopilot_data.get("vs_hold_on")
            if alt_hold is not None or vs_hold is not None:
                if alt_hold:
                    mode = "altitudeHold"
                elif vs_hold:
                    mode = "verticalSpeed"
                else:
                    mode = "disabled"
                out.setdefault("autopilot", {})["altitudeMode"] = mode

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

    async def get_sink(self, group: str, field: str) -> Any:
        """Último valor conocido de un sink. Lo usa el write path para que los
        eventos que sólo conmutan (batería, director de vuelo) sean idempotentes."""
        async with self._lock:
            source = {
                "lights": self._lights_data,
                "systems": self._systems_data,
                "autopilot": self._autopilot_data,
                "levers": self._levers_data,
                "indicators": self._indicators_data,
                "environment": self._environment_data,
                "radios": self._radios_data,
                "simulation": self._simulation_data,
            }.get(group)
            return None if source is None else source.get(field)

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
    Cliente WebSocket contra el FSUIPC WebSocket Server.

    Declara el grupo de offsets, se suscribe a lecturas periódicas, transforma
    lo que llega y lo deposita en SimData. Es también el lado que escribe:
    eventos de control por vars.calc y, para los pocos offsets verificados,
    offsets.write.
    """
    # El servidor corta conexiones de forma intermitente y sin trama de cierre:
    # un comando se queda sin respuesta y el socket resulta estar cerrado. La
    # reconexión con re-declaración y re-suscripción no es un refinamiento,
    # es un requisito.
    RECONNECT_BACKOFF_MIN_S = 1.0
    RECONNECT_BACKOFF_MAX_S = 15.0
    CALC_TIMEOUT_S = 3.0
    WRITE_ERROR_WINDOW_S = 0.8

    # Una suscripción puede morir sin que se caiga la conexión. El caso normal
    # es arrancar el puente antes de cargar el vuelo: offsets.read se rechaza
    # con NoFlightSim y el servidor no la reanuda solo cuando el simulador
    # aparece. El socket queda abierto y sano, y sin vigilancia el puente se
    # queda callado para siempre sobre una conexión que funciona.
    DATA_STALL_S = 8.0
    WATCHDOG_PERIOD_S = 4.0

    # El preselector de altitud se mueve de a 1000 ft por evento, así que ir de
    # 0 a 50 000 ft son 50 pasos más el ajuste fino de a 100.
    ALT_BUG_MAX_STEPS = 70
    ALT_BUG_TOLERANCE_FT = 60.0

    def __init__(self, sim_data: SimData, url: str = FSUIPC_WS_URL):
        self.sim_data = sim_data
        self.url = url
        self.ws: Optional[Any] = None  # WebSocket client connection
        self.last_data_received_time: Optional[float] = None

        # Último valor crudo de cada señal declarada. FSUIPC responde sólo con
        # lo que cambió desde la respuesta anterior, así que todo lo que
        # necesite el cuadro completo — los detentes de flaps, la preferencia
        # entre los dos altímetros — tiene que leerse de acá y no del payload
        # suelto que acaba de llegar.
        self.raw_state: Dict[str, Any] = {}

        # Futuros esperando la respuesta a un comando, indexados por
        # (command, name).
        self._waiters: Dict[tuple, list] = {}

        # Serializa las escrituras: puede haber más de un cliente de Shirley
        # conectado, y una trama WebSocket a medio enviar no se puede
        # intercalar con otra.
        self._write_lock = asyncio.Lock()

    @property
    def connected(self) -> bool:
        return self.ws is not None

    # ---------- ciclo de conexión ----------

    async def run(self):
        backoff = self.RECONNECT_BACKOFF_MIN_S
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
                    backoff = self.RECONNECT_BACKOFF_MIN_S
                    logger.info(f"Connected to FSUIPC (subprotocol={ws.subprotocol})")
                    await self._declare_and_subscribe(ws)

                    watchdog = asyncio.create_task(self._watch_data_flow(ws))
                    try:
                        async for msg in ws:
                            if isinstance(msg, bytes):
                                try:
                                    msg = msg.decode('utf-8', 'ignore')
                                except Exception:
                                    continue
                            if isinstance(msg, str):
                                await self._handle_incoming(msg)
                    finally:
                        watchdog.cancel()

                logger.warning("FSUIPC cerró la conexión; reconectando")
            except asyncio.CancelledError:
                raise
            except Exception as e:
                logger.error(f"FSUIPC connection error: {e!r}")
            finally:
                self.ws = None
                self.raw_state.clear()
                self._fail_pending_waiters()

            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, self.RECONNECT_BACKOFF_MAX_S)

    async def _declare_and_subscribe(self, ws, quiet: bool = False):
        """Declara el grupo y arranca la suscripción. Se repite en cada
        reconexión — el servidor no recuerda nada de la sesión anterior — y
        también cuando el vigía detecta que dejaron de llegar datos."""
        declare_msg = {
            "command": "offsets.declare",
            "name": FSUIPC_GROUP,
            "offsets": [
                {"name": key, "address": cfg["address"], "type": cfg["type"], "size": cfg["size"]}
                for key, cfg in READ_SIGNALS.items()
            ],
        }
        read_msg = {
            "command": "offsets.read",
            "name": FSUIPC_GROUP,
            "interval": int(SEND_INTERVAL * MILLISECONDS_PER_SECOND),
        }
        async with self._write_lock:
            await ws.send(json.dumps(declare_msg))
            await ws.send(json.dumps(read_msg))

        log = logger.debug if quiet else logger.info
        log(f"Declared {len(READ_SIGNALS)} FSUIPC offsets as '{FSUIPC_GROUP}', "
            f"reading every {read_msg['interval']} ms")

    async def _watch_data_flow(self, ws):
        """Rehace la suscripción mientras no lleguen datos.

        Una suscripción rechazada no se reanuda sola. El caso corriente es
        arrancar el puente antes de cargar el vuelo: FSUIPC contesta un único
        NoFlightSim y después se queda callado, con el socket abierto y en
        perfecto estado, así que ni el bucle de lectura ni la reconexión se
        enteran de nada.
        """
        started = time.time()
        complained = False
        while True:
            await asyncio.sleep(self.WATCHDOG_PERIOD_S)
            if self.ws is not ws:
                return

            last = self.last_data_received_time or started
            if time.time() - last < self.DATA_STALL_S:
                if complained:
                    logger.info("FSUIPC volvió a entregar datos")
                    complained = False
                continue

            if not complained:
                logger.warning(
                    f"Sin datos de FSUIPC hace más de {self.DATA_STALL_S:.0f}s; "
                    f"rehaciendo la suscripción (¿el vuelo todavía no cargó?)"
                )
                complained = True

            try:
                await self._declare_and_subscribe(ws, quiet=True)
            except Exception as e:
                logger.error(f"No se pudo rehacer la suscripción: {e!r}")
                return

    # ---------- correlación de respuestas ----------

    def _resolve_waiter(self, data: dict):
        key = (data.get("command"), data.get("name"))
        for fut in self._waiters.pop(key, []):
            if not fut.done():
                fut.set_result(data)

    def _fail_pending_waiters(self):
        for futs in self._waiters.values():
            for fut in futs:
                if not fut.done():
                    fut.set_result(None)
        self._waiters.clear()

    async def _await_response(self, command: str, name: str, timeout: float) -> Optional[dict]:
        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        key = (command, name)
        self._waiters.setdefault(key, []).append(fut)
        try:
            return await asyncio.wait_for(fut, timeout)
        except asyncio.TimeoutError:
            return None
        finally:
            pending = self._waiters.get(key)
            if pending and fut in pending:
                pending.remove(fut)
                if not pending:
                    self._waiters.pop(key, None)

    # ---------- lectura ----------

    async def _handle_incoming(self, msg: str):
        global FIRST_PAYLOAD
        try:
            data = json.loads(msg)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return

        if DEBUG_FSUIPC_MESSAGES or FIRST_PAYLOAD:
            logger.debug(f"FSUIPC received: {data}")
            FIRST_PAYLOAD = False

        # Respuesta a un comando: despierta a quien la esté esperando. Una
        # escritura correcta responde con un offsets.read, así que sigue de
        # largo hacia el parseo de datos; sólo las respuestas sin datos
        # terminan acá.
        if "command" in data and "success" in data:
            self._resolve_waiter(data)
            if not any(k in data for k in ("data", "values", "offsets")):
                if not data.get("success"):
                    logger.error(
                        f"FSUIPC command error ({data.get('command')}/{data.get('name')}): "
                        f"{data.get('errorCode')}: {data.get('errorMessage')}"
                    )
                return

        payload = data.get("data") or data.get("values") or data

        # Algunas versiones devuelven 'values' como lista de {name, value}
        if isinstance(payload, list):
            try:
                payload = {it["name"]: it.get("value") for it in payload if isinstance(it, dict) and "name" in it}
            except Exception:
                payload = {}

        if not isinstance(payload, dict) or not payload:
            return

        self.raw_state.update(payload)
        groups = self._decode(payload)
        await self._apply(groups)
        self.last_data_received_time = time.time()

    def _decode(self, payload: dict) -> Dict[str, Dict[str, Any]]:
        """Payload crudo -> {grupo interno: {campo: valor}}.

        Una sola pasada sobre READ_SIGNALS. La versión anterior tenía dos
        caminos de parseo en paralelo, uno a mano y otro genérico, sincronizados
        por una lista de exclusión escrita a mano: agregar una señal sin tocar
        esa lista la procesaba dos veces.
        """
        groups: Dict[str, Dict[str, Any]] = {}
        for key, cfg in READ_SIGNALS.items():
            if key not in payload:
                continue
            sink = cfg.get("sink")
            if not sink:
                continue

            val = payload[key]
            tf_name = cfg.get("transform")
            if tf_name:
                tf = TRANSFORMS.get(tf_name)
                if tf is None:
                    logger.error(f"Señal {key}: transform '{tf_name}' no está registrado")
                    continue
                val = tf(val)
            if val is None:
                continue

            group, field = sink
            groups.setdefault(group, {})[field] = val

        self._derive(payload, groups)
        return groups

    def _derive(self, payload: dict, groups: Dict[str, Dict[str, Any]]) -> None:
        """Valores que no salen de un offset solo."""
        # Luces: un único bitmask alimenta cuatro campos.
        if "LIGHTS_BITS32" in payload:
            try:
                bits = int(payload["LIGHTS_BITS32"])
            except (TypeError, ValueError):
                bits = None
            if bits is not None:
                groups.setdefault("lights", {}).update({
                    "nav_on":     bool(bits & (1 << 0)),
                    "landing_on": bool(bits & (1 << 2)),
                    "taxi_on":    bool(bits & (1 << 3)),
                    "strobe_on":  bool(bits & (1 << 4)),
                })

        # Barómetro: 0x0330 es el altímetro 1 y manda; 0x0332 es el segundo y
        # sólo entra si el primero no da un valor plausible.
        if "BARO_0330_U32" in payload or "BARO_0332_U32" in payload:
            baro = u32_baro_to_inhg(self.raw_state.get("BARO_0330_U32"))
            if baro is None or not validate_pressure(baro):
                fallback = u32_baro_to_inhg(self.raw_state.get("BARO_0332_U32"))
                baro = fallback if fallback is not None and validate_pressure(fallback) else None
            if baro is not None:
                groups.setdefault("environment", {})["pressure_inhg"] = baro
                groups.setdefault("indicators", {})["altimeter_inhg"] = baro

    async def _apply(self, groups: Dict[str, Dict[str, Any]]) -> None:
        """Vuelca los grupos decodificados en SimData.

        Se hace con await y no con create_task: los create_task sueltos no
        guardaban referencia, se podían recolectar a mitad de camino y no
        garantizaban ningún orden entre sí.
        """
        dispatch = {
            "gps":         self.sim_data.update_gps_partial,
            "att":         self.sim_data.update_att_partial,
            "lights":      self.sim_data.update_lights_partial,
            "systems":     self.sim_data.update_systems_partial,
            "autopilot":   self.sim_data.update_autopilot_partial,
            "levers":      self.sim_data.update_levers_partial,
            "indicators":  self.sim_data.update_indicators_partial,
            "environment": self.sim_data.update_environment_partial,
            "radios":      self.sim_data.update_radios_partial,
            "simulation":  self.sim_data.update_simulation_partial,
        }
        for group, kwargs in groups.items():
            update = dispatch.get(group)
            if update is None:
                logger.error(f"Grupo de sink desconocido: {group}")
                continue
            if kwargs:
                await update(**kwargs)

    # ---------- primitivas de escritura ----------

    async def calc(self, code: str, tag: str = "set") -> bool:
        """Ejecuta código de calculadora (RPN) vía vars.calc.

        Es el camino preferido para escribir: si el simulador no puede honrar
        el comando no queda nada guardado en el buffer de offsets de FSUIPC.
        """
        ws = self.ws
        if ws is None:
            logger.warning(f"vars.calc '{code}' descartado: sin conexión a FSUIPC")
            return False

        async with self._write_lock:
            try:
                await ws.send(json.dumps({"command": "vars.calc", "name": tag, "code": code, "interval": 0}))
            except Exception as e:
                logger.error(f"vars.calc '{code}' falló al enviar: {e!r}")
                return False

            reply = await self._await_response("vars.calc", tag, self.CALC_TIMEOUT_S)
        if reply is None:
            # El síntoma documentado de una conexión caída en silencio. Cerrar
            # fuerza el ciclo de reconexión en run().
            logger.error(f"vars.calc '{code}' sin respuesta en {self.CALC_TIMEOUT_S}s; forzando reconexión")
            try:
                await ws.close()
            except Exception:
                pass
            return False
        if not reply.get("success"):
            logger.error(f"vars.calc '{code}' rechazado: {reply.get('errorCode')}: {reply.get('errorMessage')}")
            return False

        logger.debug(f"vars.calc ok: {code}")
        return True

    async def write_offset(self, name: str, value: int) -> bool:
        """Escribe un offset declarado, por nombre.

        offsets.write exige el nombre del grupo y referencia los offsets por
        nombre, nunca por dirección; la versión anterior mandaba una forma
        inventada con 'values' y direcciones, que el servidor descarta sin
        emitir error alguno.

        Sólo se permiten los offsets de WRITABLE_OFFSETS: escribir uno que el
        simulador ignora envenena sus lecturas posteriores de forma permanente.
        """
        if name not in WRITABLE_OFFSETS:
            logger.error(f"Escritura bloqueada sobre '{name}': no está en WRITABLE_OFFSETS")
            return False
        ws = self.ws
        if ws is None:
            logger.warning(f"offsets.write '{name}' descartado: sin conexión a FSUIPC")
            return False

        msg = {"command": "offsets.write", "name": FSUIPC_GROUP,
               "offsets": [{"name": name, "value": int(value)}]}
        async with self._write_lock:
            try:
                await ws.send(json.dumps(msg))
            except Exception as e:
                logger.error(f"offsets.write '{name}' falló al enviar: {e!r}")
                return False

            # Una escritura correcta responde con un offsets.read; sólo se
            # recibe una respuesta offsets.write cuando falló.
            err = await self._await_response("offsets.write", FSUIPC_GROUP, self.WRITE_ERROR_WINDOW_S)
        if err is not None and not err.get("success"):
            logger.error(f"offsets.write '{name}' rechazado: {err.get('errorCode')}: {err.get('errorMessage')}")
            return False

        logger.debug(f"offsets.write ok: {name} = {value}")
        return True

    # ---------- SetSimData ----------

    async def apply_set_simdata(self, body: dict) -> list:
        """Aplica un mensaje SetSimData completo y devuelve un resultado por campo."""
        results = []
        for path, value in _flatten_set_simdata(body):
            results.append(await self.apply_set_field(path, value))
        return results

    async def apply_set_field(self, path: str, value: Any) -> dict:
        spec = SET_FIELDS.get(path)
        if spec is None:
            return {"field": path, "ok": False, "error": _unsupported_reason(path)}

        try:
            ok = await self._dispatch_set(spec, path, value)
        except Exception as e:
            logger.error(f"SetSimData {path}={value!r} falló: {e!r}")
            return {"field": path, "ok": False, "error": repr(e)}

        logger.info(f"SetSimData {path} = {value!r} -> {'ok' if ok else 'falló'}")
        if ok:
            return {"field": path, "ok": True}

        # Un fallo tiene que decir por qué. Que el comando no haya llegado al
        # simulador y que el simulador lo haya rechazado son cosas distintas, y
        # la primera es transitoria: este servidor corta la conexión cada tanto
        # sin aviso, y el comando se puede volver a mandar.
        reason = ("el comando no llegó al simulador: la conexión con FSUIPC se cortó, "
                  "se puede reintentar" if self.ws is None
                  else "el simulador no aplicó el comando")
        return {"field": path, "ok": False, "error": reason}

    async def _dispatch_set(self, spec: dict, path: str, value: Any) -> bool:
        kind = spec["kind"]

        if kind == "custom":
            handler = {
                "flaps": self._set_flaps,
                "altitude_bug": self._set_altitude_bug,
                "altitude_mode": self._set_altitude_mode,
                "zulu_time": self._set_zulu_time,
            }[spec["handler"]]
            return await handler(value)

        if kind == "offset":
            return await self.write_offset(spec["offset"], spec["encode"](value))

        if kind == "toggle":
            desired = bool(value)
            current = await self._read_sink(spec["state"])
            if current is None:
                logger.warning(f"{path}: no se conoce el estado actual, no se conmuta a ciegas")
                return False
            if bool(current) == desired:
                logger.debug(f"{path}: ya está en {desired}, no se hace nada")
                return True
            return await self.calc(spec["code"], _calc_tag(path))

        if kind == "event":
            if spec.get("when_true_only") and not value:
                return True
            code = spec["code"]
            if "code_off" in spec and not bool(value):
                code = spec["code_off"]
            if "{v}" in code:
                code = code.format(v=spec["encode"](value))
            return await self.calc(code, _calc_tag(path))

        logger.error(f"{path}: mecanismo de escritura desconocido '{kind}'")
        return False

    async def _read_sink(self, state) -> Any:
        group, field = state
        if group == "raw":
            return self.raw_state.get(field)
        return await self.sim_data.get_sink(group, field)

    # ---------- handlers propios ----------

    async def _set_flaps(self, value) -> bool:
        """Flaps por índice de detente cuando el avión informa sus detentes.

        MSFS 2024 ajusta el porcentaje a la detente más cercana. En un avión de
        tres posiciones el ajuste es tan grueso que se traga el comando entero:
        pedir 10922 cae en 8191, que en un King Air 350i o un CJ4 es justo donde
        ya estaban los flaps, así que no se movía nada ni había forma de saber
        por qué. Escribir el índice va derecho a la detente.
        """
        pct = clamp(float(value), 0.0, 100.0)
        increment = self.raw_state.get("FLAPS_DETENT_INC")
        num_positions = self.raw_state.get("FLAPS_NUM_POS")

        try:
            increment = int(increment) if increment is not None else 0
        except (TypeError, ValueError):
            increment = 0

        if increment > 0:
            index = int(round(pct / 100.0 * FSUIPC_SCALE_FACTOR_16383 / increment))
            try:
                if num_positions is not None:
                    index = int(clamp(index, 0, int(num_positions)))
            except (TypeError, ValueError):
                pass
            return await self.write_offset("FLAPS_INDEX", index)

        return await self.calc(f"{_enc_pct_16383(pct)} (>K:FLAPS_SET)", "setFlaps")

    async def _set_altitude_bug(self, value) -> bool:
        """Lleva el preselector de altitud al valor pedido, convergiendo.

        Verificado sobre un C172 G1000 en MSFS 2024: AP_ALT_VAR_SET_ENGLISH no
        fija el valor que se le manda, mueve el preselector 1000 ft *hacia* él.
        Mandarle 12000 estando en 5000 deja 6000; mandarle 3000 estando en 6000
        deja 5000. AP_ALT_VAR_INC/DEC mueven de a 100 ft.

        Por eso esto es un lazo y no una sola llamada. En un avión donde el
        evento sí fije el valor de una, la primera vuelta ya cae dentro de
        tolerancia y el lazo termina ahí. Si el avión ignora el evento, el
        guardia de progreso corta a la segunda vuelta en vez de insistir.
        """
        target = float(value)
        last = None

        for _ in range(self.ALT_BUG_MAX_STEPS):
            current = await self.sim_data.get_sink("autopilot", "alt_bug_ft")
            if current is None:
                # Sin lectura no se puede converger; queda un intento suelto.
                return await self.calc(f"{_enc_int(target)} (>K:AP_ALT_VAR_SET_ENGLISH)", "setAltBug")

            delta = target - float(current)
            if abs(delta) <= self.ALT_BUG_TOLERANCE_FT:
                return True

            if last is not None and abs(float(current) - last) < 1.0:
                logger.warning(
                    f"altitudeBugFt: el preselector no se movió desde {fmt_ft(current)}; "
                    f"esta aeronave no acepta el evento"
                )
                return False
            last = float(current)

            if abs(delta) >= 1000.0:
                code = f"{_enc_int(target)} (>K:AP_ALT_VAR_SET_ENGLISH)"
            else:
                code = "(>K:AP_ALT_VAR_INC)" if delta > 0 else "(>K:AP_ALT_VAR_DEC)"

            if not await self.calc(code, "setAltBug"):
                return False
            await self._wait_for_sink_change("autopilot", "alt_bug_ft", current)

        logger.warning(f"altitudeBugFt: no se llegó a {target} ft en "
                       f"{self.ALT_BUG_MAX_STEPS} pasos")
        return False

    async def _wait_for_sink_change(self, group: str, field: str, previous: Any,
                                    timeout: float = 1.5) -> bool:
        """Espera a que una lectura se actualice, en vez de dormir a ciegas."""
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while loop.time() < deadline:
            await asyncio.sleep(SEND_INTERVAL / 2)
            if await self.sim_data.get_sink(group, field) != previous:
                return True
        return False

    async def _set_altitude_mode(self, value) -> bool:
        mode = str(value)

        if mode in ("altitudeHold", "verticalSpeed", "disabled"):
            alt_on = bool(await self._read_sink(("autopilot", "alt_hold_on")))
            vs_on = bool(await self._read_sink(("autopilot", "vs_hold_on")))
            ok = True
            want_alt = mode == "altitudeHold"
            want_vs = mode == "verticalSpeed"
            if alt_on != want_alt:
                ok = await self.calc("1 (>K:AP_PANEL_ALTITUDE_HOLD)", "setAltHold") and ok
            if vs_on != want_vs:
                ok = await self.calc("1 (>K:AP_PANEL_VS_HOLD)", "setVsHold") and ok
            return ok

        # Sin verificar contra el simulador: no se ejercitaron en las pruebas.
        if mode == "glideSlope":
            return await self.calc("(>K:AP_APR_HOLD)", "setApr")
        if mode == "levelChange":
            return await self.calc("(>K:AP_PANEL_SPEED_HOLD)", "setFlc")

        logger.warning(f"altitudeMode '{mode}' no tiene evento genérico en MSFS 2024")
        return False

    async def _set_zulu_time(self, value) -> bool:
        hours = float(value)
        whole = int(hours) % 24
        minutes = int(round((hours - int(hours)) * MINUTES_PER_HOUR)) % 60
        ok = await self.calc(f"{whole} (>K:ZULU_HOURS_SET)", "setZuluH")
        return await self.calc(f"{minutes} (>K:ZULU_MINUTES_SET)", "setZuluM") and ok

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
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if not isinstance(data, dict):
                    continue

                body = self._as_set_simdata(data)
                if body is None:
                    logger.debug(f"Mensaje de Shirley ignorado: {list(data.keys())}")
                    continue

                results = await self.fsuipc.apply_set_simdata(body)
                if not results:
                    continue

                ack = {"type": "SetSimDataAck", "results": results}
                try:
                    await websocket.send(json.dumps(ack))
                except websockets.exceptions.ConnectionClosed:
                    break

        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            if websocket in self.connections:
                self.connections.remove(websocket)
            logger.info(f"Shirley client disconnected: {client_info}")

    @staticmethod
    def _as_set_simdata(data: dict) -> Optional[dict]:
        """Reconoce un SetSimData y devuelve su cuerpo anidado.

        Shirley manda el objeto pelado, con la misma forma anidada que SimData y
        sin ninguna clave 'type': {"levers": {"flapsHandlePercentDown": 50}}. La
        versión anterior exigía {"type": "SetSimData", "commands": [...]}, un
        formato que nadie manda, así que todo mensaje entrante se ignoraba en
        silencio.

        Se sigue aceptando la envoltura con 'type' por si un cliente propio la
        usa para depurar.
        """
        if data.get("type") == "SetSimData":
            inner = data.get("data")
            if isinstance(inner, dict):
                return inner
            return {k: v for k, v in data.items() if k in SET_SIMDATA_GROUPS}

        if any(k in SET_SIMDATA_GROUPS for k in data):
            return data

        return None

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