"""
Tests for data validation functions.

This module tests all validator functions to ensure they correctly
validate aviation data and reject invalid inputs.
"""

import pytest
from fsuipc_shirley_bridge import (
    validate_in_range,
    validate_latitude,
    validate_longitude,
    validate_altitude,
    validate_speed,
    validate_vertical_speed,
    validate_heading,
    validate_pitch,
    validate_roll,
    validate_temperature,
    validate_pressure,
    validate_rpm,
    validate_n1_percent,
    validate_percentage,
    validate_com_frequency,
    validate_nav_frequency,
    validate_transponder_code,
    validate_throttle_command,
    validate_gear_command,
    sanitize_float,
    sanitize_int,
    sanitize_bool,
)


class TestValidateInRange:
    """Tests for the generic range validator."""

    def test_valid_value_in_range(self):
        assert validate_in_range(50.0, 0.0, 100.0) is True
        assert validate_in_range(0.0, 0.0, 100.0) is True
        assert validate_in_range(100.0, 0.0, 100.0) is True

    def test_invalid_value_out_of_range(self):
        assert validate_in_range(-1.0, 0.0, 100.0) is False
        assert validate_in_range(101.0, 0.0, 100.0) is False

    def test_none_with_allow_none_true(self):
        assert validate_in_range(None, 0.0, 100.0, allow_none=True) is True

    def test_none_with_allow_none_false(self):
        assert validate_in_range(None, 0.0, 100.0, allow_none=False) is False

    def test_string_numbers(self):
        assert validate_in_range("50", 0.0, 100.0) is True
        assert validate_in_range("150", 0.0, 100.0) is False

    def test_invalid_types(self):
        assert validate_in_range("invalid", 0.0, 100.0) is False
        assert validate_in_range({}, 0.0, 100.0) is False


class TestValidateLatitude:
    """Tests for latitude validation."""

    def test_valid_latitudes(self):
        assert validate_latitude(0.0) is True
        assert validate_latitude(45.5) is True
        assert validate_latitude(-45.5) is True
        assert validate_latitude(90.0) is True
        assert validate_latitude(-90.0) is True

    def test_invalid_latitudes(self):
        assert validate_latitude(91.0) is False
        assert validate_latitude(-91.0) is False
        assert validate_latitude(180.0) is False

    def test_none_not_allowed(self):
        assert validate_latitude(None) is False


class TestValidateLongitude:
    """Tests for longitude validation."""

    def test_valid_longitudes(self):
        assert validate_longitude(0.0) is True
        assert validate_longitude(120.5) is True
        assert validate_longitude(-120.5) is True
        assert validate_longitude(180.0) is True
        assert validate_longitude(-180.0) is True

    def test_invalid_longitudes(self):
        assert validate_longitude(181.0) is False
        assert validate_longitude(-181.0) is False
        assert validate_longitude(360.0) is False

    def test_none_not_allowed(self):
        assert validate_longitude(None) is False


class TestValidateAltitude:
    """Tests for altitude validation."""

    def test_valid_altitudes(self):
        assert validate_altitude(0.0) is True
        assert validate_altitude(10000.0) is True
        assert validate_altitude(35000.0) is True
        assert validate_altitude(60000.0) is True
        assert validate_altitude(-1000.0) is True  # Below sea level

    def test_invalid_altitudes(self):
        assert validate_altitude(-2000.0) is False  # Too low
        assert validate_altitude(70000.0) is False  # Too high

    def test_none_allowed(self):
        assert validate_altitude(None) is True


class TestValidateSpeed:
    """Tests for speed validation."""

    def test_valid_speeds(self):
        assert validate_speed(0.0) is True
        assert validate_speed(100.0) is True
        assert validate_speed(250.0) is True
        assert validate_speed(600.0) is True

    def test_invalid_speeds(self):
        assert validate_speed(-1.0) is False
        assert validate_speed(700.0) is False

    def test_none_allowed(self):
        assert validate_speed(None) is True


class TestValidateVerticalSpeed:
    """Tests for vertical speed validation."""

    def test_valid_vertical_speeds(self):
        assert validate_vertical_speed(0.0) is True
        assert validate_vertical_speed(1000.0) is True
        assert validate_vertical_speed(-1000.0) is True
        assert validate_vertical_speed(6000.0) is True
        assert validate_vertical_speed(-6000.0) is True

    def test_invalid_vertical_speeds(self):
        assert validate_vertical_speed(7000.0) is False
        assert validate_vertical_speed(-7000.0) is False


class TestValidateHeading:
    """Tests for heading validation."""

    def test_valid_headings(self):
        assert validate_heading(0.0) is True
        assert validate_heading(90.0) is True
        assert validate_heading(180.0) is True
        assert validate_heading(270.0) is True
        assert validate_heading(360.0) is True

    def test_invalid_headings(self):
        assert validate_heading(-1.0) is False
        assert validate_heading(361.0) is False


class TestValidatePitch:
    """Tests for pitch validation."""

    def test_valid_pitch(self):
        assert validate_pitch(0.0) is True
        assert validate_pitch(15.0) is True
        assert validate_pitch(-15.0) is True
        assert validate_pitch(90.0) is True
        assert validate_pitch(-90.0) is True

    def test_invalid_pitch(self):
        assert validate_pitch(91.0) is False
        assert validate_pitch(-91.0) is False


class TestValidateRoll:
    """Tests for roll validation."""

    def test_valid_roll(self):
        assert validate_roll(0.0) is True
        assert validate_roll(45.0) is True
        assert validate_roll(-45.0) is True
        assert validate_roll(180.0) is True
        assert validate_roll(-180.0) is True

    def test_invalid_roll(self):
        assert validate_roll(181.0) is False
        assert validate_roll(-181.0) is False


class TestValidateTemperature:
    """Tests for temperature validation."""

    def test_valid_temperatures(self):
        assert validate_temperature(0.0) is True
        assert validate_temperature(15.0) is True
        assert validate_temperature(-40.0) is True
        assert validate_temperature(40.0) is True
        assert validate_temperature(-60.0) is True
        assert validate_temperature(60.0) is True

    def test_invalid_temperatures(self):
        assert validate_temperature(-70.0) is False
        assert validate_temperature(70.0) is False


class TestValidatePressure:
    """Tests for barometric pressure validation."""

    def test_valid_pressures(self):
        assert validate_pressure(29.92) is True  # Standard
        assert validate_pressure(28.00) is True
        assert validate_pressure(31.00) is True
        assert validate_pressure(27.50) is True

    def test_invalid_pressures(self):
        assert validate_pressure(26.00) is False
        assert validate_pressure(33.00) is False


class TestValidateRPM:
    """Tests for RPM validation."""

    def test_valid_rpm(self):
        assert validate_rpm(0.0) is True
        assert validate_rpm(2500.0) is True
        assert validate_rpm(10000.0) is True

    def test_invalid_rpm(self):
        assert validate_rpm(-100.0) is False
        assert validate_rpm(15000.0) is False


class TestValidateN1Percent:
    """Tests for N1 percentage validation."""

    def test_valid_n1(self):
        assert validate_n1_percent(0.0) is True
        assert validate_n1_percent(50.0) is True
        assert validate_n1_percent(100.0) is True
        assert validate_n1_percent(105.0) is True  # Slight over-range allowed

    def test_invalid_n1(self):
        assert validate_n1_percent(-1.0) is False
        assert validate_n1_percent(115.0) is False


class TestValidatePercentage:
    """Tests for generic percentage validation."""

    def test_valid_percentages(self):
        assert validate_percentage(0.0) is True
        assert validate_percentage(50.0) is True
        assert validate_percentage(100.0) is True

    def test_invalid_percentages(self):
        assert validate_percentage(-1.0) is False
        assert validate_percentage(101.0) is False


class TestValidateComFrequency:
    """Tests for COM radio frequency validation."""

    def test_valid_com_frequencies(self):
        assert validate_com_frequency(118000) is True  # 118.000 MHz
        assert validate_com_frequency(122750) is True  # 122.750 MHz
        assert validate_com_frequency(136975) is True  # 136.975 MHz

    def test_invalid_com_frequencies(self):
        assert validate_com_frequency(117999) is False  # Below range
        assert validate_com_frequency(137000) is False  # Above range
        assert validate_com_frequency(110000) is False  # NAV range

    def test_none_allowed(self):
        assert validate_com_frequency(None) is True

    def test_invalid_types(self):
        assert validate_com_frequency("invalid") is False


class TestValidateNavFrequency:
    """Tests for NAV radio frequency validation."""

    def test_valid_nav_frequencies(self):
        assert validate_nav_frequency(108000) is True  # 108.000 MHz
        assert validate_nav_frequency(110000) is True  # 110.000 MHz
        assert validate_nav_frequency(117950) is True  # 117.950 MHz

    def test_invalid_nav_frequencies(self):
        assert validate_nav_frequency(107999) is False  # Below range
        assert validate_nav_frequency(118000) is False  # COM range
        assert validate_nav_frequency(120000) is False  # COM range

    def test_none_allowed(self):
        assert validate_nav_frequency(None) is True


class TestValidateTransponderCode:
    """Tests for transponder code validation."""

    def test_valid_transponder_codes(self):
        assert validate_transponder_code(0) is True     # 0000
        assert validate_transponder_code(1200) is True  # VFR code
        assert validate_transponder_code(7700) is True  # Emergency
        assert validate_transponder_code(7777) is True  # Max octal

    def test_invalid_transponder_codes_out_of_range(self):
        assert validate_transponder_code(-1) is False
        assert validate_transponder_code(8000) is False  # Too high

    def test_invalid_non_octal_digits(self):
        # Transponder codes must use only octal digits (0-7)
        assert validate_transponder_code(1289) is False  # Has digit 8
        assert validate_transponder_code(1900) is False  # Has digit 9

    def test_none_allowed(self):
        assert validate_transponder_code(None) is True


class TestValidateThrottleCommand:
    """Tests for throttle command validation."""

    def test_valid_normalized_range(self):
        assert validate_throttle_command(-1.0) is True
        assert validate_throttle_command(0.0) is True
        assert validate_throttle_command(0.5) is True
        assert validate_throttle_command(1.0) is True

    def test_valid_raw_range(self):
        assert validate_throttle_command(-16384) is True
        assert validate_throttle_command(0) is True
        assert validate_throttle_command(8192) is True
        assert validate_throttle_command(16384) is True

    def test_invalid_values(self):
        assert validate_throttle_command(-1.5) is False
        assert validate_throttle_command(1.5) is False
        assert validate_throttle_command(-20000) is False
        assert validate_throttle_command(20000) is False

    def test_none_not_allowed(self):
        assert validate_throttle_command(None) is False


class TestValidateGearCommand:
    """Tests for gear command validation."""

    def test_valid_gear_positions(self):
        assert validate_gear_command(0) is True  # Retracted
        assert validate_gear_command(1) is True  # Down
        assert validate_gear_command(0.0) is True  # Float 0
        assert validate_gear_command(1.0) is True  # Float 1

    def test_invalid_gear_positions(self):
        assert validate_gear_command(2) is False
        assert validate_gear_command(-1) is False
        assert validate_gear_command(0.5) is False

    def test_none_not_allowed(self):
        assert validate_gear_command(None) is False


class TestSanitizeFloat:
    """Tests for float sanitization."""

    def test_valid_floats(self):
        assert sanitize_float(123.45) == 123.45
        assert sanitize_float("123.45") == 123.45
        assert sanitize_float(100) == 100.0

    def test_invalid_values_use_default(self):
        assert sanitize_float(None) == 0.0
        assert sanitize_float(None, 10.0) == 10.0
        assert sanitize_float("invalid") == 0.0
        assert sanitize_float("invalid", 99.9) == 99.9
        assert sanitize_float({}) == 0.0


class TestSanitizeInt:
    """Tests for integer sanitization."""

    def test_valid_integers(self):
        assert sanitize_int(123) == 123
        assert sanitize_int("123") == 123
        assert sanitize_int(123.7) == 123

    def test_invalid_values_use_default(self):
        assert sanitize_int(None) == 0
        assert sanitize_int(None, 10) == 10
        assert sanitize_int("invalid") == 0
        assert sanitize_int("invalid", 99) == 99


class TestSanitizeBool:
    """Tests for boolean sanitization."""

    def test_valid_booleans(self):
        assert sanitize_bool(True) is True
        assert sanitize_bool(False) is False
        assert sanitize_bool(1) is True
        assert sanitize_bool(0) is False
        assert sanitize_bool("yes") is True
        assert sanitize_bool("") is False

    def test_none_uses_default(self):
        assert sanitize_bool(None) is False
        assert sanitize_bool(None, True) is True
