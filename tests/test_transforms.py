"""
Tests for FSUIPC data transformation functions.

This module tests the transformation functions that convert raw FSUIPC
offset data into human-readable values.
"""

import pytest
from fsuipc_shirley_bridge import (
    knots128_to_kts,
    vs_raw_to_fpm,
    meters256_to_m,
    magvar_raw_to_deg,
    baro_to_inhg,
    lower16,
    u32_baro_to_inhg,
    bcd_to_freq_com_official,
    bcd_to_freq_nav_official,
    bcd_to_xpdr_official,
    rpm_raw_to_rpm,
    throttle_to_percent,
    mixture_to_percent,
    prop_to_percent,
    heading_bug_to_deg,
    alt_bug_to_feet,
    wind_dir_to_deg,
    sanitize_float,
    sanitize_int,
)


class TestKnots128ToKts:
    """Tests for knots/128 to knots conversion."""

    def test_valid_conversion(self):
        # 128 units = 1 knot
        assert knots128_to_kts(128) == pytest.approx(1.0)
        assert knots128_to_kts(12800) == pytest.approx(100.0)
        assert knots128_to_kts(0) == 0.0

    def test_fractional_knots(self):
        # 64 units = 0.5 knots
        assert knots128_to_kts(64) == pytest.approx(0.5)

    def test_invalid_input_returns_none(self):
        assert knots128_to_kts(None) is None
        assert knots128_to_kts("invalid") is None


class TestVsRawToFpm:
    """Tests for vertical speed conversion."""

    def test_valid_conversion(self):
        # VS raw = 256 * m/s
        # 1 m/s = 196.85 ft/min approx
        # So raw 256 = 1 m/s = ~196.85 fpm
        result = vs_raw_to_fpm(256)
        assert result is not None
        assert abs(result - 196.85) < 1.0  # Allow small rounding error

    def test_zero_vertical_speed(self):
        assert vs_raw_to_fpm(0) == 0.0

    def test_negative_vertical_speed(self):
        # Descent should give negative values
        result = vs_raw_to_fpm(-256)
        assert result is not None
        assert result < 0

    def test_invalid_input(self):
        assert vs_raw_to_fpm(None) is None


class TestMeters256ToM:
    """Tests for altitude in meters conversion."""

    def test_valid_conversion(self):
        # 256 units = 1 meter
        assert meters256_to_m(256) == pytest.approx(1.0)
        assert meters256_to_m(25600) == pytest.approx(100.0)
        assert meters256_to_m(0) == 0.0

    def test_invalid_input(self):
        assert meters256_to_m(None) is None


class TestMagvarRawToDeg:
    """Tests for magnetic variation conversion."""

    def test_zero_variation(self):
        assert magvar_raw_to_deg(0) == pytest.approx(0.0)

    def test_positive_variation_east(self):
        # Positive values = East variation
        # Full scale = 65536 = 360 degrees
        # 65536/4 = 16384 = 90 degrees
        result = magvar_raw_to_deg(16384)
        assert result is not None
        assert abs(result - 90.0) < 1.0

    def test_invalid_input(self):
        assert magvar_raw_to_deg(None) is None


class TestBaroToInhg:
    """Tests for barometric pressure conversion."""

    def test_standard_pressure(self):
        # Standard pressure: 29.92 inHg ≈ 1013 mb
        # Raw value = mb * 16
        raw = int(1013 * 16)
        result = baro_to_inhg(raw)
        assert result is not None
        assert abs(result - 29.92) < 0.1

    def test_low_pressure(self):
        # Low pressure: ~980 mb
        raw = int(980 * 16)
        result = baro_to_inhg(raw)
        assert result is not None
        assert 28.0 < result < 30.0

    def test_invalid_input(self):
        assert baro_to_inhg(None) is None


class TestLower16:
    """Tests for extracting lower 16 bits from U32."""

    def test_extract_lower_16_bits(self):
        # 0x12345678 -> lower 16 bits = 0x5678 = 22136
        assert lower16(0x12345678) == 0x5678
        assert lower16(0x12345678) == 22136

    def test_already_16_bit(self):
        # Values that fit in 16 bits remain unchanged
        assert lower16(0xFFFF) == 0xFFFF
        assert lower16(1234) == 1234

    def test_zero(self):
        assert lower16(0) == 0

    def test_invalid_input(self):
        assert lower16(None) is None


class TestU32BaroToInhg:
    """Tests for U32 barometric pressure conversion."""

    def test_extracts_and_converts(self):
        # Create a U32 with barometric data in lower 16 bits
        # Standard pressure ~1013 mb * 16 in lower 16 bits
        raw_lower = int(1013 * 16)
        u32_value = (0x1234 << 16) | raw_lower  # Upper bits irrelevant
        result = u32_baro_to_inhg(u32_value)
        assert result is not None
        assert abs(result - 29.92) < 0.2

    def test_invalid_input(self):
        assert u32_baro_to_inhg(None) is None


class TestBcdToFreqComOfficial:
    """Tests for COM frequency BCD conversion."""

    def test_common_com_frequencies(self):
        # 122.75 MHz -> BCD 0x2275 -> 122750 kHz
        assert bcd_to_freq_com_official(0x2275) == 122750

        # 118.00 MHz -> BCD 0x1800 -> 118000 kHz
        assert bcd_to_freq_com_official(0x1800) == 118000

        # 135.50 MHz -> BCD 0x3550 -> 135500 kHz
        assert bcd_to_freq_com_official(0x3550) == 135500

    def test_out_of_range_returns_default(self):
        # Below COM range
        result = bcd_to_freq_com_official(0x0750)  # 107.50 MHz
        assert result == 122750  # Default

        # Above COM range
        result = bcd_to_freq_com_official(0x3800)  # 138.00 MHz
        assert result == 122750  # Default

    def test_invalid_input(self):
        result = bcd_to_freq_com_official(None)
        assert result == 122750  # Default


class TestBcdToFreqNavOfficial:
    """Tests for NAV frequency BCD conversion."""

    def test_common_nav_frequencies(self):
        # 110.00 MHz -> BCD 0x1000 -> 110000 kHz
        assert bcd_to_freq_nav_official(0x1000) == 110000

        # 117.95 MHz -> BCD 0x1795 -> 117950 kHz
        assert bcd_to_freq_nav_official(0x1795) == 117950

    def test_out_of_range_returns_default(self):
        # Above NAV range (into COM range)
        result = bcd_to_freq_nav_official(0x2000)  # 120.00 MHz
        assert result == 110000  # Default


class TestBcdToXpdrOfficial:
    """Tests for transponder code BCD conversion."""

    def test_common_squawk_codes(self):
        # 1200 (VFR) -> BCD 0x1200
        assert bcd_to_xpdr_official(0x1200) == 1200

        # 7700 (Emergency) -> BCD 0x7700
        assert bcd_to_xpdr_official(0x7700) == 7700

        # 0000 -> BCD 0x0000
        assert bcd_to_xpdr_official(0x0000) == 0

    def test_invalid_codes_return_default(self):
        # Code with digit > 7 (invalid octal)
        # 8888 in BCD is invalid for transponder
        result = bcd_to_xpdr_official(0x8888)
        assert result == 1200  # Default

    def test_none_returns_default(self):
        assert bcd_to_xpdr_official(None) == 1200


class TestRpmRawToRpm:
    """Tests for RPM conversion."""

    def test_direct_conversion(self):
        # Direct pass-through for most aircraft
        assert rpm_raw_to_rpm(2500) == 2500.0
        assert rpm_raw_to_rpm(0) == 0.0

    def test_invalid_input(self):
        assert rpm_raw_to_rpm(None) is None


class TestThrottleToPercent:
    """Tests for throttle position conversion."""

    def test_full_throttle(self):
        # 16384 raw = 100%
        result = throttle_to_percent(16384)
        assert result is not None
        assert abs(result - 100.0) < 1.0

    def test_half_throttle(self):
        # 8192 raw ≈ 50%
        result = throttle_to_percent(8192)
        assert result is not None
        assert abs(result - 50.0) < 2.0

    def test_idle(self):
        result = throttle_to_percent(0)
        assert result is not None
        assert abs(result) < 1.0

    def test_invalid_input(self):
        assert throttle_to_percent(None) is None


class TestMixtureToPercent:
    """Tests for mixture position conversion."""

    def test_full_rich(self):
        result = mixture_to_percent(16384)
        assert result is not None
        assert abs(result - 100.0) < 1.0

    def test_lean(self):
        result = mixture_to_percent(0)
        assert result is not None
        assert abs(result) < 1.0


class TestPropToPercent:
    """Tests for propeller position conversion."""

    def test_full_forward(self):
        result = prop_to_percent(16384)
        assert result is not None
        assert abs(result - 100.0) < 1.0

    def test_feathered(self):
        result = prop_to_percent(0)
        assert result is not None
        assert abs(result) < 1.0


class TestHeadingBugToDeg:
    """Tests for autopilot heading bug conversion."""

    def test_north(self):
        assert heading_bug_to_deg(0) == 0.0

    def test_east(self):
        # 65536/4 = 16384 = 90 degrees
        result = heading_bug_to_deg(16384)
        assert abs(result - 90.0) < 1.0

    def test_south(self):
        # 65536/2 = 32768 = 180 degrees
        result = heading_bug_to_deg(32768)
        assert abs(result - 180.0) < 1.0

    def test_west(self):
        # 3*65536/4 = 49152 = 270 degrees
        result = heading_bug_to_deg(49152)
        assert abs(result - 270.0) < 1.0

    def test_invalid_input_returns_zero(self):
        # Should return 0.0 instead of None for autopilot fields
        assert heading_bug_to_deg(None) == 0.0


class TestAltBugToFeet:
    """Tests for autopilot altitude bug conversion."""

    def test_direct_passthrough(self):
        assert alt_bug_to_feet(10000) == 10000.0
        assert alt_bug_to_feet(35000) == 35000.0

    def test_zero(self):
        assert alt_bug_to_feet(0) == 0.0

    def test_invalid_input_returns_zero(self):
        assert alt_bug_to_feet(None) == 0.0


class TestWindDirToDeg:
    """Tests for wind direction conversion."""

    def test_north_wind(self):
        assert wind_dir_to_deg(0) == 0.0

    def test_east_wind(self):
        # 65536/4 = 16384 = 90 degrees
        result = wind_dir_to_deg(16384)
        assert result is not None
        assert abs(result - 90.0) < 1.0

    def test_south_wind(self):
        result = wind_dir_to_deg(32768)
        assert result is not None
        assert abs(result - 180.0) < 1.0

    def test_invalid_input(self):
        assert wind_dir_to_deg(None) is None


class TestIntegrationScenarios:
    """Integration tests combining multiple transformations."""

    def test_typical_cruise_data(self):
        """Test realistic cruise scenario values."""
        # IAS: 250 knots
        ias = knots128_to_kts(250 * 128)
        assert ias is not None
        assert abs(ias - 250.0) < 1.0

        # Altitude: 35000 feet (convert from meters*256)
        alt_m = 35000 / 3.28084  # feet to meters
        alt_raw = int(alt_m * 256)
        alt_result = meters256_to_m(alt_raw)
        assert alt_result is not None
        assert abs(alt_result - alt_m) < 1.0

        # Standard pressure
        baro = baro_to_inhg(int(1013 * 16))
        assert baro is not None
        assert abs(baro - 29.92) < 0.1

    def test_approach_configuration(self):
        """Test typical approach configuration."""
        # Lower speed: 140 knots
        ias = knots128_to_kts(140 * 128)
        assert ias is not None
        assert 139 < ias < 141

        # Descending: -700 fpm (approx -3.56 m/s)
        # -3.56 m/s * 256 = -911.36
        vs = vs_raw_to_fpm(-911)
        assert vs is not None
        assert -750 < vs < -650

        # Partial throttle: 40%
        throttle = throttle_to_percent(int(16384 * 0.4))
        assert throttle is not None
        assert 38 < throttle < 42
