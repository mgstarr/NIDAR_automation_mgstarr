"""Unit tests for the range-to-z adapter."""

import math

import pytest

from airmouse_nav.bridges.range_to_z import range_to_z


def test_valid_reading_adds_sensor_offset() -> None:
    """Hovering at 1.2 m the sensor (0.06 m below base_link) reads 1.14 m."""
    assert range_to_z(1.14, 0.04, 4.0, 0.06) == pytest.approx(1.20)


def test_tilt_correction() -> None:
    """A tilted beam is longer; the cosines bring it back to the vertical height."""
    tilt = math.radians(10)
    assert range_to_z(1.14 / math.cos(tilt), 0.04, 4.0, 0.06, roll=tilt) == pytest.approx(1.20)


def test_on_the_floor() -> None:
    """-inf (closer than the minimum range) means the drone stands on the floor."""
    assert range_to_z(-math.inf, 0.04, 4.0, 0.06) == pytest.approx(0.08)


@pytest.mark.parametrize('bad', [math.inf, math.nan, 0.01, 4.5])
def test_invalid_readings_give_no_measurement(bad: float) -> None:
    """Out-of-range, +inf and NaN readings are dropped."""
    assert range_to_z(bad, 0.04, 4.0, 0.06) is None
