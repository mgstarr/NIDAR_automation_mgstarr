"""Unit tests for the safety hook's clamp and geofence (pure functions)."""

import math

import pytest

pytest.importorskip('rclpy')
from airmouse_nav.nodes.safety_hook_node import (clamp_command, filter_body_command,  # noqa: E402
                                                 geofence)

BOUNDS = [0.0, 10.0, -5.0, 5.0]


def test_clamp_keeps_direction() -> None:
    """Horizontal speed is scaled down keeping direction; vz and yaw rate are clipped."""
    vx, vy, vz, wz = clamp_command(0.9, 1.2, -0.9, 2.0, 0.6, 0.3, 0.8)
    assert math.hypot(vx, vy) == pytest.approx(0.6) and vy / vx == pytest.approx(1.2 / 0.9)
    assert vz == -0.3 and wz == 0.8
    assert clamp_command(0.1, 0.2, 0.1, 0.1, 0.6, 0.3, 0.8) == (0.1, 0.2, 0.1, 0.1)


@pytest.mark.parametrize('x,y,v_in,v_out', [
    (9.8, 0.0, (0.5, 0.2), (0.0, 0.2)),     # at +x bound: outward x removed, sideways kept
    (9.8, 0.0, (-0.5, 0.2), (-0.5, 0.2)),   # inward kept
    (0.1, 0.0, (-0.5, 0.0), (0.0, 0.0)),    # at -x bound
    (5.0, 4.9, (0.3, 0.4), (0.3, 0.0)),     # at +y bound
    (5.0, -4.8, (0.3, -0.4), (0.3, 0.0)),   # at -y bound
    (5.0, -4.8, (0.3, 0.4), (0.3, 0.4)),    # inward from -y
    (5.0, 0.0, (0.5, -0.5), (0.5, -0.5)),   # far from bounds: untouched
    (9.9, 4.9, (0.5, 0.5), (0.0, 0.0)),     # corner
])
def test_geofence(x, y, v_in, v_out) -> None:
    """Outward velocity is removed within the margin; inward velocity is kept."""
    assert geofence(x, y, *v_in, BOUNDS, 0.30) == pytest.approx(v_out)


def test_geofence_margin_edge() -> None:
    """Outward motion is allowed until the centre is within the margin of the bound."""
    assert geofence(9.6, 0.0, 0.5, 0.0, BOUNDS, 0.30) == (0.5, 0.0)
    assert geofence(9.7, 0.0, 0.5, 0.0, BOUNDS, 0.30) == (0.0, 0.0)


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, math.pi, -0.8])
def test_geofence_in_body_frame(yaw: float) -> None:
    """Body-frame commands are rotated to the map frame, fenced, and rotated back."""
    # Body command that means "world +x" for this yaw:
    bx, by = math.cos(yaw) * 0.5, -math.sin(yaw) * 0.5
    assert filter_body_command(9.9, 0.0, yaw, bx, by, BOUNDS, 0.3) == pytest.approx((0, 0), abs=1e-12)
    assert filter_body_command(5.0, 0.0, yaw, bx, by, BOUNDS, 0.3) == pytest.approx((bx, by))
