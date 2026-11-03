"""Adapter tests: body-frame command conversion for several yaw angles, and beam ranges."""

import math

import pytest

from airmouse_nav.bridges.sim_bridge import (VelocityCommand, beam_range, body_to_plugin,
                                             yaw_from_quaternion)

YAWS = [0.0, math.pi / 4, math.pi / 2, math.pi, -math.pi / 2, 2.5]


@pytest.mark.parametrize('yaw', YAWS)
def test_body_frame_is_pass_through(yaw: float) -> None:
    """The measured plugin convention is body frame: the command is unchanged at any yaw."""
    cmd = VelocityCommand(0.5, -0.2, 0.3, 0.4)
    assert body_to_plugin(cmd, yaw, 'body') == cmd


@pytest.mark.parametrize('yaw', YAWS)
def test_world_frame_rotates_by_yaw(yaw: float) -> None:
    """'forward' maps to the heading direction, 'left' to heading + 90 deg; z and yaw rate kept."""
    fwd = body_to_plugin(VelocityCommand(1.0, 0.0, 0.3, 0.4), yaw, 'world')
    left = body_to_plugin(VelocityCommand(0.0, 1.0, 0.0, 0.0), yaw, 'world')
    assert fwd.vx == pytest.approx(math.cos(yaw)) and fwd.vy == pytest.approx(math.sin(yaw))
    assert left.vx == pytest.approx(-math.sin(yaw)) and left.vy == pytest.approx(math.cos(yaw))
    assert (fwd.vz, fwd.yaw_rate) == (0.3, 0.4)
    assert math.hypot(fwd.vx, fwd.vy) == pytest.approx(1.0)


def test_unknown_frame_rejected() -> None:
    """A typo in plugin_frame fails loudly."""
    with pytest.raises(ValueError):
        body_to_plugin(VelocityCommand(0, 0, 0, 0), 0.0, 'map')


@pytest.mark.parametrize('yaw', YAWS)
def test_yaw_from_quaternion(yaw: float) -> None:
    """Heading round-trips through a pure-yaw quaternion."""
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    assert math.remainder(yaw_from_quaternion(*q) - yaw, 2 * math.pi) == pytest.approx(0.0, abs=1e-9)


def test_beam_range() -> None:
    """In-range value kept; no return -> +inf; too close -> -inf (REP 117)."""
    assert beam_range([1.2], 0.04, 4.0) == 1.2
    assert beam_range([math.inf], 0.04, 4.0) == math.inf
    assert beam_range([5.0], 0.04, 4.0) == math.inf
    assert beam_range([0.01], 0.04, 4.0) == -math.inf
    assert beam_range([], 0.04, 4.0) == math.inf
