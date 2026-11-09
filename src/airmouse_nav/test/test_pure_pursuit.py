"""Unit tests for airmouse_nav.core.pure_pursuit with a small point-mass simulator."""

import math

import pytest

from airmouse_nav.core.pure_pursuit import (Pose, PurePursuit, PursuitParams, body_to_world,
                                            is_stale, world_to_body)

DT = 1.0 / 30.0
TAU = 0.35  # s, velocity response lag of the simulated drone (Gazebo: about 0.3-0.4 s)


def seg_dist(px: float, py: float, a, b) -> float:
    """Distance from a point to segment ab (2D)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / L2))
    return math.hypot(px - a[0] - t * dx, py - a[1] - t * dy)


def path_dist(px: float, py: float, path) -> float:
    """Distance from a point to a polyline."""
    if len(path) == 1:
        return math.hypot(px - path[0][0], py - path[0][1])
    return min(seg_dist(px, py, a, b) for a, b in zip(path, path[1:]))


def simulate(path, params: PursuitParams, start=None, goal_yaw=None, t_max: float = 60.0):
    """Fly a point mass (first-order velocity lag) along `path`. Returns (pose, max xtrack, t)."""
    pp = PurePursuit(params)
    pp.set_path(path, goal_yaw)
    x, y, z, yaw = start if start else (path[0][0], path[0][1], path[0][2], 0.0)
    vx = vy = vz = 0.0
    worst, t = 0.0, 0.0
    while t < t_max:
        cmd = pp.update(Pose(x, y, z, yaw), DT)
        if cmd.reached:
            break
        # The drone receives body-frame commands and turns them back (tests the rotation too).
        bx, by = world_to_body(cmd.vx, cmd.vy, yaw)
        wx, wy = body_to_world(bx, by, yaw)
        vx += (wx - vx) * DT / TAU
        vy += (wy - vy) * DT / TAU
        vz += (cmd.vz - vz) * DT / TAU
        x, y, z = x + vx * DT, y + vy * DT, z + vz * DT
        yaw += cmd.yaw_rate * DT
        worst = max(worst, path_dist(x, y, path))
        t += DT
    return (x, y, z, yaw), worst, t, pp.reached


SLOW = PursuitParams(max_speed=0.5)
PATHS = {
    'straight': [(0, 0, 1.2), (4, 0, 1.2)],
    'l_turn': [(0, 0, 1.2), (3, 0, 1.2), (3, 3, 1.2)],
    'u_turn': [(0, 0, 1.2), (3, 0, 1.2), (3, 1.1, 1.2), (0, 1.1, 1.2)],
    'zigzag': [(0, 0, 1.2), (1.1, 0, 1.2), (1.1, 1.1, 1.2), (2.2, 1.1, 1.2), (2.2, 0, 1.2),
               (3.3, 0, 1.2)],
}


@pytest.mark.parametrize('name', sorted(PATHS))
def test_follow_path_cross_track_and_goal(name: str) -> None:
    """Cross-track error < 0.10 m at 0.5 m/s; stops within tolerance at the goal."""
    path = PATHS[name]
    (x, y, z, _), worst, t, reached = simulate(path, SLOW)
    assert reached, f'{name}: goal not reached in {t:.1f} s'
    assert worst < 0.10, f'{name}: cross-track {worst:.3f} m'
    gx, gy, gz = path[-1]
    assert math.hypot(x - gx, y - gy) <= SLOW.goal_tolerance_xy
    assert abs(z - gz) <= SLOW.goal_tolerance_xy


def test_speed_respects_limits() -> None:
    """Commanded speed never exceeds max_speed; speed changes stay within max_accel."""
    p = PursuitParams()
    pp = PurePursuit(p)
    pp.set_path(PATHS['l_turn'])
    x, y, prev, prev_w = 0.0, 0.0, (0.0, 0.0), 0.0
    for _ in range(600):
        cmd = pp.update(Pose(x, y, 1.2, 0.0), DT)
        if cmd.reached:
            break
        assert math.hypot(cmd.vx, cmd.vy) <= p.max_speed + 1e-9
        assert math.hypot(cmd.vx - prev[0], cmd.vy - prev[1]) <= p.max_accel * DT + 1e-9
        assert abs(cmd.yaw_rate) <= p.max_yaw_rate + 1e-9
        assert abs(cmd.yaw_rate - prev_w) <= p.max_yaw_accel * DT + 1e-9
        prev = (cmd.vx, cmd.vy)
        prev_w = cmd.yaw_rate
        x, y = x + cmd.vx * DT, y + cmd.vy * DT


def test_takeoff_and_landing_paths() -> None:
    """A vertical path (same x, y) climbs to flight altitude without sideways motion; z=0 lands."""
    p = PursuitParams()
    (x, y, z, _), worst, _, reached = simulate([(1, 1, 0.0), (1, 1, 1.2)], p,
                                                start=(1, 1, 0.05, 0.3))
    assert reached and abs(z - 1.2) <= p.goal_tolerance_xy and worst < 0.02
    (x, y, z, _), worst, _, reached = simulate([(1, 1, 1.2), (1, 1, 0.0)], p,
                                                start=(1, 1, 1.2, 0.3))
    assert reached and z <= p.goal_tolerance_xy and worst < 0.02


def test_climbs_before_moving_sideways() -> None:
    """With a large altitude error the horizontal command is zero."""
    pp = PurePursuit(PursuitParams())
    pp.set_path([(0, 0, 1.2), (3, 0, 1.2)])
    cmd = pp.update(Pose(0, 0, 0.1, 0), DT)
    assert cmd.vz > 0 and cmd.vx == 0 and cmd.vy == 0


def test_goal_yaw_is_reached() -> None:
    """At the end the drone turns to the goal yaw and only then reports reached."""
    p = PursuitParams()
    (_, _, _, yaw), _, _, reached = simulate(PATHS['straight'], p, goal_yaw=math.pi / 2)
    assert reached
    assert abs(math.remainder(yaw - math.pi / 2, 2 * math.pi)) <= p.goal_tolerance_yaw


def test_heading_follows_travel_direction() -> None:
    """Mid-path the yaw command turns the drone toward the direction of travel."""
    pp = PurePursuit(PursuitParams())
    pp.set_path([(0, 0, 1.2), (0, 4, 1.2)])
    for _ in range(30):
        cmd = pp.update(Pose(0, 0, 1.2, 0.0), DT)
    assert cmd.yaw_rate > 0 and cmd.vy > 0


def test_no_path_gives_zero_and_goal_is_held() -> None:
    """Without a path the command is zero; after the goal the drone holds the goal point."""
    pp = PurePursuit(PursuitParams())
    assert pp.update(Pose(0, 0, 1.2, 0), DT)[:4] == (0, 0, 0, 0)
    pp.set_path([(0, 0, 1.2), (0.05, 0, 1.2)])
    cmd = pp.update(Pose(0.05, 0, 1.2, 0), DT)
    assert cmd.reached
    for _ in range(60):                      # pushed 0.12 m off and 0.1 m low: comes back
        cmd = pp.update(Pose(0.05, 0.12, 1.1, 0), DT)
    assert cmd.reached and cmd.vy < -0.05 and cmd.vz > 0.05 and abs(cmd.vx) < 0.01


def test_no_hold_after_landing() -> None:
    """After a landing path (z = 0) the command stays zero."""
    pp = PurePursuit(PursuitParams())
    pp.set_path([(1, 1, 1.2), (1, 1, 0.0)])
    cmd = pp.update(Pose(1.05, 1, 0.05, 0), DT)
    assert cmd.reached and cmd[:3] == (0, 0, 0)
    assert pp.update(Pose(1.1, 1, 0.06, 0), DT)[:3] == (0, 0, 0)


def test_climb_holds_xy() -> None:
    """During a vertical climb, a horizontal offset is corrected back toward the path."""
    pp = PurePursuit(PursuitParams())
    pp.set_path([(1, 1, 0.0), (1, 1, 1.2)])
    for _ in range(60):
        cmd = pp.update(Pose(1.15, 1, 0.3, 0), DT)
    assert cmd.vx < -0.1 and abs(cmd.vy) < 1e-9 and cmd.vz > 0


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, math.pi, -math.pi / 2, 0.7, -2.5])
def test_body_rotation(yaw: float) -> None:
    """World -> body rotation: the body x axis is the heading; round trip is exact."""
    bx, by = world_to_body(math.cos(yaw), math.sin(yaw), yaw)
    assert bx == pytest.approx(1.0) and by == pytest.approx(0.0, abs=1e-12)
    bx, by = world_to_body(-math.sin(yaw), math.cos(yaw), yaw)   # world "left of heading"
    assert bx == pytest.approx(0.0, abs=1e-12) and by == pytest.approx(1.0)
    wx, wy = body_to_world(*world_to_body(0.3, -0.2, yaw), yaw)
    assert (wx, wy) == pytest.approx((0.3, -0.2))


def test_stale_pose_and_path() -> None:
    """A stale pose always stops; a stale path stops only while following."""
    assert is_stale(10.0, 9.9, 9.0, 0.3, 2.0, True) is None
    assert is_stale(10.0, 9.5, 9.0, 0.3, 2.0, True) == 'pose'
    assert is_stale(10.0, None, 9.0, 0.3, 2.0, False) == 'pose'
    assert is_stale(10.0, 9.9, 7.5, 0.3, 2.0, True) == 'path'
    assert is_stale(10.0, 9.9, 7.5, 0.3, 2.0, False) is None
