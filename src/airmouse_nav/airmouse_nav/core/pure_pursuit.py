"""Holonomic pure-pursuit path follower in the map frame, 2D + altitude (no ROS imports).

Public API (used by controller_node; Phase 5's explorer only publishes paths):

    params = PursuitParams(flight_altitude=1.2, max_speed=0.6, max_accel=0.5, ...)
    pp = PurePursuit(params)
    pp.set_path([(x, y, z), ...], goal_yaw=None)   # new path; keeps the current velocity
    pp.clear()                                       # no path, zero command
    pp.stop(pose)                                    # no path, hold this pose (hover)
    cmd = pp.update(Pose(x, y, z, yaw), dt)          # -> Command (map frame)
    vx_b, vy_b = world_to_body(cmd.vx, cmd.vy, yaw)  # rotate into base_link

    Command: vx, vy (map frame, m/s), vz (m/s), yaw_rate (rad/s), reached (bool),
             cross_track (m, distance to the path), active (bool: following a path)

How it works:
  - Progress: the drone's closest point on the path, searched forward from the previous
    progress (so a U-turn's return leg is not grabbed early).
  - Look-ahead distance = lookahead_min + (lookahead_max - lookahead_min) x speed / max_speed;
    the look-ahead point lies that far along the path past the progress point.
  - Horizontal velocity points at the look-ahead point (holonomic: no need to face it).
  - Speed = min(max_speed, corner limit, goal limit), then the change in the velocity vector
    is limited to max_accel x dt.
      corner limit: each path corner ahead with turn angle a allows
        v_c = sqrt(2 max_accel CORNER_ERROR) / (2 sin(a / 2)) (the velocity change at the corner
        fits in CORNER_ERROR of overshoot), and the drone must be able to brake to v_c in the
        distance left: v <= sqrt(v_c^2 + 2 BRAKE_FRACTION max_accel (d - CORNER_SWITCH));
  - Corners sharper than CORNER_ANGLE: the look-ahead point stops at the corner until the drone
    is within CORNER_SWITCH of it, so the drone does not cut the corner;
      goal limit: v <= sqrt(2 BRAKE_FRACTION max_accel d_goal).
  - Altitude: vz = altitude_gain x (z_target - z), limited to max_vertical_speed. z_target is
    the path's z interpolated at the look-ahead point (the final z near the end). While the
    altitude error is above CLIMB_FIRST_ERROR the drone does not move sideways, so take-off
    and landing paths (same x, y; z = flight_altitude or 0) are vertical.
  - Yaw: yaw_rate = yaw_gain x heading error, limited to max_yaw_rate and to
    sqrt(2 max_yaw_accel |error|) (so it can brake without overshoot); the yaw-rate command then
    changes by at most max_yaw_accel per second. In Gazebo, a sudden yaw-rate reversal of
    0.6-0.8 rad/s saturates the rotors and the drone drops to the floor (measured in Phase 4);
    0.5-1.0 rad/s^2 ramps keep altitude. Target heading = the
    direction of travel; on the last segment within lookahead_max of the goal, the goal yaw
    (if given; else the heading is held).
  - Goal reached: within goal_tolerance_xy horizontally and vertically, and within
    goal_tolerance_yaw of the goal yaw (if given). Then the drone holds the goal point
    (horizontal P-control HOLD_GAIN, at most HOLD_MAX_SPEED, plus altitude control) until a
    new path arrives. Exception: after a landing goal (z < LANDED_Z) the command is zero.
    The Gazebo plugin holds velocity, not position, so without this hold the drone drifts
    (measured: 0.2 m during a take-off and hover).
  - While climbing or descending in place, the horizontal command holds the closest path point.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple, Sequence

CLIMB_FIRST_ERROR = 0.25   # m: altitude error above which horizontal motion waits
MIN_SPEED = 0.05           # m/s: creep speed so the drone never stalls short of the goal
PROGRESS_WINDOW = 1.5      # m of path past the current progress searched for the closest point
CORNER_ANGLE = math.radians(20.0)   # turns sharper than this are taken corner-by-corner
CORNER_ERROR = 0.05        # m: allowed overshoot at a corner; sets the corner speed
CORNER_SWITCH = 0.05       # m before a corner where the look-ahead moves past it
BRAKE_FRACTION = 0.5       # plan braking with this fraction of max_accel (covers response lag)
HOLD_GAIN = 1.0            # 1/s: position-hold P-gain (hover at the goal, climb in place)
HOLD_MAX_SPEED = 0.2       # m/s: position-hold speed limit
LANDED_Z = 0.1             # m: a goal below this is a landing; no hold after it
HOLD_MIN_Z = 0.3           # m: stop() holds position only when airborne above this


@dataclass(frozen=True)
class PursuitParams:
    """Controller parameters (controller.yaml). Angles in radians."""

    flight_altitude: float = 1.2
    max_speed: float = 0.6
    max_accel: float = 0.5
    max_yaw_rate: float = 0.8
    lookahead_min: float = 0.3
    lookahead_max: float = 0.8
    goal_tolerance_xy: float = 0.15
    goal_tolerance_yaw: float = math.radians(10.0)
    yaw_gain: float = 1.5
    altitude_gain: float = 1.0
    max_vertical_speed: float = 0.3
    max_yaw_accel: float = 0.5


class Pose(NamedTuple):
    """Drone pose in the map frame."""

    x: float
    y: float
    z: float
    yaw: float


class Command(NamedTuple):
    """Velocity command in the map frame."""

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    reached: bool = False
    cross_track: float = 0.0
    active: bool = False


def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def world_to_body(vx: float, vy: float, yaw: float) -> tuple[float, float]:
    """Rotate a map-frame horizontal velocity into the body frame (x forward, y left)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


def body_to_world(vx: float, vy: float, yaw: float) -> tuple[float, float]:
    """Rotate a body-frame horizontal velocity into the map frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx - s * vy, s * vx + c * vy


class PurePursuit:
    """Stateful path follower. Call `update` at the control rate."""

    def __init__(self, params: PursuitParams) -> None:
        self.p = params
        self.points: list[tuple[float, float, float]] = []
        self.cum: list[float] = []          # 2D arc length at each point
        self.goal_yaw: float | None = None
        self.progress = 0.0                 # arc length of the closest point
        self.reached = False
        self.vel = (0.0, 0.0)               # last horizontal command (map frame)
        self.yaw_rate = 0.0                 # last yaw-rate command
        self.hold: tuple[float, float, float] | None = None   # hover point after a goal
        self.hold_yaw: float | None = None

    # ----------------------------------------------------------------- path handling
    def set_path(self, points: Sequence[Sequence[float]], goal_yaw: float | None = None) -> None:
        """Follow a new path of (x, y, z) points (map frame). Keeps the current velocity."""
        pts = [(float(p[0]), float(p[1]), float(p[2])) for p in points]
        if not pts:
            self.clear()
            return
        self.points = pts
        self.cum = [0.0]
        for a, b in zip(pts, pts[1:]):
            self.cum.append(self.cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.turns = [0.0] * len(pts)       # turn angle at each inner point
        for i in range(1, len(pts) - 1):
            if self.cum[i] - self.cum[i - 1] < 1e-9 or self.cum[i + 1] - self.cum[i] < 1e-9:
                continue
            a, b, c = pts[i - 1], pts[i], pts[i + 1]
            self.turns[i] = abs(wrap_angle(math.atan2(c[1] - b[1], c[0] - b[0])
                                           - math.atan2(b[1] - a[1], b[0] - a[0])))
        self.goal_yaw = goal_yaw
        self.progress = 0.0
        self.reached = False
        self.hold_yaw = None
        self.hold = None

    def clear(self) -> None:
        """Drop the path: hover."""
        self.points, self.cum = [], []
        self.goal_yaw = None
        self.reached = False
        self.hold = None

    def stop(self, pose: Pose | None) -> None:
        """Drop the path and hold `pose` (hover in place). On the floor, or without a pose,
        hold nothing (zero command)."""
        self.clear()
        if pose is not None and pose.z >= HOLD_MIN_Z:
            self.hold = (pose.x, pose.y, pose.z)

    @property
    def active(self) -> bool:
        """True while following a path that is not yet reached."""
        return bool(self.points) and not self.reached

    def point_at(self, s: float) -> tuple[float, float, float]:
        """Path point at arc length s (clamped to the path)."""
        pts, cum = self.points, self.cum
        if s <= 0.0 or len(pts) == 1:
            return pts[0]
        if s >= cum[-1]:
            return pts[-1]
        i = max(0, min(len(cum) - 2, _bisect(cum, s)))
        seg = cum[i + 1] - cum[i]
        t = 0.0 if seg <= 0 else (s - cum[i]) / seg
        a, b = pts[i], pts[i + 1]
        return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2]))

    def _project(self, x: float, y: float) -> tuple[float, float]:
        """Closest point on the path at or after the current progress (within a window).

        Returns (arc length, distance)."""
        best_s, best_d = self.progress, math.inf
        pts, cum = self.points, self.cum
        if len(pts) == 1:
            return 0.0, math.hypot(x - pts[0][0], y - pts[0][1])
        limit = self.progress + PROGRESS_WINDOW
        for i in range(len(pts) - 1):
            if cum[i + 1] < self.progress - 1e-9:
                continue
            if cum[i] > limit:
                break
            ax, ay, _ = pts[i]
            bx, by, _ = pts[i + 1]
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            t = 0.0 if seg2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
            s = cum[i] + t * math.sqrt(seg2)
            if s < self.progress:          # never move progress backwards
                s = self.progress
                px, py, _ = self.point_at(s)
            else:
                px, py = ax + t * dx, ay + t * dy
            d = math.hypot(x - px, y - py)
            if d < best_d - 1e-12:
                best_s, best_d = s, d
        return best_s, best_d

    def _corner_speed_limit(self, s: float) -> float:
        """Highest speed from which every corner ahead can still be taken (see module doc)."""
        p = self.p
        limit = p.max_speed
        horizon = p.max_speed ** 2 / (2 * p.max_accel) + p.lookahead_max
        for i in range(1, len(self.points) - 1):
            d = self.cum[i] - s
            if d < -CORNER_SWITCH:
                continue
            if d > horizon:
                break
            turn = self.turns[i]
            if turn < 1e-3:
                continue
            v_c = math.sqrt(2.0 * p.max_accel * CORNER_ERROR) / (2.0 * math.sin(turn / 2.0))
            d_brake = max(0.0, d - CORNER_SWITCH)
            limit = min(limit, math.sqrt(v_c ** 2 + 2.0 * BRAKE_FRACTION * p.max_accel * d_brake))
        return limit

    def _lookahead_s(self, look: float) -> float:
        """Arc length of the look-ahead point; it stops at the next sharp corner until the
        drone is within CORNER_SWITCH of it."""
        s_target = self.progress + look
        for i in range(1, len(self.points) - 1):
            if self.cum[i] <= self.progress + CORNER_SWITCH:
                continue
            if self.cum[i] >= s_target:
                break
            if self.turns[i] > CORNER_ANGLE:
                return self.cum[i]
        return s_target

    @staticmethod
    def _hold_xy(pose: Pose, hx: float, hy: float) -> tuple[float, float]:
        """Position-hold velocity toward (hx, hy)."""
        vx, vy = HOLD_GAIN * (hx - pose.x), HOLD_GAIN * (hy - pose.y)
        v = math.hypot(vx, vy)
        if v > HOLD_MAX_SPEED:
            vx, vy = vx * HOLD_MAX_SPEED / v, vy * HOLD_MAX_SPEED / v
        return vx, vy

    def _hold_command(self, pose: Pose, hold: tuple[float, float, float],
                      dt: float) -> tuple[float, float, float]:
        """Hover at `hold`: horizontal hold (acceleration-limited) and altitude P-control."""
        want = self._hold_xy(pose, hold[0], hold[1])
        ax, ay = want[0] - self.vel[0], want[1] - self.vel[1]
        dv, max_dv = math.hypot(ax, ay), self.p.max_accel * dt
        if dv > max_dv > 0:
            ax, ay = ax * max_dv / dv, ay * max_dv / dv
        self.vel = (self.vel[0] + ax, self.vel[1] + ay)
        vz = _clamp(self.p.altitude_gain * (hold[2] - pose.z), self.p.max_vertical_speed)
        return self.vel[0], self.vel[1], vz

    def _slew_yaw(self, want: float, dt: float) -> float:
        """Change the yaw-rate command by at most max_yaw_accel x dt (see module doc)."""
        step = self.p.max_yaw_accel * dt
        self.yaw_rate += max(-step, min(step, want - self.yaw_rate))
        return self.yaw_rate

    # ----------------------------------------------------------------- control step
    def update(self, pose: Pose, dt: float) -> Command:
        """One control step. Returns a zero command when there is no path or it is reached."""
        p = self.p
        if self.hold_yaw is None:
            self.hold_yaw = pose.yaw
        if not self.points or self.reached:
            if self.hold is None:
                self.vel = (0.0, 0.0)
                return Command(yaw_rate=self._slew_yaw(0.0, dt), reached=self.reached,
                               active=False)
            vx, vy, vz = self._hold_command(pose, self.hold, dt)
            return Command(vx=vx, vy=vy, vz=vz, yaw_rate=self._slew_yaw(0.0, dt),
                           reached=self.reached, active=False)

        self.progress, cross = self._project(pose.x, pose.y)
        total = self.cum[-1]
        goal = self.points[-1]
        d_goal_xy = math.hypot(goal[0] - pose.x, goal[1] - pose.y)
        z_err_goal = goal[2] - pose.z

        # Goal check.
        yaw_ok = (self.goal_yaw is None
                  or abs(wrap_angle(self.goal_yaw - pose.yaw)) <= p.goal_tolerance_yaw)
        near_end = total - self.progress <= p.goal_tolerance_xy + 1e-9
        if (near_end and d_goal_xy <= p.goal_tolerance_xy
                and abs(z_err_goal) <= p.goal_tolerance_xy and yaw_ok):
            self.reached = True
            self.hold = goal if goal[2] >= LANDED_Z else None
            if self.hold is None:
                self.vel = (0.0, 0.0)
                return Command(yaw_rate=self._slew_yaw(0.0, dt), reached=True,
                               cross_track=cross, active=False)
            vx, vy, vz = self._hold_command(pose, self.hold, dt)
            return Command(vx=vx, vy=vy, vz=vz, yaw_rate=self._slew_yaw(0.0, dt), reached=True,
                           cross_track=cross, active=False)

        speed_now = math.hypot(*self.vel)
        look = p.lookahead_min + (p.lookahead_max - p.lookahead_min) * min(1.0, speed_now / p.max_speed)
        tx, ty, tz = self.point_at(self._lookahead_s(look))
        if total - self.progress < look:
            tz = goal[2]

        # Altitude.
        vz = _clamp(p.altitude_gain * (tz - pose.z), p.max_vertical_speed)

        # Horizontal velocity toward the look-ahead point.
        dx, dy = tx - pose.x, ty - pose.y
        dist = math.hypot(dx, dy)
        if abs(tz - pose.z) > CLIMB_FIRST_ERROR or dist < 1e-6:
            # Climb or descend in place: hold the closest path point horizontally.
            hx, hy, _ = self.point_at(self.progress)
            want = self._hold_xy(pose, hx, hy)
        else:
            v = min(p.max_speed, self._corner_speed_limit(self.progress),
                    math.sqrt(2.0 * BRAKE_FRACTION * p.max_accel * d_goal_xy))
            v = max(v, MIN_SPEED) if d_goal_xy > p.goal_tolerance_xy / 2 else min(v, d_goal_xy)
            want = (v * dx / dist, v * dy / dist)
        # Acceleration limit on the velocity vector.
        ax, ay = want[0] - self.vel[0], want[1] - self.vel[1]
        dv = math.hypot(ax, ay)
        max_dv = p.max_accel * dt
        if dv > max_dv > 0:
            ax, ay = ax * max_dv / dv, ay * max_dv / dv
        self.vel = (self.vel[0] + ax, self.vel[1] + ay)

        # Yaw.
        if (self.goal_yaw is not None and total - self.progress <= p.lookahead_max):
            target = self.goal_yaw
        elif math.hypot(*want) > MIN_SPEED / 2:
            target = math.atan2(want[1], want[0])
            self.hold_yaw = target
        else:
            target = self.hold_yaw
        err = wrap_angle(target - pose.yaw)
        # P-control, but never faster than what can still be braked to zero at max_yaw_accel.
        want_rate = math.copysign(min(p.yaw_gain * abs(err), p.max_yaw_rate,
                                      math.sqrt(2.0 * p.max_yaw_accel * abs(err))), err)
        yaw_rate = self._slew_yaw(want_rate, dt)

        return Command(vx=self.vel[0], vy=self.vel[1], vz=vz, yaw_rate=yaw_rate,
                       reached=False, cross_track=cross, active=True)


def is_stale(now: float, pose_time: float | None, path_time: float | None,
             pose_timeout: float, path_timeout: float, following: bool) -> str | None:
    """Return why the command must be zero ('pose' or 'path'), or None if data is fresh.

    The path only goes stale while it is being followed: after the goal is reached, the
    drone hovers on purpose and no new path is expected.
    """
    if pose_time is None or now - pose_time > pose_timeout:
        return 'pose'
    if following and (path_time is None or now - path_time > path_timeout):
        return 'path'
    return None


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


def _bisect(cum: list[float], s: float) -> int:
    """Index i with cum[i] <= s < cum[i + 1]."""
    lo, hi = 0, len(cum) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if cum[mid] <= s:
            lo = mid
        else:
            hi = mid
    return lo
