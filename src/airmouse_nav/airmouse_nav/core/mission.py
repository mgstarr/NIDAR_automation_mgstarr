"""Mission state machine (no ROS imports).

States:   IDLE, TAKEOFF, EXPLORE, RETURN_HOME, LAND, DONE, ABORT
Triggers: START, ALTITUDE_REACHED, EXPLORATION_DONE, BUDGET_LOW, RECALL, AT_HOME, LANDED,
          ABORT, LOCALIZATION_LOST, GEOFENCE_BREACH

    IDLE --START--> TAKEOFF --ALTITUDE_REACHED--> EXPLORE
    EXPLORE --EXPLORATION_DONE | BUDGET_LOW | RECALL--> RETURN_HOME
    TAKEOFF --RECALL--> RETURN_HOME           (recall before exploring: go home and land)
    RETURN_HOME --AT_HOME--> LAND --LANDED--> DONE
    any airborne-or-idle state --ABORT | LOCALIZATION_LOST | GEOFENCE_BREACH--> ABORT
    ABORT: hover, then land in place; stays ABORT (terminal). DONE is terminal.
    Every other (state, trigger) pair is ignored (state unchanged).

Public API:

    m = Mission()
    m.handle(Trigger.START) -> State            # applies the table, returns the new state
    m.state, m.history (list of (old, trigger, new))
    should_return(elapsed, time_limit, home_cost_m, speed, safety_factor, landing_time, reserve)
"""

from __future__ import annotations

from enum import Enum


class State(str, Enum):
    """Mission states (published on /mission/state as their name)."""

    IDLE = 'IDLE'
    TAKEOFF = 'TAKEOFF'
    EXPLORE = 'EXPLORE'
    RETURN_HOME = 'RETURN_HOME'
    LAND = 'LAND'
    DONE = 'DONE'
    ABORT = 'ABORT'


class Trigger(str, Enum):
    """Events that can change the state."""

    START = 'START'
    ALTITUDE_REACHED = 'ALTITUDE_REACHED'
    EXPLORATION_DONE = 'EXPLORATION_DONE'
    BUDGET_LOW = 'BUDGET_LOW'
    RECALL = 'RECALL'
    AT_HOME = 'AT_HOME'
    LANDED = 'LANDED'
    ABORT = 'ABORT'
    LOCALIZATION_LOST = 'LOCALIZATION_LOST'
    GEOFENCE_BREACH = 'GEOFENCE_BREACH'


ABORT_TRIGGERS = {Trigger.ABORT, Trigger.LOCALIZATION_LOST, Trigger.GEOFENCE_BREACH}
TERMINAL = {State.DONE, State.ABORT}

TRANSITIONS: dict[tuple[State, Trigger], State] = {
    (State.IDLE, Trigger.START): State.TAKEOFF,
    (State.TAKEOFF, Trigger.ALTITUDE_REACHED): State.EXPLORE,
    (State.TAKEOFF, Trigger.RECALL): State.RETURN_HOME,
    (State.EXPLORE, Trigger.EXPLORATION_DONE): State.RETURN_HOME,
    (State.EXPLORE, Trigger.BUDGET_LOW): State.RETURN_HOME,
    (State.EXPLORE, Trigger.RECALL): State.RETURN_HOME,
    (State.RETURN_HOME, Trigger.AT_HOME): State.LAND,
    (State.LAND, Trigger.LANDED): State.DONE,
}
for _s in State:
    if _s not in TERMINAL:
        for _t in ABORT_TRIGGERS:
            TRANSITIONS[(_s, _t)] = State.ABORT


class Mission:
    """The state machine. Pure: no timers, no I/O."""

    def __init__(self) -> None:
        self.state = State.IDLE
        self.history: list[tuple[State, Trigger, State]] = []

    def handle(self, trigger: Trigger) -> State:
        """Apply a trigger; unknown pairs leave the state unchanged."""
        new = TRANSITIONS.get((self.state, trigger), self.state)
        if new != self.state:
            self.history.append((self.state, trigger, new))
            self.state = new
        return self.state

    @property
    def airborne_states(self) -> set[State]:
        """States in which the drone is (or should be) flying."""
        return {State.TAKEOFF, State.EXPLORE, State.RETURN_HOME, State.LAND, State.ABORT}


def should_return(elapsed: float, time_limit: float, home_cost: float, speed: float,
                  safety_factor: float, landing_time: float, reserve: float) -> bool:
    """Return-budget rule: remaining time - (home travel time x factor + landing) < reserve.

    Args:
        elapsed: Seconds since take-off.
        time_limit: mission_time_limit (s).
        home_cost: Travel cost from the drone to home (m, Dijkstra from home).
        speed: Cruise speed used to turn cost into time (m/s; max_speed).
        safety_factor: return_safety_factor.
        landing_time: Seconds needed to land.
        reserve: return_reserve (s).
    """
    remaining = time_limit - elapsed
    need = home_cost / max(speed, 1e-6) * safety_factor + landing_time
    return remaining - need < reserve
