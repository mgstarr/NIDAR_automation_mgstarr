"""Unit tests for airmouse_nav.core.mission: full transition table and the return budget."""

import pytest

from airmouse_nav.core.mission import (ABORT_TRIGGERS, TERMINAL, Mission, State, Trigger,
                                       should_return)

EXPECTED = {
    (State.IDLE, Trigger.START): State.TAKEOFF,
    (State.TAKEOFF, Trigger.ALTITUDE_REACHED): State.EXPLORE,
    (State.TAKEOFF, Trigger.RECALL): State.RETURN_HOME,
    (State.EXPLORE, Trigger.EXPLORATION_DONE): State.RETURN_HOME,
    (State.EXPLORE, Trigger.BUDGET_LOW): State.RETURN_HOME,
    (State.EXPLORE, Trigger.RECALL): State.RETURN_HOME,
    (State.RETURN_HOME, Trigger.AT_HOME): State.LAND,
    (State.LAND, Trigger.LANDED): State.DONE,
}


@pytest.mark.parametrize('state', list(State))
@pytest.mark.parametrize('trigger', list(Trigger))
def test_transition_table(state: State, trigger: Trigger) -> None:
    """Every state x trigger: listed transitions happen, aborts work from every non-terminal
    state, terminal states never change, everything else is ignored."""
    m = Mission()
    m.state = state
    new = m.handle(trigger)
    if state in TERMINAL:
        assert new == state
    elif trigger in ABORT_TRIGGERS:
        assert new == State.ABORT
    else:
        assert new == EXPECTED.get((state, trigger), state)


def test_full_mission_sequence() -> None:
    """The nominal sequence ends at DONE and is recorded in the history."""
    m = Mission()
    for t in (Trigger.START, Trigger.ALTITUDE_REACHED, Trigger.EXPLORATION_DONE,
              Trigger.AT_HOME, Trigger.LANDED):
        m.handle(t)
    assert m.state == State.DONE and len(m.history) == 5


@pytest.mark.parametrize('elapsed,expected', [(0, False), (1500, False), (1507, False),
                                              (1508, True), (1700, True)])
def test_return_budget_fires_at_the_right_time(elapsed: float, expected: bool) -> None:
    """30 min limit, 2 min reserve; home 63 m away at 0.6 m/s x 1.5 + 15 s landing = 172.5 s
    needed, so it fires once 1800 - elapsed - 172.5 < 120, i.e. after 1507.5 s."""
    assert should_return(elapsed, 1800, 63.0, 0.6, 1.5, 15.0, 120) is expected


def test_return_budget_short_trip() -> None:
    """With home 12 m away the trigger fires just after 1635 s."""
    assert not should_return(1634, 1800, 12.0, 0.6, 1.5, 15.0, 120)
    assert should_return(1636, 1800, 12.0, 0.6, 1.5, 15.0, 120)
