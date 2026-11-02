"""Mission state machine.

Planned (Phase 5):
  IDLE -> TAKEOFF (on /mission/start) -> EXPLORE (altitude reached) -> RETURN_HOME
  -> LAND (at home) -> DONE.
  ABORT from any state (on /mission/abort, stale localization or geofence breach):
  hover, then land in place.
  EXPLORE -> RETURN_HOME when no map or view frontiers remain, or when the remaining
  time minus (home travel time x safety factor + landing time) drops below the reserve.
  /mission/return_home forces RETURN_HOME (emergency recall).
"""
