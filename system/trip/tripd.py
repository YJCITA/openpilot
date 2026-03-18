#!/usr/bin/env python3
import time

from cereal.messaging import SubMaster
from openpilot.common.swaglog import cloudlog
from openpilot.system.trip.tripdb import ActiveTrip, connect_trip_db, finalize_active_trip, read_active_trip, save_active_trip

UPDATE_TIMEOUT_MS = 100
PERSIST_INTERVAL_S = 10.0
MAX_INTEGRATION_STEP_S = 1.0


def main() -> None:
  conn = connect_trip_db()
  sm = SubMaster(["deviceState", "carState"], poll="carState", ignore_avg_freq=["deviceState"])

  active_trip: ActiveTrip | None = None
  recovered = False
  started_prev = False
  last_sample_monotonic: float | None = None
  last_persist_monotonic = time.monotonic()

  cloudlog.info("tripd started")

  try:
    while True:
      sm.update(UPDATE_TIMEOUT_MS)

      if not sm.seen["deviceState"]:
        continue

      now_wall = int(time.time())
      now_monotonic = time.monotonic()
      started = sm["deviceState"].started

      if not recovered:
        stored_trip = read_active_trip(conn)
        if stored_trip is not None:
          if started:
            active_trip = stored_trip
            started_prev = True
            last_sample_monotonic = now_monotonic
            last_persist_monotonic = now_monotonic
            cloudlog.info(f"tripd resumed active trip from {stored_trip.start_time}")
          else:
            recorded, duration_seconds = finalize_active_trip(conn, stored_trip, stored_trip.last_update_time)
            cloudlog.info(
              f"tripd recovered dangling trip: recorded={recorded}, duration_s={duration_seconds}, distance_m={stored_trip.distance_m:.1f}"
            )
        recovered = True

      if started and not started_prev:
        active_trip = ActiveTrip(start_time=now_wall, distance_m=0.0, last_update_time=now_wall)
        save_active_trip(conn, active_trip)
        last_sample_monotonic = now_monotonic
        last_persist_monotonic = now_monotonic
        cloudlog.info(f"tripd started trip at {now_wall}")
      elif not started and started_prev:
        if active_trip is not None:
          recorded, duration_seconds = finalize_active_trip(conn, active_trip, now_wall)
          cloudlog.info(
            f"tripd finished trip: recorded={recorded}, duration_s={duration_seconds}, distance_m={active_trip.distance_m:.1f}"
          )
        active_trip = None
        last_sample_monotonic = None
        last_persist_monotonic = now_monotonic

      started_prev = started

      if active_trip is None or not started:
        continue

      if sm.updated["carState"]:
        if last_sample_monotonic is not None:
          dt = now_monotonic - last_sample_monotonic
          if 0.0 < dt <= MAX_INTEGRATION_STEP_S:
            active_trip.distance_m += abs(sm["carState"].vEgo) * dt
        last_sample_monotonic = now_monotonic

      if now_monotonic - last_persist_monotonic >= PERSIST_INTERVAL_S:
        active_trip.last_update_time = now_wall
        save_active_trip(conn, active_trip)
        last_persist_monotonic = now_monotonic
  finally:
    if active_trip is not None:
      active_trip.last_update_time = int(time.time())
      save_active_trip(conn, active_trip)
    conn.close()
