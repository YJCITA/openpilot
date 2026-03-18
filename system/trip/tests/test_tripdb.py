from openpilot.system.trip.tripdb import (
  ActiveTrip,
  MIN_TRIP_DISTANCE_M,
  MIN_TRIP_DURATION_S,
  WEEK_WINDOW_S,
  connect_trip_db,
  finalize_active_trip,
  load_trip_stats,
  save_active_trip,
)


def test_short_or_too_near_trips_are_not_recorded(tmp_path):
  db_path = tmp_path / "trips.db"

  with connect_trip_db(db_path) as conn:
    short_trip = ActiveTrip(start_time=1_000, distance_m=MIN_TRIP_DISTANCE_M + 50.0, last_update_time=1_100)
    save_active_trip(conn, short_trip)
    recorded, _ = finalize_active_trip(conn, short_trip, short_trip.start_time + MIN_TRIP_DURATION_S - 1)
    assert not recorded

    short_distance_trip = ActiveTrip(start_time=2_000, distance_m=MIN_TRIP_DISTANCE_M - 1.0, last_update_time=2_500)
    save_active_trip(conn, short_distance_trip)
    recorded, _ = finalize_active_trip(conn, short_distance_trip, short_distance_trip.start_time + MIN_TRIP_DURATION_S + 10)
    assert not recorded

  stats = load_trip_stats(db_path, now_ts=10_000)
  assert stats["all"]["routes"] == 0
  assert stats["all"]["distance"] == 0.0
  assert stats["all"]["minutes"] == 0.0


def test_valid_trips_are_aggregated_for_all_time_and_past_week(tmp_path):
  db_path = tmp_path / "trips.db"
  now_ts = 2_000_000

  with connect_trip_db(db_path) as conn:
    recent_trip = ActiveTrip(
      start_time=now_ts - 3_600,
      distance_m=2_500.0,
      last_update_time=now_ts - 1_800,
    )
    save_active_trip(conn, recent_trip)
    recorded, _ = finalize_active_trip(conn, recent_trip, recent_trip.start_time + MIN_TRIP_DURATION_S + 600)
    assert recorded

    old_trip = ActiveTrip(
      start_time=now_ts - WEEK_WINDOW_S - 7_200,
      distance_m=4_000.0,
      last_update_time=now_ts - WEEK_WINDOW_S - 3_600,
    )
    save_active_trip(conn, old_trip)
    recorded, _ = finalize_active_trip(conn, old_trip, old_trip.start_time + MIN_TRIP_DURATION_S + 900)
    assert recorded

  stats = load_trip_stats(db_path, now_ts=now_ts)
  assert stats["all"]["routes"] == 2
  assert stats["week"]["routes"] == 1
  assert stats["all"]["minutes"] > stats["week"]["minutes"]
  assert stats["all"]["distance"] > stats["week"]["distance"] > 0.0
