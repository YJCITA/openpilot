import os
import sqlite3
import time
from dataclasses import dataclass
from pathlib import Path

from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import PC

TRIP_DB_FILENAME = "trips.db"
MIN_TRIP_DURATION_S = 3 * 60
MIN_TRIP_DISTANCE_M = 1000.0
WEEK_WINDOW_S = 7 * 24 * 60 * 60
ACTIVE_TRIP_ROW_ID = 1
METERS_TO_MILES = CV.KPH_TO_MPH / 1000.0


@dataclass
class ActiveTrip:
  start_time: int
  distance_m: float
  last_update_time: int


def get_drive_data_root() -> Path:
  if PC:
    comma_home = Path.home() / (".comma" + os.environ.get("OPENPILOT_PREFIX", ""))
    return comma_home / "media" / "0" / "drive_data"

  return Path("/data/media/0/drive_data")


def get_trip_db_path(db_path: str | Path | None = None) -> Path:
  if db_path is not None:
    path = Path(db_path)
  else:
    path = get_drive_data_root() / TRIP_DB_FILENAME

  path.parent.mkdir(parents=True, exist_ok=True)
  return path


def connect_trip_db(db_path: str | Path | None = None) -> sqlite3.Connection:
  conn = sqlite3.connect(get_trip_db_path(db_path), timeout=30)
  conn.row_factory = sqlite3.Row
  conn.execute("PRAGMA journal_mode=WAL")
  conn.execute("PRAGMA synchronous=NORMAL")
  conn.execute("PRAGMA busy_timeout = 30000")
  init_trip_db(conn)
  return conn


def init_trip_db(conn: sqlite3.Connection) -> None:
  conn.execute(
    """
    CREATE TABLE IF NOT EXISTS trips (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      start_time INTEGER NOT NULL,
      end_time INTEGER NOT NULL,
      duration_seconds INTEGER NOT NULL,
      distance_m REAL NOT NULL,
      created_at INTEGER NOT NULL DEFAULT (strftime('%s', 'now'))
    )
    """
  )
  conn.execute("CREATE INDEX IF NOT EXISTS idx_trips_end_time ON trips(end_time)")
  conn.execute(
    """
    CREATE TABLE IF NOT EXISTS active_trip (
      session_id INTEGER PRIMARY KEY CHECK (session_id = 1),
      start_time INTEGER NOT NULL,
      distance_m REAL NOT NULL,
      last_update_time INTEGER NOT NULL
    )
    """
  )
  conn.commit()


def read_active_trip(conn: sqlite3.Connection) -> ActiveTrip | None:
  row = conn.execute(
    "SELECT start_time, distance_m, last_update_time FROM active_trip WHERE session_id = ?",
    (ACTIVE_TRIP_ROW_ID,),
  ).fetchone()
  if row is None:
    return None

  return ActiveTrip(
    start_time=int(row["start_time"]),
    distance_m=float(row["distance_m"]),
    last_update_time=int(row["last_update_time"]),
  )


def save_active_trip(conn: sqlite3.Connection, trip: ActiveTrip) -> None:
  conn.execute(
    """
    INSERT INTO active_trip (session_id, start_time, distance_m, last_update_time)
    VALUES (?, ?, ?, ?)
    ON CONFLICT(session_id) DO UPDATE SET
      start_time = excluded.start_time,
      distance_m = excluded.distance_m,
      last_update_time = excluded.last_update_time
    """,
    (ACTIVE_TRIP_ROW_ID, trip.start_time, trip.distance_m, trip.last_update_time),
  )
  conn.commit()


def clear_active_trip(conn: sqlite3.Connection) -> None:
  conn.execute("DELETE FROM active_trip WHERE session_id = ?", (ACTIVE_TRIP_ROW_ID,))
  conn.commit()


def finalize_active_trip(conn: sqlite3.Connection, trip: ActiveTrip, end_time: int) -> tuple[bool, int]:
  normalized_end_time = max(int(end_time), trip.start_time)
  duration_seconds = normalized_end_time - trip.start_time
  should_record = duration_seconds >= MIN_TRIP_DURATION_S and trip.distance_m >= MIN_TRIP_DISTANCE_M

  if should_record:
    conn.execute(
      """
      INSERT INTO trips (start_time, end_time, duration_seconds, distance_m)
      VALUES (?, ?, ?, ?)
      """,
      (trip.start_time, normalized_end_time, duration_seconds, trip.distance_m),
    )

  conn.execute("DELETE FROM active_trip WHERE session_id = ?", (ACTIVE_TRIP_ROW_ID,))
  conn.commit()
  return should_record, duration_seconds


def _stats_row_to_dict(row: sqlite3.Row) -> dict[str, float | int]:
  routes = int(row["routes"])
  distance_m = float(row["distance_m"] or 0.0)
  duration_seconds = int(row["duration_seconds"] or 0)
  return {
    "routes": routes,
    "distance": distance_m * METERS_TO_MILES,
    "minutes": duration_seconds / 60.0,
  }


def _query_trip_stats(conn: sqlite3.Connection, min_end_time: int | None = None) -> dict[str, float | int]:
  if min_end_time is None:
    row = conn.execute(
      """
      SELECT
        COUNT(*) AS routes,
        COALESCE(SUM(distance_m), 0.0) AS distance_m,
        COALESCE(SUM(duration_seconds), 0) AS duration_seconds
      FROM trips
      """
    ).fetchone()
  else:
    row = conn.execute(
      """
      SELECT
        COUNT(*) AS routes,
        COALESCE(SUM(distance_m), 0.0) AS distance_m,
        COALESCE(SUM(duration_seconds), 0) AS duration_seconds
      FROM trips
      WHERE end_time >= ?
      """,
      (min_end_time,),
    ).fetchone()

  return _stats_row_to_dict(row)


def load_trip_stats(db_path: str | Path | None = None, now_ts: int | None = None) -> dict[str, dict[str, float | int]]:
  timestamp = int(time.time()) if now_ts is None else int(now_ts)
  try:
    with connect_trip_db(db_path) as conn:
      return {
        "all": _query_trip_stats(conn),
        "week": _query_trip_stats(conn, timestamp - WEEK_WINDOW_S),
      }
  except Exception:
    cloudlog.exception("Failed to load local trip stats")
    zero_stats = {"routes": 0, "distance": 0.0, "minutes": 0.0}
    return {"all": dict(zero_stats), "week": dict(zero_stats)}
