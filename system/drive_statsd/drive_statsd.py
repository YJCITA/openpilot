#!/usr/bin/env python3
"""
Standalone drive statistics daemon. Records mileage locally via speed (vEgo) integration.
Writes to /data/media/0/drive_info/stats.json for UI consumption.
"""
import json
import os
import time
from typing import NoReturn

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper, DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.common.utils import atomic_write
from openpilot.system.hardware.hw import Paths

# vEgo is m/s; 1 mile = 1609.344 m
M_PER_MI = 1609.344
SEC_PER_MIN = 60.0
WEEK_SEC = 7 * 24 * 3600
# Prune trips older than 10 years to bound file size while keeping all-time stats
TRIPS_MAX_AGE_SEC = 10 * 365 * 24 * 3600

STATS_PATH = os.path.join(Paths.drive_info_root(), "stats.json")


def default_stats() -> dict:
  return {
    "all": {"routes": 0, "distance": 0.0, "minutes": 0},
    "week": {"routes": 0, "distance": 0.0, "minutes": 0},
    "trips": [],
  }


def load_stats() -> dict:
  try:
    with open(STATS_PATH, "r") as f:
      data = json.load(f)
    if "trips" not in data:
      data["trips"] = []
    if "all" not in data:
      data["all"] = default_stats()["all"]
    if "week" not in data:
      data["week"] = default_stats()["week"]
    return data
  except (FileNotFoundError, json.JSONDecodeError):
    return default_stats()


def compute_aggregates(trips: list, now_sec: float) -> tuple[dict, dict]:
  """Compute all-time and past-week aggregates from trips list."""
  all_routes = len(trips)
  all_distance = sum(t["distance"] for t in trips)
  all_minutes = sum(t["minutes"] for t in trips)
  week_cutoff = now_sec - WEEK_SEC
  week_trips = [t for t in trips if t["ts"] >= week_cutoff]
  week_routes = len(week_trips)
  week_distance = sum(t["distance"] for t in week_trips)
  week_minutes = sum(t["minutes"] for t in week_trips)
  return (
    {"routes": all_routes, "distance": round(all_distance, 4), "minutes": round(all_minutes, 1)},
    {"routes": week_routes, "distance": round(week_distance, 4), "minutes": round(week_minutes, 1)},
  )


def prune_trips(trips: list, now_sec: float) -> list:
  """Keep only trips within TRIPS_MAX_AGE_SEC for file size."""
  cutoff = now_sec - TRIPS_MAX_AGE_SEC
  return [t for t in trips if t["ts"] >= cutoff]


def write_stats(stats: dict) -> None:
  root = os.path.dirname(STATS_PATH)
  os.makedirs(root, exist_ok=True)
  # UI reads all/week only; we persist trips for week recompute
  out = {"all": stats["all"], "week": stats["week"], "trips": stats["trips"]}
  with atomic_write(STATS_PATH) as f:
    json.dump(out, f, indent=0)


def main() -> NoReturn:
  sm = messaging.SubMaster(["deviceState", "carState"])
  rk = Ratekeeper(1 / DT_CTRL, print_delay_threshold=0)
  started_prev = False
  trip_distance_mi = 0.0
  trip_minutes = 0.0
  last_t = None

  while True:
    sm.update()
    started = sm["deviceState"].started
    t = time.time()

    if started:
      # Get vEgo (m/s); carState.vEgo can be List(Float32) in capnp
      v_ego = 0.0
      if sm.updated["carState"] and sm.recv_frame["carState"] > 0:
        try:
          v = sm["carState"].vEgo
          if hasattr(v, "__len__") and len(v) > 0:
            v_ego = float(v[0])
          else:
            v_ego = float(v)
        except (TypeError, IndexError):
          pass
      v_ego = max(0.0, v_ego)

      if last_t is not None:
        dt = t - last_t
        trip_distance_mi += v_ego * dt / M_PER_MI
        trip_minutes += dt / SEC_PER_MIN
      last_t = t
    else:
      last_t = None
      if started_prev:
        # Just went offroad: persist this trip
        if trip_distance_mi > 0 or trip_minutes > 0:
          stats = load_stats()
          # We need trips for week recompute; if file was UI-only (no trips key), rebuild from all/week (lose week history)
          trips = stats.get("trips", [])
          trips.append({
            "ts": int(t),
            "distance": round(trip_distance_mi, 4),
            "minutes": round(trip_minutes, 1),
          })
          trips = prune_trips(trips, t)
          stats["trips"] = trips
          all_agg, week_agg = compute_aggregates(trips, t)
          stats["all"] = all_agg
          stats["week"] = week_agg
          write_stats(stats)
          cloudlog.info("drive_statsd: trip saved", distance_mi=trip_distance_mi, minutes=trip_minutes)
        trip_distance_mi = 0.0
        trip_minutes = 0.0

    started_prev = started
    rk.keep_time()


if __name__ == "__main__":
  main()
