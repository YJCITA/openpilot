#!/usr/bin/env python3
import datetime
import subprocess
import time
import math
from typing import NoReturn

import cereal.messaging as messaging
from openpilot.common.time_helpers import min_date, system_time_valid
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service


def get_timezone_offset_from_gps(latitude, longitude):
    """
    Calculate UTC offset (hours) from GPS coordinates
    Uses predefined timezone mapping table for better accuracy
    """
    # Predefined major timezone mapping table (latitude range, longitude range, UTC offset)
    timezone_ranges = [
        # China
        ((18, 54), (73, 135), 8),  # China Standard Time UTC+8
        # US Eastern
        ((24, 49), (-85, -66), -5),  # EST UTC-5
        # US Central
        ((25, 49), (-106, -84), -6),  # CST UTC-6
        # US Mountain
        ((31, 49), (-115, -101), -7),  # MST UTC-7
        # US Pacific
        ((32, 49), (-125, -114), -8),  # PST UTC-8
        # Western Europe
        ((35, 71), (-25, 40), 0),  # WET UTC+0
        # Central Europe
        ((35, 71), (5, 40), 1),  # CET UTC+1
        # Japan
        ((24, 46), (123, 146), 9),  # JST UTC+9
        # India
        ((6, 37), (68, 97), 5.5),  # IST UTC+5:30
        # Australia Eastern
        ((-44, -10), (113, 154), 10),  # AEST UTC+10
        # Russia Moscow
        ((41, 82), (19, 180), 3),  # MSK UTC+3
    ]
    
    # Find matching timezone
    for (lat_min, lat_max), (lon_min, lon_max), offset in timezone_ranges:
        if lat_min <= latitude <= lat_max and lon_min <= longitude <= lon_max:
            return offset
    
    # If no matching timezone found, use simple longitude-based estimation
    # Approximately 15 degrees longitude equals 1 hour time difference
    timezone_offset = longitude / 15.0
    return round(timezone_offset)


def set_timezone_from_gps(latitude, longitude, last_timezone=None):
    """
    Set system timezone based on GPS coordinates
    """
    try:
        offset = get_timezone_offset_from_gps(latitude, longitude)
        
        # Construct timezone string
        if offset == 0:
            timezone_str = "UTC"
        elif offset > 0:
            if offset == int(offset):  # Integer hours
                timezone_str = f"UTC+{int(offset):02d}:00"
            else:  # Half-hour offset (like India)
                hours = int(offset)
                minutes = int((offset - hours) * 60)
                timezone_str = f"UTC+{hours:02d}:{minutes:02d}"
        else:
            if offset == int(offset):  # Integer hours
                timezone_str = f"UTC{int(offset):03d}:00"
            else:  # Half-hour offset
                hours = int(offset)
                minutes = int((offset - hours) * 60)
                timezone_str = f"UTC{hours:03d}:{minutes:02d}"
        
        # Skip setting if timezone hasn't changed
        if last_timezone == timezone_str:
            return timezone_str
        
        cloudlog.debug(f"Setting timezone to {timezone_str} based on GPS: {latitude}, {longitude}")
        
        # Use timedatectl to set timezone
        subprocess.run(["timedatectl", "set-timezone", timezone_str], check=True)
        return timezone_str
        
    except subprocess.CalledProcessError as e:
        cloudlog.exception(f"Failed to set timezone: {e}")
        return last_timezone


def set_time(new_time, latitude=None, longitude=None, last_timezone=None):
    # If GPS coordinates are available, set timezone first
    if latitude is not None and longitude is not None:
        last_timezone = set_timezone_from_gps(latitude, longitude, last_timezone)
    
    diff = datetime.datetime.now() - new_time
    if abs(diff) < datetime.timedelta(seconds=10):
        cloudlog.debug(f"Time diff too small: {diff}")
        return last_timezone

    cloudlog.debug(f"Setting time to {new_time}")
    try:
        # Set UTC time, let system automatically convert to local time
        subprocess.run(f"TZ=UTC date -s '{new_time}'", shell=True, check=True)
    except subprocess.CalledProcessError:
        cloudlog.exception("timed.failed_setting_time")
    
    return last_timezone


def main() -> NoReturn:
  """
    timed has two responsibilities:
    - getting the current time from GPS
    - publishing the time in the logs

    AGNOS will also use NTP to update the time.
  """

  params = Params()
  gps_location_service = get_gps_location_service(params)

  pm = messaging.PubMaster(['clocks'])
  sm = messaging.SubMaster([gps_location_service])
  
  # Cache last set timezone to avoid frequent settings
  last_timezone = None
  last_position = None
  
  while True:
    sm.update(1000)

    msg = messaging.new_message('clocks')
    msg.valid = system_time_valid()
    msg.clocks.wallTimeNanos = time.time_ns()
    pm.send('clocks', msg)

    gps = sm[gps_location_service]
    gps_time = datetime.datetime.fromtimestamp(gps.unixTimestampMillis / 1000.)
    if not sm.updated[gps_location_service] or (time.monotonic() - sm.logMonoTime[gps_location_service] / 1e9) > 2.0:
      continue
    if not gps.hasFix:
      continue
    if gps_time < min_date():
      continue

    # Check if position has changed significantly (more than 1 degree)
    current_position = (round(gps.latitude, 1), round(gps.longitude, 1))
    if last_position != current_position:
      cloudlog.debug(f"GPS position changed from {last_position} to {current_position}")
      last_position = current_position
      last_timezone = None  # Reset timezone cache

    # Pass GPS coordinates for timezone setting
    last_timezone = set_time(gps_time, gps.latitude, gps.longitude, last_timezone)
    time.sleep(10)

if __name__ == "__main__":
  main()
