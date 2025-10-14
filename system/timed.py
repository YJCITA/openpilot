#!/usr/bin/env python3
import datetime
import os
import subprocess
import time
import math
from typing import NoReturn

import cereal.messaging as messaging
from openpilot.common.time_helpers import min_date, system_time_valid
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service


def get_timezone_offset_from_gps(latitude, longitude, date=None):
    """
    Calculate UTC offset (hours) from GPS coordinates
    Uses predefined timezone mapping table for better accuracy
    Supports basic daylight saving time adjustments
    """
    if date is None:
        date = datetime.datetime.now()
    
    # Predefined major timezone mapping table (latitude range, longitude range, UTC offset, DST offset)
    timezone_ranges = [
        # China (no DST)
        ((18, 54), (73, 135), 8, 0),  # China Standard Time UTC+8
        # US Eastern (DST: +1 hour from 2nd Sunday in March to 1st Sunday in November)
        ((24, 49), (-85, -66), -5, 1),  # EST/EDT UTC-5/-4
        # US Central (DST: +1 hour from 2nd Sunday in March to 1st Sunday in November)
        ((25, 49), (-106, -84), -6, 1),  # CST/CDT UTC-6/-5
        # US Mountain (DST: +1 hour from 2nd Sunday in March to 1st Sunday in November)
        ((31, 49), (-115, -101), -7, 1),  # MST/MDT UTC-7/-6
        # US Pacific (DST: +1 hour from 2nd Sunday in March to 1st Sunday in November)
        ((32, 49), (-125, -114), -8, 1),  # PST/PDT UTC-8/-7
        # Western Europe (DST: +1 hour from last Sunday in March to last Sunday in October)
        ((35, 71), (-25, 40), 0, 1),  # WET/WEST UTC+0/+1
        # Central Europe (DST: +1 hour from last Sunday in March to last Sunday in October)
        ((35, 71), (5, 40), 1, 1),  # CET/CEST UTC+1/+2
        # Japan (no DST)
        ((24, 46), (123, 146), 9, 0),  # JST UTC+9
        # India (no DST)
        ((6, 37), (68, 97), 5.5, 0),  # IST UTC+5:30
        # Australia Eastern (DST: +1 hour from 1st Sunday in October to 1st Sunday in April)
        ((-44, -10), (113, 154), 10, 1),  # AEST/AEDT UTC+10/+11
        # Russia Moscow (no DST since 2014)
        ((41, 82), (19, 180), 3, 0),  # MSK UTC+3
    ]
    
    # Find matching timezone
    for (lat_min, lat_max), (lon_min, lon_max), base_offset, dst_offset in timezone_ranges:
        if lat_min <= latitude <= lat_max and lon_min <= longitude <= lon_max:
            # Simple DST check (this is a basic implementation)
            if dst_offset > 0 and is_dst_active(date, latitude, longitude):
                return base_offset + dst_offset
            return base_offset
    
    # If no matching timezone found, use simple longitude-based estimation
    # Approximately 15 degrees longitude equals 1 hour time difference
    timezone_offset = longitude / 15.0
    return round(timezone_offset)


def is_dst_active(date, latitude, longitude):
    """
    Simple DST check - this is a basic implementation
    For production use, consider using a proper timezone library
    """
    # This is a simplified DST check for major regions
    # US DST: 2nd Sunday in March to 1st Sunday in November
    if -125 <= longitude <= -66 and 24 <= latitude <= 49:  # US
        march_second_sunday = get_second_sunday_of_month(date.year, 3)
        november_first_sunday = get_first_sunday_of_month(date.year, 11)
        return march_second_sunday <= date.date() <= november_first_sunday
    
    # EU DST: Last Sunday in March to last Sunday in October
    elif -25 <= longitude <= 40 and 35 <= latitude <= 71:  # Europe
        march_last_sunday = get_last_sunday_of_month(date.year, 3)
        october_last_sunday = get_last_sunday_of_month(date.year, 10)
        return march_last_sunday <= date.date() <= october_last_sunday
    
    # Australia DST: 1st Sunday in October to 1st Sunday in April
    elif 113 <= longitude <= 154 and -44 <= latitude <= -10:  # Australia
        october_first_sunday = get_first_sunday_of_month(date.year, 10)
        april_first_sunday = get_first_sunday_of_month(date.year, 4)
        if date.month >= 10:
            return october_first_sunday <= date.date()
        elif date.month <= 4:
            return date.date() <= april_first_sunday
    
    return False


def get_first_sunday_of_month(year, month):
    """Get the first Sunday of a given month"""
    first_day = datetime.date(year, month, 1)
    days_ahead = 6 - first_day.weekday()  # Sunday is 6
    if days_ahead == 7:
        days_ahead = 0
    return first_day + datetime.timedelta(days=days_ahead)


def get_second_sunday_of_month(year, month):
    """Get the second Sunday of a given month"""
    first_sunday = get_first_sunday_of_month(year, month)
    return first_sunday + datetime.timedelta(days=7)


def get_last_sunday_of_month(year, month):
    """Get the last Sunday of a given month"""
    if month == 12:
        next_month = datetime.date(year + 1, 1, 1)
    else:
        next_month = datetime.date(year, month + 1, 1)
    
    last_day = next_month - datetime.timedelta(days=1)
    days_back = last_day.weekday() + 1  # Sunday is 6, so 6+1=7, 7%7=0
    if days_back == 7:
        days_back = 0
    return last_day - datetime.timedelta(days=days_back)


def set_timezone_from_gps(latitude, longitude, last_timezone=None, date=None):
    """
    Set system timezone based on GPS coordinates
    """
    try:
        offset = get_timezone_offset_from_gps(latitude, longitude, date)
        
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
    # -YJ- Always convert UTC time to Beijing Time (UTC+8)
    # If GPS coordinates are available, set timezone first
    print("latitude: ", latitude, "longitude: ", longitude)
    if latitude is not None and longitude is not None:
        last_timezone = set_timezone_from_gps(latitude, longitude, last_timezone, new_time)
        
        # Convert GPS UTC time to local time
        offset = get_timezone_offset_from_gps(latitude, longitude, new_time)
        local_time = new_time + datetime.timedelta(hours=offset)
        cloudlog.debug(f"Converting GPS UTC time {new_time} to local time {local_time} (offset: {offset}h)")
        new_time = local_time
    else:
        # -YJ- No GPS coordinates, use default Beijing Time (UTC+8)
        print("!!!!!!!!!!!!!!!!!!!! No GPS coordinates, converting UTC time {new_time} to Beijing Time (UTC+8)")
        cloudlog.info(f"No GPS coordinates, converting UTC time {new_time} to Beijing Time (UTC+8)")
        new_time = new_time + datetime.timedelta(hours=8)
    
    diff = datetime.datetime.now() - new_time
    if abs(diff) < datetime.timedelta(seconds=10):
        cloudlog.debug(f"Time diff too small: {diff}")
        return last_timezone

    cloudlog.debug(f"Setting local time to {new_time}")
    try:
        # Set local time directly (format: YYYY-MM-DD HH:MM:SS)
        time_str = new_time.strftime("%Y-%m-%d %H:%M:%S")
        subprocess.run(f"date -s '{time_str}'", shell=True, check=True)
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
  
  # -YJ- Initialize system time to Beijing Time at startup
  # If no GPS, assume current system time is UTC and convert to Beijing Time
  try:
    current_time = datetime.datetime.now()
    beijing_time = current_time + datetime.timedelta(hours=8)
    time_str = beijing_time.strftime("%Y-%m-%d %H:%M:%S")
    
    print(f"!!!!!!!!!!!!!!!!!!!! Initializing system time to Beijing Time")
    print(f"pre system time (UTC): {current_time}")
    # print(f"Setting to Beijing Time: {beijing_time}")
    
    subprocess.run(f"date -s '{time_str}'", shell=True, check=True)

    current_time = datetime.datetime.now()  # 现在读取到 12:00:00
    print(f"Current system time (Beijing Time): {current_time}")
    cloudlog.info(f"Initialized system time to Beijing Time: {beijing_time}")
    last_timezone = "UTC+08:00"
  except subprocess.CalledProcessError as e:
    cloudlog.exception(f"Failed to initialize system time: {e}")
    last_timezone = None
  
  # Cache last set timezone to avoid frequent settings
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
