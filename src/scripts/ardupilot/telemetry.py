import math
import time
from pymavlink import mavutil

class Telemetry:
    def __init__(self):
        # ── Eski AvciState alanları (kaynak satır 390–398, AYNEN) ────────────
        self.lat                       = 0.0    # GLOBAL_POSITION_INT doldurur
        self.lon                       = 0.0    # GLOBAL_POSITION_INT doldurur
        self.alt                       = 0.0    # GLOBAL_POSITION_INT doldurur
        self.roll_deg                  = 0.0    # ATTITUDE doldurur
        self.pitch_deg                 = 0.0    # ATTITUDE doldurur
        self.yaw_deg                   = 0.0    # ATTITUDE doldurur
        self.speed                     = 0.0    # GPS_RAW_INT.vel doldur
        self.batt                      = 0.0    # BATTERY_STATUS doldur
        self.gps_time                  = None   # GPS_RAW_INT doldurur

    def update(self, msg):
        """MAVLink mesajını alır ve dahili alanları günceller."""
        if msg.get_type() == "GLOBAL_POSITION_INT":
            self.lat = msg.lat / 1e7
            self.lon = msg.lon / 1e7
            self.alt = msg.relative_alt / 1000.0
        elif msg.get_type() == "ATTITUDE":
            self.roll_deg = math.degrees(msg.roll)
            self.pitch_deg = math.degrees(msg.pitch)
            self.yaw_deg = (math.degrees(msg.yaw) % 360.0 + 360.0) % 360.0
        elif msg.get_type() == "GPS_RAW_INT":
            if getattr(msg, "vel", None) is not None and math.isfinite(msg.vel):
                self.speed = msg.vel / 100.0
            self.gps_time = time.time()
        elif msg.get_type() == "BATTERY_STATUS":
            if getattr(msg, "battery_remaining", None) is not None:
                self.batt = msg.battery_remaining

    def to_dict(self):
        """Dahili alanları bir sözlüğe dönüştürür."""
        return {
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "roll_deg": self.roll_deg,
            "pitch_deg": self.pitch_deg,
            "yaw_deg": self.yaw_deg,
            "speed": self.speed,
            "batt": self.batt,
            "gps_time": self.gps_time
        }