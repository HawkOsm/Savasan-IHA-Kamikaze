import math
from enum import IntFlag

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import time

def connect_to_mavlink(connection_string, baudrate=57600):
    while True:
        try:
            conn = mavutil.mavlink_connection(connection_string, baud=baudrate)
            conn.wait_heartbeat(timeout=5)
            print("MAVLink connection established.")
            return conn
        except Exception as e:
            print(f"Failed to connect to MAVLink ({e}). Retrying in 3 seconds...")
            time.sleep(3)

def mode_change(vehicle, mode, fast = False):
    """Change the flight mode of the vehicle."""
    if vehicle.mode.name != mode:
        print(f"Changing mode to {mode}...")
        vehicle.mode = vehicle.mode_mapping()[mode]
        if fast:
            pass
        while vehicle.mode.name != mode:
            time.sleep(0.5)
        print(f"Mode changed to {mode}.")
        
class AttitudeTypeMask(IntFlag):
    """type_mask bits for SET_ATTITUDE_TARGET (MAVLink common message 82).

    https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET

    Deliberately excludes ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET (32) —
    that bit means "use a 3D body thrust vector instead of throttle", which
    only makes sense for copters/VTOL. Stable-wing planes always use the
    scalar `thrust` field as throttle, so that bit is never set here.
    """
    NONE = 0
    IGNORE_ROLL_RATE = 1
    IGNORE_PITCH_RATE = 2
    IGNORE_YAW_RATE = 4
    IGNORE_THROTTLE = 64
    IGNORE_ATTITUDE = 128


class SET_ATTITUDE_TARGET:
    """Builds and sends SET_ATTITUDE_TARGET for a stable-wing (fixed-wing) vehicle."""

    def __init__(self, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, thrust=0.5,
                 type_mask=AttitudeTypeMask.IGNORE_ROLL_RATE
                 | AttitudeTypeMask.IGNORE_PITCH_RATE
                 | AttitudeTypeMask.IGNORE_YAW_RATE):
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.yaw_deg = yaw_deg
        self.thrust = thrust
        self.type_mask = type_mask

    def calculate_quaternion(self):
        return QuaternionBase([
            math.radians(self.roll_deg),
            math.radians(self.pitch_deg),
            math.radians(self.yaw_deg),
        ]).q

    def send(self,vehicle):
        vehicle.mav.set_attitude_target_send(
            0,  # time_boot_ms — 0 lets the receiving autopilot use its own clock
            vehicle.target_system,
            vehicle.target_component,
            int(self.type_mask),
            self.calculate_quaternion(),
            0,  # body_roll_rate — ignored per type_mask
            0,  # body_pitch_rate — ignored per type_mask
            0,  # body_yaw_rate — ignored per type_mask
            self.thrust,
        )


class MISSION_WRITE_PARTIAL:
    """Overwrites a contiguous seq range [start, end] of the mission already on the
    autopilot, leaving every other mission item untouched (MAVLink mission protocol,
    MISSION_WRITE_PARTIAL_LIST — https://mavlink.io/en/services/mission.html).

    `items` must be dicts with keys: seq, frame, cmd, p1, p2, p3, p4, x, y, z.
    Their `seq` values must cover [start, end] exactly, with no gaps or extras.

    Not thread-safe: send() calls vehicle.recv_match() directly on the shared
    connection. If another thread (e.g. a telemetry loop) is also reading from
    the same `vehicle`, the two will race for incoming messages.
    """

    def __init__(self, items, start, end,
                 mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                 timeout=3.0):
        by_seq = {item["seq"]: item for item in items}
        if sorted(by_seq) != list(range(start, end + 1)):
            raise ValueError(f"item seqs {sorted(by_seq)} != range {start}..{end}")
        self.items = by_seq
        self.start = start
        self.end = end
        self.mission_type = mission_type
        self.timeout = timeout

    def send(self, vehicle):
        """Runs the partial-write handshake. Returns True on MAV_MISSION_ACCEPTED,
        False on timeout, rejection, or another ack type."""
        vehicle.mav.mission_write_partial_list_send(
            vehicle.target_system, vehicle.target_component,
            self.start, self.end, self.mission_type)

        last_progress = time.monotonic()
        while time.monotonic() - last_progress < self.timeout:
            msg = vehicle.recv_match(
                type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                blocking=True, timeout=1.0)
            if msg is None:
                continue
            last_progress = time.monotonic()

            if msg.get_type() == "MISSION_ACK":
                return msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED

            item = self.items.get(msg.seq)
            if item is None:
                continue
            vehicle.mav.mission_item_int_send(
                vehicle.target_system, vehicle.target_component,
                item["seq"], item["frame"], item["cmd"], 0, 1,
                item["p1"], item["p2"], item["p3"], item["p4"],
                int(item["x"]), int(item["y"]), float(item["z"]),
                self.mission_type)

        return False



