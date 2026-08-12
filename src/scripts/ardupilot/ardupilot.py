"""
Background MAVLink telemetry worker.

Runs the blocking pymavlink receive loop on its own thread and writes
into the shared Telemetry object, so the GUI thread (map.py, hud.py) can
read the latest values without needing a callback wired to every panel.
"""

import threading
import time

from ardupilot import commands
from scripts.ardupilot import telemetry


class Ardupilot:
    def __init__(self, connection_string, baudrate=57600):
        self.connection_string = connection_string
        self.baudrate = baudrate
        self._running = False

    def start(self, shared_data):
        """Spin run() up on a background thread and hand it to shared_data."""
        self._running = True
        thread = threading.Thread(target=self.run, args=(shared_data), daemon=True, sleep=0.25)
        shared_data.add_thread(thread)
        thread.start()
        return thread

    def run(self, shared_data):
        shared_data.vehicle = commands.connect_to_mavlink(self.connection_string, self.baudrate)
        while self._running and shared_data.vehicle is not None:
            try:
                msg = shared_data.vehicle.recv_match(blocking=True, timeout=1)
                if msg is not None:
                    with shared_data.lock:
                        shared_data.telemetry.update(msg)

            except Exception as e:
                print(f"Error receiving MAVLink message: {e}")
                with shared_data.lock:
                    shared_data.connection_status = "lost connection"
                break

    def stop(self):
        self._running = False
