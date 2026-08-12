import threading

from scripts.ardupilot.telemetry import Telemetry

CONNECTION_GROUND = "udp://:127.0.0.1:15000"
CONNECTION_PLANE = "udp://:127.0.0.1:25000"
DATA_PORT = 5006

class SharedData:
    def __init__(self):
        self.vehicle = None
        self.telemetry = Telemetry()
        self.lock = threading.Lock()
        self.threads = []

    def add_thread(self, thread):
        self.threads.append(thread)

    def run_threads(self):
        for thread in self.threads:
            thread.start()

    def stop_threads(self):
        for thread in self.threads:
            thread.stop(self)