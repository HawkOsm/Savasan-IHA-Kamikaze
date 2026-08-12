



from scripts.ardupilot.ardupilot import Ardupilot
from scripts.config import CONNECTION_GROUND, SharedData


def main():
    try:
        # Create a shared data object to hold the vehicle and telemetry data
        shared_data = SharedData()
        Ardupilot(connection_string=CONNECTION_GROUND, baudrate=57600, shared_data=shared_data)

        shared_data.run_threads()

    except KeyboardInterrupt:
        print("\nKullanıcı tarafından program sonlandırıldı.")
        shared_data.stop_threads()