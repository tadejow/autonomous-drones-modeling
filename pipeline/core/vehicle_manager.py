import time
from dronekit import connect, VehicleMode

class VehicleManager:
    """
    Klasa zarządzająca podstawowymi operacjami drona (połączenie, uzbrajanie, start).
    """

    def __init__(self, connection_string='udp:127.0.0.1:14550', baud=115200):
        print(f"Connecting to vehicle on: {connection_string}")
        self.vehicle = connect(connection_string, wait_ready=True, baud=baud)
        print("Connected.")

    def arm_and_takeoff(self, target_altitude):
        """
        Zbroi drona i startuje na docelową wysokość.
        """
        # Maksymalizacja prędkości nawigacyjnej
        self.vehicle.parameters['WPNAV_SPEED'] = 2000.0   # 20 m/s poziomo
        self.vehicle.parameters['WPNAV_SPEED_UP'] = 500.0 # 5 m/s do góry
        self.vehicle.parameters['WPNAV_SPEED_DN'] = 300.0  # 3 m/s w dół
        print("Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print(f"Taking off to {target_altitude}m!")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            print(f" Altitude: {altitude:.1f}m")
            if altitude >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def rtl_and_close(self):
        """
        Wywołuje powrót do miejsca startu i zamyka połączenie.
        """
        print("Returning to Launch (RTL)...")
        self.vehicle.mode = VehicleMode("RTL")
        self.vehicle.close()
        print("Vehicle connection closed.")

    def get_vehicle(self):
        return self.vehicle
