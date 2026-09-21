import time
import math
from pipeline.core.vehicle_manager import VehicleManager
from pipeline.visualization.plotter import DronePlotter
from pipeline.math.physics import get_location_meters
from dronekit import LocationGlobalRelative

class HeartTrajectoryMission:
    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        self.manager = VehicleManager(connection_string)
        self.vehicle = self.manager.get_vehicle()

    def run(self):
        self.manager.arm_and_takeoff(15.0)
        self.vehicle.groundspeed = 15.0 # Prędkość zwiększona do 15 m/s
        
        print("Czekam na stabilny sygnał GPS...")
        while True:
            base_gps = self.vehicle.location.global_relative_frame
            if base_gps and base_gps.lat is not None:
                break
            time.sleep(1)

        plotter = DronePlotter(title="Misja - Serce dla licealistow!", trail_length=3000)
        plotter.set_view(elev=90, azim=-90) # Widok z gory
        
        # Generowanie punktów serca (równania parametryczne)
        waypoints = []
        scale = 1.5 # Skala w metrach (szerokość serca to około 32*scale = 48m)
        points_count = 30
        
        for i in range(points_count + 1):
            t = (i / points_count) * 2 * math.pi
            # Obracamy osie X i Y by serce było pionowo z widoku mapy
            x = scale * (16 * math.sin(t)**3)
            y = scale * (13 * math.cos(t) - 5 * math.cos(2*t) - 2 * math.cos(3*t) - math.cos(4*t))
            waypoints.append((x, y))

        print("Lecę po obwodzie serca...")
        
        for wp_x, wp_y in waypoints:
            target_loc = get_location_meters(base_gps, wp_y, wp_x) # Zamiana osi dla odpowiedniej orientacji geograficznej
            target_loc.alt = 15.0
            self.vehicle.simple_goto(target_loc)
            
            # Czekamy aż dron doleci w pobliże punktu (z dużym przybliżeniem, żeby ruch był płynny)
            while True:
                loc = self.vehicle.location.local_frame
                if loc.north is None: 
                    time.sleep(0.1)
                    continue
                
                dx, dy, dz = loc.north, loc.east, -loc.down
                plotter.update(drone_pos=(dx, dy, dz))
                
                # Odległość do aktualnego waypointa
                dist = math.sqrt((dx - wp_y)**2 + (dy - wp_x)**2)
                if dist < 4.0:
                    break
                time.sleep(0.1)
                
        print("Serce narysowane!")
        self.manager.rtl_and_close()
        plotter.close()

if __name__ == "__main__":
    try:
        mission = HeartTrajectoryMission()
        mission.run()
    except Exception as e:
        import traceback
        with open("crash.log", "w") as f:
            f.write(traceback.format_exc())
        print(f"CRASH: {e}")
