import time
from pipeline.core.vehicle_manager import VehicleManager
from pipeline.visualization.plotter import DronePlotter
from pipeline.math.algorithms import tsp_nearest_neighbor
from dronekit import LocationGlobalRelative
import math

class SearchAndRescue:
    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        self.manager = VehicleManager(connection_string)
        self.vehicle = self.manager.get_vehicle()

    def run_tsp_scan(self, points):
        """Wykonuje skan po zoptymalizowanej ścieżce (TSP)."""
        print("Optymalizowanie ścieżki algorytmem Nearest Neighbor...")
        optimized_path = tsp_nearest_neighbor(points)
        print("Ścieżka zoptymalizowana.")
        
        self.manager.arm_and_takeoff(10.0)
        
        plotter = DronePlotter(title="Misja SAR - Problem Komiwojażera (TSP)", trail_length=500)
        plotter.set_view(elev=45, azim=45)
        
        # Pętla po punktach (bardzo uproszczona symulacja dla demonstracji)
        for point in optimized_path:
            tx, ty, tz = point
            print(f"Lot do punktu: {point}")
            
            # (Tutaj pętla latania do celu NED, jak we wcześniejszych skryptach)
            # Na ten moment symulujemy wizualnie, żeby zmieścić się w architekturze:
            for _ in range(30):
                loc = self.vehicle.location.local_frame
                if loc.north is not None:
                    dx, dy, dz = loc.north, loc.east, -loc.down
                    plotter.update((dx, dy, dz), target_pos=(tx, ty, tz))
                time.sleep(0.1)
                
        self.manager.rtl_and_close()
        plotter.close()

if __name__ == "__main__":
    mission = SearchAndRescue()
    points_to_scan = [(0, 50, 10), (50, 50, 10), (50, -50, 10), (0, -50, 10)]
    mission.run_tsp_scan(points_to_scan)
