import time
import math
from pipeline.core.vehicle_manager import VehicleManager
from pipeline.visualization.plotter import DronePlotter
from pipeline.math.physics import get_location_meters
from pymavlink import mavutil

class PursuitMission:
    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        self.manager = VehicleManager(connection_string)
        self.vehicle = self.manager.get_vehicle()
        
    def spoof_adsb_target(self, target_gps):
        """Wysyła fałszywy sygnał ADS-B z pozycją celu na mapę (UFO-1/Balloon)."""
        flags = 1 | 2 | 4 | 8 | 16 | 32
        msg = self.vehicle.message_factory.adsb_vehicle_encode(
            ICAO_address=12345, 
            lat=int(target_gps.lat * 1e7), 
            lon=int(target_gps.lon * 1e7), 
            altitude_type=0, 
            altitude=int(target_gps.alt * 1000), 
            heading=0, 
            hor_velocity=0, 
            ver_velocity=0, 
            callsign=b"BALON", 
            emitter_type=10, 
            tslc=1, 
            flags=flags,
            squawk=1200
        )
        self.vehicle.send_mavlink(msg)

    def send_ned_velocity(self, vx, vy, vz):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111, 
            0, 0, 0, 
            vx, vy, vz, 
            0, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)

    def run(self):
        self.vehicle.groundspeed = 15.0
        self.manager.arm_and_takeoff(10.0)
        
        base_gps = self.vehicle.location.global_relative_frame
        
        # Inicjalizacja celu - balon
        tx, ty, tz = 150.0, 100.0, 30.0
        tvx, tvy, tvz = 0.25, 1.0, -0.2 
        
        plotter = DronePlotter(title="Misja Pościgu - Balon", trail_length=150)
        plotter.set_view(elev=30, azim=45)
        
        last_time = time.time()
        
        print("Rozpoczynanie misji pościgu...")
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            if dt > 1.0 or dt == 0: dt = 0.1
            
            loc = self.vehicle.location.local_frame
            if loc.north is None: 
                continue
            dx, dy, dz = loc.north, loc.east, -loc.down
            
            # Aktualizacja logiki uciekającego balonu
            tx += tvx * dt
            ty += tvy * dt
            tz += tvz * dt
            
            target_gps = get_location_meters(base_gps, tx, ty)
            target_gps.alt = tz
            self.spoof_adsb_target(target_gps)
            
            # Obliczanie dystansu i kierunku (Lead Pursuit)
            dist_x, dist_y, dist_z = tx - dx, ty - dy, tz - dz
            real_distance = math.sqrt(dist_x**2 + dist_y**2 + dist_z**2)
            
            if real_distance < 4.0:
                print("CEL ZŁAPANY!")
                self.send_ned_velocity(0.0, 0.0, 0.0)
                break
                
            # Prosty wektor do celu
            speed = 15.0
            cmd_vx = (dist_x / real_distance) * speed
            cmd_vy = (dist_y / real_distance) * speed
            cmd_vz = -(dist_z / real_distance) * speed 
            self.send_ned_velocity(cmd_vx, cmd_vy, cmd_vz)
            
            # Plotter
            plotter.update(drone_pos=(dx, dy, dz), target_pos=(tx, ty, tz), balloon_icon=True)
            time.sleep(0.05)
            
        self.manager.rtl_and_close()
        plotter.close()

if __name__ == "__main__":
    mission = PursuitMission()
    mission.run()
