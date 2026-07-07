# --- ŁATKA DLA NOWYCH WERSJI PYTHONA ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

from dronekit import connect, VehicleMode
import time
import math

print("Łączenie z dronem...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# -------------------------------------------------------------
# ZAAWANSOWANE: Nasłuchiwanie surowych komunikatów z silnika fizyki
# -------------------------------------------------------------
# DroneKit nie ma wbudowanej zmiennej "vehicle.wind". 
# Ale możemy podpiąć się bezpośrednio pod protokół MAVLink i czytać wiadomości 'WIND'
@vehicle.on_message('WIND')
def wind_listener(self, name, message):
    # message.direction - kierunek z którego wieje (stopnie)
    # message.speed - prędkość wiatru (m/s)
    # message.speed_z - prędkość wiatru pionowego (m/s)
    global wiatr_predkosc, wiatr_kierunek
    wiatr_predkosc = message.speed
    wiatr_kierunek = message.direction

wiatr_predkosc = 0.0
wiatr_kierunek = 0.0

# -------------------------------------------------------------

print("\n--- TEST FIZYKI I ODPORNOŚCI NA WIATR ---")
print("Uzbrajam i startuję na 15 metrów...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    time.sleep(1)

vehicle.simple_takeoff(15)

# Czekamy aż doleci na 15 metrów
while vehicle.location.global_relative_frame.alt < 14.5:
    time.sleep(1)

print("\nDron wisi w powietrzu. Włącz wiatr w konsoli MAVProxy (param set SIM_WIND_SPD 10)")
print("Obserwuj, jak dron przechyla się (Roll/Pitch), aby utrzymać pozycję!\n")

# Pętla telemetrii - potrwa 30 sekund
for i in range(30):
    # 1. Odczyt Kątów Eulera (Orientacja drona w przestrzeni)
    # Kąty są w radianach, przeliczamy na stopnie dla czytelności
    roll = math.degrees(vehicle.attitude.roll)
    pitch = math.degrees(vehicle.attitude.pitch)
    yaw = math.degrees(vehicle.attitude.yaw)
    
    # 2. Odczyt prędkości fizycznej w układzie NED (m/s)
    vx, vy, vz = vehicle.velocity
    predkosc_calkowita = math.sqrt(vx**2 + vy**2 + vz**2)
    
    # Drukujemy piękny raport fizyczny
    print(f"[Sekunda {i+1}/30] FIZYKA DRONA:")
    print(f" -> Orientacja : Pitch(Pochylenie)={pitch:5.1f}°, Roll(Przechył)={roll:5.1f}°, Yaw(Kierunek)={yaw:5.1f}°")
    print(f" -> Prędkość   : {predkosc_calkowita:.2f} m/s (Ciąg wektorowy: X={vx:.1f}, Y={vy:.1f}, Z={vz:.1f})")
    print(f" -> Szac. Wiatr: {wiatr_predkosc:.1f} m/s z kierunku {wiatr_kierunek:.0f}°")
    print("-" * 60)
    
    time.sleep(1)

print("\nKoniec eksperymentu. Lądowanie...")
vehicle.mode = VehicleMode("RTL")
vehicle.close()
