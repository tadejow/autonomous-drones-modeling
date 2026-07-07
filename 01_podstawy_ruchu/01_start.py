import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass

from dronekit import connect, VehicleMode
import time

# 1. Połączenie z symulatorem. 
# Gdy sim_vehicle działa, tworzy router MAVProxy, który udostępnia drona na porcie UDP 14550.
print("Łączenie z dronem...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
print("Połączono pomyślnie!\n")

# 2. Sprawdzenie, czy dron ma sygnał GPS i jest gotowy do lotu
while not vehicle.is_armable:
    print(" Czekam na inicjalizację drona (szukam GPS)...")
    time.sleep(1)
print("Dron gotowy do lotu!")

# 3. Zmiana trybu na GUIDED
# Tylko w trybie GUIDED dron "słucha" poleceń z Pythona dotyczących ruchu.
print("Zmiana trybu lotu na GUIDED...")
vehicle.mode = VehicleMode("GUIDED")

# 4. Uzbrojenie silników (Arming)
print("Uzbrajanie silników...")
vehicle.armed = True

# Czekamy, aż silniki faktycznie się zakręcą
while not vehicle.armed:
    print(" Czekam na uzbrojenie...")
    time.sleep(1)
print("Silniki uzbrojone! (Uwaga na śmigła)")

# 5. Start (Takeoff)
docelowa_wysokosc = 10.0
print(f"Startujemy na wysokość: {docelowa_wysokosc} metrów")
vehicle.simple_takeoff(docelowa_wysokosc)

# Pętla sprawdzająca aktualną wysokość
while True:
    aktualna_wysokosc = vehicle.location.global_relative_frame.alt
    print(f" Aktualna wysokość: {aktualna_wysokosc:.1f} m")
    
    # Jeśli osiągniemy 95% wysokości docelowej, przerywamy pętlę
    if aktualna_wysokosc >= docelowa_wysokosc * 0.95:
        print("Osiągnięto wysokość przelotową!")
        break
    time.sleep(1)

# 6. Zawis i powrót na ziemię
print("Wiszę w powietrzu przez 5 sekund...")
time.sleep(5)

print("Wracam na miejsce startu (Tryb RTL)...")
vehicle.mode = VehicleMode("RTL")  # RTL = Return To Launch

# Zamykamy połączenie ze statkiem
vehicle.close()
print("Koniec misji.")
