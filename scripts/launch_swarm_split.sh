#!/bin/bash
# Skrypt uruchamia 3 instancje SITL do demonstracji roju.

echo "Uruchamianie roju dronów (3 instancje)..."

# Dron 1 (Porty domyślne)
sim_vehicle.py -v ArduCopter -I0 --sysid 1 -f quad -l 51.1078,17.0385,120,0 --out 127.0.0.1:14550 &

# Dron 2 
sim_vehicle.py -v ArduCopter -I1 --sysid 2 -f quad -l 51.1078,17.0385,120,0 --out 127.0.0.1:14560 &

# Dron 3
sim_vehicle.py -v ArduCopter -I2 --sysid 3 -f quad -l 51.1078,17.0385,120,0 --out 127.0.0.1:14570 &

echo "SITL instances started. Wait for MAVProxy to initialize."
echo "Następnie uruchom kod zarządzający modelem Cuckera-Smale'a z pakietu pipeline."
