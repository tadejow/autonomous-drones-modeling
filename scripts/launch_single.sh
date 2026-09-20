#!/bin/bash
# Skrypt uruchamia pojedynczą instancję SITL i czeka na połączenie

echo "Uruchamianie symulatora SITL (ArduCopter)..."
sim_vehicle.py -v ArduCopter -f quad -L Wroclaw --console --map &
SIM_PID=$!

sleep 10
echo "Symulator gotowy. Aby uruchomić skrypty Pythona (np. physics_demo):"
echo "python3 -m pipeline.missions.physics_demo"

wait $SIM_PID
