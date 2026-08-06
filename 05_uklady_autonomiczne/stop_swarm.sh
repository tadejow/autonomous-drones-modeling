#!/bin/bash
echo "Zamykanie wszystkich dronów i okienek symulatora..."

# Brutalne i błyskawiczne wyczyszczenie wszystkich procesów w tle
killall -9 arducopter 2>/dev/null
killall -9 mavproxy.py 2>/dev/null
killall -9 xterm 2>/dev/null
pkill -9 -f sim_vehicle.py 2>/dev/null

echo "Gotowe! Wszystkie okienka i drony zostały pomyślnie zamknięte."
