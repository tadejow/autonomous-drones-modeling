#!/bin/bash
echo "Initializing Drone Swarm with 1 leader-drone (SITL)..."

# Zabijamy stare procesy
killall arducopter mavproxy.py xterm 2>/dev/null

KATALOG="cd ~/ardupilot/ArduCopter"

# Dron 0 (Lider) - ZAMIENIONO --console NA --map !
xterm -hold -e "bash -c 'source ~/.profile; $KATALOG; sim_vehicle.py -v ArduCopter --no-rebuild -f + -I0 --sysid 1 --map -l -35.3627,149.1650,584,0 --out=127.0.0.1:14550'" &

# Dron 1 (Sąsiad A)
xterm -hold -e "bash -c 'source ~/.profile; $KATALOG; sim_vehicle.py -v ArduCopter --no-rebuild -f + -I1 --sysid 2 -l -35.3627,149.1651,584,0 --out=127.0.0.1:14560'" &

# Dron 2 (Sąsiad B)
xterm -hold -e "bash -c 'source ~/.profile; $KATALOG; sim_vehicle.py -v ArduCopter --no-rebuild -f + -I2 --sysid 3 -l -35.3627,149.1649,584,0 --out=127.0.0.1:14570'" &

echo "SITL instances deployed in the background!"
