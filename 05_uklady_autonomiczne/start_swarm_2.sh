#!/bin/bash
echo "Initializing 8-Drone Swarm (SITL)..."

killall arducopter mavproxy.py xterm 2>/dev/null
KATALOG="cd ~/ardupilot/ArduCopter"

# LIDERZY (Z przodu: Lewy i Prawy)
xterm -hold -e "bash -c 'source ~/.profile; $KATALOG; sim_vehicle.py -v ArduCopter --no-rebuild -f + -I0 --sysid 1 --map -l -35.3627,149.1648,584,0 --out=127.0.0.1:14550'" &
xterm -hold -e "bash -c 'source ~/.profile; $KATALOG; sim_vehicle.py -v ArduCopter --no-rebuild -f + -I1 --sysid 2 -l -35.3627,149.1652,584,0 --out=127.0.0.1:14560'" &

# KLASTER 1 (Skrzydłowi Lidera 1 - przesunięci na południe i zachód)
xterm -hold -e "bash -c 'source ~/.profile; $KATALOG; sim_vehicle.py -v ArduCopter --no-rebuild -f + -I2 --sysid 3 -l -35.3628,149.1647,584,0 --out=127.0.0.1:14570'" &
xterm -hold -e "bash -c 'source ~/.profile; $KATALOG; sim_vehicle.py -v ArduCopter --no-rebuild -f + -I3 --sysid 4 -l -35.3628,149.1648,584,0 --out=127.0.0.1:14580'" &
xterm -hold -e "bash -c 'source ~/.profile; $KATALOG; sim_vehicle.py -v ArduCopter --no-rebuild -f + -I4 --sysid 5 -l -35.3628,149.1649,584,0 --out=127.0.0.1:14590'" &

# KLASTER 2 (Skrzydłowi Lidera 2 - przesunięci na południe i wschód)
xterm -hold -e "bash -c 'source ~/.profile; $KATALOG; sim_vehicle.py -v ArduCopter --no-rebuild -f + -I5 --sysid 6 -l -35.3628,149.1651,584,0 --out=127.0.0.1:14600'" &
xterm -hold -e "bash -c 'source ~/.profile; $KATALOG; sim_vehicle.py -v ArduCopter --no-rebuild -f + -I6 --sysid 7 -l -35.3628,149.1652,584,0 --out=127.0.0.1:14610'" &
xterm -hold -e "bash -c 'source ~/.profile; $KATALOG; sim_vehicle.py -v ArduCopter --no-rebuild -f + -I7 --sysid 8 -l -35.3628,149.1653,584,0 --out=127.0.0.1:14620'" &

echo "SITL instances deployed in the background!"
