#!/bin/bash
echo "Initializing 8-Drone Swarm (SITL)..."

killall arducopter mavproxy.py xterm xfce4-terminal 2>/dev/null
KATALOG="cd ~/ardupilot/ArduCopter"

# LIDERZY (Z przodu: Lewy i Prawy)
xfce4-terminal --disable-server -x bash -c "source ~/.profile; $KATALOG; $HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild -f + -I0 --sysid 1 --map -l -35.3627,149.1648,584,0 --out=127.0.0.1:14550; echo 'Zakończono'; read" &
xfce4-terminal --disable-server -x bash -c "source ~/.profile; $KATALOG; $HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild -f + -I1 --sysid 2 -l -35.3627,149.1652,584,0 --out=127.0.0.1:14560; echo 'Zakończono'; read" &

# KLASTER 1 (Skrzydłowi Lidera 1 - przesunięci na południe i zachód)
xfce4-terminal --disable-server -x bash -c "source ~/.profile; $KATALOG; $HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild -f + -I2 --sysid 3 -l -35.3628,149.1647,584,0 --out=127.0.0.1:14570; echo 'Zakończono'; read" &
xfce4-terminal --disable-server -x bash -c "source ~/.profile; $KATALOG; $HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild -f + -I3 --sysid 4 -l -35.3628,149.1648,584,0 --out=127.0.0.1:14580; echo 'Zakończono'; read" &
xfce4-terminal --disable-server -x bash -c "source ~/.profile; $KATALOG; $HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild -f + -I4 --sysid 5 -l -35.3628,149.1649,584,0 --out=127.0.0.1:14590; echo 'Zakończono'; read" &

# KLASTER 2 (Skrzydłowi Lidera 2 - przesunięci na południe i wschód)
xfce4-terminal --disable-server -x bash -c "source ~/.profile; $KATALOG; $HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild -f + -I5 --sysid 6 -l -35.3628,149.1651,584,0 --out=127.0.0.1:14600; echo 'Zakończono'; read" &
xfce4-terminal --disable-server -x bash -c "source ~/.profile; $KATALOG; $HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild -f + -I6 --sysid 7 -l -35.3628,149.1652,584,0 --out=127.0.0.1:14610; echo 'Zakończono'; read" &
xfce4-terminal --disable-server -x bash -c "source ~/.profile; $KATALOG; $HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild -f + -I7 --sysid 8 -l -35.3628,149.1653,584,0 --out=127.0.0.1:14620; echo 'Zakończono'; read" &

echo "SITL instances deployed in the background!"
