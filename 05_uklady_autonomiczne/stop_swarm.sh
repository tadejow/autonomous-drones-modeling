#!/bin/bash
echo "Terminating all simulator instances..."

killall -9 arducopter 2>/dev/null
killall -9 mavproxy.py 2>/dev/null
killall -9 xterm 2>/dev/null
pkill -9 -f sim_vehicle.py 2>/dev/null

echo "Done! Environment cleared."
