#!/usr/bin/env bash
# Stops only the processes started by start_arena.sh (PIDs from .arena_pids plus
# SITL/MAVProxy processes that use this arena's parameter file or ports).
# Your own terminals are never touched.
ARENA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIDFILE="$ARENA_DIR/.arena_pids"

echo "Stopping the drone battle arena..."
if [[ -f "$PIDFILE" ]]; then
    while read -r pid; do
        [[ -n "$pid" ]] && kill "$pid" 2>/dev/null
    done < "$PIDFILE"
    rm -f "$PIDFILE"
fi

pkill -f "sim_vehicle.py.*$ARENA_DIR/arena.parm" 2>/dev/null
pkill -f "arducopter.*$ARENA_DIR/arena.parm" 2>/dev/null
# Up to 10 drones: orchestrator ports 14550..14640, map ports 14650..14740 or tcp 5762..5852.
pkill -f "mavproxy.py.*127.0.0.1:14(5[5-9]|6[0-4])0" 2>/dev/null
pkill -f "mavproxy.py.*--master=(udp:127.0.0.1:14(6[5-9]|7[0-4])0|tcp:127.0.0.1:5(7[6-9]|8[0-5])2)" 2>/dev/null
sleep 1
pkill -9 -f "arducopter.*$ARENA_DIR/arena.parm" 2>/dev/null
echo "Done."
