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
pkill -f "mavproxy.py.*127.0.0.1:14(55|56|57|58|59|60)0" 2>/dev/null
pkill -f "mavproxy.py.*--master=(udp:127.0.0.1:14(65|66|67|68|69|70)0|tcp:127.0.0.1:5(76|77|78|79|80|81)2)" 2>/dev/null
sleep 1
pkill -9 -f "arducopter.*$ARENA_DIR/arena.parm" 2>/dev/null
echo "Done."
