#!/usr/bin/env bash
# Starts the drone battle arena: 6 ArduCopter SITL instances + one MAVProxy map
# showing all drones (window 1 of the specification).
#
#   attackers: SysID 1, 2, 3 at the north end of the CMAC airfield
#   defenders: SysID 4, 5, 6 at the south end (the base), 150 m away
#
# Positions and ports come from arena_config.toml (one source of truth with the
# orchestrator). Environment variables:
#   ARENA_TERMINAL  terminal command, default "xterm -hold -e";
#                   e.g. "xfce4-terminal --disable-server -x"; "" = no windows (logs in logs/)
#   ARDUPILOT_DIR   default ~/ardupilot
#   PYTHON          default python3
#   ARENA_NO_MAP=1  skip the MAVProxy map
set -euo pipefail

ARENA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$ARENA_DIR/../.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
SIM="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
PYTHON="${PYTHON:-python3}"
PIDFILE="$ARENA_DIR/.arena_pids"
LOG_DIR="$ARENA_DIR/logs"
TERM_CMD="${ARENA_TERMINAL-xterm -hold -e}"

# --- checks --------------------------------------------------------------------
[[ -x "$SIM" ]] || { echo "ERROR: $SIM not found (set ARDUPILOT_DIR)."; exit 1; }
if ! ls "$ARDUPILOT_DIR"/build/sitl/bin/arducopter >/dev/null 2>&1; then
    echo "ERROR: ArduCopter SITL is not built. Run once: cd $ARDUPILOT_DIR && ./waf configure --board sitl && ./waf copter"
    exit 1
fi
if [[ -n "$TERM_CMD" ]] && ! command -v "${TERM_CMD%% *}" >/dev/null 2>&1; then
    echo "WARNING: '${TERM_CMD%% *}' not found, running SITL without windows (logs in $LOG_DIR)."
    TERM_CMD=""
fi

# --- clean up only a previous arena (never the user's own terminals) ----------
"$ARENA_DIR/stop_arena.sh" >/dev/null 2>&1 || true
mkdir -p "$LOG_DIR"
: > "$PIDFILE"

launch() {  # $1 = title, $2 = command
    if [[ -n "$TERM_CMD" ]]; then
        # shellcheck disable=SC2086
        $TERM_CMD bash -c "$2" &
    else
        bash -c "$2" > "$LOG_DIR/$1.log" 2>&1 &
    fi
    echo $! >> "$PIDFILE"
}

# --- read the layout from the configuration ----------------------------------
LAYOUT="$(cd "$REPO_DIR" && "$PYTHON" -m pipeline.drones_battle.core.layout)"
MODE="$(echo "$LAYOUT" | awk '/^mode/ {print $2}')"
MAP_MASTERS=""
echo "Starting 6 SITL instances (mode: $MODE)..."

while read -r instance sysid lat lon alt heading port map_port; do
    if [[ "$MODE" == "tcp" ]]; then
        outputs="--no-mavproxy"
        MAP_MASTERS+=" --master=tcp:127.0.0.1:$map_port"
    else
        outputs="--out=udp:127.0.0.1:$port --out=udp:127.0.0.1:$map_port"
        MAP_MASTERS+=" --master=udp:127.0.0.1:$map_port"
    fi
    team="attacker"; [[ "$sysid" -ge 4 ]] && team="defender"
    echo "  SysID $sysid ($team): $lat, $lon, heading $heading -> port $port"
    launch "sitl_$sysid" "cd $ARDUPILOT_DIR/ArduCopter && $SIM -v ArduCopter --no-rebuild -f quad \
        -I$instance --sysid $sysid -l $lat,$lon,$alt,$heading \
        --add-param-file=$ARENA_DIR/arena.parm $outputs"
    sleep 1
done < <(echo "$LAYOUT" | grep -v '^mode')

# --- window 1: one MAVProxy map with all six drones ---------------------------
if [[ "${ARENA_NO_MAP:-0}" != "1" ]]; then
    echo "Waiting for the simulators before opening the map..."
    sleep 10
    launch "map" "mavproxy.py $MAP_MASTERS --map --console"
fi

echo
echo "Arena is starting. When all drones show 'EKF ... using GPS' run (from $REPO_DIR):"
echo "  $PYTHON -m pipeline.drones_battle.arena_orchestrator --backend sitl"
echo "Stop everything with: $ARENA_DIR/stop_arena.sh"
