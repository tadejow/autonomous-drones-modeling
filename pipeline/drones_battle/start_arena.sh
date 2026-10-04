#!/usr/bin/env bash
# Starts the drone battle arena: 6 ArduCopter SITL instances + one MAVProxy map
# showing all drones (window 1 of the specification).
#
#   attackers: SysID 1, 2, 3 at the north end of the CMAC airfield
#   defenders: SysID 4, 5, 6 at the south end (the base), 150 m away
#
# Positions and ports come from arena_config.toml (one source of truth with the
# orchestrator).
#
# Window layout: the simulator terminals start minimized; the MAVProxy map is
# placed on the left half of the screen (needs wmctrl), and the orchestrator puts
# its 3D window on the right half, so the two windows fill the screen.
#
# Environment variables:
#   ARENA_TERMINAL  auto (default: xfce4-terminal, else xterm), xfce4-terminal, xterm,
#                   or none / "" (no terminal windows, logs in logs/)
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
TERMINAL="${ARENA_TERMINAL-auto}"
[[ -z "$TERMINAL" ]] && TERMINAL=none   # ARENA_TERMINAL="" also means no windows

# --- checks --------------------------------------------------------------------
[[ -x "$SIM" ]] || { echo "ERROR: $SIM not found (set ARDUPILOT_DIR)."; exit 1; }
if ! ls "$ARDUPILOT_DIR"/build/sitl/bin/arducopter >/dev/null 2>&1; then
    echo "ERROR: ArduCopter SITL is not built. Run once: cd $ARDUPILOT_DIR && ./waf configure --board sitl && ./waf copter"
    exit 1
fi
if [[ "$TERMINAL" == "auto" ]]; then
    TERMINAL=none
    for candidate in xfce4-terminal xterm; do
        command -v "$candidate" >/dev/null 2>&1 && { TERMINAL="$candidate"; break; }
    done
fi
if [[ "$TERMINAL" != "none" ]] && ! command -v "$TERMINAL" >/dev/null 2>&1; then
    echo "WARNING: '$TERMINAL' not found, running SITL without windows (logs in $LOG_DIR)."
    TERMINAL=none
fi
if ! command -v wmctrl >/dev/null 2>&1; then
    echo "TIP: install wmctrl (sudo apt install wmctrl) to place the map on the left half automatically."
fi

# --- clean up only a previous arena (never the user's own terminals) ----------
"$ARENA_DIR/stop_arena.sh" >/dev/null 2>&1 || true
mkdir -p "$LOG_DIR"
: > "$PIDFILE"

launch() {  # $1 = name, $2 = command; terminal windows start minimized
    local title="arena-$1"
    case "$TERMINAL" in
        xfce4-terminal)
            xfce4-terminal --disable-server --minimize --hold -T "$title" -x bash -c "$2" & ;;
        xterm)
            xterm -iconic -hold -T "$title" -e bash -c "$2" & ;;
        *)
            bash -c "$2" > "$LOG_DIR/$1.log" 2>&1 & ;;
    esac
    echo $! >> "$PIDFILE"
}

minimize_arena_terminals() {  # fallback for window managers that ignore --minimize / -iconic
    command -v xdotool >/dev/null 2>&1 || return 0
    local window
    for window in $(xdotool search --name '^arena-' 2>/dev/null); do
        xdotool windowminimize "$window" 2>/dev/null || true
    done
}

work_area() {  # prints "x y width height" of the usable screen area (without panels)
    local area
    area="$(wmctrl -d 2>/dev/null | awk '$2 == "*" { for (i = 1; i <= NF; i++) if ($i == "WA:") {
        split($(i + 1), p, ","); split($(i + 2), s, "x"); print p[1], p[2], s[1], s[2] } }' || true)"
    if [[ -z "$area" ]] && command -v xdpyinfo >/dev/null 2>&1; then
        area="0 0 $(xdpyinfo | awk '/dimensions/ { split($2, s, "x"); print s[1], s[2] - 40 }' || true)"
    fi
    echo "${area:-0 0 1920 1040}"
}

place_map_window() {  # waits for the MAVProxy map and puts it on the left half of the screen
    command -v wmctrl >/dev/null 2>&1 || return 0
    local title="" x y width height
    for _ in $(seq 1 120); do
        title="$(wmctrl -l | awk '{ $1 = $2 = $3 = ""; sub(/^ +/, ""); print }' | grep -m1 -E '(^|[^-])Map$' || true)"
        [[ -n "$title" ]] && break
        sleep 1
    done
    [[ -z "$title" ]] && { echo "WARNING: MAVProxy map window not found, leaving it where it is."; return 0; }
    read -r x y width height < <(work_area)
    wmctrl -F -r "$title" -b remove,maximized_vert,maximized_horz
    wmctrl -F -r "$title" -e "0,$x,$y,$((width / 2)),$((height - 32))"
    wmctrl -F -a "$title"
    minimize_arena_terminals
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
    launch "map" "mavproxy.py $MAP_MASTERS --map"
    place_map_window &
fi
minimize_arena_terminals

echo
echo "Arena is starting (simulator terminals are minimized in the taskbar)."
echo "When all drones are visible on the map, run (from $REPO_DIR):"
echo "  $PYTHON -m pipeline.drones_battle.arena_orchestrator --backend sitl"
echo "Stop everything with: $ARENA_DIR/stop_arena.sh"
