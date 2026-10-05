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
#   PYTHON          default python3 (only reads the arena configuration)
#   ARDUPILOT_VENV  Python environment for sim_vehicle.py and MAVProxy, default ~/venv-ardupilot
#                   when it exists (the one from ArduPilot's install script, it has wxPython
#                   for the map); otherwise the current python3 is used
#   ARENA_NO_MAP=1  skip the MAVProxy map
#   ARENA_MAP_DELAY seconds to wait before opening the map (default 15), so that the
#                   simulators are up; mavproxy_arena.py then gives the map window up to 60 s
#                   to start (MAVProxy alone allows 5 s and leaves a blank window otherwise)
#   ARENA_CONFIG    configuration file (default arena_config.toml), e.g. arena_config_5v5.toml;
#                   pass the same file to the orchestrator with --config
set -euo pipefail

ARENA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$ARENA_DIR/../.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
SIM="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
PYTHON="${PYTHON:-python3}"
ARDUPILOT_VENV="${ARDUPILOT_VENV:-$HOME/venv-ardupilot}"
if [[ -f "$ARDUPILOT_VENV/bin/activate" ]]; then
    SIM_PYTHON="$ARDUPILOT_VENV/bin/python3"
    ACTIVATE="source $ARDUPILOT_VENV/bin/activate && "
    MAVPROXY="$ARDUPILOT_VENV/bin/mavproxy.py"
else
    SIM_PYTHON="$(command -v python3)"
    ACTIVATE=""
    MAVPROXY="$(command -v mavproxy.py || echo mavproxy.py)"
fi
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
echo "sim_vehicle.py and MAVProxy use: $SIM_PYTHON"
missing="$("$SIM_PYTHON" -c 'import importlib.util as u; print(" ".join(m for m in ("pexpect", "MAVProxy") if u.find_spec(m) is None))' 2>/dev/null || echo "?")"
if [[ -n "$missing" ]] || ! [[ -x "$MAVPROXY" ]]; then
    echo "ERROR: that Python is missing: ${missing:-mavproxy.py}. Install it there:"
    echo "         $SIM_PYTHON -m pip install pexpect MAVProxy"
    exit 1
fi
if [[ "${ARENA_NO_MAP:-0}" != "1" ]] && ! "$SIM_PYTHON" -c "import wx" >/dev/null 2>&1; then
    echo "WARNING: no wxPython in $SIM_PYTHON, so the MAVProxy map cannot open. Either"
    echo "           sudo apt install python3-wxgtk4.0      (system Python), or"
    echo "           $SIM_PYTHON -m pip install wxPython   (long build), or"
    echo "         use the top-down map of the 3D window: arena_orchestrator ... --topdown"
    ARENA_NO_MAP=1
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
    # arena-* = our terminals, ArduCopter = the xterm that sim_vehicle.py opens for each simulator
    for window in $(xdotool search --name '^(arena-|ArduCopter)' 2>/dev/null); do
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
CONFIG_ARGS=()
[[ -n "${ARENA_CONFIG:-}" ]] && CONFIG_ARGS=(--config "$(cd "$(dirname "$ARENA_CONFIG")" && pwd)/$(basename "$ARENA_CONFIG")")
LAYOUT="$(cd "$REPO_DIR" && "$PYTHON" -m pipeline.drones_battle.core.layout "${CONFIG_ARGS[@]}")"
MODE="$(echo "$LAYOUT" | awk '/^mode/ {print $2}')"
MAP_MASTERS=""
COUNT="$(echo "$LAYOUT" | grep -vc '^mode')"
echo "Starting $COUNT SITL instances (mode: $MODE)..."

while read -r instance sysid lat lon alt heading port map_port team; do
    if [[ "$MODE" == "tcp" ]]; then
        outputs="--no-mavproxy"
        MAP_MASTERS+=" --master=tcp:127.0.0.1:$map_port"
    else
        outputs="--out=udp:127.0.0.1:$port --out=udp:127.0.0.1:$map_port"
        MAP_MASTERS+=" --master=udp:127.0.0.1:$map_port"
    fi
    echo "  SysID $sysid ($team): $lat, $lon, heading $heading -> port $port"
    launch "sitl_$sysid" "${ACTIVATE}cd $ARDUPILOT_DIR/ArduCopter && $SIM -v ArduCopter --no-rebuild -f quad \
        -I$instance --sysid $sysid -l $lat,$lon,$alt,$heading \
        --add-param-file=$ARENA_DIR/arena.parm $outputs"
    sleep 1
done < <(echo "$LAYOUT" | grep -v '^mode')

# --- window 1: one MAVProxy map with all six drones ---------------------------
if [[ "${ARENA_NO_MAP:-0}" != "1" ]]; then
    echo "Waiting for the simulators before opening the map..."
    sleep "${ARENA_MAP_DELAY:-15}"
    # mavproxy_arena.py (a MAVProxy module from this repository) draws attackers red and
    # defenders blue and fits the view to the drones (it hides the standard icons itself,
    # so if it fails to load the map still shows the drones).
    map_teams="$(echo "$LAYOUT" | grep -v '^mode' |
        awk '{ printf "%s%s:%s", (NR > 1 ? "," : ""), $2, ($9 == "attackers" ? "red" : "blue") }')"
    # The module opens the map itself (with a longer start-up limit than MAVProxy's 5 s).
    # No extra "module load map": MAVProxy would open a second map window ("Map2").
    map_cmds="module load pipeline.drones_battle.mavproxy_arena"
    # Shape of the window that place_map_window gives the map (left half of the work area).
    map_aspect="$(work_area | awk '{ printf "%.3f\n", ($3 / 2) / ($4 - 32) }')"
    launch "map" "${ACTIVATE}export PYTHONPATH=$REPO_DIR ARENA_MAP_TEAMS=$map_teams ARENA_MAP_ASPECT=$map_aspect && \
        $MAVPROXY $MAP_MASTERS --cmd='$map_cmds'"
    place_map_window &
fi
minimize_arena_terminals

echo
echo "Arena is starting (simulator terminals are minimized in the taskbar)."
echo "When all drones are visible on the map, run (from $REPO_DIR):"
echo "  $PYTHON -m pipeline.drones_battle.arena_orchestrator --backend sitl ${ARENA_CONFIG:+--config $ARENA_CONFIG}$([[ "${ARENA_NO_MAP:-0}" == "1" ]] && echo " --topdown")"
echo "Stop everything with: $ARENA_DIR/stop_arena.sh"
