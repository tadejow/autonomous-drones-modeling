#!/bin/bash
# Master script do automatycznego nagrywania eksperymentów (Widoczny pulpit)
# Uruchomi wszystko na Twoim własnym ekranie!

# Przejście do głównego katalogu repozytorium (by PYTHONPATH działał dla misji)
cd "$(dirname "$0")/.."
export PYTHONPATH=$(pwd)
PROJECT_DIR=$(pwd)

echo "Sprawdzanie zależności..."
for cmd in wmctrl ffmpeg sim_vehicle.py python3 xdpyinfo; do
    if ! command -v $cmd &> /dev/null; then
        echo "BŁĄD: Nie znaleziono polecenia '$cmd'."
        echo "Zainstaluj je (np. sudo apt install wmctrl ffmpeg x11-utils) lub dodaj do PATH."
        exit 1
    fi
done

mkdir -p nagrania

# Pobranie faktycznej rozdzielczości Twojego pulpitu do nagrywania (np. 1920x1080)
RESOLUTION=$(xdpyinfo | awk '/dimensions/{print $2}')
echo "Wykryto rozdzielczość ekranu: $RESOLUTION"

# Obliczenie szerokości połowy ekranu dla wmctrl
HALF_WIDTH=$(echo $RESOLUTION | cut -d'x' -f1)
HALF_WIDTH=$((HALF_WIDTH / 2))
HEIGHT=$(echo $RESOLUTION | cut -d'x' -f2)

EXPERIMENTS=(
    "pipeline.missions.physics_demo"
    "pipeline.missions.pursuit"
    "pipeline.missions.search_rescue"
)

for EXP in "${EXPERIMENTS[@]}"; do
    echo "================================================="
    echo "Rozpoczynam nagrywanie eksperymentu: $EXP"
    echo "================================================="
    
    FILENAME=${EXP//./_}

    # 1. Uruchomienie z naturalnego katalogu ArduPilota (bez pty i Xvfb!)
    # Wstrzykujemy aktywację środowiska i ścieżki wewnątrz nowego okna!
    cd $HOME/ardupilot/ArduCopter
    xfce4-terminal -e "bash -c 'source $HOME/venv-ardupilot/bin/activate; export PATH=\$PATH:\$HOME/ardupilot/Tools/autotest; sim_vehicle.py -f quad -l 51.1078,17.0385,120,0 --map; read'" &
    XFCE_PID=$!
    
    echo "Oczekiwanie na okno MAVProxy..."
    for i in {1..45}; do
        if wmctrl -l | grep -q "MAVProxy"; then
            # Ułożenie po LEWEJ stronie ekranu
            wmctrl -r "MAVProxy" -e 0,0,0,$HALF_WIDTH,$HEIGHT
            echo "Znaleziono i ustawiono MAVProxy."
            break
        fi
        sleep 1
    done

    # Wracamy do katalogu głównego
    cd $PROJECT_DIR

    # 2. Uruchom nagrywanie faktycznego pulpitu!
    ffmpeg -y -video_size $RESOLUTION -framerate 30 -f x11grab -i $DISPLAY -c:v libx264 -preset fast -pix_fmt yuv420p "nagrania/${FILENAME}.mp4" < /dev/null > /dev/null 2>&1 &
    FFMPEG_PID=$!

    # 3. Uruchom eksperyment (Python)
    python3 -m $EXP &
    PY_PID=$!
    
    echo "Oczekiwanie na okno Matplotlib..."
    for i in {1..30}; do
        if wmctrl -l | grep -q "Figure 1"; then
            # Ułożenie po PRAWEJ stronie ekranu
            wmctrl -r "Figure 1" -e 0,$HALF_WIDTH,0,$HALF_WIDTH,$HEIGHT
            echo "Znaleziono i ustawiono okno Matplotlib."
            break
        fi
        sleep 1
    done

    # 4. Czekamy na zakończenie skryptu Pythona
    wait $PY_PID
    
    # 5. Zatrzymujemy nagrywanie (bezpiecznie przez SIGINT)
    kill -SIGINT $FFMPEG_PID
    wait $FFMPEG_PID 2>/dev/null

    # 6. Zamykamy symulator terminala (co zabije też sim_vehicle.py)
    kill -9 $XFCE_PID 2>/dev/null
    sleep 2
done

echo "Wszystkie eksperymenty zostały nagrane prosto z Twojego ekranu i zapisane w katalogu 'nagrania/'!"
