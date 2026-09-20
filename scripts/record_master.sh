#!/bin/bash
# Master script do automatycznego nagrywania eksperymentów (Left: MAVProxy, Right: Matplotlib)

# Przejście do głównego katalogu repozytorium (katalog wyżej względem skryptu)
cd "$(dirname "$0")/.."
export PYTHONPATH=$(pwd)

echo "Sprawdzanie zależności..."
for cmd in Xvfb fluxbox wmctrl ffmpeg sim_vehicle.py python3 xterm; do
    if ! command -v $cmd &> /dev/null; then
        echo "BŁĄD: Nie znaleziono polecenia '$cmd'."
        echo "Zainstaluj je (np. sudo apt install xvfb fluxbox wmctrl ffmpeg xterm) lub upewnij się, że jest w PATH."
        exit 1
    fi
done
echo "Zależności OK."

# Utworzenie katalogu na nagrania
mkdir -p nagrania

# 1. Konfiguracja wirtualnego ekranu (jeśli serwer jest headless)
export DISPLAY=:99
Xvfb :99 -screen 0 1920x1080x24 &
XVFB_PID=$!
sleep 2 # Czekamy na start serwera X

# Uruchamiamy prosty menedżer okien, aby wmctrl miał czym zarządzać
fluxbox -display :99 &
FLUXBOX_PID=$!
sleep 1

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

    # 1. Uruchom SITL i mapę (z jawnymi koordynatami zamiast Geocodera -L Wroclaw)
    sim_vehicle.py -v ArduCopter -f quad -l 51.1078,17.0385,120,0 --map &
    SITL_PID=$!
    
    echo "Oczekiwanie na okno MAVProxy..."
    for i in {1..45}; do
        if wmctrl -l | grep -q "MAVProxy"; then
            wmctrl -r "MAVProxy" -e 0,0,0,960,1080
            echo "Znaleziono i ustawiono MAVProxy."
            break
        fi
        sleep 1
    done

    # 2. Uruchom nagrywanie ekranu
    ffmpeg -y -video_size 1920x1080 -framerate 30 -f x11grab -i :99.0 -c:v libx264 -preset fast -pix_fmt yuv420p "nagrania/${FILENAME}.mp4" < /dev/null > /dev/null 2>&1 &
    FFMPEG_PID=$!

    # 3. Uruchom eksperyment (Python)
    python3 -m $EXP &
    PY_PID=$!
    
    echo "Oczekiwanie na okno Matplotlib..."
    for i in {1..30}; do
        if wmctrl -l | grep -q "Figure 1"; then
            wmctrl -r "Figure 1" -e 0,960,0,960,1080
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

    # 6. Zamykamy SITL i xterm
    kill -9 $SITL_PID 2>/dev/null
    killall -9 xterm 2>/dev/null
    sleep 2
done

# Sprzątanie Xvfb i WM
kill -9 $FLUXBOX_PID 2>/dev/null
kill -9 $XVFB_PID 2>/dev/null
echo "Wszystkie eksperymenty zostały nagrane i zapisane w katalogu 'nagrania/'!"
