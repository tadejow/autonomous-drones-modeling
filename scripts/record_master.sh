#!/bin/bash
# Master script do automatycznego nagrywania eksperymentów (Left: MAVProxy, Right: Matplotlib)
# Wymaga: Xvfb, wmctrl, xdotool, ffmpeg

# 1. Konfiguracja wirtualnego ekranu (jeśli serwer jest headless)
# Uruchamiamy Xvfb na DISPLAY=:99 w rozdzielczości 1920x1080
export DISPLAY=:99
Xvfb :99 -screen 0 1920x1080x24 &
XVFB_PID=$!
sleep 2 # Czekamy na start serwera X

EXPERIMENTS=(
    "pipeline.missions.physics_demo"
    "pipeline.missions.pursuit"
    "pipeline.missions.search_rescue"
)

for EXP in "${EXPERIMENTS[@]}"; do
    echo "================================================="
    echo "Rozpoczynam nagrywanie eksperymentu: $EXP"
    echo "================================================="
    
    # Zastąp kropki podkreślnikami do nazwy pliku
    FILENAME=${EXP//./_}

    # 1. Uruchom SITL i mapę
    sim_vehicle.py -v ArduCopter -f quad -L Wroclaw --map &
    SITL_PID=$!
    
    # Czekamy aż okno mapy (MAVProxy) się pojawi i układamy je po LEWEJ (960x1080)
    # Wymaga zainstalowanego wmctrl
    echo "Oczekiwanie na okno MAVProxy..."
    sleep 10 
    wmctrl -r "MAVProxy" -e 0,0,0,960,1080 || echo "Nie znaleziono okna MAVProxy, kontynuuję..."

    # 2. Uruchom nagrywanie ekranu (całe 1920x1080)
    ffmpeg -y -video_size 1920x1080 -framerate 30 -f x11grab -i :99.0 -c:v libx264 -preset fast -pix_fmt yuv420p "nagraia/${FILENAME}.mp4" < /dev/null > /dev/null 2>&1 &
    FFMPEG_PID=$!

    # 3. Uruchom eksperyment (Python)
    python3 -m $EXP &
    PY_PID=$!
    
    # Czekamy na pojawienie się okna Matplotlib i układamy je po PRAWEJ (x=960, w=960)
    sleep 5
    wmctrl -r "Figure 1" -e 0,960,0,960,1080 || echo "Nie znaleziono okna Matplotlib."

    # 4. Czekamy na zakończenie skryptu Pythona
    wait $PY_PID
    
    # 5. Zatrzymujemy nagrywanie (bezpiecznie przez SIGINT)
    kill -SIGINT $FFMPEG_PID
    wait $FFMPEG_PID

    # 6. Zamykamy SITL
    kill $SITL_PID
    sleep 2
done

# Sprzątanie Xvfb
kill $XVFB_PID
echo "Wszystkie eksperymenty zostały nagrane!"
