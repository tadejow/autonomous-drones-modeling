# Pipeline - Architektura Autonomicznych Dronów

Katalog `pipeline` zawiera ujednolicony, zgodny ze standardami inżynierii oprogramowania (ISO) zbiór modułów do symulacji, kontroli i wizualizacji bezzałogowych statków powietrznych (BSP). Kod ten zastępuje i rozwija skrypty szkoleniowe z fazy POC (01-05).

## Struktura Pakietu

### `pipeline.core`
Moduł odpowiedzialny za absolutne podstawy sprzętowe i protokolarne.
- **`VehicleManager`** (w `vehicle_manager.py`): Klasa zarządzająca połączeniem telemetrycznym z dronem za pomocą biblioteki `dronekit`. 
  - `__init__(connection_string, baud)`: Łączy się ze wskazanym adresem i portem UDP/TCP (domyślnie `udp:127.0.0.1:14550`).
  - `arm_and_takeoff(target_altitude)`: Bezpieczna sekwencja uzbrajania (Pre-Arm checks) i startu (Takeoff) na określoną wysokość.
  - `rtl_and_close()`: Powrót do miejsca startu (Return To Launch) i bezpieczne zamknięcie połączenia MAVLink.
  - `get_vehicle()`: Zwraca surowy obiekt pojazdu (`Vehicle`) dla zaawansowanych modyfikacji.

### `pipeline.math`
Moduł agregujący wszystkie narzędzia matematyczne wymagane do zaawansowanych misji.
- **`trajectory.py`**:
  - `generate_helix(radius, altitude_step, num_points)`: Zwraca punkty (x,y,z) opisujące linię śrubową.
  - `generate_polygon_scan(points, spacing)`: Prototyp wyznaczania punktów pokrycia poligonu algorytmem ray-casting.
- **`physics.py`**:
  - `get_location_meters(original_location, d_north, d_east)`: Przelicza przesunięcie w metrach (NED) na wektor współrzędnych geograficznych (GPS / WGS84).
- **`algorithms.py`**:
  - `tsp_nearest_neighbor(points, start_idx)`: Heurystyczne rozwiązanie Problemu Komiwojażera ułatwiające optymalizację punktów nawigacyjnych (Waypoints).

### `pipeline.missions`
Wysokopoziomowe definicje zadań wykorzystujące obiekty z modułów `core` i `math`.
- **`pursuit.py` (Misja pościgu)**: Algorytm *Lead Pursuit* – przewidywanie przyszłej pozycji uciekającego celu (oznaczonego ikoną balonu) i dążenie do przechwycenia. 
  - Używa własnego obiektu `DronePlotter` i spoofingu MAVLink (fałszywe komunikaty ADS-B `spoof_adsb_target`) do wizualizowania celu na mapach GCS.
- **`search_rescue.py` (Misja poszukiwawczo-ratownicza)**: Pokazuje, jak w praktyce wstrzyknąć punkty wygenerowane przez `tsp_nearest_neighbor` do sterownika, wizualizując ten proces.
- **`physics_demo.py` (Demonstracja wiatru)**: Startuje i rejestruje dokładny wpływ wektora wiatru (SIM_WIND_SPD) na *Attitude* (Pitch, Roll, Yaw). Dron fizycznie przechyla się, aby skompensować zniesienie.

### `pipeline.visualization`
Narzędzia analityczne i plotery działające w czasie rzeczywistym.
- **`plotter.py`**: 
  - Klasa `DronePlotter(title, trail_length)`: Obudowuje `matplotlib.pyplot` w wygodny menedżer perspektywy 3D.
  - `set_view(elev, azim)`: Obraca wirtualną kamerę, co pozwala m.in. na dokładną obserwację "bujania" na wietrze (w połączeniu z wektorem przechyłu).
  - `update(drone_pos, target_pos, balloon_icon, attitude)`: Metoda dodająca nową ramkę. Buforuje historię lotu (`deque`), rysując ciągły, wydłużony ślad po którym poruszał się obiekt, a przekazanie kątów `attitude` rysuje dodatkowy wektor ułatwiający obserwację zjawisk fizycznych.

### `pipeline.swarm`
Systemy wieloagentowe.
- **`cucker_smale.py`**:
  - Implementacja flokowania. Agenci (Drony) aktualizują swoje prędkości zależnie od położenia sąsiadów, realizując funkcje separacji, aliniacji i kohezji.

## Jak zacząć?
Zajrzyj do katalogu `scripts/` (w głównym drzewie projektu), gdzie umieszczone zostały skrypty powłoki uruchamiające odpowiednie symulacje, np. `launch_single.sh` lub `launch_swarm_split.sh`. Następnie uruchamiaj moduły poprzez standardowe wykonanie np. `python -m pipeline.missions.physics_demo`.
