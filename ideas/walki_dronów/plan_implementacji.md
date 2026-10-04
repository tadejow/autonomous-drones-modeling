# Walki Dronów 3 vs 3: plan implementacji

> Dokument roboczy do specyfikacji z `ideas/walki_dronów/walki_dronów_prompt.txt`.
> Zakres: nowy katalog `06_turniej_areny/`. Katalogi `01`–`05` pozostają bez zmian (tylko do odczytu, służą jako wzorzec stylu i źródło sprawdzonych fragmentów).
> Kod: angielski, PEP 8, pełne type hints. Dokumentacja: polski (jak w pozostałych modułach kursu).

---

## 0. Streszczenie

Specyfikacja jest dobra jako punkt wyjścia, ale w dosłownej formie ma kilka problemów, które ujawnią się dopiero na zajęciach przy 6 symulatorach i 15 studentach:

1. **Porty i mapa:** zbiorcza instancja MAVProxy podpięta pod `tcp:5760`, `tcp:5770`, … zgłosi konflikt, bo port `5760+10·I` jest już zajęty przez MAVProxy uruchomione przez `sim_vehicle.py` (SITL obsługuje tam jednego klienta).
2. **Nazwy parametrów:** w ArduCopterze nie istnieją `BATT_FS` ani `WP_SPEED`. Właściwe to `BATT_FS_LOW_ACT`, `BATT_FS_CRT_ACT` oraz `WPNAV_SPEED` (cm/s).
3. **Pętla sterowania i wizualizacja w jednym wątku:** dwa subploty 3D z `ax.clear()` i `plt.pause(0.05)` dają realnie 3–6 FPS. Przy prędkości względnej 20 m/s dron przelatuje 3–6 m między klatkami, więc silnik kolizji z progiem 2 m **przegapi trafienia** (tunelowanie).
4. **`try...except` nie chroni przed zawieszeniem:** `while True:` albo `time.sleep(5)` w kodzie studenta zatrzyma całą arenę, a w trybie GUIDED dron po ok. 3 s bez komendy sam się zatrzymuje.
5. **`killall xterm`** zamknie też terminale użytkownika (ten sam problem był już naprawiany w commicie `449bf34` dla skryptów roju).
6. **Balans gry:** obrońcy niewrażliwi na trafienia mogą po prostu „zaparkować” na bazie. Brak reguły anty-campingowej.
7. **Brak trybu bez SITL:** każdy test strategii wymaga 6 symulatorów (ok. 1–2 min rozruchu). Studenci nie przetestują pomysłów w domu na słabszym laptopie, a turniej każdy z każdym jest niewykonalny.

Proponowana architektura zachowuje **dokładnie** interfejs dla studentów i 5 plików ze specyfikacji jako punkty wejścia, ale rozdziela odpowiedzialności na mały pakiet `arena/` z:
- **wymiennym backendem fizyki** (`SitlBackend` lub szybki `KinematicBackend` bez ArduPilota),
- **sędzią** jako czystymi funkcjami (testowalnymi jednostkowo) z **ciągłą detekcją kolizji**,
- **piaskownicą** uruchamiającą kod studentów w osobnym procesie z limitem czasu,
- **wizualizatorem w osobnym procesie** (pętla sterowania ma stałe 10 Hz niezależnie od FPS wykresów),
- **rejestratorem meczu** (JSONL) i **odtwarzaczem powtórek**.

---

## 1. Co już jest w repozytorium i co z tego bierzemy

| Źródło | Co zawiera | Jak wykorzystujemy w arenie |
|---|---|---|
| `05_uklady_autonomiczne/05_a_*.py`, `05_b_*.py` | Łączenie z N dronami po portach `14550 + 10·i`, wyłączanie `BATT_FS_*`, równoległy start, `send_ned_velocity` z maską `0b0000111111000111`, `get_distance_ned` (GPS→NED z `cos(lat)`) | Ta sama maska i ta sama matematyka, przeniesione do `arena/geo.py` i `arena/backends/sitl.py` z type hints i testami |
| `05_uklady_autonomiczne/start_swarm*.sh` | `sim_vehicle.py -I<i> --sysid <n> -l lat,lon,alt,hdg --out=...`, `xfce4-terminal --disable-server` | Wzorzec dla `start_arena.sh`; uwzględniamy lekcję z `449bf34` (nie zabijać terminali użytkownika) |
| `04_wizualizacja_3D/07_b_pursuit.py` | Przechwycenie predykcyjne (równanie kwadratowe na czas spotkania), spoofing ADS-B | Gotowa strategia przykładowa dla obrońców („lead pursuit”); ADS-B jako opcjonalne znaczniki bazy na mapie |
| `pipeline/visualization/plotter.py` | `DronePlotter`: `deque` jako historia, `TkAgg`, `view_init` | Ten sam idiom historii; nowy `ArenaVisualizer` z artystami tworzonymi raz i aktualizowanymi (zamiast `ax.clear()`) |
| `pipeline/math/physics.py` | NED→GPS (`get_location_meters`) | Odwrotność i ta sama stała `R = 6378137.0` w `arena/geo.py` |
| Łatka `collections.MutableMapping` na górze skryptów | Zgodność DroneKit 2.9.2 z Pythonem 3.10+ | Jedno miejsce: `arena/compat.py`, importowane przed `dronekit` |

**Obserwacja o układzie osi:** dotychczasowe skrypty rysują `(N, E, Up)` jako `(x, y, z)`. Ponieważ NED jest prawoskrętny, a `N × E = D`, trójka `(N, E, Up)` jest **lewoskrętna**, więc wykres 3D jest lustrzanym odbiciem mapy MAVProxy (wschód i zachód zamienione przy patrzeniu z góry). W arenie rysujemy w układzie **ENU** `(x=E, y=N, z=Up)`, który jest prawoskrętny i zgodny z mapą. Ma to znaczenie, bo studenci będą porównywać okno 1 (mapa) z oknami 2 i 3.

---

## 2. Docelowa struktura katalogu

```text
06_turniej_areny/
├── README.md                    # Zasady gry, API dla studentów, instrukcja uruchomienia (PL)
├── start_arena.sh               # [spec] 6× SITL + zbiorcza mapa MAVProxy
├── stop_arena.sh                # Sprząta tylko procesy areny (pidfile)
├── arena.parm                   # Parametry ArduCoptera ładowane przy starcie SITL
├── arena_config.toml            # Wszystkie liczby gry w jednym miejscu
├── arena_orchestrator.py        # [spec] CLI: jeden mecz (SITL lub kinematic)
├── arena_visualizer.py          # [spec] ArenaVisualizer (2 subploty 3D)
├── team_attacker.py             # [spec] szablon studenta
├── team_defender.py             # [spec] szablon studenta
├── tournament.py                # Turniej każdy z każdym na backendzie kinematycznym
├── replay.py                    # Odtwarzanie meczu z logu (bez SITL), eksport MP4/GIF
├── arena/
│   ├── __init__.py
│   ├── compat.py                # Łatka collections.MutableMapping dla DroneKit
│   ├── config.py                # @dataclass(frozen=True) ArenaConfig + loader TOML
│   ├── types.py                 # TypedDict DroneState, Vec3, TeamName, GameEvent, MatchResult
│   ├── geo.py                   # GPS <-> NED, NED <-> ENU
│   ├── referee.py               # Kolizje (CCD), warunki zwycięstwa, kolejność rozstrzygania
│   ├── safety.py                # Safety Limiter, walidacja wyjścia, geofence
│   ├── sandbox.py               # Uruchamianie kodu studentów w procesie z timeoutem
│   ├── recorder.py              # Zapis JSONL tick po ticku
│   ├── clock.py                 # Pętla o stałej częstotliwości, pomiar jittera
│   └── backends/
│       ├── base.py              # Protocol PhysicsBackend
│       ├── sitl.py              # DroneKit + ArduPilot SITL
│       └── kinematic.py         # Model punktu materialnego z opóźnieniem 1. rzędu
├── teams/                       # Strategie przykładowe (i miejsce na zgłoszenia studentów)
│   ├── baseline/                # = heurystyki ze szablonów
│   ├── lead_pursuit/            # obrońcy: przechwycenie predykcyjne (z 07_b)
│   └── evasive/                 # atakujący: odpychanie kulombowskie od obrońców (z 05)
└── tests/
    ├── test_geo.py
    ├── test_referee.py
    ├── test_safety.py
    ├── test_sandbox.py
    └── test_match_kinematic.py  # pełny mecz headless (MPLBACKEND=Agg)
```

Pięć plików ze specyfikacji zostaje punktami wejścia o tych samych nazwach i z tym samym interfejsem; logika nietrywialna siedzi w `arena/`, dzięki czemu jest testowalna bez SITL.

---

## 3. Zasady gry: wartości domyślne i doprecyzowania

Specyfikacja zostawia kilka luk. Proponuję poniższe wartości domyślne (wszystkie w `arena_config.toml`, więc łatwo je zmienić na zajęciach).

| Parametr | Wartość domyślna | Uzasadnienie |
|---|---|---|
| Częstotliwość ticku sterowania | 10 Hz (`dt = 0.1 s`) | Wystarczająca dla SITL, bezpiecznie powyżej limitu `GUID_TIMEOUT` |
| Promień trafienia | 2.0 m (spec) | Liczony **ciągle** między tickami (rozdz. 7.2) |
| Promień celu | 5.0 m (spec), w 3D | Patrz punkt o wysokości celu niżej |
| Punkt celu | `(0, 0, -10)` NED, czyli 10 m nad bazą obrońców | Gdyby cel był na ziemi (`D = 0`), atakujący musiałby zejść do 5 m nad gruntem przy 10 m/s; w SITL kończy się to uderzeniem w ziemię. 10 m daje sensowną grę w 3D |
| Limit czasu | 120 s (spec) liczone od sygnału START, nie od uruchomienia skryptu | Start i odliczanie nie zjadają czasu meczu |
| Maks. prędkość komendy | 10.0 m/s, moduł wektora 3D (spec) | Dodatkowo osobny limit pionowy 3 m/s (realizm, ochrona przed uderzeniem w ziemię) |
| Wysokość startowa | 15 m (spec) | |
| Geofence | `N ∈ [-40, 210]`, `E ∈ [-60, 60]`, wysokość `[3, 40]` m | Miękka ściana w Safety Limiterze (rozdz. 7.3) |
| Strefa zakazana dla obrońców | kula 8 m wokół celu | **Reguła anty-campingowa**, patrz niżej |
| Śmierć | atakujący `alive = False` + jednorazowe `LAND` (spec) | Opcjonalnie `kill_mode = "freefall"` (rozbrojenie w powietrzu) |
| Trafienia obrońców | wyłączone (spec) | Opcjonalnie `mutual_kill = true` (wariant „kamikadze”) |

**Kolejność rozstrzygania w jednym ticku** (musi być ustalona, bo inaczej wynik zależy od przypadku):
1. Aktualizacja stanu z backendu.
2. Detekcja trafień (CCD) dla wszystkich par (żywy atakujący, żywy obrońca) na odcinku `[t-dt, t]`. Wszystkie trafienia z tego ticku są stosowane jednocześnie.
3. Sprawdzenie celu tylko dla atakujących, którzy **przeżyli krok 2**. Remis (trafienie i dotarcie w tym samym ticku) rozstrzygany na korzyść obrońców; jest to jawnie opisane w README.
4. Sprawdzenie „wszyscy atakujący nie żyją” oraz limitu czasu.
5. Dopiero potem wywołanie strategii i wysłanie komend.

**Reguła anty-campingowa (zalecana):** przy dosłownej specyfikacji trzech obrońców wiszących w odległości 2–3 m od celu praktycznie uniemożliwia atak, bo każdy tor do kuli 5 m przechodzi przez ich strefę 2 m. Proponuję, by obrońca przebywający w kuli 8 m wokół celu był „wyłączony” (nie zabija), a orkiestrator wypychał go na zewnątrz w Safety Limiterze. To klasyczne założenie w literaturze gier Target–Attacker–Defender i wymusza na obrońcach przechwytywanie, a nie blokowanie. W konfiguracji: `defender_exclusion_radius = 8.0` (`0` wyłącza regułę, zgodnie ze specyfikacją).

**Balans:** wartości (promienie, prędkości, strefa) należy dostroić na backendzie kinematycznym symulacją Monte Carlo strategii przykładowych (rozdz. 11, faza 5): cel to ok. 40–60% zwycięstw atakujących dla par „baseline vs baseline” i wyraźna przewaga lepszych strategii.

---

## 4. Interfejs dla studentów (bez zmian względem specyfikacji)

```python
def compute_commands(
    my_team: dict[int, DroneState],
    enemy_team: dict[int, DroneState],
    target_pos: tuple[float, float, float],
    current_time: float,
) -> dict[int, tuple[float, float, float]]:
    ...
```

- `DroneState = TypedDict("DroneState", {"pos": Vec3, "vel": Vec3, "alive": bool})`, wszystko w metrach i m/s, **NED względem bazy obrońców**.
- Identyfikatory dronów: `int` = SysID (atakujący `1, 2, 3`, obrońcy `4, 5, 6`), takie same jak etykiety na mapie MAVProxy.
- `current_time`: sekundy od sygnału START.
- Wynik: `{drone_id: (vN, vE, vD)}`. Brakujący klucz oznacza „zawis” (0, 0, 0). Klucze obcej drużyny i martwych dronów są ignorowane (z ostrzeżeniem w logu).

Szablony zawierają w docstringu duże ostrzeżenie: **`vD > 0` oznacza opadanie**. To najczęstszy błąd przy pracy z NED; kontrolka w orkiestratorze dodatkowo loguje ostrzeżenie, gdy dron strategii spadnie poniżej 5 m.

Heurystyki w szablonach (wymagane przez specyfikację, żeby system dało się przetestować):
- `team_attacker.py`: każdy żywy atakujący leci po prostej do `target_pos` z prędkością 10 m/s, z członem P na wysokość.
- `team_defender.py`: każdy żywy obrońca leci do najbliższego żywego atakującego (pure pursuit); gdy atakujących brak, wraca nad bazę.

Oba pliki są czystymi funkcjami: nie importują `dronekit`, nie mają stanu globalnego. W README dopisujemy, że wolno trzymać stan między wywołaniami w zmiennych modułu (np. przydział celów), bo proces piaskownicy jest trwały przez cały mecz.

**Opcjonalne rozszerzenie (wersja 2, zgodne wstecz):** jeśli funkcja przyjmuje dodatkowy parametr `game: GameInfo | None = None`, orkiestrator przekaże w nim `dt`, `time_left`, promienie z konfiguracji i granice areny. Wykrywane przez `inspect.signature`, więc stare zgłoszenia działają bez zmian.

---

## 5. `start_arena.sh` i `stop_arena.sh`

### 5.1. Problem z mapą zbiorczą w specyfikacji

`sim_vehicle.py -I i` uruchamia SITL nasłuchujący na `tcp:5760+10·i` (SERIAL0) i domyślnie dokleja do niego własne MAVProxy. SITL obsługuje na tym porcie jednego klienta, więc druga, zbiorcza instancja MAVProxy z `--master tcp:127.0.0.1:5760 ...` się nie połączy. Są dwa poprawne warianty.

**Wariant A (zgodny z portami UDP ze specyfikacji, zalecany na start):**
każde `sim_vehicle.py` dostaje dwa wyjścia UDP: jedno dla orkiestratora, drugie dla mapy.

```bash
--out=udp:127.0.0.1:$((14550 + 10*i))   # orkiestrator (DroneKit)
--out=udp:127.0.0.1:$((14650 + 10*i))   # zbiorcza mapa
```
Na końcu skryptu:
```bash
mavproxy.py --master=udp:127.0.0.1:14650 --master=udp:127.0.0.1:14660 ... \
            --master=udp:127.0.0.1:14700 --map --console
```

**Wariant B (lżejszy, docelowy):** `sim_vehicle.py --no-mavproxy` (6 procesów `arducopter` bez 6 konsol MAVProxy). Orkiestrator łączy się z `tcp:127.0.0.1:5760+10·i`, a mapa z drugim portem SITL `tcp:127.0.0.1:5762+10·i` (SERIAL1, domyślnie MAVLink). Oszczędza ok. 6 procesów Pythona i 6 okien. Wymaga weryfikacji w fazie 0, że SERIAL1 w używanej wersji ArduPilota rzeczywiście wystawia MAVLink na `5762`.

Skrypt przyjmuje `ARENA_MODE=udp|tcp` (domyślnie `udp`), a orkiestrator ma ten sam przełącznik w konfiguracji, więc oba warianty działają z tym samym kodem.

### 5.2. Pozycje startowe (policzone)

Stała `R = 6378137 m`, `lat₀ ≈ -35.362°`, `cos(lat₀) ≈ 0.8155`:
- 1° szerokości ≈ `π/180 · R` ≈ **111 319.5 m**,
- 1° długości ≈ `111 319.5 · 0.8155` ≈ **90 785 m**.

| | Szerokość | Długość (spacing 2 m) | Kurs |
|---|---|---|---|
| Atakujący 1 / 2 / 3 | -35.36200 | 149.164978 / 149.165000 / 149.165022 | 180° (na południe) |
| Obrońcy 4 / 5 / 6 | -35.36335 | 149.164978 / 149.165000 / 149.165022 | 0° (na północ) |

- 2 m na wschód to `2 / 90 785 ≈ 2.20·10⁻⁵°`.
- Specyfikacja proponuje dla obrońców `-35.3635`, co daje `0.0015° · 111 319.5 ≈ 167 m`, a nie 150 m. Dla 150 m należy użyć **`-35.36335`** (`0.00135° → 150.3 m`).
- Wysokość terenu CMAC: 584 m, więc `-l lat,lon,584,hdg`.
- **Zalecenie:** spacing 5 m zamiast 2 m (`5.51·10⁻⁵°`). SITL nie symuluje zderzeń między dronami, ale przy 2 m znaczniki na mapie i w 3D zlewają się w jeden punkt, a dryf EKF przy starcie bywa rzędu 1 m. Wartość jest zmienną na górze skryptu.
- Skrypt nie wpisuje współrzędnych ręcznie: liczy je jedną linijką `python3 -c` z tych samych stałych, które czyta orkiestrator (`arena_config.toml`), żeby nie było dwóch źródeł prawdy.

### 5.3. Szkielet skryptu

```bash
#!/usr/bin/env bash
set -euo pipefail
ARENA_DIR="$(cd "$(dirname "$0")" && pwd)"
PIDFILE="$ARENA_DIR/.arena_pids"
SIM="$HOME/ardupilot/Tools/autotest/sim_vehicle.py"
TERM_CMD="${ARENA_TERMINAL:-xterm -hold -e}"      # lub: xfce4-terminal --disable-server -x

"$ARENA_DIR/stop_arena.sh"                          # sprząta tylko poprzednią arenę
[[ -x "$HOME/ardupilot/build/sitl/bin/arducopter" ]] || { echo "Najpierw zbuduj SITL"; exit 1; }

launch() {  # $1=instance $2=sysid $3=lat $4=lon $5=heading $6=udp_port
  $TERM_CMD bash -c "cd ~/ardupilot/ArduCopter && $SIM -v ArduCopter --no-rebuild \
     -I$1 --sysid $2 -l $3,$4,584,$5 \
     --add-param-file=$ARENA_DIR/arena.parm \
     --out=udp:127.0.0.1:$6 --out=udp:127.0.0.1:$(( $6 + 100 ))" &
  echo $! >> "$PIDFILE"
}
# ... 6 wywołań launch ...
sleep 5
mavproxy.py $(for p in 14650 14660 14670 14680 14690 14700; do echo --master=udp:127.0.0.1:$p; done) \
            --map --console &
echo $! >> "$PIDFILE"
```

Kluczowe decyzje:
- **Nie ma `killall xterm`.** `stop_arena.sh` zabija PID-y z `.arena_pids` oraz `pkill -f "sim_vehicle.py.*arena.parm"` i `pkill -f "arducopter.*--sysid [1-6]"`. Terminale użytkownika są bezpieczne.
- `--add-param-file=arena.parm` zamiast ustawiania parametrów z Pythona: parametry obowiązują od pierwszej sekundy, bez wyścigu z pre-arm checkami.
- `ARENA_TERMINAL` pozwala użyć `xfce4-terminal` (jak w module 05, commit `798bff4`) lub `xterm -hold` (jak w specyfikacji). Na WSL2 bez WSLg można uruchomić bez okien: `ARENA_TERMINAL=""` z logami do `logs/sitl_<i>.log`.
- Kontrola zależności na początku (`xterm`, `mavproxy.py`, zbudowany `arducopter`), z czytelnym komunikatem, tak jak w `scripts/record_master.sh`.

### 5.4. `arena.parm`

```text
BATT_FS_LOW_ACT  0       # spec: wyłączenie failsafe baterii
BATT_FS_CRT_ACT  0
FS_GCS_ENABLE    0       # brak GCS-failsafe przy zawieszeniu orkiestratora
WPNAV_SPEED      1500    # spec "WP_SPEED = 1500": 15 m/s
WPNAV_SPEED_UP   500
WPNAV_SPEED_DN   300
WPNAV_ACCEL      500     # do strojenia: szybsze manewry
LOIT_SPEED       1500
SR0_POSITION     10      # GLOBAL_POSITION_INT 10 Hz (domyślnie bywa 2–4 Hz)
SR1_POSITION     10
SR2_POSITION     10
SR0_EXTRA1       10      # ATTITUDE
GUID_TIMEOUT     3       # jawnie: po 3 s bez komendy dron staje
```

Wszystkie nazwy do potwierdzenia na zainstalowanej wersji ArduPilota w fazie 0 (`param show WPNAV*` w MAVProxy). Jeśli limit prędkości w GUIDED (komendy prędkościowe) w danej wersji zależy od innego parametru, ustawimy go tutaj, nie w kodzie.

---

## 6. Backend fizyki

### 6.1. Protokół

```python
class PhysicsBackend(Protocol):
    def connect(self) -> None: ...
    def takeoff_all(self, altitude_m: float) -> None: ...
    def read_states(self) -> dict[int, RawDroneState]: ...      # pos/vel NED + stamp
    def send_velocity(self, drone_id: int, vel_ned: Vec3) -> None: ...
    def kill(self, drone_id: int, mode: KillMode) -> None: ...
    def shutdown(self) -> None: ...                                # LAND/RTL + close
    def now(self) -> float: ...                                    # zegar meczu
```

Orkiestrator nie wie, czy pod spodem jest ArduPilot. To jest najważniejsza decyzja architektoniczna planu.

### 6.2. `SitlBackend` (DroneKit)

- **Równoległe łączenie** 6 dronów `ThreadPoolExecutor(max_workers=6)`: `connect(..., wait_ready=True)` trwa kilka sekund na drona, sekwencyjnie wychodzi ok. 30–60 s.
- **Spójny odczyt pozycji:** listener `@vehicle.on_message("GLOBAL_POSITION_INT")` zapisuje atomowo `lat, lon, alt (AMSL), vx, vy, vz, time_boot_ms` i czas odbioru (`time.monotonic()`). Atrybuty `vehicle.location` i `vehicle.velocity` w DroneKit mogą pochodzić z różnych wiadomości i być z różnych chwil.
- **Wykrywanie nieaktualnej telemetrii:** jeśli ostatnia wiadomość jest starsza niż 1 s, stan drona dostaje `stale=True`; sędzia nie liczy dla niego trafień w tym ticku, a log dostaje ostrzeżenie.
- **Komenda prędkości:** `SET_POSITION_TARGET_LOCAL_NED`, `MAV_FRAME_LOCAL_NED`, maska `0b0000111111000111` (jak w module 05). Wysyłana **co tick do każdego żywego drona**, także gdy wynik strategii się nie zmienił, żeby nie wpaść w `GUID_TIMEOUT`.
- **INIT:** `GUIDED` → `armed = True` dla wszystkich → `simple_takeoff(15)` dla wszystkich → czekanie aż każdy ma `alt ≥ 0.95·15` (z timeoutem 60 s i czytelnym błędem, który dron nie wystartował) → 3 s stabilizacji → odliczanie → START.
- **Kill:** `vehicle.mode = VehicleMode("LAND")` jednorazowo (spec). Wariant `freefall`: `MAV_CMD_COMPONENT_ARM_DISARM` z `param2 = 21196` (wymuszone rozbrojenie w locie); w SITL dron spada, co wizualnie lepiej oddaje „zestrzelenie”.
- **Wiatr (opcja):** `SIM_WIND_SPD` / `SIM_WIND_DIR` jak w `04_physics_wind.py`, ustawiane identycznie dla wszystkich 6 instancji.

### 6.3. `KinematicBackend` (bez SITL)

Model punktu materialnego, w którym prędkość dąży do komendy z opóźnieniem pierwszego rzędu i ograniczonym przyspieszeniem:

```
v̇ = clip_a( (v_cmd − v) / τ ),   |v̇| ≤ a_max
ṗ = v
```

- Całkowanie dokładne dla członu liniowego: `v(t+dt) = v_cmd + (v − v_cmd)·e^(−dt/τ)`, następnie przycięcie przyspieszenia.
- Parametry `τ` i `a_max` **kalibrujemy na SITL** (faza 4): skok komendy 0 → 10 m/s, rejestracja odpowiedzi, dopasowanie metodą najmniejszych kwadratów. To samo w sobie jest dobrym zadaniem na zajęcia (identyfikacja systemu).
- Opcjonalny szum pozycji (σ ≈ 0.3 m) i opóźnienie telemetrii (1 tick), żeby strategie nie były przeuczone na idealne dane.
- Tryb czasu: `realtime` (z wizualizacją) albo `fast` (bez `sleep`, mecz 120 s liczy się w ułamku sekundy). `fast` jest podstawą turnieju i testów.

---

## 7. Sędzia i bezpieczeństwo (czysta matematyka, w pełni testowalna)

### 7.1. GPS → NED (`arena/geo.py`)

Przybliżenie równoodległościowe (equirectangular) względem bazy obrońców `(φ₀, λ₀, h₀)`:

```
N = R · (φ − φ₀)
E = R · cos(φ₀) · (λ − λ₀)
D = −(h − h₀)
```
z kątami w radianach i `R = 6 378 137 m`.

- Używamy `cos(φ₀)` (stała bazy), a nie `cos(φ)`, żeby przekształcenie było liniowe i dokładnie odwracalne (`ned_to_gps` w testach daje identyczność do 1e-9).
- Błąd przybliżenia na dystansie 200 m jest rzędu milimetrów; test porównuje z formułą haversine i asercją `< 1 cm`.
- `h` to wysokość AMSL z `GLOBAL_POSITION_INT.alt`, a `h₀ = 584 m`. Nie używamy `relative_alt`, bo jest liczony względem domu **każdego** drona; tu przypadkiem wszystkie domy mają 584 m, ale na innym lotnisku z nachyleniem terenu dałoby to błąd.
- Do wizualizacji `ned_to_enu(n, e, d) = (e, n, −d)`.

### 7.2. Ciągła detekcja kolizji (CCD) zamiast próbkowania

Problem: przy 10 Hz i prędkości względnej 20 m/s drony zbliżają się o 2 m na tick; przy 4 FPS (gdyby pętla czekała na wykres) o 5 m. Sprawdzanie `|p_A − p_D| < 2` tylko w chwilach próbkowania gubi trafienia.

Rozwiązanie: na odcinku `[t−dt, t]` zakładamy ruch liniowy obu dronów. Wektor względny:

```
r(s) = r₀ + s·Δ,   s ∈ [0, 1]
r₀ = p_A(t−dt) − p_D(t−dt)
Δ  = (p_A(t) − p_A(t−dt)) − (p_D(t) − p_D(t−dt))
s* = clamp( −(r₀·Δ) / |Δ|², 0, 1 )      (dla |Δ| ≈ 0: s* = 0)
d_min = |r₀ + s*·Δ|
```

Trafienie, gdy `d_min < kill_radius`. Funkcja `closest_approach(...) -> tuple[float, float]` zwraca `(d_min, s*)`; `s*` służy do zapisania dokładnego czasu i miejsca trafienia w logu i na wykresie (znacznik eksplozji). Ten sam wzór stosujemy dla celu (odcinek atakującego vs punkt celu), żeby szybki atakujący nie „przeskoczył” kuli 5 m.

Testy: przelot na wprost przez siebie między próbkami (stara metoda: brak trafienia, CCD: trafienie), równoległy lot w odległości 2.1 m (brak), dron nieruchomy, `Δ = 0`.

### 7.3. Safety Limiter (`arena/safety.py`)

Kolejność operacji na każdym wektorze zwróconym przez studenta:
1. **Walidacja typu:** krotka/lista 3 liczb; `NaN`/`inf` → `(0, 0, 0)` i ostrzeżenie.
2. **Geofence (miękka ściana):** dla każdej granicy, jeśli dron jest w odległości `m < margin` od ściany, składowa w stronę ściany jest ograniczana do `k·m`, a poza ścianą wymuszana jest prędkość powrotna. Szczególnie: wysokość < 3 m wymusza `vD ≤ −1 m/s` (wznoszenie). Anty-camping obrońców (rozdz. 3) realizowany tak samo, kulą wokół celu.
3. **Limit pionowy:** `|vD| ≤ 3 m/s`.
4. **Limit modułu (spec):** `v ← v · min(1, v_max / |v|)`, `v_max = 10 m/s`. Skalowanie całego wektora zachowuje kierunek (przycięcie składowych osobno by go zmieniało).

Każda interwencja jest liczona w statystykach meczu („ile razy limiter poprawił twoją strategię”), co jest użyteczną informacją zwrotną dla studentów.

### 7.4. Warunki zwycięstwa

`referee.evaluate(prev, curr, cfg) -> TickOutcome` zwraca listę zdarzeń (`HIT`, `TARGET_REACHED`, `TIMEOUT`, `ALL_ATTACKERS_DOWN`) i ewentualny `MatchResult(winner, reason, time, survivors, ...)`. Orkiestrator tylko wykonuje skutki (np. `backend.kill`). Dzięki temu cała logika gry jest testowana bez SITL i bez czasu rzeczywistego.

---

## 8. Piaskownica dla kodu studentów (`arena/sandbox.py`)

`try...except` ze specyfikacji łapie wyjątki, ale nie zawieszenia, nie zbyt wolny kod i nie modyfikację słowników przekazanych do funkcji. Proponuję:

- **Osobny, trwały proces na drużynę** (`multiprocessing`, kontekst `spawn`), który importuje moduł studenta raz i w pętli odbiera stan przez `Pipe`. Stan modułu (np. przydziały celów) przeżywa między tickami.
- **Budżet czasu na tick:** domyślnie 30 ms. Po przekroczeniu orkiestrator używa **ostatniej poprawnej komendy** (lub zawisu) i zwiększa licznik spóźnień. Po 3 s ciągłych spóźnień proces jest restartowany; po 3 restartach drużyna przegrywa walkowerem (konfigurowalne).
- **Wyjątki:** łapane w procesie potomnym, traceback logowany do pliku drużyny i wyświetlany skrócony w konsoli; komendy jak przy timeoucie.
- **Kopie danych:** proces potomny dostaje serializowane kopie, więc student nie może zmienić stanu sędziego ani zobaczyć obiektów `Vehicle`.
- **Tryb debug:** `--inline` wywołuje funkcję w tym samym procesie (z `try...except` jak w spec), żeby studenci mogli używać breakpointów w PyCharm.

To **nie jest** zabezpieczenie przed złośliwym kodem (student może wywołać `os.system`). Na zajęciach wystarcza; przy turnieju z nagrodami warto uruchamiać zgłoszenia w kontenerze (opisane w README jako opcja, poza zakresem v1).

---

## 9. Orkiestrator (`arena_orchestrator.py`)

### 9.1. CLI

```bash
python arena_orchestrator.py \
    --attacker team_attacker --defender team_defender \
    --backend sitl|kinematic --config arena_config.toml \
    [--no-viz] [--inline] [--seed 42] [--record matches/2026-10-12_alpha_vs_beta.jsonl]
```
`--attacker` i `--defender` przyjmują ścieżkę modułu (`teams.lead_pursuit.defender`) albo pliku, więc ten sam orkiestrator gra szablonami, przykładami i zgłoszeniami studentów.

### 9.2. Pętla BATTLE

```python
clock = FixedRateClock(hz=cfg.tick_hz)
while result is None:
    states = backend.read_states()                       # 1. telemetria
    outcome = referee.evaluate(prev_states, states, cfg)  # 2-4. trafienia, cel, czas
    for ev in outcome.hits:
        backend.kill(ev.victim, cfg.kill_mode)            #    jednorazowe LAND
    result = outcome.result
    if result is None:
        cmds_a = sandbox_a.request(view_for("attackers", states), timeout=cfg.budget)
        cmds_d = sandbox_d.request(view_for("defenders", states), timeout=cfg.budget)
        for drone_id, v in safety.apply(cmds_a | cmds_d, states, cfg).items():
            backend.send_velocity(drone_id, v)            # 5. komendy
    recorder.write(tick, states, cmds, outcome)
    viz_queue.put_nowait_latest(snapshot)                 # 6. wizualizacja (bez blokowania)
    prev_states = states
    clock.sleep_until_next_tick()                         # mierzy jitter
```

- `FixedRateClock` używa `time.monotonic()` i planuje od **zaplanowanego** czasu poprzedniego ticku (bez kumulacji dryfu). Jitter jest zapisywany; w podsumowaniu meczu pojawia się 95. percentyl, który od razu pokazuje, czy maszyna wyrabia.
- Obie drużyny są pytane **równolegle** (oba procesy dostają stan, potem odbieramy odpowiedzi), żeby żadna nie widziała reakcji drugiej w tym samym ticku.
- `viz_queue` to kolejka o długości 1 z nadpisywaniem: wizualizator zawsze rysuje najnowszy stan i nigdy nie spowalnia sterowania.
- `KeyboardInterrupt` i każdy nieobsłużony wyjątek → blok `finally`: `LAND` dla wszystkich dronów, zamknięcie połączeń, zapis logu, zamknięcie procesów (jak `except KeyboardInterrupt` w module 05, tylko pewniej).

### 9.3. Po meczu

Wyświetla i zapisuje (`results/*.json`): zwycięzcę i powód, czas, ocalałych, listę trafień (kto, kogo, kiedy, gdzie), statystyki limitera i spóźnień obu drużyn, jitter pętli. Drony dostają `LAND` (pozostałe przy życiu) lub `RTL` (konfigurowalne).

---

## 10. Wizualizator (`arena_visualizer.py`)

### 10.1. Zgodność ze specyfikacją

- `matplotlib.use("TkAgg")`, jedna duża figura (`figsize=(18, 8)`), dwa subploty 3D.
- Metoda `update_frame(drones_state: dict, hud: HudInfo | None = None) -> None` z `plt.pause(0.05)`.
- Kolory: atakujący czerwoni, obrońcy niebiescy, zestrzeleni szarzy; przerywane trajektorie (`linestyle="--"`, historia w `deque(maxlen=...)` jak w `DronePlotter`); żółta gwiazdka (`marker="*"`, `s=300`) w miejscu celu.

### 10.2. Ulepszenia

1. **Osobny proces.** `ArenaVisualizer` działa w `multiprocessing.Process` czytającym z kolejki. Tk musi żyć w głównym wątku swojego procesu, więc to też rozwiązuje problemy z wątkami. Orkiestrator nie czeka na rysowanie, a `plt.pause(0.05)` (spec) nie spowalnia sterowania.
2. **Artyści tworzeni raz.** Zamiast `ax.clear()` co klatkę: `Line3D.set_data_3d(...)`, `scatter._offsets3d = (...)`, `set_color(...)`. To rząd wielkości szybciej niż przebudowa sceny, a widok kamery nie skacze.
3. **Układ ENU** (rozdz. 1): `x = East`, `y = North`, `z = Up`. Wykres zgadza się z mapą MAVProxy.
4. **Kamery:**
   - Subplot 1, perspektywa obrońców: `view_init(elev=12, azim=-90)` (patrzymy w stronę `+y`, czyli na północ, z południa), `set_proj_type("persp", focal_length=0.4)`.
   - Subplot 2, perspektywa atakujących: `view_init(elev=12, azim=90)` (patrzymy na południe).
   - Stałe granice osi równe arenie (`ylim = [-40, 210]` itd.) i `set_box_aspect((120, 250, 40))`, żeby skala była prawdziwa, a obraz nie skakał wraz z autoskalowaniem.
   - Uwaga: `view_init` obraca kamerę wokół środka sceny; nie da się nią postawić kamery dokładnie w punkcie bazy. To przybliżenie ze specyfikacji jest wystarczające. **Wariant edukacyjny (opcja v2):** prawdziwy widok z pierwszej osoby jako subplot 2D z własnym rzutem perspektywicznym (macierz kamery: look-at z bazy na północ, rzut `u = f·x/z`). To ładne ćwiczenie z geometrii rzutowej na zajęcia.
5. **HUD** nad wykresami: zegar meczu, liczba żywych po każdej stronie, ostatnie zdarzenie („D5 zestrzelił A2, t = 37.4 s”), FPS wizualizacji. W miejscu trafienia na 2 s pojawia się znacznik `X`.
6. **Półprzezroczysta kula celu** (promień 5 m) i strefa anty-campingowa (8 m) jako siatki `plot_wireframe`, żeby studenci widzieli geometrię reguł.
7. **Okno 1 zapasowe:** jeśli mapa MAVProxy nie zadziała (częste na WSL2 bez `wxPython`), flaga `--topdown` dodaje trzeci subplot 2D z widokiem z góry w Matplotlib.

---

## 11. Rejestracja, powtórki i turniej

- **`recorder.py`:** jeden wiersz JSONL na tick (`t`, stany, komendy przed i po limiterze, zdarzenia) plus nagłówek z konfiguracją, wersją kodu (`git rev-parse HEAD`) i nazwami drużyn. Mecz 120 s przy 10 Hz to ok. 1 MB.
- **`replay.py`:** odtwarza log tym samym `ArenaVisualizer` (bez SITL, z suwakiem czasu `matplotlib.widgets.Slider`) i eksportuje MP4/GIF przez `FuncAnimation` (podobnie jak `generate_euler_gif.py`). Materiał do prezentacji i omawiania meczów na zajęciach.
- **`tournament.py`:** każdy z każdym na `KinematicBackend` w trybie `fast`, każda para gra **obie strony**, N meczów z losowym zaburzeniem pozycji startowych (`--seed`). Punktacja: zwycięstwo 3 pkt, dodatkowo czas do celu (atak) lub liczba ocalałych sekund (obrona) jako tie-breaker. Wynik: tabela Markdown/CSV i opcjonalnie ranking Elo. Finał (np. 2 najlepsze drużyny) rozgrywany na SITL z wizualizacją.
- **Strojenie balansu:** ten sam mechanizm uruchamia 1000 meczów „baseline vs baseline” dla siatki parametrów (promienie, strefa, prędkości) i wypisuje procent zwycięstw atakujących.

---

## 12. Strategie przykładowe (`teams/`) i powiązanie z teorią

| Strategia | Strona | Idea | Powiązanie z kursem |
|---|---|---|---|
| `baseline` | obie | Prosto do celu / pure pursuit najbliższego | Szablony ze specyfikacji |
| `lead_pursuit` | obrońcy | Celowanie w punkt spotkania z równania `A t² + B t + C = 0` | Bezpośrednio z `07_b_pursuit.py` |
| `assignment` | obrońcy | Przydział obrońca → atakujący minimalizujący sumę czasów przechwycenia; przy 3×3 wystarczy przejrzeć `3! = 6` permutacji (bez SciPy) | Optymalizacja kombinatoryczna jak TSP w module 03 |
| `evasive` | atakujący | Przyciąganie do celu + odpychanie kulombowskie `∝ r / |r|³` od obrońców | Człon repulsji z Cuckera–Smale'a (moduł 05) |
| `decoy` | atakujący | Dwóch atakujących ściąga obrońców po bokach, trzeci leci środkiem z opóźnieniem | Koordynacja roju |
| `apollonius` | obrońcy | Pozycjonowanie na granicy koła Apoloniusza (zbiór punktów, do których obie strony dolatują jednocześnie) | Gry różniczkowe pościgu i ucieczki (Isaacs), gra Target–Attacker–Defender |

Ostatnia pozycja jest dobrym tematem wykładu: z prostej geometrii (stosunek prędkości) wynika, kto wygra przy optymalnej grze obu stron.

**Kolejne poziomy trudności dla studentów** (konfiguracja, ten sam kod areny):
1. Pełna informacja, brak wiatru (spec).
2. Szum pomiaru pozycji wroga (σ = 1 m) → potrzebne filtrowanie (średnia ruchoma, filtr alfa-beta).
3. Wiatr (`SIM_WIND_SPD`) → strategia musi kompensować dryf.
4. Ograniczony zasięg sensorów (wróg widoczny tylko w promieniu 50 m).
5. Sterowanie zdecentralizowane: funkcja wywoływana osobno dla każdego drona, znająca tylko własny stan i sąsiadów.

---

## 13. Plan prac (fazy i kryteria odbioru)

### Faza 0: weryfikacja założeń na SITL (krótki spike, zanim powstanie właściwy kod)
- Uruchomić 6 instancji z `arena.parm`, sprawdzić obciążenie CPU (cel: < 70% na komputerze w sali).
- Potwierdzić warianty portów: A (UDP fan-out) i B (`--no-mavproxy` + `5762`).
- Potwierdzić, że mapa MAVProxy z sześcioma `--master` pokazuje 6 osobnych pojazdów (różne SysID).
- Potwierdzić nazwy i działanie `WPNAV_SPEED`, `GUID_TIMEOUT`, `SR*_POSITION`; zmierzyć rzeczywistą częstotliwość `GLOBAL_POSITION_INT`.
- Sprawdzić `xterm` vs `xfce4-terminal` na docelowym systemie (Ubuntu w sali, WSL2 u studentów).

**Odbiór:** notatka z wynikami w `06_turniej_areny/README.md` (sekcja „Znane ograniczenia”), działający `start_arena.sh` i `stop_arena.sh`.

### Faza 1: rdzeń bez SITL
`config.py`, `types.py`, `geo.py`, `referee.py`, `safety.py` + testy `pytest`.
**Odbiór:** testy zielone; CCD łapie przelot na wprost między próbkami; GPS→NED→GPS identyczność.

### Faza 2: mecz headless na backendzie kinematycznym
`backends/kinematic.py`, `sandbox.py`, `recorder.py`, `clock.py`, szablony studentów, `arena_orchestrator.py --backend kinematic --no-viz`.
**Odbiór:** `test_match_kinematic.py` rozgrywa pełny mecz szablon vs szablon w < 2 s i daje deterministyczny wynik dla danego `--seed`; strategia z `while True:` nie zawiesza areny.

### Faza 3: wizualizacja
`arena_visualizer.py` w osobnym procesie, HUD, `replay.py`.
**Odbiór:** podgląd 10 Hz meczu kinematycznego w czasie rzeczywistym bez spadku częstotliwości sterowania (jitter p95 < 10 ms); powtórka z logu i eksport GIF.

### Faza 4: SITL
`backends/sitl.py`, INIT, kill, finally/cleanup; kalibracja `τ`, `a_max` modelu kinematycznego na podstawie odpowiedzi skokowej SITL.
**Odbiór:** najpierw 1 vs 1, potem 3 vs 3 na SITL; trzy okna (mapa + 2 widoki 3D); `Ctrl+C` w dowolnej fazie ląduje wszystkimi dronami.

### Faza 5: turniej i balans
`tournament.py`, strategie przykładowe z rozdz. 12, strojenie parametrów gry metodą Monte Carlo.
**Odbiór:** baseline vs baseline w przedziale 40–60%; `lead_pursuit` i `evasive` wyraźnie lepsze od baseline.

### Faza 6: dokumentacja i próba generalna
`06_turniej_areny/README.md` w stylu modułów 01–05 (słowniczek, analiza kodu, matematyka: GPS→NED, CCD, limiter, koło Apoloniusza), instrukcja zgłaszania strategii, próba generalna na czystej maszynie (zgodnie z zadaniem 5.2 z `ROADMAP.md`).
**Odbiór:** osoba spoza projektu uruchamia mecz według README bez pomocy.

---

## 14. Testy

- **Jednostkowe** (`pytest`, bez SITL i bez okien): geo, CCD (przypadki brzegowe z rozdz. 7.2), limiter (NaN, inf, zły typ, kierunek zachowany po skalowaniu, geofence), kolejność rozstrzygania remisów, piaskownica (wyjątek, timeout, zła sygnatura, zwrot `None`).
- **Integracyjne:** pełny mecz kinematyczny z `MPLBACKEND=Agg`, deterministyczny dla ziarna.
- **SITL smoke test** (ręczny, opisany w README): `start_arena.sh` → `arena_orchestrator.py` → zwycięstwo którejś strony → `stop_arena.sh` nie zostawia procesów (`pgrep arducopter` pusty).
- Testy działają na Windows bez WSL dla wszystkiego poza `SitlBackend` (importy `dronekit` tylko wewnątrz `backends/sitl.py`).

---

## 15. Zależności

Bez nowych zależności w rdzeniu: `dronekit`, `pymavlink`, `numpy`, `matplotlib` (już w `requirements.txt`), `tomllib` ze standardowej biblioteki (Python ≥ 3.11). `pytest` tylko dla prowadzącego. Brak SciPy (przydział 3×3 przez permutacje).

**Długoterminowo:** DroneKit 2.9.2 nie jest rozwijany od 2019 r. i wymaga łatki `collections.MutableMapping`. Dzięki protokołowi `PhysicsBackend` przejście na czysty `pymavlink` lub MAVSDK w przyszłości dotyczy jednego pliku (`backends/sitl.py`) i nie zmienia API dla studentów.

---

## 16. Ryzyka

| Ryzyko | Skutek | Środek zaradczy |
|---|---|---|
| 6 instancji SITL za ciężkie dla komputerów w sali | SITL wolniejszy niż czas rzeczywisty, rozjazd czasu | Wariant B (`--no-mavproxy`), pomiar w fazie 0, mecze treningowe na backendzie kinematycznym |
| Mapa MAVProxy nie działa na WSL2 | Brak okna 1 | `--topdown` w Matplotlib |
| Różnice wersji ArduPilota (nazwy parametrów, limity GUIDED) | Drony wolniejsze niż 10 m/s | `arena.parm` jako jedno miejsce zmian, weryfikacja w fazie 0 |
| Kod studenta zawiesza proces | Mecz stoi | Piaskownica z budżetem czasu |
| Niezbalansowane zasady | Jedna strona zawsze wygrywa, spada motywacja | Anty-camping + strojenie Monte Carlo |
| Rozjazd zachowania kinematic vs SITL | Strategia dobra w treningu, słaba w finale | Kalibracja `τ`, `a_max`; szum; finał zawsze na SITL |

---

## 17. Zestawienie odstępstw od specyfikacji

| Specyfikacja | Plan | Powód |
|---|---|---|
| Mapa: `--master tcp:5760 ...` | UDP fan-out (A) lub `tcp:5762+10i` (B) | Port 5760 jest zajęty przez MAVProxy danej instancji |
| `BATT_FS`, `WP_SPEED = 1500` | `BATT_FS_LOW_ACT/CRT_ACT = 0`, `WPNAV_SPEED = 1500` w `arena.parm` | Właściwe nazwy parametrów ArduCoptera; ustawione przed startem |
| `killall xterm` | Pidfile + `pkill` po wzorcu | Nie zabijać terminali użytkownika (por. `449bf34`) |
| Obrońcy `lat -35.3635` „ok. 150 m” | `-35.36335` (150 m) | `-35.3635` to 167 m |
| Spacing 2 m | Domyślnie 5 m (konfigurowalne) | Czytelność wizualizacji, dryf EKF |
| Kolizja: dystans w chwili próbki | Ciągła detekcja (CCD) | Brak gubienia trafień przy szybkich przelotach |
| `try...except` | Proces na drużynę + budżet czasu (+ `--inline` do debugowania) | Ochrona przed zawieszeniem i spowolnieniem |
| Wizualizacja w pętli sterowania | Osobny proces, kolejka „najnowszy stan” | Stałe 10 Hz sterowania |
| `ax.clear()` co klatkę | Artyści tworzeni raz | Wydajność, stabilna kamera |
| Osie `(N, E, Up)` | ENU `(E, N, Up)` | Zgodność kierunków z mapą |
| Cel w bazie (wysokość niesprecyzowana) | Punkt 10 m nad bazą, kula 5 m | Grywalność bez uderzeń w ziemię |
| Brak reguły anty-campingowej | Strefa 8 m wokół celu dla obrońców (wyłączalna) | Balans gry |
| Tylko SITL | Dodatkowo backend kinematyczny, powtórki, turniej | Testy w domu, turniej każdy z każdym, regresja |

Wszystko, co dodaje plan, jest wyłączalne konfiguracją, więc „tryb dokładnie jak w specyfikacji” (`arena_config.spec.toml`) pozostaje dostępny.
