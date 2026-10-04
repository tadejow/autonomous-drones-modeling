# Walki Dronów 3 vs 3 (`pipeline/drones_battle`)

Środowisko turniejowe dla studentów: 3 drony atakujące kontra 3 drony broniące bazy. Studenci piszą jedną funkcję `compute_commands`, a arena wywołuje ją 10 razy na sekundę, pilnuje zasad gry, ogranicza prędkości i rysuje przebieg walki. Fizyka pochodzi z ArduPilot SITL albo, do szybkich testów, z prostego modelu kinematycznego.

Zgodnie z resztą kursu kod jest po angielsku (PEP 8, type hints), a dokumentacja po polsku. Szczegółowe uzasadnienie decyzji znajduje się w `ideas/walki_dronów/plan_implementacji.md`.

---

## Szybki start

Wszystkie polecenia uruchamiamy z **katalogu głównego repozytorium**.

### Bez symulatora (Windows, Linux, macOS)

```bash
# Jeden mecz z podglądem 3D w czasie rzeczywistym
python -m pipeline.drones_battle.arena_orchestrator --backend kinematic

# Mecz bez okien, tak szybko jak się da
python -m pipeline.drones_battle.arena_orchestrator --backend kinematic --fast --no-viz

# Własna strategia (ścieżka do pliku lub nazwa modułu)
python -m pipeline.drones_battle.arena_orchestrator --backend kinematic \
    --attacker moj_atak.py --defender pipeline.drones_battle.teams.hunters.defender

# Turniej każdy z każdym wszystkich drużyn z katalogu teams/
python -m pipeline.drones_battle.tournament --rounds 10
```

### Z ArduPilot SITL (Linux / WSL2)

```bash
./pipeline/drones_battle/start_arena.sh          # 6 symulatorów + zbiorcza mapa MAVProxy (okno 1)
python -m pipeline.drones_battle.arena_orchestrator --backend sitl   # okna 2 i 3
./pipeline/drones_battle/stop_arena.sh           # sprząta tylko procesy areny
```

`start_arena.sh` domyślnie otwiera każdy symulator w `xterm -hold`. Inny terminal: `ARENA_TERMINAL="xfce4-terminal --disable-server -x"`, bez okien (logi w `logs/`): `ARENA_TERMINAL=""`. Skrypt nigdy nie zabija terminali użytkownika (`stop_arena.sh` korzysta z pliku `.arena_pids`).

### Powtórka meczu

Każdy mecz jest zapisywany w `matches/<data>.jsonl`.

```bash
python -m pipeline.drones_battle.replay pipeline/drones_battle/matches/<plik>.jsonl
python -m pipeline.drones_battle.replay <plik>.jsonl --save walka.gif --step 5
```

---

## Zasady gry

| Zasada | Wartość domyślna |
|---|---|
| Drużyny | atakujący SysID 1, 2, 3 (północ), obrońcy SysID 4, 5, 6 (baza, południe), 150 m odstępu |
| Cel atakujących | punkt 10 m nad bazą obrońców; wygrana, gdy żywy atakujący zbliży się na < 5 m |
| Zestrzelenie | atakujący w odległości < 2 m od aktywnego obrońcy ginie (tryb `LAND` lub swobodny spadek) |
| Wygrana obrońców | wszyscy atakujący zestrzeleni albo minęło 120 s |
| Reguła anty-campingowa | obrońca w kuli 8 m wokół celu nie strzela i jest z niej wypychany |
| Prędkość | maks. 10 m/s (moduł wektora 3D), w pionie 3 m/s |
| Geofence | N ∈ [-40, 210] m, E ∈ [-60, 60] m, wysokość 3–40 m (miękkie ściany) |

**Kolejność rozstrzygania w jednym ticku:** najpierw zestrzelenia (wszystkie jednocześnie), potem dotarcie do celu tylko przez atakujących, którzy przeżyli, potem „wszyscy atakujący zestrzeleni”, na końcu limit czasu. Zestrzelenie i dotarcie w tym samym ticku liczy się więc na korzyść obrońców.

Wszystkie liczby są w `arena_config.toml`. Ustawienie `defender_exclusion_radius_m = 0` daje zasady dokładnie jak w pierwotnej specyfikacji.

---

## API dla studentów

Szablony: `team_attacker.py` i `team_defender.py`. Każdy zawiera jedną czystą funkcję:

```python
def compute_commands(my_team: dict, enemy_team: dict, target_pos: tuple, current_time: float) -> dict:
    ...
    return {drone_id: (v_north, v_east, v_down)}
```

* `my_team`, `enemy_team`: `{drone_id: {"pos": (N, E, D), "vel": (VN, VE, VD), "alive": bool}}`, metry i m/s.
* Układ **NED** względem bazy obrońców. **Uwaga: `v_down > 0` oznacza opadanie.**
* `target_pos`: `(N, E, D)` celu; dla obrońców to ich własna baza.
* `current_time`: sekundy od sygnału START.
* Brak klucza dla drona oznacza zawis. Komendy dla dronów przeciwnika lub martwych są ignorowane.
* Opcjonalnie piąty parametr `game: dict | None = None`. Arena przekazuje w nim `dt`, `time_left`, promienie, granice areny, `max_speed` własnej drużyny i `enemy_max_speed`.
* Wolno trzymać stan w zmiennych modułu (proces strategii żyje przez cały mecz i jest restartowany przed kolejnym).
* Pomocnicze funkcje wektorowe (w tym czas przechwycenia z `07_b_pursuit.py`): `strategy_utils.py`.

**Co się dzieje, gdy kod studenta zawiedzie:**

| Sytuacja | Reakcja areny |
|---|---|
| wyjątek | drony tej drużyny wiszą w tym ticku, traceback w konsoli |
| odpowiedź później niż 30 ms | używane są ostatnie poprawne komendy |
| brak odpowiedzi przez 3 s | restart procesu strategii; po 3 restartach walkower |
| `NaN`, `inf`, zły typ | zawis tego drona (licznik „invalid” w statystykach) |

Do debugowania z breakpointami (PyCharm): `--inline`, wtedy funkcja działa w tym samym procesie.

---

## Architektura

```text
pipeline/drones_battle/
├── arena_orchestrator.py   # CLI + klasa Match: pętla 10 Hz
├── arena_visualizer.py     # ArenaVisualizer (2 widoki 3D + opcjonalna mapa 2D), osobny proces
├── team_attacker.py        # szablon studenta
├── team_defender.py        # szablon studenta
├── strategy_utils.py       # wektory, przechwycenie predykcyjne
├── tournament.py           # każdy z każdym, tabela, strojenie balansu
├── replay.py               # odtwarzanie z logu, eksport GIF/MP4
├── start_arena.sh / stop_arena.sh / arena.parm / arena_config.toml
├── core/
│   ├── config.py           # ArenaConfig (dataclasses) + wczytywanie TOML
│   ├── layout.py           # pozycje startowe i porty (wspólne dla Pythona i Basha)
│   ├── referee.py          # ciągła detekcja kolizji, warunki zwycięstwa
│   ├── safety.py           # Safety Limiter: walidacja, geofence, limity prędkości
│   ├── sandbox.py          # strategie w osobnych procesach z budżetem czasu
│   ├── recorder.py         # zapis JSONL
│   ├── clock.py            # pętla o stałej częstotliwości, jitter
│   └── types.py, compat.py
├── backends/
│   ├── kinematic.py        # model punktu materialnego, bez SITL
│   └── sitl.py             # DroneKit + ArduPilot SITL
├── teams/                  # baseline, hunters, tricksters
└── tests/                  # pytest, bez SITL
```

Przeliczenia GPS ↔ NED są w `pipeline/math/geodesy.py` (bez zależności od DroneKit); z tej samej funkcji korzysta teraz `pipeline/math/physics.py`.

### Pętla jednego ticku

1. Odczyt telemetrii (SITL: własny listener `GLOBAL_POSITION_INT`, więc pozycja i prędkość pochodzą z tej samej wiadomości; dane starsze niż 1 s oznaczane jako nieaktualne).
2. Sędzia: zestrzelenia, cel, limit czasu. Zestrzelony dron dostaje jednorazowo `LAND`.
3. Obie strategie dostają ten sam stan jednocześnie (żadna nie widzi reakcji drugiej).
4. Safety Limiter, potem `SET_POSITION_TARGET_LOCAL_NED` (tylko prędkość) do każdego żywego drona, co tick, żeby nie zadziałał `GUID_TIMEOUT`.
5. Zapis do logu i wysłanie najnowszej klatki do procesu wizualizacji (bez czekania na rysowanie).

---

## Matematyka

### GPS → NED

Przybliżenie równoodległościowe względem bazy (`φ₀, λ₀, h₀`), `R = 6 378 137 m`:

$$N = R(\varphi-\varphi_0),\qquad E = R\cos\varphi_0\,(\lambda-\lambda_0),\qquad D = -(h-h_0).$$

Stały `cos φ₀` czyni przekształcenie liniowym i dokładnie odwracalnym. Na 200 m błąd względem odległości po kole wielkim jest poniżej 1 cm (test `test_geo.py`). Wysokość liczymy z AMSL, a nie z `relative_alt`, bo ta jest względna wobec domu każdego drona osobno.

### Ciągła detekcja kolizji

Przy 10 Hz i prędkości względnej 20 m/s drony zbliżają się o 2 m na tick, więc sprawdzanie odległości tylko w chwilach próbkowania gubi trafienia. Zakładamy ruch liniowy w ticku: `r(s) = r₀ + sΔ`, `s ∈ [0, 1]`, gdzie `r₀` to wektor względny na początku, a `Δ` zmiana wektora względnego. Minimum:

$$s^* = \mathrm{clamp}\!\left(-\frac{r_0\cdot\Delta}{|\Delta|^2},\,0,\,1\right),\qquad d_{\min} = |r_0 + s^*\Delta|.$$

Ten sam wzór sprawdza dotarcie do celu, więc szybki atakujący nie „przeskoczy” kuli 5 m.

### Safety Limiter

Miękka ściana: jeśli `d` to odległość od ściany (dodatnia wewnątrz), składowa prędkości w stronę ściany jest ograniczana do `k·d`, gdy `d < 5 m`. Za ścianą staje się to minimalną prędkością powrotu. Na koniec `v ← v · min(1, v_max/|v|)`, co skaluje cały wektor i zachowuje kierunek.

### Model kinematyczny

`v(t+dt) = v_cmd + (v − v_cmd)·e^(−dt/τ)`, zmiana prędkości ograniczona do `a_max·dt`. Parametry `τ = 0.6 s`, `a_max = 6 m/s²` są wartościami startowymi; należy je skalibrować na odpowiedzi skokowej SITL (dobre zadanie z identyfikacji systemów).

---

## Balans

Turniej z 10 rundami na parę (losowe przesunięcie startu do 3 m), przykładowe drużyny:

| Ustawienie | Wygrane atakujących |
|---|---:|
| domyślne (10 m/s obie strony, jak w specyfikacji) | 31% |
| `defender_max_speed_mps = 9` | 42% |
| `defender_max_speed_mps = 8` | 51% |
| `kill_radius_m = 1.5` | 44% |

Przy równych prędkościach obrońca stojący między atakującym a celem zawsze zdąży przechwycić (koło Apoloniusza degeneruje się do symetralnej). Jeśli studenci będą sfrustrowani po stronie ataku, najprostszą zmianą jest `defender_max_speed_mps = 9`:

```bash
python -m pipeline.drones_battle.tournament --rounds 10 --set safety.defender_max_speed_mps=9
```

---

## Testy

```bash
python -m pytest pipeline/drones_battle/tests
```

Testy nie wymagają SITL ani okien: geodezja, sędzia (w tym przelot „na wylot” między próbkami), limiter, piaskownica (wyjątek, nieskończona pętla, walkower), pełny mecz kinematyczny (deterministyczny dla ziarna) i turniej.

---

## Do sprawdzenia na SITL

Ten kod został przetestowany na backendzie kinematycznym. Na prawdziwym SITL trzeba jeszcze potwierdzić (faza 0 planu):

* nazwy parametrów z `arena.parm` w zainstalowanej wersji ArduPilota (`param show WPNAV*`, `GUID_TIMEOUT`),
* czy zbiorcza mapa MAVProxy z sześcioma `--master` pokazuje 6 osobnych pojazdów,
* tryb `connection_mode = "tcp"` (`--no-mavproxy`, mapa na porcie `5762 + 10·i`),
* obciążenie CPU przy 6 instancjach na komputerach w sali.
