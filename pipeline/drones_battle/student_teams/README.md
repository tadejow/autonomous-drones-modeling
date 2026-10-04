# Katalogi drużyn studentów

Tu rozpakowujemy zgłoszenia studentów do „Walk Dronów”. Każda drużyna to **jeden katalog nazwany numerem indeksu** (np. `300538/`). Arena (`arena_gui.py`, `tournament.py --submissions`) wczytuje automatycznie wszystkie katalogi z tego miejsca, z wyjątkiem tych, których nazwa zaczyna się od `_` lub `.` (np. `_szablon/`).

Zawartość tego katalogu (poza tym plikiem i `_szablon/`) jest w `.gitignore`: kod studentów nie trafia do repozytorium.

---

## Co musi zawierać zgłoszenie

```text
300538/                 ← numer indeksu (dla zespołu: indeks osoby wysyłającej)
├── attacker.py         ← WYMAGANY: strategia, gdy drużyna atakuje
├── defender.py         ← WYMAGANY: strategia, gdy drużyna broni
├── team.toml           ← opcjonalny: nazwa drużyny i autorzy (widoczne w GUI)
└── *.py                ← opcjonalne moduły pomocnicze, np. geometry.py
```

Najprościej zacząć od kopii `_szablon/`:

```bash
cp -r pipeline/drones_battle/student_teams/_szablon pipeline/drones_battle/student_teams/300538
```

### `attacker.py` i `defender.py`

Każdy z tych plików musi definiować funkcję `compute_commands` z dokładnie taką sygnaturą:

```python
def compute_commands(my_team: dict, enemy_team: dict, target_pos: tuple, current_time: float) -> dict:
    return {drone_id: (v_north, v_east, v_down), ...}
```

| Argument | Znaczenie |
|---|---|
| `my_team` | `{drone_id: {"pos": (N, E, D), "vel": (VN, VE, VD), "alive": bool}}`: Twoje drony |
| `enemy_team` | to samo dla drużyny przeciwnej |
| `target_pos` | `(N, E, D)` celu: baza obrońców 10 m nad ziemią (dla obrońców to ich własna baza) |
| `current_time` | sekundy od startu walki |
| wynik | prędkości w m/s dla Twoich dronów; brak drona w wyniku = dron wisi w miejscu |

Opcjonalnie funkcja może mieć piąty parametr `game: dict | None = None`; arena poda w nim m.in. `max_speed`, `time_left`, `kill_radius`, `target_radius`, `defender_exclusion_radius` i granice areny (`bounds`).

### `team.toml` (opcjonalny)

```toml
name = "Sokoły"
authors = ["Anna Nowak (300538)", "Jan Kowalski (300539)"]
```

### Moduły pomocnicze (opcjonalne)

Kod można podzielić na kilka plików w katalogu drużyny i importować je zwykłym `import geometry` **na górze** `attacker.py` / `defender.py`. Dwie drużyny mogą mieć pliki o tej samej nazwie; arena ich nie pomiesza.

---

## Zasady techniczne

1. **Układ NED, metry, m/s.** Początek układu to baza obrońców. `D` rośnie w dół: **`v_down > 0` oznacza opadanie**.
2. **Nie zakładaj liczby dronów ani ich numerów.** Ten sam kod musi działać w trybie 3 vs 3 (atak: 1–3, obrona: 4–6) i 5 vs 5 (atak: 1–5, obrona: 6–10). Iteruj po `my_team`.
3. **Czas:** funkcja jest wywoływana 10 razy na sekundę i ma **30 ms** na odpowiedź. Spóźniona odpowiedź = drony lecą według poprzednich komend; zawieszenie na 3 s = restart strategii; trzy restarty = walkower.
4. **Prędkość** jest przycinana do 10 m/s (w pionie 3 m/s), a drony nie mogą wylecieć poza arenę ani zejść poniżej 3 m (miękkie ściany).
5. **Stan między wywołaniami** wolno trzymać w zmiennych globalnych modułu; przed każdym meczem moduł jest wczytywany od nowa.
6. **Dozwolone biblioteki:** biblioteka standardowa Pythona i `numpy`. **Niedozwolone:** `dronekit`, `pymavlink`, sieć, procesy, wątki, czytanie i zapisywanie plików, `input()`, `print` w każdym kroku (spowalnia).
7. **Wyjątek** w funkcji nie przerywa meczu, ale Twoje drony w tym kroku wiszą w miejscu.
8. **Zasady gry:** obrońcy są dronami kamikaze (zderzenie < 2 m niszczy oba drony), atakujący wygrywa, gdy zbliży się do celu na < 5 m, obrońca w kuli 8 m wokół celu nie może zestrzeliwać, mecz trwa maks. 120 s. Pełny opis: `pipeline/drones_battle/README.md`.

---

## Jak sprawdzić zgłoszenie przed wysłaniem

Z katalogu głównego repozytorium:

```bash
# Walidacja: czy pliki istnieją, czy się importują, czy zwracają poprawne komendy w 3 vs 3 i 5 vs 5, ile trwają
python -m pipeline.drones_battle.core.submissions sciezka/do/300538

# Mecz próbny bez symulatora (Twój atak kontra przykładowa obrona), z podglądem 3D
python -m pipeline.drones_battle.arena_orchestrator --backend kinematic \
    --attacker sciezka/do/300538/attacker.py --defender pipeline.drones_battle.teams.hunters.defender
```

## Jak wysłać

Spakuj **zawartość** katalogu (albo cały katalog) do pliku `300538.zip`. Prowadzący wrzuca ZIP-y do tego katalogu; arena rozpakowuje je sama do `300538/` przy odświeżeniu listy drużyn (oba układy archiwum są akceptowane).

---

## Dla prowadzącego

1. Wrzuć ZIP-y (albo rozpakowane katalogi) do `pipeline/drones_battle/student_teams/`.
2. Uruchom GUI: `python -m pipeline.drones_battle.arena_gui`. Każda drużyna dostaje status (OK / ostrzeżenia / BŁĄD z opisem).
3. Wybierz dwie drużyny i kliknij **Walka**, albo zaznacz kilka i kliknij **Turniej**.

Bez GUI: `python -m pipeline.drones_battle.tournament --submissions --rounds 2`.
