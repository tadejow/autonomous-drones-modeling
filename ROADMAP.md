# Roadmapa Projektu: Kurs Programowania Trajektorii Lotu Dronów

Niniejszy dokument przedstawia plan przygotowania materiałów dydaktycznych, skryptów oraz środowiska testowego na potrzeby realizacji kursu. Projekt łączy zagadnienia teoretyczne (fizyka, matematyka, teoria sterowania) z praktyczną implementacją w języku Python z wykorzystaniem symulatora SITL, oprogramowania ArduPilot oraz biblioteki DroneKit.

---

## Faza 1: Przygotowanie środowiska i struktury repozytorium (SITL & Toolchain)
Celem tej fazy jest stworzenie powtarzalnego środowiska pracy dla prowadzącego i studentów, co zminimalizuje problemy techniczne na początku kursu.

*   **Zadanie 1.1: Konfiguracja ArduPilot SITL (Software-In-The-Loop)**
    *   Przygotowanie instrukcji instalacji SITL dla systemów Linux (zalecane) oraz Windows/macOS.
    *   Konfiguracja kontenera Docker (opcjonalnie, w celu ujednolicenia środowiska u wszystkich studentów).
*   **Zadanie 1.2: Integracja z DroneKit i MAVLink**
    *   Przygotowanie środowiska wirtualnego Python (venv/conda) z zainstalowaną biblioteką `dronekit` oraz niezbędnymi zależnościami (`pymavlink`, `numpy`, `matplotlib`).
    *   Przetestowanie podstawowego skryptu łączącego się z symulowanym dronem (odczyt podstawowych parametrów telemetrycznych).
*   **Zadanie 1.3: Konfiguracja wizualizacji (Mission Planner / QGroundControl)**
    *   Przygotowanie instrukcji integracji symulatora z oprogramowaniem do wizualizacji misji w czasie rzeczywistym.
*   **Zadanie 1.4: Inicjalizacja struktury repozytorium Git**
    *   Stworzenie logicznego podziału katalogów (np. `/teoria`, `/warsztaty`, `/notebooki`, `/zadania`).

---

## Faza 2: Opracowanie materiałów teoretycznych (Wykłady 1–5)
Faza ta obejmuje przygotowanie skryptu teoretycznego, stanowiącego podbudowę pod zadania programistyczne.

*   **Zadanie 2.1: Kinematyka i dynamika lotu BSP**
    *   Opracowanie opisu układów odniesienia (układ związany z ziemią NED, układ związany z ciałem drona).
    *   Wprowadzenie do sił i momentów działających na wielowypustowiec.
*   **Zadanie 2.2: Orientacja w przestrzeni**
    *   Przygotowanie materiałów dotyczących kątów Eulera (pitch, roll, yaw) oraz kwaternionów (konwersje i zastosowanie).
*   **Zadanie 2.3: Podstawy teorii sterowania i planowania trajektorii**
    *   Wprowadzenie do regulatorów PID w kontekście stabilizacji lotu.
    *   Algorytmiczne podejście do wyznaczania trajektorii (np. interpolacja wielomianowa, krzywe Béziera).

---

## Faza 3: Przygotowanie warsztatów podstawowych (Warsztaty 1–4)
Opracowanie pierwszych notatników Jupyter Notebook wprowadzających w praktyczne programowanie z DroneKit.

*   **Zadanie 3.1: Wprowadzenie do API DroneKit (Warsztat 1)**
    *   Tworzenie połączenia, zmiana trybów lotu (GUIDED, AUTO, LAND).
    *   Obsługa procedury uzbrajania silników (Arming) i bezpiecznego startu (Takeoff).
*   **Zadanie 3.2: Sterowanie pozycją i prędkością (Warsztat 2-3)**
    *   Implementacja wysyłania komend ruchu przy użyciu wiadomości MAVLink (`SET_POSITION_TARGET_LOCAL_NED`).
    *   Ruch drona w układzie lokalnym i globalnym.
*   **Zadanie 3.3: Odczyt i analiza telemetrii (Warsztat 4)**
    *   Monitorowanie stanu drona (bateria, GPS, prędkość, orientacja) i asynchroniczna obsługa zdarzeń (listeners).

---

## Faza 4: Przygotowanie warsztatów zaawansowanych (Warsztaty 5–10)
Implementacja złożonych scenariuszy zgodnych z założeniami projektu.

*   **Zadanie 4.1: Algorytmiczne planowanie trajektorii (Warsztat 5-6)**
    *   Implementacja algorytmów generowania gładkich ścieżek omijających przeszkody lub realizujących zadany kształt (np. ósemka, spirala).
*   **Zadanie 4.2: Śledzenie obiektów na ziemi w czasie rzeczywistym (Warsztat 7)**
    *   Symulacja ruchomego celu na ziemi i implementacja algorytmu nadążnego dla drona.
*   **Zadanie 4.3: Optymalizacja zadań logistycznych (Warsztat 8-9)**
    *   Rozwiązywanie problemu komiwojażera (TSP) lub problemu marszrutyzacji (VRP) w kontekście planowania misji patrolowej/dostawczej dla drona.
*   **Zadanie 4.4: Wprowadzenie do roju dronów (Warsztat 10)**
    *   Uruchomienie wielu instancji SITL jednocześnie.
    *   Podstawowa koordynacja lotu i unikanie kolizji między jednostkami (proste algorytmy rozproszone).

---

## Faza 5: Prace końcowe, weryfikacja i dokumentacja
Prace przed rozpoczęciem semestru zimowego 2026.

*   **Zadanie 5.1: Przygotowanie list zadań domowych oraz projektów zaliczeniowych**
    *   Opracowanie kryteriów oceny i szablonów kodu (z brakującymi fragmentami do uzupełnienia przez studentów).
*   **Zadanie 5.2: Testowanie materiałów (Dry Run)**
    *   Przejście przez wszystkie przygotowane notatniki Jupyter od zera na czystej instalacji systemu w celu wykrycia ewentualnych błędów w kodzie lub nieaktualnych bibliotekach.
*   **Zadanie 5.3: Dokumentacja końcowa**
    *   Stworzenie przejrzystego pliku `README.md` w repozytorium z wymaganiami technicznymi i sylabusem zajęć.

---

## Proponowana struktura repozytorium Git

```text
├── .gitignore
├── README.md                  # Instrukcja uruchomienia, wymagania, sylabus
├── requirements.txt           # Zależności Pythona
├── docker-compose.yml         # (Opcjonalnie) Kontener z gotowym środowiskiem SITL
│
├── theory/                    # Materiały teoretyczne (PDF/Markdown)
│   ├── wykład_01_kinematyka.md
│   ├── wykład_02_orientacja.md
│   └── ...
│
├── notebooks/                 # Jupyter Notebooks na zajęcia warsztatowe
│   ├── 01_intro_sitl.ipynb
│   ├── 03_trajectory_planning.ipynb
```│   ├── ex_01_takeoff/
│   ├── ex_02_circle_flight/
│   └── ...
│
└── utils/                     # Pomocnicze skrypty (np. do uruchamiania wielu SITL)
    └── start_multi_sitl.sh
│   └── ...
│
├── exercises/                 # Szablony zadań domowych dla studentów

