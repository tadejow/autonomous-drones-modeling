# Moduł 2: Fizyka i Matematyka w Planowaniu Trajektorii

Ten moduł łączy zagadnienia analizy matematycznej (równania parametryczne) z surową fizyką symulatora SITL (aerodynamika i dynamika bryły sztywnej). Umożliwia studentom bezpośrednie zaobserwowanie, jak abstrakcyjne wzory matematyczne przekładają się na ruch fizycznej maszyny w przestrzeni narażonej na siły zewnętrzne.

Zgodnie z dobrymi praktykami inżynierii oprogramowania (ISO/PEP 8), kody źródłowe w tym repozytorium są napisane w języku angielskim.

## Słowniczek pojęć (Glossary)

* **Równania Parametryczne:** Zbiór równań, w których współrzędne punktu (np. X i Y) wyrażone są za pomocą zmiennej pomocniczej (parametru), zazwyczaj oznaczanej jako $t$. Idealne do opisywania krzywych ciągłych (np. okręgów).
* **Pitch, Roll, Yaw (Kąty Eulera):** 
  * *Pitch (Pochylenie):* Obrót wokół osi poprzecznej (dziób w górę/dół). Odpowiada za ruch do przodu/tyłu.
  * *Roll (Przechył):* Obrót wokół osi podłużnej (przechył na lewe/prawe skrzydło). Odpowiada za ruch na boki.
  * *Yaw (Odchylenie):* Obrót wokół osi pionowej. Odpowiada za obrót w miejscu (zmiana kierunku kompasu).
* **EKF (Extended Kalman Filter):** Zaawansowany algorytm matematyczny działający w tle kontrolera lotu. Estymuje on faktyczną pozycję oraz siłę i kierunek wiatru, łącząc zaszumione dane z GPS, akcelerometru i żyroskopu.

---

## Analiza Kodu

### Skrypt 1: `03_parametric_shapes.py` (Krzywe parametryczne)

Skrypt prezentuje przejście od dyskretnych komend nawigacyjnych do płynnego planowania ścieżki ciągłej.

1. **Generatory matematyczne:**
   Zaimplementowano trzy funkcje: `generate_circle`, `generate_infinity` (Lemniskata Gerono) oraz `generate_heart` (Kardioida). Każda z nich generuje listę tupli z przesunięciami kartezjańskimi, korzystając z pętli po parametrze $t \in [0, 2\pi]$. Użycie funkcji trygonometrycznych pozwala na idealne skalowanie i wygładzanie kształtów.
2. **Smooth Cornering (Pętla Nadążna z dużą tolerancją):**
   W przeciwieństwie do Skryptu `02_trajectory`, gdzie tolerancja błędu wynosiła 1.5 m a dron musiał się zatrzymać (Hover), tutaj tolerancja została zwiększona do 2.0 m, a opóźnienie w pętli `time.sleep()` skrócone do 0.1s. 
   **Efekt inżynierski:** Skrypt wysyła komendę lotu do kolejnego punktu, zanim dron zdąży wyhamować przy obecnym. Powoduje to, że algorytmy kontrolera lotu drona same "ścinają" wirtualne zakręty, tworząc idealnie gładką krzywą bez utraty pędu statku.

### Skrypt 2: `04_physics_wind.py` (Fizyka i Reakcja na Wiatr)

Skrypt pozwala odczytać surowe dane z silnika fizyki środowiska SITL.

1. **Podsłuchiwanie protokołu (`@vehicle.on_message('WIND')`):**
   Biblioteka DroneKit udostępnia podstawowe zmienne (np. wysokość). Wiatr wymaga jednak "zhakowania" strumienia danych. Użyto wbudowanego w bibliotekę dekoratora. Funkcja `wind_listener` przechwytuje pakiety MAVLink o identyfikatorze `WIND` przelatujące przez sieć w tle i zapisuje ich wartości (prędkość i kąt) jako nowe, dynamicznie utworzone atrybuty obiektu `vehicle`.
2. **Korelacja Fizyczna (I zasada dynamiki Newtona):**
   Podczas eksperymentu można zaobserwować zjawisko fizyczne: jeśli ustawimy w konsoli symulatora siłę wiatru na 10 m/s, dron natychmiast zareaguje pochyleniem (*Pitch* lub *Roll*) na np. 25°. Wygenerowany wektor ciągu silników przeciwstawia się wektorowi wiatru, dzięki czemu odczytywana całkowita prędkość pozioma (*Velocity*) wynosi niemal 0.0 m/s, a dron utrzymuje stałą pozycję GPS pomimo wichury.

## Jak przeprowadzić eksperyment z wiatrem?
1. Uruchom symulator: `sim_vehicle.py --console --map`
2. Uruchom skrypt: `python 04_physics_wind.py`
3. Gdy skrypt zamelduje, że dron wisi w powietrzu, wpisz w konsoli MAVProxy (`MAV>`):
   ```bash
   param set SIM_WIND_SPD 15
   param set SIM_WIND_DIR 90
