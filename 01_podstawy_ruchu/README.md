# Moduł 1: Podstawy Ruchu i Komunikacji z Autopilotem

Niniejszy moduł stanowi wprowadzenie do programowania bezzałogowych statków powietrznych (BSP). Zawiera podstawowe mechanizmy łączenia skryptów w języku Python z symulatorem fizyki lotu (SITL) oraz tłumaczy fundamentalne pojęcia nawigacji.

Zgodnie z dobrymi praktykami inżynierii oprogramowania (ISO/PEP 8), kody źródłowe w tym repozytorium są napisane w języku angielskim. Poniżej znajduje się ich analiza merytoryczna.

## Słowniczek pojęć (Glossary)

Dla osób ze środowiska matematycznego/informatycznego, które pierwszy raz stykają się z lotnictwem:

* **SITL (Software-In-The-Loop):** Symulator, w którym "mózg" drona (oprogramowanie C++) działa na komputerze stacjonarnym zamiast na fizycznym mikrokontrolerze, korzystając z wirtualnego silnika fizycznego.
* **MAVLink:** Binarny protokół komunikacyjny służący do wysyłania komend i odbierania telemetrii z drona.
* **UDP (User Datagram Protocol):** Protokół sieciowy używany tu do komunikacji. Symulator wysyła dane na port (np. `14550`), a nasz skrypt w Pythonie ich nasłuchuje.
* **Arming (Uzbrojenie):** Krytyczna procedura bezpieczeństwa. Dopóki silniki nie zostaną "uzbrojone", dron odrzuci wszelkie komendy ruchu.
* **GUIDED Mode:** Tryb lotu, w którym wbudowany autopilot utrzymuje drona stabilnie w powietrzu, ale czeka na wektory prędkości lub współrzędne z naszego skryptu w Pythonie.
* **RTL (Return To Launch):** Tryb powrotu do bazy. Autopilot ignoruje skrypt, wznosi się na bezpieczną wysokość i ląduje w miejscu, w którym został włączony.
* **NED (North, East, Down):** Prawoskrętny kartezjański układ współrzędnych używany w lotnictwie. Oś X to Północ, oś Y to Wschód, a oś Z skierowana jest w **dół** (Dlatego wysokość podaje się często jako wartość ujemną w układzie NED).

---

## Analiza Kodu

### Skrypt 1: `01_start.py` (Procedura startowa)

Skrypt ten obrazuje koncepcję asynchronicznego oczekiwania na spełnienie warunków fizycznych. 

1. **Inicjalizacja (`vehicle.is_armable`):** W prawdziwym świecie drony posiadają filtry Kalmana (EKF), które muszą ustabilizować się przed lotem (połączenie danych z akcelerometrów, żyroskopów i GPS). Pętla asynchronicznie odpytuje drona o status gotowości.
2. **Pętla nadążna wysokości:** Po wysłaniu komendy `simple_takeoff(10.0)`, skrypt nie może po prostu "iść dalej". Musi w pętli odpytywać barometr/GPS o aktualną wysokość (`current_altitude`) i porównywać ją z zadaną. Próg akceptacji (zbieżności) ustawiono na 95%, co zapobiega nieskończonemu działaniu pętli z powodu szumu czujników.

### Skrypt 2: `02_trajectory.py` (Sterowanie pozycyjne)

Ten skrypt wprowadza matematykę sferyczną do planowania trajektorii.

1. **Aproksymacja sferyczna (`get_location_metres`):**
   Dron oczekuje komend w postaci długości i szerokości geograficznej (stopnie), jednak dla algorytmów łatwiej operuje się na metrach. Przesunięcie o 1 metr na Północ zawsze zmienia szerokość geograficzną o stałą wartość (wynikającą z obwodu Ziemi). 
   *Niuns matematyczny:* Przesunięcie o 1 metr na Wschód jest zmienne! Zależy od tego, jak blisko równika się znajdujemy. Dlatego we wzorze na długość geograficzną pojawia się poprawka kosinusowa: `math.cos(latitude)`.
2. **Uproszczony wzór Haversine'a (`get_distance_metres`):**
   Używamy twierdzenia Pitagorasa z poprawką skalującą $1.113195 \cdot 10^5$ (ilość metrów w jednym stopniu geograficznym), aby szybko obliczać błąd odległości do celu.
3. **Control Loop (Pętla sterowania):**
   Wykorzystano model listy punktów trasy (`trajectory_points`). Skrypt nakazuje dronowi lot do punktu (komenda wysłana raz) i wchodzi w pętlę sprawdzającą kryterium zbieżności ($d \le 1.5$ metra). Zastosowanie tolerancji 1.5 m kompensuje inercję bryły sztywnej (dron nie może zatrzymać się w miejscu o grubości 0 mm).

## Jak uruchomić?
1. Włącz symulator MAVProxy w osobnym terminalu:
   ```bash
   cd ~/ardupilot/ArduCopter
   sim_vehicle.py --console --map
