# Moduł 3: Misje Autonomiczne i Algorytmy Pokrycia

W tym module przechodzimy od ciągłego, asynchronicznego sterowania dronem ze skryptu, do wgrywania gotowych, bezstanowych misji bezpośrednio do pamięci EEPROM autopilota. 

Zgodnie z dobrymi praktykami inżynierii oprogramowania (ISO/PEP 8), kody źródłowe w tym repozytorium są napisane w języku angielskim.

## Słowniczek pojęć (Glossary)

* **Waypoint (Punkt trasy):** Zestaw współrzędnych 3D (Szerokość, Długość, Wysokość), do których dron ma polecieć.
* **AUTO Mode:** Tryb pracy kontrolera lotu, w którym dron ignoruje zewnętrzne komendy sterujące z komputera, a zamiast tego samodzielnie realizuje listę Waypointów wgraną wcześniej do jego pamięci.
* **MAV_CMD_NAV_WAYPOINT:** Komenda protokołu MAVLink informująca drona, że dany punkt na liście jest standardowym punktem nawigacyjnym.
* **Swath Width:** W fotogrametrii – szerokość paska ziemi "widzianego" przez kamerę drona. Wykorzystywana do obliczania dystansu między sąsiednimi liniami lotu w algorytmach skanowania.
* **TSP (Traveling Salesperson Problem - Problem Komiwojażera):** Klasyczny problem optymalizacyjny z teorii grafów, polegający na znalezieniu najkrótszej możliwej trasy łączącej zestaw zadanych punktów.
* **Greedy Algorithm (Algorytm Zachłanny):** Algorytm heurystyczny, który w każdym kroku podejmuje lokalnie optymalną decyzję (np. leć do najbliższego punktu), nie analizując konsekwencji globalnych.

---

## Analiza Kodu

### Skrypt 1: `05_scan_area.py` (Algorytm Boustrophedon / "Kosiarka")

Skrypt demonstruje sposób układania misji dla dronów rolniczych i geodezyjnych.

1. **Obiekt `Command` w MAVLink:**
   Tworzenie punktu trasy w kodzie polega na wygenerowaniu obiektu `Command`. Kluczowe argumenty to `frame` (układ odniesienia - używamy `MAV_FRAME_GLOBAL_RELATIVE_ALT`, aby wysokość dotyczyła ziemi, z której wystartowaliśmy) oraz parametr `autocontinue=1`. Przekazanie `1` informuje drona, aby po osiągnięciu punktu nie czekał na pozwolenie, lecz od razu leciał do następnego.
2. **Logika Zygzaka (Sweep-line):**
   Algorytm przesuwa się po osi *East* dodając do siebie stałą wartość `SWATH_WIDTH`. Oś *North* jest naprzemiennie ustawiana na `0` oraz na maksymalną długość pola (`FIELD_LENGTH`) przy użyciu zmiennej `direction`, która w każdej iteracji jest mnożona przez `-1`.
3. **Praktyka Inżynierska Startu (GUIDED $\rightarrow$ AUTO):**
   Ze względów bezpieczeństwa oprogramowanie ArduCopter potrafi zablokować start bezpośrednio w trybie `AUTO`, jeśli uzna, że pierwszy waypoint wymusza niebezpieczny kąt ataku. Skrypt podnosi drona w trybie `GUIDED` (start wertykalny) i powierza mu samodzielną misję (`vehicle.mode = VehicleMode("AUTO")`) dopiero na bezpiecznej wysokości 20 metrów.

### Skrypt 2: `06_tsp_mission.py` (Dron Kurierski - Logistyka)

Skrypt łączy generowanie losowego grafu z podstawowymi rozwiązaniami logistycznymi dla flot dostawczych.

1. **Model problemu (Random Graph):**
   Punkty dostaw generowane są z wykorzystaniem rozkładu jednorodnego `random.uniform()` w promieniu operacyjnym 100 metrów wokół punktu startu.
2. **Heurystyka "Najbliższego Sąsiada":**
   W funkcji `solve_tsp_nearest_neighbor` implementujemy najprostszą formę rozwiązania problemu TSP. W każdym kroku pętli `while`, algorytm przeszukuje listę nieodwiedzonych punktów (`unvisited`) szukając tego, dla którego funkcja `distance_2d` zwraca najmniejszą wartość. Jest to doskonały przykład zastosowania w Pythonie wyrażenia `lambda`: `min(unvisited, key=lambda p: distance_2d(current_point, p))`.
3. **Analiza zbieżności i wydajności:**
   Choć algorytm zachłanny nie gwarantuje znalezienia globalnie najkrótszej trasy dla bardzo złożonych grafów, dla zbioru 8-15 punktów działa w czasie niemal $O(n^2)$ i dostarcza trasę wystarczająco optymalną, zapobiegając szybkiemu wyczerpaniu baterii statku.

## Jak uruchomić i obserwować?
Aby zobaczyć misję, po uruchomieniu symulatora i odpaleniu skryptu w `venv`, należy w konsoli symulatora (`MAV>`) wpisać komendę:
```bash
wp list
