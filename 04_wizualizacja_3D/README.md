# Moduł 4: Wizualizacja 3D i Geometria Obliczeniowa

Niniejszy moduł prezentuje zaawansowane techniki łączenia analityki przestrzennej w języku Python z systemami nawigacyjnymi drona. Kody skupiają się na dynamicznym wizualizowaniu przestrzeni (przy użyciu `matplotlib`) oraz rozwiązywaniu złożonych równań na żywo (real-time physics).

Zgodnie z dobrymi praktykami inżynierii oprogramowania (ISO/PEP 8), kody źródłowe w tym repozytorium są napisane w języku angielskim.

## Słowniczek pojęć (Glossary)

* **Sweep-Line Algorithm (Algorytm Zamiatania):** Metoda w geometrii obliczeniowej, w której wirtualna linia "przesuwa się" po przestrzeni, przecinając się z krawędziami wielokątów. Używana tutaj do generowania optymalnych ścieżek cięcia (kosiarki) wewnątrz nieregularnych obszarów.
* **Bounding Box:** Najmniejszy prostokąt równoległy do osi układu współrzędnych, który całkowicie zamyka w sobie dany kształt geometryczny. Wykorzystywany do optymalizacji pętli przeszukujących.
* **ADS-B (Automatic Dependent Surveillance–Broadcast):** System lotniczy, w którym statki powietrzne automatycznie nadają swoją pozycję za pośrednictwem fal radiowych. W tym module używany jest do oszukiwania radarów (Spoofing MAVLink).
* **Predictive Intercept (ProNav):** Algorytmy naprowadzania stosowane m.in. w systemach przeciwrakietowych. W przeciwieństwie do algorytmów naśladujących (Pure Pursuit), algorytmy predykcyjne celują w wyliczony w przyszłości punkt spotkania (Lead Pursuit).
* **Geodetic vs NED:** Transformacje między układem geodezyjnym (szerokość/długość geograficzna z GPS) a lokalnym układem kartezjańskim NED (w metrach).

---

## Analiza Kodu

### Skrypt 1: `07_a_polygon_scan.py` (Zamiatanie Obszaru Nieregularnego)

Skrypt rozwiązuje problem generowania misji dla obszarów niebędących prostokątami (np. działy rolne wyznaczone na mapie).

1. **Równanie Prostej i Intersekcje:**
   Dla każdej poziomej linii $N$ sprawdzane jest przecięcie z krawędziami wielokąta. Wykorzystano równanie prostej: 
   $E = E_1 + (E_2 - E_1) \cdot \frac{N - N_1}{N_2 - N_1}$
2. **Optymalizacja Nawrotów:**
   Zależnie od wartości zmiennej `direction`, algorytm wpisuje wyliczone punkty przecięć do autopilota od lewej do prawej, a następnie w kolejnym pasku skanowania – od prawej do lewej. Redukuje to straty energetyczne związane ze zbędnym powracaniem na lewą krawędź pola.

### Skrypt 2: `07_b_pursuit.py` (Naprowadzanie Predykcyjne i Radar)

Implementacja lotu bojowego lub systemu przechwytującego (np. drony anty-dronowe).

1. **Synchronizacja Czasu Rzeczywistego (`dt`):**
   Użycie `time.time()` gwarantuje, że pętla symulacyjna i obliczeniowa są zsynchronizowane ze światem rzeczywistym. Symulator odrysowuje wykres w miarę możliwości procesora, a fizyka wrogiego obiektu jest skalowana zgodnie z faktycznym upływem czasu (chroni to przed "zagięciem czasu", w którym uciekający cel nagle drastycznie przyspiesza przy braku limitów klatkażu).
2. **Matematyka Przechwycenia:**
   Kluczowym elementem jest wyliczenie wektora wyprzedzającego. Równanie to w postaci kwadratowej: 
   $A t^2 + B t + C = 0$, gdzie $A = V_{drona}^2 - V_{celu}^2$.
   Z wykorzystaniem wyróżnika ($\Delta$) znajdujemy dodatni czas $t$, po którym nastąpi kolizja, a następnie wektor wynikowy przesyłany jest do drona jako polecenie prędkosciowe komendą `send_ned_velocity`.
3. **Spoofing ADS-B:**
   Funkcja `spoof_adsb_target` formuje pakiety w surowym standardzie binarnym i wysyła je do MAVProxy, oszukując aplikację naziemną o istnieniu balonu w przestrzeni powietrznej. Ważny jest parametr `squawk=1200` określający tryb lotu z widocznością (VFR).

### Skrypt 3: `07_c_cylinder_scan.py` (Inspekcja ze zmianą referencji GPS)

Prawdziwa praca geodezyjna, łącząca układy współrzędnych.

1. **Transformacja GPS -> Metry (`get_distance_ned`):**
   Odwrotność funkcji używanej na poprzednich zajęciach. Po odczytaniu przez operatora współrzędnych geograficznych interesującego go budynku, algorytm wylicza wektor metryczny między bazą startową drona a celem, na podstawie którego orientuje grafikę `matplotlib`.
2. **Przesunięcie Względne Orbit:**
   Wszystkie współrzędne orbity wyliczane są najpierw względem środka budynku ($x = R\cdot \cos(\alpha)$), a następnie dodawane do macierzy bazowej `TARGET_LAT`/`TARGET_LON`. Gwarantuje to perfekcyjne naniesienie ścieżki okręgu na absolutne pozycje fizyczne kuli ziemskiej.

## Jak to uruchomić?
Aby w pełni docenić te skrypty, wymagane jest jednoczesne śledzenie mapy MAVProxy oraz okna 3D generowanego przez Matplotlib:
1. Uruchom symulator: `sim_vehicle.py --console --map`
2. Włącz radar: `param set ADSB_TYPE 1` -> wciśnij Enter -> wpisz `reboot`
3. W środowisku wirtualnym: `python 07_b_pursuit.py`
