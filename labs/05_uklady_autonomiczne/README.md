# Moduł 5: Układy Autonomiczne i Systemy Wieloagentowe (Swarm Robotics)

Ostatni moduł kursu stanowi wstęp do robotyki roju. Drony nie realizują tu wgranej misji punkt po punkcie, lecz na bieżąco analizują pozycje swoich sąsiadów i dostosowują swoje parametry lotu wykorzystując modele zaczerpnięte z natury i fizyki matematycznej.

Zgodnie z dobrymi praktykami inżynierii oprogramowania (ISO/PEP 8), kody źródłowe w tym repozytorium są napisane w języku angielskim.

## Słowniczek pojęć (Glossary)

* **Swarm Robotics:** Dziedzina robotyki, w której duża liczba stosunkowo prostych robotów współpracuje, aby zrealizować skomplikowane zadanie. Bazuje na zachowaniach emergentnych (np. klucz ptaków).
* **Cucker-Smale Model:** Klasyczny model matematyczny używany w fizyce do opisu zjawiska stada (flocking). Opiera się na prawie, w którym każdy agent dostosowuje swój wektor prędkości proporcjonalnie do różnicy prędkości swoich sąsiadów. Skutkuje to zbieżnością prędkości całego stada do wspólnego wektora.
* **Cohesion (Kohezja):** Siła przyciągania zmuszająca agentów do pozostawania w grupie i unikania fragmentacji stada.
* **Repulsion (Repulsja):** Siła odpychania (antykolizyjna), rosnąca wykładniczo wraz ze zmniejszaniem się dystansu. Zapobiega zderzeniom dronów. Wzorowana na prawie Coulomba.
* **Multi-threading / Multi-port:** Łączenie się z wieloma symulatorami jednocześnie, wykorzystując wiele portów MAVLink (np. `14550`, `14560`).

---

## Analiza Kodu

### Skrypt 1: `05_a_cucker_smale_model.py` (Fundament Flocking'u)

Skrypt uruchamia 3 drony pokazując zjawisko synchronizacji w przestrzeni 3D.
1. **Regulator PD w 3 wymiarach:** 
   Oś $Z$ nie jest sztucznie blokowana na danej wysokości. Lider dąży do wysokości $15\text{m}$, a skrzydłowi podążają za nim za pomocą przestrzennego wektora Kohezji. Jeśli znajdą się zbyt blisko, siła Repulsji może wypchnąć jednego z nich w górę a drugiego w dół, tworząc zachowanie przypominające zachowanie roju owadów.
2. **Synchronizacja Czasowa:** 
   Osiągnięto całkowitą izolację fizyki od wydajności procesora poprzez mierzenie lokalnej różnicy czasu `dt`. Każda wyliczona siła mnożona jest przez rzeczywiste `dt`, co czyni wektory przyspieszeń odpornymi na zawieszenia silnika graficznego.

### Skrypt 2: `05_b_flock_split.py` (Zachowania Emergentne 8 Dronów)

Skrypt implementuje scenariusz podziału stada i analizy danych w czasie rzeczywistym.
1. **Pasywne Rozdzielenie:** 
   Skrzydłowi nie mają zdefiniowane w kodzie przypisania do konkretnego Lidera. Używają metody wyszukania najbliższego atraktora: `closest_leader = X[0] if d_to_l1 < d_to_l2 else X[1]`.
2. **Inteligentna Analityka Grafu Par:** 
   Obliczenie 28 odległości między 8 dronami skutkowałoby nałożeniem 28 krzywych na prawy wykres Matplotlib. Zastosowano filtrację historii: wyświetlane (półprzezroczyście) są wszystkie linie, jednak dynamiczne zacienienie pozwala ukryć relacje wewnątrz strefy bezpiecznej. Jedynie pary znajdujące się poniżej bariery bezpieczeństwa `D_SAFE` wyzwalają kolorowy alert na żywo.
