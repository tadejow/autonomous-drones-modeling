# Scenariusz Wykładu: Matematyczne podstawy modelowania trajektorii lotu dronów

**Czas trwania:** ok. 45 minut wykładu + 5 minut na sesję Q&A
**Grupa docelowa:** Młodzież licealna (Dolnośląski Kongres Młodego Odkrywcy)
**Materiały:** Slajdy (LaTeX/Beamer), Filmy/Animacje symulacji z katalogów 01-05.

---

## 1. Wstęp (5 minut)

**Slajd:** Kiedy myślimy o dronach...
**Mowa:**
Cześć wszystkim! Nazywam się Maciej Tadej i dzisiaj zabiorę Was w podróż za kulisy nowoczesnej inżynierii. Wyobraźcie sobie drona. Prawdopodobnie widzicie przed oczami operatora, który z wypiekami na twarzy pociąga za drążki kontrolera, żeby ominąć drzewo. Ale w nowoczesnym świecie – w logistyce, ratownictwie, czy wojsku – nikt nie ma czasu ani możliwości, żeby sterować każdym dronem ręcznie z użyciem pada.
Wtedy do akcji wkracza autonomia.

**Slajd:** Czym jest autonomia?
**Mowa:**
Ale co to znaczy, że maszyna jest autonomiczna? Oznacza to, że sama musi sobie odpowiedzieć na kilka trudnych pytań. Gdzie jestem? Gdzie chcę lecieć? Jak ominąć tę wysoką wieżę? 
Okazuje się, że dron nie ma "mózgu", który myśli jak człowiek. Dron "myśli" matematyką. Narzędzia, o których uczycie się na lekcjach – układy współrzędnych, wektory, funkcje – to właśnie jego zmysły i logika. Zobaczmy, jak to wygląda pod maską!

---

## 2. Podstawy ruchu i przestrzeń (10 minut)

**Slajd:** Gdzie jestem? Układ współrzędnych w praktyce
**Mowa:**
Zacznijmy od podstaw. Jak dron odnajduje się w przestrzeni? Większość z Was zna układ współrzędnych z zeszytu w kratkę: osie X i Y. Dron porusza się w 3D, więc dodajemy oś Z, oznaczającą wysokość. Sensory takie jak moduł GPS i specjalne żyroskopy ciągle dostarczają komputerowi pokładowemu informacje o tym, w jakim punkcie (x, y, z) maszyna się znajduje i pod jakim kątem jest pochylona.

**Slajd:** Start drona – pierwszy kod
**Mowa:**
Gdy programujemy lot drona, nie wysyłamy mu komendy "leć szybciej do przodu". Zamiast tego mówimy: "obecnie jesteś w punkcie (0, 0, 0), twoim zadaniem jest znalezienie się w punkcie (10, 5, 20)". Matematyka robi resztę – wylicza trajektorię, czyli linię, po której maszyna ma się przemieścić. Zobaczmy, jak wygląda najprostszy, automatyczny start.
**[WIDEO: `01_podstawy_ruchu/01_start.py` i `02_trajectory.py`]** 
(Omawiamy krótko start maszyny i lot od punktu do punktu po prostej trajektorii).

---

## 3. Fizyka i matematyka lotu – starcie z rzeczywistością (10 minut)

**Slajd:** Siły i Wektory - Wiatr jako wyzwanie
**Mowa:**
Na papierze lot po idealnej prostej wydaje się prosty. Ale nasz dron lata na zewnątrz. A tam wieje wiatr. Co to jest wiatr z punktu widzenia drona? To po prostu wektor siły, który w każdej sekundzie spycha go z kursu! Żeby lecieć prosto, komputer drona musi co ułamek sekundy dodawać wektor swojej prędkości do wektora wiatru, a następnie nanosić odpowiednie korekty, odchylając się pod wiatr. Widzicie, to zwykłe dodawanie wektorów, które znacie ze szkoły!

**Slajd:** Trajektorie - krzywe parametryczne
**Mowa:**
A co, gdy dron ma sfilmować obiekt z każdej strony? Musi krążyć. Do tego nie wystarczy jeden punkt. Wykorzystujemy coś, co w matematyce nazywamy krzywymi parametrycznymi. Możemy zaprogramować tor lotu przypominający okrąg, elipsę czy spiralę, posługując się chociażby funkcjami trygonometrycznymi (sinus i cosinus).
**[WIDEO: `02_fizyka_i_matematyka/03_parametric_shapes.py` i `04_physics_wind.py`]**
(Pokazujemy lot po pięknych, matematycznych krzywych i jak system stara się go utrzymać w przypadku uderzeń wiatru).

---

## 4. Misje autonomiczne i sprytne algorytmy (10 minut)

**Slajd:** Skanowanie obszaru i optymalizacja tras
**Mowa:**
Przejdźmy teraz do prawdziwych wyzwań. Wyobraźmy sobie akcję poszukiwawczą. Dron ma zbadać duży obszar lasu. Jak zaplanować lot, aby baterii wystarczyło na jak najdłużej? Skąd wie, w jakiej kolejności odwiedzać tzw. punkty kontrolne?
To zagadnienie znane w informatyce i matematyce jako "Problem komiwojażera". Algorytm grafowy szuka w ułamku sekundy takiej sekwencji połączeń między punktami, żeby całkowity pokonany dystans był jak najkrótszy. Zamiast rysować długopisem na mapie, używamy skomplikowanych obliczeń do automatycznej nawigacji.

**Slajd:** Zaawansowane misje ratunkowe 3D
**Mowa:**
Poszukiwania często odbywają się w zróżnicowanym terenie górskim, albo wymagają śledzenia poruszającego się obiektu. Dron musi nieustannie przeliczać swoje położenie względem uciekającego celu.
**[WIDEO: `03_misje_autonomiczne/06_tsp_mission.py` oraz `04_wizualizacja_3D/07_a_polygon_search.py`, `07_b_pursuit.py`, `07_c_cylinder_scan.py`]**
(Prezentujemy na nagraniach skanowanie obszaru, oraz widowiskowy pościg i przeszukiwanie objętości cylindra – np. nad płonącym budynkiem).

---

## 5. Układy autonomiczne i roje – jak naśladować przyrodę? (8 minut)

**Slajd:** Jak ptaki i owady stają się inspiracją?
**Mowa:**
Wydawałoby się, że osiągnęliśmy dużo. Ale wyobraźcie sobie sytuację ekstremalną. Zamiast wysyłać jednego potężnego, drogiego drona, wysyłamy 30 małych dronów, które komunikują się ze sobą.
Skąd czerpiemy inspirację? Z natury! Obserwując ławice ryb czy stada ptaków. Naukowcy sformułowali wzory opisujące to zachowanie – nazywamy to m.in. modelem Cuckera-Smale'a.
Każdy dron musi spełnić 3 zasady matematyczne:
1. Nie uderzaj w sąsiadów (separacja).
2. Podążaj tam, gdzie reszta grupy (aliniacja).
3. Nie oddalaj się od stada (kohezja).

**Slajd:** Rój w akcji
**Mowa:**
Rozwiązując w każdym ułamku sekundy zaledwie kilka równań, te 30 maszyn zaczyna "żyć" jako jeden organizm, wspólnie omijając przeszkody i realizując cel misji bez centralnego dowodzenia!
**[WIDEO: `05_uklady_autonomiczne/05_a_cucker_smale_model.py` i `05_b_flock_split.py`]**
(Puszczamy wideo ze startu całego roju oraz wideo pokazujące piękne rozdzielenie się stada na dwie grupy przy omijaniu przeszkody).

---

## 6. Podsumowanie i Zakończenie (2 minuty)

**Slajd:** Podsumowanie
**Mowa:**
W ten sposób przeszliśmy od prostego przesunięcia w przód, po potężne, inteligentne układy składające się z całych armad statków latających.
Jak sami widzicie – w programowaniu nowoczesnych robotów nie chodzi tylko o to, by znać języki programowania. Chodzi o to, żeby umieć używać języka, którym opisany jest nasz świat fizyczny. A tym językiem jest właśnie matematyka. To wzory sprawiają, że drony mogą uratować komuś życie.
Gdy następnym razem zobaczycie na lekcji matematyki równanie z dwiema niewiadomymi albo funkcję trygonometryczną, pamiętajcie, że to nie jest tylko szkolny wymóg – to klucz do otwarcia drzwi z napisem "przyszłość technologii".

Dziękuję za Wasz czas i uwagę! Zapraszam do zadawania pytań.

---

## 7. Q&A (5 minut)
- Przewidzieć dodatkowe pytania odnośnie np. awarii GPS, czasu lotu drona na jednym ładowaniu lub o to jak zacząć z DroneKitem/Pythonem.
