# 🚁 Kurs Programowania Trajektorii Lotu Dronów

[![Python](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-orange.svg)](https://ardupilot.org/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

Witaj w repozytorium materiałów dydaktycznych przygotowanych dla studentów **Wydziału Matematyki i Informatyki Uniwersytetu Wrocławskiego** (w szczególności w ramach działalności Koła Naukowego Zastosowań Matematyki).

Repozytorium zawiera kompletny zbiór skryptów, narzędzi analitycznych oraz dokumentacji stanowiących fundament kursu. Łączy on zagadnienia teoretyczne z dziedziny geometrii obliczeniowej, fizyki i układów wieloagentowych z ich praktyczną implementacją w robotyce i systemach BSP (Bezzałogowych Statków Powietrznych).

---

## 📖 O projekcie

Celem kursu jest wypełnienie luki pomiędzy teoretyczną analizą matematyczną a inżynierią oprogramowania maszyn latających. Zamiast manualnego sterowania, kurs uczy **architektury systemów autonomicznych**, pozwalając na symulację i weryfikację złożonych misji (np. rolnictwo precyzyjne, poszukiwania ratunkowe) w wirtualnym środowisku **ArduPilot SITL (Software-In-The-Loop)**, przed ich potencjalnym przeniesieniem na fizyczne urządzenia.

*Wszystkie kody źródłowe w tym repozytorium zostały przygotowane zgodnie z dobrymi praktykami inżynierskimi (PEP 8) i są pisane w języku angielskim. Dokumentacja merytoryczna pozostaje w języku polskim.*

---

## 📂 Struktura repozytorium

Projekt został podzielony na pięć modułów tematycznych o rosnącym stopniu skomplikowania. W każdym katalogu znajduje się osobny plik `README.md` z zaawansowaną analizą merytoryczną zastosowanej tam matematyki.

*   `01_podstawy_ruchu/` – Inicjalizacja asynchronicznych pętli kontrolnych, start, lądowanie oraz podstawowa nawigacja po prostej trajektorii w układzie NED.
*   `02_fizyka_i_matematyka/` – Przejście do ciągłego planowania lotu przy użyciu równań parametrycznych. Analiza wpływu surowej fizyki (wiatru, bezwładności) na zachowanie regulatorów PID i EKF.
*   `03_misje_autonomiczne/` – Geometria obliczeniowa w zastosowaniach przemysłowych. Wykorzystanie algorytmów optymalizacyjnych (Problem Komiwojażera) i bezstanowych misji pokładowych.
*   `04_wizualizacja_3D/` – Zaawansowane transformacje układów współrzędnych i "zamiatanie" (sweep-line) nieregularnych poligonów. Zawiera wbudowane na żywo wykresy `matplotlib` z naprowadzaniem predykcyjnym (ProNav).
*   `05_uklady_autonomiczne/` – Robotyka roju (Swarm Robotics). Skrypty i bash-owe instalatory łączące się z klastrami od 3 do 8 dronów, implementujące hybrydowy model Cuckera-Smale'a dla zachowań emergentnych.

---

## 🛠️ Wymagania techniczne i Instalacja

Do uruchomienia skryptów z tego repozytorium wymagany jest system z rodziny **Linux** (natywny np. Ubuntu, lub Windows Subsystem for Linux - WSL2) w celu skompilowania symulatora środowiska fizycznego ArduPilot.

### 1. Przygotowanie symulatora (SITL)
Zainstaluj narzędzia kompilacyjne i sklonuj główne repozytorium silnika fizycznego (poza folderem tego kursu):

```bash
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install git python3-pip python3-venv xterm python3-tk -y

cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile
```

### 2. Konfiguracja środowiska Python dla tego kursu
Sklonuj niniejsze repozytorium i zbuduj wyizolowane środowisko wirtualne dla zależności programistycznych:

```bash
git clone https://github.com/tadejow/autonomous-drones-wrones.git
cd autonomous-drones-wrones
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

---

## 🚀 Szybki start (Jak używać kodów)

Złotą zasadą symulacji BSP jest rozdzielenie silnika fizyki (serwera) od skryptu matematycznego (klienta). 

1. **Uruchom silnik fizyki (Terminal 1):**
   Uruchom wirtualny kontroler drona, który zacznie udostępniać nasłuch na porcie UDP `14550`.
   ```bash
   cd ~/ardupilot/ArduCopter
   sim_vehicle.py --console --map
   ```
   *(Poczekaj na pojawienie się drona na mapie).*

2. **Uruchom skrypt matematyczny (Terminal 2):**
   W terminalu z aktywowanym środowiskiem wirtualnym `(venv)` przejdź do wybranego modułu i go uruchom.
   ```bash
   cd ~/autonomous-drones-wrones/01_podstawy_ruchu
   python 01_start.py
   ```

---

## 📜 Licencja

Ten projekt udostępniany jest na licencji [MIT](LICENSE), zachęcając do swobodnego modyfikowania, rozwijania i dzielenia się wiedzą w celach edukacyjnych i naukowych.
