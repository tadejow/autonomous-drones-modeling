# --- ŁATKA DLA NOWYCH WERSJI PYTHONA ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import matplotlib
matplotlib.use('TkAgg')

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import matplotlib.pyplot as plt
import time
import math
import numpy as np

def send_ned_velocity(vehicle, v_x, v_y, v_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 0, 0, 0, v_x, v_y, v_z, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def get_distance_ned(loc_baza, loc_cel):
    earth_radius = 6378137.0
    dlat = math.radians(loc_cel.lat - loc_baza.lat)
    dlon = math.radians(loc_cel.lon - loc_baza.lon)
    n = dlat * earth_radius
    e = dlon * earth_radius * math.cos(math.radians(loc_baza.lat))
    d = -(loc_cel.alt - loc_baza.alt)
    return np.array([n, e, d])

print("=== INICJALIZACJA ROJU 8 DRONÓW ===")
porty = ['udp:127.0.0.1:14550', 'udp:127.0.0.1:14560'] + [f'udp:127.0.0.1:{14570 + i*10}' for i in range(6)]
roje = []

for i, port in enumerate(porty):
    print(f"Łączenie z Dronem {i} (Port: {port})...")
    v = connect(port, wait_ready=True)
    v.parameters['BATT_FS_LOW_ACT'] = 0
    v.parameters['BATT_FS_CRT_ACT'] = 0
    while not v.is_armable: time.sleep(0.3)
    roje.append(v)

WYSOKOSC = 15.0
print(f"\nUzbrajanie i start na {WYSOKOSC}m...")
for v in roje:
    v.mode = VehicleMode("GUIDED")
    v.armed = True

time.sleep(2)
for v in roje: v.simple_takeoff(WYSOKOSC)

while not all(v.location.global_relative_frame.alt > 13.5 for v in roje):
    time.sleep(1)

print("\nRój w powietrzu! Rozpoczynamy misję rozdzielenia.")

# ==========================================
# PARAMETRY MODELU
# ==========================================
K_CS = 1.8
BETA = 0.5
K_COHESION = 0.3
R_REP = 35.0
D_SAFE = 5.0 
V_MAX = 7.5

baza_startowa = roje[0].location.global_relative_frame
czas_startu = time.time()
last_time = time.time()

pary = [(i, j) for i in range(8) for j in range(i + 1, 8)]
historia_par = {p: {'t': [], 'd': []} for p in pary}

plt.ion()
fig = plt.figure(figsize=(15, 6))
ax1 = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122)

# Własny dobór kolorów: Liderzy kolorowi, skrzydłowi na szaro
kolory_dronow = ['red', 'magenta'] + ['gray'] * 6

historia_x = [[] for _ in range(8)]
historia_y = [[] for _ in range(8)]
historia_z = [[] for _ in range(8)]

try:
    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        if dt > 1.0 or dt == 0: dt = 0.2
        
        t_rel = current_time - czas_startu
        
        X = np.zeros((8, 3))
        V = np.zeros((8, 3))
        
        for i, v in enumerate(roje):
            X[i] = get_distance_ned(baza_startowa, v.location.global_relative_frame)
            vel = v.velocity
            V[i] = np.array([vel[0], vel[1], vel[2]])
            
            historia_x[i].append(X[i][0])
            historia_y[i].append(X[i][1])
            historia_z[i].append(-X[i][2])
            
        for i, j in pary:
            dist = np.linalg.norm(X[i] - X[j])
            historia_par[(i, j)]['t'].append(t_rel)
            historia_par[(i, j)]['d'].append(dist)

        # ==========================================
        # LINIA CZASU I LANDING PROFILE
        # ==========================================
        if t_rel < 110.0:
            target_alt = 15.0
        elif t_rel < 150.0:
            postep_ladowania = (t_rel - 110.0) / (150.0 - 110.0)
            target_alt = max(0.5, 15.0 * (1.0 - postep_ladowania))
        else:
            print("Misja zakończona, lądowanie w dwóch strefach!")
            break

        v_l1_z = -1.0 * (target_alt - (-X[0][2]))
        v_l2_z = -1.0 * (target_alt - (-X[1][2]))

        # MANEWRY LIDERÓW:
        if t_rel < 50.0:
            # FAZA 1: Ciasny lot równoległy (0-50s)
            v_l1 = np.array([2.5, -0.8, v_l1_z])
            v_l2 = np.array([2.5,  0.8, v_l2_z])
        else:
            # FAZA 2 & 3: BARDZO SZEROKIE ROZDZIELENIE (Odbicie baczne V_E = 5 m/s!)
            v_l1 = np.array([1.2, -5.0, v_l1_z])
            v_l2 = np.array([1.2,  5.0, v_l2_z])

        send_ned_velocity(roje[0], v_l1[0], v_l1[1], v_l1[2])
        send_ned_velocity(roje[1], v_l2[0], v_l2[1], v_l2[2])

        # ==========================================
        # LOGIKA SKRZYDŁOWYCH
        # ==========================================
        for i in range(2, 8):
            d_to_l1 = np.linalg.norm(X[0] - X[i])
            d_to_l2 = np.linalg.norm(X[1] - X[i])
            
            blizszy_lider = X[0] if d_to_l1 < d_to_l2 else X[1]
            v_cohesion = K_COHESION * (blizszy_lider - X[i])
            
            v_cucker = np.zeros(3)
            v_repulsja = np.zeros(3)
            
            for j in range(8):
                if i == j: continue
                diff_pos = X[i] - X[j]
                dystans = np.linalg.norm(diff_pos)
                if dystans == 0: dystans = 0.1
                
                waga_cs = K_CS / (1.0 + (dystans / 10.0)**BETA)
                v_cucker += waga_cs * (V[j] - V[i])
                
                if dystans < D_SAFE:
                    v_repulsja += R_REP * (diff_pos / (dystans**3))
                    
            v_target = V[i] + (v_cucker + v_repulsja + v_cohesion) * dt
            
            speed = np.linalg.norm(v_target[:2])
            if speed > V_MAX: v_target[:2] = (v_target[:2] / speed) * V_MAX
            
            send_ned_velocity(roje[i], v_target[0], v_target[1], v_target[2])

        # ==========================================
        # WIZUALIZACJA Z OPTYMALNĄ PERSPEKTYWĄ 3D
        # ==========================================
        ax1.clear()
        ax2.clear()

        all_x = [x for h in historia_x for x in h]
        all_y = [y for h in historia_y for y in h]
        
        status_fazy = "Lot w grupie" if t_rel < 50 else ("Rozdzielenie" if t_rel < 110 else "Lądowanie podgrup")
        ax1.set_title(f'Rój 3D: {status_fazy} (t={t_rel:.1f}s)')
        
        # NAJLEPSZA PERSPEKTYWA 3D DO ROZDZIELENIA (Kąt widzenia z ukosa)
        ax1.view_init(elev=22, azim=-75)
        
        ax1.set_xlim([min(all_x) - 10, max(all_x) + 10])
        ax1.set_ylim([min(all_y) - 10, max(all_y) + 10])
        ax1.set_zlim([0, 25])
        
        ax1.set_xlabel('North (m)')
        ax1.set_ylabel('East (m)')
        ax1.set_zlabel('Altitude (m)')

        # Rysowanie Liderów (Grube, kolorowe linie)
        ax1.plot(historia_x[0], historia_y[0], historia_z[0], color='red', label='Lider 1', linewidth=2.5)
        ax1.scatter(historia_x[0][-1], historia_y[0][-1], historia_z[0][-1], color='red', s=80)

        ax1.plot(historia_x[1], historia_y[1], historia_z[1], color='magenta', label='Lider 2', linewidth=2.5)
        ax1.scatter(historia_x[1][-1], historia_y[1][-1], historia_z[1][-1], color='magenta', s=80)

        # Rysowanie Skrzydłowych (Szare, przerywane linie - eleganckie tło)
        for i in range(2, 8):
            lbl = "Skrzydłowi (Rój)" if i == 2 else "" # Legenda tylko raz
            ax1.plot(historia_x[i], historia_y[i], historia_z[i], color='gray', linestyle='--', alpha=0.6, label=lbl)
            ax1.scatter(historia_x[i][-1], historia_y[i][-1], historia_z[i][-1], color='dimgray', s=35)

        ax1.legend(loc='upper left', fontsize='small')

        # SUBPLOT 2: ANALIZA ALERTÓW
        ax2.set_title('Monitor Zbliżeniowy (Ostatnie 10s)')
        ax2.axhline(y=D_SAFE, color='red', linestyle='--', label=f'Strefa kolizyjna ({D_SAFE}m)')

        aktywne_alerty = 0

        for i, j in pary:
            t_arr = np.array(historia_par[(i, j)]['t'])
            d_arr = np.array(historia_par[(i, j)]['d'])

            maska_10s = t_arr >= (t_rel - 10.0)
            d_ostatnie = d_arr[maska_10s]

            naruszenie = np.any(d_ostatnie < D_SAFE) if len(d_ostatnie) > 0 else False

            if naruszenie:
                if i == 0 or j == 0:
                    line_color = 'red'
                elif i == 1 or j == 1:
                    line_color = 'magenta'
                else:
                    line_color = 'black'

                ax2.plot(t_arr, d_arr, color=line_color, linewidth=1.5, alpha=0.85)
                
                if d_arr[-1] < D_SAFE and aktywne_alerty < 3:
                    ax2.annotate(f'ALERT {i}-{j}: {d_arr[-1]:.1f}m',
                                 xy=(t_arr[-1], d_arr[-1]),
                                 xytext=(t_arr[-1] - 8, d_arr[-1] + 1.8),
                                 arrowprops=dict(arrowstyle='->', color=line_color),
                                 color=line_color, fontweight='bold', fontsize=8)
                    aktywne_alerty += 1
            else:
                ax2.plot(t_arr, d_arr, color='lightgray', linewidth=0.6, alpha=0.3)

        ax2.set_xlabel('Czas (s)')
        ax2.set_ylabel('Odległość (m)')
        ax2.set_ylim([0, 35])
        ax2.grid(True)
        ax2.legend(loc='upper right')

        plt.draw()
        plt.pause(0.05)

except KeyboardInterrupt:
    pass

print("\nMisja zakończona.")
for v in roje:
    v.mode = VehicleMode("RTL")
    v.close()
plt.ioff(); plt.show()
