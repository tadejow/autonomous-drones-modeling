# --- PYTHON 3.14 COMPATIBILITY PATCH ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import matplotlib
matplotlib.use('TkAgg')

import time
import math
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import matplotlib.pyplot as plt

def send_ned_velocity(vehicle, v_x: float, v_y: float, v_z: float) -> None:
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 0, 0, 0, v_x, v_y, v_z, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def get_distance_ned(base_loc: LocationGlobalRelative, target_loc: LocationGlobalRelative) -> np.ndarray:
    earth_radius = 6378137.0
    d_lat = math.radians(target_loc.lat - base_loc.lat)
    d_lon = math.radians(target_loc.lon - base_loc.lon)
    n = d_lat * earth_radius
    e = d_lon * earth_radius * math.cos(math.radians(base_loc.lat))
    d = -(target_loc.alt - base_loc.alt)
    return np.array([n, e, d])

def main():
    print("=== SWARM INITIALIZATION (8 DRONES) ===")
    ports = ['udp:127.0.0.1:14550', 'udp:127.0.0.1:14560'] + [f'udp:127.0.0.1:{14570 + i*10}' for i in range(6)]
    swarm = []

    for i, port in enumerate(ports):
        print(f"Connecting to Drone {i} on {port}...")
        v = connect(port, wait_ready=True)
        v.parameters['BATT_FS_LOW_ACT'] = 0
        v.parameters['BATT_FS_CRT_ACT'] = 0
        while not v.is_armable: time.sleep(0.3)
        swarm.append(v)

    ALTITUDE = 15.0
    print(f"\nArming and taking off to {ALTITUDE}m...")
    for v in swarm:
        v.mode = VehicleMode("GUIDED")
        v.armed = True

    time.sleep(2)
    for v in swarm: v.simple_takeoff(ALTITUDE)

    while not all(v.location.global_relative_frame.alt > 13.5 for v in swarm):
        time.sleep(1)

    print("\nSwarm airborne! Executing full mission (Flock -> Split -> Land).")

    # Algorithm parameters
    K_CS = 1.8
    BETA = 0.5
    K_COHESION = 0.25
    R_REP = 30.0
    D_SAFE = 5.0 
    V_MAX = 7.0

    base_origin = swarm[0].location.global_relative_frame
    start_time = time.time()
    last_time = time.time()

    pairs = [(i, j) for i in range(8) for j in range(i + 1, 8)]
    pair_history = {p: {'t': [], 'd': []} for p in pairs}

    plt.ion()
    fig = plt.figure(figsize=(15, 6))
    ax1 = fig.add_subplot(121, projection='3d')
    ax2 = fig.add_subplot(122)

    drone_colors = ['red', 'magenta'] + ['gray'] * 6
    hist_x = [[] for _ in range(8)]
    hist_y = [[] for _ in range(8)]
    hist_z = [[] for _ in range(8)]

    try:
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            if dt > 1.0 or dt == 0: dt = 0.2
            
            sim_time = current_time - start_time
            
            X = np.zeros((8, 3))
            V = np.zeros((8, 3))
            
            for i, v in enumerate(swarm):
                X[i] = get_distance_ned(base_origin, v.location.global_relative_frame)
                vel = v.velocity
                V[i] = np.array([vel[0], vel[1], vel[2]])
                
                hist_x[i].append(X[i][0])
                hist_y[i].append(X[i][1])
                hist_z[i].append(-X[i][2])
                
            for i, j in pairs:
                dist = np.linalg.norm(X[i] - X[j])
                pair_history[(i, j)]['t'].append(sim_time)
                pair_history[(i, j)]['d'].append(dist)

            # ==========================================
            # LEADERS MISSION LOGIC
            # ==========================================
            if sim_time < 110.0:
                target_alt = 15.0
            elif sim_time < 150.0:
                landing_progress = (sim_time - 110.0) / (150.0 - 110.0)
                target_alt = max(0.5, 15.0 * (1.0 - landing_progress))
            else:
                print("Mission completed. Both sub-swarms have landed!")
                break

            v_l1_z = -1.0 * (target_alt - (-X[0][2]))
            v_l2_z = -1.0 * (target_alt - (-X[1][2]))

            if sim_time < 50.0:
                # PHASE 1: Parallel Flight
                v_l1 = np.array([2.5, -1.0, v_l1_z])
                v_l2 = np.array([2.5,  1.0, v_l2_z])
            else:
                # PHASE 2: Wide Split
                v_l1 = np.array([1.5, -4.5, v_l1_z])
                v_l2 = np.array([1.5,  4.5, v_l2_z])

            send_ned_velocity(swarm[0], v_l1[0], v_l1[1], v_l1[2])
            send_ned_velocity(swarm[1], v_l2[0], v_l2[1], v_l2[2])

            # ==========================================
            # FOLLOWERS LOGIC
            # ==========================================
            for i in range(2, 8):
                d_to_l1 = np.linalg.norm(X[0] - X[i])
                d_to_l2 = np.linalg.norm(X[1] - X[i])
                
                closest_leader = X[0] if d_to_l1 < d_to_l2 else X[1]
                v_cohesion = K_COHESION * (closest_leader - X[i])
                
                v_cucker = np.zeros(3)
                v_repulsion = np.zeros(3)
                
                for j in range(8):
                    if i == j: continue
                    diff_pos = X[i] - X[j]
                    dist = np.linalg.norm(diff_pos)
                    if dist == 0: dist = 0.1
                    
                    weight_cs = K_CS / (1.0 + (dist / 10.0)**BETA)
                    v_cucker += weight_cs * (V[j] - V[i])
                    
                    if dist < D_SAFE:
                        v_repulsion += R_REP * (diff_pos / (dist**3))
                        
                v_target = V[i] + (v_cucker + v_repulsion + v_cohesion) * dt
                
                speed = np.linalg.norm(v_target[:2])
                if speed > V_MAX: v_target[:2] = (v_target[:2] / speed) * V_MAX
                
                send_ned_velocity(swarm[i], v_target[0], v_target[1], v_target[2])

            # ==========================================
            # 3D AND 2D PLOTTING WITH ALERT LOGIC
            # ==========================================
            ax1.clear()
            ax2.clear()

            all_x = [x for h in hist_x for x in h]
            all_y = [y for h in hist_y for y in h]
            
            phase_status = "Flock Flight" if sim_time < 50 else ("Swarm Split" if sim_time < 110 else "Landing")
            ax1.set_title(f'Swarm 3D: {phase_status} (t={sim_time:.1f}s)')
            ax1.view_init(elev=22, azim=-75)
            
            ax1.set_xlim([min(all_x) - 10, max(all_x) + 10])
            ax1.set_ylim([min(all_y) - 10, max(all_y) + 10])
            ax1.set_zlim([0, 25])
            
            ax1.set_xlabel('North (m)')
            ax1.set_ylabel('East (m)')
            ax1.set_zlabel('Altitude (m)')

            for i in range(8):
                lbl = f"Leader {i+1}" if i < 2 else ("Follower" if i == 2 else "")
                alpha_val = 1.0 if i < 2 else 0.6
                line_style = '-' if i < 2 else '--'
                size_val = 80 if i < 2 else 35
                
                ax1.plot(hist_x[i], hist_y[i], hist_z[i], color=drone_colors[i], linestyle=line_style, alpha=alpha_val, label=lbl if lbl else "_nolegend_")
                ax1.scatter(hist_x[i][-1], hist_y[i][-1], hist_z[i][-1], color=drone_colors[i], s=size_val)
            
            ax1.legend(loc='upper left', fontsize='small')

            ax2.set_title('Proximity Monitor (Last 10s)')
            ax2.axhline(y=D_SAFE, color='red', linestyle='--', label=f'Collision Zone ({D_SAFE}m)')

            active_alerts = 0

            for i, j in pairs:
                t_arr = np.array(pair_history[(i, j)]['t'])
                d_arr = np.array(pair_history[(i, j)]['d'])

                mask_10s = t_arr >= (sim_time - 10.0)
                d_recent = d_arr[mask_10s]

                violation = np.any(d_recent < D_SAFE) if len(d_recent) > 0 else False

                if violation:
                    line_color = 'red' if (i == 0 or j == 0) else ('magenta' if (i == 1 or j == 1) else 'black')
                    ax2.plot(t_arr, d_arr, color=line_color, linewidth=1.5, alpha=0.85)
                    
                    if d_arr[-1] < D_SAFE and active_alerts < 3:
                        ax2.annotate(f'ALERT {i}-{j}: {d_arr[-1]:.1f}m',
                                     xy=(t_arr[-1], d_arr[-1]),
                                     xytext=(t_arr[-1] - 8, d_arr[-1] + 1.8),
                                     arrowprops=dict(arrowstyle='->', color=line_color),
                                     color=line_color, fontweight='bold', fontsize=8)
                        active_alerts += 1
                else:
                    ax2.plot(t_arr, d_arr, color='lightgray', linewidth=0.6, alpha=0.3)

            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Distance (m)')
            ax2.set_ylim([0, 35])
            ax2.grid(True)
            ax2.legend(loc='upper right')

            plt.draw()
            plt.pause(0.05)

    except KeyboardInterrupt:
        pass

    print("\nShutting down motors and returning to base.")
    for v in swarm:
        v.mode = VehicleMode("RTL")
        v.close()
    plt.ioff(); plt.show()

if __name__ == "__main__":
    main()
