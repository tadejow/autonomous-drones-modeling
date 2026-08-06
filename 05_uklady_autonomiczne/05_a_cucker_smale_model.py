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

# ==========================================
# MATH & KINEMATICS UTILITIES
# ==========================================

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

# ==========================================
# MAIN PROGRAM LOGIC
# ==========================================

def main():
    print("=== SWARM INITIALIZATION (3 DRONES) ===")
    ports = ['udp:127.0.0.1:14550', 'udp:127.0.0.1:14560', 'udp:127.0.0.1:14570']
    swarm = []

    for i, port in enumerate(ports):
        print(f"Connecting to Drone {i} on {port}...")
        v = connect(port, wait_ready=True)
        v.parameters['BATT_FS_LOW_ACT'] = 0
        v.parameters['BATT_FS_CRT_ACT'] = 0
        while not v.is_armable: time.sleep(0.5)
        swarm.append(v)

    LEADER_ALTITUDE = 15.0

    print("\nArming swarm and taking off...")
    for v in swarm:
        v.mode = VehicleMode("GUIDED")
        v.armed = True

    time.sleep(2)
    swarm[0].simple_takeoff(LEADER_ALTITUDE)
    swarm[1].simple_takeoff(12.0) 
    swarm[2].simple_takeoff(10.0) 

    print("Waiting for safe altitude...")
    all_airborne = False
    while not all_airborne:
        all_airborne = all(v.location.global_relative_frame.alt > 8.0 for v in swarm)
        time.sleep(1)

    print("Swarm airborne! Starting Cucker-Smale continuous control.")

    # Swarm Model Parameters
    K_CS = 2.0        
    BETA = 0.5        
    K_COHESION = 0.3  
    R_REP = 35.0      
    D_SAFE = 5.0      
    V_MAX = 8.0       

    leader_origin = swarm[0].location.global_relative_frame
    start_time = time.time()
    last_time = time.time()

    plt.ion()
    fig = plt.figure(figsize=(14, 6))
    ax1 = fig.add_subplot(121, projection='3d')
    ax2 = fig.add_subplot(122)

    colors = ['red', 'blue', 'green']
    hist_x = [[], [], []]
    hist_y = [[], [], []]
    hist_z = [[], [], []]

    t_hist = []
    d01_hist, d02_hist, d12_hist = [], [], []

    try:
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            if dt > 1.0 or dt == 0: dt = 0.2
            
            sim_time = current_time - start_time
            
            X = np.zeros((3, 3))
            V = np.zeros((3, 3))
            
            for i, v in enumerate(swarm):
                X[i] = get_distance_ned(leader_origin, v.location.global_relative_frame)
                vel = v.velocity
                V[i] = np.array([vel[0], vel[1], vel[2]])
                
                hist_x[i].append(X[i][0])
                hist_y[i].append(X[i][1])
                hist_z[i].append(-X[i][2])
                
            d01 = np.linalg.norm(X[0] - X[1])
            d02 = np.linalg.norm(X[0] - X[2])
            d12 = np.linalg.norm(X[1] - X[2])
            
            t_hist.append(sim_time)
            d01_hist.append(d01); d02_hist.append(d02); d12_hist.append(d12)
                
            # 1. LEADER LOGIC (Fixed Altitude)
            v_leader_n = 2.5
            v_leader_e = math.sin(sim_time * 0.4) * 3.0 
            v_leader_d = -1.0 * (LEADER_ALTITUDE - (-X[0][2])) 
            send_ned_velocity(swarm[0], v_leader_n, v_leader_e, v_leader_d)
            
            # 2. FOLLOWERS LOGIC (Full 3D vectors)
            for i in range(1, 3):
                vector_to_leader = X[0] - X[i]
                v_cohesion = K_COHESION * vector_to_leader
                
                v_cucker = np.zeros(3)
                v_repulsion = np.zeros(3)
                
                for j in range(3):
                    if i == j: continue
                    diff_pos = X[i] - X[j]
                    dist = np.linalg.norm(diff_pos)
                    if dist == 0: dist = 0.1
                    
                    weight_cs = K_CS / (1.0 + (dist / 10.0)**BETA)
                    v_cucker += weight_cs * (V[j] - V[i])
                    
                    if dist < D_SAFE:
                        v_repulsion += R_REP * (diff_pos / (dist**3))
                        
                v_target = V[i] + (v_cucker + v_repulsion + v_cohesion) * dt
                
                speed_total = np.linalg.norm(v_target)
                if speed_total > V_MAX:
                    v_target = (v_target / speed_total) * V_MAX
                    
                send_ned_velocity(swarm[i], v_target[0], v_target[1], v_target[2])
                
            # 3. 3D AND 2D PLOTTING
            ax1.clear()
            ax2.clear()
            
            all_x = [x for h in hist_x for x in h]
            all_y = [y for h in hist_y for y in h]
            all_z = [z for h in hist_z for z in h]
            
            ax1.set_title(f'Free Swarm 3D (Time: {sim_time:.1f}s)')
            ax1.set_xlim([min(all_x) - 10, max(all_x) + 10])
            ax1.set_ylim([min(all_y) - 10, max(all_y) + 10])
            ax1.set_zlim([0, max(max(all_z) + 5, 20)])
            
            ax1.set_xlabel('North (m)')
            ax1.set_ylabel('East (m)')
            ax1.set_zlabel('Altitude (m)')
            
            for i in range(3):
                lbl = "Leader" if i == 0 else f"Follower {i}"
                ax1.plot(hist_x[i], hist_y[i], hist_z[i], color=colors[i], label=lbl)
                ax1.scatter(hist_x[i][-1], hist_y[i][-1], hist_z[i][-1], color=colors[i], s=80)
                
            ax1.legend(loc='upper left')
            
            ax2.set_title('Inter-drone Distances (m)')
            ax2.plot(t_hist, d01_hist, color='purple', label='Drone 0 - 1')
            ax2.plot(t_hist, d02_hist, color='orange', label='Drone 0 - 2')
            ax2.plot(t_hist, d12_hist, color='cyan', label='Drone 1 - 2')
            ax2.axhline(y=D_SAFE, color='black', linestyle='--', label=f'Safe Zone ({D_SAFE}m)')
            
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Distance (m)')
            max_dist = max(max(d01_hist), max(d02_hist), max(d12_hist))
            ax2.set_ylim([0, max(max_dist + 5, 15)])
            ax2.grid(True)
            ax2.legend(loc='upper right')
            
            plt.draw()
            plt.pause(0.05)

    except KeyboardInterrupt:
        print("\nSimulation aborted by user. Initiating Return to Launch (RTL).")
        for v in swarm:
            v.mode = VehicleMode("RTL")
            v.close()
        plt.ioff(); plt.show()

if __name__ == "__main__":
    main()
