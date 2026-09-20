import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque

class DronePlotter:
    """
    Manager wizualizacji 3D. Wydłuża ślad lotu (trail_length) i pozwala na różne widoki.
    """
    def __init__(self, title="Drone Trajectory", trail_length=150):
        self.plt = plt
        self.plt.ion()
        self.fig = self.plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.title = title
        
        # Ograniczamy długość ogona za pomocą deque
        self.history_x = deque(maxlen=trail_length)
        self.history_y = deque(maxlen=trail_length)
        self.history_z = deque(maxlen=trail_length)

        # Cele opcjonalne (np. uciekający balon)
        self.target_x = deque(maxlen=trail_length)
        self.target_y = deque(maxlen=trail_length)
        self.target_z = deque(maxlen=trail_length)

    def set_view(self, elev, azim):
        """Ustawia kąt kamery by np. ukazać bujanie wiatrem."""
        self.ax.view_init(elev=elev, azim=azim)

    def update(self, drone_pos, target_pos=None, balloon_icon=False, attitude=None):
        """
        drone_pos: (x, y, z)
        target_pos: opcjonalny (x, y, z)
        attitude: opcjonalny (pitch, roll, yaw) w radianach
        """
        dx, dy, dz = drone_pos
        self.history_x.append(dx)
        self.history_y.append(dy)
        self.history_z.append(dz)

        if target_pos:
            tx, ty, tz = target_pos
            self.target_x.append(tx)
            self.target_y.append(ty)
            self.target_z.append(tz)

        self.ax.clear()
        self.ax.set_title(self.title)

        # Skalowanie osi do historii
        all_x = list(self.history_x) + (list(self.target_x) if target_pos else [])
        all_y = list(self.history_y) + (list(self.target_y) if target_pos else [])
        all_z = list(self.history_z) + (list(self.target_z) if target_pos else [])
        
        if len(all_x) > 0:
            margin = 5
            self.ax.set_xlim([min(all_x)-margin, max(all_x)+margin])
            self.ax.set_ylim([min(all_y)-margin, max(all_y)+margin])
            self.ax.set_zlim([0, max(all_z)+margin if max(all_z) > 10 else 20])

        # Rysowanie wydłużonego śladu drona
        self.ax.plot(list(self.history_x), list(self.history_y), list(self.history_z), color='blue', label='Dron Ślad', linewidth=2)
        
        # Wskaźnik drona (tilt indicator)
        if attitude:
            pitch, roll, yaw = attitude
            import numpy as np
            # Długość ramienia do wizualizacji przechyłu (znacznie powiększona dla lepszego widoku bujania!)
            arm_len = 5.0
            # Uproszczona rotacja
            tilt_x = dx + arm_len * np.sin(pitch)
            tilt_y = dy + arm_len * np.sin(roll)
            tilt_z = dz + arm_len * np.cos(pitch) * np.cos(roll)
            self.ax.plot([dx, tilt_x], [dy, tilt_y], [dz, tilt_z], color='green', linewidth=4, label='Wektor przechyłu')
            self.ax.scatter(dx, dy, dz, color='black', s=60) # Środek drona
        else:
            self.ax.scatter(dx, dy, dz, color='blue', s=50) # aktualna pozycja

        # Rysowanie celu (opcjonalne)
        if target_pos:
            self.ax.plot(list(self.target_x), list(self.target_y), list(self.target_z), color='red', label='Cel Ślad')
            if balloon_icon:
                self.ax.text(tx, ty, tz, '🎈', fontsize=20, color='red', ha='center', va='center')
            else:
                self.ax.scatter(tx, ty, tz, color='red', s=50)
            
        self.ax.legend()
        self.plt.draw()
        self.plt.pause(0.01)

    def close(self):
        self.plt.ioff()
        self.plt.close('all')
