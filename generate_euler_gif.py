import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import os

# Drone geometry
L = 1.0  # Arm length
rotors_base = np.array([[L, L, 0], [-L, L, 0], [-L, -L, 0], [L, -L, 0]]).T
center = np.array([[0], [0], [0]])

# Body axes
axes_len = 1.5
body_axes_base = np.array([[axes_len, 0, 0], [0, axes_len, 0], [0, 0, -axes_len]]).T

def get_rot_x(phi):
    return np.array([[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])

def get_rot_y(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

def get_rot_z(psi):
    return np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]])

fig = plt.figure(figsize=(6, 5), facecolor='white')
ax = fig.add_subplot(111, projection='3d')

out_dir = "prezentacja/images/euler_frames"
os.makedirs(out_dir, exist_ok=True)

def update(frame):
    ax.cla()
    ax.axis('off')
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    ax.view_init(elev=25, azim=-135)
    
    phase = frame // 60
    sub_frame = frame % 60
    phi, theta, psi = 0, 0, 0
    angle_val = 35 * np.pi / 180 * np.sin(sub_frame * 2 * np.pi / 60)
    
    if phase == 0:
        phi = angle_val
        title_text = f"Przechylenie / Roll ($\\phi = {np.degrees(phi):.0f}^\\circ$)"
    elif phase == 1:
        theta = angle_val
        title_text = f"Pochylenie / Pitch ($\\theta = {np.degrees(theta):.0f}^\\circ$)"
    else:
        psi = angle_val
        title_text = f"Odchylenie / Yaw ($\\psi = {np.degrees(psi):.0f}^\\circ$)"
        
    ax.set_title(title_text, fontsize=14, y=0.95)
    
    R = get_rot_z(psi) @ get_rot_y(theta) @ get_rot_x(phi)
    rotors = R @ rotors_base
    b_axes = R @ body_axes_base
    
    ax.plot([-2, 2], [0, 0], [0, 0], color='gray', linestyle=':', alpha=0.5)
    ax.plot([0, 0], [-2, 2], [0, 0], color='gray', linestyle=':', alpha=0.5)
    ax.plot([0, 0], [0, 0], [-2, 2], color='gray', linestyle=':', alpha=0.5)
    
    ax.plot([rotors[0,0], rotors[0,2]], [rotors[1,0], rotors[1,2]], [rotors[2,0], rotors[2,2]], color='gray', linewidth=3)
    ax.plot([rotors[0,1], rotors[0,3]], [rotors[1,1], rotors[1,3]], [rotors[2,1], rotors[2,3]], color='gray', linewidth=3)
    
    ax.scatter([0], [0], [0], color='black', s=50)
    ax.scatter(rotors[0,:], rotors[1,:], rotors[2,:], color='darkgray', s=300, depthshade=False, edgecolors='black')
    
    ax.quiver(0, 0, 0, b_axes[0,0], b_axes[1,0], b_axes[2,0], color='red', linewidth=2, arrow_length_ratio=0.15)
    ax.text(b_axes[0,0]*1.1, b_axes[1,0]*1.1, b_axes[2,0]*1.1, "$X_B$ (Przód)", color='red', fontsize=12)
    
    ax.quiver(0, 0, 0, b_axes[0,1], b_axes[1,1], b_axes[2,1], color='green', linewidth=2, arrow_length_ratio=0.15)
    ax.text(b_axes[0,1]*1.1, b_axes[1,1]*1.1, b_axes[2,1]*1.1, "$Y_B$ (Prawa)", color='green', fontsize=12)
    
    ax.quiver(0, 0, 0, b_axes[0,2], b_axes[1,2], b_axes[2,2], color='blue', linewidth=2, arrow_length_ratio=0.15)
    ax.text(b_axes[0,2]*1.1, b_axes[1,2]*1.1, b_axes[2,2]*1.1, "$Z_B$ (Dół)", color='blue', fontsize=12)

# Save frames for LaTeX
for i in range(180):
    update(i)
    plt.savefig(f"{out_dir}/frame_{i:03d}.png", dpi=120, bbox_inches='tight', pad_inches=0.1)

# Optional: Still save gif just in case
ani = animation.FuncAnimation(fig, update, frames=180, interval=50)
ani.save('euler_angles.gif', writer='pillow', fps=20)
print("Saved PNG frames and euler_angles.gif")
