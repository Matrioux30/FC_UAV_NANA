import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

os.makedirs('results', exist_ok=True)

# 1. Plot Skenario Hover
df_h = pd.read_csv('results/hover_mpc.csv')
fig, axs = plt.subplots(3, 2, figsize=(12, 10))
fig.suptitle("MPC Golden Model: Skenario Hover (Stabilitas Titik Setimbang)", fontsize=14)

axs[0, 0].plot(df_h['time'], df_h['x_ref'], 'r--', label='Ref')
axs[0, 0].plot(df_h['time'], df_h['x'], 'b-', label='Actual (MPC)')
axs[0, 0].set_title("X Position")
axs[0, 0].set_xlabel("Time (s)"); axs[0, 0].set_ylabel("x (m)")
axs[0, 0].legend(); axs[0, 0].grid(True)

axs[0, 1].plot(df_h['time'], df_h['y_ref'], 'r--', label='Ref')
axs[0, 1].plot(df_h['time'], df_h['y'], 'b-', label='Actual (MPC)')
axs[0, 1].set_title("Y Position")
axs[0, 1].set_xlabel("Time (s)"); axs[0, 1].set_ylabel("y (m)")
axs[0, 1].legend(); axs[0, 1].grid(True)

axs[1, 0].plot(df_h['time'], df_h['z_ref'], 'r--', label='Ref')
axs[1, 0].plot(df_h['time'], df_h['z'], 'b-', label='Actual (MPC)')
axs[1, 0].set_title("Z Position (Altitude)")
axs[1, 0].set_xlabel("Time (s)"); axs[1, 0].set_ylabel("z (m)")
axs[1, 0].legend(); axs[1, 0].grid(True)

axs[1, 1].plot(df_h['time'], df_h['psi_ref'], 'r--', label='Ref')
axs[1, 1].plot(df_h['time'], df_h['psi'], 'b-', label='Actual (MPC)')
axs[1, 1].set_title("Yaw Angle (psi)")
axs[1, 1].set_xlabel("Time (s)"); axs[1, 1].set_ylabel("psi (rad)")
axs[1, 1].legend(); axs[1, 1].grid(True)

axs[2, 0].plot(df_h['time'], np.zeros_like(df_h['time']), 'r--', label='Ref (0)')
axs[2, 0].plot(df_h['time'], df_h['phi'], 'g-', label='Roll (phi)')
axs[2, 0].set_title("Roll Angle (phi)")
axs[2, 0].set_xlabel("Time (s)"); axs[2, 0].set_ylabel("phi (rad)")
axs[2, 0].legend(); axs[2, 0].grid(True)

axs[2, 1].plot(df_h['time'], np.zeros_like(df_h['time']), 'r--', label='Ref (0)')
axs[2, 1].plot(df_h['time'], df_h['theta'], 'm-', label='Pitch (theta)')
axs[2, 1].set_title("Pitch Angle (theta)")
axs[2, 1].set_xlabel("Time (s)"); axs[2, 1].set_ylabel("theta (rad)")
axs[2, 1].legend(); axs[2, 1].grid(True)

plt.tight_layout()
plt.subplots_adjust(top=0.92)
plt.savefig('results/hover_mpc.png', dpi=300)
plt.close()
print("Saved results/hover_mpc.png")

# 2. Plot Skenario Step
df_s = pd.read_csv('results/step_mpc.csv')
fig, axs = plt.subplots(3, 2, figsize=(12, 10))
fig.suptitle("MPC Golden Model: Skenario Step (Altitude 0 -> 1.5 m)", fontsize=14)

axs[0, 0].plot(df_s['time'], df_s['x_ref'], 'r--', label='Ref')
axs[0, 0].plot(df_s['time'], df_s['x'], 'b-', label='Actual (MPC)')
axs[0, 0].set_title("X Position (Decoupled)")
axs[0, 0].set_xlabel("Time (s)"); axs[0, 0].set_ylabel("x (m)")
axs[0, 0].legend(); axs[0, 0].grid(True)

axs[0, 1].plot(df_s['time'], df_s['y_ref'], 'r--', label='Ref')
axs[0, 1].plot(df_s['time'], df_s['y'], 'b-', label='Actual (MPC)')
axs[0, 1].set_title("Y Position (Decoupled)")
axs[0, 1].set_xlabel("Time (s)"); axs[0, 1].set_ylabel("y (m)")
axs[0, 1].legend(); axs[0, 1].grid(True)

axs[1, 0].plot(df_s['time'], df_s['z_ref'], 'r--', label='Ref (Step 1.5m)')
axs[1, 0].plot(df_s['time'], df_s['z'], 'b-', label='Actual Z (MPC)')
axs[1, 0].set_title("Z Position Step Response")
axs[1, 0].set_xlabel("Time (s)"); axs[1, 0].set_ylabel("z (m)")
axs[1, 0].legend(); axs[1, 0].grid(True)

axs[1, 1].plot(df_s['time'], df_s['u_float0'], 'k-', label='F_total (MPC)')
axs[1, 1].plot(df_s['time'], df_s['u_q16_0'], 'g--', label='F_total (Q16.16)')
axs[1, 1].axhline(y=0.66708 + 0.5, color='gray', linestyle='-.', label='F_max (1.167 N)')
axs[1, 1].set_title("Thrust Force with Actuator Saturation")
axs[1, 1].set_xlabel("Time (s)"); axs[1, 1].set_ylabel("Force (N)")
axs[1, 1].legend(); axs[1, 1].grid(True)

axs[2, 0].plot(df_s['time'], df_s['phi'], 'g-', label='Roll')
axs[2, 0].set_title("Roll Angle")
axs[2, 0].set_xlabel("Time (s)"); axs[2, 0].set_ylabel("rad")
axs[2, 0].legend(); axs[2, 0].grid(True)

axs[2, 1].plot(df_s['time'], df_s['theta'], 'm-', label='Pitch')
axs[2, 1].set_title("Pitch Angle")
axs[2, 1].set_xlabel("Time (s)"); axs[2, 1].set_ylabel("rad")
axs[2, 1].legend(); axs[2, 1].grid(True)

plt.tight_layout()
plt.subplots_adjust(top=0.92)
plt.savefig('results/step_mpc.png', dpi=300)
plt.close()
print("Saved results/step_mpc.png")

# 3. Plot Skenario Trajectory
df_t = pd.read_csv('results/trajectory_mpc.csv')
fig, axs = plt.subplots(3, 2, figsize=(12, 10))
fig.suptitle("MPC Golden Model: Skenario Trajectory 3D Tracking", fontsize=14)

axs[0, 0].plot(df_t['time'], df_t['x_ref'], 'r--', label='Ref X')
axs[0, 0].plot(df_t['time'], df_t['x'], 'b-', label='Actual X (MPC)')
axs[0, 0].set_title("X Position Tracking")
axs[0, 0].set_xlabel("Time (s)"); axs[0, 0].set_ylabel("x (m)")
axs[0, 0].legend(); axs[0, 0].grid(True)

axs[0, 1].plot(df_t['time'], df_t['y_ref'], 'r--', label='Ref Y')
axs[0, 1].plot(df_t['time'], df_t['y'], 'b-', label='Actual Y (MPC)')
axs[0, 1].set_title("Y Position Tracking")
axs[0, 1].set_xlabel("Time (s)"); axs[0, 1].set_ylabel("y (m)")
axs[0, 1].legend(); axs[0, 1].grid(True)

axs[1, 0].plot(df_t['time'], df_t['z_ref'], 'r--', label='Ref Z')
axs[1, 0].plot(df_t['time'], df_t['z'], 'b-', label='Actual Z (MPC)')
axs[1, 0].set_title("Z Position Tracking")
axs[1, 0].set_xlabel("Time (s)"); axs[1, 0].set_ylabel("z (m)")
axs[1, 0].legend(); axs[1, 0].grid(True)

axs[1, 1].plot(df_t['time'], df_t['psi_ref'], 'r--', label='Ref Yaw')
axs[1, 1].plot(df_t['time'], df_t['psi'], 'b-', label='Actual Yaw (MPC)')
axs[1, 1].set_title("Yaw Angle")
axs[1, 1].set_xlabel("Time (s)"); axs[1, 1].set_ylabel("rad")
axs[1, 1].legend(); axs[1, 1].grid(True)

axs[2, 0].plot(df_t['time'], df_t['phi'], 'g-', label='Roll (phi)')
axs[2, 0].set_title("Roll Angle (Dynamic Tilting)")
axs[2, 0].set_xlabel("Time (s)"); axs[2, 0].set_ylabel("rad")
axs[2, 0].legend(); axs[2, 0].grid(True)

axs[2, 1].plot(df_t['time'], df_t['theta'], 'm-', label='Pitch (theta)')
axs[2, 1].set_title("Pitch Angle (Dynamic Tilting)")
axs[2, 1].set_xlabel("Time (s)"); axs[2, 1].set_ylabel("rad")
axs[2, 1].legend(); axs[2, 1].grid(True)

plt.tight_layout()
plt.subplots_adjust(top=0.92)
plt.savefig('results/trajectory_mpc.png', dpi=300)
plt.close()
print("Saved results/trajectory_mpc.png")

# 4. Plot Trajectory 3D Isometric View
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(df_t['x_ref'], df_t['y_ref'], df_t['z_ref'], 'r--', label='3D Reference Trajectory', linewidth=2)
ax.plot(df_t['x'], df_t['y'], df_t['z'], 'b-', label='Actual Flight Path (MPC Golden Model)', linewidth=1.5)
ax.set_title("3D Helical Trajectory Tracking: Reference vs MPC Golden Model", fontsize=14)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.legend()
plt.tight_layout()
plt.savefig('results/trajectory_3d_mpc.png', dpi=300)
plt.close()
print("Saved results/trajectory_3d_mpc.png")
