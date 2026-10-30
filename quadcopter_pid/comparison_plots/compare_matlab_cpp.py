import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Configuration
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)

MATLAB_DIR = os.path.join(PROJECT_DIR, 'matlab_pid_sim', 'results')
CPP_DIR = SCRIPT_DIR
PLOT_DIR = SCRIPT_DIR
SUMMARY_FILE = os.path.join(SCRIPT_DIR, 'comparison_summary.csv')

# Tolerance configuration 
# Tolerance RMSE is 1% of the reference range, but bounded to a minimum of 1e-4 
# so we don't trigger false alarms on signals that are meant to stay at zero.
TOLERANCE_PERCENT = 0.01
MIN_TOLERANCE = 1e-4

os.makedirs(PLOT_DIR, exist_ok=True)

scenarios = ['hover', 'step', 'trajectory', 'trajectory_okasha']
channels = [
    {'col': 'x', 'ref': 'x_ref', 'name': 'X Position', 'unit': 'm'},
    {'col': 'y', 'ref': 'y_ref', 'name': 'Y Position', 'unit': 'm'},
    {'col': 'z', 'ref': 'z_ref', 'name': 'Z Position', 'unit': 'm'},
    {'col': 'psi', 'ref': 'psi_ref', 'name': 'Yaw', 'unit': 'rad'},
    {'col': 'phi', 'ref': 'phi', 'name': 'Roll', 'unit': 'rad'},  
    {'col': 'theta', 'ref': 'theta', 'name': 'Pitch', 'unit': 'rad'} 
]

summary_data = []
warnings = []

for scenario in scenarios:
    matlab_file = os.path.join(MATLAB_DIR, f"{scenario}.csv")
    cpp_file = os.path.join(CPP_DIR, f"{scenario}.csv")
    
    if not os.path.exists(matlab_file) or not os.path.exists(cpp_file):
        print(f"Skipping {scenario}, missing files.")
        continue
        
    df_matlab = pd.read_csv(matlab_file)
    df_cpp = pd.read_csv(cpp_file)
    
    t_m = df_matlab['time'].values
    t_c = df_cpp['time'].values
    
    fig, axs = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle(f"Comparison: {scenario.capitalize()} Scenario", fontsize=16)
    
    for i, ch in enumerate(channels):
        ax = axs[i // 2, i % 2]
        col = ch['col']
        ref_col = ch['ref']
        
        # Plotting
        if ref_col in df_matlab.columns and 'ref' in ref_col:
            ax.plot(t_m, df_matlab[ref_col], 'r--', label='Reference')
            ref_vals = df_matlab[ref_col].values
        else:
            # If no ref column (e.g., Roll/Pitch), reference is 0
            ax.plot(t_m, np.zeros_like(t_m), 'r--', label='Reference (0)')
            ref_vals = np.zeros_like(t_m)
            
        ax.plot(t_m, df_matlab[col], 'b-', label='MATLAB')
        ax.plot(t_c, df_cpp[col], 'g--', linewidth=1, label='C++')
        
        ax.set_title(ch['name'])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(ch['unit'])
        ax.legend()
        ax.grid(True)
        
        # Metrics Calculation
        # Interpolate C++ data onto MATLAB time steps to synchronize
        cpp_interp = np.interp(t_m, t_c, df_cpp[col].values)
        diff = df_matlab[col].values - cpp_interp
        
        rmse = np.sqrt(np.mean(diff**2))
        max_diff = np.max(np.abs(diff))
        
        summary_data.append({
            'Skenario': scenario,
            'Channel': ch['name'],
            'RMSE': rmse,
            'Max Diff': max_diff
        })
        
        # Tolerance check
        ref_range = np.max(ref_vals) - np.min(ref_vals)
        tol = max(MIN_TOLERANCE, TOLERANCE_PERCENT * ref_range)
        if rmse > tol:
            warnings.append(f"WARNING: Skenario '{scenario}' - Channel '{ch['name']}' RMSE melebihi batas toleransi! (RMSE={rmse:.6f}, Tol={tol:.6f})")

    plt.tight_layout()
    plt.subplots_adjust(top=0.9)
    fig.savefig(os.path.join(PLOT_DIR, f"{scenario}_comparison.png"))

# Save summary
df_summary = pd.DataFrame(summary_data)
df_summary.to_csv(SUMMARY_FILE, index=False)

# Print Summary
print("\n" + "="*65)
print(" COMPARISON METRICS SUMMARY (MATLAB vs C++)")
print("="*65)
print(df_summary.to_string(index=False))
print("="*65 + "\n")

if warnings:
    print("!!! PERINGATAN TOLERANSI !!!")
    for w in warnings:
        print(w)
else:
    print("SUCCESS: Semua channel pada semua skenario berada di bawah ambang batas toleransi.")

# Simpan seluruh plot tanpa blocking GUI
plt.close('all')
