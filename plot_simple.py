#!/usr/bin/env python3
"""
Simple 3D Rocket Trajectory Plotter
Uses only built-in Python libraries + matplotlib
"""

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def plot_rocket_trajectory(csv_file='rocket_trajectory.csv'):
    """
    Plot 3D trajectory of rocket flight from CSV data
    """
    # Read trajectory data
    try:
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            data = list(reader)
        print(f"Loaded {len(data)} data points from {csv_file}")
    except FileNotFoundError:
        print(f"Error: {csv_file} not found. Run the rocket simulation first.")
        return
    
    # Convert data to lists
    time = [float(row['Time']) for row in data]
    x_ft = [float(row['X_ft']) for row in data]
    y_ft = [float(row['Y_ft']) for row in data]
    z_ft = [float(row['Z_ft']) for row in data]
    altitude = [float(row['Altitude']) for row in data]
    vertical_velocity = [float(row['Vertical_Velocity']) for row in data]
    drogue = [int(row['Drogue_Deployed']) for row in data]
    main = [int(row['Main_Deployed']) for row in data]
    
    # Create figure with subplots
    fig = plt.figure(figsize=(15, 10))
    
    # 3D trajectory plot
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.plot(x_ft, y_ft, z_ft, 'b-', linewidth=2, alpha=0.8, label='Flight path')
    
    # Mark key points
    ax1.scatter([0], [0], [0], color='black', s=100, marker='o', label='Launch')
    ax1.scatter([x_ft[-1]], [y_ft[-1]], [z_ft[-1]], color='red', s=100, marker='X', label='Landing')
    
    # Mark apogee
    max_alt_idx = z_ft.index(max(z_ft))
    ax1.scatter([x_ft[max_alt_idx]], [y_ft[max_alt_idx]], [z_ft[max_alt_idx]], 
               color='gold', s=150, marker='*', label=f'Apogee ({max(z_ft):.0f} ft)')
    
    ax1.set_xlabel('X (North) [ft]')
    ax1.set_ylabel('Y (East) [ft]') 
    ax1.set_zlabel('Z (Up) [ft]')
    ax1.set_title('3D Rocket Trajectory')
    ax1.legend()
    
    # Altitude vs Time
    ax2 = fig.add_subplot(222)
    ax2.plot(time, z_ft, 'b-', linewidth=2, label='Altitude')
    ax2.axhline(y=max(z_ft), color='r', linestyle='--', alpha=0.7, label=f'Max Alt: {max(z_ft):.0f} ft')
    
    # Mark parachute deployments
    drogue_time = None
    main_time = None
    for i in range(1, len(drogue)):
        if drogue[i] == 1 and drogue[i-1] == 0:
            drogue_time = time[i]
            ax2.axvline(x=drogue_time, color='orange', linestyle='--', alpha=0.7, label='Drogue Deploy')
            break
    
    for i in range(1, len(main)):
        if main[i] == 1 and main[i-1] == 0:
            main_time = time[i]
            ax2.axvline(x=main_time, color='green', linestyle='--', alpha=0.7, label='Main Deploy')
            break
    
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Altitude [ft]')
    ax2.set_title('Altitude vs Time')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Velocity vs Time
    ax3 = fig.add_subplot(223)
    ax3.plot(time, vertical_velocity, 'g-', linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Velocity [ft/s]')
    ax3.set_title('Velocity vs Time')
    ax3.grid(True, alpha=0.3)
    
    # Horizontal displacement
    ax4 = fig.add_subplot(224)
    horizontal_dist = [np.sqrt(x**2 + y**2) for x, y in zip(x_ft, y_ft)]
    ax4.plot(time, horizontal_dist, 'm-', linewidth=2)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Horizontal Distance [ft]')
    ax4.set_title('Horizontal Drift vs Time')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Print flight statistics
    print(f"\n=== FLIGHT STATISTICS ===")
    print(f"Max Altitude: {max(z_ft):.1f} ft")
    print(f"Flight Time: {time[-1]:.1f} s")
    print(f"Max Velocity: {max(vertical_velocity):.1f} ft/s")
    print(f"Horizontal Drift: {horizontal_dist[-1]:.1f} ft")
    
    if drogue_time:
        drogue_idx = time.index(drogue_time)
        print(f"Drogue Deploy: {drogue_time:.1f} s at {z_ft[drogue_idx]:.0f} ft")
    if main_time:
        main_idx = time.index(main_time)
        print(f"Main Deploy: {main_time:.1f} s at {z_ft[main_idx]:.0f} ft")
    
    plt.show()

if __name__ == "__main__":
    plot_rocket_trajectory() 