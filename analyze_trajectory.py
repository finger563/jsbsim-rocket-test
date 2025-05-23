#!/usr/bin/env python3
"""
Simple Rocket Trajectory Analyzer
Analyzes CSV trajectory data without requiring external libraries
"""

import csv
import math

def analyze_trajectory(csv_file='rocket_trajectory.csv'):
    """
    Analyze rocket trajectory from CSV data and print statistics
    """
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
    
    # Calculate key statistics
    max_altitude = max(z_ft)
    max_alt_idx = z_ft.index(max_altitude)
    max_alt_time = time[max_alt_idx]
    
    max_velocity = max(vertical_velocity)
    max_vel_idx = vertical_velocity.index(max_velocity)
    max_vel_time = time[max_vel_idx]
    
    flight_time = time[-1]
    
    # Calculate horizontal drift
    horizontal_dist = [math.sqrt(x**2 + y**2) for x, y in zip(x_ft, y_ft)]
    max_horizontal_drift = max(horizontal_dist)
    final_horizontal_drift = horizontal_dist[-1]
    
    # Find parachute deployment times
    drogue_time = None
    main_time = None
    drogue_altitude = None
    main_altitude = None
    
    for i in range(1, len(drogue)):
        if drogue[i] == 1 and drogue[i-1] == 0:
            drogue_time = time[i]
            drogue_altitude = z_ft[i]
            break
    
    for i in range(1, len(main)):
        if main[i] == 1 and main[i-1] == 0:
            main_time = time[i]
            main_altitude = z_ft[i]
            break
    
    # Print comprehensive analysis
    print(f"\n{'='*50}")
    print(f"🚀 ROCKET TRAJECTORY ANALYSIS")
    print(f"{'='*50}")
    
    print(f"\n📊 FLIGHT PERFORMANCE:")
    print(f"  Max Altitude:     {max_altitude:8.1f} ft at t={max_alt_time:.1f}s")
    print(f"  Max Velocity:     {max_velocity:8.1f} ft/s at t={max_vel_time:.1f}s")
    print(f"  Total Flight Time: {flight_time:7.1f} s")
    print(f"  Landing Velocity:  {vertical_velocity[-1]:7.1f} ft/s")
    
    print(f"\n🪂 PARACHUTE DEPLOYMENT:")
    if drogue_time:
        print(f"  Drogue Deploy:    t={drogue_time:6.1f}s at {drogue_altitude:6.0f} ft")
    if main_time:
        print(f"  Main Deploy:      t={main_time:6.1f}s at {main_altitude:6.0f} ft")
    
    print(f"\n📍 TRAJECTORY ANALYSIS:")
    print(f"  Launch Position:   X=0.0 ft, Y=0.0 ft")
    print(f"  Landing Position:  X={x_ft[-1]:6.1f} ft, Y={y_ft[-1]:6.1f} ft")
    print(f"  Horizontal Drift:  {final_horizontal_drift:6.1f} ft")
    print(f"  Max Drift:         {max_horizontal_drift:6.1f} ft")
    
    # Calculate descent rates
    if drogue_time and main_time:
        drogue_idx = time.index(drogue_time)
        main_idx = time.index(main_time)
        
        # Average descent rate with drogue
        drogue_alt_drop = z_ft[drogue_idx] - z_ft[main_idx]
        drogue_time_period = main_time - drogue_time
        drogue_descent_rate = drogue_alt_drop / drogue_time_period
        
        # Average descent rate with main chute
        main_alt_drop = z_ft[main_idx] - z_ft[-1]
        main_time_period = flight_time - main_time
        main_descent_rate = main_alt_drop / main_time_period
        
        print(f"\n⬇️  DESCENT RATES:")
        print(f"  Drogue Descent:   {drogue_descent_rate:6.1f} ft/s")
        print(f"  Main Descent:     {main_descent_rate:6.1f} ft/s")
    
    # Flight phases
    print(f"\n⏱️  FLIGHT PHASES:")
    if drogue_time:
        print(f"  Powered/Coast:    0.0s - {drogue_time:.1f}s ({drogue_time:.1f}s)")
        if main_time:
            print(f"  Drogue Descent:   {drogue_time:.1f}s - {main_time:.1f}s ({main_time-drogue_time:.1f}s)")
            print(f"  Main Descent:     {main_time:.1f}s - {flight_time:.1f}s ({flight_time-main_time:.1f}s)")
    
    # Sample some trajectory points for verification
    print(f"\n📋 TRAJECTORY SAMPLE (every 10 seconds):")
    print(f"{'Time':>6} {'X':>8} {'Y':>8} {'Z':>8} {'Vel':>8} {'Phase'}")
    print(f"{'(s)':>6} {'(ft)':>8} {'(ft)':>8} {'(ft)':>8} {'(ft/s)':>8}")
    print(f"{'-'*50}")
    
    for i, t in enumerate(time):
        if i % 1200 == 0 or i == len(time)-1:  # Every 10 seconds (120Hz * 10s = 1200 samples)
            phase = "Launch"
            if drogue_time and t >= drogue_time:
                phase = "Drogue"
            if main_time and t >= main_time:
                phase = "Main"
            if i == len(time)-1:
                phase = "Land"
                
            print(f"{t:6.1f} {x_ft[i]:8.1f} {y_ft[i]:8.1f} {z_ft[i]:8.1f} {vertical_velocity[i]:8.1f} {phase}")

if __name__ == "__main__":
    analyze_trajectory() 