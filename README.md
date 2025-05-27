# JSBSim Rocket Simulation

This project simulates a small suborbital amateur rocket using JSBSim flight dynamics engine.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [JSBSim Rocket Simulation](#jsbsim-rocket-simulation)
  - [Rocket Specifications](#rocket-specifications)
  - [Center of Gravity and Pressure](#center-of-gravity-and-pressure)
  - [Build Requirements](#build-requirements)
  - [Compilation](#compilation)
  - [Running the Simulation](#running-the-simulation)
  - [Output Files](#output-files)
  - [Analysis and Plotting](#analysis-and-plotting)
    - [Example Output](#example-output)
  - [Aircraft Configuration](#aircraft-configuration)
  - [Motor Characteristics](#motor-characteristics)
  - [Recovery System](#recovery-system)

<!-- markdown-toc end -->

## Rocket Specifications

- **Diameter**: 6 inches
- **Length**: 109 inches  
- **Dry Mass**: 43.0 lbs (including recovery system)
- **Wet Mass**: 46.9 lbs (with motor propellant)
- **Motor**: Cesaroni L1720 solid rocket motor
- **Expected Apogee**: ~4000 feet
- **Recovery**: Two-stage parachute system
  - Drogue chute at apogee
  - Main chute deployment at 500 feet

## Center of Gravity and Pressure

- **Center of Gravity**: 54.9 inches from nose
- **Center of Pressure**: 68.1 inches from nose  
- **Static Stability Margin**: 2.20 calibers (on pad), 2.28 calibers (rail exit)

## Build Requirements

- JSBSim library and headers
- C++ compiler with C++11 support
- CMake (optional)

## Compilation

```bash
./build.sh
```

## Running the Simulation

```bash
./build/rocket_sim
```

The simulation will:
1. Initialize the rocket on the launch pad in vertical orientation
2. Ignite the motor at t=0.1 seconds
3. Track liftoff when velocity > 0 and altitude > 10 feet
4. Deploy drogue chute at apogee detection (velocity becomes negative)
5. Deploy main chute at 500 feet AGL
6. Output trajectory data to `rocket_trajectory.csv`

## Output Files

- `rocket_trajectory.csv`: Time-series data of altitude, velocity, and parachute states
- Console output: Real-time simulation status and key events

## Analysis and Plotting

### Example Output

![CleanShot 2025-05-26 at 17 13 17](https://github.com/user-attachments/assets/fa76638b-1a8d-4b06-a082-6342d4413c31)

```console
(env) ➜  jsbsim-rocket-test git:(rocket-test) ✗ python analyze_trajectory.py                                                                                                                                                                               [25/05/26| 5:13PM]
Loaded 8652 data points from rocket_trajectory.csv

==================================================
🚀 ROCKET TRAJECTORY ANALYSIS
==================================================

📊 FLIGHT PERFORMANCE:
  Max Altitude:       3163.0 ft at t=14.6s
  Max Velocity:        460.1 ft/s at t=2.1s
  Total Flight Time:    72.1 s
  Landing Velocity:    -26.1 ft/s

🪂 PARACHUTE DEPLOYMENT:
  Drogue Deploy:    t=  15.2s at   3157 ft
  Main Deploy:      t=  53.7s at    489 ft

📍 TRAJECTORY ANALYSIS:
  Launch Position:   X=0.0 ft, Y=0.0 ft
  Landing Position:  X=-569.8 ft, Y=  -3.1 ft
  Horizontal Drift:   569.8 ft
  Max Drift:          920.6 ft

⬇️  DESCENT RATES:
  Drogue Descent:     69.4 ft/s
  Main Descent:       26.9 ft/s

⏱️  FLIGHT PHASES:
  Powered/Coast:    0.0s - 15.2s (15.2s)
  Drogue Descent:   15.2s - 53.7s (38.4s)
  Main Descent:     53.7s - 72.1s (18.4s)

📋 TRAJECTORY SAMPLE (every 10 seconds):
  Time        X        Y        Z      Vel Phase
   (s)     (ft)     (ft)     (ft)   (ft/s)
--------------------------------------------------
   0.0     -0.0      0.0      0.0     -0.3 Launch
  10.0   -532.9     -1.3   2817.6    151.2 Launch
  20.0   -919.8     -3.7   2897.2    -70.4 Drogue
  30.0   -870.4     -3.7   2175.4    -72.1 Drogue
  40.0   -799.8     -3.5   1457.9    -71.4 Drogue
  50.0   -729.0     -3.3    747.8    -70.6 Drogue
  60.0   -657.2     -3.2    310.1    -26.2 Main
  70.0   -584.9     -3.2     48.8    -26.1 Main
  72.1   -569.8     -3.1     -5.7    -26.1 Land
```

## Aircraft Configuration

The JSBSim aircraft configuration files are located in:
- `aircraft/rocket/rocket.xml` - Main aircraft definition
- `aircraft/rocket/Engines/cesaroni_l1720_engine.xml` - Motor thrust curve
- `aircraft/rocket/Engines/l1720_nozzle.xml` - Nozzle specifications

## Motor Characteristics

The Cesaroni L1720 motor simulation includes:
- **Total Impulse**: 1720 N⋅s (386 lbf⋅s)
- **Burn Time**: ~2.1 seconds  
- **Average Thrust**: 121 lbf
- **Peak Thrust**: 157 lbf
- **Specific Impulse**: 220 seconds

## Recovery System

- **Drogue Chute**: 8 ft² effective drag area, deploys at apogee
- **Main Chute**: 50 ft² effective drag area, deploys at 500 feet AGL
- Dual-deployment system ensures controlled descent rate
