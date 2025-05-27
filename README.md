# JSBSim Rocket Simulation

This project simulates a small suborbital amateur rocket using JSBSim flight dynamics engine.

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
