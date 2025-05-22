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
g++ -std=c++11 -I/path/to/jsbsim/include -L/path/to/jsbsim/lib \
    rocket_sim.cpp -ljsbsim -o rocket_sim
```

## Running the Simulation

```bash
./rocket_sim
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
- `aircraft/my_aircraft/my_aircraft.xml` - Main aircraft definition
- `aircraft/my_aircraft/Engines/cesaroni_l1720_engine.xml` - Motor thrust curve
- `aircraft/my_aircraft/Engines/l1720_nozzle.xml` - Nozzle specifications

## Motor Characteristics

The Cesaroni L1720 motor simulation includes:
- **Total Impulse**: 1720 N⋅s (386 lbf⋅s)
- **Burn Time**: ~3.2 seconds  
- **Average Thrust**: 121 lbf
- **Peak Thrust**: 157 lbf
- **Specific Impulse**: 220 seconds

## Recovery System

- **Drogue Chute**: 8 ft² effective drag area, deploys at apogee
- **Main Chute**: 50 ft² effective drag area, deploys at 500 feet AGL
- Dual-deployment system ensures controlled descent rate
