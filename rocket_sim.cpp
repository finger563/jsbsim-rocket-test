#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <cmath>
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGInertial.h>
#include <models/FGPropulsion.h>
#include <models/FGMassBalance.h>
#include <models/FGFCS.h>
#include <models/propulsion/FGTank.h>

int main() {
    // Create an instance of the JSBSim flight dynamics model executor
    std::unique_ptr<JSBSim::FGFDMExec> fdmExec(new JSBSim::FGFDMExec());

    // Set the simulation to run at 120 Hz
    fdmExec->Setdt(1.0 / 120.0);

    // Load the aircraft configuration file
    if (!fdmExec->LoadModel("my_aircraft")) {
        std::cerr << "Failed to load the aircraft model" << std::endl;
        return 1;
    }

    // Initialize the JSBSim model
    if (!fdmExec->GetPropagate()->InitModel()) {
        std::cerr << "Failed to initialize the aircraft model" << std::endl;
        return 1;
    }

    // Set initial conditions for vertical launch - CORRECTED ORIENTATION
    fdmExec->GetIC()->SetAltitudeASLFtIC(10.5);    // Start slightly higher to avoid ground contact
    fdmExec->GetIC()->SetLatitudeDegIC(37.0);
    fdmExec->GetIC()->SetLongitudeDegIC(-122.0);
    fdmExec->GetIC()->SetThetaDegIC(90.0);         // Nose up orientation (+90 deg pitch) so +X points up
    fdmExec->GetIC()->SetPhiDegIC(0.0);            // No roll
    fdmExec->GetIC()->SetPsiDegIC(0.0);            // No yaw
    fdmExec->GetIC()->SetVNorthFpsIC(0.0);
    fdmExec->GetIC()->SetVEastFpsIC(0.0);
    fdmExec->GetIC()->SetVDownFpsIC(0.0);          // No initial velocity
    fdmExec->GetIC()->SetPRadpsIC(0.0);            // No roll rate
    fdmExec->GetIC()->SetQRadpsIC(0.0);            // No pitch rate
    fdmExec->GetIC()->SetRRadpsIC(0.0);            // No yaw rate

    // Initialize FCS properties
    fdmExec->SetPropertyValue("fcs/elevator-cmd-norm", 0.0);
    fdmExec->SetPropertyValue("fcs/pitch-trim-cmd-norm", 0.0);
    fdmExec->SetPropertyValue("fcs/aileron-cmd-norm", 0.0);
    fdmExec->SetPropertyValue("fcs/rudder-cmd-norm", 0.0);

    // Initialize variables for parachute deployment
    bool drogue_deployed = false;
    bool main_deployed = false;
    double max_altitude = 0.0;
    bool reached_apogee = false;

    // Initialize the model
    fdmExec->RunIC();

    // Open an output file to save the trajectory data
    std::ofstream outputFile("rocket_trajectory.csv");
    outputFile << "Time,Altitude,Velocity,Drogue_Deployed,Main_Deployed\n";

    // Initialize motor ignition sequence
    bool motor_ignited = false;
    double ignition_time = 0.05; // Quick ignition after simulation start
    
    std::cout << "Starting L1720 rocket simulation - target apogee ~4000 ft" << std::endl;
    std::cout << "Motor ignition scheduled for t=" << ignition_time << " seconds" << std::endl;

    float liftoff_threshold_agl = 10.0f;
    bool did_liftoff = false;

    // Run the simulation loop - extended for high altitude flight
    while (fdmExec->GetSimTime() < 120.0 && fdmExec->GetPropagate()->GetAltitudeASL() >= -10.0) {
        // Run the JSBSim simulation for one time step
        fdmExec->Run();

        // Get current state
        double time = fdmExec->GetSimTime();
        double altitude = fdmExec->GetPropagate()->GetAltitudeASL();
        
        // Get velocity components for proper apogee detection
        double vx = fdmExec->GetPropagate()->GetVel(1); // X velocity (forward/aft)
        double vy = fdmExec->GetPropagate()->GetVel(2); // Y velocity (left/right)  
        double vz = fdmExec->GetPropagate()->GetVel(3); // Z velocity (up/down)
        
        // For vertical rocket, use velocity magnitude for display but VZ for apogee detection
        double velocity_magnitude = sqrt(vx*vx + vy*vy + vz*vz);
        double vertical_velocity = -vz; // In JSBSim: positive Z is down, so -Z is up
        
        // Simple velocity debugging during motor burn only
        if (motor_ignited && time < 1.0 && fmod(time, 0.5) < 0.01) {
            std::cout << "DEBUG: Altitude=" << altitude << "ft, Vertical_vel=" << vertical_velocity << "ft/s" << std::endl;
        }

        // Ignite motor at scheduled time using throttle setting for solid rockets
        if (!motor_ignited && time >= ignition_time) {
            std::cout << "Igniting engine at t=" << time << "s" << std::endl;
            fdmExec->GetFCS()->SetThrottleCmd(0, 1.0);  // Set throttle to 100% for solid rocket ignition
            auto engine = fdmExec->GetPropulsion()->GetEngine(0);
            engine->SetRunning(true);  // Also set engine to running state
            
            // Disable ground contacts during powered flight to prevent false contact detection
            // Try multiple property naming conventions
            fdmExec->SetPropertyValue("gear/unit[0]/spring-coeff-lbs_ft", 0.0);  
            fdmExec->SetPropertyValue("gear/unit[1]/spring-coeff-lbs_ft", 0.0);  
            fdmExec->SetPropertyValue("gear/unit[0]/damping-coeff-lbs_ft_sec", 0.0);  
            fdmExec->SetPropertyValue("gear/unit[1]/damping-coeff-lbs_ft_sec", 0.0);  
            
            // Also try moving contact points far away
            fdmExec->SetPropertyValue("gear/unit[0]/location-z-in", -1000.0);  // Move launch rail far down
            fdmExec->SetPropertyValue("gear/unit[1]/location-z-in", -1000.0);  // Move nose contact far down
            
            // Alternative property names
            fdmExec->SetPropertyValue("contact/unit[0]/spring-coeff-lbs_ft", 0.0);
            fdmExec->SetPropertyValue("contact/unit[1]/spring-coeff-lbs_ft", 0.0);
            
            std::cout << "Ground contacts disabled for powered flight" << std::endl;
            
            motor_ignited = true;
        }

        // Debug output for engine state during motor burn phase
        if (motor_ignited && time < 5.0) {
            if (time - ignition_time < 0.2 || fmod(time, 0.5) < 0.01) {  // Show for first 0.2s, then every 0.5s
                auto engine = fdmExec->GetPropulsion()->GetEngine(0);
                double throttle = fdmExec->GetFCS()->GetThrottlePos(0);
                double propellant_remaining = fdmExec->GetPropulsion()->GetTank(0)->GetContents();
                double propellant_consumed = 3.9 - propellant_remaining;
                double burn_percentage = (propellant_consumed / 3.9) * 100.0;
                
                std::cout << "t=" << std::fixed << std::setprecision(2) << time << "s: "
                          << "Thrust=" << std::setprecision(1) << engine->GetThrust() << "lbs, " 
                          << "Fuel=" << std::setprecision(2) << propellant_remaining << "lbs, "
                          << "Burn=" << std::setprecision(1) << burn_percentage << "%" << std::endl;
            }
        }

        if (velocity_magnitude > 0.0f && altitude > liftoff_threshold_agl && !did_liftoff) {
            std::cout << "Liftoff detected at " << altitude << " ft" << std::endl;
            did_liftoff = true;
        }

        // Track maximum altitude and detect apogee more robustly
        if (did_liftoff && altitude > max_altitude) {
            max_altitude = altitude;
        }
        
        // Detect apogee when vertical velocity becomes negative after liftoff
        if (did_liftoff && !reached_apogee && vertical_velocity < -20.0 && altitude > 200.0) { // Very conservative thresholds
            reached_apogee = true;
            std::cout << "Apogee reached at " << max_altitude << " ft (current alt: " << altitude << " ft)" << std::endl;
        }

        // Deploy drogue chute at apogee (but only if above minimum altitude)
        // RE-ENABLED WITH FIXED LOGIC
        if (reached_apogee && !drogue_deployed && altitude > 800.0) { // Very high altitude threshold
            fdmExec->SetPropertyValue("external_reactions/drogue_chute/drag_area", 8.0); // Drogue chute drag area
            drogue_deployed = true;
            std::cout << "Drogue chute deployed at " << altitude << " ft" << std::endl;
        }

        // Deploy main chute below 500 feet (per requirements) 
        if (reached_apogee && !main_deployed && altitude < 500.0) {
            fdmExec->SetPropertyValue("external_reactions/main_chute/drag_area", 50.0); // Larger main chute
            main_deployed = true;
            std::cout << "Main chute deployed at " << altitude << " ft" << std::endl;
        }

        // Print and save trajectory data with improved output formatting
        if (fmod(time, 0.2) < 0.01) {  // Print every 0.2 seconds for detailed tracking
            std::cout << std::fixed << std::setprecision(1);
            std::cout << "t=" << time << "s: Alt=" << altitude << "ft, Vel=" << velocity_magnitude << "ft/s";
            
            // Show flight phase information
            if (!did_liftoff) {
                std::cout << " [ON PAD]";
            } else if (motor_ignited && time < 4.0) {
                std::cout << " [POWERED FLIGHT]";
            } else if (!reached_apogee) {
                std::cout << " [COASTING UP]";
            } else if (!drogue_deployed) {
                std::cout << " [FALLING]";
            } else if (!main_deployed) {
                std::cout << " [DROGUE DESCENT]";
            } else {
                std::cout << " [MAIN CHUTE]";
            }
            std::cout << std::endl;
        }
        outputFile << time << "," << altitude << "," << velocity_magnitude << "," << drogue_deployed << "," << main_deployed << "\n";

        if (did_liftoff && reached_apogee && altitude < 5.0) {  // Only terminate after apogee and very low altitude
            std::cout << "Rocket has reached the ground after flight." << std::endl;
            break;
        }
    }

    std::cout << "Simulation complete." << std::endl;
    std::cout << "Maximum altitude reached: " << max_altitude << " ft" << std::endl;
    std::cout << "Time to reach the ground: " << fdmExec->GetSimTime() << " s" << std::endl;

    outputFile.close();
    return 0;
}
