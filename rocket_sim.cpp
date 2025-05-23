#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <cmath>
#define _USE_MATH_DEFINES
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGInertial.h>
#include <models/FGPropulsion.h>
#include <models/FGMassBalance.h>
#include <models/FGFCS.h>
#include <models/propulsion/FGTank.h>
#include <models/FGAuxiliary.h>

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
    fdmExec->GetIC()->SetThetaDegIC(89.95);        // Very slightly off vertical to avoid numerical singularities
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
    
    // Enable realistic atmospheric turbulence and wind for final testing
    fdmExec->SetPropertyValue("atmosphere/turb-rate", 0.1);      // Moderate turbulence
    fdmExec->SetPropertyValue("atmosphere/turb-gain", 1.0);      // Normal gain
    fdmExec->SetPropertyValue("atmosphere/wind-north-fps", 7.33); // 5 mph north wind (7.33 ft/s)
    fdmExec->SetPropertyValue("atmosphere/wind-east-fps", 0.0);   // No east wind
    fdmExec->SetPropertyValue("atmosphere/wind-down-fps", 0.0);   // No vertical wind
    
    // Debug: Check current wind conditions
    std::cout << "\n=== REALISTIC ATMOSPHERIC CONDITIONS ===" << std::endl;
    std::cout << "Wind North: " << fdmExec->GetPropertyValue("atmosphere/wind-north-fps") << " fps (5 mph)" << std::endl;
    std::cout << "Wind East:  " << fdmExec->GetPropertyValue("atmosphere/wind-east-fps") << " fps" << std::endl;
    std::cout << "Wind Down:  " << fdmExec->GetPropertyValue("atmosphere/wind-down-fps") << " fps" << std::endl;
    std::cout << "Wind Mag:   " << fdmExec->GetPropertyValue("atmosphere/wind-mag-fps") << " fps" << std::endl;
    std::cout << "Turb Rate:  " << fdmExec->GetPropertyValue("atmosphere/turb-rate") << std::endl;
    std::cout << "=========================================" << std::endl;

    // Open an output file to save the trajectory data
    std::ofstream outputFile("rocket_trajectory.csv");
    outputFile << "Time,X_ft,Y_ft,Z_ft,Altitude,Vertical_Velocity,Drogue_Deployed,Main_Deployed\n";

    // Initialize motor ignition sequence
    bool motor_ignited = false;
    bool engine_shutdown = false;  // Track engine shutdown state
    double ignition_time = 0.05; // Quick ignition after simulation start
    double total_impulse = 0.0;  // Track total impulse delivered
    double last_time = 0.0;      // For impulse integration
    
    // Store initial position for 3D trajectory tracking
    double initial_latitude = 37.0;  // Launch latitude
    double initial_longitude = -122.0; // Launch longitude
    double initial_altitude = 10.5;   // Launch altitude
    
    std::cout << "Starting L1720 rocket simulation - real manufacturer thrust curve data" << std::endl;
    std::cout << "Real L1720: 437.4 lbf peak thrust, 2.1s burn, regressive profile" << std::endl;
    std::cout << "Expected apogee from manufacturer: 3812 ft" << std::endl;
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
        
        // Check for numerical divergence and terminate gracefully
        if (velocity_magnitude > 10000.0 || altitude > 100000.0 || std::isnan(velocity_magnitude) || std::isnan(altitude)) {
            std::cout << "ERROR: Numerical divergence detected!" << std::endl;
            std::cout << "Time=" << time << "s, Alt=" << altitude << "ft, Vel=" << velocity_magnitude << "ft/s" << std::endl;
            std::cout << "VX=" << vx << " VY=" << vy << " VZ=" << vz << std::endl;
            std::cout << "Vertical_vel=" << vertical_velocity << "ft/s" << std::endl;
            std::cout << "Terminating simulation to prevent crash..." << std::endl;
            break;
        }

        // Simple velocity debugging during motor burn only
        if (motor_ignited && time < 1.0 && fmod(time, 0.5) < 0.01) {
            std::cout << "DEBUG: Altitude=" << altitude << "ft, Vertical_vel=" << vertical_velocity << "ft/s" << std::endl;
        }

        // Debug angular orientation during early flight
        if (time < 5.0 && fmod(time, 0.2) < 0.01) {
            double pitch_deg = fdmExec->GetPropagate()->GetEuler(2) * 180.0 / 3.14159; // Theta (pitch)
            double yaw_deg = fdmExec->GetPropagate()->GetEuler(3) * 180.0 / 3.14159;   // Psi (yaw) 
            double roll_deg = fdmExec->GetPropagate()->GetEuler(1) * 180.0 / 3.14159;  // Phi (roll)
            
            std::cout << "ORIENTATION t=" << std::fixed << std::setprecision(2) << time 
                      << "s: Pitch=" << std::setprecision(1) << pitch_deg 
                      << "°, Yaw=" << yaw_deg << "°, Roll=" << roll_deg << "°" << std::endl;
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
        if (motor_ignited && time < 3.0) {  // Shortened from 5.0s to cover the 2.1s burn + coast
            if (time - ignition_time < 0.2 || fmod(time, 0.5) < 0.01) {  // Show for first 0.2s, then every 0.5s
                auto engine = fdmExec->GetPropulsion()->GetEngine(0);
                double throttle = fdmExec->GetFCS()->GetThrottlePos(0);
                double propellant_remaining = fdmExec->GetPropulsion()->GetTank(0)->GetContents();
                double propellant_consumed = 3.9 - propellant_remaining;
                double burn_percentage = (propellant_consumed / 3.9) * 100.0;
                
                // Integrate impulse
                double dt = time - last_time;
                if (dt > 0) {
                    total_impulse += engine->GetThrust() * dt;
                }
                
                std::cout << "t=" << std::fixed << std::setprecision(2) << time << "s: "
                          << "Thrust=" << std::setprecision(1) << engine->GetThrust() << "lbs, " 
                          << "Fuel=" << std::setprecision(2) << propellant_remaining << "lbs, "
                          << "Burn=" << std::setprecision(1) << burn_percentage << "%, "
                          << "Impulse=" << std::setprecision(1) << total_impulse << "lbf⋅s" << std::endl;
            }
        }
        
        // Continue impulse integration even after debug output stops
        if (motor_ignited && !engine_shutdown) {
            auto engine = fdmExec->GetPropulsion()->GetEngine(0);
            double dt = time - last_time;
            if (dt > 0) {
                total_impulse += engine->GetThrust() * dt;
            }
        }
        
        last_time = time;

        // Shut down engine when fuel is exhausted or after expected burn time
        if (motor_ignited && !engine_shutdown) {
            double propellant_remaining = fdmExec->GetPropulsion()->GetTank(0)->GetContents();
            double burn_time = time - ignition_time;
            
            if (propellant_remaining < 0.1 || burn_time > 2.2) {  // Shut down when fuel low or after 2.2s (real L1720 burn time)
                auto engine = fdmExec->GetPropulsion()->GetEngine(0);
                engine->SetRunning(false);
                fdmExec->GetFCS()->SetThrottleCmd(0, 0.0);
                
                // Force fuel tank to zero to prevent further table lookups
                fdmExec->GetPropulsion()->GetTank(0)->SetContents(0.0);
                
                engine_shutdown = true;
                std::cout << "Engine shutdown at t=" << time << "s (fuel=" << propellant_remaining 
                          << "lbs, burn_time=" << burn_time << "s)" << std::endl;
                std::cout << "TOTAL IMPULSE DELIVERED: " << total_impulse << " lbf⋅s (expected: 822 lbf⋅s)" << std::endl;
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

        // Deploy drogue chute at apogee
        if (reached_apogee && !drogue_deployed) {
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
        
        // Calculate 3D position relative to launch point
        double current_lat = fdmExec->GetPropagate()->GetLocation().GetLatitudeDeg();
        double current_lon = fdmExec->GetPropagate()->GetLocation().GetLongitudeDeg();
        double current_alt = fdmExec->GetPropagate()->GetAltitudeASL();
        
        // Convert to local coordinates in feet (X=North, Y=East, Z=Up)
        double x_pos = (current_lat - initial_latitude) * 364000.0;  // North-South in feet
        double y_pos = (current_lon - initial_longitude) * 364000.0 * cos(initial_latitude * 3.14159265359 / 180.0);  // East-West in feet
        double z_pos = current_alt - initial_altitude;  // Height above launch point in feet
        
        outputFile << time << "," << x_pos << "," << y_pos << "," << z_pos << "," << altitude << "," << vertical_velocity << "," << drogue_deployed << "," << main_deployed << "\n";

        if (did_liftoff && reached_apogee && altitude < 5.0) {  // Only terminate after apogee and very low altitude
            std::cout << "Rocket has reached the ground after flight." << std::endl;
            break;
        }

        // Add detailed monitoring during descent phase
        if (reached_apogee && altitude < 700.0 && time > 11.0) {
            if (fmod(time, 0.1) < 0.01) {  // Every 0.1 seconds during critical descent
                double alpha = fdmExec->GetAuxiliary()->Getalpha() * 180.0/3.14159; // Convert to degrees
                double beta = fdmExec->GetAuxiliary()->Getbeta() * 180.0/3.14159;
                double mach = fdmExec->GetAuxiliary()->GetMach();
                double qbar = fdmExec->GetAuxiliary()->Getqbar();
                
                std::cout << "DESCENT DEBUG t=" << std::fixed << std::setprecision(1) << time 
                          << "s: Alt=" << altitude << "ft, VMag=" << velocity_magnitude 
                          << "ft/s, VZ=" << vertical_velocity << "ft/s" << std::endl;
                std::cout << "  Alpha=" << alpha << "deg, Beta=" << beta 
                          << "deg, Mach=" << mach << ", Qbar=" << qbar << "psf" << std::endl;
            }
        }
    }

    std::cout << "Simulation complete." << std::endl;
    std::cout << "Maximum altitude reached: " << max_altitude << " ft" << std::endl;
    std::cout << "Time to reach the ground: " << fdmExec->GetSimTime() << " s" << std::endl;

    outputFile.close();
    return 0;
}
