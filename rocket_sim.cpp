#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
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

    // Set initial conditions for vertical launch
    fdmExec->GetIC()->SetAltitudeASLFtIC(10.5);    // Start slightly higher to avoid ground contact
    fdmExec->GetIC()->SetLatitudeDegIC(37.0);
    fdmExec->GetIC()->SetLongitudeDegIC(-122.0);
    fdmExec->GetIC()->SetThetaDegIC(90.0);         // Vertical orientation (90 deg pitch)
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
    double ignition_time = 0.05; // Ignite 0.05 seconds after simulation start
    
    std::cout << "Motor ignition scheduled for t=" << ignition_time << " seconds" << std::endl;

    float liftoff_threshold_agl = 10.0f;
    bool did_liftoff = false;

    // Run the simulation loop
    while (fdmExec->GetSimTime() < 1000.0 && fdmExec->GetPropagate()->GetAltitudeASL() >= -10.0) {
        // Run the JSBSim simulation for one time step
        fdmExec->Run();

        // Get current state
        double time = fdmExec->GetSimTime();
        double altitude = fdmExec->GetPropagate()->GetAltitudeASL();
        double velocity = fdmExec->GetPropagate()->GetVel(3); // Vertical velocity

        // Ignite motor at scheduled time using throttle setting for solid rockets
        if (!motor_ignited && time >= ignition_time) {
            std::cout << "Igniting engine at t=" << time << "s" << std::endl;
            fdmExec->GetFCS()->SetThrottleCmd(0, 1.0);  // Set throttle to 100% for solid rocket ignition
            auto engine = fdmExec->GetPropulsion()->GetEngine(0);
            engine->SetRunning(true);  // Also set engine to running state
            motor_ignited = true;
        }

        // Debug output for engine state immediately after ignition and periodically
        if (motor_ignited && time < 10.0) {
            if (time - ignition_time < 0.1 || fmod(time, 0.2) < 0.01) {  // Show for first 0.1s after ignition, then every 0.2s
                auto engine = fdmExec->GetPropulsion()->GetEngine(0);
                double throttle = fdmExec->GetFCS()->GetThrottlePos(0);
                double propellant_consumed = 3.9 - fdmExec->GetPropulsion()->GetTank(0)->GetContents();
                std::cout << "t=" << std::fixed << std::setprecision(3) << time << "s: Throttle=" << throttle 
                          << ", Engine running=" << engine->GetRunning() 
                          << ", Thrust=" << engine->GetThrust() << "lbs" 
                          << ", Fuel remaining=" << fdmExec->GetPropulsion()->GetTank(0)->GetContents() << "lbs"
                          << ", Propellant consumed=" << propellant_consumed << "lbs" << std::endl;
            }
        }

        if (velocity > 0.0f && altitude > liftoff_threshold_agl && !did_liftoff) {
            std::cout << "Liftoff detected at " << altitude << " ft" << std::endl;
            did_liftoff = true;
        }

        // Track maximum altitude and detect apogee
        if (did_liftoff && altitude > max_altitude) {
            max_altitude = altitude;
        } else if (did_liftoff && !reached_apogee && velocity < 0.0 && altitude < max_altitude) {
            // Apogee reached when velocity becomes negative and altitude starts decreasing
            reached_apogee = true;
            std::cout << "Apogee reached at " << max_altitude << " ft" << std::endl;
        }

        // Deploy drogue chute at apogee
        if (reached_apogee && !drogue_deployed) {
            fdmExec->SetPropertyValue("external_reactions/drogue_chute/drag_area", 8.0); // Drogue chute drag area
            drogue_deployed = true;
            std::cout << "Drogue chute deployed at " << altitude << " ft" << std::endl;
        }

        // Deploy main chute below 500 feet (per requirements)
        if (drogue_deployed && !main_deployed && altitude < 500.0) {
            fdmExec->SetPropertyValue("external_reactions/main_chute/drag_area", 50.0); // Larger main chute
            main_deployed = true;
            std::cout << "Main chute deployed at " << altitude << " ft" << std::endl;
        }

        // Print and save the trajectory data (reduced output frequency)
        if (fmod(time, 0.1) < 0.01) {  // Print every 0.1 seconds
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Time: " << time << " s, Altitude: " << altitude << " ft, Velocity: " << velocity << " ft/s" << std::endl;
        }
        outputFile << time << "," << altitude << "," << velocity << "," << drogue_deployed << "," << main_deployed << "\n";

        if (did_liftoff && altitude < liftoff_threshold_agl) {
            std::cout << "Rocket has reached the ground." << std::endl;
            break;
        }
    }

    std::cout << "Simulation complete." << std::endl;
    std::cout << "Maximum altitude reached: " << max_altitude << " ft" << std::endl;
    std::cout << "Time to reach the ground: " << fdmExec->GetSimTime() << " s" << std::endl;

    outputFile.close();
    return 0;
}
