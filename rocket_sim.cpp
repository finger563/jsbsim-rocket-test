#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGInertial.h>
#include <models/FGPropulsion.h>
#include <models/FGMassBalance.h>

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

    // Set initial conditions
    fdmExec->GetIC()->SetAltitudeASLFtIC(0.0);
    fdmExec->GetIC()->SetLatitudeDegIC(37.0);
    fdmExec->GetIC()->SetLongitudeDegIC(-122.0);
    fdmExec->GetIC()->SetThetaDegIC(85.0); // Slightly off vertical for stability
    fdmExec->GetIC()->SetVNorthFpsIC(0.0);
    fdmExec->GetIC()->SetVEastFpsIC(0.0);
    fdmExec->GetIC()->SetVDownFpsIC(0.0);

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

    // Start the rocket engine
    for (int i = 0; i < fdmExec->GetPropulsion()->GetNumEngines(); i++) {
        std::cout << "Starting engine " << i << std::endl;
        auto engine = fdmExec->GetPropulsion()->GetEngine(i);
        engine->SetRunning(true);
        engine->SetStarter(true);
        fdmExec->GetPropulsion()->SetActiveEngine(i);
    }

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

        if (velocity > 0.0f && altitude > liftoff_threshold_agl && !did_liftoff) {
            std::cout << "Liftoff detected at " << altitude << " ft" << std::endl;
            did_liftoff = true;
        }

        // Track maximum altitude
        if (did_liftoff && altitude > max_altitude) {
            max_altitude = altitude;
        } else if (did_liftoff && !reached_apogee && altitude < max_altitude) {
            reached_apogee = true;
        }

        // Deploy drogue chute at apogee
        if (reached_apogee && !drogue_deployed) {
            fdmExec->SetPropertyValue("external_reactions/drogue_chute/drag_area", 5.0); // Reduced drag area
            drogue_deployed = true;
            std::cout << "Drogue chute deployed at " << altitude << " ft" << std::endl;
        }

        // Deploy main chute below 1000 feet
        if (drogue_deployed && !main_deployed && altitude < 1000.0) {
            fdmExec->SetPropertyValue("external_reactions/main_chute/drag_area", 30.0); // Adjusted drag area
            main_deployed = true;
            std::cout << "Main chute deployed at " << altitude << " ft" << std::endl;
        }

        // Print and save the trajectory data
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Time: " << time << " s, Altitude: " << altitude << " ft, Velocity: " << velocity << " ft/s" << std::endl;
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
