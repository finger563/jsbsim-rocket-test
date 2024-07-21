#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGInertial.h>

int main() {
    // Create an instance of the JSBSim flight dynamics model executor
    std::unique_ptr<JSBSim::FGFDMExec> fdmExec(new JSBSim::FGFDMExec());

    // Set the simulation to run at 120 Hz
    fdmExec->Setdt(1.0 / 120.0);

    // Load the aircraft configuration file
    if (!fdmExec->LoadModel("Suborbital_Rocket")) {
        std::cerr << "Failed to load the aircraft model" << std::endl;
        return 1;
    }

    // Initialize the JSBSim model
    if (!fdmExec->GetPropagate()->InitModel()) {
        std::cerr << "Failed to initialize the aircraft model" << std::endl;
        return 1;
    }

    // Set initial conditions if needed (example: setting initial altitude)
    fdmExec->GetIC()->SetAltitudeASLFtIC(0.0);
    fdmExec->GetIC()->SetLatitudeDegIC(0.0);
    fdmExec->GetIC()->SetLongitudeDegIC(0.0);
    fdmExec->GetIC()->SetThetaDegIC(90.0); // Launch angle

    // Initialize the model again to apply initial conditions
    fdmExec->RunIC();

    // Open an output file to save the trajectory and velocity data
    std::ofstream outputFile("rocket_trajectory.csv");
    outputFile << "Time,Altitude,Velocity\n";

    std::cout << "Starting simulation..." << std::endl;

    // Run the simulation loop until the rocket reaches the ground or 1000 seconds have passed
    while (fdmExec->GetSimTime() < 10.0 || fdmExec->GetPropagate()->GetAltitudeASL() > 0.0) {
        // Run the JSBSim simulation for one time step
        fdmExec->Run();

        // Get the current time, altitude, and velocity
        double time = fdmExec->GetSimTime();
        double altitude = fdmExec->GetPropagate()->GetAltitudeASL();
        // velocity is a 3 vector (north, east, down, in ft/s)
        double velocity = fdmExec->GetPropagate()->GetVel(3); // get the downwards velocity

        // Print the trajectory and velocity
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Time: " << time << " s, Altitude: " << altitude << " ft, Velocity: " << velocity << " ft/s" << std::endl;

        // Write the data to the output file
        outputFile << time << "," << altitude << "," << velocity << "\n";
    }

    std::cout << "Simulation complete." << std::endl;
    // print number of seconds to reach the ground
    std::cout << "Time to reach the ground: " << fdmExec->GetSimTime() << " s" << std::endl;

    // Close the output file
    outputFile.close();

    return 0;
}
