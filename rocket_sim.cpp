#include <iostream>
#include <FGFDMExec.h>
#include <models/FGPropagate.h>

using namespace JSBSim;

int main() {
    FGFDMExec FDMExec;

    // Load your rocket configuration file
    if (!FDMExec.LoadScript(SGPath("rocket.xml"))) {
        std::cerr << "Failed to load rocket configuration file." << std::endl;
        return 1;
    }

    FDMExec.RunIC();

    double simulation_time = 0.0;
    double time_step = 0.01; // Time step in seconds
    double simulation_duration = 10.0; // Duration of the simulation in seconds

    while (simulation_time < simulation_duration) {
        FDMExec.Run();
        simulation_time += time_step;

        // Access and print simulation data (e.g., altitude)
        // double altitude = FDMExec.GetPropagate()->GetLocation().GetAltitudeASL();
        // std::cout << "Time: " << simulation_time << " seconds, Altitude: " << altitude << " meters" << std::endl;
    }

    return 0;
}
