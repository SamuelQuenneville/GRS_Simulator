/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "world.h"


int main() {

    World myWorld;

    // TODO get from user input maybe opengl window in future
    std::string fileName = "C:/Users/ques2902/Desktop/simulationV1.yaml";

    myWorld.readConfigurationFile(fileName);
    myWorld.createSystem();

    myWorld.start();
    while (myWorld.isRunning())
    {
        // ------------------------------- HLC Call -------------------------------
        // Call at HLC rate < LLC rate (simulation rate)

        // ------------------------------------------------------------------------


        // ----------------------------- Tether Update ----------------------------
        // Calculate forces applied by tethers on UAVs and payload

        // ------------------------------------------------------------------------


        // ---------------------------- Dynamic Update ----------------------------
        // Update payload and UAVs states

        // ------------------------------------------------------------------------


        // --------------------------------- Log ----------------------------------

        // ------------------------------------------------------------------------
    }

    return 0;
}
