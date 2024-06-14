/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "world.h"
#include "RigidBodyAndLink/link.h"

int main() {

    std::cout << "It runs!";
    World myWorld;
    myWorld.createSystem(World::ONE_AIRCRAFT);

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
