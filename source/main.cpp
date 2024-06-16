/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "world.h"
#include "configurationParser.h"
#include "RigidBodyAndLink/link.h"

#include "yaml-cpp/yaml.h"
#include "Definitions/configurationDefinitions.h"

int main() {

    std::cout << "It runs!" << "\n";
    World myWorld;
    myWorld.createSystem(World::ONE_AIRCRAFT);

    YAML::Node nodeConfiguration = YAML::LoadFile("C:/Users/ques2902/Desktop/simulationV1.yaml");
    simulationParameters simParams = ConfigurationParser::parseSimulationParameters(nodeConfiguration);

    World::systemType type = myWorld.systemTypeStringToEnum.at(simParams.systemConfiguration);
    int n = World::getNumberOfAircraftInSystem(type);
    std::cout << n << "\n";

    fixedWingProperties aircraftProperties = ConfigurationParser::parseFixedWingProperties(simParams.aircraftPropertiesFileDirectory);
    tetherProperties tetherProperties = ConfigurationParser::parseTetherProperties(simParams.tetherPropertiesFileDirectory);
    std::cout << aircraftProperties.mass << "\n";
    std::cout << tetherProperties.length << "\n";

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
