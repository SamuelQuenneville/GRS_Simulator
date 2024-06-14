/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_CONFIGURATIONDEFINITIONS_H
#define GRS_SIMULATOR_CONFIGURATIONDEFINITIONS_H

#include <string>
#include <vector>

/*
 *   VARIOUS SIMULATION STRUCTURES
 */

struct SimulationParameters {
    std::string logFileDirectory;
    std::string uavPropertiesFileDirectory;
    bool nullRoll;
    bool tetherAero;
    float time;
    int hlcRate;
    int llcRate;
    std::vector<double> windEarth;
};

struct States {
    std::vector<double> position;      // [x y z]
    std::vector<double> speed;         // [vx vy vz]
    std::vector<double> orientation;   // [roll pitch yaw]
};

struct InitialStates {
    States uav1States;
    States uav2States;
    States payloadStates;
};

struct Target {
    std::vector<double> positionPayload;    // [x y z]
    float groundspeed;                      // m/s
};


#endif //GRS_SIMULATOR_CONFIGURATIONDEFINITIONS_H
