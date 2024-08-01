/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_CONFIGURATIONDEFINITIONS_H
#define GRS_SIMULATOR_CONFIGURATIONDEFINITIONS_H

#pragma once

#include <string>
#include <vector>

/*
 *   VARIOUS SIMULATION STRUCTURES
 */

struct simulationParameters {
    std::string logFilesDirectory;
    std::string aircraftPropertiesFileDirectory;
    std::string tetherPropertiesFileDirectory;
    std::string payloadPropertiesFileDirectory;
    std::string systemConfiguration;
    bool fixedDuration;
    float duration;
    int hlcRate;
    int llcRate;
    bool windOn;
    bool tetherAeroOn;
};

struct initialStates {
    std::vector<double> position;      // [x y z]
    std::vector<double> speed;         // [vx vy vz]
    std::vector<double> orientation;   // [roll pitch yaw]
};

struct target {
    std::vector<double> positionPayload;    // [x y z]
    float groundspeed;                      // m/s
    float radius;                           // m
    float aircraftPitch;                    // degrees
    float aircraftRoll;                     // degrees
};


#endif //GRS_SIMULATOR_CONFIGURATIONDEFINITIONS_H
