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

struct programArguments {
    bool sitlMode;
    bool inFlightLaunch;
    bool loggingEnabled;
};

struct simulationParameters {
    std::string logFilesDirectory;
    std::string aircraftPropertiesFileDirectory;
    std::string tetherPropertiesFileDirectory;
    std::string payloadPropertiesFileDirectory;
    std::string launcherPropertiesFileDirectory;
    std::string controllerGainsFileDirectory;
    bool fixedDuration;
    float duration;
    bool windOn;
};

struct systemConfiguration {
    int numberOfUav;
    std::string configuration;
    bool grounded;
    bool haveLauncher;
    bool tetherAeroOn;
    int hlcRate;
    int llcRate;
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
