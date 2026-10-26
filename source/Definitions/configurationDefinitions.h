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
    double renderFps = 60.0; // Viewer frame cap and snapshot publication rate (Hz)
    int msaaSamples = 4;     // Viewer anti-aliasing samples (0 disables)
    bool headless = false;   // Run without window or render snapshots
    std::string configFile;  // Loaded at startup (--config) or run headless (--headless)
};

struct simulationParameters {
    std::string logFilesDirectory;
    std::string aircraftPropertiesFileDirectory;
    std::string tetherPropertiesFileDirectory;
    std::string payloadPropertiesFileDirectory;
    std::string launcherPropertiesFileDirectory;
    std::string controllerGainsFileDirectory;
};

struct systemConfiguration {
    int numberOfUav;
    std::string configuration;
    bool grounded;
    bool haveLauncher;
    bool haveTarget;
    bool tetherAeroOn;
    int hlcRate;
    int llcRate;
};

struct sceneConfiguration {
    std::string configuration;
    bool haveLauncher;
    bool haveTarget;
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
    float altOffsetY;                       // altitude target offset for Y tether config
};


#endif //GRS_SIMULATOR_CONFIGURATIONDEFINITIONS_H
