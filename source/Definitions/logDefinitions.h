/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_LOGDEFINITIONS_H
#define GRS_SIMULATOR_LOGDEFINITIONS_H

#pragma once

#include <map>

#include "Mathematics/vector3.h"
#include "Definitions/fixedWingDefinitions.h"
#include "Control/pid.h"
#include "Control/lateralController.h"

struct fixedWingLogMessage {
    int uavId{};
    double aileronCommand{};
    double elevatorCommand{};
    double rudderCommand{};
    double thrustCommand{};
    Vector3<double> nedPosition;
    Vector3<double> earthSpeed;
    Vector3<double> bodySpeed;
    Vector3<double> attitude;
    Vector3<double> angularSpeed;
    Vector3<double> bodyForcesAero;
    Vector3<double> bodyMomentsAero;
    Vector3<double> bodyForcesTether;
};

struct payloadLogMessage {
    Vector3<double> nedPosition;
    Vector3<double> earthSpeed;
    std::map<int, Vector3<double>> earthForcesTethers;
};

struct tetherLogMessage {
    int tetherId{};
    double elongation{};
    Vector3<double> startPointPosition;
    Vector3<double> startPointSpeed;
    Vector3<double> endPointPosition;
    Vector3<double> endPointSpeed;
    Vector3<double> earthForcesSpring;
    Vector3<double> earthForcesAero;
};

struct hlcLogMessage {
    fixedWingCommands commands1;
    PID::pidInfo groundspeedPid1;
    LateralController::ctrlInfo lateralController1;
    fixedWingCommands commands2;
    PID::pidInfo groundspeedPid2;
    LateralController::ctrlInfo lateralController2;
};

struct llcLogMessage {
    int aircraftId;
    fixedWingControlInputs inputs;
    PID::pidInfo pitchPid;
    PID::pidInfo rollPid;
    PID::pidInfo altitudePid;
};

#endif //GRS_SIMULATOR_LOGDEFINITIONS_H
