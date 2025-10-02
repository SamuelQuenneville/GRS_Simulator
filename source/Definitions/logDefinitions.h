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

#include "Definitions/fixedWingDefinitions.h"
#include "Control/pid.h"
#include "Control/lateralController.h"
#include "Control/phaseController.h"

struct fixedWingLogMessage {
    int uavId{};
    double aileronCommand{};
    double elevatorCommand{};
    double rudderCommand{};
    double thrustCommand{};
    grs::Vec3d nedPosition;
    grs::Vec3d earthSpeed;
    grs::Vec3d bodySpeed;
    grs::Vec3d attitude;
    grs::Vec3d angularSpeed;
    grs::Vec3d bodyForcesAero;
    grs::Vec3d bodyMomentsAero;
    grs::Vec3d bodyForcesTether;
    grs::Vec3d bodyMomentsTether;
};

struct payloadLogMessage {
    grs::Vec3d nedPosition;
    grs::Vec3d earthSpeed;
    std::map<int, grs::Vec3d> earthForcesTethers;
};

struct tetherLogMessage {
    int tetherId{};
    double elongation{};
    grs::Vec3d startPointPosition;
    grs::Vec3d startPointSpeed;
    grs::Vec3d endPointPosition;
    grs::Vec3d endPointSpeed;
    grs::Vec3d earthForcesSpring;
    grs::Vec3d earthForcesAero;
};

struct hlcLogMessage {
    fixedWingCommands commands1;
    PID::pidInfo groundspeedPid1;
    LateralController::ctrlInfo lateralController1;
    PhaseController::ctrlInfo phaseController1;
    fixedWingCommands commands2;
    PID::pidInfo groundspeedPid2;
    LateralController::ctrlInfo lateralController2;
    PhaseController::ctrlInfo phaseController2;
};

struct llcLogMessage {
    int aircraftId;
    fixedWingControlInputs inputs;
    PID::pidInfo pitchPid;
    PID::pidInfo rollPid;
    PID::pidInfo altitudePid;
};

#endif //GRS_SIMULATOR_LOGDEFINITIONS_H
