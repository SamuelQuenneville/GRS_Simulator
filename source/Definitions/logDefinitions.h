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

#include "Mathematics/vector3.h"

struct fixedWingLogMessage {
    int uavId;
    double aileronCommand;
    double elevatorCommand;
    double rudderCommand;
    double thrustCommand;
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
    Vector3<double> earthForcesTether;
};

struct tetherLogMessage {
    int tetherId;
    double elongation;
    Vector3<double> startPointPosition;
    Vector3<double> startPointSpeed;
    Vector3<double> endPointPosition;
    Vector3<double> endPointSpeed;
    Vector3<double> earthForcesSpring;
    Vector3<double> earthForcesAero;
};

struct hlcLogMessage {

};

struct llcLogMessage {

};

#endif //GRS_SIMULATOR_LOGDEFINITIONS_H
