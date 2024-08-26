/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_CONTROLLERGAINS_H
#define GRS_SIMULATOR_CONTROLLERGAINS_H

#pragma once

struct lateralControllerGains {
    float P;
    float I;
    float D;
    float iLimit;
    float rollSaturation;
};
struct phaseControllerGains {
    float P;
    float I;
    float D;
    float iLimit;
    float thrustSaturation;
};

struct groundspeedControllerGains {
    float P;
    float I;
    float D;
    float FF;
    float iLimit;
    float targetFilter;
    float derivativeFilter;
    float errorFilter;
    float thrustMin;
    float thrustMax;
};

struct altitudeControllerGains {
    float P;
    float I;
    float D;
    float FF;
    float iLimit;
    float targetFilter;
    float derivativeFilter;
    float errorFilter;
    float pitchMin;
    float pitchMax;
};

struct pitchControllerGains {
    float P;
    float I;
    float D;
    float FF;
    float iLimit;
    float targetFilter;
    float derivativeFilter;
    float errorFilter;
    float elevatorMin;
    float elevatorMax;
};

struct rollControllerGains {
    float P;
    float I;
    float D;
    float FF;
    float iLimit;
    float targetFilter;
    float derivativeFilter;
    float errorFilter;
    float aileronMin;
    float aileronMax;
};

struct llcGains {
    altitudeControllerGains altitudeCtrlGains;
    pitchControllerGains pitchCtrlGains;
    rollControllerGains rollCtrlGains;
};

struct hlcGains {
    phaseControllerGains phaseCtrlGains;
    lateralControllerGains lateralCtrlGains;
    groundspeedControllerGains groundspeedCtrlGains;
};

struct ctrlGains {
    llcGains llc;
    hlcGains hlc;
};

#endif //GRS_SIMULATOR_CONTROLLERGAINS_H
