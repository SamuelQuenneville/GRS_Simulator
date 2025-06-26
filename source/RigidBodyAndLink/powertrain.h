/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef POWERTRAIN_H
#define POWERTRAIN_H

#include "Mathematics/grsMath.h"

#define MAX_ITER_RPM  100

// function for propeller thrust --> 0 = [coeff_thrust * rho*n^2*D^4] - thrustTarget
float f(float n, float x, float y, float z, float phi, float psi, float thrustTarget);

// derivative of propeller thrust
float df(float n, float x, float y, float z, float phi, float psi);

double thrust2rpm(float airspeed, float thrustTarget);
double rpm2thrust(float airspeed, float rpmTarget);
float maxThrust(float airspeed);

#endif //POWERTRAIN_H
