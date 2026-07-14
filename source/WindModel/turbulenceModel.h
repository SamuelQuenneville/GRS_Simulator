/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TURBULENCEMODEL_H
#define GRS_SIMULATOR_TURBULENCEMODEL_H

#include "Mathematics/math.h"

class TurbulenceModel {

public:
    virtual ~TurbulenceModel() = default;
    [[nodiscard]] virtual grs::Vec3d getTurbulence(const grs::Vec3d& position, const grs::Vec3d& advectionVelocity, double time) const = 0;
};


#endif //GRS_SIMULATOR_TURBULENCEMODEL_H
