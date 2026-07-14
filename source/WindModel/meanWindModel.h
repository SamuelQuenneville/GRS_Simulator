/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_MEANWINDMODEL_H
#define GRS_SIMULATOR_MEANWINDMODEL_H

#include "Mathematics/math.h"

class MeanWindModel {

public:
    virtual ~MeanWindModel() = default;
    [[nodiscard]] virtual grs::Vec3d getMeanWind(const grs::Vec3d& position) const = 0;
};



#endif //GRS_SIMULATOR_MEANWINDMODEL_H
