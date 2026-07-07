/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_LAUNCHER_H
#define GRS_SIMULATOR_LAUNCHER_H

#include "physicsObject.h"

class Launcher final : public PhysicsObject {

public :
    Launcher(const grs::Vec3d& scaling, const std::string& executablePath);
    ~Launcher() override = default;

    void updateTransform(double interpolationFactor) override;

private :
    grs::Matrix4d m_scalingMatrix{};
};


#endif //GRS_SIMULATOR_LAUNCHER_H
