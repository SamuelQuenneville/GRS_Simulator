/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_CUBE_H
#define GRS_SIMULATOR_CUBE_H

#include "physicsObject.h"

class Cube final : public PhysicsObject {

public:
    Cube(const grs::Vec3f& size, const std::string& executablePath);
    ~Cube() override = default;

    void updateTransform(double interpolationFactor) override;

private:
    float m_size[3] = {1,1,1};

    grs::Matrix4d m_scalingMatrix{};
};

#endif //GRS_SIMULATOR_CUBE_H
