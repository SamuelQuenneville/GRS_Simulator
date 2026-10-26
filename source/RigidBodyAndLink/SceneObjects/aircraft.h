/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_AIRCRAFT_H
#define GRS_SIMULATOR_AIRCRAFT_H

#include "physicsObject.h"
#include "RigidBodyAndLink/fixedWing.h"

class Aircraft final : public PhysicsObject {

public :
    Aircraft(const bodyRenderData &aircraftRenderData, const grs::Vec3d& scaling, const std::string& executablePath);
    ~Aircraft() override = default;

    void updateTransform(double interpolationFactor) override;

    // Scale used in the main view only; the mesh itself is at true size
    void setDisplayScale(double scale);

private :
    grs::Matrix4d m_scalingMatrix{};
};


#endif //GRS_SIMULATOR_AIRCRAFT_H
