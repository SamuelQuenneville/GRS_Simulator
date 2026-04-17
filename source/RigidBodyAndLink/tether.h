/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TETHER_H
#define GRS_SIMULATOR_TETHER_H

#include "link.h"
#include "Definitions/renderObjectDefinitions.h"
#include "Definitions/worldPhysics.h"
#include "Definitions/tetherDefinitions.h"
#include "Log/logger.h"

class Tether final : public Link {

public:
    Tether(const worldPhysics& physics, const segmentProperties& properties);

    [[nodiscard]] linkType getType() const;

    void update(double dt) override;
    void grounded() const;

    [[nodiscard]] tetherRenderData packRenderData() const;

private:
    double m_simTime = 0.0;

    linkType m_type;

    worldPhysics m_physics;
    segmentProperties m_properties;

    tetherLogInfos m_tetherLogInfos{};

    // endPoint - startPoint
    grs::Vec3d m_relativePosition{};
    grs::Vec3d m_relativeSpeed{};

    void m_updateRelativePosition();
    void m_updateRelativeSpeed();

    // Spring model
    grs::Vec3d m_calcTensionForce();

    // Tether aerodynamic
    [[nodiscard]] double m_calcSkinFrictionCoeff(double parallelVelocity) const;
    [[nodiscard]] double m_calcCrossflowDrag(double normalVelocity) const;
    static std::tuple<double, double> m_calcLiftDragCoeff(double Cf, double Cn, double theta);
    grs::Vec3d m_calcTetherAeroForces(grs::Vec3d windSpeed);

    void m_writeLog() const;
};


#endif //GRS_SIMULATOR_TETHER_H
