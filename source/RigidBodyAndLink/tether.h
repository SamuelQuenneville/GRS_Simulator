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
#include "Definitions/worldPhysics.h"
#include "Definitions/tetherDefinitions.h"

class Tether : public Link {

public:
    Tether(worldPhysics& physics, tetherProperties& properties);

    [[nodiscard]] int getId() const;
    [[nodiscard]] linkType getType() const;

    void update(double dt) override;
    void grounded();

private:
    int m_id;
    static int m_idCounter;
    linkType m_type;

    worldPhysics m_physics;
    tetherProperties m_properties;

    tetherLogInfos m_tetherLogInfos{};

    // endPoint - startPoint
    Vector3<double> m_relativePosition{};
    Vector3<double> m_relativeSpeed;

    void m_updateRelativePosition();
    void m_updateRelativeSpeed();

    // Spring model
    Vector3<double> m_calcTensionForce();

    // Tether aerodynamic
    [[nodiscard]] double m_calcSkinFrictionCoeff(double parallelVelocity) const;
    [[nodiscard]] double m_calcCrossflowDrag(double normalVelocity) const;
    static std::tuple<double, double> m_calcLiftDragCoeff(double Cf, double Cn, double theta);
    Vector3<double> m_calcTetherAeroForces(Vector3<double> windSpeed);
};


#endif //GRS_SIMULATOR_TETHER_H
