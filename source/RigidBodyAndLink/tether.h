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

class Tether final : public Link {

public:
    Tether(const worldPhysics& physics, Logger& logger, const tetherProperties& properties);

    [[nodiscard]] int getId() const;
    [[nodiscard]] linkType getType() const;

    void update(double dt) override;
    void grounded() const;

private:
    int m_id;
    static int m_idCounter;
    linkType m_type;

    worldPhysics m_physics;
    Logger& m_logger;
    tetherProperties m_properties;

    tetherLogInfos m_tetherLogInfos{};

    // endPoint - startPoint
    Vector3<double> m_relativePosition{};
    Vector3<double> m_relativeSpeed{};

    void m_updateRelativePosition();
    void m_updateRelativeSpeed();

    // Spring model
    Vector3<double> m_calcTensionForce();

    // Tether aerodynamic
    [[nodiscard]] double m_calcSkinFrictionCoeff(double parallelVelocity) const;
    [[nodiscard]] double m_calcCrossflowDrag(double normalVelocity) const;
    static std::tuple<double, double> m_calcLiftDragCoeff(double Cf, double Cn, double theta);
    Vector3<double> m_calcTetherAeroForces(Vector3<double> windSpeed);

    void m_writeLog() const;
};


#endif //GRS_SIMULATOR_TETHER_H
