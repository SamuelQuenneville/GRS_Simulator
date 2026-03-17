/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "tether.h"

Tether::Tether(const worldPhysics& physics, const segmentProperties& properties)
    : m_type(Link::RIGID)
    , m_physics(physics)
    , m_properties(properties)
{

}

Link::linkType Tether::getType() const {
    return m_type;
}

void Tether::update(const double dt) {
    m_simTime += dt;

    // Update node position base on connection if aircraft or payload
    const std::vector<Node*> &startPointConnections = connections[&startPointNode()];
    const std::vector<Node*> &endPointConnections = connections[&endPointNode()];

    // Should be connected to a payload
    if (startPointConnections.size() == 1) {
        startPoint->setPosition(startPointConnections.at(0)->position());
        startPoint->setVelocity(startPointConnections.at(0)->velocity());
    }

    // Should be an aircraft position or a Y tether node
    if (endPointConnections.size() == 1) {
        endPoint->setPosition(endPointConnections.at(0)->position());
        endPoint->setVelocity(endPointConnections.at(0)->velocity());
    }

    // Update
    m_updateRelativePosition();
    m_updateRelativeSpeed();

    // Calculate the tension force and distribute to the nodes
    const grs::Vec3d tensionForce = m_calcTensionForce();
    startPoint->setForce(tensionForce);
    endPoint->setForce(-tensionForce);

    // Log
    m_writeLog();
}

void Tether::m_updateRelativePosition() {
    m_relativePosition = endPoint->position() - startPoint->position();
}

void Tether::m_updateRelativeSpeed() {
    m_relativeSpeed = endPoint->velocity() - startPoint->velocity();
}

grs::Vec3d Tether::m_calcTensionForce() {
    // Find tether position relative to startPoint
    const double tetherMagnitude = m_relativePosition.norm();
    const grs::Vec3d tetherNormalize = grs::normalize(m_relativePosition);

    // Calculate tether elongation
    const double tetherDeformation = tetherMagnitude - m_properties.length;

    grs::Vec3d forceSpring = (m_properties.rigidity * tetherDeformation * tetherNormalize) + (m_properties.damping * grs::dot(m_relativeSpeed, tetherNormalize) * tetherNormalize);

    const double tension = grs::dot(tetherNormalize, forceSpring);
    if (tension < 0.0) {
        forceSpring = grs::Vec3d::zeros();
    }

    m_tetherLogInfos.deformation = tetherDeformation;
    m_tetherLogInfos.forceSpring = forceSpring;

    return forceSpring;
}

double Tether::m_calcSkinFrictionCoeff(const double parallelVelocity) const {

    // Calculate Mach number
    const double parallelMach = parallelVelocity / m_physics.soundSpeed;

    // Calculate Cf
    double Cf;
    if (parallelMach < 0.4) {
        Cf = 0.038 - 0.0425 * parallelMach;
    } else {
        Cf = 0.013 + 0.0395 * (parallelMach - 0.85) * (parallelMach - 0.85);
    }

    return Cf;
}

double Tether::m_calcCrossflowDrag(const double normalVelocity) const {

    // Calculate Mach number
    const double normalMach = normalVelocity / m_physics.soundSpeed;
    const double normalMachSquare = normalMach * normalMach;

    // Calculate Cn
    const double Cn = 1.17 + normalMach / 40.0 + normalMachSquare / 4.0 + normalMachSquare*normalMach / 1.6;

    return Cn;
}

std::tuple<double, double> Tether::m_calcLiftDragCoeff(const double Cf, const double Cn, const double theta) {

    const double sinTheta = std::sin(theta);
    const double cosTheta = std::cos(theta);

    double Cd = Cf + Cn * sinTheta * sinTheta * sinTheta;
    double Cl = Cn * sinTheta * sinTheta * cosTheta;

    return std::make_tuple(Cl, Cd);
}

// TODO implement tether aerodynamic and discretization
grs::Vec3d Tether::m_calcTetherAeroForces(grs::Vec3d windSpeed) {
    const grs::Vec3d forceAero = grs::Vec3d::zeros();
    m_tetherLogInfos.forceAero = forceAero;

    return forceAero;
}

void Tether::grounded() const {
    startPoint->setPosition(grs::Vec3d::zeros());
    startPoint->setVelocity(grs::Vec3d::zeros());
}

void Tether::m_writeLog() const {

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(4)
        << m_simTime << "," << Logger::nowWallTimeMs() << "," << m_properties.name << "," << m_tetherLogInfos.deformation << ","
        << startPoint->position()[0] << "," << startPoint->position()[1] << "," << startPoint->position()[2] << ","
        << startPoint->velocity()[0] << "," << startPoint->velocity()[1] << "," << startPoint->velocity()[2] << ","
        << endPoint->position()[0]   << "," << endPoint->position()[1]   << "," << endPoint->position()[2]   << ","
        << endPoint->velocity()[0]   << "," << endPoint->velocity()[1]   << "," << endPoint->velocity()[2]   << ","
        << m_tetherLogInfos.forceSpring[0] << "," << m_tetherLogInfos.forceSpring[1] << "," << m_tetherLogInfos.forceSpring[2] << ","
        << m_tetherLogInfos.forceAero[0]   << "," << m_tetherLogInfos.forceAero[1]   << "," << m_tetherLogInfos.forceAero[2];

    Logger::instance().log(LogType::Tether, msg.str());
}
