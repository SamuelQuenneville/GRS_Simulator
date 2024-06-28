/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "tether.h"

int Tether::m_idCounter = 0;

Tether::Tether(worldPhysics& physics, Logger& logger, tetherProperties& properties)
    : m_id(++m_idCounter)
    , m_type(Link::RIGID)
    , m_physics(physics)
    , m_logger(logger)
    , m_properties(properties)
{
    std::cout << "Tether created --> ID = " << m_id << "\n";
}

int Tether::getId() const {
    return m_id;
}

Link::linkType Tether::getType() const {
    return m_type;
}

void Tether::update(double dt) {
    // Update node position base on connection if aircraft or payload
    std::vector<Node*> &startPointConnections = connections[&startPointNode()];
    std::vector<Node*> &endPointConnections = connections[&endPointNode()];

    // Should be connect to a payload
    // TODO what happen if Y tether config
    if (startPointConnections.size() == 1) {
        startPoint->setPosition(startPointConnections.at(0)->position());
        startPoint->setVelocity(startPointConnections.at(0)->velocity());
    }

    // Should be an aircraft position or a Y tether node
    // TODO change if Y tether config
    if (endPointConnections.size() == 1) {
        endPoint->setPosition(endPointConnections.at(0)->position());
        endPoint->setVelocity(endPointConnections.at(0)->velocity());
    }

    // Calculate the tension force and distribute to the nodes
    Vector3<double> tensionForce = m_calcTensionForce();
    startPoint->setForce(-tensionForce);    // TODO confirm negative sign here
    endPoint->setForce(tensionForce);

    // Log
    m_writeLog();
}

void Tether::m_updateRelativePosition() {
    m_relativePosition = endPoint->position() - startPoint->position();
}

void Tether::m_updateRelativeSpeed() {
    m_relativeSpeed = endPoint->velocity() - startPoint->velocity();
}

Vector3<double> Tether::m_calcTensionForce() {
    // Update
    m_updateRelativePosition();
    m_updateRelativeSpeed();

    // Find tether position relative to startPoint
    double tetherMagnitude = m_relativePosition.magnitude();
    Vector3<double> tetherNormalize = m_relativePosition.normalize();

    // Calculate tether elongation
    double tetherDeformation = tetherMagnitude - m_properties.length;

    Vector3<double> forceSpring = (-m_properties.rigidity * tetherDeformation * tetherNormalize) - (m_properties.damping * m_relativeSpeed.dot(tetherNormalize) * tetherNormalize);

    double tension = -tetherNormalize.dot(forceSpring);
    if(tension<0) {
        forceSpring = {0,0,0};
    }

    m_tetherLogInfos.deformation = tetherDeformation;
    m_tetherLogInfos.forceSpring = forceSpring;

    return forceSpring;
}

double Tether::m_calcSkinFrictionCoeff(double parallelVelocity) const {

    // Calculate Mach number
    double parallelMach = parallelVelocity / m_physics.soundSpeed;

    // Calculate Cf
    double Cf;
    if (parallelMach < 0.4) {
        Cf = 0.038 - 0.0425 * parallelMach;
    } else {
        Cf = 0.013 + 0.0395 * (parallelMach - 0.85) * (parallelMach - 0.85);
    }

    return Cf;
}

double Tether::m_calcCrossflowDrag(double normalVelocity) const {

    // Calculate Mach number
    double normalMach = normalVelocity / m_physics.soundSpeed;
    double normalMachSquare = normalMach * normalMach;

    // Calculate Cn
    double Cn = 1.17 + normalMach / 40.0 + normalMachSquare / 4.0 + normalMachSquare*normalMach / 1.6;

    return Cn;
}

std::tuple<double, double> Tether::m_calcLiftDragCoeff(double Cf, double Cn, double theta) {

    double sinTheta = std::sin(theta);
    double cosTheta = std::cos(theta);

    double Cd = Cf + Cn * sinTheta * sinTheta * sinTheta;
    double Cl = Cn * sinTheta * sinTheta * cosTheta;

    return std::make_tuple(Cl, Cd);
}

// TODO implement tether aerodynamic and discretization
Vector3<double> Tether::m_calcTetherAeroForces(Vector3<double> windSpeed) {
    Vector3<double> forceAero = {0,0,0};
    m_tetherLogInfos.forceAero = forceAero;

    return forceAero;
}

void Tether::grounded() {
    startPoint->setPosition({0,0,0});
    startPoint->setVelocity({0,0,0});
}

void Tether::m_writeLog() {
    tetherLogMessage logMessage{};

    logMessage = {
            m_id,
            m_tetherLogInfos.deformation,
            startPoint->position(),
            startPoint->velocity(),
            endPoint->position(),
            endPoint->velocity(),
            m_tetherLogInfos.forceSpring,
            m_tetherLogInfos.forceAero
    };

    m_logger.writeTetherMessage(logMessage);
}
