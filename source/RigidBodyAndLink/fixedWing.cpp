/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "fixedWing.h"

int FixedWing::m_idCounter = 0;

FixedWing::FixedWing(worldPhysics& physics, Logger& logger, fixedWingProperties& properties, std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>>&& controller)
        : m_id(++m_idCounter)
        , m_type(RigidBody::AIRCRAFT)
        , m_physics(physics)
        , m_logger(logger)
        , m_properties(properties)
        , m_controller(std::move(controller))
{

}

int FixedWing::getId() const {
    return m_id;
}

RigidBody::rigidBodyType FixedWing::getType() const {
    return m_type;
}

void FixedWing::update(double dt) {

    fixedWingControlInputs inputs(m_controller->calculateAircraftControlInputs(getStates(), getCommands()));
    setInputs(inputs);

    Vector3<double> externalForcesEarth = calcExternalForce();
    Vector3<double> forcesAeroBody   = m_calcForceAeroBodyFrame(inputs);
    Vector3<double> momentsAeroBody  = m_calcMomentAeroBodyFrame(inputs);

    fixedWingStates derivatives   = m_calcStatesDerivatives(forcesAeroBody, momentsAeroBody, externalForcesEarth);
    fixedWingStates updatedStates = m_eulerIntegration(derivatives, dt);

    // Set new states to this body
    setStates(updatedStates);

    // Set new states to the node of this body
    centerOfMass->update({m_states.x, m_states.y, m_states.z}, {m_states.vx, m_states.vy, m_states.vz}, m_bodyToEarth(forcesAeroBody) + externalForcesEarth,m_bodyToEarth(momentsAeroBody));

    // Log
    m_writeLog(forcesAeroBody, momentsAeroBody, m_earthToBody(externalForcesEarth));
}

Vector3<double> FixedWing::m_calcForceAeroBodyFrame(const fixedWingControlInputs& inputs) const {

    // airspeed (Va)
    double Va = airspeed();

    // alpha and beta
    double alpha = 0.0;  // (rad)
    double beta  = 0.0;  // (rad)

    if(Va >= 0.1){
        alpha = std::atan2(m_states.w, m_states.u);  // (rad)
        beta  = std::asin(m_states.v / Va);             // (rad)
    }

    double delAlpha = alpha - radians(m_properties.trimAlpha);
    // CL and CD
    double CL = m_properties.CL_0 + m_properties.CL_ALPHA * delAlpha;
    double CD = m_properties.CD_0 + (CL*CL) / (M_PI * m_properties.oswaldEfficiency * m_properties.aspectRatio);

    // stability vs body frame
    double CX = -CD * std::cos(alpha) + CL * std::sin(alpha);
    double CZ = -CD * std::sin(alpha) - CL * std::cos(alpha);

    // q_bar
    double qBarSurface = 0.5 * m_physics.airDensity * Va * Va * m_properties.wingSurface;

    // del_commands
    double delElevator = inputs.deltaElevator - m_properties.trimElevator;

    // Forces in body frame
    double Fx = inputs.deltaThrust + qBarSurface * (CX + m_properties.Cx_q * m_properties.wingChord/(2.0*Va) * m_states.q + m_properties.Cx_DELTA_ELE * delElevator);
    double Fy = qBarSurface * (m_properties.Cy_0 + m_properties.Cy_BETA * beta + m_properties.Cy_p * m_properties.wingSpan / (2.0*Va) * m_states.p + m_properties.Cy_r * m_properties.wingSpan / (2.0*Va) * m_states.r + m_properties.Cy_DELTA_AIL * inputs.deltaAileron + m_properties.Cy_DELTA_RUD * inputs.deltaRudder);
    double Fz = qBarSurface * (CZ + m_properties.Cz_q * m_properties.wingChord / (2.0*Va) * m_states.q + m_properties.Cz_DELTA_ELE * delElevator);

    Vector3<double> forcesBody{Fx,Fy,Fz};
    return forcesBody;
}

Vector3<double> FixedWing::m_calcMomentAeroBodyFrame(const fixedWingControlInputs& inputs) const {

    // airspeed (Va)
    double Va = airspeed();

    // alpha and beta
    double alpha = 0.0;   // (rad)
    double beta  = 0.0;   // (rad)

    if(Va >= 0.1){
        alpha = std::atan2(m_states.w, m_states.u);  // (rad)
        beta  = std::asin(m_states.v / Va);             // (rad)
    }

    double delAlpha = alpha - radians(m_properties.trimAlpha);

    // q_bar
    double qBarSurface = 0.5 * m_physics.airDensity * Va * Va * m_properties.wingSurface;

    // del_commands
    double delElevator = inputs.deltaElevator - m_properties.trimElevator;

    // Moments in body frame
    double Mx = qBarSurface * m_properties.wingSpan * (m_properties.Cl_0 + m_properties.Cl_BETA * beta + m_properties.Cl_p*m_properties.wingSpan/(2.0*Va)*m_states.p + m_properties.Cl_r*m_properties.wingSpan/(2.0*Va)*m_states.r + m_properties.Cl_DELTA_AIL* inputs.deltaAileron + m_properties.Cl_DELTA_RUD*inputs.deltaRudder);
    double My = qBarSurface * m_properties.wingChord * (m_properties.Cm_0 + m_properties.Cm_ALPHA * delAlpha + m_properties.Cm_q * m_properties.wingChord / (2.0*Va) * m_states.q + m_properties.Cm_DELTA_ELE * delElevator);
    double Mz = qBarSurface * m_properties.wingSpan * (m_properties.Cn_0 + m_properties.Cn_BETA * beta + m_properties.Cn_p*m_properties.wingSpan/(2.0*Va)*m_states.p + m_properties.Cn_r*m_properties.wingSpan/(2.0*Va)*m_states.r + m_properties.Cn_DELTA_AIL* inputs.deltaAileron + m_properties.Cn_DELTA_RUD*inputs.deltaRudder);

    Vector3<double> momentsBody{Mx,My,Mz};
    return momentsBody;
}

fixedWingStates FixedWing::m_calcStatesDerivatives(Vector3<double> forceAeroBody, Vector3<double> momentAeroBody, Vector3<double> externalForceEarth) {

    // Convert external forces in Earth frame to Body frame
    Vector3<double> externalsForcesBody = m_earthToBody(externalForceEarth);

    // Matrix K_BG^-1
    /*
     * |dp|   | 1  sin(phi)tan(theta)  cos(phi)tan(theta) | |p|
     * |dq| = | 0       cos(phi)            -sin(phi)     | |q|
     * |dr|   | 0  sin(phi)sec(theta)  cos(phi)sec(theta) | |r|
    */
    double cosPhi   = std::cos(m_states.phi);
    double sinPhi   = std::sin(m_states.phi);
    double cosTheta = std::cos(m_states.theta);
    double sinTheta = std::sin(m_states.theta);
    double cosPsi   = std::cos(m_states.psi);
    double sinPsi   = std::sin(m_states.psi);
    double tanTheta = std::tan(m_states.theta);
    double secTheta = 1.0 / cosTheta;

    // Calculate the derivatives of the states
    fixedWingStates derivatives{};
    double tau = m_properties.inertiaX * m_properties.inertiaZ - m_properties.inertiaXZ * m_properties.inertiaXZ;

    derivatives.x     = m_states.u * (cosTheta*cosPsi) + m_states.v * (-cosPhi*sinPsi + sinPhi*sinTheta*cosPsi) + m_states.w * (sinPhi*sinPsi + cosPhi*sinTheta*cosPsi) + m_physics.wind[0];
    derivatives.y     = m_states.u * (cosTheta*sinPsi) + m_states.v * (cosPhi*cosPsi + sinPhi*sinTheta*sinPsi) + m_states.w * (-sinPhi*cosPsi + cosPhi*sinTheta*sinPsi) + m_physics.wind[1];
    derivatives.z     = m_states.u * (-sinTheta) + m_states.v * (sinPhi*cosTheta) + m_states.w * (cosPhi*cosTheta) + m_physics.wind[2];
    derivatives.u     = m_states.r * m_states.v - m_states.q * m_states.w - m_physics.gravity * sinTheta + (forceAeroBody[0] + externalsForcesBody[0]) / m_properties.mass;
    derivatives.v     = m_states.p * m_states.w - m_states.r * m_states.u + m_physics.gravity * sinPhi * cosTheta + (forceAeroBody[1] + externalsForcesBody[1]) / m_properties.mass;
    derivatives.w     = m_states.q * m_states.u - m_states.p * m_states.v + m_physics.gravity * cosPhi * cosTheta + (forceAeroBody[2] + externalsForcesBody[2]) / m_properties.mass;
    derivatives.phi   = m_states.p + (m_states.q * sinPhi + m_states.r * cosPhi) * tanTheta;
    derivatives.theta = m_states.q * cosPhi - m_states.r * sinPhi;
    derivatives.psi   = (m_states.q * sinPhi + m_states.r * cosPhi) * secTheta;
    derivatives.p     = 1.0 / tau * (m_states.q * m_states.r * (m_properties.inertiaZ * (m_properties.inertiaY - m_properties.inertiaZ) - m_properties.inertiaXZ * m_properties.inertiaXZ) + m_states.q * m_states.p * (m_properties.inertiaXZ * (m_properties.inertiaZ - m_properties.inertiaY + m_properties.inertiaX))) + 1.0 / tau * (m_properties.inertiaZ * momentAeroBody[0] + m_properties.inertiaXZ * momentAeroBody[2]);
    derivatives.q     = 1.0 / m_properties.inertiaY * (m_states.p * m_states.r * (m_properties.inertiaZ - m_properties.inertiaX) - (m_states.p * m_states.p - m_states.r * m_states.r) * m_properties.inertiaXZ) + 1.0 / m_properties.inertiaY * (momentAeroBody[1]);
    derivatives.r     = 1.0 / tau * (m_states.q * m_states.p * (m_properties.inertiaX * (m_properties.inertiaX - m_properties.inertiaY) + m_properties.inertiaXZ * m_properties.inertiaXZ) - m_states.q * m_states.r * (m_properties.inertiaXZ * (m_properties.inertiaZ - m_properties.inertiaY + m_properties.inertiaX))) + 1.0 / tau * (m_properties.inertiaXZ * momentAeroBody[0] + m_properties.inertiaX * momentAeroBody[2]);

    return derivatives;
}

fixedWingStates FixedWing::m_eulerIntegration(fixedWingStates& derivatives, double dt) const {

    fixedWingStates updatedStates{};

    updatedStates.x     = m_states.x + derivatives.x * dt;
    updatedStates.y     = m_states.y + derivatives.y * dt;
    updatedStates.z     = m_states.z + derivatives.z * dt;
    updatedStates.u     = m_states.u + derivatives.u * dt;
    updatedStates.v     = m_states.v + derivatives.v * dt;
    updatedStates.w     = m_states.w + derivatives.w * dt;
    updatedStates.phi   = m_states.phi + derivatives.phi * dt;
    updatedStates.theta = m_states.theta + derivatives.theta * dt;
    updatedStates.psi   = m_states.psi + derivatives.psi * dt;
    updatedStates.p     = m_states.p + derivatives.p * dt;
    updatedStates.q     = m_states.q + derivatives.q * dt;
    updatedStates.r     = m_states.r + derivatives.r * dt;

    updatedStates.vx    = derivatives.x;
    updatedStates.vy    = derivatives.y;
    updatedStates.vz    = derivatives.z;

    return updatedStates;
}

double FixedWing::airspeed() const {
    return std::sqrt(m_states.u * m_states.u + m_states.v * m_states.v + m_states.w * m_states.w);
}

Matrix3<double> FixedWing::rotationMatrixCBodyEarth() const {
    // Calculate the rotation matrix between the Earth and Body frame
    Matrix3 phiRotationC1   = Matrix3<double>::makeXRotation(m_states.phi);
    Matrix3 thetaRotationC2 = Matrix3<double>::makeYRotation(m_states.theta);
    Matrix3 psiRotationC3   = Matrix3<double>::makeZRotation(m_states.psi);

    // C_BodyEarth
    return (phiRotationC1*thetaRotationC2)*psiRotationC3;
}

Vector3<double> FixedWing::m_earthToBody(const Vector3<double>& earthVector) const {

    Matrix3 C_BodyEarth = rotationMatrixCBodyEarth();

    // Convert Earth frame to Body frame
    Vector3<double> bodyVector = C_BodyEarth * earthVector;
    return bodyVector;
}

Vector3<double> FixedWing::m_bodyToEarth(const Vector3<double>& bodyVector) const {

    Matrix3 C_EarthBody = rotationMatrixCBodyEarth().transpose();

    // Convert Earth frame to Body frame
    Vector3<double> earthVector = C_EarthBody * bodyVector;
    return earthVector;
}

void FixedWing::setInitialStates(initialStates &states) {
    m_states.x = states.position.at(0);
    m_states.y = states.position.at(1);
    m_states.z = states.position.at(2);

    m_states.u = states.speed.at(0);
    m_states.v = states.speed.at(1);
    m_states.w = states.speed.at(2);

    m_states.theta = states.orientation.at(0);
    m_states.phi = states.orientation.at(1);
    m_states.psi = states.orientation.at(2);

    centerOfMass->setPosition({m_states.x, m_states.y, m_states.z});
    centerOfMass->setVelocity({m_states.vx, m_states.vy, m_states.vz});
}

void FixedWing::m_writeLog(Vector3<double> forceAeroBody, Vector3<double> momentAeroBody, Vector3<double> tetherForceBody) {
    fixedWingLogMessage logMessage{};

    logMessage = {
            m_id,
            getInputs().deltaAileron,
            getInputs().deltaElevator,
            getInputs().deltaRudder,
            getInputs().deltaThrust,
            node().position(),
            node().velocity(),
            {m_states.u, m_states.v, m_states.w},
            {m_states.phi, m_states.theta, m_states.psi},
            {m_states.p, m_states.q, m_states.r},
            forceAeroBody,
            momentAeroBody,
            tetherForceBody
    };

    m_logger.writeAircraftMessage(logMessage);
}
