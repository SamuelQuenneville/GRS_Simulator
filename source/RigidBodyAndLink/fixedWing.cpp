/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "fixedWing.h"

int FixedWing::m_idCounter = 0;

FixedWing::FixedWing(worldPhysics& physics, Logger& logger, const fixedWingProperties& properties)
    : m_id(++m_idCounter)
    , m_physics(physics)
    , m_logger(logger)
    , m_properties(properties)
{

}

FixedWing::FixedWing(worldPhysics& physics, Logger& logger, const fixedWingProperties& properties, std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>>&& controller)
        : FixedWing(physics, logger, properties)
{
    m_controller = std::move(controller);
}

FixedWing::~FixedWing() {
    --m_idCounter;
}

int FixedWing::getId() const {
    return m_id;
}

void FixedWing::update(double dt) {

    fixedWingControlInputs inputs(m_controller->calculateAircraftControlInputs(getId(), getStates(), getCommands()));
    setInputs(inputs);

    Vector3<double> externalForcesEarth = calcExternalForce();

    auto&&[airspeed, alphaRadian, betaRadian] = m_angleOfAttackSideSlip();

    Vector3<double> forcesAeroBody  = m_calcForceAeroBodyFrame(airspeed, alphaRadian, betaRadian);
    Vector3<double> momentsAeroBody = m_calcMomentAeroBodyFrame(airspeed, alphaRadian, betaRadian);

    fixedWingStates derivatives = m_calcStatesDerivatives(forcesAeroBody, momentsAeroBody, externalForcesEarth);
    m_eulerIntegration(derivatives, dt);

    // Set new forces/moments to the node of this body
    centerOfMass->setPosition({m_states.x, m_states.y, m_states.z});
    centerOfMass->setVelocity({m_states.vx, m_states.vy, m_states.vz});
    centerOfMass->setForce(m_bodyToEarth(forcesAeroBody) + externalForcesEarth);
    centerOfMass->setMoment(m_bodyToEarth(momentsAeroBody));

    setAttitude({m_states.phi, m_states.theta, m_states.psi});
    setAngularVelocity({m_states.p, m_states.q, m_states.r});

    // phi rotated because aircraft mesh is rotated
    setTransform(Transform(centerOfMass->position().toOpenglRef(), Quaternion::fromEulerAngles(m_states.phi - M_PI/2,m_states.theta,m_states.psi)));

    // Log
    m_writeLog(forcesAeroBody, momentsAeroBody, m_earthToBody(externalForcesEarth));
}

void FixedWing::updateSitl(const double dt, const std::array<uint16_t, 16> &pwm) {

    fixedWingControlInputs inputs{
        m_pwmToAngle("elevator",pwm[1]),
        m_pwmToAngle("aileron",pwm[0]),
        m_pwmToAngle("rudder",pwm[3]),
        m_pwmToAngle("throttle",pwm[2])
    };

    if (pwm[0] == 0) {
        inputs = {0.0,0.0,0.0,0.0};
    }

    setInputs(inputs);

    Vector3<double> externalForcesEarth = calcExternalForce();

    auto&&[airspeed, alphaRadian, betaRadian] = m_angleOfAttackSideSlip();

    Vector3<double> forcesAeroBody  = m_calcForceAeroBodyFrame(airspeed, alphaRadian, betaRadian);
    Vector3<double> momentsAeroBody = m_calcMomentAeroBodyFrame(airspeed, alphaRadian, betaRadian);

    fixedWingStates derivatives = m_calcStatesDerivatives(forcesAeroBody, momentsAeroBody, externalForcesEarth);
    m_eulerIntegration(derivatives, dt);

    // Set new forces/moments to the node of this body
    centerOfMass->setPosition({m_states.x, m_states.y, m_states.z});
    centerOfMass->setVelocity({m_states.vx, m_states.vy, m_states.vz});
    centerOfMass->setForce(m_bodyToEarth(forcesAeroBody) + externalForcesEarth);
    centerOfMass->setMoment(m_bodyToEarth(momentsAeroBody));

    setAttitude({m_states.phi, m_states.theta, m_states.psi});
    setAngularVelocity({m_states.p, m_states.q, m_states.r});

    // phi rotated because aircraft mesh is rotated
    setTransform(Transform(centerOfMass->position().toOpenglRef(), Quaternion::fromEulerAngles(m_states.phi - M_PI/2,m_states.theta,m_states.psi)));

    // Log
    m_writeLog(forcesAeroBody, momentsAeroBody, m_earthToBody(externalForcesEarth));
}


std::tuple<double, double, double> FixedWing::m_angleOfAttackSideSlip() const {
    // airspeed (Va)
    double Va = airspeed();

    // alphaRadian and betaRadian
    double alphaRadian = 0.0;
    double betaRadian  = 0.0;

    if(Va >= 0.1){
        alphaRadian = std::atan2(m_states.w, m_states.u);
        betaRadian  = std::asin(m_states.v / Va);
    }

    return {Va, alphaRadian, betaRadian};
}

Vector3<double> FixedWing::m_calcForceAeroBodyFrame(double airspeed, double alphaRadian, double betaRadian) const {

    double delAlpha = alphaRadian - radians(m_properties.trimAlpha);
    // CL and CD
    double CL = m_properties.CL_0 + m_properties.CL_ALPHA * delAlpha;
    double CD = m_properties.CD_0 + (CL*CL) / (M_PI * m_properties.oswaldEfficiency * m_properties.aspectRatio);

    // stability vs body frame
    double CX = -CD * std::cos(alphaRadian) + CL * std::sin(alphaRadian);
    double CZ = -CD * std::sin(alphaRadian) - CL * std::cos(alphaRadian);

    // q_bar
    double qBarSurface = 0.5 * m_physics.airDensity * airspeed * airspeed * m_properties.wingSurface;

    // del_commands
    double delElevator = m_inputs.deltaElevator - m_properties.trimElevator;

    // Forces in body frame
    double Fx = m_inputs.deltaThrust;
    double Fy = 0.0;
    double Fz = 0.0;
    if (qBarSurface > 0.0) {
        Fx += qBarSurface * (CX + m_properties.Cx_q * m_properties.wingChord / (2.0 * airspeed) * m_states.q + m_properties.Cx_DELTA_ELE * delElevator);
        Fy += qBarSurface * (m_properties.Cy_0 + m_properties.Cy_BETA * betaRadian + m_properties.Cy_p * m_properties.wingSpan / (2.0 * airspeed) * m_states.p + m_properties.Cy_r * m_properties.wingSpan / (2.0 * airspeed) * m_states.r + m_properties.Cy_DELTA_AIL * m_inputs.deltaAileron + m_properties.Cy_DELTA_RUD * m_inputs.deltaRudder);
        Fz += qBarSurface * (CZ + m_properties.Cz_q * m_properties.wingChord / (2.0 * airspeed) * m_states.q + m_properties.Cz_DELTA_ELE * delElevator);
    }

    return {Fx,Fy,Fz};
}

Vector3<double> FixedWing::m_calcMomentAeroBodyFrame(double airspeed, double alphaRadian, double betaRadian) const {

    double delAlpha = alphaRadian - radians(m_properties.trimAlpha);

    // q_bar
    double qBarSurface = 0.5 * m_physics.airDensity * airspeed * airspeed * m_properties.wingSurface;

    // del_commands
    double delElevator = m_inputs.deltaElevator - m_properties.trimElevator;

    // Moments in body frame
    double Mx = 0.0;
    double My = 0.0;
    double Mz = 0.0;
    if (qBarSurface > 0.0) {
        Mx += qBarSurface * m_properties.wingSpan * (m_properties.Cl_0 + m_properties.Cl_BETA * betaRadian + m_properties.Cl_p * m_properties.wingSpan / (2.0 * airspeed) * m_states.p + m_properties.Cl_r * m_properties.wingSpan / (2.0 * airspeed) * m_states.r + m_properties.Cl_DELTA_AIL * m_inputs.deltaAileron + m_properties.Cl_DELTA_RUD * m_inputs.deltaRudder);
        My += qBarSurface * m_properties.wingChord * (m_properties.Cm_0 + m_properties.Cm_ALPHA * delAlpha + m_properties.Cm_q * m_properties.wingChord / (2.0 * airspeed) * m_states.q + m_properties.Cm_DELTA_ELE * delElevator);
        Mz += qBarSurface * m_properties.wingSpan * (m_properties.Cn_0 + m_properties.Cn_BETA * betaRadian + m_properties.Cn_p * m_properties.wingSpan / (2.0 * airspeed) * m_states.p + m_properties.Cn_r * m_properties.wingSpan / (2.0 * airspeed) * m_states.r + m_properties.Cn_DELTA_AIL * m_inputs.deltaAileron + m_properties.Cn_DELTA_RUD * m_inputs.deltaRudder);
    }

    return {Mx,My,Mz};
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
    double secTheta = 1.0 / cosTheta;
    double tanTheta = sinTheta * secTheta;


    // Calculate the derivatives of the states
    fixedWingStates derivatives{};
    double oneOverTau = 1.0 / (m_properties.inertiaX * m_properties.inertiaZ - m_properties.inertiaXZ * m_properties.inertiaXZ);

    derivatives.x     = m_states.u * (cosTheta*cosPsi) + m_states.v * (-cosPhi*sinPsi + sinPhi*sinTheta*cosPsi) + m_states.w * (sinPhi*sinPsi + cosPhi*sinTheta*cosPsi) + m_physics.wind[0];
    derivatives.y     = m_states.u * (cosTheta*sinPsi) + m_states.v * (cosPhi*cosPsi + sinPhi*sinTheta*sinPsi) + m_states.w * (-sinPhi*cosPsi + cosPhi*sinTheta*sinPsi) + m_physics.wind[1];
    derivatives.z     = m_states.u * (-sinTheta) + m_states.v * (sinPhi*cosTheta) + m_states.w * (cosPhi*cosTheta) + m_physics.wind[2];
    derivatives.u     = m_states.r * m_states.v - m_states.q * m_states.w - m_physics.gravity * sinTheta + (forceAeroBody[0] + externalsForcesBody[0]) / m_properties.mass;
    derivatives.v     = m_states.p * m_states.w - m_states.r * m_states.u + m_physics.gravity * sinPhi * cosTheta + (forceAeroBody[1] + externalsForcesBody[1]) / m_properties.mass;
    derivatives.w     = m_states.q * m_states.u - m_states.p * m_states.v + m_physics.gravity * cosPhi * cosTheta + (forceAeroBody[2] + externalsForcesBody[2]) / m_properties.mass;
    derivatives.phi   = m_states.p + (m_states.q * sinPhi + m_states.r * cosPhi) * tanTheta;
    derivatives.theta = m_states.q * cosPhi - m_states.r * sinPhi;
    derivatives.psi   = (m_states.q * sinPhi + m_states.r * cosPhi) * secTheta;
    derivatives.p     = oneOverTau * (m_states.q * m_states.r * (m_properties.inertiaZ * (m_properties.inertiaY - m_properties.inertiaZ) - m_properties.inertiaXZ * m_properties.inertiaXZ) + m_states.q * m_states.p * (m_properties.inertiaXZ * (m_properties.inertiaZ - m_properties.inertiaY + m_properties.inertiaX))) + oneOverTau * (m_properties.inertiaZ * momentAeroBody[0] + m_properties.inertiaXZ * momentAeroBody[2]);
    derivatives.q     = 1.0 / m_properties.inertiaY * (m_states.p * m_states.r * (m_properties.inertiaZ - m_properties.inertiaX) - (m_states.p * m_states.p - m_states.r * m_states.r) * m_properties.inertiaXZ) + 1.0 / m_properties.inertiaY * (momentAeroBody[1]);
    derivatives.r     = oneOverTau * (m_states.q * m_states.p * (m_properties.inertiaX * (m_properties.inertiaX - m_properties.inertiaY) + m_properties.inertiaXZ * m_properties.inertiaXZ) - m_states.q * m_states.r * (m_properties.inertiaXZ * (m_properties.inertiaZ - m_properties.inertiaY + m_properties.inertiaX))) + oneOverTau * (m_properties.inertiaXZ * momentAeroBody[0] + m_properties.inertiaX * momentAeroBody[2]);

    return derivatives;
}

void FixedWing::m_eulerIntegration(fixedWingStates& derivatives, double dt) {
    m_states.x     += cleanSmallValue(derivatives.x) * dt;
    m_states.y     += cleanSmallValue(derivatives.y) * dt;
    m_states.z     += cleanSmallValue(derivatives.z) * dt;
    m_states.u     += cleanSmallValue(derivatives.u) * dt;
    m_states.v     += cleanSmallValue(derivatives.v) * dt;
    m_states.w     += cleanSmallValue(derivatives.w) * dt;
    m_states.phi   += cleanSmallValue(derivatives.phi) * dt;
    m_states.theta += cleanSmallValue(derivatives.theta) * dt;
    m_states.psi   += cleanSmallValue(derivatives.psi) * dt;
    m_states.p     += cleanSmallValue(derivatives.p) * dt;
    m_states.q     += cleanSmallValue(derivatives.q) * dt;
    m_states.r     += cleanSmallValue(derivatives.r) * dt;

    m_states.vx = cleanSmallValue(derivatives.x);
    m_states.vy = cleanSmallValue(derivatives.y);
    m_states.vz = cleanSmallValue(derivatives.z);

    centerOfMass->setAcceleration({cleanSmallValue(derivatives.u), cleanSmallValue(derivatives.v), -cleanSmallValue(derivatives.w)});

    // Simulate the ground
    if (m_states.z >= -0.001 && derivatives.vz >= -0.001) {
        m_states.z  = 0.0;
        m_states.vz = 0.0;

        m_states.w = 0.0;
    }
}

double FixedWing::airspeed() const {
    return std::sqrt(m_states.u * m_states.u + m_states.v * m_states.v + m_states.w * m_states.w);
}

double FixedWing::groundspeed() const {
    return std::sqrt(m_states.vx * m_states.vx + m_states.vy * m_states.vy);
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
    return rotationMatrixCBodyEarth() * earthVector;
}

Vector3<double> FixedWing::m_bodyToEarth(const Vector3<double>& bodyVector) const {
    return rotationMatrixCBodyEarth().transpose() * bodyVector;
}

void FixedWing::setInitialStates(initialStates& states) {
    m_states.x = states.position.at(0);
    m_states.y = states.position.at(1);
    m_states.z = states.position.at(2);

    m_states.u = states.speed.at(0);
    m_states.v = states.speed.at(1);
    m_states.w = states.speed.at(2);

    m_states.phi   = states.orientation.at(0);
    m_states.theta = states.orientation.at(1);
    m_states.psi   = states.orientation.at(2);

    centerOfMass->setPosition({m_states.x, m_states.y, m_states.z});
    centerOfMass->setVelocity({m_states.vx, m_states.vy, m_states.vz});

    setTransform(Transform(centerOfMass->position().toOpenglRef(), Quaternion::fromEulerAngles(m_states.phi - M_PI/2,m_states.theta,m_states.psi)));
}

double FixedWing::m_pwmToAngle(const std::string& surface, const uint16_t pwm) const {
    if (surface == "aileron") {
        constexpr double trimAileron = 0.0;
        if (pwm < 1500) {
            return trimAileron - (1500 - pwm) * ((trimAileron - m_properties.ailMinDeg) / 500.0);
        } else if (pwm > 1500) {
            return trimAileron + (pwm - 1500) * ((m_properties.ailMaxDeg - trimAileron) / 500.0);
        } else {
            return trimAileron;
        }
    } else if (surface == "elevator") {
        if (pwm < 1500) {
            return m_properties.trimElevator - (1500 - pwm) * ((m_properties.trimElevator - m_properties.eleMinDeg) / 500.0);
        } else if (pwm > 1500) {
            return m_properties.trimElevator + (pwm - 1500) * ((m_properties.eleMaxDeg - m_properties.trimElevator) / 500.0);
        } else {
            return 0.0;
        }
    } else if (surface == "rudder") {
        constexpr double trimRudder = 0.0;
        if (pwm < 1500) {
            return trimRudder - (1500 - pwm) * ((trimRudder - m_properties.rudMinDeg) / 500.0);
        } else if (pwm > 1500) {
            return trimRudder + (pwm - 1500) * ((m_properties.rudMaxDeg - trimRudder) / 500.0);
        } else {
            return trimRudder;
        }
    } else if (surface == "throttle") {
        const double thrust = (pwm - 1100) * 0.044;
        if (thrust < 0.0) {
            return 0.0;
        }
        return thrust;
    } else {
        return 0.0;
    }
}

void FixedWing::m_writeLog(Vector3<double> forceAeroBody, Vector3<double> momentAeroBody, Vector3<double> tetherForceBody) {
    fixedWingLogMessage logMessage{};

    logMessage = {
            m_id,
            m_inputs.deltaAileron,
            m_inputs.deltaElevator,
            m_inputs.deltaRudder,
            m_inputs.deltaThrust,
            centerOfMass->position(),
            centerOfMass->velocity(),
            {m_states.u, m_states.v, m_states.w},
            {m_states.phi, m_states.theta, m_states.psi},
            {m_states.p, m_states.q, m_states.r},
            forceAeroBody,
            momentAeroBody,
            tetherForceBody
    };

    m_logger.writeAircraftMessage(logMessage);
}
