/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "fixedWing.h"

int FixedWing::m_idCounter = 0;

FixedWing::FixedWing(const worldPhysics& physics, Logger& logger, const fixedWingProperties& properties)
    : m_id(++m_idCounter)
    , m_physics(physics)
    , m_logger(logger)
    , m_properties(properties)
{

}

FixedWing::FixedWing(const worldPhysics& physics, Logger& logger, const fixedWingProperties& properties, std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>>&& controller)
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

void FixedWing::update(const double dt) {

    const fixedWingControlInputs inputs(m_controller->calculateAircraftControlInputs(getId(), getStates(), getCommands()));
    setInputs(inputs);

    // Compute rotation matrix
    updateRotationMatrixCBodyEarth();

    // Compute aerodynamic state
    const auto [airspeed, alphaRadian, betaRadian] = m_angleOfAttackSideSlip();

    // Calculate forces and moments
    const Vector3<double> externalForcesEarth = calcExternalForce();
    const Vector3<double> forcesAeroBody  = m_calcForceAeroBodyFrame(airspeed, alphaRadian, betaRadian);
    const Vector3<double> momentsAeroBody = m_calcMomentAeroBodyFrame(airspeed, alphaRadian, betaRadian);

    // Dynamics integration
    const fixedWingStates derivatives = m_calcStatesDerivatives(forcesAeroBody, momentsAeroBody, externalForcesEarth);
    m_eulerIntegration(derivatives, dt);

    // Set new forces/moments to the node of this body
    centerOfMass->setPosition({m_states.x, m_states.y, m_states.z});
    centerOfMass->setVelocity({m_states.vx, m_states.vy, m_states.vz});
    centerOfMass->setForce(m_bodyToEarth(forcesAeroBody) + externalForcesEarth);
    centerOfMass->setMoment(m_bodyToEarth(momentsAeroBody));

    setAttitude({m_states.phi, m_states.theta, m_states.psi});
    setAngularVelocity({m_states.p, m_states.q, m_states.r});

    // phi rotated because aircraft mesh is rotated
    setTransform(Transform(centerOfMass->position().toOpenglRef(), Quaternion::fromEulerAngles(m_states.phi - M_PI/2.0,m_states.theta,m_states.psi)));

    // Log
    m_writeLog(forcesAeroBody, momentsAeroBody, m_earthToBody(externalForcesEarth));
}

void FixedWing::updateSitl(const double dt, const std::array<uint16_t, 16>& pwm, const bool shouldMove) {

    // Detect SITL mode once
    if (!m_sitl) {
        m_sitl = true;
    }

    // Read from .csv(GCS pass through pixhawk) if we should move (launch triggered)
    m_flightLaunchTriggered = shouldMove;

    // Convert PWM to normalized control inputs
    fixedWingControlInputs inputs{
        m_pwmToControl(ELEVATOR, pwm[1]),
         m_pwmToControl(AILERON,  pwm[0]),
         m_pwmToControl(RUDDER,   pwm[3]),
         m_pwmToControl(THROTTLE, pwm[2])
    };

    // All-zero input guard (fail-safe or inactive)
    if (pwm[0] == 0) {
        inputs = {0.0,0.0,0.0,0.0};
    }

    m_updateRpmSensor(dt, pwm[2]);

    setInputs(inputs);

    // Compute rotation matrix
    updateRotationMatrixCBodyEarth();

    // Compute aerodynamic state
    const auto [airspeed, alphaRadian, betaRadian] = m_angleOfAttackSideSlip();

    // Calculate forces and moments
    const Vector3<double> externalForcesEarth = calcExternalForce();
    const Vector3<double> forcesAeroBody  = m_calcForceAeroBodyFrame(airspeed, alphaRadian, betaRadian);
    const Vector3<double> momentsAeroBody = m_calcMomentAeroBodyFrame(airspeed, alphaRadian, betaRadian);

    // Dynamics integration
    const fixedWingStates derivatives = m_calcStatesDerivatives(forcesAeroBody, momentsAeroBody, externalForcesEarth);
    m_eulerIntegration(derivatives, dt);

    // Set new forces/moments to the node of this body
    centerOfMass->setPosition({m_states.x, m_states.y, m_states.z});
    centerOfMass->setVelocity({m_states.vx, m_states.vy, m_states.vz});
    centerOfMass->setForce(m_bodyToEarth(forcesAeroBody) + externalForcesEarth);
    centerOfMass->setMoment(m_bodyToEarth(momentsAeroBody));

    setAttitude({m_states.phi, m_states.theta, m_states.psi});
    setAngularVelocity({m_states.p, m_states.q, m_states.r});

    // phi rotated because aircraft mesh is rotated
    setTransform(Transform(centerOfMass->position().toOpenglRef(), Quaternion::fromEulerAngles(m_states.phi - M_PI/2.0,m_states.theta,m_states.psi)));

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

Vector3<double> FixedWing::m_calcForceAeroBodyFrame(const double airspeed, const double alphaRadian, const double betaRadian) const {

    const double delAlpha = alphaRadian;
    // CL and CD
    const double CL = m_properties.CL_0 + m_properties.CL_ALPHA * delAlpha;
    const double CD = m_properties.CD_0 + (CL*CL) / (M_PI * m_properties.oswaldEfficiency * m_properties.aspectRatio);

    // stability vs body frame
    const double CX = -CD * std::cos(alphaRadian) + CL * std::sin(alphaRadian);
    const double CZ = -CD * std::sin(alphaRadian) - CL * std::cos(alphaRadian);

    // q_bar
    const double qBarSurface = 0.5 * m_physics.airDensity * airspeed * airspeed * m_properties.wingSurface;

    // 1 / 2*Va
    const double oneOverTwoVa = 1.0 / (2.0 * airspeed);

    // del_commands
    const double delElevator = m_inputs.deltaElevator;

    // Forces in body frame
    double Fx = m_inputs.deltaThrust;
    double Fy = 0.0;
    double Fz = 0.0;
    if (qBarSurface > 0.0) {
        Fx += qBarSurface * (CX + m_properties.Cx_q * m_properties.wingChord * oneOverTwoVa * m_states.q + m_properties.Cx_DELTA_ELE * delElevator);
        Fy += qBarSurface * (m_properties.Cy_0 + m_properties.Cy_BETA * betaRadian + m_properties.Cy_p * m_properties.wingSpan * oneOverTwoVa * m_states.p + m_properties.Cy_r * m_properties.wingSpan * oneOverTwoVa * m_states.r + m_properties.Cy_DELTA_AIL * m_inputs.deltaAileron + m_properties.Cy_DELTA_RUD * m_inputs.deltaRudder);
        Fz += qBarSurface * (CZ + m_properties.Cz_q * m_properties.wingChord * oneOverTwoVa * m_states.q + m_properties.Cz_DELTA_ELE * delElevator);
    }

    // Easiest place to put it, don't required a transformation in Earth reference frame
    if (m_inLaunch) {
        Fx += m_properties.mass * m_launcherAccel;
    }

    if (m_launchTriggered || !m_sitl || m_inFlightLaunch) {
        return {Fx,Fy,Fz};
    }

    return Vector3<double>::zeros();
}

Vector3<double> FixedWing::m_calcMomentAeroBodyFrame(const double airspeed, const double alphaRadian, const double betaRadian) const {

    const double delAlpha = alphaRadian;

    // q_bar
    const double qBarSurface = 0.5 * m_physics.airDensity * airspeed * airspeed * m_properties.wingSurface;

    // 1 / 2*Va
    const double oneOverTwoVa = 1.0 / (2.0 * airspeed);

    // del_commands
    const double delElevator = m_inputs.deltaElevator;

    // Moments in body frame
    double Mx = 0.0;
    double My = 0.0;
    double Mz = 0.0;
    if (qBarSurface > 0.0) {
        Mx += qBarSurface * m_properties.wingSpan * (m_properties.Cl_0 + m_properties.Cl_BETA * betaRadian + m_properties.Cl_p * m_properties.wingSpan * oneOverTwoVa * m_states.p + m_properties.Cl_r * m_properties.wingSpan * oneOverTwoVa * m_states.r + m_properties.Cl_DELTA_AIL * m_inputs.deltaAileron + m_properties.Cl_DELTA_RUD * m_inputs.deltaRudder);
        My += qBarSurface * m_properties.wingChord * (m_properties.Cm_0 + m_properties.Cm_ALPHA * delAlpha + m_properties.Cm_q * m_properties.wingChord * oneOverTwoVa * m_states.q + m_properties.Cm_DELTA_ELE * delElevator);
        Mz += qBarSurface * m_properties.wingSpan * (m_properties.Cn_0 + m_properties.Cn_BETA * betaRadian + m_properties.Cn_p * m_properties.wingSpan * oneOverTwoVa * m_states.p + m_properties.Cn_r * m_properties.wingSpan * oneOverTwoVa * m_states.r + m_properties.Cn_DELTA_AIL * m_inputs.deltaAileron + m_properties.Cn_DELTA_RUD * m_inputs.deltaRudder);
    }

    if (m_launchTriggered || !m_sitl || m_inFlightLaunch) {
        return {Mx,My,Mz};
    }

    return Vector3<double>::zeros();
}

fixedWingStates FixedWing::m_calcStatesDerivatives(Vector3<double> forceAeroBody, Vector3<double> momentAeroBody, const Vector3<double>& externalForceEarth) {

    // Convert external forces in Earth frame to Body frame
    Vector3<double> externalsForcesBody = m_earthToBody(externalForceEarth);

    // Matrix K_BG^-1
    /*
     * |dp|   | 1  sin(phi)tan(theta)  cos(phi)tan(theta) | |p|
     * |dq| = | 0       cos(phi)            -sin(phi)     | |q|
     * |dr|   | 0  sin(phi)sec(theta)  cos(phi)sec(theta) | |r|
    */
    const double cosPhi   = std::cos(m_states.phi);
    const double sinPhi   = std::sin(m_states.phi);
    const double cosTheta = std::cos(m_states.theta);
    const double sinTheta = std::sin(m_states.theta);
    const double cosPsi   = std::cos(m_states.psi);
    const double sinPsi   = std::sin(m_states.psi);
    const double secTheta = 1.0 / cosTheta;
    const double tanTheta = sinTheta * secTheta;


    // Calculate the derivatives of the states
    fixedWingStates derivatives{};
    const double oneOverTau = 1.0 / (m_properties.inertiaX * m_properties.inertiaZ - m_properties.inertiaXZ * m_properties.inertiaXZ);

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

    // Required by SITL PixHawk (always need to see gravity)
    centerOfMass->setAcceleration({derivatives.u, derivatives.v, -derivatives.w});

    if (m_sitl) {
        if (!m_inFlightLaunch) {
            if (!m_launchTriggered) {
                derivatives.x = 0.0;
                derivatives.y = 0.0;
                derivatives.z = 0.0;
                derivatives.u = 0.0;
                derivatives.v = 0.0;
                derivatives.w = 0.0;
            }

            if (m_inLaunch) {
                derivatives.p = 0.0;
                derivatives.q = 0.0;
                derivatives.r = 0.0;
            }
        } else {
            if (!m_flightLaunchTriggered) {
                derivatives.x = 0.0;
                derivatives.y = 0.0;
                derivatives.z = 0.0;
            }
        }
    }

    return derivatives;
}

void FixedWing::m_eulerIntegration(const fixedWingStates& derivatives, const double dt) {

    m_states.x     += derivatives.x * dt;
    m_states.y     += derivatives.y * dt;
    m_states.z     += derivatives.z * dt;
    m_states.u     += derivatives.u * dt;
    m_states.v     += derivatives.v * dt;
    m_states.w     += derivatives.w * dt;
    m_states.phi   += derivatives.phi * dt;
    m_states.theta += derivatives.theta * dt;
    m_states.psi   += derivatives.psi * dt;
    m_states.p     += derivatives.p * dt;
    m_states.q     += derivatives.q * dt;
    m_states.r     += derivatives.r * dt;

    m_states.vx = derivatives.x;
    m_states.vy = derivatives.y;
    m_states.vz = derivatives.z;

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

void FixedWing::updateRotationMatrixCBodyEarth() {
    // Calculate the rotation matrix between the Earth and Body frame
    Matrix3 phiRotationC1   = Matrix3<double>::makeXRotation(m_states.phi);
    Matrix3 thetaRotationC2 = Matrix3<double>::makeYRotation(m_states.theta);
    Matrix3 psiRotationC3   = Matrix3<double>::makeZRotation(m_states.psi);

    // C_BodyEarth
    m_rotationMatrixCBodyEarth = (phiRotationC1*thetaRotationC2)*psiRotationC3;
}

Vector3<double> FixedWing::m_earthToBody(const Vector3<double>& earthVector) {
    return m_rotationMatrixCBodyEarth * earthVector;
}

Vector3<double> FixedWing::m_bodyToEarth(const Vector3<double>& bodyVector) {
    return m_rotationMatrixCBodyEarth.transpose() * bodyVector;
}

void FixedWing::setInitialStates(initialStates& states) {
    m_states.x = states.position[0];
    m_states.y = states.position[1];
    m_states.z = states.position[2];

    m_states.u = states.speed[0];
    m_states.v = states.speed[1];
    m_states.w = states.speed[2];

    m_states.phi   = states.orientation[0];
    m_states.theta = states.orientation[1];
    m_states.psi   = states.orientation[2];

    centerOfMass->setPosition({m_states.x, m_states.y, m_states.z});
    centerOfMass->setVelocity({m_states.vx, m_states.vy, m_states.vz});

    setTransform(Transform(centerOfMass->position().toOpenglRef(), Quaternion::fromEulerAngles(m_states.phi - M_PI/2.0,m_states.theta,m_states.psi)));
}

double FixedWing::m_pwmToControl(const controlAxe control, const uint16_t pwm) const {

    constexpr double PWM_MIN  = 1100;
    constexpr double PWM_MEAN = 1500;
    constexpr double PWM_MAX  = 1900;

    switch (control) {
        case AILERON: {
            if (pwm < PWM_MEAN) {
                return - m_properties.ailMinDeg / (PWM_MEAN - PWM_MIN) * (pwm - PWM_MEAN);
            }

            return m_properties.ailMaxDeg / (PWM_MAX - PWM_MEAN) * (pwm - PWM_MEAN);
        }

        case ELEVATOR: {
            // PixHawk defined a negative elevator down (pwm < 1500), here we defined a negative elevator up
            if (pwm < PWM_MEAN) {
                return m_properties.eleMinDeg / (PWM_MEAN - PWM_MIN) * (pwm - PWM_MEAN);
            }

            return - m_properties.eleMaxDeg / (PWM_MAX - PWM_MEAN) * (pwm - PWM_MEAN);

        }

        case RUDDER: {
            if (pwm < PWM_MEAN) {
                return - m_properties.rudMinDeg / (PWM_MEAN - PWM_MIN) * (pwm - PWM_MEAN);
            }

            return m_properties.rudMaxDeg / (PWM_MAX - PWM_MEAN) * (pwm - PWM_MEAN);
        }

        case THROTTLE: {
            constexpr double RPM_MAX = 9000.0;

            const double rpm = (pwm - PWM_MIN) * RPM_MAX / (PWM_MAX - PWM_MIN);
            if (rpm < 10.0) {
                return 0.0;
            }

            const double thrust = rpm2thrust(static_cast<float>(airspeed()), static_cast<float>(rpm));
            if (thrust < 0.0) {
                return 0.0;
            }
            return thrust;
        }

        default:
            return 0.0;
    }
}

void FixedWing::m_updateRpmSensor(const double dt, const double throttleInput) {

    double rpm = (throttleInput - 1100.0) * 9000.0/900.0;
    if (rpm < 10.0) {
        rpm = 0.0;
    }

    // 1st order lag: dRPM/dt = (desired - current) / tau
    const double rpm_dot = (rpm - m_rpm) / m_rpmTau;

    // Integrate
    m_rpm += rpm_dot * dt;
}

void FixedWing::m_writeLog(const Vector3<double>& forceAeroBody, const Vector3<double>& momentAeroBody, const Vector3<double>& tetherForceBody) {

    fixedWingLogMessage logMessage = {
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
