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
    const std::string idString = "uav" + std::to_string(m_id);

    // Create a cg Frame
    Frame& cg = addFrame(idString + "_centerOfMass", this->bodyOriginFrame(), properties.geom.cgPosition, properties.geom.cgOrientation, Color::HexColor::BLACK);
    setCenterOfMassFrame(cg);

    // Create other Frames and assign function
    Frame& aeroReference       = addFrame(idString + "_aeroRef", this->bodyOriginFrame(), properties.geom.aeroRefPosition, properties.geom.aeroRefOrientation, Color::HexColor::CYAN);
    Frame& propulsionReference = addFrame(idString + "_propulsionRef", this->bodyOriginFrame(), properties.geom.propulsionPosition, properties.geom.propulsionOrientation, Color::HexColor::WHITE);
    Frame& tetherAttach        = addFrame(idString + "_tetherAttach", this->bodyOriginFrame(), properties.geom.tetherAttachPosition, properties.geom.tetherAttachOrientation, Color::HexColor::YELLOW);

    aeroReference.setLoadCallback([this](const Frame& f) {
        const auto [airspeed, alphaRadian, betaRadian] = m_angleOfAttackSideSlip();

        // Calculate forces and moments
        const grs::Vec3d forcesAeroBody  = m_calcForceAeroBodyFrame(airspeed, alphaRadian, betaRadian);
        const grs::Vec3d momentsAeroBody = m_calcMomentAeroBodyFrame(airspeed, alphaRadian, betaRadian);

        f.setForce(forcesAeroBody);
        f.setMoment(momentsAeroBody);
    });

    propulsionReference.setLoadCallback([this](const Frame& f) {
        // TODO Moment?

        f.setForce({m_inputs.deltaThrust,0.0,0.0});
        f.setMoment({0.0,0.0,0.0});
    });

    tetherAttach.setLoadCallback([this](const Frame& f) {
        grs::Vec3d tetherForces = grs::Vec3d::zeros();

        const auto connectedNode = connections[&f.originNode()];
        for(const Node* node : connectedNode) {
            tetherForces += node->force(); // Tether endPoints forces are in Earth Frame
        }

        f.setForce(m_earthToBody(tetherForces));
        f.setMoment({0.0,0.0,0.0});
    });
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

    const fixedWingControlInputs inputs = m_controller->calculateAircraftControlInputs(getId(), getStates(), getCommands());
    setInputs(inputs);

    // Compute rotation matrix
    updateRotationMatrixCBodyEarth();

    // Go through all Frames
    updateForcesOnBody();

    // Dynamics integration
    const fixedWingStates derivatives = m_calcStatesDerivatives();
    m_eulerIntegration(derivatives, dt);

    // phi rotated because aircraft mesh is rotated
    constexpr double meshRollOffset = -M_PI_2;
    setCenterOfMassTransform(grs::TransformD(centerOfMass().position().toOpenglRef(), grs::eulerToQuat(m_states.phi + meshRollOffset,m_states.theta,m_states.psi).nedToYup()));

    m_writeLog();
}

void FixedWing::updateSitl(const double dt, const std::array<uint16_t, MAX_SERVO_CHANNELS>& pwm, const bool shouldMove) {

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
    if (pwm[0] == 0 || inputs.deltaThrust == 0.0) {
        inputs = {0.0,0.0,0.0,0.0};
    }

    m_updateRpmSensor(dt, pwm[2]);

    setInputs(inputs);

    // Compute rotation matrix
    updateRotationMatrixCBodyEarth();

    // Go through all Frames
    updateForcesOnBody();

    // Dynamics integration
    const fixedWingStates derivatives = m_calcStatesDerivatives();
    m_eulerIntegration(derivatives, dt);

    // phi rotated because aircraft mesh is rotated
    constexpr double meshRollOffset = -M_PI_2;
    setCenterOfMassTransform(grs::TransformD(centerOfMass().position().toOpenglRef(), grs::eulerToQuat(m_states.phi + meshRollOffset,m_states.theta,m_states.psi).nedToYup()));

    m_writeLog();
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

grs::Vec3d FixedWing::m_calcForceAeroBodyFrame(const double airspeed, const double alphaRadian, const double betaRadian) const {

    const double delAlpha = alphaRadian;
    // CL and CD
    const double CL = m_properties.CL_0 + m_properties.CL_ALPHA * delAlpha;
    const double CD = m_properties.CD_0 + (CL*CL) / (M_PI * m_properties.oswaldEfficiency * m_properties.aspectRatio);

    // stability vs body frame
    const double cosAlpha = std::cos(alphaRadian);
    const double sinAlpha = std::sin(alphaRadian);
    const double CX = -CD * cosAlpha + CL * sinAlpha;
    const double CZ = -CD * sinAlpha - CL * cosAlpha;

    // q_bar
    const double qBarSurface = 0.5 * m_physics.airDensity * airspeed * airspeed * m_properties.wingSurface;

    // del_commands
    const double delElevator = m_inputs.deltaElevator;

    // Forces in body frame
    double Fx = 0.0;
    double Fy = 0.0;
    double Fz = 0.0;
    if (qBarSurface > 0.0) {
        const double oneOverTwoVa = 0.5 / airspeed;

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

    return grs::Vec3d::zeros();
}

grs::Vec3d FixedWing::m_calcMomentAeroBodyFrame(const double airspeed, const double alphaRadian, const double betaRadian) const {

    const double delAlpha = alphaRadian;

    // q_bar
    const double qBarSurface = 0.5 * m_physics.airDensity * airspeed * airspeed * m_properties.wingSurface;

    // del_commands
    const double delElevator = m_inputs.deltaElevator;

    // Moments in body frame
    double Mx = 0.0;
    double My = 0.0;
    double Mz = 0.0;
    if (qBarSurface > 0.0) {
        const double oneOverTwoVa = 0.5 / airspeed;

        Mx += qBarSurface * m_properties.wingSpan * (m_properties.Cl_0 + m_properties.Cl_BETA * betaRadian + m_properties.Cl_p * m_properties.wingSpan * oneOverTwoVa * m_states.p + m_properties.Cl_r * m_properties.wingSpan * oneOverTwoVa * m_states.r + m_properties.Cl_DELTA_AIL * m_inputs.deltaAileron + m_properties.Cl_DELTA_RUD * m_inputs.deltaRudder);
        My += qBarSurface * m_properties.wingChord * (m_properties.Cm_0 + m_properties.Cm_ALPHA * delAlpha + m_properties.Cm_q * m_properties.wingChord * oneOverTwoVa * m_states.q + m_properties.Cm_DELTA_ELE * delElevator);
        Mz += qBarSurface * m_properties.wingSpan * (m_properties.Cn_0 + m_properties.Cn_BETA * betaRadian + m_properties.Cn_p * m_properties.wingSpan * oneOverTwoVa * m_states.p + m_properties.Cn_r * m_properties.wingSpan * oneOverTwoVa * m_states.r + m_properties.Cn_DELTA_AIL * m_inputs.deltaAileron + m_properties.Cn_DELTA_RUD * m_inputs.deltaRudder);
    }

    if (m_launchTriggered || !m_sitl || m_inFlightLaunch) {
        return {Mx,My,Mz};
    }

    return grs::Vec3d::zeros();
}

fixedWingStates FixedWing::m_calcStatesDerivatives() {

    // Convert external forces in Earth frame to Body frame
    grs::Vec3d totalForceBody  = centerOfMass().force();
    grs::Vec3d totalMomentBody = centerOfMass().moment();

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
    derivatives.u     = m_states.r * m_states.v - m_states.q * m_states.w - m_physics.gravity * sinTheta + (totalForceBody[0]) / m_properties.mass;
    derivatives.v     = m_states.p * m_states.w - m_states.r * m_states.u + m_physics.gravity * sinPhi * cosTheta + (totalForceBody[1]) / m_properties.mass;
    derivatives.w     = m_states.q * m_states.u - m_states.p * m_states.v + m_physics.gravity * cosPhi * cosTheta + (totalForceBody[2]) / m_properties.mass;
    derivatives.phi   = m_states.p + (m_states.q * sinPhi + m_states.r * cosPhi) * tanTheta;
    derivatives.theta = m_states.q * cosPhi - m_states.r * sinPhi;
    derivatives.psi   = (m_states.q * sinPhi + m_states.r * cosPhi) * secTheta;
    derivatives.p     = oneOverTau * (m_states.q * m_states.r * (m_properties.inertiaZ * (m_properties.inertiaY - m_properties.inertiaZ) - m_properties.inertiaXZ * m_properties.inertiaXZ) + m_states.q * m_states.p * (m_properties.inertiaXZ * (m_properties.inertiaZ - m_properties.inertiaY + m_properties.inertiaX))) + oneOverTau * (m_properties.inertiaZ * totalMomentBody[0] + m_properties.inertiaXZ * totalMomentBody[2]);
    derivatives.q     = 1.0 / m_properties.inertiaY * (m_states.p * m_states.r * (m_properties.inertiaZ - m_properties.inertiaX) - (m_states.p * m_states.p - m_states.r * m_states.r) * m_properties.inertiaXZ) + 1.0 / m_properties.inertiaY * (totalMomentBody[1]);
    derivatives.r     = oneOverTau * (m_states.q * m_states.p * (m_properties.inertiaX * (m_properties.inertiaX - m_properties.inertiaY) + m_properties.inertiaXZ * m_properties.inertiaXZ) - m_states.q * m_states.r * (m_properties.inertiaXZ * (m_properties.inertiaZ - m_properties.inertiaY + m_properties.inertiaX))) + oneOverTau * (m_properties.inertiaXZ * totalMomentBody[0] + m_properties.inertiaX * totalMomentBody[2]);

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

                derivatives.psi   = 0.0;
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

    // Simulate the ground plane at z = 0
    if (m_states.z >= 0.0 && m_states.vz > 0.0) { // penetrating and moving further down
        m_states.z  = 0.0;
        m_states.vz = 0.0;

        m_states.w = 0.0;
    }

    // Required by SITL PixHawk (always need to see gravity)
    setAttitude({m_states.phi, m_states.theta, m_states.psi});
    updateWorldInfos({m_states.x, m_states.y, m_states.z}, {m_states.vx, m_states.vy, m_states.vz}, {derivatives.u, derivatives.v, -derivatives.w});

    if (!m_flightLaunchTriggered && m_sitl) {
        setAngularVelocity({m_states.p, m_states.q, m_physics.gravity*std::tan(m_states.phi) / airspeed()});

        // Give a somewhat coherent value by neglecting airspeed derivative, we don't use frame angular acceleration
        // r_dot = m_physics.gravity * sec(phi)^2 * phi_dot / V - tan(phi) * V_dot / (V * V)
        setAngularAcceleration({derivatives.p, derivatives.q,  m_physics.gravity * (std::pow(1.0 / std::cos(m_states.phi), 2) * derivatives.phi) / airspeed()});
    } else {
        setAngularVelocity({m_states.p, m_states.q, m_states.r});
        setAngularAcceleration({derivatives.p, derivatives.q, derivatives.r});
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
    const grs::Matrix3d phiRotationC1   = grs::rotationX(m_states.phi);
    const grs::Matrix3d thetaRotationC2 = grs::rotationY(m_states.theta);
    const grs::Matrix3d psiRotationC3   = grs::rotationZ(m_states.psi);

    // C_BodyEarth
    setRotationMatrixCBodyEarth(phiRotationC1*thetaRotationC2*psiRotationC3);
}

grs::Vec3d FixedWing::m_earthToBody(const grs::Vec3d& earthVector) const {
    return getRotationMatrixCBodyEarth() * earthVector;
}

grs::Vec3d FixedWing::m_bodyToEarth(const grs::Vec3d& bodyVector) const {
    const auto r = getRotationMatrixCBodyEarth();
    return r.transpose() * bodyVector;
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

    setAttitude({m_states.phi, m_states.theta, m_states.psi});
    updateRotationMatrixCBodyEarth();

    updateWorldInfos({m_states.x, m_states.y, m_states.z}, {m_states.vx, m_states.vy, m_states.vz}, {0.0,0.0,0.0});

    constexpr double meshRollOffset = -M_PI_2;
    setCenterOfMassTransform(grs::TransformD(centerOfMass().position().toOpenglRef(), grs::eulerToQuat(m_states.phi + meshRollOffset,m_states.theta,m_states.psi).nedToYup()));
}

double FixedWing::m_pwmToControl(const controlAxis control, const uint16_t pwm) const {

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

void FixedWing::m_writeLog() {

    const fixedWingLogMessage logMessage = {
        m_id,
        m_inputs.deltaAileron,
        m_inputs.deltaElevator,
        m_inputs.deltaRudder,
        m_inputs.deltaThrust,
        centerOfMass().position(),
        centerOfMass().velocity(),
        {m_states.u, m_states.v, m_states.w},
        {m_states.phi, m_states.theta, m_states.psi},
        {m_states.p, m_states.q, m_states.r},
        getFrame("uav" + std::to_string(m_id) + "_aeroRef")->originNode().force(),
        getFrame("uav" + std::to_string(m_id) + "_aeroRef")->originNode().moment(),
        m_earthToBody(getFrame("uav" + std::to_string(m_id) + "_tetherAttach")->originNode().force()),
        m_earthToBody(getFrame("uav" + std::to_string(m_id) + "_tetherAttach")->originNode().moment())
    };

    m_logger.writeAircraftMessage(logMessage);
}
