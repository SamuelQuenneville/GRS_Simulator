/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "fixedWing.h"

int FixedWing::m_idCounter = 0;

FixedWing::FixedWing(const worldPhysics& physics, const fixedWingProperties& properties)
    : m_id(++m_idCounter)
    , m_physics(physics)
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

        f.setForce(grs::Vec3d(m_inputs.deltaThrust,0.0,0.0));
        f.setMoment(grs::Vec3d::zeros());
    });

    tetherAttach.setLoadCallback([this](const Frame& f) {
        grs::Vec3d tetherForces = grs::Vec3d::zeros();

        const auto connectedNode = connections[&f.originNode()];
        for(const Node* node : connectedNode) {
            tetherForces += node->force(); // Tether endPoints forces are in Earth Frame
        }

        f.setForce(m_earthToBody(tetherForces));
        f.setMoment(grs::Vec3d::zeros());
    });
}

FixedWing::FixedWing(const worldPhysics& physics, const fixedWingProperties& properties, std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>>&& controller)
        : FixedWing(physics, properties)
{
    m_controller = std::move(controller);
}

FixedWing::~FixedWing() {
    --m_idCounter;
}

int FixedWing::getId() const {
    return m_id;
}

void FixedWing::update(const double dt, const WindEnvironment& windEnv) {
    m_simTime += dt;

    // Compute rotation matrix
    updateRotationMatrixCBodyEarth();

    m_windEarth = windEnv.getTotalWind(centerOfMass().position(), m_simTime);
    m_windBody = m_earthToBody(m_windEarth);
    centerOfMass().setWind(m_windBody);

    const fixedWingControlInputs inputs = m_controller->calculateAircraftControlInputs(getId(), getStates(), getCommands());
    setInputs(inputs);

    // Go through all Frames
    updateForcesOnBody();

    // Dynamics integration
    if (m_inLaunch) {
        m_integrateCatapult(dt);
    } else {
        const fixedWingStates derivatives = m_calcStatesDerivatives();
        m_eulerIntegration(derivatives, dt);
    }

    // phi rotated because aircraft mesh is rotated
    constexpr double meshRollOffset = -M_PI_2;
    setCenterOfMassTransform(grs::TransformD(centerOfMass().position().toOpenglRef(), grs::eulerToQuat(m_states.phi + meshRollOffset,m_states.theta,m_states.psi).nedToYup()));

    m_writeLog();
}

void FixedWing::updateSitl(const double dt, const WindEnvironment& windEnv, const std::array<uint16_t, MAX_SERVO_CHANNELS>& pwm, const bool shouldMove) {
    m_simTime += dt;

    // Compute rotation matrix
    updateRotationMatrixCBodyEarth();

    m_windEarth = windEnv.getTotalWind(centerOfMass().position(), m_simTime);
    m_windBody = m_earthToBody(m_windEarth);
    centerOfMass().setWind(m_windBody);

    // Detect SITL mode once
    if (!m_sitl) {
        m_sitl = true;
    }

    // Read from .csv(GCS pass through pixhawk) if we should move (launch triggered)
    m_flightLaunchTriggered = shouldMove;

    // Convert PWM to normalized control inputs
    fixedWingControlInputs inputs{
         .deltaElevator = m_pwmToControl(ELEVATOR, pwm[1]),
         .deltaAileron  = m_pwmToControl(AILERON,  pwm[0]),
         .deltaRudder   = m_pwmToControl(RUDDER,   pwm[3]),
         .deltaThrust   = m_pwmToControl(THROTTLE, pwm[2])
    };

    // All-zero input guard (fail-safe or inactive)
    if (pwm[0] == 0) {
        inputs = {.deltaElevator = 0.0, .deltaAileron = 0.0, .deltaRudder = 0.0, .deltaThrust = 0.0};
    }

    // Log
    std::ostringstream msg;
    msg << std::fixed << std::setprecision(4)
        << m_simTime << "," << Logger::nowWallTimeMs() << "," << m_id << ","
        << pwm[0] << "," << pwm[1] << "," << pwm[2] << "," << pwm[3] << ","
        << inputs.deltaAileron << "," << inputs.deltaElevator << "," << inputs.deltaThrust << "," << inputs.deltaRudder;
    Logger::instance().log(LogType::SitlIn, msg.str());


    m_updateRpmSensor(dt, pwm[2]);

    setInputs(inputs);

    // Go through all Frames
    updateForcesOnBody();

    // Dynamics integration
    if (m_inLaunch) {
        m_integrateCatapult(dt);
    } else {
        const fixedWingStates derivatives = m_calcStatesDerivatives();
        m_eulerIntegration(derivatives, dt);
    }

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

    derivatives.x     = m_states.u * (cosTheta*cosPsi) + m_states.v * (-cosPhi*sinPsi + sinPhi*sinTheta*cosPsi) + m_states.w * (sinPhi*sinPsi + cosPhi*sinTheta*cosPsi) + m_windEarth[0];
    derivatives.y     = m_states.u * (cosTheta*sinPsi) + m_states.v * (cosPhi*cosPsi + sinPhi*sinTheta*sinPsi) + m_states.w * (-sinPhi*cosPsi + cosPhi*sinTheta*sinPsi) + m_windEarth[1];
    derivatives.z     = m_states.u * (-sinTheta) + m_states.v * (sinPhi*cosTheta) + m_states.w * (cosPhi*cosTheta) + m_windEarth[2];
    derivatives.u     = m_states.r * m_states.v - m_states.q * m_states.w - m_physics.gravity * sinTheta + totalForceBody[0] / m_properties.mass;
    derivatives.v     = m_states.p * m_states.w - m_states.r * m_states.u + m_physics.gravity * sinPhi * cosTheta + totalForceBody[1] / m_properties.mass;
    derivatives.w     = m_states.q * m_states.u - m_states.p * m_states.v + m_physics.gravity * cosPhi * cosTheta + totalForceBody[2] / m_properties.mass;
    derivatives.phi   = m_states.p + (m_states.q * sinPhi + m_states.r * cosPhi) * tanTheta;
    derivatives.theta = m_states.q * cosPhi - m_states.r * sinPhi;
    derivatives.psi   = (m_states.q * sinPhi + m_states.r * cosPhi) * secTheta;
    derivatives.p     = oneOverTau * (m_states.q * m_states.r * (m_properties.inertiaZ * (m_properties.inertiaY - m_properties.inertiaZ) - m_properties.inertiaXZ * m_properties.inertiaXZ) + m_states.q * m_states.p * (m_properties.inertiaXZ * (m_properties.inertiaZ - m_properties.inertiaY + m_properties.inertiaX))) + oneOverTau * (m_properties.inertiaZ * totalMomentBody[0] + m_properties.inertiaXZ * totalMomentBody[2]);
    derivatives.q     = 1.0 / m_properties.inertiaY * (m_states.p * m_states.r * (m_properties.inertiaZ - m_properties.inertiaX) - (m_states.p * m_states.p - m_states.r * m_states.r) * m_properties.inertiaXZ) + 1.0 / m_properties.inertiaY * (totalMomentBody[1]);
    derivatives.r     = oneOverTau * (m_states.q * m_states.p * (m_properties.inertiaX * (m_properties.inertiaX - m_properties.inertiaY) + m_properties.inertiaXZ * m_properties.inertiaXZ) - m_states.q * m_states.r * (m_properties.inertiaXZ * (m_properties.inertiaZ - m_properties.inertiaY + m_properties.inertiaX))) + oneOverTau * (m_properties.inertiaXZ * totalMomentBody[0] + m_properties.inertiaX * totalMomentBody[2]);

    if (m_isTranslationHeld()) {
        derivatives.x = 0.0;
        derivatives.y = 0.0;
        derivatives.z = 0.0;

        if (!m_inFlightLaunch) {
            // On the launcher, waiting for the launch
            derivatives.u = 0.0;
            derivatives.v = 0.0;
            derivatives.w = 0.0;
        } else {
            // In-flight launch, waiting for shouldMove
            derivatives.psi = 0.0;
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

    // Inertial acceleration of the CG in NED. With the air-relative body velocities used here,
    // d/dt(C_EarthBody * v_air) = C_EarthBody * F_body/m + g (the wind time derivative is neglected).
    // The rotation matrix is the one the forces were computed with (start of step), so the IMU reads exactly F_body/m.
    const grs::Matrix3d C_EarthBody = getRotationMatrixCBodyEarth().transpose();
    grs::Vec3d accelNed = C_EarthBody * (centerOfMass().force() * (1.0 / m_properties.mass)) + grs::Vec3d(0.0, 0.0, m_physics.gravity);

    if (m_isTranslationHeld()) {
        accelNed = grs::Vec3d::zeros();
    }

    // Simulate the ground plane at z = 0
    if (m_states.z >= 0.0 && m_states.vz > 0.0) { // penetrating and moving further down
        m_states.z  = 0.0;
        m_states.vz = 0.0;

        m_states.w = 0.0;

        // The ground can only push up (normal force): no downward acceleration while in contact
        accelNed[2] = std::min(accelNed[2], 0.0);
    }

    setAttitude({m_states.phi, m_states.theta, m_states.psi});
    updateWorldInfos({m_states.x, m_states.y, m_states.z}, {m_states.vx, m_states.vy, m_states.vz}, accelNed);

    // Accelerometer output (what a Pixhawk IMU measures and logs)
    setSpecificForceBody(m_calcSpecificForceBody(accelNed));

    if (!m_flightLaunchTriggered && m_sitl) {
        double r = 0.0;
        double dr = 0.0;
        if (airspeed() > 0.1) {
            r = m_physics.gravity*std::tan(m_states.phi) / airspeed();
            dr = m_physics.gravity * (std::pow(1.0 / std::cos(m_states.phi), 2) * derivatives.phi) / airspeed();
        }
        setAngularVelocity({m_states.p, m_states.q, r});

        // Give a somewhat coherent value by neglecting airspeed derivative, we don't use frame angular acceleration
        // r_dot = m_physics.gravity * sec(phi)^2 * phi_dot / V - tan(phi) * V_dot / (V * V)
        setAngularAcceleration({derivatives.p, derivatives.q,  dr});
    } else {
        setAngularVelocity({m_states.p, m_states.q, m_states.r});
        setAngularAcceleration({derivatives.p, derivatives.q, derivatives.r});
    }

}

bool FixedWing::m_isTranslationHeld() const {
    if (!m_sitl) {
        return false;
    }

    if (m_inFlightLaunch) {
        return !m_flightLaunchTriggered;
    }

    return !m_launchTriggered;
}

grs::Vec3d FixedWing::m_calcSpecificForceBody(const grs::Vec3d& accelNed) const {
    // f_b = C_BodyEarth * (a_NED - g_NED), g_NED = [0 0 g]
    return getRotationMatrixCBodyEarth() * (accelNed - grs::Vec3d(0.0, 0.0, m_physics.gravity));
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

void FixedWing::m_integrateCatapult(const double dt) {
    // Rail direction in NED
    const double cp = std::cos(m_launcherPitch);
    const double sp = std::sin(m_launcherPitch);
    const double cy = std::cos(m_launcherYaw);
    const double sy = std::sin(m_launcherYaw);

    const grs::Vec3d railDir = {cp * cy, cp * sy, -sp};

    // Current speed along rail
    double V = m_states.u;

    // Accelerations
    const double thrustAccel = m_inputs.deltaThrust / m_properties.mass;
    const double gravityAlongRail = m_physics.gravity * sp;
    const double accel = m_launcherAccel + thrustAccel - gravityAlongRail;

    // Integrate constrained longitudinal speed
    V += accel * dt;

    // Constrained body velocities
    m_states.u = V;
    m_states.v = 0.0;
    m_states.w = 0.0;

    // Constrained angular rates
    m_states.p = 0.0;
    m_states.q = 0.0;
    m_states.r = 0.0;

    // Earth-frame velocity
    const grs::Vec3d velEarth = V * railDir;

    m_states.vx = velEarth[0];
    m_states.vy = velEarth[1];
    m_states.vz = velEarth[2];

    // Integrate constrained position
    m_states.x += velEarth[0] * dt;
    m_states.y += velEarth[1] * dt;
    m_states.z += velEarth[2] * dt;

    // Inertial acceleration along the rail (NED) and accelerometer output
    const grs::Vec3d accelNed = accel * railDir;

    setAttitude({m_states.phi, m_states.theta, m_states.psi});
    updateWorldInfos({m_states.x, m_states.y, m_states.z}, {m_states.vx, m_states.vy, m_states.vz}, accelNed);
    setSpecificForceBody(m_calcSpecificForceBody(accelNed));

    setAngularVelocity({m_states.p, m_states.q, m_states.r});
    setAngularAcceleration({0.0, 0.0, 0.0});
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

    m_launcherPitch = m_states.theta;
    m_launcherYaw = m_states.psi;

    setAttitude({m_states.phi, m_states.theta, m_states.psi});
    updateRotationMatrixCBodyEarth();

    updateWorldInfos({m_states.x, m_states.y, m_states.z}, {m_states.vx, m_states.vy, m_states.vz}, {0.0,0.0,0.0});
    setSpecificForceBody(m_calcSpecificForceBody(grs::Vec3d::zeros()));

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

    double rpm = (throttleInput - 1100.0) * 9000.0/800.0;
    if (rpm < 10.0) {
        rpm = 0.0;
    }

    // 1st order lag: dRPM/dt = (desired - current) / tau
    const double rpm_dot = (rpm - m_rpm) / m_rpmTau;

    // Integrate
    m_rpm += rpm_dot * dt;
}

void FixedWing::m_writeLog() {

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(4)
        << m_simTime    << "," << Logger::nowWallTimeMs() << "," << m_id << ","
        << m_states.x   << "," << m_states.y     << "," << m_states.z   << ","
        << m_states.vx  << "," << m_states.vy    << "," << m_states.vz  << ","
        << m_states.u   << "," << m_states.v     << "," << m_states.w   << ","
        << m_states.phi << "," << m_states.theta << "," << m_states.psi << ","
        << m_states.p   << "," << m_states.q     << "," << m_states.r   << ","
        << m_windEarth[0] << "," << m_windEarth[1] << "," << m_windEarth[2] << ","
        << m_windBody[0]  << "," << m_windBody[1]  << "," << m_windBody[2] << ","
        << centerOfMass().acceleration()[0] << "," << centerOfMass().acceleration()[1] << "," << centerOfMass().acceleration()[2] << ","
        << getSpecificForceBody()[0] << "," << getSpecificForceBody()[1] << "," << getSpecificForceBody()[2];

    Logger::instance().log(LogType::Aircraft, msg.str());

    logFramesData(m_simTime);
}

bodyRenderData FixedWing::packRenderData() const {

    std::vector<frameRenderData> framesData;

    for (const auto& f : frames()) {
        framesData.push_back(f->packRenderData());
    }

    return bodyRenderData{.type = RenderObjectType::AIRCRAFT, .id = m_id, .transform = transform(), .frames = framesData};
}
