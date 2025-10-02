/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "lateralController.h"

LateralController::LateralController(const worldPhysics& physics, lateralControllerGains& gains, const float rate)
    : m_physics(physics)
    , m_gains(gains)
    , m_dt(1.0f/rate)
    , m_periodL1(25.0f)
    , m_dampingL1(0.707f)
{

}

void LateralController::updateLoiter(const grs::Vec2d& centerWP, const float radius, const float loiterDirection, const bool isTethered) {

    const auto groundspeed = static_cast<float>(std::sqrt(m_aircraftStates.vx * m_aircraftStates.vx + m_aircraftStates.vy * m_aircraftStates.vy));

    // Calculate initial guidance gains
    float qL = m_periodL1 * m_dampingL1 / M_PI;
    const float kL = 4.0f * m_dampingL1 * m_dampingL1;
    m_distanceL1 = qL * groundspeed;

    //Calculate the NE position of the aircraft relative to WP
    const grs::Vec2d distanceNorthEastFromWP = {(m_aircraftStates.x - centerWP[0]),(m_aircraftStates.y - centerWP[1])};

    const auto distanceFromWP = static_cast<float>(distanceNorthEastFromWP.norm());
    grs::Vec2d unitDistanceNorthEastFromWP = grs::normalize(distanceNorthEastFromWP);

    // Adaptive L1 for small radii
    if (m_distanceL1 > radius && std::fabs(distanceFromWP - radius) <= m_distanceL1) {
        m_distanceL1 = std::fmax(std::fabs(distanceFromWP - radius),radius);
        qL = m_distanceL1 / groundspeed;
        m_periodL1 = M_PI * qL / m_dampingL1;
    }

    const float omega = 2.0f * M_PIf / m_periodL1;
    const float kx = omega * omega;
    const float kv = 2.0f * m_dampingL1 * omega;

    // Calculate nu to capture centerWP
    const auto xtrackVelCap = static_cast<float>(unitDistanceNorthEastFromWP[0] * m_aircraftStates.vy - unitDistanceNorthEastFromWP[1] * m_aircraftStates.vx);
    const float ltrackVelCap = - static_cast<float>(m_aircraftStates.vx * unitDistanceNorthEastFromWP[0] + m_aircraftStates.vy * unitDistanceNorthEastFromWP[1]);
    float nu = std::atan2(xtrackVelCap,ltrackVelCap);

    nu = std::clamp(nu, -M_PI_2f, M_PI_2f);

    // Calculate radial position and velocity errors
    const float xtrackVelCirc = -ltrackVelCap;
    const float xtrackErrCirc = distanceFromWP - radius;

    // Calculate acceleration to capture centerWP
    const float latAccDemCap = kL * groundspeed * groundspeed / m_distanceL1 * std::sin(nu);

    // Tangent velocity
    const float velTangent = xtrackVelCap * loiterDirection;

    // Centripetal acceleration
    float latAccDemCircCtrl;
    if (isTethered) {
        latAccDemCircCtrl = 0.0f;
    } else {
        latAccDemCircCtrl = velTangent * velTangent / radius;
    }

    // Control acceleration
    m_integrator += (xtrackErrCirc * m_gains.I) * m_dt;
    m_integrator = std::clamp(m_integrator, -m_gains.iLimit, m_gains.iLimit);

    const float latAccDemPID = xtrackErrCirc * kx * m_gains.P + xtrackVelCirc * kv * m_gains.D + m_integrator;

    // Sum PID control and centripetal acceleration for lateral manoeuvre demand
    const float latAccDemCirc = loiterDirection * (latAccDemPID + latAccDemCircCtrl);

    // Switch between a capture and loiter
    if (xtrackErrCirc > 0.0f && loiterDirection * latAccDemCap < loiterDirection * latAccDemCirc)
    {
        m_latAccDem = latAccDemCap;
    } else {
        m_latAccDem = latAccDemCirc;
    }

    m_ctrlInfo.actualPositionX     = m_aircraftStates.x;
    m_ctrlInfo.targetPositionX     = centerWP[0];
    m_ctrlInfo.actualPositionY     = m_aircraftStates.y;
    m_ctrlInfo.targetPositionY     = centerWP[1];
    m_ctrlInfo.radius              = m_integrator; //radius;
    m_ctrlInfo.kx                  = xtrackErrCirc * kx * m_gains.P; //kx;
    m_ctrlInfo.xtrackPositionError = xtrackErrCirc;
    m_ctrlInfo.kv                  = xtrackVelCirc * kv * m_gains.D; //kv;
    m_ctrlInfo.xtrackVelocityError = xtrackVelCirc;
    m_ctrlInfo.accelerationTarget  = m_latAccDem;
}

float LateralController::navRollCmd() const {
    float cmd = std::atan(m_latAccDem / static_cast<float>(m_physics.gravity));
    cmd = std::clamp(cmd, -m_gains.rollSaturation, m_gains.rollSaturation);
    return cmd;
}
