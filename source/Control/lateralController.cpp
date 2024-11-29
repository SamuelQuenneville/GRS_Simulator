/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "lateralController.h"

LateralController::LateralController(worldPhysics& physics, lateralControllerGains& gains, float rate)
    : m_physics(physics)
    , m_gains(gains)
    , m_dt(1.0f/rate)
    , m_periodL1(25.0)
    , m_dampingL1(0.707)
{

}

void LateralController::updateLoiter(const Vector2<double>& centerWP, float radius, float loiterDirection, bool isTethered) {

    auto groundspeed = static_cast<float>(std::sqrt(m_aircraftStates.vx * m_aircraftStates.vx + m_aircraftStates.vy * m_aircraftStates.vy));

    // Calculate initial guidance gains
    float qL = m_periodL1 * m_dampingL1 / M_PI;
    float kL = 4.0f * m_dampingL1 * m_dampingL1;
    m_distanceL1 = qL * groundspeed;

    //Calculate the NE position of the aircraft relative to WP
    Vector2<double> distanceNorthEastFromWP = {(m_aircraftStates.x - centerWP.x()),(m_aircraftStates.y - centerWP.y())};

    auto distanceFromWP = static_cast<float>(distanceNorthEastFromWP.length());
    Vector2<double> unitDistanceNorthEastFromWP = distanceNorthEastFromWP.normalize();

    // Adaptive L1 for small radii
    if (m_distanceL1 > radius && std::fabs(distanceFromWP - radius) <= m_distanceL1) {
        m_distanceL1 = std::fmax(std::fabs(distanceFromWP - radius),radius);
        qL = m_distanceL1 / groundspeed;
        m_periodL1 = M_PI * qL / m_dampingL1;
    }

    float omega = M_2PI / m_periodL1;
    float kx = omega * omega;
    float kv = 2.0f * m_dampingL1 * omega;

    // Calculate nu to capture centerWP
    auto xtrackVelCap = static_cast<float>(unitDistanceNorthEastFromWP.x() * m_aircraftStates.vy - unitDistanceNorthEastFromWP.y() * m_aircraftStates.vx);
    float ltrackVelCap = - static_cast<float>(m_aircraftStates.vx * unitDistanceNorthEastFromWP.x() + m_aircraftStates.vy * unitDistanceNorthEastFromWP.y());
    float nu = std::atan2(xtrackVelCap,ltrackVelCap);

    nu = constrainFloat(nu, -M_PI / 2.0f, M_PI / 2.0f);

    // Calculate radial position and velocity errors
    float xtrackVelCirc = -ltrackVelCap;
    float xtrackErrCirc = distanceFromWP - radius;

    // Calculate acceleration to capture centerWP
    float latAccDemCap = kL * groundspeed * groundspeed / m_distanceL1 * std::sin(nu);

    // Tangent velocity
    float velTangent = xtrackVelCap * loiterDirection;

    // Centripetal acceleration
    float latAccDemCircCtrl;
    if (isTethered) {
        latAccDemCircCtrl = 0.0f;
    } else {
        latAccDemCircCtrl = velTangent * velTangent / radius;
    }

    // Control acceleration
    m_integrator += (xtrackErrCirc * m_gains.I) * m_dt;
    m_integrator = constrainFloat(m_integrator, -m_gains.iLimit, m_gains.iLimit);

    float latAccDemPID = xtrackErrCirc * kx * m_gains.P + xtrackVelCirc * kv * m_gains.D + m_integrator;

    // Sum PID control and centripetal acceleration for lateral manoeuvre demand
    float latAccDemCirc = loiterDirection * (latAccDemPID + latAccDemCircCtrl);

    // Switch between a capture and loiter
    if (xtrackErrCirc > 0.0f && loiterDirection * latAccDemCap < loiterDirection * latAccDemCirc)
    {
        m_latAccDem = latAccDemCap;
    } else {
        m_latAccDem = latAccDemCirc;
    }

    m_ctrlInfo.actualPositionX     = m_aircraftStates.x;
    m_ctrlInfo.targetPositionX     = centerWP.x();
    m_ctrlInfo.actualPositionY     = m_aircraftStates.y;
    m_ctrlInfo.targetPositionY     = centerWP.y();
    m_ctrlInfo.radius              = m_integrator; //radius;
    m_ctrlInfo.kx                  = xtrackErrCirc * kx * m_gains.P; //kx;
    m_ctrlInfo.xtrackPositionError = xtrackErrCirc;
    m_ctrlInfo.kv                  = xtrackVelCirc * kv * m_gains.D; //kv;
    m_ctrlInfo.xtrackVelocityError = xtrackVelCirc;
    m_ctrlInfo.accelerationTarget  = m_latAccDem;
}

float LateralController::navRollCmd() const {
    float cmd = std::atan(m_latAccDem / static_cast<float>(m_physics.gravity));
    cmd = constrainFloat(cmd, -m_gains.rollSaturation, m_gains.rollSaturation);
    return cmd;
}
