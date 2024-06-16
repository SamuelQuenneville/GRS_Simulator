/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "llcBaseVersion.h"

LlcBaseVersion::LlcBaseVersion(worldPhysics& physics, float rate)
    : m_physics(physics)
    , m_rate(rate)
{
    m_rollController = std::make_unique<RollController>(m_physics, m_rate);
    m_pitchController = std::make_unique<PitchController>(m_physics, m_rate);
    m_altController = std::make_unique<AltitudeController>(m_physics, m_rate);
}

fixedWingControlInputs LlcBaseVersion::calculateAircraftControlInputs(const fixedWingStates& currentStates, const fixedWingCommands& commands, double desiredThrust) {

    m_rollController->setAircraftStates(currentStates);
    m_pitchController->setAircraftStates(currentStates);
    m_altController->setAircraftStates(currentStates);

    double altPitch = m_altController->getPitchOut(10, false)/100.0;

    fixedWingControlInputs controlInputs{};
    controlInputs.deltaAileron = m_stabilizeRoll(0) / 100.0;
    controlInputs.deltaElevator = -1.0 * m_stabilizePitch(altPitch) / 100.0; // PixHawk define positive del_e up, we use Cm_de negative so del_e must be positive down
    controlInputs.deltaRudder = 0.0;//_stabilizeYaw(0.0); // Always 0
    controlInputs.deltaThrust = desiredThrust;

    return controlInputs;
}

void LlcBaseVersion::m_calcNavigationRoll(double commandedRoll) {
    m_navigationRoll = commandedRoll;
}

void LlcBaseVersion::m_calcNavigationPitch(double commandedPitch) {
    m_navigationPitch = commandedPitch;
}

void LlcBaseVersion::m_calcNavigationYaw(double commandedYaw) {
    m_navigationYaw = commandedYaw;
}

int32_t LlcBaseVersion::m_stabilizeRoll(double commandedRoll) {
    m_calcNavigationRoll(commandedRoll);

    int32_t rollOut = m_rollController->getServoOut(m_navigationRoll, 1.0f, false);
    return rollOut;
}

int32_t LlcBaseVersion::m_stabilizePitch(double commandedPitch) {
    m_calcNavigationPitch(commandedPitch);

    int32_t pitchOut = m_pitchController->getServoOut(m_navigationPitch, 1.0f, false);
    return pitchOut;
}

int32_t LlcBaseVersion::m_stabilizeYaw(double commandedYaw) {
    m_calcNavigationYaw(commandedYaw);

    int32_t yawOut = m_navigationYaw;
    return yawOut;
}

int32_t LlcBaseVersion::m_stabilizeRpm(double commandedRpm) {
    return true;
}

