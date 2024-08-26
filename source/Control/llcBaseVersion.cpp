/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "llcBaseVersion.h"

LlcBaseVersion::LlcBaseVersion(worldPhysics& physics, Logger& logger,llcGains& gains, float rate)
    : m_physics(physics)
    , m_logger(logger)
    , m_gains(gains)
    , m_rate(rate)
{
    m_rollController = std::make_unique<RollController>(m_physics, m_gains.rollCtrlGains,m_rate);
    m_pitchController = std::make_unique<PitchController>(m_physics, m_gains.pitchCtrlGains, m_rate);
    m_altController = std::make_unique<AltitudeController>(m_physics, m_gains.altitudeCtrlGains, m_rate);
}

fixedWingControlInputs LlcBaseVersion::calculateAircraftControlInputs(int aircraftId, const fixedWingStates& currentStates, const fixedWingCommands& commands) {

    m_rollController->setAircraftStates(currentStates);
    m_pitchController->setAircraftStates(currentStates);
    m_altController->setAircraftStates(currentStates);

    double altPitch = m_altController->getPitchOut(static_cast<float>(commands.altitude), false);

    fixedWingControlInputs controlInputs{};
    controlInputs.deltaAileron = m_stabilizeRoll(commands.phi);
    controlInputs.deltaElevator = -1.0 * m_stabilizePitch(altPitch + commands.theta); // PixHawk define positive del_e up, we use Cm_de negative so del_e must be positive down
    controlInputs.deltaRudder = 0.0;//_stabilizeYaw(0.0), Always 0
    controlInputs.deltaThrust = commands.thrust;

    m_writeLog(aircraftId, controlInputs);
    return controlInputs;
}

void LlcBaseVersion::m_calcNavigationRoll(double commandedRoll) {
    m_navigationRoll = static_cast<float>(commandedRoll);
}

void LlcBaseVersion::m_calcNavigationPitch(double commandedPitch) {
    m_navigationPitch = static_cast<float>(commandedPitch);
}

void LlcBaseVersion::m_calcNavigationYaw(double commandedYaw) {
    m_navigationYaw = static_cast<float>(commandedYaw);
}

float LlcBaseVersion::m_stabilizeRoll(double commandedRoll) {
    m_calcNavigationRoll(commandedRoll);

    float rollOut = m_rollController->getServoOut(m_navigationRoll, 1.0f, false);
    return rollOut;
}

float LlcBaseVersion::m_stabilizePitch(double commandedPitch) {
    m_calcNavigationPitch(commandedPitch);

    float pitchOut = m_pitchController->getServoOut(m_navigationPitch, 1.0f, false);
    return pitchOut;
}

int32_t LlcBaseVersion::m_stabilizeYaw(double commandedYaw) {
    m_calcNavigationYaw(commandedYaw);

    auto yawOut = static_cast<int32_t>(m_navigationYaw);
    return yawOut;
}

int32_t LlcBaseVersion::m_stabilizeRpm(double commandedRpm) {
    return true;
}

void LlcBaseVersion::m_writeLog(int aircraftId, fixedWingControlInputs& inputs) {
    llcLogMessage logMessage{};

    logMessage = {
            aircraftId,
            inputs,
            m_pitchController->getPidInfo(),
            m_rollController->getPidInfo(),
            m_altController->getPidInfo()
    };

    m_logger.writeLlcMessage(logMessage);
}
