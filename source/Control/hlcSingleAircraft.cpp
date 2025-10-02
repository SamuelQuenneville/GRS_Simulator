/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "hlcSingleAircraft.h"

HlcSingleAircraft::HlcSingleAircraft(const worldPhysics& physics, Logger& logger, hlcGains& gains, const float rate)
        : m_physics(physics)
        , m_logger(logger)
        , m_gains(gains)
        , m_rate(rate)
{
    m_groundspeedController = std::make_unique<GroundspeedController>(m_physics, m_gains.groundspeedCtrlGains, m_rate);
}

// currentStates should be of dimension 1
std::map<int, fixedWingCommands> HlcSingleAircraft::calculateInputs(const std::vector<int>& ids, const std::vector<fixedWingStates>& currentStates, const target& target) {
    m_groundspeedController->setAircraftStates(currentStates[0]);

    const float requiredThrust = m_groundspeedController->getThrustOut(target.groundspeed, false);

    fixedWingCommands commandsInputs{};
    commandsInputs.thrust = requiredThrust;
    commandsInputs.theta = target.aircraftPitch;
    commandsInputs.phi = target.aircraftRoll;
    commandsInputs.altitude = target.positionPayload[2];

    m_writeLog(commandsInputs);
    return std::map<int, fixedWingCommands> {{ids[0], commandsInputs}};
}

void HlcSingleAircraft::m_writeLog(const fixedWingCommands& commands) const {
    hlcLogMessage logMessage;

    logMessage = {
            commands,
            m_groundspeedController->getPidInfo()
    };

    m_logger.writeHlcMessage(logMessage);
}

std::map<int, fixedWingCommands> HlcSingleAircraft::calculateInputs(const std::vector<int> &ids, const std::vector<fixedWingStates> &currentStates, const payloadStates &payloadStates, const target &target) {
    return {};
}
