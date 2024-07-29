/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "hlcTwinAircraft.h"

HlcTwinAircraft::HlcTwinAircraft(worldPhysics& physics, Logger& logger, float rate)
    : m_physics(physics)
    , m_logger(logger)
    , m_rate(rate)
{
    m_groundspeedController = std::make_unique<GroundspeedController>(m_physics, m_rate);
}

std::map<int, fixedWingCommands> HlcTwinAircraft::calculateInputs(const std::vector<int>& ids, const std::vector<fixedWingStates>& currentStates, const target& target) {
    m_groundspeedController->setAircraftStates(currentStates[0]);

    float requiredThrust = m_groundspeedController->getThrustOut(target.groundspeed, false);

    fixedWingCommands commandsInputs{};
    commandsInputs.thrust = requiredThrust;
    commandsInputs.theta = target.aircraftPitch;
    commandsInputs.phi = target.aircraftRoll;
    commandsInputs.altitude = target.positionPayload[2];

    m_writeLog(commandsInputs);
    return std::map<int, fixedWingCommands> {{ids[0], commandsInputs}, {ids[1], commandsInputs}};
}

void HlcTwinAircraft::m_writeLog(const fixedWingCommands& commands) {
    hlcLogMessage logMessage{};

    logMessage = {
            commands,
            m_groundspeedController->getPidInfo()
    };

    m_logger.writeHlcMessage(logMessage);
}
