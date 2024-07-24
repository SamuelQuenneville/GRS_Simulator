/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "hlcSingleAircraft.h"

HlcSingleAircraft::HlcSingleAircraft(worldPhysics& physics, Logger& logger, float rate)
        : m_physics(physics)
        , m_logger(logger)
        , m_rate(rate)
{
    m_groundspeedController = std::make_unique<GroundspeedController>(m_physics, m_rate);
}

// currentStates should be of dimension 1
fixedWingCommands HlcSingleAircraft::calculateInputs(const std::vector<fixedWingStates>& currentStates, const target& target) {
    m_groundspeedController->setAircraftStates(currentStates[0]);

    float requiredThrust = m_groundspeedController->getThrustOut(target.groundspeed, false);

    fixedWingCommands commandsInputs{};
    commandsInputs.thrust = requiredThrust;
    commandsInputs.theta = target.aircraftPitch;
    commandsInputs.phi = target.aircraftRoll;
    commandsInputs.altitude = target.positionPayload[2];

    m_writeLog(commandsInputs);
    return commandsInputs;
}

void HlcSingleAircraft::m_writeLog(const fixedWingCommands& commands) {
    hlcLogMessage logMessage{};

    logMessage = {
            commands,
            m_groundspeedController->getPidInfo()
    };

    m_logger.writeHlcMessage(logMessage);
}
