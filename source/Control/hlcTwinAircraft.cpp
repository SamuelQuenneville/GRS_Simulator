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
    m_groundspeedController1 = std::make_unique<GroundspeedController>(m_physics, m_rate);
    m_groundspeedController2 = std::make_unique<GroundspeedController>(m_physics, m_rate);
    m_lateralController1 = std::make_unique<LateralController>(m_physics);
    m_lateralController2 = std::make_unique<LateralController>(m_physics);
}

std::map<int, fixedWingCommands> HlcTwinAircraft::calculateInputs(const std::vector<int>& ids, const std::vector<fixedWingStates>& currentStates, const target& target) {
    m_groundspeedController1->setAircraftStates(currentStates[0]);
    m_groundspeedController2->setAircraftStates(currentStates[1]);
    m_lateralController1->setAircraftStates(currentStates[0]);
    m_lateralController2->setAircraftStates(currentStates[1]);

    float requiredThrust1 = m_groundspeedController1->getThrustOut(target.groundspeed, false);
    float requiredThrust2 = m_groundspeedController2->getThrustOut(target.groundspeed, false);

    m_lateralController1->updateLoiter({target.positionPayload[0],target.positionPayload[1]},target.radius,-1.0f, true);
    m_lateralController2->updateLoiter({target.positionPayload[0],target.positionPayload[1]},target.radius,-1.0f, true);

    fixedWingCommands commandsInputs1{};
    commandsInputs1.thrust = requiredThrust1;
    commandsInputs1.theta = target.aircraftPitch;
    commandsInputs1.phi = degrees(m_lateralController1->navRollCmd());
    commandsInputs1.altitude = target.positionPayload[2] + std::sqrt(900.0 - target.radius*target.radius); // y = y_payload + sqrt(L0^2 - R^2)

    fixedWingCommands commandsInputs2{};
    commandsInputs2.thrust = requiredThrust2;
    commandsInputs2.theta = target.aircraftPitch;
    commandsInputs2.phi = degrees(m_lateralController2->navRollCmd());
    commandsInputs2.altitude = target.positionPayload[2] + std::sqrt(900.0 - target.radius*target.radius);

    m_writeLog(commandsInputs1, commandsInputs2);
    return std::map<int, fixedWingCommands> {{ids[0], commandsInputs1}, {ids[1], commandsInputs2}};
}

void HlcTwinAircraft::m_writeLog(const fixedWingCommands& commands1, const fixedWingCommands& commands2) {
    hlcLogMessage logMessage{};

    logMessage = {
            commands1,
            m_groundspeedController1->getPidInfo(),
            m_lateralController1->getCtrlInfo(),
            commands2,
            m_groundspeedController2->getPidInfo(),
            m_lateralController2->getCtrlInfo()
    };

    m_logger.writeHlcMessage(logMessage);
}
