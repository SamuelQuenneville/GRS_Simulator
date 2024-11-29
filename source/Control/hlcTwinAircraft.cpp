/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "hlcTwinAircraft.h"

HlcTwinAircraft::HlcTwinAircraft(worldPhysics& physics, Logger& logger, hlcGains& gains, float rate, double tetherLength)
    : m_physics(physics)
    , m_logger(logger)
    , m_gains(gains)
    , m_rate(rate)
    , m_tetherLength(tetherLength)
{
    m_groundspeedController1 = std::make_unique<GroundspeedController>(m_physics, m_gains.groundspeedCtrlGains, m_rate);
    m_groundspeedController2 = std::make_unique<GroundspeedController>(m_physics, m_gains.groundspeedCtrlGains,m_rate);
    m_lateralController1 = std::make_unique<LateralController>(m_physics, m_gains.lateralCtrlGains, m_rate);
    m_lateralController2 = std::make_unique<LateralController>(m_physics, m_gains.lateralCtrlGains,m_rate);
    m_phaseController1 = std::make_unique<PhaseController>(m_physics, m_gains.phaseCtrlGains, m_rate);
    m_phaseController2 = std::make_unique<PhaseController>(m_physics, m_gains.phaseCtrlGains, m_rate);
}

std::map<int, fixedWingCommands> HlcTwinAircraft::calculateInputs(const std::vector<int> &ids, const std::vector<fixedWingStates> &currentStates, const target &target) {
    // Set states
    m_groundspeedController1->setAircraftStates(currentStates[0]);
    m_groundspeedController2->setAircraftStates(currentStates[1]);

    m_lateralController1->setAircraftStates(currentStates[0]);
    m_lateralController2->setAircraftStates(currentStates[1]);

    // Lateral controller --> roll
    m_lateralController1->updateLoiter({target.positionPayload[0],target.positionPayload[1]},target.radius,-1.0f, true);
    m_lateralController2->updateLoiter({target.positionPayload[0],target.positionPayload[1]},target.radius,-1.0f, true);

    // Speed & Phase --> Thrust
    float requiredThrustSpeed1 = m_groundspeedController1->getThrustOut(target.groundspeed, false);
    float requiredThrustSpeed2 = m_groundspeedController2->getThrustOut(target.groundspeed, false);

    // Altitude --> y = y_payload + sqrt(L0^2 - R^2)
    double altTarget = target.positionPayload[2] + std::sqrt(m_tetherLength*m_tetherLength - target.radius*target.radius);

    // Create commands struct
    fixedWingCommands commandsInputs1{};
    commandsInputs1.thrust   = constrainFloat(requiredThrustSpeed1, 0.0f, 60.0f);;
    commandsInputs1.theta    = target.aircraftPitch;
    commandsInputs1.phi      = degrees(m_lateralController1->navRollCmd());
    commandsInputs1.altitude = altTarget;

    fixedWingCommands commandsInputs2{};
    commandsInputs2.thrust   = constrainFloat(requiredThrustSpeed2, 0.0f, 60.0f);
    commandsInputs2.theta    = target.aircraftPitch;
    commandsInputs2.phi      = degrees(m_lateralController2->navRollCmd());
    commandsInputs2.altitude = altTarget;

    m_writeLog(commandsInputs1, commandsInputs2);
    return std::map<int, fixedWingCommands> {{ids[0], commandsInputs1}, {ids[1], commandsInputs2}};
}


std::map<int, fixedWingCommands> HlcTwinAircraft::calculateInputs(const std::vector<int>& ids, const std::vector<fixedWingStates>& currentStates, const payloadStates& payloadStates, const target& target) {
    // Set states
    m_groundspeedController1->setAircraftStates(currentStates[0]);
    m_groundspeedController2->setAircraftStates(currentStates[1]);

    m_lateralController1->setAircraftStates(currentStates[0]);
    m_lateralController2->setAircraftStates(currentStates[1]);

    m_phaseController1->setAircraftStates(currentStates[0]);
    m_phaseController2->setAircraftStates(currentStates[1]);

    // Lateral controller --> roll
    m_lateralController1->updateLoiter({target.positionPayload[0],target.positionPayload[1]},target.radius,-1.0f, true);
    m_lateralController2->updateLoiter({target.positionPayload[0],target.positionPayload[1]},target.radius,-1.0f, true);

    // Speed & Phase --> Thrust
    float requiredThrustSpeed1 = m_groundspeedController1->getThrustOut(target.groundspeed, false);
    float requiredThrustSpeed2 = m_groundspeedController2->getThrustOut(target.groundspeed, false);

    m_phaseDesired1 = static_cast<float>(std::atan2(currentStates[0].y - payloadStates.y, currentStates[0].x - payloadStates.x));
    m_phaseDesired2 = wrapPI(m_phaseDesired1 + M_PI);

    float requiredThrustPhase1 = m_phaseController1->getThrustOut(m_phaseDesired1, payloadStates);
    float requiredThrustPhase2 = m_phaseController2->getThrustOut(m_phaseDesired2, payloadStates);

    // Altitude --> y = y_payload + sqrt(L0^2 - R^2)
    double altTarget = target.positionPayload[2] + std::sqrt(m_tetherLength*m_tetherLength - target.radius*target.radius);

    // Create commands struct
    fixedWingCommands commandsInputs1{};
    commandsInputs1.thrust   = constrainFloat(requiredThrustSpeed1 + requiredThrustPhase1, 0.0f, 60.0f);;
    commandsInputs1.theta    = target.aircraftPitch;
    commandsInputs1.phi      = degrees(m_lateralController1->navRollCmd());
    commandsInputs1.altitude = altTarget;

    fixedWingCommands commandsInputs2{};
    commandsInputs2.thrust   = constrainFloat(requiredThrustSpeed2 + requiredThrustPhase2, 0.0f, 60.0f);
    commandsInputs2.theta    = target.aircraftPitch;
    commandsInputs2.phi      = degrees(m_lateralController2->navRollCmd());
    commandsInputs2.altitude = altTarget;

    m_writeLog(commandsInputs1, commandsInputs2);
    return std::map<int, fixedWingCommands> {{ids[0], commandsInputs1}, {ids[1], commandsInputs2}};
}

void HlcTwinAircraft::m_writeLog(const fixedWingCommands& commands1, const fixedWingCommands& commands2) {
    hlcLogMessage logMessage{};

    logMessage = {
            commands1,
            m_groundspeedController1->getPidInfo(),
            m_lateralController1->getCtrlInfo(),
            m_phaseController1->getCtrlInfo(),
            commands2,
            m_groundspeedController2->getPidInfo(),
            m_lateralController2->getCtrlInfo(),
            m_phaseController2->getCtrlInfo()
    };

    m_logger.writeHlcMessage(logMessage);
}
