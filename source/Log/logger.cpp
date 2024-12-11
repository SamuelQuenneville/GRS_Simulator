/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "logger.h"

Logger::Logger(const std::string& logDirectory)
    : m_logDirectory(logDirectory)
{
    m_initDirectory(logDirectory);
    m_simulationTime = 0.0;
}

Logger::~Logger() {
    stopLog(m_aircraftLog);
    stopLog(m_tethersLog);
    stopLog(m_payloadLog);
    stopLog(m_hlcLog);
    stopLog(m_llcLog);
}

void Logger::loggerClockTick(double dt) {
    m_simulationTime += dt;
}

double Logger::getLoggerTime() const {
    return m_simulationTime;
}

void Logger::m_initDirectory(const std::string& logDirectory) {
    std::filesystem::create_directories(logDirectory);

    time_t rawTime;
    struct tm* timeInfo;
    char buffer[80];

    time(&rawTime);
    timeInfo = localtime(&rawTime);

    strftime(buffer,sizeof(buffer),"/%Y-%m-%d_%H-%M-%S",timeInfo);
    std::string date(buffer);

    m_aircraftLog = startLog(logDirectory + date + "_aircraft.csv");
    m_tethersLog = startLog(logDirectory + date + "_tethers.csv");
    m_payloadLog = startLog(logDirectory + date + "_payload.csv");
    m_hlcLog = startLog(logDirectory + date + "_hlc.csv");
    m_llcLog = startLog(logDirectory + date + "_llc.csv");
}

std::ofstream Logger::startLog(const std::string& fileName) {
    std::ofstream logFile;
    logFile.open(fileName);

    return logFile;
}

void Logger::stopLog(std::ofstream& logHandle) {
    logHandle.close();
}

void Logger::writeAircraftMessage(fixedWingLogMessage& message) {

    m_aircraftLog << message.uavId            << ","
                  << getLoggerTime()          << ","
                  << message.aileronCommand   << ","
                  << message.elevatorCommand  << ","
                  << message.rudderCommand    << ","
                  << message.thrustCommand    << ","
                  << message.nedPosition      << ","
                  << message.earthSpeed       << ","
                  << message.bodySpeed        << ","
                  << message.attitude         << ","
                  << message.angularSpeed     << ","
                  << message.bodyForcesAero   << ","
                  << message.bodyMomentsAero  << ","
                  << message.bodyForcesTether << std::endl;
}

void Logger::writeTetherMessage(tetherLogMessage& message) {

    m_tethersLog << message.tetherId            << ","
                 << getLoggerTime()             << ","
                 << message.elongation          << ","
                 << message.startPointPosition  << ","
                 << message.startPointSpeed     << ","
                 << message.endPointPosition    << ","
                 << message.endPointSpeed       << ","
                 << message.earthForcesSpring   << ","
                 << message.earthForcesAero     << std::endl;
}

void Logger::writeHlcMessage(hlcLogMessage& message) {

    m_hlcLog << "1"                               << ","
             << getLoggerTime()                   << ","
             << message.commands1.altitude        << ","
             << message.commands1.phi             << ","
             << message.commands1.theta           << ","
             << message.commands1.thrust          << ","
             << message.groundspeedPid1.target    << ","
             << message.groundspeedPid1.actual    << ","
             << message.groundspeedPid1.error     << ","
             << message.groundspeedPid1.P         << ","
             << message.groundspeedPid1.I         << ","
             << message.groundspeedPid1.D         << ","
             << message.groundspeedPid1.FF        << ","
             << message.groundspeedPid1.Dmod      << ","
             << message.groundspeedPid1.slew_rate << ","
             << message.groundspeedPid1.limit     << ","
             << message.lateralController1.actualPositionX       << ","
             << message.lateralController1.targetPositionX       << ","
             << message.lateralController1.actualPositionY       << ","
             << message.lateralController1.targetPositionY       << ","
             << message.lateralController1.radius                << ","
             << message.lateralController1.kx                    << ","
             << message.lateralController1.xtrackPositionError   << ","
             << message.lateralController1.kv                    << ","
             << message.lateralController1.xtrackVelocityError   << ","
             << message.lateralController1.accelerationTarget    << ","
             << message.phaseController1.phase                   << ","
             << message.phaseController1.phaseTarget             << ","
             << message.phaseController1.P                       << ","
             << message.phaseController1.I                       << ","
             << message.phaseController1.D                       << ","
             << message.phaseController1.thrustOut               << std::endl;

    m_hlcLog << "2"                               << ","
             << getLoggerTime()                   << ","
             << message.commands2.altitude        << ","
             << message.commands2.phi             << ","
             << message.commands2.theta           << ","
             << message.commands2.thrust          << ","
             << message.groundspeedPid2.target    << ","
             << message.groundspeedPid2.actual    << ","
             << message.groundspeedPid2.error     << ","
             << message.groundspeedPid2.P         << ","
             << message.groundspeedPid2.I         << ","
             << message.groundspeedPid2.D         << ","
             << message.groundspeedPid2.FF        << ","
             << message.groundspeedPid2.Dmod      << ","
             << message.groundspeedPid2.slew_rate << ","
             << message.groundspeedPid2.limit     << ","
             << message.lateralController2.actualPositionX       << ","
             << message.lateralController2.targetPositionX       << ","
             << message.lateralController2.actualPositionY       << ","
             << message.lateralController2.targetPositionY       << ","
             << message.lateralController2.radius                << ","
             << message.lateralController2.kx                    << ","
             << message.lateralController2.xtrackPositionError   << ","
             << message.lateralController2.kv                    << ","
             << message.lateralController2.xtrackVelocityError   << ","
             << message.lateralController2.accelerationTarget    << ","
             << message.phaseController2.phase                   << ","
             << message.phaseController2.phaseTarget             << ","
             << message.phaseController2.P                       << ","
             << message.phaseController2.I                       << ","
             << message.phaseController2.D                       << ","
             << message.phaseController2.thrustOut               << std::endl;

}

void Logger::writeLlcMessage(llcLogMessage& message) {

    m_llcLog << message.aircraftId            << ","
             << getLoggerTime()               << ","
             << message.inputs.deltaAileron   << ","
             << message.inputs.deltaElevator  << ","
             << message.inputs.deltaRudder    << ","
             << message.inputs.deltaThrust    << ","
             << message.pitchPid.target       << ","
             << message.pitchPid.actual       << ","
             << message.pitchPid.error        << ","
             << message.pitchPid.P            << ","
             << message.pitchPid.I            << ","
             << message.pitchPid.D            << ","
             << message.pitchPid.FF           << ","
             << message.pitchPid.Dmod         << ","
             << message.pitchPid.slew_rate    << ","
             << message.pitchPid.limit        << ","
             << message.rollPid.target        << ","
             << message.rollPid.actual        << ","
             << message.rollPid.error         << ","
             << message.rollPid.P             << ","
             << message.rollPid.I             << ","
             << message.rollPid.D             << ","
             << message.rollPid.FF            << ","
             << message.rollPid.Dmod          << ","
             << message.rollPid.slew_rate     << ","
             << message.rollPid.limit         << ","
             << message.altitudePid.target    << ","
             << message.altitudePid.actual    << ","
             << message.altitudePid.error     << ","
             << message.altitudePid.P         << ","
             << message.altitudePid.I         << ","
             << message.altitudePid.D         << ","
             << message.altitudePid.FF        << ","
             << message.altitudePid.Dmod      << ","
             << message.altitudePid.slew_rate << ","
             << message.altitudePid.limit     << std::endl;
}

void Logger::writePayloadMessage(payloadLogMessage& message) {
    m_payloadLog << getLoggerTime()     << ","
                 << message.nedPosition     << ","
                 << message.earthSpeed      << ",";

    for (const auto& force : message.earthForcesTethers) {
        m_payloadLog << force.first  << ","
                     << force.second << ",";
    }

    m_payloadLog << std::endl;

}
