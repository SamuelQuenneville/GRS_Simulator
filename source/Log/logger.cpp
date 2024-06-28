/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "logger.h"

Logger::Logger(std::string logDirectory)
    : m_logDirectory(logDirectory)
{
    m_initDirectory(logDirectory);
    m_simulationTime = 0;
}

Logger::~Logger() {
    stopLog(m_aircraftLog);
    stopLog(m_tethersLog);
    stopLog(m_hlcLog);
    stopLog(m_llcLog);
}

void Logger::loggerClockTick(double dt) {
    m_simulationTime += dt;
}

double Logger::getLoggerTime() const {
    return m_simulationTime;
}

void Logger::m_initDirectory(std::string logDirectory) {
    std::filesystem::create_directories(logDirectory);

    time_t rawTime;
    struct tm * timeInfo;
    char buffer[80];

    time(&rawTime);
    timeInfo = localtime(&rawTime);

    strftime(buffer,sizeof(buffer),"/%Y-%m-%d_%H-%M-%S",timeInfo);
    std::string date(buffer);

    m_aircraftLog = startLog(logDirectory + date + "_aircraft.csv");
    m_tethersLog = startLog(logDirectory + date + "_tethers.csv");
    m_hlcLog = startLog(logDirectory + date + "_hlc.csv");
    m_llcLog = startLog(logDirectory + date + "_llc.csv");
}

std::ofstream Logger::startLog(std::string fileName) {
    std::ofstream logFile;
    logFile.open(fileName);

    return logFile;
}

void Logger::stopLog(std::ofstream &logHandle) {
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

void Logger::writeHlcMessage(hlcLogMessage &message) {

    m_hlcLog << getLoggerTime()                  << ","
             << message.commands.altitude        << ","
             << message.commands.phi             << ","
             << message.commands.theta           << ","
             << message.commands.thrust          << ","
             << message.groundspeedPid.target    << ","
             << message.groundspeedPid.actual    << ","
             << message.groundspeedPid.error     << ","
             << message.groundspeedPid.P         << ","
             << message.groundspeedPid.I         << ","
             << message.groundspeedPid.D         << ","
             << message.groundspeedPid.FF        << ","
             << message.groundspeedPid.Dmod      << ","
             << message.groundspeedPid.slew_rate << ","
             << message.groundspeedPid.limit     << std::endl;
}

void Logger::writeLlcMessage(llcLogMessage &message) {

    m_llcLog << getLoggerTime()               << ","
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
