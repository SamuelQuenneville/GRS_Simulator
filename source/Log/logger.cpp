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
}

void Logger::loggerClockTick(double dt) {
    m_simulationTime += dt;
}

double Logger::getLoggerTime() {
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
