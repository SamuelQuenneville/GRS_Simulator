/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_LOGGER_H
#define GRS_SIMULATOR_LOGGER_H

#include <string>
#include <filesystem>
#include <fstream>
#include <ctime>

#include "Definitions/logDefinitions.h"

class Logger {

public:
    explicit Logger(std::string logDirectory);
    ~Logger();

    static std::ofstream startLog(std::string fileName);
    static void stopLog(std::ofstream& logHandle);

    void loggerClockTick(double dt);
    double getLoggerTime() const;

    void writeAircraftMessage(fixedWingLogMessage& message);
    void writeTetherMessage(tetherLogMessage& message);

private:
    void m_initDirectory(std::string logDirectory);
    std::string m_logDirectory;

    double m_simulationTime;

    std::ofstream m_aircraftLog;
    std::ofstream m_tethersLog;
};


#endif //GRS_SIMULATOR_LOGGER_H
