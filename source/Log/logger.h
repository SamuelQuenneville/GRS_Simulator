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
#include <vector>

#include "Definitions/logDefinitions.h"

class Logger {

public:
    explicit Logger(const std::string& logDirectory);
    ~Logger();

    static std::ofstream startLog(const std::string& fileName);
    static void stopLog(std::ofstream& logHandle);

    void loggerClockTick(double dt);
    double getLoggerTime() const;

    void writeAircraftMessage(fixedWingLogMessage& message);
    void writeTetherMessage(tetherLogMessage& message);
    void writeHlcMessage(hlcLogMessage& message);
    void writeLlcMessage(llcLogMessage& message);

private:
    void m_initDirectory(const std::string& logDirectory);
    std::string m_logDirectory;

    double m_simulationTime;

    std::ofstream m_aircraftLog;
    std::ofstream m_tethersLog;
    std::ofstream m_hlcLog;
    std::ofstream m_llcLog;
};


#endif //GRS_SIMULATOR_LOGGER_H
