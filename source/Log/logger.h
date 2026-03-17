/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_LOGGER_H
#define GRS_SIMULATOR_LOGGER_H

#include <atomic>
#include <string>
#include <filesystem>
#include <fstream>
#include <ctime>
#include <thread>
#include <unordered_map>
#include <vector>

#include "Definitions/logDefinitions.h"
#include "threadSafeQueue.h"

class Logger {
public:
    static Logger& instance();
    ~Logger();

    void start(bool enabled, const std::string& logDirectory);
    void stop();

    void log(LogType type, const std::string& line);

    static uint64_t nowWallTimeMs();

private:
    Logger() = default;

    void m_writeHeaders();
    void m_writerLoop();

    std::unordered_map<LogType, std::ofstream> m_files;
    ThreadSafeQueue<LogItem> m_queue;
    std::thread m_writerThread;
    std::atomic<bool> m_running = false;
    std::atomic<bool> m_enabled = false;
};


#endif //GRS_SIMULATOR_LOGGER_H
