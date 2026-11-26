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

// What the log folder records about the run besides the CSVs (manifest.yaml
// and the copied configuration files).
struct LogRunInfo {
    std::string commandLine;                // program arguments as typed
    std::string runDirectory;               // --run-dir, empty if not given
    std::string configFile;                 // simulation YAML, copied as config.yaml
    std::vector<std::string> propertyFiles; // files it references, copied into config/
};

class Logger {
public:
    static Logger& instance();
    ~Logger();

    // Opens every CSV in logDirectory (line 1: names, line 2: units), writes
    // manifest.yaml and copies the configuration files.
    void start(bool enabled, const std::string& logDirectory, const LogRunInfo& info);
    void stop();

    void log(LogType type, const std::string& line);

    // Sparse run events (events.csv). id: aircraft id, 0 for the whole system.
    void logEvent(double simTime, int id, const std::string& event, const std::string& detail = "");

    [[nodiscard]] const std::string& directory() const { return m_directory; }

    static uint64_t nowWallTimeMs();

private:
    Logger() = default;

    void m_writeHeaders();
    void m_writeManifest(const LogRunInfo& info) const;
    void m_copyConfigFiles(const LogRunInfo& info) const;
    void m_writerLoop();

    std::unordered_map<LogType, std::ofstream> m_files;
    ThreadSafeQueue<LogItem> m_queue;
    std::thread m_writerThread;
    std::atomic<bool> m_running = false;
    std::atomic<bool> m_enabled = false;
    std::string m_directory;
    uint64_t m_startWallTimeMs = 0;
};


#endif //GRS_SIMULATOR_LOGGER_H
