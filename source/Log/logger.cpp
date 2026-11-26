/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "logger.h"

#include <iomanip>
#include <sstream>
#include <utility>

namespace {

using Column = std::pair<const char*, const char*>; // name, unit

// Line 1: names, line 2: units. MATLAB reads both with
// detectImportOptions(f, 'VariableNamesLine', 1, 'VariableUnitsLine', 2, 'DataLines', 3).
void writeHeader(std::ofstream& file, const std::vector<Column>& columns) {
    for (size_t i = 0; i < columns.size(); ++i) file << (i ? "," : "") << columns[i].first;
    file << '\n';
    for (size_t i = 0; i < columns.size(); ++i) file << (i ? "," : "") << columns[i].second;
    file << '\n';
}

// Double-quoted CSV field (the text may hold commas).
std::string csvQuoted(const std::string& text) {
    std::string out = "\"";
    for (const char c : text) {
        if (c == '"') out += '"';
        out += c;
    }
    return out + "\"";
}

// Single-quoted YAML scalar.
std::string yamlQuoted(const std::string& text) {
    std::string out = "'";
    for (const char c : text) {
        if (c == '\'') out += '\'';
        out += c;
    }
    return out + "'";
}

} // namespace

Logger& Logger::instance() {
    static Logger inst;
    return inst;
}

Logger::~Logger() {
    stop();
}


void Logger::start(const bool enabled, const std::string& logDirectory, const LogRunInfo& info) {
    m_enabled = enabled;
    if (m_running || !m_enabled) return;

    std::filesystem::create_directories(logDirectory);
    m_directory = logDirectory;
    m_startWallTimeMs = nowWallTimeMs();

    m_files[LogType::Aircraft].open(logDirectory + "/aircraft.csv");
    m_files[LogType::Payload].open(logDirectory + "/payload.csv");
    m_files[LogType::Tether].open(logDirectory + "/tether.csv");
    m_files[LogType::SitlOut].open(logDirectory + "/sitlOut.csv");
    m_files[LogType::SitlIn].open(logDirectory + "/sitlIn.csv");
    m_files[LogType::Frame].open(logDirectory + "/frame.csv");
    m_files[LogType::Event].open(logDirectory + "/events.csv");

    m_writeHeaders();
    m_writeManifest(info);
    m_copyConfigFiles(info);

    m_running = true;
    m_writerThread = std::thread(&Logger::m_writerLoop, this);
}

void Logger::stop() {
    if (!m_running) return;

    m_running = false;
    m_queue.stop();

    if (m_writerThread.joinable()) {
        m_writerThread.join();
    }

    m_queue.clear();

    for (auto& [_, file] : m_files) {
        if (file.is_open()) {
            file.close();
        }
    }
}

void Logger::log(const LogType type, const std::string& line) {
    if (!m_running || !m_enabled) return;
    m_queue.push(LogItem{.type = type, .line = line});
}

void Logger::logEvent(const double simTime, const int id, const std::string& event, const std::string& detail) {
    std::ostringstream msg;
    msg << std::fixed << std::setprecision(4)
        << simTime << "," << nowWallTimeMs() << "," << id << "," << csvQuoted(event) << "," << csvQuoted(detail);
    log(LogType::Event, msg.str());
}

uint64_t Logger::nowWallTimeMs() {
    const auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

void Logger::m_writeHeaders() {
    writeHeader(m_files[LogType::Aircraft], {
        {"simTimeSec", "s"}, {"wallTimeMs", "ms"}, {"id", "-"},
        {"x", "m"}, {"y", "m"}, {"z", "m"},
        {"vx", "m/s"}, {"vy", "m/s"}, {"vz", "m/s"},
        {"u", "m/s"}, {"v", "m/s"}, {"w", "m/s"},
        {"phi", "rad"}, {"theta", "rad"}, {"psi", "rad"},
        {"p", "rad/s"}, {"q", "rad/s"}, {"r", "rad/s"},
        {"wx", "m/s"}, {"wy", "m/s"}, {"wz", "m/s"},
        {"wu", "m/s"}, {"wv", "m/s"}, {"ww", "m/s"},
        {"a_n", "m/s^2"}, {"a_e", "m/s^2"}, {"a_d", "m/s^2"},
        {"f_x", "m/s^2"}, {"f_y", "m/s^2"}, {"f_z", "m/s^2"}});

    writeHeader(m_files[LogType::Payload], {
        {"simTimeSec", "s"}, {"wallTimeMs", "ms"}, {"name", "-"},
        {"x", "m"}, {"y", "m"}, {"z", "m"},
        {"vx", "m/s"}, {"vy", "m/s"}, {"vz", "m/s"},
        {"wx", "m/s"}, {"wy", "m/s"}, {"wz", "m/s"},
        {"a_n", "m/s^2"}, {"a_e", "m/s^2"}, {"a_d", "m/s^2"},
        {"f_x", "m/s^2"}, {"f_y", "m/s^2"}, {"f_z", "m/s^2"}});

    writeHeader(m_files[LogType::Tether], {
        {"simTimeSec", "s"}, {"wallTimeMs", "ms"}, {"name", "-"}, {"delta_x", "m"},
        {"p_start_x", "m"}, {"p_start_y", "m"}, {"p_start_z", "m"},
        {"v_start_x", "m/s"}, {"v_start_y", "m/s"}, {"v_start_z", "m/s"},
        {"p_end_x", "m"}, {"p_end_y", "m"}, {"p_end_z", "m"},
        {"v_end_x", "m/s"}, {"v_end_y", "m/s"}, {"v_end_z", "m/s"},
        {"fs_x", "N"}, {"fs_y", "N"}, {"fs_z", "N"},
        {"fa_x", "N"}, {"fa_y", "N"}, {"fa_z", "N"},
        {"length", "m"}});

    // First column: the timestamp sent to ArduPilot in the JSON frame (Unix
    // time), which drives the autopilot clock. Not the physics time.
    writeHeader(m_files[LogType::SitlOut], {
        {"jsonTimestampSec", "s"}, {"wallTimeMs", "ms"}, {"id", "-"},
        {"gyro_x", "rad/s"}, {"gyro_y", "rad/s"}, {"gyro_z", "rad/s"},
        {"f_x", "m/s^2"}, {"f_y", "m/s^2"}, {"f_z", "m/s^2"},
        {"x", "m"}, {"y", "m"}, {"z", "m"},
        {"phi", "rad"}, {"theta", "rad"}, {"psi", "rad"},
        {"vx", "m/s"}, {"vy", "m/s"}, {"vz", "m/s"},
        {"airspeed", "m/s"}, {"rpm", "rpm"}});

    writeHeader(m_files[LogType::SitlIn], {
        {"simTimeSec", "s"}, {"wallTimeMs", "ms"}, {"id", "-"},
        {"pwm_0", "us"}, {"pwm_1", "us"}, {"pwm_2", "us"}, {"pwm_3", "us"},
        {"aileron_deg", "deg"}, {"elevator_deg", "deg"}, {"throttle", "rpm"}, {"rudder_deg", "deg"}});

    writeHeader(m_files[LogType::Frame], {
        {"simTimeSec", "s"}, {"wallTimeMs", "ms"}, {"name", "-"}, {"node_id", "-"},
        {"x", "m"}, {"y", "m"}, {"z", "m"},
        {"vx", "m/s"}, {"vy", "m/s"}, {"vz", "m/s"},
        {"ax", "m/s^2"}, {"ay", "m/s^2"}, {"az", "m/s^2"},
        {"fx", "N"}, {"fy", "N"}, {"fz", "N"},
        {"mx", "N*m"}, {"my", "N*m"}, {"mz", "N*m"}});

    writeHeader(m_files[LogType::Event], {
        {"simTimeSec", "s"}, {"wallTimeMs", "ms"}, {"id", "-"}, {"event", "-"}, {"detail", "-"}});
}

void Logger::m_writeManifest(const LogRunInfo& info) const {
    std::ofstream manifest(m_directory + "/manifest.yaml");

    const std::time_t startSec = static_cast<std::time_t>(m_startWallTimeMs / 1000);
    char localTime[40];
    std::strftime(localTime, sizeof(localTime), "%Y-%m-%dT%H:%M:%S%z", std::localtime(&startSec));

    manifest << "program: GRS_Simulator\n"
             << "logSchema: " << kLogSchemaVersion << "\n"
             << "startWallTimeMs: " << m_startWallTimeMs << "\n"
             << "startLocalTime: " << yamlQuoted(localTime) << "\n"
             << "runDirectory: " << yamlQuoted(info.runDirectory) << "\n"
             << "commandLine: " << yamlQuoted(info.commandLine) << "\n"
             << "configFile: " << yamlQuoted(info.configFile) << "\n"
             << "clocks:\n"
             << "  simTimeSec: physics time, advances only while the autopilots answer\n"
             << "  wallTimeMs: Unix time (system clock) when the row was formed\n"
             << "  jsonTimestampSec: Unix time sent to ArduPilot in the JSON frame (sitlOut.csv)\n"
             << "frames:\n"
             << "  earth: NED, simulation origin (x, y, z, vx..vz, a_n..a_d, wx..wz)\n"
             << "  body: aircraft body axes (u, v, w, p, q, r, wu..ww, f_x..f_z)\n";
}

void Logger::m_copyConfigFiles(const LogRunInfo& info) const {
    namespace fs = std::filesystem;
    std::error_code ec;

    if (!info.configFile.empty()) {
        fs::copy_file(info.configFile, m_directory + "/config.yaml", fs::copy_options::overwrite_existing, ec);
    }

    fs::create_directories(m_directory + "/config", ec);
    for (const auto& file : info.propertyFiles) {
        if (file.empty() || !fs::exists(file)) continue;
        fs::copy_file(file, fs::path(m_directory) / "config" / fs::path(file).filename(), fs::copy_options::overwrite_existing, ec);
    }
}

void Logger::m_writerLoop() {
    while (m_running || !m_queue.empty()) {
        auto item = m_queue.pop();
        if (!item.has_value()) {
            break;
        }

        auto it = m_files.find(item->type);
        if (it != m_files.end() && it->second.is_open()) {
            it->second << item->line << '\n';
        }
    }
}
