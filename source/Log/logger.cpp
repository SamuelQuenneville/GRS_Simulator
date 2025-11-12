/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "logger.h"

Logger& Logger::instance() {
    static Logger inst;
    return inst;
}

Logger::~Logger() {
    stop();
}


void Logger::start(const bool enabled, const std::string& logDirectory) {
    m_enabled = enabled;
    if (m_running || !m_enabled) return;

    std::filesystem::create_directories(logDirectory);

    m_files[LogType::Aircraft].open(logDirectory + "/aircraft.csv");
    m_files[LogType::Payload].open(logDirectory + "/payload.csv");
    m_files[LogType::Tether].open(logDirectory + "/tether.csv");
    m_files[LogType::SitlOut].open(logDirectory + "/sitlOut.csv");
    m_files[LogType::SitlIn].open(logDirectory + "/sitlIn.csv");
    m_files[LogType::Frame].open(logDirectory + "/frame.csv");

    m_writeHeaders();

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
    m_queue.push(LogItem{type, line});
}

void Logger::m_writeHeaders() {
    m_files[LogType::Aircraft] << "time,id,x,y,z,vx,vy,vz,phi,theta,psi,p,q,r\n";
    m_files[LogType::Payload]  << "time,id,x,y,z,vx,vy,vz\n";
    m_files[LogType::Tether]   << "time,name,delta_x,p_start_x,p_start_y,p_start_z,p_end_x,p_end_y,p_end_z,fs_x,fs_y,fs_z,fa_x,fa_y,fa_z\n";
    m_files[LogType::SitlOut]  << "time,id,gyro_x,gyro_y,gyro_z,a_x,a_y,a_z,x,y,z,phi,theta,psi,vx,vy,vz,airspeed,rpm\n";
    m_files[LogType::SitlIn]   << "time,id,pwm_0,pwm_1,pwm_2,pwm_3,aileron_deg,elevator_deg,throttle,rudder_deg\n";
    m_files[LogType::Frame]    << "time,name,node_id,x,y,z,vx,vy,vz,ax,ay,az,fx,fy,fz,mx,my,mz\n";
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