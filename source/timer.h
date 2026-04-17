/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TIMER_H
#define GRS_SIMULATOR_TIMER_H

#include <chrono>

class Timer {

    using clock = std::chrono::high_resolution_clock;

public:
    Timer();

    void start();
    void stop();
    void reset();

    [[nodiscard]] bool isRunning() const;

private:
    std::chrono::time_point<clock> m_startTime;
    bool m_isRunning;
};


#endif //GRS_SIMULATOR_TIMER_H
