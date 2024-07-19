/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TIMER_H
#define GRS_SIMULATOR_TIMER_H

#include <stdexcept>
#include <iostream>
#include <chrono>
#include <cassert>

class Timer {

    using clock = std::chrono::high_resolution_clock;

public:
    Timer();

    [[nodiscard]] std::chrono::duration<double> getElapsedPhysicsTime() const;

    void start();
    void stop();
    void reset();
    void update();

    [[nodiscard]] bool isRunning() const;
    [[nodiscard]] bool isPossibleToTakeStep(std::chrono::duration<double> timeStep) const;

    void nextStep(std::chrono::duration<double> timeStep);

    double computeInterpolationFactor(std::chrono::duration<double> timeStep);

private:
    std::chrono::time_point<clock> m_startTime;
    std::chrono::time_point<clock> m_lastUpdateTime;

    std::chrono::duration<double> m_accumulator;

    bool m_isRunning;

    Timer(const Timer& timer);
    Timer& operator=(const Timer& timer);
};


#endif //GRS_SIMULATOR_TIMER_H
