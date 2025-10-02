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

    [[nodiscard]] std::chrono::duration<double> getElapsedPhysicsTime() const;

    void start();
    void stop();
    void reset();
    void update();

    [[nodiscard]] bool isRunning() const;
    [[nodiscard]] bool isPossibleToTakeStep(std::chrono::duration<double> timeStep) const;

    void nextStep(std::chrono::duration<double> timeStep);

    [[nodiscard]] double computeInterpolationFactor(std::chrono::duration<double> timeStep) const;

private:
    std::chrono::time_point<clock> m_startTime;
    std::chrono::time_point<clock> m_lastUpdateTime;

    std::chrono::duration<double> m_accumulator;

    bool m_isRunning;
};


#endif //GRS_SIMULATOR_TIMER_H
