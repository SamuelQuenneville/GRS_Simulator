/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "timer.h"

Timer::Timer()
    : m_accumulator(0.0)
    , m_isRunning(false)
{

}

std::chrono::duration<double> Timer::getElapsedPhysicsTime() const {
    return m_lastUpdateTime - m_startTime;
}

void Timer::start() {
    if (!m_isRunning) {
        // Get the current system time
        m_lastUpdateTime = clock::now();

        m_accumulator = std::chrono::duration<double>::zero();
        m_isRunning = true;
    }
}

void Timer::stop() {
    m_isRunning = false;
}

void Timer::reset() {
    m_accumulator = std::chrono::milliseconds::zero();
    m_startTime = clock::now();
    m_lastUpdateTime = m_startTime;
}

void Timer::update() {
    // Get the current system time
    const std::chrono::time_point<clock> currentTime = clock::now();

    // Compute the delta display time between two display frames
    const std::chrono::duration<double> deltaTime = currentTime - m_lastUpdateTime;

    // Update the current display time
    m_lastUpdateTime = currentTime;

    // Update the accumulator value
    m_accumulator += deltaTime;
}

bool Timer::isRunning() const {
    return m_isRunning;
}

bool Timer::isPossibleToTakeStep(const std::chrono::duration<double> timeStep) const {
    return (m_accumulator >= timeStep);
}

void Timer::nextStep(const std::chrono::duration<double> timeStep) {
    m_accumulator -= timeStep;
}

double Timer::computeInterpolationFactor(const std::chrono::duration<double> timeStep) const {
    return m_accumulator.count() / timeStep.count();
}
