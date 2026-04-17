/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "timer.h"

Timer::Timer()
    : m_isRunning(false)
{

}

void Timer::start() {
    if (!m_isRunning) {
        m_isRunning = true;
    }
}

void Timer::stop() {
    m_isRunning = false;
}

void Timer::reset() {
    m_startTime = clock::now();
}

bool Timer::isRunning() const {
    return m_isRunning;
}