/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

/* slew rate limiting filter. This is used to prevent oscillation of a
 * controller by modifying the controllers output based on a maximum
 * slew rate
*/

#ifndef GRS_SIMULATOR_SLEWLIMITER_H
#define GRS_SIMULATOR_SLEWLIMITER_H

#pragma once

#include <stdint.h>
#include <chrono>
#include "Control/lowPassFilter.h"

#define N_EVENTS 2 // number of positive and negative consecutive slew rate exceedance events recorded where a value of 2 corresponds to a complete cycle
#define WINDOW_MS 300 // time in msec required for a half cycle of the slowest oscillation frequency expected
#define MODIFIER_GAIN 1.5f // ratio of modifier reduction to slew rate exceedance ratio
#define DERIVATIVE_CUTOFF_FREQ 25.0f

class SlewLimiter {
public:
    SlewLimiter(const float& slewRateMax, const float& slewRateTau);

    /*
      apply filter to sample, returning multiplier between 0 and 1 to keep
      output within slew rate
    */
    float modifier(float sample, float dt);

    /*
      get last oscillation slew rate
     */
    float getSlewRate(void) const {
        return m_outputSlewRate;
    }

private:
    const float& m_slewRateMax;
    const float& m_slewRateTau;
    LowPassFilterFloat m_slewFilter;
    float m_outputSlewRate;
    float m_modifierSlewRate;
    float m_lastSample;
    float m_maxPosSlewRate;
    float m_maxNegSlewRate;
    uint32_t m_maxPosSlewEventMs;
    uint32_t m_maxNegSlewEventMs;
    uint8_t m_posEventIndex;
    uint8_t m_negEventIndex;
    uint32_t m_posEventMs[N_EVENTS];
    uint32_t m_negEventMs[N_EVENTS];
    bool m_posEventStored;
    bool m_negEventStored;
};

#endif //GRS_SIMULATOR_SLEWLIMITER_H
