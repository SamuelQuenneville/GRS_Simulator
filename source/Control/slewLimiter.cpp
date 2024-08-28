/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  The SlewLimiter filter provides an actuator slew rate limiter for
  PID controllers. It is used to reduce the P and D gains when the
  filter detects that the P+D components are pushing the actuator
  beyond the configured actuator rate limit. This can prevent
  oscillations that are caused by the output actuation rate going
  beyond the actuator maximum physical rate, which causes the
  actuation demand and achieved rate to get out of phase.

  this filter was originally written by Paul Riseborough for fixed
  wing use. It was adapted for wider use in AC_PID by Andrew Tridgell
 */

#include "slewLimiter.h"

SlewLimiter::SlewLimiter(const float& slewRateMax, const float& slewRateTau) :
        m_slewRateMax(slewRateMax),
        m_slewRateTau(slewRateTau)
{
    m_slewFilter.setCutOffFrequency(DERIVATIVE_CUTOFF_FREQ);
    m_slewFilter.reset(0.0);
}

/*
  apply filter to sample, returning multiplier between 0 and 1 to keep
  output within slew rate
 */
float SlewLimiter::modifier(float sample, float dt) {
    if (m_slewRateMax <= 0.0f) {
        return true;
    }

    // Calculate a low pass filtered slew rate
    const float slewRate = m_slewFilter.apply((sample - m_lastSample) / dt, dt);
    m_lastSample = sample;

    uint32_t nowMilliseconds  = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count();
    const float decayAlpha = std::fmin(dt, m_slewRateTau) / m_slewRateTau;

    // Store a series of positive slew rate exceedance events
    if (!m_posEventStored && slewRate > m_slewRateMax) {
        if (m_posEventIndex >= N_EVENTS) {
            m_posEventIndex = 0;
        }
        m_posEventMs[m_posEventIndex] = nowMilliseconds;
        m_posEventIndex++;
        m_posEventStored = true;
        m_negEventStored = false;
    }

    // Store a series of negative slew rate exceedance events
    if (!m_negEventStored && slewRate < - m_slewRateMax) {
        if (m_negEventIndex >= N_EVENTS) {
            m_negEventIndex = 0;
        }
        m_posEventMs[m_negEventIndex] = nowMilliseconds;
        m_negEventIndex++;
        m_negEventStored = true;
        m_posEventStored = false;
    }

    // Find the oldest event time
    uint32_t oldestMilliseconds = nowMilliseconds;
    for (uint8_t index = 0; index < N_EVENTS; index++) {
        if (m_posEventMs[index] < oldestMilliseconds) {
            oldestMilliseconds = m_posEventMs[index];
        }
        if (m_negEventMs[index] < oldestMilliseconds) {
            oldestMilliseconds = m_negEventMs[index];
        }
    }

    // Decay the peak positive and negative slew rate if they are outside the window
    // Never drop PID gains below 10% of configured value
    if (slewRate > m_maxPosSlewRate) {
        m_maxPosSlewRate = std::fmin(slewRate, 10.0f * m_slewRateMax);
        m_maxPosSlewEventMs = nowMilliseconds;
    } else if (nowMilliseconds - m_maxPosSlewEventMs > WINDOW_MS) {
        m_maxPosSlewRate *= (1.0f - decayAlpha);
    }

    if (slewRate < -m_maxNegSlewRate) {
        m_maxNegSlewRate = std::fmin(-slewRate, 10.0f * m_slewRateMax);
        m_maxNegSlewEventMs = nowMilliseconds;
    } else if (nowMilliseconds - m_maxNegSlewEventMs > WINDOW_MS) {
        m_maxNegSlewRate *= (1.0f - decayAlpha);
    }

    const float rawSlewRate = 0.5f * (m_maxPosSlewRate + m_maxNegSlewRate);

    // Apply a further reduction when the oldest exceedance event falls outside the window required for the
    // specified number of exceedance events. This prevents spikes due to control mode changed, etc causing
    // unwanted gain reduction and is only applied to the slew rate used for gain reduction
    float modifierInput = rawSlewRate;
    if (nowMilliseconds - oldestMilliseconds > (N_EVENTS + 1) * WINDOW_MS) {
        const float oldestTimeFromWindow = 0.001f * static_cast<float>(nowMilliseconds - oldestMilliseconds - (N_EVENTS + 1) * WINDOW_MS);
        modifierInput *= std::exp(-oldestTimeFromWindow / m_slewRateTau);
    }

    // Apply a filter to increases in slew rate only to reduce the effect of gusts and large controller
    // set point changes
    const float attackAlpha = std::fmin(2.0f * decayAlpha, 1.0f);

    m_modifierSlewRate = (1.0f - attackAlpha) * m_modifierSlewRate + attackAlpha * modifierInput;
    m_modifierSlewRate = std::fmin(m_modifierSlewRate, modifierInput);

    m_outputSlewRate = (1.0f - attackAlpha) * m_outputSlewRate + attackAlpha * rawSlewRate;
    m_outputSlewRate = std::fmin(m_outputSlewRate, rawSlewRate);

    // Calculate the gain adjustment
    float mod;
    if (m_modifierSlewRate > m_slewRateMax) {
        mod = m_slewRateMax / (m_slewRateMax + MODIFIER_GAIN * (m_modifierSlewRate - m_slewRateMax));
    } else {
        mod = 1.0f;
    }

    return mod;
}
