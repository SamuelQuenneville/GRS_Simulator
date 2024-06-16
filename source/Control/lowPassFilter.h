/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_LOWPASSFILTER_H
#define GRS_SIMULATOR_LOWPASSFILTER_H


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

//
/// @file	LowPassFilter.h
/// @brief	A class to implement a low pass filter without losing precision even for int types
///         the downside being that it's a little slower as it internally uses a float
///         and it consumes an extra 4 bytes of memory to hold the constant gain

/*
  Note that this filter can be used in 2 ways:

   1) providing dt on every sample, and calling apply like this:

      // call once
      filter.set_cutoff_frequency(frequency_hz);

      // then on each sample
      output = filter.apply(sample, dt);

   2) providing a sample freq and cutoff_freq once at start

      // call once
      filter.set_cutoff_frequency(sample_freq, frequency_hz);

      // then on each sample
      output = filter.apply(sample);

  The second approach is more CPU efficient as it doesn't have to
  recalculate alpha each time, but it assumes that dt is constant
 */

#pragma once

#include <Control/filter.h>
#include "Mathematics/grsMath.h"
#include "Mathematics/vector2.h"
#include "Mathematics/vector3.h"

// DigitalLPF implements the filter math
template <class T>
class DigitalLPF {
public:
    DigitalLPF();
    // add a new raw value to the filter, retrieve the filtered result
    T apply(const T& sample, float cutOffFrequency, float dt);
    T apply(const T& sample);

    void computeAlpha(float sampleFrequency, float cutOffFrequency);

    // get latest filtered value from filter (equal to the value returned by latest call to apply method)
    const T& get() const;
    void reset(T value);
    void reset() {
        m_initialised = false;
    }

private:
    T m_output;
    float m_alpha = 1.0f;
    bool m_initialised;
};

// LPF base class
template <class T>
class LowPassFilter {
public:
    LowPassFilter();
    LowPassFilter(float cutOffFrequency);
    LowPassFilter(float sampleFrequency, float cutOffFrequency);

    // change parameters
    void setCutOffFrequency(float cutOffFrequency);
    void setCutOffFrequency(float sampleFrequency, float cutOffFrequency);

    // return the cutoff frequency
    float get_cutoff_freq(void) const;
    T apply(T sample, float dt);
    T apply(T sample);
    const T& get() const;
    void reset(T value);
    void reset(void) { m_filter.reset(); }

protected:
    float m_cutOffFrequency;

private:
    DigitalLPF<T> m_filter;
};

// typedefs for compatibility
typedef LowPassFilter<int>      LowPassFilterInt;
typedef LowPassFilter<long>     LowPassFilterLong;
typedef LowPassFilter<float>    LowPassFilterFloat;
typedef LowPassFilter<Vector2<float>> LowPassFilterVector2f;
typedef LowPassFilter<Vector3<float>> LowPassFilterVector3f;


#endif //GRS_SIMULATOR_LOWPASSFILTER_H
