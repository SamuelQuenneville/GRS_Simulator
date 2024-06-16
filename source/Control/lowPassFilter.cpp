/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "lowPassFilter.h"


//
/// @file LowPassFilter.cpp
/// @brief  A class to implement a low pass filter without losing precision even for int types
///         the downside being that it's a little slower as it internally uses a float
///         and it consumes an extra 4 bytes of memory to hold the constant gain


////////////////////////////////////////////////////////////////////////////////////////////
// DigitalLPF
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
DigitalLPF<T>::DigitalLPF() {
    // built in initialization
    m_output = T();
}

// add a new raw value to the filter, retrieve the filtered result
template <class T>
T DigitalLPF<T>::apply(const T& sample, float cutOffFrequency, float dt) {
    if (cutOffFrequency <= 0.0f || dt <= 0.0f) {
        m_output = sample;
        return m_output;
    }
    float rc = 1.0f/(M_2PI * cutOffFrequency);
    m_alpha = constrainFloat(dt/(dt+rc), 0.0f, 1.0f);
    m_output += (sample - m_output) * m_alpha;
    if (!m_initialised) {
        m_initialised = true;
        m_output = sample;
    }
    return m_output;
}

template <class T>
T DigitalLPF<T>::apply(const T& sample) {
    m_output += (sample - m_output) * m_alpha;
    if (!m_initialised) {
        m_initialised = true;
        m_output = sample;
    }
    return m_output;
}

template <class T>
void DigitalLPF<T>::computeAlpha(float sampleFrequency, float cutOffFrequency) {
    if (sampleFrequency <= 0) {
        m_alpha = 1;
    } else {
        m_alpha = calcLowpassAlphaDt(1.0/sampleFrequency, cutOffFrequency);
    }
}

// get latest filtered value from filter (equal to the value returned by latest call to apply method)
template <class T>
const T& DigitalLPF<T>::get() const {
    return m_output;
}

template <class T>
void DigitalLPF<T>::reset(T value) {
    m_output = value;
    m_initialised = true;
}

////////////////////////////////////////////////////////////////////////////////////////////
// LowPassFilter
////////////////////////////////////////////////////////////////////////////////////////////

// constructors
template <class T>
LowPassFilter<T>::LowPassFilter() :
        m_cutOffFrequency(0.0f) {}

template <class T>
LowPassFilter<T>::LowPassFilter(float cutOffFrequency) :
        m_cutOffFrequency(cutOffFrequency) {}

template <class T>
LowPassFilter<T>::LowPassFilter(float sampleFrequency, float cutOffFrequency)
{
    setCutOffFrequency(sampleFrequency, cutOffFrequency);
}

// change parameters
template <class T>
void LowPassFilter<T>::setCutOffFrequency(float cutOffFrequency) {
    m_cutOffFrequency = cutOffFrequency;
}

template <class T>
void LowPassFilter<T>::setCutOffFrequency(float sampleFrequency, float cutOffFrequency) {
    m_cutOffFrequency = cutOffFrequency;
    m_filter.computeAlpha(sampleFrequency, cutOffFrequency);
}

// return the cutoff frequency
template <class T>
float LowPassFilter<T>::get_cutoff_freq(void) const {
    return m_cutOffFrequency;
}

template <class T>
T LowPassFilter<T>::apply(T sample, float dt) {
    return m_filter.apply(sample, m_cutOffFrequency, dt);
}

template <class T>
T LowPassFilter<T>::apply(T sample) {
    return m_filter.apply(sample);
}

template <class T>
const T& LowPassFilter<T>::get() const {
    return m_filter.get();
}

template <class T>
void LowPassFilter<T>::reset(T value) {
    m_filter.reset(value);
}

/*
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class LowPassFilter<int>;
template class LowPassFilter<long>;
template class LowPassFilter<float>;
template class LowPassFilter<Vector2<float>>;
template class LowPassFilter<Vector3<float>>;

