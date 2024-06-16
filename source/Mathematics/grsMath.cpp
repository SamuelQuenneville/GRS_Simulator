/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "grsMath.h"

template <typename T>
float wrapPI(const T radian) {
    auto res = wrap2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}

template <typename T>
float wrap2PI(const T radian) {
    float res = fmod(static_cast<float>(radian), M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

template <typename T>
T constrainValueLine(const T amt, const T low, const T high, uint32_t line) {
    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

template float constrainValueLine<float>(const float amt, const float low, const float high, uint32_t line);

template <typename T>
T constrainValue(const T amt, const T low, const T high) {
    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

template int constrainValue<int>(const int amt, const int low, const int high);
template long constrainValue<long>(const long amt, const long low, const long high);
template long long constrainValue<long long>(const long long amt, const long long low, const long long high);
template short constrainValue<short>(const short amt, const short low, const short high);
template float constrainValue<float>(const float amt, const float low, const float high);
template double constrainValue<double>(const double amt, const double low, const double high);

/*
  calculate a low pass filter alpha value
 */
float calcLowpassAlphaDt(float dt, float cutOffFrequency)
{
    if (dt <= 0.0f || cutOffFrequency <= 0.0f) {
        return 1.0;
    }
    float rc = 1.0f/(M_2PI * cutOffFrequency);
    return constrainFloat(dt/(dt+rc), 0.0f, 1.0f);
}

template <typename T>
T angleNorm(T res) {
    if (fabs(res) == M_PI){
        return res;
    }

    res = wrap2PI(res + M_PI);
    return res - M_PI;
}

template <typename T>
T unwrap(const T prev, const T now) {
    return prev + angleNorm(now - prev);
}