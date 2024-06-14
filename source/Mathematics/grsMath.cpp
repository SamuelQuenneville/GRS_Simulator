/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "grsMath.h"


template <typename T>
float wrap_PI(const T radian)
{
    auto res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}

template <typename T>
float wrap_2PI(const T radian)
{
    float res = fmod(static_cast<float>(radian), M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

template <typename T>
T constrain_value_line(const T amt, const T low, const T high, uint32_t line)
{
    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

template float constrain_value_line<float>(const float amt, const float low, const float high, uint32_t line);

template <typename T>
T constrain_value(const T amt, const T low, const T high)
{
    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

template int constrain_value<int>(const int amt, const int low, const int high);
template long constrain_value<long>(const long amt, const long low, const long high);
template long long constrain_value<long long>(const long long amt, const long long low, const long long high);
template short constrain_value<short>(const short amt, const short low, const short high);
template float constrain_value<float>(const float amt, const float low, const float high);
template double constrain_value<double>(const double amt, const double low, const double high);

/*
  calculate a low pass filter alpha value
 */
float calc_lowpass_alpha_dt(float dt, float cutoff_freq)
{
    if (dt <= 0.0f || cutoff_freq <= 0.0f) {
        return 1.0;
    }
    float rc = 1.0f/(M_2PI*cutoff_freq);
    return constrain_float(dt/(dt+rc), 0.0f, 1.0f);
}

template <typename T>
T angle_norm(T res)
{
    if (fabs(res) == M_PI){
        return res;
    }

    res = wrap_2PI(res + M_PI);
    return res - M_PI;
}

template <typename T>
T unwrap(const T prev, const T now)
{
    return prev + angle_norm(now - prev);
}