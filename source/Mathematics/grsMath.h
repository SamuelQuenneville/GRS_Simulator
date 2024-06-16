/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_GRSMATH_H
#define GRS_SIMULATOR_GRSMATH_H

#pragma once

#include <cmath>
#include <cstdint>

#ifdef M_PI
#undef M_PI
#endif
#define M_PI (3.141592653589793f)

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

#ifdef M_PI_2
# undef M_PI_2
#endif
#define M_2PI (M_PI * 2)

template<typename A, typename B>
static inline auto MIN(const A& one, const B& two) -> decltype(one < two ? one : two) {
    return one < two ? one : two;
}

template<typename A, typename B>
static inline auto MAX(const A& one, const B& two) -> decltype(one > two ? one : two) {
    return one > two ? one : two;
}

/*
  wrap an angle in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
template <typename T>
float wrapPI(T radian);

/*
 * wrap an angle in radians to 0..2PI
 */
template <typename T>
float wrap2PI(T radian);

// degrees -> radians
template <typename T>
static inline constexpr double radians(T deg) {
    return deg * DEG_TO_RAD;
}

// radians -> degrees
template <typename T>
static inline constexpr float degrees(T rad) {
    return rad * RAD_TO_DEG;
}

/*
 * Constrain a value to be within the range: low and high
 */
template <typename T>
T constrainValue(T amt, T low, T high);

template <typename T>
T constrainValueLine(T amt, T low, T high, uint32_t line);

#define constrainFloat(amt, low, high) constrainValueLine(float(amt), float(low), float(high), uint32_t(__LINE__))

inline int32_t constrainInt32(const int32_t amt, const int32_t low, const int32_t high) {
    return constrainValue(amt, low, high);
}

/*
 * calculate a low pass filter alpha value
 */
float calcLowpassAlphaDt(float dt, float cutOffFrequency);

/*
 *  use for unwrap
 */
template <typename T>
T angleNorm(T res);

/*
 * Whenever the jump between consecutive angles is greater than or equal to π radians,
 * unwrap shifts the angles by adding multiples of ±2π until the jump is less than π
 */
template <typename T>
T unwrap(T prev, T now);


/*
 * @brief: Check whether a float is zero
 */
template <typename T>
inline bool isZero(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,float>::value,
                  "Template parameter not of type float");
    return (fabsf(static_cast<float>(fVal1)) < __FLT_EPSILON__);
}

/*
 * @brief: Check whether a float is greater than zero
 */
template <typename T>
inline bool isPositive(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,float>::value,
                  "Template parameter not of type float");
    return (static_cast<float>(fVal1) >= __FLT_EPSILON__);
}

/*
 * @brief: Check whether a float is less than zero
 */
template <typename T>
inline bool isNegative(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,float>::value,
                  "Template parameter not of type float");
    return (static_cast<float>(fVal1) <= (-1.0 * __FLT_EPSILON__));
}

#endif //GRS_SIMULATOR_GRSMATH_H
