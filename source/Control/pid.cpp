/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "pid.h"

#include <float.h>

PID::PID(float proportionalGain, float integralGain, float derivativeGain, float feedForwardGain, float IntegratorMaxValue, float inputFilterTargetHz,
         float inputFilterDerivativeHz, float inputFilterErrorHz, float dt, float initialSrmax, float initialSrtau)
        : m_dt(dt)
{
    m_kp = proportionalGain;
    m_ki = integralGain;
    m_kd = derivativeGain;
    m_kff = feedForwardGain;
    m_kimax = fabsf(IntegratorMaxValue);
    m_kimin = -m_kimax;
    filterTargetHz(inputFilterTargetHz);
    filterErrorHz(inputFilterErrorHz);
    filterDerivativeHz(inputFilterDerivativeHz);

    m_slewRateMax = initialSrmax;
    m_slewRateTau = initialSrtau;
    m_slewLimitScale = 1;

    flags.resetFilter = true;

    std::memset(&m_pidInfo, 0, sizeof(m_pidInfo));
}

// Overload the function call operator to permit easy initialisation
void PID::operator()(float proportionalGain, float integralGain, float derivativeGain, float feedForwardGain, float integralMinValue, float integralMaxValue,
                     float inputFilterTargetHz, float inputFilterErrorHz, float inputFilterDerivativeHz, float dt) {
    m_kp = proportionalGain;
    m_ki = integralGain;
    m_kd = derivativeGain;
    m_kff = feedForwardGain;
    m_kimin = integralMinValue;
    m_kimax = fabsf(integralMaxValue);
    m_filterTargetHz = inputFilterTargetHz;
    m_filterErrorHz = inputFilterErrorHz;
    m_filterDerivativeHz = inputFilterDerivativeHz;
    m_dt = dt;
}

float PID::updateAll(const float target, const float measurement, const bool limit) {
    if (!std::isfinite(target) || !std::isfinite(measurement)){
        return 0.0f;
    }

    // reset input filter to value
    if (flags.resetFilter){
        flags.resetFilter = false;
        m_target = target;
        m_error = m_target - measurement;
        m_derivative = 0.0f;
    } else {
        const float errorLast = m_error;
        m_target += getFilterTargetAlpha() * (target - m_target);
        m_error += getFilterErrorAlpha() * ((m_target - measurement) - m_error);

        // calculate derivative than filter it
        if (m_dt > 0.0f) {
            const float derivative = (m_error - errorLast) / m_dt;
            m_derivative += getFilterDerivativeAlpha() * (derivative - m_derivative);
        }
    }

    // update integral term
    updateIntegral(limit);

    float proportionalOut = m_error * m_kp;
    float derivativeOut = m_derivative * m_kd;

    // calculate slew limit modifier and applied them to PD
    m_pidInfo.Dmod = m_slewLimiter.modifier((m_pidInfo.P + m_pidInfo.D) * static_cast<float>(m_slewLimitScale), m_dt);
    m_pidInfo.slew_rate = m_slewLimiter.getSlewRate();

    proportionalOut *= m_pidInfo.Dmod;
    derivativeOut *= m_pidInfo.Dmod;

    m_pidInfo.target = m_target;
    m_pidInfo.actual = measurement;
    m_pidInfo.error = m_error;
    m_pidInfo.P = proportionalOut;
    m_pidInfo.D = derivativeOut;

    return proportionalOut + m_integrator + derivativeOut;
}

void PID::updateIntegral(const bool limit) {
    if (std::abs(m_ki) > FLT_EPSILON && m_dt > FLT_EPSILON) {
        // ensure that integrator can only be reduced if the output is saturated
        if (!limit || ((m_integrator > FLT_EPSILON && m_error < -FLT_EPSILON) || (m_integrator < -FLT_EPSILON && m_error > FLT_EPSILON))) {
            m_integrator += (m_error * m_ki) * m_dt;
            m_integrator = std::clamp(m_integrator, m_kimin, m_kimax);
        }
    } else {
        m_integrator = 0.0f;
    }
    m_pidInfo.I = m_integrator;
    m_pidInfo.limit = limit;
}

void PID::resetI() {
    m_integrator = 0;
}

// set time step in seconds
void PID::setDt(const float dt) {
    m_dt = dt;
}

// set target filter hz
void PID::filterTargetHz(const float hz) {
    m_filterTargetHz = hz;
}

// set error filter hz
void PID::filterErrorHz(const float hz) {
    m_filterErrorHz = hz;
}

// set derivative filter hz
void PID::filterDerivativeHz(const float hz) {
    m_filterDerivativeHz = hz;
}

// get the target filter alpha
float PID::getFilterTargetAlpha() const {
    return getFilterAlpha(m_filterTargetHz);
}

// get the error filter alpha
float PID::getFilterErrorAlpha() const {
    return getFilterAlpha(m_filterErrorHz);
}

// get the derivative filter alpha
float PID::getFilterDerivativeAlpha() const {
    return getFilterAlpha(m_filterDerivativeHz);
}

// calculate a filter alpha
float PID::getFilterAlpha(const float filterHz) const {
    if (m_dt <= 0.0f || filterHz <= 0.0f) {
        return 1.0f;
    }
    const float rc = 1.0f / (2.0f * M_PIf * filterHz);
    return std::clamp(m_dt / (m_dt + rc), 0.0f, 1.0f);
}

float PID::getI() const {
    return m_integrator;
}

float PID::getFF() {
    m_pidInfo.FF = m_target * m_kff;
    return m_target * m_kff;
}

void PID::setIntegrator(const float i) {
    m_integrator = std::clamp(i, m_kimin, m_kimax);
    m_pidInfo.I = m_integrator;
}
