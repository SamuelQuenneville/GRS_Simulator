/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_PID_H
#define GRS_SIMULATOR_PID_H

#include <cstring>
#include "Control/slewLimiter.h"
#include "Mathematics//grsMath.h"

class PID
{
public:
    PID(float proportionalGain, float integralGain, float derivativeGain, float feedForwardGain, float IntegratorMaxValue, float inputFilterTargetHz,
        float inputFilterDerivativeHz, float inputFilterErrorHz, float dt, float initialSrmax=0.0, float initialSrtau=1.0);

    // set time step in seconds
    void setDt(float dt);

    /* set target and measured inputs to PID controller and calculate outputs
     * target and error are filtered
     * the derivative is then calculated and filtered
     * the integral is then updated based on the setting of the limit flag
     */
    float updateAll(float target, float measurement, bool limit = false);

    // update the integral
    // if the limit flag is set the integral is only allowed to shrink
    void updateIntegral(bool limit);

    // get results from pid controller
    float getI() const;
    float getFF();

    // reset the integrator
    void resetI();

    // get accessor
    float getFilterAlpha(float filterHz) const;
    float getFilterTargetAlpha() const;
    float getFilterErrorAlpha() const;
    float getFilterDerivativeAlpha() const;

    // set accessor
    void filterTargetHz(const float hz);
    void filterErrorHz(const float hz);
    void filterDerivativeHz(const float hz);

    // operator function call for easy initialisation
    void operator()(float proportionalGain, float integralGain, float derivativeGain, float feedForwardGain, float integralMinValue, float integralMaxValue,
                    float inputFilterTargetHz, float inputFilterErrorHz, float inputFilterDerivativeHz, float dt);

    // integrator setting function
    void setIntegrator(float i);

    // flags
    struct pidFlags {
        bool resetFilter :1; // true when input filter should be reset during next call to set_input
    } flags;

    struct PidInfo {
        float target;
        float actual;
        float error;
        float P;
        float I;
        float D;
        float FF;
        float Dmod;
        float slew_rate;
        bool  limit;
    };

    const PidInfo& getPidInfo(void) const { return m_pidInfo;};

protected:
    float m_kp;
    float m_ki;
    float m_kd;
    float m_kff;
    float m_kimin;
    float m_kimax;
    float m_filterTargetHz;
    float m_filterErrorHz;
    float m_filterDerivativeHz;
    float m_slewRateMax;
    float m_slewRateTau;

    float m_dt;
    float m_integrator;
    float m_derivative;
    float m_target;
    float m_error;
    int8_t m_slewLimitScale;
    uint16_t m_resetCounter;

    PidInfo m_pidInfo;
    SlewLimiter m_slewLimiter{m_slewRateMax, m_slewRateTau};
};

#endif //GRS_SIMULATOR_PID_H
