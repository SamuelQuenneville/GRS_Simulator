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
    PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filter_target_hz,
        float initial_filter_derivative_hz, float initial_filter_error_hz, float dt, float initial_srmax=0.0, float initial_srtau=1.0);

    // set time step in seconds
    void set_dt(float dt);

    /* set target and measured inputs to PID controller and calculate outputs
     * target and error are filtered
     * the derivative is then calculated and filtered
     * the integral is then updated based on the setting of the limit flag
     */
    float update_all(float target, float measurement, bool limit = false);

    // update the integral
    // if the limit flag is set the integral is only allowed to shrink
    void update_integral(bool limit);

    // get results from pid controller
    float get_i() const;
    float get_ff();

    // reset the integrator
    void reset_I();

    // get accessor
    float get_filter_alpha(float filter_hz) const;
    float get_filter_target_alpha() const;
    float get_filter_error_alpha() const;
    float get_filter_derivative_alpha() const;

    // set accessor
    void filter_target_hz(const float v);
    void filter_error_hz(const float v);
    void filter_derivative_hz(const float v);

    // operator function call for easy initialisation
    void operator()(float p_val, float i_val, float d_val, float ff_val, float imin_val, float imax_val,
                    float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz, float dt);

    // integrator setting function
    void set_integrator(float i);

    // flags
    struct pid_flags {
        bool _reset_filter :1; // true when input filter should be reset during next call to set_input
    } _flags;

    void set_reset_flag();

    struct PID_Info {
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

    const PID_Info& getPID_info(void) const { return _pid_info; };

protected:
    float _kp;
    float _ki;
    float _kd;
    float _kff;
    float _kimin;
    float _kimax;
    float _filter_target_hz;
    float _filter_error_hz;
    float _filter_derivative_hz;
    float _slew_rate_max;
    float _slew_rate_tau;

    float _dt;
    float _integrator;
    float _derivative;
    float _target;
    float _error;
    int8_t _slew_limit_scale;
    uint16_t _reset_counter;

    PID_Info _pid_info;
    SlewLimiter _slew_limiter{_slew_rate_max, _slew_rate_tau};

};

#endif //GRS_SIMULATOR_PID_H
