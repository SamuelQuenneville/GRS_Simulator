/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "pid.h"


PID::PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filter_target_hz,
         float initial_filter_derivative_hz, float initial_filter_error_hz, float dt, float initial_srmax, float initial_srtau)
        : _dt(dt)
{
    _kp = initial_p;
    _ki = initial_i;
    _kd = initial_d;
    _kff = initial_ff;
    _kimax = fabsf(initial_imax);
    _kimin = -_kimax;
    filter_target_hz(initial_filter_target_hz);
    filter_error_hz(initial_filter_error_hz);
    filter_derivative_hz(initial_filter_derivative_hz);

    _slew_rate_max = initial_srmax;
    _slew_limit_scale = 1;

    _flags._reset_filter = true;

    std::memset(&_pid_info, 0, sizeof(_pid_info));
}

// Overload the function call operator to permit easy initialisation
void PID::operator()(float p_val, float i_val, float d_val, float ff_val, float imin_val, float imax_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz, float dt)
{
    _kp = p_val;
    _ki = i_val;
    _kd = d_val;
    _kff = ff_val;
    _kimin = imin_val;
    _kimax = fabsf(imax_val);
    _filter_target_hz = input_filt_T_hz;
    _filter_error_hz = input_filt_E_hz;
    _filter_derivative_hz = input_filt_D_hz;
    _dt = dt;
}

float PID::update_all(float target, float measurement, bool limit)
{
    if (!std::isfinite(target) || !std::isfinite(measurement)){
        return 0.0f;
    }

    // reset input filter to value
    if (_flags._reset_filter){
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _target += get_filter_target_alpha() * (target - _target);
        _error += get_filter_error_alpha() * ((_target - measurement) - _error);

        // calculate derivative than filter it
        if (_dt > 0.0f) {
            float derivative = (_error - error_last) / _dt;
            _derivative += get_filter_derivative_alpha() * (derivative - _derivative);
        }
    }

    // update integral term
    update_integral(limit);

    float proportional_out = _error * _kp;
    float derivative_out = _derivative * _kd;

    // calculate slew limit modifier and applied them to PD
    _pid_info.Dmod = _slew_limiter.modifier((_pid_info.P + _pid_info.D) * _slew_limit_scale, _dt);
    _pid_info.slew_rate = _slew_limiter.get_slew_rate();

    proportional_out *= _pid_info.Dmod;
    derivative_out *= _pid_info.Dmod;

    _pid_info.target = _target;
    _pid_info.actual = measurement;
    _pid_info.error = _error;
    _pid_info.P = proportional_out;
    _pid_info.D = derivative_out;

    return proportional_out + _integrator + derivative_out;
}

void PID::update_integral(bool limit)
{
    if (!is_zero(_ki) && is_positive(_dt)) {
        // ensure that integrator can only be reduced if the output is saturated
        if (!limit || ((is_positive(_integrator) && is_negative(_error)) || (is_negative(_integrator) && is_positive(_error)))) {
            _integrator += ((float)_error * _ki) * _dt;
            _integrator = constrain_float(_integrator, _kimin, _kimax);
        }
    } else {
        _integrator = 0.0f;
    }
    _pid_info.I = _integrator;
    _pid_info.limit = limit;
}

void PID::reset_I()
{
    _integrator = 0;
}

// set time step in seconds
void PID::set_dt(float dt)
{
    _dt = dt;
}

// set target filter hz
void PID::filter_target_hz(float hz)
{
    _filter_target_hz = hz;
}

// set error filter hz
void PID::filter_error_hz(float hz)
{
    _filter_error_hz = hz;
}

// set derivative filter hz
void PID::filter_derivative_hz(float hz)
{
    _filter_derivative_hz = hz;
}

// get the target filter alpha
float PID::get_filter_target_alpha() const
{
    return get_filter_alpha(_filter_target_hz);
}

// get the error filter alpha
float PID::get_filter_error_alpha() const
{
    return get_filter_alpha(_filter_error_hz);
}

// get the derivative filter alpha
float PID::get_filter_derivative_alpha() const
{
    return get_filter_alpha(_filter_derivative_hz);
}

// calculate a filter alpha
float PID::get_filter_alpha(float filter_hz) const
{
    return calc_lowpass_alpha_dt(_dt, filter_hz);
}

float PID::get_i() const
{
    return _integrator;
}

float PID::get_ff()
{
    _pid_info.FF = _target * _kff;
    return _target * _kff;
}

void PID::set_integrator(float i)
{
    _integrator = constrain_float(i, _kimin, _kimax);
    _pid_info.I = _integrator;
}
