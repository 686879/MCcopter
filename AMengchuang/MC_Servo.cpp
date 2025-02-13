#include "MC_Servo.h"

extern const AP_HAL::HAL &hal;

void MC_Servo::set_pwm_range(uint16_t min, uint16_t max, uint16_t mid, double angle_max, double angle_min)
{
    _pwm_min = min;
    _pwm_mid = mid;
    _pwm_max = max;
    _angle_max = angle_max;
    _angle_min = angle_min;
    _scale = (_pwm_max - _pwm_min)/(_angle_max - _angle_min);
}

void MC_Servo::init(uint16_t frequency)
{
    _frequency = frequency;
    hal.rcout->set_freq(_rc_num,_frequency);
}

void MC_Servo::output(double target_angle)
{
    uint16_t pwm_out;
    pwm_out = _pwm_mid+_scale*target_angle;
    if (_scale>0) 
        pwm_out = constrain_uint16(pwm_out,_pwm_min,_pwm_max);
    else 
        pwm_out = constrain_uint16(pwm_out,_pwm_max,_pwm_min);
    hal.rcout->write(_rc_num,pwm_out+_def);
    _def = -_def;
}

