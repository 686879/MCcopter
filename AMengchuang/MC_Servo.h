#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
class MC_Servo
{
public:
    MC_Servo(uint16_t rc_num)
    {
        _rc_num = rc_num;
    }
    void set_pwm_range(uint16_t min, uint16_t max, uint16_t mid, double angle_max, double angle_min);
    void output(double target_angle);
    void init(uint16_t frequency);
private:
    uint16_t _rc_num;
    uint16_t _pwm_max;
    uint16_t _pwm_min;
    uint16_t _pwm_mid;
    uint16_t _frequency;
    uint16_t _def = 5;
    double _angle_max;
    double _angle_min;
    double _scale;
};