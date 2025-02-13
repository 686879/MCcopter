#pragma once
#include <AP_HAL/AP_HAL.h>
#include "MC_Servo.h"
#include "MC_Leg.h"

#define LEG_NUM 4

class ArduDog
{
public:
    ArduDog(double leg_length[],uint16_t rc_nums[LEG_NUM][2]);
    enum class GaitMode : uint8_t {
        Stand=0,
        Walk=1,
        Trot=2
    };
    void init(double angle_ranges[LEG_NUM][2][2], uint16_t pwm_ranges[LEG_NUM][2][3]);
    void update();
    void set_gait(GaitMode gait)
    {
        _gait = gait;
    }
    void Trot_run();

private:
    double _X_S;
    double _X_F;
    double _Z_S;
    double _Z_F;
    double _target_X;
    double _target_Z;
    double _Ts;
    double _sigma;
    GaitMode _gait;
    MC_Leg *_backend[LEG_NUM];
};
