#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "MC_Servo.h"
#define MAX_SERVO_NUM 4
#define MAX_JOINT_NUM 4
#define L1 _joint_length[0]
#define L2 _joint_length[1]

class MC_Leg
{
public:
    MC_Leg(uint8_t joint_num, uint8_t servo_num, double joint_length[], uint16_t rc_nums[]);
    void init(double angle_range[][2], uint16_t pwm_range[][3]);
    void Forward_Kinematics(double angle_1, double angle_2, double &result_x, double &result_z);
    void F2Torque(double F_X, double F_Z, double angle_1, double angle_2, double &torque_1, double &torque_2);
    void Torque2Angle(double torque_1, double torque_2, double &angle_1, double &angle_2);
    void set_target_X_Z(double target_x, double target_z);//cm
    void update();

private:
    uint8_t _joint_num;
    uint8_t _servo_num;
    double _joint_length[MAX_JOINT_NUM];
    MC_Servo *_backend[MAX_SERVO_NUM];
    double _target_x;
    double _target_z;
    double _current_x;
    double _current_z;
    double _last_x;
    double _last_z;
    double _target_angle_1;
    double _target_angle_2;
    double _torque_1;
    double _torque_2;
    double _KP_F_X;
    double _KD_F_X;
    double _KP_F_Z;
    double _KD_F_Z;
    double _KP_S;
    double _F_X;
    double _F_Z;
};