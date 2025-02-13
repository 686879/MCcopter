#include "MC_Leg.h"

extern const AP_HAL::HAL &hal;
MC_Leg::MC_Leg(uint8_t joint_num, uint8_t servo_num, double joint_length[], uint16_t rc_nums[])
{
    _KP_S = 1.1459156;
    _KP_F_X = 0.4;
    _KP_F_Z = 0.4;
    _KD_F_X = 0;
    _KD_F_Z = 0;
    _joint_num = joint_num;
    if (_joint_num<MAX_JOINT_NUM)
    {
        for (int i=0;i<_joint_num;i++)
        {
            _joint_length[i] = joint_length[i];
        }
    }
    _servo_num = servo_num;
    if (_servo_num<MAX_SERVO_NUM)
    {
        for (int i=0;i<_servo_num;i++)
        {
            _backend[i] = new MC_Servo(rc_nums[i]);
        }
    }
}

void MC_Leg::init(double angle_range[][2], uint16_t pwm_range[][3])
{
    for (int i=0;i<_servo_num;i++)
    {
        _backend[i]->init(50);
        _backend[i]->set_pwm_range(pwm_range[i][0],pwm_range[i][1],pwm_range[i][2],angle_range[i][0],angle_range[i][1]);
    }
    _target_angle_1 = -45.0;
    _target_angle_2 = 90.0;
    Forward_Kinematics(_target_angle_1, _target_angle_2, _current_x, _current_z);
    _last_x = _current_x;
    _last_z = _current_z;
    _target_x = _current_x;
    _target_z = _current_z;
    _backend[0]->output(_target_angle_1);
    _backend[1]->output(_target_angle_2);
}

void MC_Leg::set_target_X_Z(double target_x, double target_z)
{
    _target_x = target_x;
    _target_z = target_z; 
}

void MC_Leg::F2Torque(double F_X, double F_Z, double angle_1, double angle_2, double &torque_1, double &torque_2)
{
    torque_1 = F_X*(L1*cosf(radians(angle_1))+L2*cosf(radians(angle_1))*cosf(radians(angle_2))-L2*sinf(radians(angle_2))*sinf(radians(angle_1))) - F_Z*(L1*sinf(radians(angle_1))+L2*sinf(radians(angle_1))*cosf(radians(angle_2))+L2*cosf(radians(angle_1))*sinf(radians(angle_2)));
    torque_2 = F_X*(L2*cosf(radians(angle_1))*cosf(radians(angle_2))-L2*sinf(radians(angle_2))*sinf(radians(angle_1))) - F_Z*(L2*sinf(radians(angle_1))*cosf(radians(angle_2))+L2*cosf(radians(angle_1))*sinf(radians(angle_2)));
}

void MC_Leg::Torque2Angle(double torque_1, double torque_2, double &angle_1, double &angle_2)
{
    angle_1 = angle_1 + _KP_S*torque_1;
    angle_2 = angle_2 + _KP_S*torque_2;
}

void MC_Leg::Forward_Kinematics(double angle_1, double angle_2, double &result_x, double &result_z)
{
    result_z = L1*cosf(radians(angle_1))+L2*cosf(radians(angle_1))*cosf(radians(angle_2))-L2*sinf(radians(angle_1))*sinf(radians(angle_2));
    result_x = L1*sinf(radians(angle_1))+L2*sinf(radians(angle_1))*cosf(radians(angle_2))+L2*cosf(radians(angle_1))*sinf(radians(angle_2));
}

void MC_Leg::update()
{
    Forward_Kinematics(_target_angle_1, _target_angle_2, _current_x, _current_z);
    _F_X = _KP_F_X*(_target_x - _current_x) + _KD_F_X*(_last_x - _current_x);
    _F_Z = _KP_F_Z*(_target_z - _current_z) + _KD_F_Z*(_last_z - _current_z);
    F2Torque(_F_X,_F_Z,_target_angle_1,_target_angle_2,_torque_1,_torque_2);
    Torque2Angle(_torque_1,_torque_2,_target_angle_1,_target_angle_2);
    _backend[0]->output(_target_angle_1);
    _backend[1]->output(_target_angle_2);
}