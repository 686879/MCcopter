// #include "ArduDog.h"

// const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// ArduDog::ArduDog(double leg_length[],uint16_t rc_nums[LEG_NUM][2])
// {
//     for (int i = 0;i<LEG_NUM;i++)
//     {
//         _backend[i] = new MC_Leg(2,2,leg_length,rc_nums[i]);
//     }
// }

// void ArduDog::init(double angle_ranges[LEG_NUM][2][2], uint16_t pwm_ranges[LEG_NUM][2][3])
// {
//     for (int i = 0;i<LEG_NUM;i++)
//     {
//         _backend[i]->init(angle_ranges[i],pwm_ranges[i]);
//     }
//     _Ts = 100;  //ms
//     _Z_S = 50;
//     _Z_F = 50;
//     _X_S = 0;
//     _X_F = 0;
// }

// void ArduDog::Trot_run()
// {
    
// }
