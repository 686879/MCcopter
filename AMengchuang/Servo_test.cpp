// #include "ArduDog.h"

// const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// #if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
// #include <AP_BoardConfig/AP_BoardConfig.h>
// #include <AP_IOMCU/AP_IOMCU.h>
// AP_BoardConfig BoardConfig;
// #endif

// MC_Servo servo1(0);
// void setup();
// void loop();

// double angle;
// void setup(void)
// {
//     #if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS //初始化IO板
//     hal.console->printf("boardinit");
//     BoardConfig.init();
//     #endif
//     angle = 0;
//     servo1.init(50);
//     servo1.set_pwm_range(2440,940,1840,120.0,-80.0);
//     hal.rcout->force_safety_off();//安全按钮强制解锁
// }
// void loop(void)
// {
//     int16_t user_input;
//     while (hal.console->available()) { 
//         user_input = hal.console->read();//获取串口输入，根据串口输入改变占空比

//         if (user_input == 'U' || user_input == 'u') {
//            angle = angle+5;
//         }

//         if (user_input == 'D' || user_input == 'd') {
//            angle = angle-5 ;
//         }
//     }
//     servo1.output(angle);
//     hal.scheduler->delay(50);
// }
// //AP_HAL_MAIN();