/*
  simple test of RC output interface
  Attention: If your board has safety switch,
  don't forget to push it to enable the PWM output.
 */

#include <AP_HAL/AP_HAL.h>

// we need a boardconfig created so that the io processor's enable
// parameter is available
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static uint16_t pwm[8] = {1240,1800,2060,940,1300,1760,1300,1700};
static uint16_t ch = 0;
static uint16_t def;
void setup (void)
{
    def = 5;
    hal.console->printf("Starting AP_HAL::RCOutput test\n");
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS //初始化IO板
    hal.console->printf("boardinit");
    BoardConfig.init();
#endif
    for (uint8_t i = 0; i< 14; i++) {
        hal.rcout->enable_ch(i); //使能PWM输出通道（包括IO板及主控板）
    }
    hal.rcout->set_freq(0,50); //设置PWM输出通道频率，默认值为50HZ
    hal.rcout->set_freq(1,50); //注意：根据PWM通道分组的不同部分PWM通道不允许频率超过50HZ
    hal.rcout->set_freq(2,50);
    hal.rcout->set_freq(3,50); //设置PWM输出通道频率，默认值为50HZ
    hal.rcout->set_freq(4,50); //注意：根据PWM通道分组的不同部分PWM通道不允许频率超过50HZ
    hal.rcout->set_freq(5,50);
    hal.rcout->set_freq(6,50); //设置PWM输出通道频率，默认值为50HZ
    hal.rcout->set_freq(7,50); //注意：根据PWM通道分组的不同部分PWM通道不允许频率超过50HZ
    hal.rcout->force_safety_off();//安全按钮强制解锁
}


void loop (void)
{
    int16_t user_input;

    
    // read in user input
    while (hal.console->available()) { 
        user_input = hal.console->read();//获取串口输入，根据串口输入改变占空比

        if (user_input == 'U' || user_input == 'u') {
           pwm[ch]=pwm[ch]+100;
        }

        if (user_input == 'D' || user_input == 'd') {
           pwm[ch]=pwm[ch]-100; ;
        }
        if (user_input == 'w' || user_input == 'W') {
           pwm[ch]=pwm[ch]+10;
        }

        if (user_input == 's' || user_input == 'S') {
           pwm[ch]=pwm[ch]-10; ;
        }
        if (user_input == 'c')
        {
            ch = (ch+1)%8;
            hal.console->printf("ch:%d",ch);
        }
        hal.scheduler->delay(200);
        
    }
    def = -def;
    for (uint16_t i=0;i<8;i++)
    {
        hal.rcout->write(i,pwm[i]+def);
        hal.console->printf("pwm:%d,%d\n",pwm[i],i);
        
    }
    hal.scheduler->delay(100);

}

AP_HAL_MAIN();
