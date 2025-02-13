#include "led_demo.h"



const AP_HAL::HAL& hal = AP_HAL::get_HAL();
void setup();
void loop();

void setup()
{ 
   
}

void loop()
{
    hal.gpio->write(53,1);
    hal.scheduler->delay(100);
    hal.gpio->write(53,0);
    hal.scheduler->delay(100);
}


AP_HAL_MAIN();