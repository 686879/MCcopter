#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

// we need a boardconfig created so that the io processor's enable
// parameter is available
#if HAL_WITH_IO_MCU
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

#define Turn_Channel (int16_t)last_value[0]
#define Thr_Channel (int16_t)last_value[2]
#define Lock_Channel (int16_t)last_value[4]
#define Turn_Mid 1500
#define Thr_Mid 1500
#define Threshold 350
#define Up_Rate 35
int16_t Differ_Speed=0;
uint16_t Thr_Forward_L=0;
uint16_t Thr_Backup_L=0;
uint16_t Thr_Forward_R=0;
uint16_t Thr_Backup_R=0;
uint16_t arm_flag = 0;
uint32_t ArmChekTime_Tick = 0;
void setup();
void loop();
void read_channels(void);
void update_Thr(void);
void ArmCheck();
void set_pwms(void);
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define MAX_CHANNELS 16

static uint8_t max_channels_display = 8;  // Set to 0 for display numbers of channels detected.
static uint16_t last_value[MAX_CHANNELS];

void setup(void)
{
    hal.console->printf("Starting RCInput test\n");
#if HAL_WITH_IO_MCU
    BoardConfig.init();
#endif
    for (uint8_t i = 0; i< 14; i++) {
        hal.rcout->enable_ch(i); //使能PWM输出通道（包括IO板及主控板）
    }
    hal.rcout->force_safety_off();//安全按钮强制解锁
    ArmChekTime_Tick = hal.util->get_hw_rtc();
}

void ArmCheck()
{
	uint32_t UpdateTime =0;

		//printf("yaw:%d ",Lock_Channel);
    if(!arm_flag)
    {
		if (Lock_Channel>1800) //输入，Yaw通道值大于5000，打到最大
		{
			UpdateTime = hal.util->get_hw_rtc() - ArmChekTime_Tick;
            hal.console->printf("%lu",UpdateTime);
			//printf(" ArmChekTime_Tick=%d ",ArmChekTime_Tick);
			//printf(" UpdateTime=%d ",UpdateTime);
			if (UpdateTime>3000000)
			{
                hal.console->printf("clock up");
				ArmChekTime_Tick = hal.util->get_hw_rtc();	
				arm_flag = 1;
			}
		}
		
		else
		{
		  ArmChekTime_Tick = hal.util->get_hw_rtc();	
		}
		
    }
	else
	{
		if (Lock_Channel<1200) //输入，Yaw通道值大于5000，打到最大
		{
			UpdateTime = hal.util->get_hw_rtc() - ArmChekTime_Tick;
            hal.console->printf("%lu",UpdateTime);
			//printf(" ArmChekTime_Tick=%d ",ArmChekTime_Tick);
			//printf(" UpdateTime=%d ",UpdateTime);
			if (UpdateTime>3000)
			{
                hal.console->printf("clock up");
				ArmChekTime_Tick = hal.util->get_hw_rtc();	
				arm_flag = 0;
			}
		}
		
		else
		{
		  ArmChekTime_Tick = hal.util->get_hw_rtc();	
		}
	}

}


void set_pwms(void)
{
    if (arm_flag)
    {
        hal.rcout->write(0,Thr_Forward_L);
        hal.rcout->write(1,Thr_Backup_L);
        hal.rcout->write(2,Thr_Forward_L);
        hal.rcout->write(3,Thr_Backup_L);
        hal.rcout->write(4,Thr_Forward_R);
        hal.rcout->write(5,Thr_Backup_R);
        hal.rcout->write(6,Thr_Forward_R);
        hal.rcout->write(7,Thr_Backup_R);
    }
    else
    {
        hal.rcout->write(0,0);
        hal.rcout->write(1,0);
        hal.rcout->write(2,0);
        hal.rcout->write(3,0);
        hal.rcout->write(4,0);
        hal.rcout->write(5,0);
        hal.rcout->write(6,0);
        hal.rcout->write(7,0);
    }
    

}

void update_Thr(void)
{
    int16_t Thr_Fix = 0;
		Differ_Speed = Turn_Channel - Turn_Mid;
	    //printf(" Differ_Speed:%d",Differ_Speed);
		if (Differ_Speed>Threshold)//上边界 threshold=1000
		{
			Differ_Speed = Threshold;
		}
		if (Differ_Speed<-Threshold)//下边界
		{
			Differ_Speed = -Threshold;
		}
		//left
		Thr_Fix = Thr_Channel+Differ_Speed;
		if ((Thr_Fix-Thr_Mid)>0)
		{
			if ((Thr_Fix-Thr_Mid)>500)
			{
				Thr_Fix = 500+Thr_Mid;
			}
			Thr_Forward_L = (Thr_Fix-Thr_Mid)*Up_Rate;//保证在0——10000之间 ，前轮
			Thr_Backup_L = 0; 
		}
		else
		{
			if ((Thr_Fix-Thr_Mid)<-500)
			{
				Thr_Fix = Thr_Mid-500;
			}
			Thr_Backup_L = (Thr_Fix-Thr_Mid)*-Up_Rate;
			Thr_Forward_L = 0;
		}
		//right
		Thr_Fix = Thr_Channel-Differ_Speed;
		if ((Thr_Fix-Thr_Mid)>0)
		{
			if ((Thr_Fix-Thr_Mid)>1500)
			{
				Thr_Fix = 1500+Thr_Mid;
			}
			Thr_Forward_R = (Thr_Fix-Thr_Mid)*Up_Rate;
			Thr_Backup_R = 0;
		}
		else
		{
			if ((Thr_Fix-Thr_Mid)<-1500)
			{
				Thr_Fix = Thr_Mid-1500;
			}
			Thr_Backup_R = (Thr_Fix-Thr_Mid)*-Up_Rate;
			Thr_Forward_R = 0;
		}
}
void read_channels(void)
{
    uint8_t nchannels = hal.rcin->num_channels();  // Get the numbers channels detected by RC_INPUT.
    if (nchannels == 0) {
        return;
    }
    if (max_channels_display == 0) {
        hal.console->printf("Channels detected: %2u\n", nchannels);
        hal.console->printf("Set max_channels_display > 0 to display channels values\n");
        return;
    }

    if (nchannels > MAX_CHANNELS) {
        nchannels = MAX_CHANNELS;
    }

    for (uint8_t i = 0; i < nchannels; i++) {
        uint16_t v = hal.rcin->read(i);
        if (last_value[i] != v) {
            last_value[i] = v;
        }
    }
    if (max_channels_display > nchannels) {
        max_channels_display = nchannels;
    }

}



void loop(void) {
    ArmCheck();
    read_channels();
    update_Thr();
    set_pwms();
    hal.scheduler->delay(10);
}

AP_HAL_MAIN();
