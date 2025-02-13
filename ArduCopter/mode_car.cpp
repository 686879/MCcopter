#include "Copter.h"
//注意，当前版本已修改
#if MODE_CAR_ENABLED == ENABLED
#define Turn_Channel (int16_t)last_value[1]
#define Thr_Channel (int16_t)last_value[2]
#define Lock_Channel (int16_t)last_value[5]
#define Mode_Channel (int16_t)last_value[8]
#define Deita_Channel (int16_t)last_value[12]
#define Turn_Mid 1500
#define Thr_Mid 1500
#define Threshold 350
#define Up_Rate 3
#define PWM_Max 2350
#define PWM_Min 900
#define PWM_delta_1 1
#define PWM_delta_2 5
bool ModeCar::init(bool ignore_checks)
{
    // only allow flip from some flight modes, for example ACRO, Stabilize, AltHold or FlowHold flight modes
    // ensure roll input is less than 40deg
    if (abs(channel_roll->get_control_in()) >= 4000) {
        return false;
    }
    Differ_Speed=0;
    Thr_Forward_L=0;
	Thr_PWM=0;
    Thr_Backup_L=0;
    Thr_Forward_R=0;
    Thr_Backup_R=0;
    pwm_10 = 1500*10;
    return true;
}

// run - runs the flip controller
// should be called at 100hz or more
void ModeCar::run()
{
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f,0.0f , 0.0f);
    read_channels();
    update_Thr();
    attitude_control->set_throttle_out(0, true, g.throttle_filt);

}


void ModeCar::output_to_motors()
{
    if (Lock_Channel<1500){
        motors->rc_write(4,0);
        motors->rc_write(5,0);
        motors->rc_write(6,0);
        motors->rc_write(7,0);
        motors->rc_write(0,0);
        motors->rc_write(1,0);
        motors->rc_write(2,0);
        motors->rc_write(3,0);
    }
    else{
        motors->rc_write(6,0);
        motors->rc_write(7,0);
        motors->rc_write(5,0);
        motors->rc_write(4,Thr_PWM);
        motors->rc_write(0,Thr_Forward_R);
        motors->rc_write(1,Thr_Backup_R);
        motors->rc_write(2,Thr_Forward_L);
        motors->rc_write(3,Thr_Backup_L);
    }

}

void ModeCar::update_Thr(void)
{
    int16_t Thr_Fix = 0; 
	int16_t delta_pwm = 0;
		Differ_Speed = 0;
	    //printf(" Differ_Speed:%d",Differ_Speed);
		if (Mode_Channel<1500)
		{
			delta_pwm = PWM_delta_2;
		}
		else
		{
			delta_pwm = PWM_delta_1;
		}
		if ((Deita_Channel-1500)>100)
		{
			pwm_10 = MIN(pwm_10+delta_pwm,PWM_Max*10);
			Thr_PWM = uint16_t(pwm_10/10);
		}
		else if ((Deita_Channel-1500)<-100)
		{
			pwm_10 = MAX(pwm_10-delta_pwm,PWM_Min*10);
			Thr_PWM = uint16_t(pwm_10/10);
		}
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
		if ((Thr_Fix-Thr_Mid)>100)
		{
			if ((Thr_Fix-Thr_Mid)>500)
			{
				Thr_Fix = 500+Thr_Mid;
			}
			Thr_Forward_L = (Thr_Fix-Thr_Mid)*Up_Rate;//保证在0——10000之间 ，前轮
			Thr_Backup_L = 0; 
		}
		else if ((Thr_Fix-Thr_Mid)<-100)
		{
			if ((Thr_Fix-Thr_Mid)<-500)
			{
				Thr_Fix = Thr_Mid-500;
			}
			Thr_Backup_L = (Thr_Fix-Thr_Mid)*-Up_Rate;
			Thr_Forward_L = 0;
		}
		else
		{
			Thr_Backup_L = 0;
			Thr_Forward_L = 0;
		}
		//right
		Thr_Fix = Turn_Channel-Differ_Speed;
		if ((Thr_Fix-Thr_Mid)>100)
		{
			if ((Thr_Fix-Thr_Mid)>1500)
			{
				Thr_Fix = 1500+Thr_Mid;
			}
			Thr_Forward_R = (Thr_Fix-Thr_Mid)*Up_Rate;
			Thr_Backup_R = 0;
		}
		else if ((Thr_Fix-Thr_Mid)<-100)
		{
			if ((Thr_Fix-Thr_Mid)<-1500)
			{
				Thr_Fix = Thr_Mid-1500;
			}
			Thr_Backup_R = (Thr_Fix-Thr_Mid)*-Up_Rate;
			Thr_Forward_R = 0;
		}
		else
		{
			Thr_Backup_R = 0;
			Thr_Forward_R = 0;
		}
}
void ModeCar::read_channels(void)
{
    uint8_t nchannels = hal.rcin->num_channels();  // Get the numbers channels detected by RC_INPUT.
    if (nchannels == 0) {
        return;
    }

    if (nchannels > 16) {
        nchannels = 16;
    }

    for (uint8_t i = 0; i < nchannels; i++) {
        uint16_t v = hal.rcin->read(i);
        if (last_value[i] != v) {
            last_value[i] = v;
        }
    }

}

#endif
