#include "Copter.h"

#if MODE_TOP_ENABLED == ENABLED
#define TOP_THR_INC        0.25f   // throttle increase during FlipState::Start stage (under 45deg lean angle)
#define TOP_THR_DEC        0.24f   // throttle decrease during FlipState::Roll stage (between 45deg ~ -90deg roll)
#define TOP_ROTATION_RATE  8500   // rotation rate request in centi-degrees / sec (i.e. 400 deg/sec)
#define TOP_TIMEOUT_MS     2500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode
#define Turn_Channel (int16_t)last_value[0]
#define Thr_Channel (int16_t)last_value[2]
#define Lock_Channel (int16_t)last_value[5]
#define Turn_Mid 1500
#define Thr_Mid 1500
#define Threshold 350
#define Up_Rate 35
// flip_init - initialise flip controller
bool ModeTop::init(bool ignore_checks)
{
    // only allow flip from some flight modes, for example ACRO, Stabilize, AltHold or FlowHold flight modes

    // if in acro or stabilize ensure throttle is above zero
    //状态检查
    if (copter.ap.throttle_zero && (copter.flightmode->mode_number() == Mode::Number::ACRO || copter.flightmode->mode_number() == Mode::Number::STABILIZE)) {
        return false;
    }
    if (copter.flightmode->mode_number() != Mode::Number::STABILIZE&&copter.flightmode->mode_number() != Mode::Number::LOITER&&copter.flightmode->mode_number() != Mode::Number::ALT_HOLD){
        return false;
    }
    // ensure roll input is less than 40deg
    if (abs(channel_roll->get_control_in()) >= 4000) {
        return false;
    }
    // only allow flip when flying
    if (!motors->armed() || copter.ap.land_complete) {
        return false;
    }
    // capture original flight mode so that we can return to it after completion
    //获取原飞行模式
    orig_control_mode = copter.flightmode->mode_number();
    // initialise state
    //设置初始状态
    _state = TopState::Start;
    start_time_ms = millis();
    pitch_dir = -1;
    throttle_out = 0.0f;
    // 设置倾角限幅
    const float angle_max = copter.aparm.angle_max;
    orig_attitude.x = constrain_float(ahrs.roll_sensor, -angle_max, angle_max);
    orig_attitude.y = constrain_float(ahrs.pitch_sensor, -angle_max, angle_max);
    orig_attitude.z = ahrs.yaw_sensor;

    return true;
}

// run - runs the flip controller
// should be called at 100hz or more
void ModeTop::run()
{
    if (!motors->armed()) {
        _state = TopState::Abandon;
    }
    //获取遥控数据
    read_channels();
    // get pilot's desired throttle
    

    // set motors to full range
    //设置电机状态
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    switch (_state) {
//上升状态
    case TopState::Start:
        // under 45 degrees request 400deg/sec roll or pitch
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f , 0.0f);

        // increase throttle
        throttle_out = 0.5f;

        // 遥控器控制状态变换
        if (Lock_Channel>1500) {
            _state = TopState::Top;
        }
         break;
//顶部运动状态

    case TopState::Top:
        // between 45deg ~ -90deg request 400deg/sec roll
        //更新飞行器及车体运动输出量
        update_Thr();
        // decrease throttle
        break;


    case TopState::Abandon:
        // restore original flight mode
        if (!copter.set_mode(orig_control_mode, ModeReason::UNKNOWN)) {
            // this should never happen but just in case
            copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
        }

        break;
    }

}

void ModeTop::output_to_motors()
{

    if (_state == TopState::Top){
       if (Lock_Channel<1500){
        motors->output();
        motors->rc_write(0,1700);
        motors->rc_write(1,1700);
        motors->rc_write(2,1700);
        motors->rc_write(3,1700);
        motors->rc_write(4,0);
        motors->rc_write(5,0);
        motors->rc_write(6,0);
        motors->rc_write(7,0);
        }
        else{
            motors->output();
            motors->rc_write(0,1700);
            motors->rc_write(1,1700);
            motors->rc_write(2,1700);
            motors->rc_write(3,1700);
            motors->rc_write(6,Thr_Backup_R);
            motors->rc_write(7,Thr_Forward_R);
            motors->rc_write(5,Thr_Backup_L);
            motors->rc_write(4,Thr_Forward_L);
        }
    }
    else{
        motors->output();
        motors->rc_write(4,0);
        motors->rc_write(5,0);
        motors->rc_write(6,0);
        motors->rc_write(7,0);
    }

}

void ModeTop::update_Thr(void)
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
void ModeTop::read_channels(void)
{
    uint8_t nchannels = hal.rcin->num_channels();  // Get the numbers channels detected by RC_INPUT.
    if (nchannels == 0) {
        return;
    }

    if (nchannels > 8) {
        nchannels = 8;
    }

    for (uint8_t i = 0; i < nchannels; i++) {
        uint16_t v = hal.rcin->read(i);
        if (last_value[i] != v) {
            last_value[i] = v;
        }
    }

}

#endif
