#include "Copter.h"

#if MODE_RECOVERY_ENABLED == ENABLED

/*
 * Init and run calls for flip flight mode
 *      original implementation in 2010 by Jose Julio
 *      Adapted and updated for AC2 in 2011 by Jason Short
 *
 *      Controls:
 *          RC7_OPTION - RC12_OPTION parameter must be set to "Flip" (AUXSW_FLIP) which is "2"
 *          Pilot switches to Stabilize, Acro or AltHold flight mode and puts ch7/ch8 switch to ON position
 *          Vehicle will Roll right by default but if roll or pitch stick is held slightly left, forward or back it will flip in that direction
 *          Vehicle should complete the roll within 2.5sec and will then return to the original flight mode it was in before flip was triggered
 *          Pilot may manually exit flip by switching off ch7/ch8 or by moving roll stick to >40deg left or right
 *
 *      State machine approach:
 *          FlipState::Start (while copter is leaning <45deg) : roll right at 400deg/sec, increase throttle
 *          FlipState::Roll (while copter is between +45deg ~ -90) : roll right at 400deg/sec, reduce throttle
 *          FlipState::Recover (while copter is between -90deg and original target angle) : use earth frame angle controller to return vehicle to original attitude
 */

#define RECOVERY_THR_INC        0.15f   // throttle increase during FlipState::Start stage (under 45deg lean angle)
#define RECOVERY_THR_DEC        0.30f   // throttle decrease during FlipState::Roll stage (between 45deg ~ -90deg roll)
#define RECOVERY_ROTATION_RATE  40000   // rotation rate request in centi-degrees / sec (i.e. 400 deg/sec)
#define RECOVERY_TIMEOUT_MS     1500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode

// flip_init - initialise flip controller
bool ModeRecovery::init(bool ignore_checks)
{
    // 配置原模式
    desired_control_mode = copter.desired_mode;

    // initialise state
    _state = RecoveryState::Start;
    start_time_ms = millis();
    pitch_dir = -1;
    throttle_out = 1.0f;
    //设置反转方向并关闭所有电机
    motors->rc_write(4,0);
    motors->rc_write(5,0);
    motors->rc_write(6,0);
    motors->rc_write(7,0);
    return true;

}

// run - runs the flip controller
// should be called at 100hz or more
void ModeRecovery::run()
{

    if (!motors->armed() || ((millis() - start_time_ms) > RECOVERY_TIMEOUT_MS)) {
        _state = RecoveryState::Abandon;
    }
    
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    int32_t flip_angle;
    flip_angle = ahrs.pitch_sensor * pitch_dir;
    // state machine
    switch (_state) {

    case RecoveryState::Start:
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f * pitch_dir, 0.0f);;
    //暂停飞行器输出，并检测飞行器姿态
        throttle_out -= RECOVERY_THR_DEC;
        throttle_out = MAX(0.0f,throttle_out);
        // beyond 80deg lean angle move to next stage
        if (flip_angle < 4000) {
            //倾角小于40度进入恢复状态
            _state = RecoveryState::Recovery;
        }
        break;
    case RecoveryState::Recovery:
        //逐渐增加油门并改平姿态
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f * pitch_dir, 0.0f);
        throttle_out += RECOVERY_THR_INC;
        throttle_out = MIN(0.50f,throttle_out);
        float recovery_angle;
      
        recovery_angle = fabsf(0.0f - (float)ahrs.pitch_sensor);
        if (fabsf(recovery_angle) <= 1000) {
            //成功改平后恢复原飞行状态
            if (!copter.set_mode(desired_control_mode, ModeReason::UNKNOWN)) {
                copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
            }
        }
        break;
    
    case RecoveryState::Abandon:
        // restore original flight mode
        if (!copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN)) {
            // this should never happen but just in case
            copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
        }

        break;
    }
    attitude_control->set_throttle_out(throttle_out, true, g.throttle_filt);
}

#endif
