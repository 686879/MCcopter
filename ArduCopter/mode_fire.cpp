#include "Copter.h"

#if MODE_FIRE_ENABLED == ENABLED
#define UART_NUM 3
/*
 * mode_follow.cpp - follow another mavlink-enabled vehicle by system id
 *
 * TODO: stick control to move around on sphere
 * TODO: stick control to change sphere diameter
 * TODO: "channel 7 option" to lock onto "pointed at" target
 * TODO: do better in terms of loitering around the moving point; may need a PID?  Maybe use loiter controller somehow?
 * TODO: extrapolate target vehicle position using its velocity and acceleration
 * TODO: ensure AC_AVOID_ENABLED is true because we rely on it velocity limiting functions
 */

// initialise follow mode
bool ModeFire::init(const bool ignore_checks)
{
    hal.serial(UART_NUM)->begin(115200);

    // re-use guided mode
    return ModeGuided::init(ignore_checks);
}

// perform cleanup required when leaving follow mode
void ModeFire::exit()
{
    g2.follow.clear_offsets_if_required();
}

void ModeFire::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode

    // variables to be sent to velocity controller
    Vector3f desired_velocity_neu_cms;
    bool use_yaw = false;
    float yaw_cd = 0.0f;

    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    Vector3f vel_of_target;  // velocity of lead vehicle
    if (get_target_uart(dist_vec, dist_vec_offs, vel_of_target,UART_NUM)) {
        // convert dist_vec_offs to cm in NEU
        const Vector3f dist_vec_offs_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);

        // calculate desired velocity vector in cm/s in NEU
        const float kp = g2.follow.get_pos_p().kP();
        desired_velocity_neu_cms.x = (vel_of_target.x * 100.0f) + (dist_vec_offs_neu.x * kp);
        desired_velocity_neu_cms.y = (vel_of_target.y * 100.0f) + (dist_vec_offs_neu.y * kp);
        desired_velocity_neu_cms.z = (-vel_of_target.z * 100.0f) + (dist_vec_offs_neu.z * kp);

        // scale desired velocity to stay within horizontal speed limit
        float desired_speed_xy = safe_sqrt(sq(desired_velocity_neu_cms.x) + sq(desired_velocity_neu_cms.y));
        if (!is_zero(desired_speed_xy) && (desired_speed_xy > pos_control->get_max_speed_xy_cms())) {
            const float scalar_xy = pos_control->get_max_speed_xy_cms() / desired_speed_xy;
            desired_velocity_neu_cms.x *= scalar_xy;
            desired_velocity_neu_cms.y *= scalar_xy;
            desired_speed_xy = pos_control->get_max_speed_xy_cms();
        }

        // limit desired velocity to be between maximum climb and descent rates
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());

        // unit vector towards target position (i.e. vector to lead vehicle + offset)
        Vector3f dir_to_target_neu = dist_vec_offs_neu;
        const float dir_to_target_neu_len = dir_to_target_neu.length();
        if (!is_zero(dir_to_target_neu_len)) {
            dir_to_target_neu /= dir_to_target_neu_len;
        }

        // create horizontal desired velocity vector (required for slow down calculations)
        Vector2f desired_velocity_xy_cms(desired_velocity_neu_cms.x, desired_velocity_neu_cms.y);

        // create horizontal unit vector towards target (required for slow down calculations)
        Vector2f dir_to_target_xy(desired_velocity_xy_cms.x, desired_velocity_xy_cms.y);
        if (!dir_to_target_xy.is_zero()) {
            dir_to_target_xy.normalize();
        }

        // slow down horizontally as we approach target (use 1/2 of maximum deceleration for gentle slow down)
        const float dist_to_target_xy = Vector2f(dist_vec_offs_neu.x, dist_vec_offs_neu.y).length();
        copter.avoid.limit_velocity_2D(pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss() * 0.5f, desired_velocity_xy_cms, dir_to_target_xy, dist_to_target_xy, copter.G_Dt);
        // copy horizontal velocity limits back to 3d vector
        desired_velocity_neu_cms.x = desired_velocity_xy_cms.x;
        desired_velocity_neu_cms.y = desired_velocity_xy_cms.y;

        // limit vertical desired_velocity_neu_cms to slow as we approach target (we use 1/2 of maximum deceleration for gentle slow down)
        const float des_vel_z_max = copter.avoid.get_max_speed(pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss() * 0.5f, fabsf(dist_vec_offs_neu.z), copter.G_Dt);
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -des_vel_z_max, des_vel_z_max);

        // limit the velocity for obstacle/fence avoidance
        copter.avoid.adjust_velocity(desired_velocity_neu_cms, pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss(), G_Dt);

        // calculate vehicle heading
       
    }

    // log output at 10hz
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0)) {
        log_request = true;
        last_log_ms = now;
    }
    // re-use guided mode's velocity controller (takes NEU)
    ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);

    ModeGuided::run();
}
bool ModeFire::get_target_uart(Vector3f &dist_ned, Vector3f &dist_with_offs, Vector3f &vel_ned,uint16_t uart)
{
    uint8_t buf[3];
    if ((hal.serial(UART_NUM)->read(buf,3))==-1)
    {
        return false;
    }
    dist_ned = Vector3f(buf[0]/100.f,buf[1]/100.f,buf[2]/100.f);
    dist_with_offs = Vector3f(buf[0]/100.f,buf[1]/100.f,buf[2]/100.f);
    vel_ned = Vector3f(buf[0]/100.f,buf[1]/100.f,buf[2]/100.f);
    return true;
}

#endif // MODE_FIRE_ENABLED == ENABLED
