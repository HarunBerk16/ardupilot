#include "mode.h"
#include "Plane.h"

#if MODE_OFFBOARD_ENABLED

bool ModeOffboard::_enter()
{
    IGNORE_RETURN(ahrs.get_quaternion(target_q));
    rate_ff_rads.zero();
    throttle_pct = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    last_cmd_ms = AP_HAL::millis();
    timed_out = false;
    fs_action_done = false;
    return true;
}

/*
  set targets from an external controller. A nullptr means the field
  was not supplied and the previous value is kept
 */
void ModeOffboard::set_target(const Quaternion *q, const Vector3f *rate_ff, const float *thr)
{
    if (q != nullptr) {
        if (q->is_nan() || is_zero(q->length_squared())) {
            return;
        }
        target_q = *q;
        target_q.normalize();
    }
    if (rate_ff != nullptr) {
        if (rate_ff->is_nan()) {
            return;
        }
        rate_ff_rads = *rate_ff;
    }
    if (thr != nullptr && !isnan(*thr)) {
        throttle_pct = constrain_float(*thr, 0, 100);
    }

    last_cmd_ms = AP_HAL::millis();
    if (timed_out) {
        timed_out = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Offboard: control resumed");
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: OFFB
    // @Description: Offboard mode external targets
    // @Field: TimeUS: Time since system startup
    // @Field: Roll: target roll
    // @Field: Pitch: target pitch
    // @Field: Yaw: target yaw
    // @Field: RR: feedforward body roll rate
    // @Field: PR: feedforward body pitch rate
    // @Field: YR: feedforward body yaw rate
    // @Field: Thr: target throttle
    AP::logger().WriteStreaming("OFFB", "TimeUS,Roll,Pitch,Yaw,RR,PR,YR,Thr",
                                "sdddkkk%",
                                "F0000000",
                                "Qfffffff",
                                AP_HAL::micros64(),
                                degrees(target_q.get_euler_roll()),
                                degrees(target_q.get_euler_pitch()),
                                degrees(target_q.get_euler_yaw()),
                                degrees(rate_ff_rads.x),
                                degrees(rate_ff_rads.y),
                                degrees(rate_ff_rads.z),
                                throttle_pct);
#endif
}

void ModeOffboard::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t cmd_age_ms = now_ms - last_cmd_ms;

    if (!timed_out && cmd_age_ms > uint32_t(plane.g2.offboard_timeout * 1000)) {
        // lost external control, hold wings level at a gentle pitch
        timed_out = true;
        const float pitch = constrain_float(ahrs.get_pitch_rad(), radians(-10), radians(10));
        target_q.from_euler(0, pitch, ahrs.get_yaw_rad());
        rate_ff_rads.zero();
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Offboard: command timeout, wings level");
    }

    if (timed_out && !fs_action_done &&
        cmd_age_ms > uint32_t((plane.g2.offboard_timeout + plane.g2.offboard_fs_time) * 1000)) {
        fs_action_done = true;
        switch (plane.g2.offboard_fs_action) {
        case 1:
            plane.set_mode(plane.mode_rtl, ModeReason::GCS_FAILSAFE);
            return;
        case 2:
            plane.set_mode(plane.mode_loiter, ModeReason::GCS_FAILSAFE);
            return;
        default:
            break;
        }
    }

    // fill in target roll/pitch for GCS/logs
    plane.nav_roll_cd = degrees(target_q.get_euler_roll()) * 100;
    plane.nav_pitch_cd = degrees(target_q.get_euler_pitch()) * 100;
}

/*
  quaternion attitude controller, based on ModeAcro::stabilize_quaternion()
  with the target supplied externally
 */
void ModeOffboard::run()
{
    const float speed_scaler = plane.get_speed_scaler();

    // propagate target with feedforward rates so it stays smooth between messages
    Vector3f r = rate_ff_rads * plane.G_Dt;
    target_q.rotate_fast(r);
    target_q.normalize();

    Quaternion ahrs_q;
    IGNORE_RETURN(ahrs.get_quaternion(ahrs_q));

    // cope with sitting on the ground with zero throttle
    if (!plane.is_flying() && is_zero(throttle_pct)) {
        target_q = ahrs_q;
    }

    // attitude error in body frame
    const Quaternion error_quat = ahrs_q.inverse() * target_q;
    Vector3f error_rad;
    error_quat.to_axis_angle(error_rad);

    // desired body rates in deg/s
    Vector3f desired_rates {
        degrees(error_rad.x / plane.rollController.tau() + rate_ff_rads.x),
        degrees(error_rad.y / plane.pitchController.tau() + rate_ff_rads.y),
        degrees(error_rad.z / plane.pitchController.tau() + rate_ff_rads.z)  // no yaw tau parameter, use pitch
    };
    desired_rates.x = constrain_float(desired_rates.x, -plane.g.acro_roll_rate, plane.g.acro_roll_rate);
    desired_rates.y = constrain_float(desired_rates.y, -plane.g.acro_pitch_rate, plane.g.acro_pitch_rate);

    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron,  plane.rollController.get_rate_out(desired_rates.x, speed_scaler));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitchController.get_rate_out(desired_rates.y, speed_scaler));

    float rudder_output;
    if (plane.g.acro_yaw_rate > 0 && plane.yawController.rate_control_enabled()) {
        desired_rates.z = constrain_float(desired_rates.z, -plane.g.acro_yaw_rate, plane.g.acro_yaw_rate);
        rudder_output = plane.yawController.get_rate_out(desired_rates.z, speed_scaler, false);
    } else {
        // no yaw rate controller, use turn coordination
        rudder_output = plane.calc_nav_yaw_coordinated();
    }
    output_rudder_and_steering(rudder_output);

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle_pct);
}

#endif // MODE_OFFBOARD_ENABLED
