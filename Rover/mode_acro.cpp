#include "Rover.h"

void ModeAcro::update()
{
    // get speed forward
    float speed, desired_steering, desired_throttle, desired_speed = 0.0;
    if (!attitude_control.get_forward_speed(speed)) {
        // convert pilot stick input into desired steering and throttle
        get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);

        // if vehicle is balance bot, calculate actual throttle required for balancing
        if (rover.is_balancebot()) {
            rover.balancebot_pitch_control(desired_throttle);
        }

        // no valid speed, just use the provided throttle
        g2.motors.set_throttle(desired_throttle);
    } else {
        // convert pilot stick input into desired steering and speed
        get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
        calc_throttle(desired_speed, true);
    }

    float steering_out;

    // handle sailboats
    if (!is_zero(desired_steering)) {
        // steering input return control to user
        g2.sailboat.clear_tack();
    }
    if (g2.sailboat.tacking()) {
        // call heading controller during tacking

        steering_out = attitude_control.get_steering_out_heading(g2.sailboat.get_tack_heading_rad(),
                                                                 g2.wp_nav.get_pivot_rate(),
                                                                 g2.motors.limit.steer_left,
                                                                 g2.motors.limit.steer_right,
                                                                 rover.G_Dt);
    } else if (rover.g2.motors.have_swivel_steering() && is_zero(desired_throttle) && is_zero(desired_speed) && is_zero(desired_steering)) {
        // no steering rate correction when both desired rate and throttle are 0
        steering_out = 0.0;
    } else {
        // convert pilot steering input to desired turn rate in radians/sec
        const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);

        // run steering turn rate controller and throttle controller
        steering_out = attitude_control.get_steering_out_rate(target_turn_rate,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);
    }

    set_steering(steering_out * 4500.0f);
}

bool ModeAcro::requires_velocity() const
{
    return !g2.motors.have_skid_steering();
}

// sailboats in acro mode support user manually initiating tacking from transmitter
void ModeAcro::handle_tack_request()
{
    g2.sailboat.handle_tack_request_acro();
}
