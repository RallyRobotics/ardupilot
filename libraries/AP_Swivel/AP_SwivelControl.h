#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AP_Swivel/AP_Swivel.h>

// timeout
#define AP_SWIVEL_TIMEOUT_MS         200

// swivel torque vectoring defaults
#define AP_SWIVEL_WHEELBASE          0.775f
#define AP_SWIVEL_TRACKWIDTH         0.40f

// swivel position control defaults
#define AP_SWIVEL_POS_P              1.00f
#define AP_SWIVEL_POS_I              2.00f
#define AP_SWIVEL_POS_D              0.00f
#define AP_SWIVEL_POS_FF             0.00f
#define AP_SWIVEL_POS_IMAX           1.00f
#define AP_SWIVEL_POS_FILT           0.00f
#define AP_SWIVEL_POS_DT             0.00f

// swivel rate control defaults
#define AP_SWIVEL_PWM_MAX            0.50f
#define AP_SWIVEL_RATE_MAX           90.0f
#define AP_SWIVEL_RATE_P             2.00f
#define AP_SWIVEL_RATE_I             2.00f
#define AP_SWIVEL_RATE_D             0.00f
#define AP_SWIVEL_RATE_FF            6.00f
#define AP_SWIVEL_RATE_IMAX          1.00f
#define AP_SWIVEL_RATE_FILT          0.00f
#define AP_SWIVEL_RATE_DT            0.00f

class AP_SwivelControl {

public:

    AP_SwivelControl(const AP_Swivel &swivel_ref);

    // Returns true if SwivelControl is enabled
    bool enabled();

    // Returns the steering output required to achieve desired Swivel angle
    float get_swivel_position_correction(float target, float &throttle, float dt);

    // get rate maximum in radians/sec
    float get_rate_max_rads() const { return MAX(_rate_max, 0.0f); }

    // get limit flag
    bool is_limited() const { return _error_limit || _rate_limit || _pwm_limit; }

    // get pid object for reporting
    AC_PID& get_pos_pid();
    AC_PID& get_rate_pid();

    // set the PID notch sample rate
    void set_notch_sample_rate(float sample_rate);

    static const struct AP_Param::GroupInfo        var_info[];

private:

    // parameters
    AP_Int8         _enabled;     // Swivel rate control enable/disable
    AP_Float        _wheelbase;   // Distance between Swivel and vehicle's center of rotation
    AP_Float        _trackwidth;  // Distance between Swivel's wheels
    AP_Float        _error_max;   // Angle error threshold to set limit flag
    AP_Float        _pwm_max;     // Swivel max steering output allowed for correcting angle
    AP_Float        _rate_max;    // Swivel max rotation rate
    AC_PID          _pos_pid{AP_SWIVEL_POS_P, AP_SWIVEL_POS_I, AP_SWIVEL_POS_D, AP_SWIVEL_POS_FF, AP_SWIVEL_POS_IMAX, AP_SWIVEL_POS_FILT, AP_SWIVEL_POS_FILT, AP_SWIVEL_POS_FILT, AP_SWIVEL_POS_DT};
    AC_PID          _rate_pid{AP_SWIVEL_RATE_P, AP_SWIVEL_RATE_I, AP_SWIVEL_RATE_D, AP_SWIVEL_RATE_FF, AP_SWIVEL_RATE_IMAX, AP_SWIVEL_RATE_FILT, AP_SWIVEL_RATE_FILT, AP_SWIVEL_RATE_FILT, AP_SWIVEL_RATE_DT};

    // limit flags
    bool _error_limit, _rate_limit, _pwm_limit;

    // internal variables
    const AP_Swivel&        _swivel;     // pointer to accompanying swivel
    uint32_t                _last_update_ms;    // system time of last call to get_rate_controlled_throttle
};
