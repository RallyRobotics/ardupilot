#include "AP_Swivel.h"

#if AP_SWIVEL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <dronecan_msgs.h>

extern const AP_HAL::HAL& hal;

AP_Swivel *AP_Swivel::_singleton = nullptr;

const AP_Param::GroupInfo AP_Swivel::var_info[] = {
    // @Group: 1_
    // @Path: AP_Swivel_Params.cpp
    AP_SUBGROUPINFO(_params, "1_", 0, AP_Swivel, AP_Swivel_Params),

    AP_GROUPEND
};

AP_Swivel::AP_Swivel()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Swivel must be singleton");
    }
    _singleton = this;
}

void AP_Swivel::init()
{
    // nothing to construct
}

bool AP_Swivel::enabled() const
{
    return _params.enable != 0;
}

bool AP_Swivel::healthy() const
{
    WITH_SEMAPHORE(_sem);
    return _state.healthy;
}

bool AP_Swivel::get_angle(float &angle_value) const
{
    WITH_SEMAPHORE(_sem);

    if (!_state.healthy) {
        return false;
    }

    angle_value = _state.angle_rad;
    return true;
}

bool AP_Swivel::get_rate(float &rate_value) const
{
    WITH_SEMAPHORE(_sem);

    if (!_state.healthy) {
        return false;
    }

    rate_value = _state.rate_rad_s;
    return true;
}

bool AP_Swivel::get_raw_voltage(float &voltage) const
{
    WITH_SEMAPHORE(_sem);

    if (!_state.healthy) {
        return false;
    }

    voltage = _state.raw_voltage;
    return true;
}

float AP_Swivel::map_voltage_to_angle(float voltage) const
{
    const float vmin = _params.volt_min;
    const float vmax = _params.volt_max;
    const float amin = radians(_params.ang_min_deg);
    const float amax = radians(_params.ang_max_deg);

    if (is_equal(vmin, vmax)) {
        return 0.0f;
    }

    float t = (voltage - vmin) / (vmax - vmin);
    t = constrain_float(t, 0.0f, 1.0f);

    float angle = amin + t * (amax - amin);

    if (_params.reversed != 0) {
        angle = amin + amax - angle;
    }

    return angle;
}

void AP_Swivel::handle_voltage_sample(uint8_t sensor_id, float voltage, uint32_t now_ms)
{
    if (!enabled()) {
        return;
    }
    if (sensor_id != uint8_t(_params.sensor_id)) {
        return;
    }
    if (!isfinite(voltage)) {
        return;
    }

    // callback only caches the latest raw sample
    WITH_SEMAPHORE(_sem);
    _rx.raw_voltage = voltage;
    _rx.sample_ms = now_ms;
    _rx.new_sample = true;
}

void AP_Swivel::update()
{
    if (!enabled()) {
        WITH_SEMAPHORE(_sem);
        _rx.new_sample = false;
        _state.healthy = false;
        _state.rate_rad_s = 0.0f;
        return;
    }

    float sample_voltage = 0.0f;
    uint32_t sample_ms = 0;
    float prev_angle = 0.0f;
    uint32_t prev_ms = 0;
    bool have_new_sample = false;

    {
        WITH_SEMAPHORE(_sem);

        if (_rx.new_sample) {
            sample_voltage = _rx.raw_voltage;
            sample_ms = _rx.sample_ms;
            prev_angle = _state.angle_rad;
            prev_ms = _state.last_sample_ms;
            _rx.new_sample = false;
            have_new_sample = true;
        }
    }

    if (have_new_sample) {
        const float new_angle = map_voltage_to_angle(sample_voltage);

        float new_rate = 0.0f;
        if (prev_ms != 0 && sample_ms > prev_ms) {
            const float dt = (sample_ms - prev_ms) * 0.001f;
            if (dt > 0.0f) {
                new_rate = (new_angle - prev_angle) / dt;
            }
        }

        WITH_SEMAPHORE(_sem);
        _state.raw_voltage = sample_voltage;
        _state.angle_rad = new_angle;
        _state.rate_rad_s = new_rate;
        _state.last_sample_ms = sample_ms;
        _state.healthy = true;
    }

    const uint32_t now = AP_HAL::millis();
    const uint32_t timeout_ms = MAX((int16_t)_params.timeout_ms, (int16_t)1);

    WITH_SEMAPHORE(_sem);
    if (_state.healthy && ((now - _state.last_sample_ms) > timeout_ms)) {
        _state.healthy = false;
        _state.rate_rad_s = 0.0f;
    }
}

void AP_Swivel::handle_swivel_feedback(AP_DroneCAN *ap_dronecan,
                                       const CanardRxTransfer& transfer,
                                       const uavcan_equipment_actuator_Status &msg)
{
    (void)ap_dronecan;
    (void)transfer;

    AP_Swivel *swivel = AP::swivel();
    if (swivel == nullptr) {
        return;
    }

    // convention for now:
    // actuator_id = source ID
    // position    = raw voltage
    // speed       = unused
    swivel->handle_voltage_sample(msg.actuator_id, msg.position, AP_HAL::millis());
}

void AP_Swivel::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan,
                                          &handle_swivel_feedback,
                                          ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("swivel_sub");
    }
}

namespace AP {

AP_Swivel *swivel()
{
    return AP_Swivel::get_singleton();
}

}

#endif  // AP_SWIVEL_ENABLED