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

bool AP_Swivel::timed_out(uint32_t now_ms) const
{
    const uint32_t timeout_ms = MAX((int16_t)_params.timeout_ms, (int16_t)1);
    return _state.healthy && ((now_ms - _state.last_sample_ms) > timeout_ms);
}

bool AP_Swivel::healthy() const
{
    WITH_SEMAPHORE(_sem);

    if (!enabled()) {
        return false;
    }

    if (timed_out(AP_HAL::millis())) {
        _state.healthy = false;
    }

    return _state.healthy;
}

bool AP_Swivel::get_angle(float &angle_value) const
{
    WITH_SEMAPHORE(_sem);

    if (!enabled()) {
        return false;
    }

    if (timed_out(AP_HAL::millis())) {
        _state.healthy = false;
        return false;
    }

    if (!_state.healthy) {
        return false;
    }

    angle_value = _state.angle_rad;
    return true;
}

bool AP_Swivel::get_rate(float &rate_value) const
{
    WITH_SEMAPHORE(_sem);

    if (!enabled()) {
        return false;
    }

    if (timed_out(AP_HAL::millis())) {
        _state.healthy = false;
        return false;
    }

    if (!_state.healthy) {
        return false;
    }

    rate_value = _state.rate_rad_s;
    return true;
}

bool AP_Swivel::get_raw_voltage(float &voltage) const
{
    WITH_SEMAPHORE(_sem);

    if (!enabled()) {
        return false;
    }

    if (timed_out(AP_HAL::millis())) {
        _state.healthy = false;
        _state.rate_rad_s = 0.0f;
        return false;
    }

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

float AP_Swivel::map_voltage_rate_to_angle_rate(float voltage_rate) const
{
    const float vmin = _params.volt_min;
    const float vmax = _params.volt_max;
    const float amin = radians(_params.ang_min_deg);
    const float amax = radians(_params.ang_max_deg);

    if (is_equal(vmin, vmax)) {
        return 0.0f;
    }

    float slope = (amax - amin) / (vmax - vmin);

    if (_params.reversed != 0) {
        slope = -slope;
    }

    return voltage_rate * slope;
}

void AP_Swivel::handle_feedback_sample(uint8_t sensor_id,
                                       float voltage,
                                       float voltage_rate,
                                       uint32_t now_ms)
{
    if (!enabled()) {
        return;
    }

    if (sensor_id != uint8_t(_params.sensor_id)) {
        return;
    }

    WITH_SEMAPHORE(_sem);
    _state.raw_voltage = voltage;
    _state.angle_rad = map_voltage_to_angle(voltage);
    _state.rate_rad_s = map_voltage_rate_to_angle_rate(voltage_rate);
    _state.last_sample_ms = now_ms;
    _state.healthy = true;
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

    swivel->handle_feedback_sample(msg.actuator_id,
                                   msg.position,
                                   msg.speed,
                                   AP_HAL::millis());
}

bool AP_Swivel::subscribe_msgs(AP_DroneCAN *ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_swivel_feedback, driver_index) != nullptr);
}

namespace AP {

AP_Swivel *swivel()
{
    return AP_Swivel::get_singleton();
}

}

#endif  // AP_SWIVEL_ENABLED