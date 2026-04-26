#include "AP_BallBay.h"

#if AP_BALLBAY_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <dronecan_msgs.h>

extern const AP_HAL::HAL& hal;

AP_BallBay *AP_BallBay::_singleton = nullptr;

const AP_Param::GroupInfo AP_BallBay::var_info[] = {
    // @Group: 1_
    // @Path: AP_BallBay_Params.cpp
    AP_SUBGROUPINFO(_params, "1_", 0, AP_BallBay, AP_BallBay_Params),

    AP_GROUPEND
};

AP_BallBay::AP_BallBay()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_BallBay must be singleton");
    }
    _singleton = this;
}

void AP_BallBay::init()
{
    // nothing to construct
}

bool AP_BallBay::enabled() const
{
    return _params.enable != 0;
}

bool AP_BallBay::timed_out(uint32_t now_ms) const
{
    const uint32_t timeout_ms = MAX((int16_t)_params.timeout_ms, (int16_t)1);
    return _state.healthy && ((now_ms - _state.last_sample_ms) > timeout_ms);
}

bool AP_BallBay::healthy() const
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

bool AP_BallBay::get_position(float &position_value) const
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

    position_value = _state.position;
    return true;
}

bool AP_BallBay::get_speed(float &speed_value) const
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

    speed_value = _state.speed;
    return true;
}

bool AP_BallBay::get_force(float &force_value) const
{
    WITH_SEMAPHORE(_sem);

    if (!enabled()) {
        return false;
    }

    if (timed_out(AP_HAL::millis())) {
        _state.healthy = false;
        _state.speed = 0.0f;
        _state.force = 0.0f;
        return false;
    }

    if (!_state.healthy) {
        return false;
    }

    force_value = _state.force;
    return true;
}

void AP_BallBay::handle_feedback_sample(uint8_t sensor_id,
                                        float position,
                                        float speed,
                                        float force,
                                        uint32_t now_ms)
{
    if (!enabled()) {
        return;
    }

    if (sensor_id != uint8_t(_params.sensor_id)) {
        return;
    }

    WITH_SEMAPHORE(_sem);
    _state.position = position;
    _state.speed = speed;
    _state.force = force;
    _state.last_sample_ms = now_ms;
    _state.healthy = true;
}

void AP_BallBay::handle_ballbay_feedback(AP_DroneCAN *ap_dronecan,
                                         const CanardRxTransfer& transfer,
                                         const uavcan_equipment_actuator_Status &msg)
{
    (void)ap_dronecan;
    (void)transfer;

    AP_BallBay *ballbay = AP::ballbay();
    if (ballbay == nullptr) {
        return;
    }

    ballbay->handle_feedback_sample(msg.actuator_id,
                                    msg.position,
                                    msg.speed,
                                    msg.force,
                                    AP_HAL::millis());
}

bool AP_BallBay::subscribe_msgs(AP_DroneCAN *ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan,
                                              &handle_ballbay_feedback,
                                              driver_index) != nullptr);
}

namespace AP {

AP_BallBay *ballbay()
{
    return AP_BallBay::get_singleton();
}

}

#endif  // AP_BALLBAY_ENABLED