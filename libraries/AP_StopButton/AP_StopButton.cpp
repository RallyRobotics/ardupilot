#include "AP_StopButton.h"

#if AP_STOPBUTTON_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Arming/AP_Arming.h>
#include <dronecan_msgs.h>

extern const AP_HAL::HAL& hal;

AP_StopButton *AP_StopButton::_singleton = nullptr;

const AP_Param::GroupInfo AP_StopButton::var_info[] = {
    // @Group: 1_
    // @Path: AP_StopButton_Params.cpp
    AP_SUBGROUPINFO(_params, "1_", 0, AP_StopButton, AP_StopButton_Params),

    AP_GROUPEND
};

AP_StopButton::AP_StopButton()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_StopButton must be singleton");
    }
    _singleton = this;
}

void AP_StopButton::init()
{
    // nothing to construct
}

bool AP_StopButton::enabled() const
{
    return _params.enable != 0;
}

bool AP_StopButton::timed_out(uint32_t now_ms) const
{
    const int16_t param_timeout_ms = _params.timeout_ms;
    const uint32_t timeout_ms = param_timeout_ms > 0 ? uint32_t(param_timeout_ms) : 1U;

    return _state.healthy && ((now_ms - _state.last_sample_ms) > timeout_ms);
}

bool AP_StopButton::healthy() const
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

bool AP_StopButton::get_engaged(bool &is_engaged) const
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

    is_engaged = _state.engaged;
    return true;
}

void AP_StopButton::handle_status_sample(uint8_t sensor_id,
                                         float position,
                                         uint32_t now_ms)
{
    if (!enabled()) {
        return;
    }

    if (sensor_id != uint8_t(_params.sensor_id)) {
        return;
    }

    bool stop_engaged = false;

    {
        WITH_SEMAPHORE(_sem);

        _state.engaged = position >= 0.5f;
        _state.last_sample_ms = now_ms;
        _state.healthy = true;

        stop_engaged = _state.engaged;
    }

    if (!stop_engaged) {
        return;
    }

    if (!AP::arming().is_armed()) {
        return;
    }

    AP::arming().disarm(AP_Arming::Method::AUXSWITCH);
}

void AP_StopButton::handle_stopbutton_status(AP_DroneCAN *ap_dronecan,
                                             const CanardRxTransfer& transfer,
                                             const uavcan_equipment_actuator_Status &msg)
{
    (void)ap_dronecan;
    (void)transfer;

    AP_StopButton *stopbutton = AP::stopbutton();
    if (stopbutton == nullptr) {
        return;
    }

    stopbutton->handle_status_sample(msg.actuator_id,
                                     msg.position,
                                     AP_HAL::millis());
}

bool AP_StopButton::subscribe_msgs(AP_DroneCAN *ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan,
                                              &handle_stopbutton_status,
                                              driver_index) != nullptr);
}

namespace AP {

AP_StopButton *stopbutton()
{
    return AP_StopButton::get_singleton();
}

}

#endif  // AP_STOPBUTTON_ENABLED