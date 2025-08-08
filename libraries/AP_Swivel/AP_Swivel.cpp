#include "AP_Swivel.h"

#if AP_SWIVEL_ENABLED

#include "AP_Swivel_Backend.h"
#include "AP_Swivel_Analog.h"
#include "AP_Swivel_DroneCAN.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Swivel::var_info[] = {
    // @Group: 1_
    // @Path: AP_Swivel_Params.cpp
    AP_SUBGROUPINFO(_params, "1_", 0, AP_Swivel, AP_Swivel_Params),

    AP_GROUPEND
};

AP_Swivel::AP_Swivel(void)
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Swivel must be singleton");
    }
    _singleton = this;
}

void AP_Swivel::init(void)
{
    switch (get_type()) {
        case Type::NONE:
            return;
#if AP_SWIVEL_PIN_ENABLED
        case Type::ANALOG:
            driver = new AP_Swivel_Analog(*this, state);
            break;
#endif
#if AP_SWIVEL_DRONECAN_ENABLED
        case Type::DRONECAN:
            driver = new AP_Swivel_DroneCAN(*this, state);
            break;
#endif
    }
}

void AP_Swivel::update(void)
{
    if (driver != nullptr) {
        driver->update();
    }
}

bool AP_Swivel::enabled() const
{
    return (get_type() != Type::NONE);
}

bool AP_Swivel::get_angle(float &angle_value) const
{
    if (!enabled()) {
        return false;
    }
    angle_value = state.angle;
    return true;
}

bool AP_Swivel::get_rate(float &rate_value) const
{
    if (!enabled()) {
        return false;
    }
    rate_value = state.rate;
    return true;
}

AP_Swivel *AP_Swivel::_singleton;

namespace AP {

AP_Swivel *swivel()
{
    return AP_Swivel::get_singleton();
}

}

#endif  // AP_SWIVEL_ENABLED
