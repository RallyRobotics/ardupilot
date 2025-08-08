#include "AP_Swivel_config.h"

#if AP_SWIVEL_PIN_ENABLED

#include "AP_Swivel_Analog.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_Swivel_Analog::AP_Swivel_Analog(AP_Swivel &_ap_swivel, AP_Swivel::Swivel_State &_state) :
    AP_Swivel_Backend(_ap_swivel, _state)
{
    source = hal.analogin->channel(ANALOG_INPUT_NONE);
}

void AP_Swivel_Analog::update(void)
{
    if (!source || !source->set_pin(volt_pin)) {
        state.angle = 0;
        state.rate = 0;
    }
    uint32_t current_time = AP_HAL::millis();
    float voltage = source->voltage_average();
    state.angle = (voltage - volt_center) / volt_per_radian;
    state.rate = (state.angle - last_angle) / (current_time - state.last_reading_ms) * 1000;
    state.last_reading_ms = current_time;
    last_angle = state.angle;
}

#endif  // AP_SWIVEL_PIN_ENABLED
