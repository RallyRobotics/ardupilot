#pragma once

#include "AP_Swivel_config.h"

#if AP_SWIVEL_PIN_ENABLED

#include "AP_Swivel_Backend.h"

class AP_Swivel_Analog : public AP_Swivel_Backend
{
public:

    AP_Swivel_Analog(AP_Swivel &_ap_swivel, AP_Swivel::Swivel_State &_state);

    void update(void) override;

private:

    AP_HAL::AnalogSource *source;
    
    int8_t volt_pin        = ap_swivel._params.volt_pin.get();
    float  volt_min        = ap_swivel._params.volt_min.get();
    float  volt_max        = ap_swivel._params.volt_max.get();
    float  volt_range      = volt_max - volt_min;
    float  volt_center     = volt_min + (volt_range / 2);
    float  volt_per_radian = volt_range / M_PI;
    float  last_angle = 0.0;
};

#endif  // AP_SWIVEL_PIN_ENABLED
