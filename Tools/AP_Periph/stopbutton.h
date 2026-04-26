#pragma once

#if AP_PERIPH_STOPBUTTON_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>

class StopButton {
public:
    StopButton();

    void init();
    bool enabled() const { return _enable != 0; }

    // Sample STOP state.
    // Returns true when enabled and a valid GPIO is configured.
    bool sample(bool &engaged);

    uint8_t get_sensor_id() const { return MAX((int8_t)0, _id.get()); }
    uint16_t get_rate_hz() const { return MAX((int16_t)1, _rate_hz.get()); }

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8  _enable;
    AP_Int8  _id;
    AP_Int16 _rate_hz;
};

#endif // AP_PERIPH_STOPBUTTON_ENABLED