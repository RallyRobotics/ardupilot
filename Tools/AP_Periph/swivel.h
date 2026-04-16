#pragma once

#if AP_PERIPH_SWIVEL_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>

class SwivelSensor {
public:
    SwivelSensor();

    void init();
    bool enabled() const { return _enable != 0; }

    // Sample raw voltage and voltage rate (volts/sec)
    bool sample(uint32_t now_us, float &voltage, float &rate_vps);

    uint8_t get_sensor_id() const { return MAX((int8_t)0, _id.get()); }
    uint16_t get_rate_hz() const { return MAX((int16_t)1, _rate_hz.get()); }

    static const struct AP_Param::GroupInfo var_info[];

private:
    void reset_state();

    AP_Int8  _enable;
    AP_Int8  _pin;
    AP_Int8  _id;
    AP_Int16 _rate_hz;

    AP_HAL::AnalogSource *_source = nullptr;

    float _last_voltage = 0.0f;
    uint32_t _last_sample_us = 0;
    bool _have_sample = false;
};

#endif // AP_PERIPH_SWIVEL_ENABLED