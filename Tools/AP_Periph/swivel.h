#pragma once

#if AP_PERIPH_SWIVEL_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>

class SwivelSensor {
public:
    SwivelSensor(void);

    void init();
    void update();
    bool enabled() const { return _enable != 0; }
    bool get_voltage(float &voltage) const;
    uint8_t get_sensor_id() const { return MAX((int8_t)0, _id.get()); }
    uint16_t get_rate_hz() const { return MAX((int16_t)1, _rate_hz.get()); }

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8  _enable;
    AP_Int8  _pin;
    AP_Int8  _id;
    AP_Int16 _rate_hz;
    AP_HAL::AnalogSource *_source = nullptr;
    
    bool ensure_source();
    int8_t _configured_pin = -1;
    float _voltage = 0.0f;
    bool _voltage_valid = false;
};

#endif // AP_PERIPH_SWIVEL_ENABLED