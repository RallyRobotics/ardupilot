#include "AP_Periph.h"

#if AP_PERIPH_SWIVEL_ENABLED

#include <dronecan_msgs.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo SwivelSensor::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Swivel analog feedback enable
    // @Description: Enable raw swivel analog feedback publishing over CAN
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, SwivelSensor, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PIN
    // @DisplayName: Swivel analog pin
    // @Description: Analog pin used for raw swivel feedback voltage
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO("PIN", 2, SwivelSensor, _pin, 1),

    // @Param: ID
    // @DisplayName: Swivel source ID
    // @Description: Source ID placed in actuator_id when publishing swivel raw voltage
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("ID", 3, SwivelSensor, _id, 1),

    // @Param: RATE
    // @DisplayName: Swivel publish rate
    // @Description: Publish rate for raw swivel voltage feedback
    // @Range: 1 200
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("RATE", 4, SwivelSensor, _rate_hz, 50),

    AP_GROUPEND
};

SwivelSensor::SwivelSensor()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void SwivelSensor::reset_state()
{
    _last_voltage = 0.0f;
    _last_sample_us = 0;
    _have_sample = false;
}

void SwivelSensor::init()
{
    _source = hal.analogin->channel(ANALOG_INPUT_NONE);
    if (_source == nullptr) {
        reset_state();
        return;
    }

    if (!_source->set_pin(_pin)) {
        _source = nullptr;
        reset_state();
        return;
    }

    reset_state();
}

bool SwivelSensor::sample(uint32_t now_us, float &voltage, float &rate_vps)
{
    if (!enabled() || _source == nullptr) {
        reset_state();
        return false;
    }

    const float new_voltage = _source->voltage_average();
    if (!isfinite(new_voltage)) {
        reset_state();
        return false;
    }

    float new_rate_vps = 0.0f;

    if (_have_sample) {
        // Unsigned subtraction handles normal uint32_t micros() rollover.
        const uint32_t dt_us = now_us - _last_sample_us;

        if (dt_us > 0) {
            const float dt = dt_us * 1.0e-6f;
            new_rate_vps = (new_voltage - _last_voltage) / dt;
        }
    }

    _last_voltage = new_voltage;
    _last_sample_us = now_us;
    _have_sample = true;

    voltage = new_voltage;
    rate_vps = new_rate_vps;

    return true;
}

void AP_Periph_FW::can_swivel_update()
{
    static uint32_t last_publish_ms;

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t rate_hz = swivel.get_rate_hz();

    const uint32_t safe_rate_hz = MAX<uint32_t>(1U, rate_hz);
    const uint32_t interval_ms = MAX<uint32_t>(1U, 1000U / safe_rate_hz);

    // Important:
    // Do not sample until it is actually time to publish.
    // Otherwise the voltage delta is measured over the main loop period instead
    // of the CAN publish period.
    if ((now_ms - last_publish_ms) < interval_ms) {
        return;
    }

    const uint32_t now_us = AP_HAL::micros();

    float voltage = 0.0f;
    float rate_vps = 0.0f;

    if (!swivel.sample(now_us, voltage, rate_vps)) {
        return;
    }

    last_publish_ms = now_ms;

    uavcan_equipment_actuator_Status pkt {};
    pkt.actuator_id = swivel.get_sensor_id();
    pkt.position = voltage;   // raw volts
    pkt.speed = rate_vps;     // volts/sec
    pkt.force = NAN;
    pkt.power_rating_pct = UAVCAN_EQUIPMENT_ACTUATOR_STATUS_POWER_RATING_PCT_UNKNOWN;

    uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE] {};
    const uint16_t total_size = uavcan_equipment_actuator_Status_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                     UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     &buffer[0],
                     total_size);
}

#endif // AP_PERIPH_SWIVEL_ENABLED