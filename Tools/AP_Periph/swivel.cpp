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

SwivelSensor::SwivelSensor(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void SwivelSensor::init()
{
    ensure_source();
}

bool SwivelSensor::ensure_source()
{
    if (_source == nullptr) {
        _source = hal.analogin->channel(ANALOG_INPUT_NONE);
        if (_source == nullptr) {
            _voltage_valid = false;
            _configured_pin = -1;
            return false;
        }
    }

    const int8_t desired_pin = _pin;
    if (_configured_pin != desired_pin) {
        if (!_source->set_pin(desired_pin)) {
            _voltage_valid = false;
            _configured_pin = -1;
            return false;
        }
        _configured_pin = desired_pin;
    }

    return true;
}

void SwivelSensor::update()
{
    if (!enabled()) {
        _voltage_valid = false;
        return;
    }

    if (!ensure_source()) {
        _voltage_valid = false;
        return;
    }

    const float voltage = _source->voltage_average();
    if (!isfinite(voltage)) {
        _voltage_valid = false;
        return;
    }

    _voltage = voltage;
    _voltage_valid = true;
}

bool SwivelSensor::get_voltage(float &voltage) const
{
    if (!_voltage_valid) {
        return false;
    }
    voltage = _voltage;
    return true;
}

void AP_Periph_FW::can_swivel_update()
{
    swivel.update();

    if (!swivel.enabled()) {
        return;
    }

    static uint32_t last_publish_ms;
    const uint32_t now = AP_HAL::millis();
    const uint32_t rate_hz = swivel.get_rate_hz();
    const uint32_t interval_ms = MAX<uint32_t>(1U, 1000U / MAX<uint32_t>(1U, rate_hz));

    if ((now - last_publish_ms) < interval_ms) {
        return;
    }

    float voltage = 0.0f;
    if (!swivel.get_voltage(voltage)) {
        return;
    }

    last_publish_ms = now;

    uavcan_equipment_actuator_Status pkt {};
    pkt.actuator_id = swivel.get_sensor_id();
    pkt.position = voltage;
    pkt.speed = NAN;
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