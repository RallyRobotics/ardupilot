#include "AP_Periph.h"

#if AP_PERIPH_STOPBUTTON_ENABLED

#include <dronecan_msgs.h>

extern const AP_HAL::HAL& hal;

#ifndef HAL_STOP_BUTTON_ON
#define HAL_STOP_BUTTON_ON 1
#endif

const AP_Param::GroupInfo StopButton::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Stop button enable
    // @Description: Enable STOP button state publishing over CAN
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, StopButton, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ID
    // @DisplayName: Stop button source ID
    // @Description: Source ID placed in actuator_id when publishing STOP button state
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("ID", 2, StopButton, _id, 9),

    // @Param: RATE
    // @DisplayName: Stop button publish rate
    // @Description: Publish rate for STOP button state
    // @Range: 1 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("RATE", 3, StopButton, _rate_hz, 10),

    AP_GROUPEND
};

StopButton::StopButton()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void StopButton::init()
{
    // Nothing to construct. GPIO comes from hwdef.
}

bool StopButton::sample(bool &engaged)
{
    if (!enabled()) {
        return false;
    }

#ifndef HAL_GPIO_PIN_STOP_BUTTON
    return false;
#else
    engaged = (palReadLine(HAL_GPIO_PIN_STOP_BUTTON) == HAL_STOP_BUTTON_ON);
    return true;
#endif
}

void AP_Periph_FW::can_stopbutton_update()
{
    bool engaged = false;
    if (!stopbutton.sample(engaged)) {
        return;
    }

    static uint32_t last_publish_ms;
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t rate_hz = stopbutton.get_rate_hz();
    const uint32_t interval_ms = MAX<uint32_t>(1U, 1000U / MAX<uint32_t>(1U, rate_hz));

    if ((now_ms - last_publish_ms) < interval_ms) {
        return;
    }
    last_publish_ms = now_ms;

    uavcan_equipment_actuator_Status pkt {};
    pkt.actuator_id = stopbutton.get_sensor_id();

    // Custom convention:
    //   position = 0.0f -> STOP disengaged / normal
    //   position = 1.0f -> STOP engaged
    pkt.position = engaged ? 1.0f : 0.0f;

    pkt.speed = 0.0f;
    pkt.force = NAN;
    pkt.power_rating_pct = UAVCAN_EQUIPMENT_ACTUATOR_STATUS_POWER_RATING_PCT_UNKNOWN;

    uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE] {};
    const uint16_t total_size = uavcan_equipment_actuator_Status_encode(&pkt,
                                                                        buffer,
                                                                        !periph.canfdout());

    canard_broadcast(UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                     UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     &buffer[0],
                     total_size);
}

#endif // AP_PERIPH_STOPBUTTON_ENABLED