#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_BALLBAY

/*
  ballbay support
 */

#include <dronecan_msgs.h>

void AP_Periph_FW::can_ballbay_update()
{
    if (ballbay.get_type() == AP_BallBay::Type::NONE) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    static uint32_t last_update_ms;
    if (now - last_update_ms < 20) {
        return;
    }
    last_update_ms = now;

    ballbay.update();

    uavcan_equipment_actuator_Status pkt {};

    pkt.actuator_id = uint8_t(ballbay._params.actuator_id);
    pkt.position = ballbay.get_report_position();
    pkt.force = ballbay.get_loadcell_filtered_sum();
    pkt.speed = ballbay.get_report_speed();
    pkt.power_rating_pct = UAVCAN_EQUIPMENT_ACTUATOR_STATUS_POWER_RATING_PCT_UNKNOWN;

    uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE] {};
    uint16_t total_size = uavcan_equipment_actuator_Status_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                     UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     &buffer[0],
                     total_size);
}

void AP_Periph_FW::ballbay_srv_unitless(uint8_t actuator_id, const float command_value)
{
    if (actuator_id != uint8_t(ballbay._params.actuator_id)) {
        return;
    }
    ballbay.handle_unitless_command(command_value);
}

#endif // HAL_PERIPH_ENABLE_BALLBAY