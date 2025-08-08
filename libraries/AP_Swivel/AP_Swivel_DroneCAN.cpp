#include "AP_Swivel_config.h"

#if AP_SWIVEL_DRONECAN_ENABLED

#include "AP_Swivel_DroneCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

AP_Swivel_DroneCAN* AP_Swivel_DroneCAN::_driver;
HAL_Semaphore AP_Swivel_DroneCAN::_driver_sem;

AP_Swivel_DroneCAN::AP_Swivel_DroneCAN(AP_Swivel &_ap_swivel, AP_Swivel::Swivel_State &_state) :
    AP_Swivel_Backend(_ap_swivel, _state)
{
    WITH_SEMAPHORE(_driver_sem);
    _driver = this;
}

void AP_Swivel_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_angle, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("swivel_sub");
    }
}

// Receive new CAN message
void AP_Swivel_DroneCAN::handle_angle(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_actuator_Status &msg)
{
    WITH_SEMAPHORE(_driver_sem);
    if (_driver == nullptr) {
        return;
    }
    _driver->last_reading_ms = AP_HAL::millis();
    _driver->angle = msg.position;
    _driver->rate = msg.speed;
}

void AP_Swivel_DroneCAN::update(void)
{
    WITH_SEMAPHORE(_driver_sem);
    state.last_reading_ms = last_reading_ms;
    state.angle = angle;
    state.rate = rate;
    if ((AP_HAL::millis() - state.last_reading_ms) > 1000) {
        state.angle = 0;
        state.rate = 0;
    }
}

#endif // AP_SWIVEL_DRONECAN_ENABLED
