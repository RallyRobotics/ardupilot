#pragma once

#include "AP_StopButton_config.h"

#if AP_STOPBUTTON_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_HAL/Semaphores.h>
#include "AP_StopButton_Params.h"

class AP_StopButton
{
public:
    AP_StopButton();
    CLASS_NO_COPY(AP_StopButton);

    static const struct AP_Param::GroupInfo var_info[];

    void init();

    bool enabled() const;
    bool healthy() const;

    bool get_engaged(bool &is_engaged) const;

    static bool subscribe_msgs(AP_DroneCAN *ap_dronecan);
    static AP_StopButton *get_singleton() { return _singleton; }

private:
    struct State {
        bool engaged = false;
        bool healthy = false;
        uint32_t last_sample_ms = 0;
    };

    static void handle_stopbutton_status(AP_DroneCAN *ap_dronecan,
                                         const CanardRxTransfer& transfer,
                                         const uavcan_equipment_actuator_Status &msg);

    void handle_status_sample(uint8_t sensor_id,
                              float position,
                              uint32_t now_ms);

    bool timed_out(uint32_t now_ms) const;

    AP_StopButton_Params _params;
    mutable HAL_Semaphore _sem;
    mutable State _state {};

    static AP_StopButton *_singleton;
};

namespace AP {
    AP_StopButton *stopbutton();
}

#endif  // AP_STOPBUTTON_ENABLED