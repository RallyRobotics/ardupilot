#pragma once

#include "AP_Swivel_config.h"

#if AP_SWIVEL_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_HAL/Semaphores.h>
#include "AP_Swivel_Params.h"

class AP_Swivel
{
public:
    AP_Swivel();
    CLASS_NO_COPY(AP_Swivel);

    static const struct AP_Param::GroupInfo var_info[];

    void init();

    bool enabled() const;
    bool healthy() const;

    bool get_angle(float &angle_value) const;
    bool get_rate(float &rate_value) const;
    bool get_raw_voltage(float &voltage) const;

    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static AP_Swivel *get_singleton() { return _singleton; }

private:
    struct State {
        float raw_voltage = 0.0f;
        float angle_rad = 0.0f;
        float rate_rad_s = 0.0f;
        uint32_t last_sample_ms = 0;
        bool healthy = false;
    };

    static void handle_swivel_feedback(AP_DroneCAN *ap_dronecan,
                                       const CanardRxTransfer& transfer,
                                       const uavcan_equipment_actuator_Status &msg);

    void handle_feedback_sample(uint8_t sensor_id,
                                float voltage,
                                float voltage_rate,
                                uint32_t now_ms);

    float map_voltage_to_angle(float voltage) const;
    float map_voltage_rate_to_angle_rate(float voltage_rate) const;
    bool timed_out(uint32_t now_ms) const;

    AP_Swivel_Params _params;
    mutable HAL_Semaphore _sem;
    mutable State _state {};

    static AP_Swivel *_singleton;
};

namespace AP {
    AP_Swivel *swivel();
}

#endif  // AP_SWIVEL_ENABLED