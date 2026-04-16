#pragma once

#include "AP_BallBay_config.h"

#if AP_BALLBAY_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_HAL/Semaphores.h>
#include "AP_BallBay_Params.h"

class AP_BallBay
{
public:
    AP_BallBay();
    CLASS_NO_COPY(AP_BallBay);

    static const struct AP_Param::GroupInfo var_info[];

    void init();

    bool enabled() const;
    bool healthy() const;

    bool get_position(float &position_value) const;
    bool get_speed(float &speed_value) const;
    bool get_force(float &force_value) const;

    static bool subscribe_msgs(AP_DroneCAN *ap_dronecan);
    static AP_BallBay *get_singleton() { return _singleton; }

private:
    struct State {
        float position = 0.0f;
        float speed = 0.0f;
        float force = 0.0f;
        uint32_t last_sample_ms = 0;
        bool healthy = false;
    };

    static void handle_ballbay_feedback(AP_DroneCAN *ap_dronecan,
                                        const CanardRxTransfer& transfer,
                                        const uavcan_equipment_actuator_Status &msg);

    void handle_feedback_sample(uint8_t sensor_id,
                                float position,
                                float speed,
                                float force,
                                uint32_t now_ms);

    bool timed_out(uint32_t now_ms) const;

    AP_BallBay_Params _params;
    mutable HAL_Semaphore _sem;
    mutable State _state {};

    static AP_BallBay *_singleton;
};

namespace AP {
    AP_BallBay *ballbay();
}

#endif  // AP_BALLBAY_ENABLED