#pragma once

#include <AP_Param/AP_Param.h>
#include "AP_BallBay_config.h"

class AP_BallBay_Params
{
public:
    AP_BallBay_Params();

    AP_Int8  enable;       // 0=disabled, 1=enabled
    AP_Int8  sensor_id;    // CAN source ID from remote node
    AP_Int16 timeout_ms;   // stale-data timeout

    static const struct AP_Param::GroupInfo var_info[];
};