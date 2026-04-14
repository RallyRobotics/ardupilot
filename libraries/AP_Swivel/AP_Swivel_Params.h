#pragma once

#include <AP_Param/AP_Param.h>
#include "AP_Swivel_config.h"

class AP_Swivel_Params
{
public:
    AP_Swivel_Params();

    AP_Int8  enable;       // 0=disabled, 1=enabled
    AP_Int8  sensor_id;    // CAN source ID from remote node

    AP_Float volt_min;     // voltage at minimum mechanical angle
    AP_Float volt_max;     // voltage at maximum mechanical angle
    AP_Float ang_min_deg;  // angle corresponding to volt_min
    AP_Float ang_max_deg;  // angle corresponding to volt_max

    AP_Int8  reversed;     // 0=normal, 1=reversed
    AP_Int16 timeout_ms;   // stale-data timeout

    static const struct AP_Param::GroupInfo var_info[];
};