#pragma once

#include <AP_Param/AP_Param.h>
#include "AP_StopButton_config.h"

#if AP_STOPBUTTON_ENABLED

class AP_StopButton_Params
{
public:
    AP_StopButton_Params();

    AP_Int8  enable;       // 0=disabled, 1=enabled
    AP_Int8  sensor_id;    // actuator_id from remote CAN node
    AP_Int16 timeout_ms;   // stale-data timeout

    static const struct AP_Param::GroupInfo var_info[];
};

#endif  // AP_STOPBUTTON_ENABLED