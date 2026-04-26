#include "AP_StopButton_Params.h"

#if AP_STOPBUTTON_ENABLED

const AP_Param::GroupInfo AP_StopButton_Params::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: StopButton enable
    // @Description: Enable stop button feedback from remote CAN node
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_StopButton_Params, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ID
    // @DisplayName: StopButton actuator ID
    // @Description: actuator_id sent by the remote StopButton CAN node
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("ID", 2, AP_StopButton_Params, sensor_id, 9),

    // @Param: TO_MS
    // @DisplayName: StopButton timeout
    // @Description: Timeout before StopButton data is considered stale
    // @Range: 50 5000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("TO_MS", 3, AP_StopButton_Params, timeout_ms, 300),

    AP_GROUPEND
};

AP_StopButton_Params::AP_StopButton_Params()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#endif  // AP_STOPBUTTON_ENABLED