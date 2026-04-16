#include <AP_BallBay/AP_BallBay_Params.h>

const AP_Param::GroupInfo AP_BallBay_Params::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: BallBay feedback enable
    // @Description: Enable ballbay feedback from remote CAN node
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_BallBay_Params, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ID
    // @DisplayName: BallBay feedback source ID
    // @Description: Source ID sent by the remote BallBay CAN node
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("ID", 2, AP_BallBay_Params, sensor_id, 8),

    // @Param: TO_MS
    // @DisplayName: BallBay timeout
    // @Description: Timeout before BallBay data is considered stale
    // @Range: 50 5000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("TO_MS", 3, AP_BallBay_Params, timeout_ms, 200),

    AP_GROUPEND
};

AP_BallBay_Params::AP_BallBay_Params()
{
    AP_Param::setup_object_defaults(this, var_info);
}