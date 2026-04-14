#include <AP_Swivel/AP_Swivel_Params.h>

const AP_Param::GroupInfo AP_Swivel_Params::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Swivel feedback enable
    // @Description: Enable swivel feedback from remote CAN node
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Swivel_Params, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ID
    // @DisplayName: Swivel feedback source ID
    // @Description: Source ID sent by the remote swivel CAN node
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("ID", 2, AP_Swivel_Params, sensor_id, 1),

    // @Param: V_MIN
    // @DisplayName: Swivel minimum voltage
    // @Description: Voltage corresponding to ANG_MIN
    // @Range: 0.000 5.000
    // @User: Standard
    AP_GROUPINFO("V_MIN", 3, AP_Swivel_Params, volt_min, 0.0f),

    // @Param: V_MAX
    // @DisplayName: Swivel maximum voltage
    // @Description: Voltage corresponding to ANG_MAX
    // @Range: 0.000 5.000
    // @User: Standard
    AP_GROUPINFO("V_MAX", 4, AP_Swivel_Params, volt_max, 3.3f),

    // @Param: ANG_MIN
    // @DisplayName: Swivel minimum angle
    // @Description: Mechanical angle corresponding to V_MIN
    // @Range: -180 180
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("ANG_MIN", 5, AP_Swivel_Params, ang_min_deg, -90.0f),

    // @Param: ANG_MAX
    // @DisplayName: Swivel maximum angle
    // @Description: Mechanical angle corresponding to V_MAX
    // @Range: -180 180
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("ANG_MAX", 6, AP_Swivel_Params, ang_max_deg, 90.0f),

    // @Param: REV
    // @DisplayName: Swivel reverse
    // @Description: Reverse swivel direction after voltage-to-angle mapping
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    AP_GROUPINFO("REV", 7, AP_Swivel_Params, reversed, 0),

    // @Param: TO_MS
    // @DisplayName: Swivel timeout
    // @Description: Timeout before swivel data is considered stale
    // @Range: 50 5000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("TO_MS", 8, AP_Swivel_Params, timeout_ms, 200),

    AP_GROUPEND
};

AP_Swivel_Params::AP_Swivel_Params()
{
    AP_Param::setup_object_defaults(this, var_info);
}