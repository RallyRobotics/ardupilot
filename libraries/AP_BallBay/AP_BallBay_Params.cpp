#include "AP_BallBay_Params.h"

const AP_Param::GroupInfo AP_BallBay_Params::var_info[] = {
    AP_GROUPINFO_FLAGS("TYPE",  1,  AP_BallBay_Params, type,           1, AP_PARAM_FLAG_ENABLE),
    AP_GROUPINFO("ACT_ID",      2,  AP_BallBay_Params, actuator_id,    8),
    AP_GROUPINFO("MAX_STEPS",   3,  AP_BallBay_Params, max_steps,      24000),
    AP_GROUPINFO("START_SPS",   4,  AP_BallBay_Params, start_sps,      50),
    AP_GROUPINFO("MOVE_VMAX",   5,  AP_BallBay_Params, vmax_sps,       3000),
    AP_GROUPINFO("MOVE_AMAX",   6,  AP_BallBay_Params, amax_sps2,      1000),
    AP_GROUPINFO("PULSE_US",    7,  AP_BallBay_Params, pulse_us,       20),
    AP_GROUPINFO("HOME_VMAX",   8,  AP_BallBay_Params, home_sps,       480),
    AP_GROUPINFO("HOME_AMAX",   9,  AP_BallBay_Params, home_amax_sps2, 240),
    AP_GROUPINFO("HOME_OFFS",   10, AP_BallBay_Params, home_off,       800),
    AP_GROUPINFO("HOME_TIME",   11, AP_BallBay_Params, home_timeout_s, 60),
    AP_GROUPINFO("HOLD_CURR",   12, AP_BallBay_Params, tmc_ihold,      1),
    AP_GROUPINFO("MOVE_CURR",   13, AP_BallBay_Params, tmc_irun,       20),
    AP_GROUPINFO("MICRO_STEP",  14, AP_BallBay_Params, tmc_mstep,      8),
    AP_GROUPINFO("V_SENSE",     15, AP_BallBay_Params, tmc_vsense,     0),
    AP_GROUPINFO("TPWM_THRS",   16, AP_BallBay_Params, tmc_tpwmthrs,   0),
    AP_GROUPINFO("TCOOL_THRS",  17, AP_BallBay_Params, tmc_tcoolthrs,  200),
    AP_GROUPINFO("SG_THRS",     18, AP_BallBay_Params, tmc_sgthrs,     50),
    AP_GROUPINFO("FILTER_HZ",   19, AP_BallBay_Params, load_filt_hz,   0.2f),
    AP_GROUPEND
};

AP_BallBay_Params::AP_BallBay_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
