#pragma once
#include <AP_Param/AP_Param.h>
#include "AP_BallBay_config.h"

class AP_BallBay_Params {

public:

    AP_BallBay_Params(void);

    AP_Int8  type;
    AP_Int8  actuator_id;
    AP_Int16 max_steps;
    AP_Int16 start_sps;
    AP_Int16 vmax_sps;
    AP_Int16 amax_sps2;
    AP_Int16 pulse_us;
    AP_Int16 home_sps;
    AP_Int16 home_amax_sps2;
    AP_Int16 home_off;
    AP_Int16 home_timeout_s;
    AP_Int8  tmc_ihold;
    AP_Int8  tmc_irun;
    AP_Int8  tmc_mstep;
    AP_Int8  tmc_vsense;
    AP_Int16 tmc_tpwmthrs;
    AP_Int16 tmc_tcoolthrs;
    AP_Int16 tmc_sgthrs;
    AP_Float load_filt_hz;

    static const struct AP_Param::GroupInfo var_info[];

};
