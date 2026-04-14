#pragma once

#if AP_PERIPH_BALLBAY_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>
#include <stdint.h>
#include "NAU7802_ADC.h"
#include "TMC2209_Pulse.h"
#include "TMC2209_Serial.h"

class TMC2209_Pulse;
class TMC2209_Serial;
class NAU7802_ADC;

class BallBay
{
public:
    BallBay();

    enum class Mode : uint8_t {
        INACTIVE = 0,
        HOMING,
        ACTIVE,
    };

    enum class HomePhase : uint8_t {
        NONE = 0,
        CHECK_SWITCH,
        CLEAR_SWITCH,
        CLEAR_SWITCH_OFFSET,
        SEEK_HOME,
        SETTLE_OFFSET,
        ZERO_POSITION,
        DONE,
    };

    void init();
    void update();

    bool enabled() const { return _enable.get() != 0; }
    bool initialized() const { return _init_ok; }

    uint8_t get_actuator_id() const;

    bool handle_unitless_command(float value);

    float get_report_position() const;
    float get_report_speed() const;
    float get_report_force() const;

    bool loadcell_healthy() const;
    float get_loadcell_filtered_sum() const;
    bool loadcell_filtered_valid() const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    static constexpr uint8_t ENABLE_GPIO_NUM = 2;
    static constexpr uint8_t HOME_GPIO_NUM   = 3;
    static constexpr uint8_t DIAG_GPIO_NUM   = 4;

    static constexpr int32_t HOMING_SEEK_STEPS = 100000;
    static constexpr float COMMAND_INACTIVE = -1.0f;
    static constexpr float COMMAND_HOME = -0.5f;

    struct State {
        Mode mode = Mode::INACTIVE;
        HomePhase home_phase = HomePhase::NONE;
        int32_t phase_target_steps = 0;
        uint32_t phase_start_ms = 0;
        uint32_t last_update_ms = 0;
    };

    // parameters
    AP_Int8  _enable;
    AP_Int8  _actuator_id;
    AP_Int16 _max_steps;
    AP_Int16 _start_sps;
    AP_Int16 _vmax_sps;
    AP_Int16 _amax_sps2;
    AP_Int16 _pulse_us;
    AP_Int16 _home_sps;
    AP_Int16 _home_amax_sps2;
    AP_Int16 _home_off;
    AP_Int16 _home_timeout_s;
    AP_Int8  _tmc_ihold;
    AP_Int8  _tmc_irun;
    AP_Int8  _tmc_mstep;
    AP_Int8  _tmc_vsense;
    AP_Int16 _tmc_tpwmthrs;
    AP_Int16 _tmc_tcoolthrs;
    AP_Int16 _tmc_sgthrs;
    AP_Float _load_filt_hz;

    State _state {};

    TMC2209_Pulse *pulse = nullptr;
    TMC2209_Serial *uart_cfg = nullptr;
    NAU7802_ADC *loadcell = nullptr;

    bool _init_ok = false;
    volatile bool _stall_latched = false;

    float _input_value = COMMAND_INACTIVE;
    bool _homing_command_latched = false;

    float _load_sum_filtered = 0.0f;
    bool _load_sum_filtered_valid = false;
    uint32_t _last_load_pair_ms = 0;
    LowPassFilterFloat _load_sum_filter;

    static BallBay *_singleton;

    void enter_inactive();
    void begin_homing();
    void enter_active_idle();

    void update_homing();
    void start_homing_move(int32_t target_steps);
    void maybe_start_active_move();
    void set_active_target_from_input(float value);

    bool homing_timed_out() const;
    bool home_switch_active() const;
    void drive_enable(bool enable);

    static void diag_irq_handler();
};

#endif // AP_PERIPH_BALLBAY_ENABLED