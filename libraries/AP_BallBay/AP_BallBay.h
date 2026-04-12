#pragma once

#include "AP_BallBay_config.h"

#if AP_BALLBAY_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>

#include "AP_BallBay_Params.h"

class TMC2209_Pulse;
class TMC2209_Serial;
class NAU7802_ADC;

class AP_BallBay
{
public:
    AP_BallBay();
    CLASS_NO_COPY(AP_BallBay);

    enum class Type : uint8_t {
        NONE   = 0,
        TIMER6 = 1,
    };

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

    AP_BallBay_Params _params;

    static const struct AP_Param::GroupInfo var_info[];

    void init(void);
    void update(void);

    Type get_type() const { return (Type)((uint8_t)_params.type); }
    bool enabled() const { return (get_type() != Type::NONE); }

    bool handle_unitless_command(float value);

    float get_report_position() const;
    float get_report_speed() const;

    bool loadcell_healthy() const;
    float get_loadcell_filtered_sum() const;
    bool loadcell_filtered_valid() const;

    static AP_BallBay *get_singleton() { return _singleton; }

private:
    static constexpr uint8_t ENABLE_GPIO_NUM = 2;
    static constexpr uint8_t HOME_GPIO_NUM   = 3;
    static constexpr uint8_t DIAG_GPIO_NUM   = 4;

    static constexpr int32_t HOMING_SEEK_STEPS = 100000;

    struct State {
        Mode mode = Mode::INACTIVE;
        HomePhase home_phase = HomePhase::NONE;
        int32_t phase_target_steps = 0;
        uint32_t phase_start_ms = 0;
        uint32_t last_update_ms = 0;
    };

    static AP_BallBay *_singleton;

    State _state {};
    TMC2209_Pulse *pulse = nullptr;
    TMC2209_Serial *uart_cfg = nullptr;
    NAU7802_ADC *loadcell = nullptr;

    bool _init_ok = false;

    volatile bool _stall_latched = false;

    float _input_value = -1.0f;
    bool _homing_command_latched = false;

    LowPassFilterFloat _load_sum_filter;
    float _load_sum_filtered = 0.0f;
    bool _load_sum_filtered_valid = false;
    uint32_t _last_load_pair_ms = 0;

    void enter_inactive(void);
    void begin_homing(void);
    void enter_active_idle(void);

    void update_homing(void);
    void start_homing_move(int32_t target_steps);
    void maybe_start_active_move(void);
    void set_active_target_from_input(float value);

    bool homing_timed_out(void) const;
    bool home_switch_active(void) const;
    void drive_enable(bool enable);

    static void diag_irq_handler();
};

namespace AP {
    AP_BallBay *ballbay();
}

#endif