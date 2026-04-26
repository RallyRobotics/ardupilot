#include "AP_Periph.h"

#if AP_PERIPH_BALLBAY_ENABLED

#include <dronecan_msgs.h>

extern const AP_HAL::HAL& hal;

BallBay *BallBay::_singleton = nullptr;

const AP_Param::GroupInfo BallBay::var_info[] = {

    AP_GROUPINFO_FLAGS("ENABLE",     1,  BallBay, _enable,           1, AP_PARAM_FLAG_ENABLE),
    AP_GROUPINFO("ACT_ID",           2,  BallBay, _actuator_id,      8),
    AP_GROUPINFO("TELEM_RATE",       3,  BallBay, _report_rate_hz,   10),
    AP_GROUPINFO("MAX_STEPS",        4,  BallBay, _max_steps,        24000),
    AP_GROUPINFO("START_SPS",        5,  BallBay, _start_sps,        50),
    AP_GROUPINFO("MOVE_VMAX",        6,  BallBay, _vmax_sps,         3000),
    AP_GROUPINFO("MOVE_AMAX",        7,  BallBay, _amax_sps2,        1000),
    AP_GROUPINFO("PULSE_US",         8,  BallBay, _pulse_us,         20),
    AP_GROUPINFO("HOME_VMAX",        9,  BallBay, _home_sps,         480),
    AP_GROUPINFO("HOME_AMAX",        10, BallBay, _home_amax_sps2,   240),
    AP_GROUPINFO("HOME_OFFS",        11, BallBay, _home_off,         800),
    AP_GROUPINFO("HOME_TIME",        12, BallBay, _home_timeout_s,   60),
    AP_GROUPINFO("HOLD_CURR",        13, BallBay, _tmc_ihold,        1),
    AP_GROUPINFO("MOVE_CURR",        14, BallBay, _tmc_irun,         20),
    AP_GROUPINFO("MICRO_STEP",       15, BallBay, _tmc_mstep,        8),
    AP_GROUPINFO("V_SENSE",          16, BallBay, _tmc_vsense,       0),
    AP_GROUPINFO("TPWM_THRS",        17, BallBay, _tmc_tpwmthrs,     0),
    AP_GROUPINFO("TCOOL_THRS",       18, BallBay, _tmc_tcoolthrs,    200),
    AP_GROUPINFO("SG_THRS",          19, BallBay, _tmc_sgthrs,       50),
    AP_GROUPINFO("FILTER_HZ",        20, BallBay, _load_filt_hz,     0.2f),

    AP_GROUPEND
};

BallBay::BallBay()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("BallBay must be singleton");
    }
    _singleton = this;
}

void BallBay::init()
{
    _state = {};
    _state.mode = Mode::INACTIVE;
    _state.home_phase = HomePhase::NONE;
    _state.phase_target_steps = 0;
    _state.phase_start_ms = AP_HAL::millis();
    _state.last_update_ms = _state.phase_start_ms;

    _stall_latched = false;
    _input_value = COMMAND_INACTIVE;
    _homing_command_latched = false;
    _init_ok = false;

    _load_sum_filter.reset();
    _load_sum_filtered = 0.0f;
    _load_sum_filtered_valid = false;
    _last_load_pair_ms = 0;

    drive_enable(false); // active-low enable

    if (!enabled()) {
        return;
    }

    pulse = NEW_NOTHROW TMC2209_Pulse();
    if (pulse == nullptr || !pulse->init()) {
        enter_inactive();
        return;
    }

    pulse->set_current_steps(0);
    pulse->set_target_steps(0);

    uart_cfg = NEW_NOTHROW TMC2209_Serial();
    if (uart_cfg == nullptr) {
        enter_inactive();
        return;
    }

    TMC2209_Serial::Config tmc_cfg {};
    tmc_cfg.ihold = uint8_t(MAX<int16_t>(_tmc_ihold.get(), 0));
    tmc_cfg.irun = uint8_t(MAX<int16_t>(_tmc_irun.get(), 0));
    tmc_cfg.microsteps = uint16_t(MAX<int16_t>(_tmc_mstep.get(), 1));
    tmc_cfg.vsense = (_tmc_vsense.get() != 0);
    tmc_cfg.tpwmthrs = uint32_t(MAX<int16_t>(_tmc_tpwmthrs.get(), 0));
    tmc_cfg.tcoolthrs = uint32_t(MAX<int16_t>(_tmc_tcoolthrs.get(), 0));
    tmc_cfg.sgthrs = uint8_t(MAX<int16_t>(_tmc_sgthrs.get(), 0));

    if (!uart_cfg->configure_driver(tmc_cfg)) {
        enter_inactive();
        return;
    }

    if (!hal.gpio->attach_interrupt(DIAG_GPIO_NUM,
                                    (AP_HAL::Proc)diag_irq_handler,
                                    AP_HAL::GPIO::INTERRUPT_RISING)) {
        enter_inactive();
        return;
    }

    loadcell = NEW_NOTHROW NAU7802_ADC();
    if (loadcell != nullptr) {
        (void)loadcell->init();
    }

    _init_ok = true;
    enter_inactive();
}

bool BallBay::handle_unitless_command(float value)
{
    if (!enabled() || !_init_ok) {
        return false;
    }

    if (value < -1.0f || value > 1.0f) {
        return false;
    }

    chSysLock();
    _input_value = value;
    chSysUnlock();

    return true;
}

void BallBay::update()
{
    _state.last_update_ms = AP_HAL::millis();

    if (!enabled() || !_init_ok) {
        enter_inactive();
        return;
    }

    if (loadcell != nullptr) {
        loadcell->update();

        int32_t load_sum = 0;
        if (loadcell->read_load(load_sum)) {
            const uint32_t now_ms = AP_HAL::millis();
            const float raw_sum = float(load_sum) / 100000.0f;

            float cutoff_hz = _load_filt_hz.get();
            if (cutoff_hz < 0.0f) {
                cutoff_hz = 0.0f;
            }

            if (!_load_sum_filtered_valid || cutoff_hz <= 0.0f) {
                _load_sum_filter.reset(raw_sum);
                _load_sum_filtered = raw_sum;
                _load_sum_filtered_valid = true;
            } else {
                float dt = float(now_ms - _last_load_pair_ms) * 0.001f;
                if (dt < 0.001f) {
                    dt = 0.001f;
                }

                _load_sum_filter.set_cutoff_frequency(cutoff_hz);
                _load_sum_filtered = _load_sum_filter.apply(raw_sum, dt);
            }

            _last_load_pair_ms = now_ms;
        }
    }

    if (_stall_latched && _state.mode != Mode::INACTIVE) {
        enter_inactive();
        return;
    }

    float input_value;
    chSysLock();
    input_value = _input_value;
    chSysUnlock();

    if (!is_equal(input_value, COMMAND_HOME)) {
        _homing_command_latched = false;
    }

    if (is_equal(input_value, COMMAND_INACTIVE)) {
        if (_state.mode != Mode::INACTIVE) {
            enter_inactive();
        }
        return;
    }

    if (is_equal(input_value, COMMAND_HOME)) {
        if (!_homing_command_latched) {
            _homing_command_latched = true;
            begin_homing();
        }
    } else if (input_value >= 0.0f && input_value <= 1.0f) {
        if (_state.mode == Mode::ACTIVE && pulse != nullptr) {
            set_active_target_from_input(input_value);
        }
    }

    switch (_state.mode) {
    case Mode::INACTIVE:
        break;

    case Mode::HOMING:
        update_homing();
        break;

    case Mode::ACTIVE:
        maybe_start_active_move();
        break;
    }
}

void BallBay::enter_inactive()
{
    if (pulse != nullptr) {
        pulse->stop_move();
    }

    drive_enable(false); // active-low enable

    _stall_latched = false;
    _state.mode = Mode::INACTIVE;
    _state.home_phase = HomePhase::NONE;
    _state.phase_target_steps = 0;
    _state.phase_start_ms = AP_HAL::millis();
}

void BallBay::begin_homing()
{
    if (pulse == nullptr) {
        enter_inactive();
        return;
    }

    pulse->stop_move();
    pulse->set_current_steps(0);
    pulse->set_target_steps(0);

    drive_enable(true); // active-low enable

    _stall_latched = false;

    _state.mode = Mode::HOMING;
    _state.home_phase = HomePhase::CHECK_SWITCH;
    _state.phase_target_steps = 0;
    _state.phase_start_ms = AP_HAL::millis();
}

void BallBay::enter_active_idle()
{
    if (pulse == nullptr) {
        enter_inactive();
        return;
    }

    pulse->stop_move();

    const int32_t current_steps = pulse->get_current_steps();
    pulse->set_target_steps(current_steps);

    drive_enable(true); // active-low enable

    _state.mode = Mode::ACTIVE;
    _state.home_phase = HomePhase::NONE;
    _state.phase_target_steps = 0;
    _state.phase_start_ms = AP_HAL::millis();
}

void BallBay::update_homing()
{
    if (pulse == nullptr) {
        enter_inactive();
        return;
    }

    if (homing_timed_out()) {
        enter_inactive();
        return;
    }

    const uint32_t now = AP_HAL::millis();
    const int32_t current_steps = pulse->get_current_steps();

    int32_t max_steps = _max_steps.get();
    if (max_steps < 0) {
        max_steps = 0;
    }

    int32_t home_off = _home_off.get();
    if (home_off < 0) {
        home_off = 0;
    }

    switch (_state.home_phase) {
    case HomePhase::NONE:
        _state.home_phase = HomePhase::CHECK_SWITCH;
        _state.phase_start_ms = now;
        break;

    case HomePhase::CHECK_SWITCH:
        if (home_switch_active()) {
            _state.home_phase = HomePhase::CLEAR_SWITCH;
            _state.phase_start_ms = now;
            start_homing_move(current_steps + max_steps + HOMING_SEEK_STEPS);
        } else {
            _state.home_phase = HomePhase::SEEK_HOME;
            _state.phase_start_ms = now;
            start_homing_move(current_steps - HOMING_SEEK_STEPS);
        }
        break;

    case HomePhase::CLEAR_SWITCH:
        if (!home_switch_active()) {
            _state.home_phase = HomePhase::CLEAR_SWITCH_OFFSET;
            _state.phase_start_ms = now;
            _state.phase_target_steps = pulse->get_current_steps() + home_off;
            start_homing_move(_state.phase_target_steps);
        }
        break;

    case HomePhase::CLEAR_SWITCH_OFFSET:
        if (!pulse->is_running() &&
            pulse->get_current_steps() == _state.phase_target_steps) {
            _state.home_phase = HomePhase::SEEK_HOME;
            _state.phase_start_ms = now;
            start_homing_move(pulse->get_current_steps() - HOMING_SEEK_STEPS);
        }
        break;

    case HomePhase::SEEK_HOME:
        if (home_switch_active()) {
            _state.home_phase = HomePhase::SETTLE_OFFSET;
            _state.phase_start_ms = now;
            _state.phase_target_steps = pulse->get_current_steps() - home_off;
            start_homing_move(_state.phase_target_steps);
        }
        break;

    case HomePhase::SETTLE_OFFSET:
        if (!pulse->is_running() &&
            pulse->get_current_steps() == _state.phase_target_steps) {
            _state.home_phase = HomePhase::ZERO_POSITION;
            _state.phase_start_ms = now;
        }
        break;

    case HomePhase::ZERO_POSITION:
        pulse->stop_move();
        pulse->set_current_steps(0);
        pulse->set_target_steps(0);
        _state.home_phase = HomePhase::DONE;
        _state.phase_start_ms = now;
        break;

    case HomePhase::DONE:
        enter_active_idle();
        break;
    }
}

void BallBay::start_homing_move(int32_t target_steps)
{
    if (pulse == nullptr) {
        enter_inactive();
        return;
    }

    float home_sps = float(_home_sps.get());
    if (home_sps < 1.0f) {
        home_sps = 1.0f;
    }

    float home_amax_sps2 = float(_home_amax_sps2.get());
    if (home_amax_sps2 < 1.0f) {
        home_amax_sps2 = 1.0f;
    }

    int16_t pulse_us = _pulse_us.get();
    if (pulse_us < 1) {
        pulse_us = 1;
    }

    pulse->stop_move();
    pulse->set_target_steps(target_steps);
    pulse->start_move(
        home_sps,
        home_sps,
        home_amax_sps2,
        uint16_t(pulse_us)
    );
}

void BallBay::set_active_target_from_input(float value)
{
    if (pulse == nullptr) {
        return;
    }

    value = constrain_float(value, 0.0f, 1.0f);

    int32_t max_steps = _max_steps.get();
    if (max_steps < 0) {
        max_steps = 0;
    }

    const int32_t target_steps = constrain_int32(
        int32_t(lrintf(value * float(max_steps))),
        0,
        max_steps
    );

    pulse->set_target_steps(target_steps);
}

void BallBay::maybe_start_active_move()
{
    if (pulse == nullptr) {
        return;
    }

    if (!pulse->is_running() &&
        pulse->get_current_steps() != pulse->get_target_steps()) {

        float start_sps = float(_start_sps.get());
        if (start_sps < 1.0f) {
            start_sps = 1.0f;
        }

        float vmax_sps = float(_vmax_sps.get());
        if (vmax_sps < start_sps) {
            vmax_sps = start_sps;
        }

        float amax_sps2 = float(_amax_sps2.get());
        if (amax_sps2 < 1.0f) {
            amax_sps2 = 1.0f;
        }

        int16_t pulse_us = _pulse_us.get();
        if (pulse_us < 1) {
            pulse_us = 1;
        }

        pulse->start_move(
            start_sps,
            vmax_sps,
            amax_sps2,
            uint16_t(pulse_us)
        );
    }
}

bool BallBay::homing_timed_out() const
{
    int16_t timeout_s = _home_timeout_s.get();
    if (timeout_s < 1) {
        timeout_s = 1;
    }

    const uint32_t timeout_ms = uint32_t(timeout_s) * 1000U;
    return (AP_HAL::millis() - _state.phase_start_ms) > timeout_ms;
}

bool BallBay::home_switch_active() const
{
    return hal.gpio->read(HOME_GPIO_NUM) == 0;
}

void BallBay::drive_enable(bool enable)
{
    // active-low enable
    hal.gpio->write(ENABLE_GPIO_NUM, enable ? 0 : 1);
}

float BallBay::get_report_position() const
{
    if (pulse == nullptr) {
        return COMMAND_INACTIVE;
    }

    switch (_state.mode) {
    case Mode::INACTIVE:
        return COMMAND_INACTIVE;

    case Mode::HOMING:
        return COMMAND_HOME;

    case Mode::ACTIVE: {
        int32_t max_steps = _max_steps.get();
        if (max_steps <= 0) {
            return 0.0f;
        }

        return constrain_float(
            float(pulse->get_current_steps()) / float(max_steps),
            0.0f,
            1.0f
        );
    }
    }

    return COMMAND_INACTIVE;
}

float BallBay::get_report_speed() const
{
    if (pulse == nullptr) {
        return 0.0f;
    }

    switch (_state.mode) {
    case Mode::INACTIVE:
        return 0.0f;

    case Mode::HOMING: {
        float vmax = float(_home_sps.get());
        if (vmax < 1.0f) {
            vmax = 1.0f;
        }

        return constrain_float(pulse->get_speed_sps() / vmax, 0.0f, 1.0f);
    }

    case Mode::ACTIVE: {
        float start_sps = float(_start_sps.get());
        if (start_sps < 1.0f) {
            start_sps = 1.0f;
        }

        float vmax = float(_vmax_sps.get());
        if (vmax < start_sps) {
            vmax = start_sps;
        }

        return constrain_float(pulse->get_speed_sps() / vmax, 0.0f, 1.0f);
    }
    }

    return 0.0f;
}

float BallBay::get_report_force() const
{
    return _load_sum_filtered_valid ? _load_sum_filtered : NAN;
}

bool BallBay::loadcell_healthy() const
{
    return loadcell != nullptr && loadcell->healthy();
}

float BallBay::get_loadcell_filtered_sum() const
{
    return _load_sum_filtered_valid ? _load_sum_filtered : 0.0f;
}

bool BallBay::loadcell_filtered_valid() const
{
    return _load_sum_filtered_valid;
}

void BallBay::diag_irq_handler()
{
    if (_singleton != nullptr) {
        _singleton->_stall_latched = true;
    }
}

void AP_Periph_FW::can_ballbay_update()
{
    static uint32_t last_update_ms;
    static uint32_t last_publish_ms;

    const uint32_t now = AP_HAL::millis();

    if ((now - last_update_ms) >= 20U) {
        last_update_ms = now;
        ballbay.update();
    }

    if (!ballbay.enabled() || !ballbay.initialized()) {
        return;
    }

    const uint32_t rate_hz = ballbay.get_report_rate_hz();
    const uint32_t interval_ms = MAX<uint32_t>(1U, 1000U / MAX<uint32_t>(1U, rate_hz));

    if ((now - last_publish_ms) < interval_ms) {
        return;
    }
    last_publish_ms = now;

    uavcan_equipment_actuator_Status pkt {};
    pkt.actuator_id = ballbay.get_actuator_id();
    pkt.position = ballbay.get_report_position();
    pkt.force = ballbay.get_report_force();
    pkt.speed = ballbay.get_report_speed();
    pkt.power_rating_pct = UAVCAN_EQUIPMENT_ACTUATOR_STATUS_POWER_RATING_PCT_UNKNOWN;

    uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE] {};
    const uint16_t total_size = uavcan_equipment_actuator_Status_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                     UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     &buffer[0],
                     total_size);
}

void AP_Periph_FW::ballbay_srv_unitless(uint8_t actuator_id, const float command_value)
{
    if (actuator_id != ballbay.get_actuator_id()) {
        return;
    }

    (void)ballbay.handle_unitless_command(command_value);
}

#endif // AP_PERIPH_BALLBAY_ENABLED