#include "AP_BallBay.h"

#if AP_BALLBAY_ENABLED

#include "TMC2209_Pulse.h"
#include "TMC2209_Serial.h"
#include "NAU7802_ADC.h"

#include <AP_HAL/AP_HAL.h>
#include <hal.h>
#include <math.h>

extern const AP_HAL::HAL& hal;

AP_BallBay *AP_BallBay::_singleton = nullptr;

const AP_Param::GroupInfo AP_BallBay::var_info[] = {
    AP_SUBGROUPINFO(_params, "1_", 0, AP_BallBay, AP_BallBay_Params),
    AP_GROUPEND
};

AP_BallBay::AP_BallBay()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_BallBay must be singleton");
    }
    _singleton = this;
}

void AP_BallBay::init(void)
{
    _state = {};
    _state.mode = Mode::INACTIVE;
    _state.home_phase = HomePhase::NONE;
    _state.phase_target_steps = 0;
    _state.phase_start_ms = AP_HAL::millis();
    _state.last_update_ms = _state.phase_start_ms;

    _stall_latched = false;
    _input_value = -1.0f;
    _homing_command_latched = false;
    _init_ok = false;

    _load_sum_filter.reset();
    _load_sum_filtered = 0.0f;
    _load_sum_filtered_valid = false;
    _last_load_pair_ms = 0;

    drive_enable(false); // disable driver (active-low enable)

    if (!enabled()) {
        return;
    }

    if (get_type() != Type::TIMER6) {
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
    if (uart_cfg == nullptr || !uart_cfg->configure_driver(_params)) {
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

bool AP_BallBay::handle_unitless_command(float value)
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

void AP_BallBay::update(void)
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
            const float raw_sum = float(load_sum) / 100000;
            const float cutoff_hz = MAX(_params.load_filt_hz.get(), 0.0f);

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

    if (!is_equal(input_value, -0.5f)) {
        _homing_command_latched = false;
    }

    if (is_equal(input_value, -1.0f)) {
        if (_state.mode != Mode::INACTIVE) {
            enter_inactive();
        }
        return;
    }

    if (is_equal(input_value, -0.5f)) {
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

void AP_BallBay::enter_inactive(void)
{
    if (pulse != nullptr) {
        pulse->stop_move();
    }

    drive_enable(false); // disable driver (active-low enable)

    _stall_latched = false;
    _state.mode = Mode::INACTIVE;
    _state.home_phase = HomePhase::NONE;
    _state.phase_target_steps = 0;
    _state.phase_start_ms = AP_HAL::millis();
}

void AP_BallBay::begin_homing(void)
{
    if (pulse == nullptr) {
        enter_inactive();
        return;
    }

    pulse->stop_move();
    pulse->set_current_steps(0);
    pulse->set_target_steps(0);

    drive_enable(true);  // enable driver (active-low enable)

    _stall_latched = false;

    _state.mode = Mode::HOMING;
    _state.home_phase = HomePhase::CHECK_SWITCH;
    _state.phase_target_steps = 0;
    _state.phase_start_ms = AP_HAL::millis();
}

void AP_BallBay::enter_active_idle(void)
{
    if (pulse == nullptr) {
        enter_inactive();
        return;
    }

    pulse->stop_move();

    const int32_t current_steps = pulse->get_current_steps();
    pulse->set_target_steps(current_steps);

    drive_enable(true);  // enable driver (active-low enable)

    _state.mode = Mode::ACTIVE;
    _state.home_phase = HomePhase::NONE;
    _state.phase_target_steps = 0;
    _state.phase_start_ms = AP_HAL::millis();
}

void AP_BallBay::update_homing(void)
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
    const int32_t home_off = MAX(_params.home_off.get(), 0);

    switch (_state.home_phase) {
    case HomePhase::NONE:
        _state.home_phase = HomePhase::CHECK_SWITCH;
        _state.phase_start_ms = now;
        break;

    case HomePhase::CHECK_SWITCH:
        if (home_switch_active()) {
            _state.home_phase = HomePhase::CLEAR_SWITCH;
            _state.phase_start_ms = now;
            start_homing_move(current_steps + MAX(_params.max_steps.get(), 0) + HOMING_SEEK_STEPS);
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

void AP_BallBay::start_homing_move(int32_t target_steps)
{
    if (pulse == nullptr) {
        enter_inactive();
        return;
    }

    pulse->stop_move();
    pulse->set_target_steps(target_steps);
    pulse->start_move(
        MAX(_params.home_sps.get(), 1.0f),
        MAX(_params.home_sps.get(), 1.0f),
        MAX(_params.home_amax_sps2.get(), 1.0f),
        uint16_t(MAX<int16_t>(_params.pulse_us.get(), 1))
    );
}

void AP_BallBay::set_active_target_from_input(float value)
{
    if (pulse == nullptr) {
        return;
    }

    value = constrain_float(value, 0.0f, 1.0f);

    const int32_t max_steps = MAX(_params.max_steps.get(), 0);
    const int32_t target_steps = constrain_int32(
        int32_t(lrintf(value * float(max_steps))),
        0,
        max_steps
    );

    pulse->set_target_steps(target_steps);
}

void AP_BallBay::maybe_start_active_move(void)
{
    if (pulse == nullptr) {
        return;
    }

    if (!pulse->is_running() &&
        pulse->get_current_steps() != pulse->get_target_steps()) {
        const float start_sps = MAX(_params.start_sps.get(), 1.0f);

        pulse->start_move(
            start_sps,
            MAX(_params.vmax_sps.get(), start_sps),
            MAX(_params.amax_sps2.get(), 1.0f),
            uint16_t(MAX<int16_t>(_params.pulse_us.get(), 1))
        );
    }
}

bool AP_BallBay::homing_timed_out(void) const
{
    const uint32_t timeout_ms =
        uint32_t(MAX(_params.home_timeout_s.get(), 1)) * 1000U;

    return (AP_HAL::millis() - _state.phase_start_ms) > timeout_ms;
}

bool AP_BallBay::home_switch_active(void) const
{
    return hal.gpio->read(HOME_GPIO_NUM) == 0;
}

void AP_BallBay::drive_enable(bool enable)
{
    // Enable is active-low
    hal.gpio->write(ENABLE_GPIO_NUM, enable ? 0 : 1);
}

float AP_BallBay::get_report_position() const
{
    if (pulse == nullptr) {
        return -1.0f;
    }

    switch (_state.mode) {
    case Mode::INACTIVE:
        return -1.0f;

    case Mode::HOMING:
        return -0.5f;

    case Mode::ACTIVE: {
        const int32_t max_steps = MAX(_params.max_steps.get(), 0);
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

    return -1.0f;
}

float AP_BallBay::get_report_speed() const
{
    if (pulse == nullptr) {
        return 0.0f;
    }

    switch (_state.mode) {
    case Mode::INACTIVE:
        return 0.0f;

    case Mode::HOMING: {
        const float vmax = MAX(_params.home_sps.get(), 1.0f);
        return constrain_float(pulse->get_speed_sps() / vmax, 0.0f, 1.0f);
    }

    case Mode::ACTIVE: {
        const float start_sps = MAX(_params.start_sps.get(), 1.0f);
        const float vmax = MAX(_params.vmax_sps.get(), start_sps);
        return constrain_float(pulse->get_speed_sps() / vmax, 0.0f, 1.0f);
    }
    }

    return 0.0f;
}

bool AP_BallBay::loadcell_healthy() const
{
    return loadcell != nullptr && loadcell->healthy();
}

float AP_BallBay::get_loadcell_filtered_sum() const
{
    return _load_sum_filtered_valid ? _load_sum_filtered : 0.0f;
}

bool AP_BallBay::loadcell_filtered_valid() const
{
    return _load_sum_filtered_valid;
}

void AP_BallBay::diag_irq_handler()
{
    if (_singleton != nullptr) {
        _singleton->_stall_latched = true;
    }
}

namespace AP {
AP_BallBay *ballbay()
{
    return AP_BallBay::get_singleton();
}
}

#endif