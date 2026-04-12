#include "TMC2209_Pulse.h"

#if AP_BALLBAY_ENABLED

#include <math.h>

TMC2209_Pulse *TMC2209_Pulse::_driver = nullptr;

static const GPTConfig gptcfg = {
    1000000U,
    TMC2209_Pulse::tim6_cb,
    0U,
    0U
};

TMC2209_Pulse::TMC2209_Pulse()
{
    _driver = this;
}

bool TMC2209_Pulse::init()
{
    step_pin_low();
    dir_pin_write(true);
    gptStart(&GPTD6, &gptcfg);
    return true;
}

void TMC2209_Pulse::set_current_steps(int32_t steps)
{
    chSysLock();
    _current_steps = steps;
    chSysUnlock();
}

int32_t TMC2209_Pulse::get_current_steps() const
{
    return _current_steps;
}

void TMC2209_Pulse::set_target_steps(int32_t steps)
{
    chSysLock();
    _target_steps = steps;
    chSysUnlock();
}

int32_t TMC2209_Pulse::get_target_steps() const
{
    return _target_steps;
}

bool TMC2209_Pulse::is_running() const
{
    return _running;
}

float TMC2209_Pulse::get_speed_sps() const
{
    return _speed_sps;
}

void TMC2209_Pulse::start_move(float start_sps, float vmax, float amax, uint16_t pulse_us)
{
    chSysLock();

    if (_running) {
        chSysUnlock();
        return;
    }

    const int32_t error = _target_steps - _current_steps;
    if (error == 0) {
        chSysUnlock();
        return;
    }

    if (start_sps < 1.0f) {
        start_sps = 1.0f;
    }

    if (vmax < start_sps) {
        vmax = start_sps;
    }

    if (amax < 1.0f) {
        amax = 1.0f;
    }

    if (pulse_us < 1U) {
        pulse_us = 1U;
    }

    _start_sps = start_sps;
    _vmax_sps = vmax;
    _amax_sps2 = amax;
    _pulse_us = pulse_us;

    _dir_positive = (error > 0);
    _speed_sps = _start_sps;
    _running = true;
    _step_high = true;

    dir_pin_write(_dir_positive);
    step_pin_high();

    gptStartOneShotI(&GPTD6, _pulse_us);

    chSysUnlock();
}

void TMC2209_Pulse::stop_move()
{
    chSysLock();

    _running = false;
    _step_high = false;
    _speed_sps = 0.0f;

    gptStopTimerI(&GPTD6);
    step_pin_low();

    chSysUnlock();
}

void TMC2209_Pulse::tim6_cb(GPTDriver *gptp)
{
    (void)gptp;
    if (_driver != nullptr) {
        _driver->timer_cb();
    }
}

void TMC2209_Pulse::timer_cb()
{
    if (!_running) {
        step_pin_low();
        _step_high = false;
        return;
    }

    if (_step_high) {
        // falling edge commits the step
        step_pin_low();
        _step_high = false;

        _current_steps += _dir_positive ? 1 : -1;

        const int32_t error = _target_steps - _current_steps;
        if (error == 0) {
            _speed_sps = 0.0f;
            _running = false;
            return;
        }

        const int desired_sign = (error > 0) ? +1 : -1;
        const int move_sign = _dir_positive ? +1 : -1;

        float v = fabsf(_speed_sps);
        if (v < _start_sps) {
            v = _start_sps;
        }

        const float dv = _amax_sps2 / v;

        if (desired_sign != move_sign) {
            // brake first, then reverse once we get back to start speed
            v -= dv;
            if (v <= _start_sps) {
                v = _start_sps;
                _dir_positive = (desired_sign > 0);
            }
        } else {
            const float remaining = fabsf((float)error);
            const float d_stop = (v * v) / (2.0f * _amax_sps2);

            if (remaining <= d_stop + 1.0f) {
                v -= dv;
                if (v < _start_sps) {
                    v = _start_sps;
                }
            } else {
                v += dv;
                if (v > _vmax_sps) {
                    v = _vmax_sps;
                }
            }
        }

        _speed_sps = v;
        dir_pin_write(_dir_positive);

        uint16_t high_ticks = _pulse_us;
        if (high_ticks < 1U) {
            high_ticks = 1U;
        }

        uint32_t full_ticks32 = (uint32_t)lrintf((float)GPT_HZ / fabsf(_speed_sps));
        const uint16_t min_ticks = high_ticks + 1U;

        if (full_ticks32 < min_ticks) {
            full_ticks32 = min_ticks;
        }
        if (full_ticks32 > 65535U) {
            full_ticks32 = 65535U;
        }

        const uint16_t full_ticks = (uint16_t)full_ticks32;
        const uint16_t low_ticks = (full_ticks > high_ticks) ? (full_ticks - high_ticks) : 1U;
        gptStartOneShotI(&GPTD6, low_ticks);
        return;
    }

    // rising edge
    step_pin_high();
    _step_high = true;

    uint16_t high_ticks = _pulse_us;
    if (high_ticks < 1U) {
        high_ticks = 1U;
    }

    gptStartOneShotI(&GPTD6, high_ticks);
}

#endif