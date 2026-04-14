#pragma once

#if AP_PERIPH_BALLBAY_ENABLED

#include <hal.h>
#include <stdint.h>
#include <math.h>

class TMC2209_Pulse
{
public:
    TMC2209_Pulse();

    bool init();

    void set_current_steps(int32_t steps);
    int32_t get_current_steps() const;

    void set_target_steps(int32_t steps);
    int32_t get_target_steps() const;

    bool is_running() const;
    float get_speed_sps() const;

    void start_move(float start_sps, float vmax, float amax, uint16_t pulse_us);
    void stop_move();

    static void tim6_cb(GPTDriver *gptp);

private:
    static TMC2209_Pulse *_driver;
    static constexpr uint32_t GPT_HZ = 1000000U;

    volatile bool _running = false;
    volatile bool _step_high = false;
    volatile bool _dir_positive = true;

    volatile int32_t _current_steps = 0;
    volatile int32_t _target_steps = 0;
    volatile float _speed_sps = 0.0f;

    float _start_sps = 1.0f;
    float _vmax_sps = 1.0f;
    float _amax_sps2 = 1.0f;
    uint16_t _pulse_us = 1;

    void timer_cb();

    static inline void step_pin_high() { palWriteLine(HAL_STEPPER_STEP_LINE, PAL_HIGH); }
    static inline void step_pin_low()  { palWriteLine(HAL_STEPPER_STEP_LINE, PAL_LOW); }

    static inline void dir_pin_write(bool positive)
    {
        palWriteLine(HAL_STEPPER_DIR_LINE, positive ? PAL_HIGH : PAL_LOW);
    }
};

#endif // AP_PERIPH_BALLBAY_ENABLED