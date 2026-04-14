#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <stdint.h>

class NAU7802_ADC
{
public:
    NAU7802_ADC() = default;

    bool init();
    void update();

    bool healthy() const { return _healthy; }

    // Returns true only when a fresh summed load is available.
    bool read_load(int32_t &sum);

private:
    static constexpr uint8_t I2C_BUS_INDEX = 0;
    static constexpr uint8_t I2C_ADDR = 0x2A;
    static constexpr uint8_t I2C_RETRIES = 0;

    static constexpr uint8_t RATE_CODE_80SPS = 0x03;
    static constexpr uint8_t GAIN_128 = 128;

    enum class Reg : uint8_t {
        PU_CTRL = 0x00,
        CTRL1   = 0x01,
        CTRL2   = 0x02,
        ADCO_B2 = 0x12,
    };

    enum PU_CTRL_Bits : uint8_t {
        PU_CTRL_RR   = 1U << 0,
        PU_CTRL_PUD  = 1U << 1,
        PU_CTRL_PUA  = 1U << 2,
        PU_CTRL_PUR  = 1U << 3,
    };

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    bool _healthy = false;

    int32_t _ch1_working = 0;
    int32_t _ch2_working = 0;

    int32_t _load_sum = 0;
    bool _new_load = false;

    uint8_t _ctrl2_shadow = 0;
    uint8_t _active_channel = 1;

    bool write_reg(Reg reg, uint8_t value);
    bool read_reg(Reg reg, uint8_t &value);
    bool read_regs(Reg reg, uint8_t *buf, uint8_t len);

    bool wait_powerup_ready(uint32_t timeout_ms);
    bool configure_device();
    bool set_channel(uint8_t channel);
    bool read_sample(int32_t &sample);

    void publish_load();

    static uint8_t pga_bits_from_gain(uint8_t gain);
    static int32_t sign_extend24(uint32_t value);
};