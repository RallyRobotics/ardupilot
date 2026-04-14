#pragma once

#include <AP_HAL/AP_HAL.h>

class TMC2209_Serial
{
public:
    struct Config {
        uint8_t ihold = 1;
        uint8_t irun = 20;
        uint16_t microsteps = 8;
        bool vsense = false;
        uint32_t tpwmthrs = 0;
        uint32_t tcoolthrs = 200;
        uint8_t sgthrs = 50;
    };

    TMC2209_Serial() = default;

    bool configure_driver(const Config &cfg);
    bool configured() const { return _configured; }

private:
    AP_HAL::UARTDriver *uart = nullptr;
    bool _configured = false;

    static constexpr uint32_t BAUD = 115200;
    static constexpr uint8_t  DRIVER_ADDR = 0;
    static constexpr uint8_t  SYNC = 0x05;
    static constexpr uint8_t  WRITE_DELAY_MS = 2;

    enum class Reg : uint8_t {
        GCONF      = 0x00,
        NODECONF   = 0x03,
        IHOLD_IRUN = 0x10,
        TPOWERDOWN = 0x11,
        TPWMTHRS   = 0x13,
        TCOOLTHRS  = 0x14,
        SGTHRS     = 0x40,
        COOLCONF   = 0x42,
        CHOPCONF   = 0x6C,
    };

    bool open_port();
    bool write_reg(Reg reg, uint32_t value);
    bool write_reg_with_delay(Reg reg, uint32_t value);

    static uint32_t build_gconf();
    static uint32_t build_nodeconf();
    static uint32_t build_ihold_irun(const Config &cfg);
    static uint32_t build_coolconf();
    static uint32_t build_chopconf(const Config &cfg);

    static uint8_t crc8_atm(const uint8_t *data, uint8_t len_without_crc);
    static uint8_t mres_code_from_microsteps(uint16_t microsteps);
};