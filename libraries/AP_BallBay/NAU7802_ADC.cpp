#include "NAU7802_ADC.h"

#if AP_BALLBAY_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

bool NAU7802_ADC::init()
{
    _dev = hal.i2c_mgr->get_device(I2C_BUS_INDEX, I2C_ADDR);
    if (!_dev) {
        _healthy = false;
        return false;
    }

    _dev->set_retries(I2C_RETRIES);

    {
        WITH_SEMAPHORE(_dev->get_semaphore());

        if (!write_reg(Reg::PU_CTRL, PU_CTRL_RR)) {
            _healthy = false;
            return false;
        }
        hal.scheduler->delay(1);

        if (!write_reg(Reg::PU_CTRL, PU_CTRL_PUD | PU_CTRL_PUA)) {
            _healthy = false;
            return false;
        }

        if (!wait_powerup_ready(100)) {
            _healthy = false;
            return false;
        }

        if (!configure_device()) {
            _healthy = false;
            return false;
        }

        if (!set_channel(1)) {
            _healthy = false;
            return false;
        }
    }

    _ch1_working = 0;
    _ch2_working = 0;
    _load_sum = 0;
    _new_load = false;
    _active_channel = 1;
    _healthy = true;

    return true;
}

void NAU7802_ADC::update()
{
    if (!_healthy || !_dev) {
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    int32_t sample = 0;
    if (!read_sample(sample)) {
        _healthy = false;
        return;
    }

    if (_active_channel == 1) {
        _ch1_working = sample;

        if (!set_channel(2)) {
            _healthy = false;
            return;
        }

        return;
    }

    _ch2_working = sample;
    publish_load();

    if (!set_channel(1)) {
        _healthy = false;
        return;
    }
}

bool NAU7802_ADC::read_load(int32_t &sum)
{
    if (!_new_load) {
        return false;
    }

    sum = _load_sum;
    _new_load = false;
    return true;
}

bool NAU7802_ADC::write_reg(Reg reg, uint8_t value)
{
    return _dev && _dev->write_register(uint8_t(reg), value);
}

bool NAU7802_ADC::read_reg(Reg reg, uint8_t &value)
{
    return _dev && _dev->read_registers(uint8_t(reg), &value, 1);
}

bool NAU7802_ADC::read_regs(Reg reg, uint8_t *buf, uint8_t len)
{
    return _dev && _dev->read_registers(uint8_t(reg), buf, len);
}

bool NAU7802_ADC::wait_powerup_ready(uint32_t timeout_ms)
{
    const uint32_t start_ms = AP_HAL::millis();

    while ((AP_HAL::millis() - start_ms) < timeout_ms) {
        uint8_t v = 0;
        if (!read_reg(Reg::PU_CTRL, v)) {
            return false;
        }

        if ((v & PU_CTRL_PUR) != 0U) {
            return true;
        }

        hal.scheduler->delay(1);
    }

    return false;
}

bool NAU7802_ADC::configure_device()
{
    if (!write_reg(Reg::CTRL1, pga_bits_from_gain(GAIN_128))) {
        return false;
    }

    // CHS = 0 initially, CRS = 011 => 80 SPS
    _ctrl2_shadow = uint8_t((RATE_CODE_80SPS & 0x07U) << 4);

    if (!write_reg(Reg::CTRL2, _ctrl2_shadow)) {
        return false;
    }

    return true;
}

bool NAU7802_ADC::set_channel(uint8_t channel)
{
    if (channel == 1) {
        _ctrl2_shadow &= ~uint8_t(0x80U);
    } else if (channel == 2) {
        _ctrl2_shadow |= uint8_t(0x80U);
    } else {
        return false;
    }

    _active_channel = channel;
    return write_reg(Reg::CTRL2, _ctrl2_shadow);
}

bool NAU7802_ADC::read_sample(int32_t &sample)
{
    uint8_t buf[3] {};
    if (!read_regs(Reg::ADCO_B2, buf, sizeof(buf))) {
        return false;
    }

    const uint32_t raw24 =
        (uint32_t(buf[0]) << 16) |
        (uint32_t(buf[1]) << 8)  |
        uint32_t(buf[2]);

    sample = sign_extend24(raw24);
    return true;
}

void NAU7802_ADC::publish_load()
{
    _load_sum = _ch1_working + _ch2_working;
    _new_load = true;
}

uint8_t NAU7802_ADC::pga_bits_from_gain(uint8_t gain)
{
    switch (gain) {
    case 128: return 0x07;
    case 64:  return 0x06;
    case 32:  return 0x05;
    case 16:  return 0x04;
    case 8:   return 0x03;
    case 4:   return 0x02;
    case 2:   return 0x01;
    case 1:
    default:  return 0x00;
    }
}

int32_t NAU7802_ADC::sign_extend24(uint32_t value)
{
    if ((value & 0x00800000U) != 0U) {
        value |= 0xFF000000U;
    }
    return int32_t(value);
}

#endif