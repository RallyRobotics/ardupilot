#include "AP_Periph.h"

#if AP_PERIPH_BALLBAY_ENABLED
#include "TMC2209_Serial.h"

extern const AP_HAL::HAL& hal;

bool TMC2209_Serial::open_port()
{
    // SERIAL_ORDER EMPTY USART1 => hal.serial(1) is USART1
    uart = hal.serial(1);
    if (uart == nullptr) {
        return false;
    }

    uart->configure_parity(0);
    uart->set_stop_bits(1);
    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    uart->set_options(uart->get_options() |
                      AP_HAL::UARTDriver::OPTION_NODMA_TX |
                      AP_HAL::UARTDriver::OPTION_NODMA_RX);
    uart->begin(BAUD, 64, 64);
    uart->discard_input();

    return true;
}

uint8_t TMC2209_Serial::crc8_atm(const uint8_t *data, uint8_t len_without_crc)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len_without_crc; i++) {
        uint8_t current_byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (((crc >> 7) & 0x01U) ^ (current_byte & 0x01U)) {
                crc = uint8_t((crc << 1) ^ 0x07U);
            } else {
                crc = uint8_t(crc << 1);
            }
            current_byte >>= 1;
        }
    }

    return crc;
}

uint8_t TMC2209_Serial::mres_code_from_microsteps(uint16_t microsteps)
{
    switch (microsteps) {
    case 256: return 0;
    case 128: return 1;
    case 64:  return 2;
    case 32:  return 3;
    case 16:  return 4;
    case 8:   return 5;
    case 4:   return 6;
    case 2:   return 7;
    case 1:   return 8;
    default:  return 4; // default to 16 microsteps
    }
}

bool TMC2209_Serial::write_reg(Reg reg, uint32_t value)
{
    if (uart == nullptr) {
        return false;
    }

    uint8_t pkt[8] {};
    pkt[0] = SYNC;
    pkt[1] = DRIVER_ADDR;
    pkt[2] = uint8_t((uint8_t(reg) & 0x7FU) | 0x80U); // write access
    pkt[3] = uint8_t((value >> 24) & 0xFFU);
    pkt[4] = uint8_t((value >> 16) & 0xFFU);
    pkt[5] = uint8_t((value >> 8) & 0xFFU);
    pkt[6] = uint8_t(value & 0xFFU);
    pkt[7] = crc8_atm(pkt, 7);

    uart->discard_input();
    uart->write(pkt, sizeof(pkt));
    uart->flush();

    return true;
}

bool TMC2209_Serial::write_reg_with_delay(Reg reg, uint32_t value)
{
    if (!write_reg(reg, value)) {
        return false;
    }

    hal.scheduler->delay(WRITE_DELAY_MS);
    return true;
}

uint32_t TMC2209_Serial::build_gconf()
{
    uint32_t v = 0;

    // bit 6: pdn_disable = 1
    // bit 7: mstep_reg_select = 1
    // bit 8: multistep_filt = 1
    v |= (1U << 6);
    v |= (1U << 7);
    v |= (1U << 8);

    return v;
}

uint32_t TMC2209_Serial::build_nodeconf()
{
    // SENDDELAY = 2
    return (2U << 8);
}

uint32_t TMC2209_Serial::build_ihold_irun(const Config &cfg)
{
    const uint32_t ihold = uint32_t(cfg.ihold) & 0x1FU;
    const uint32_t irun = uint32_t(cfg.irun) & 0x1FU;
    const uint32_t iholddelay = 8U;

    return
        (ihold      << 0)  |
        (irun       << 8)  |
        (iholddelay << 16);
}

uint32_t TMC2209_Serial::build_chopconf(const Config &cfg)
{
    const uint32_t mres = uint32_t(
        mres_code_from_microsteps(cfg.microsteps)
    ) & 0x0FU;

    const uint32_t vsense = cfg.vsense ? 1U : 0U;

    return
        (1U     << 28) |
        (mres   << 24) |
        (vsense << 17) |
        (2U     << 15) |
        (0U     << 7)  |
        (4U     << 4)  |
        (5U     << 0);
}


uint32_t TMC2209_Serial::build_coolconf()
{
    // Disable CoolStep completely: SEMIN = 0
    return 0U;
}

bool TMC2209_Serial::configure_driver(const Config &cfg)
{
    if (_configured) {
        return true;
    }

    if (uart == nullptr && !open_port()) {
        return false;
    }

    if (!write_reg_with_delay(Reg::GCONF, build_gconf())) {
        return false;
    }

    if (!write_reg_with_delay(Reg::NODECONF, build_nodeconf())) {
        return false;
    }

    if (!write_reg_with_delay(Reg::IHOLD_IRUN, build_ihold_irun(cfg))) {
        return false;
    }

    if (!write_reg_with_delay(Reg::TPOWERDOWN, 20U)) {
        return false;
    }

    if (!write_reg_with_delay(Reg::COOLCONF, build_coolconf())) {
        return false;
    }

    if (!write_reg_with_delay(Reg::CHOPCONF, build_chopconf(cfg))) {
        return false;
    }

    if (!write_reg_with_delay(Reg::TPWMTHRS, cfg.tpwmthrs)) {
        return false;
    }

    if (!write_reg_with_delay(Reg::TCOOLTHRS, cfg.tcoolthrs)) {
        return false;
    }

    if (!write_reg_with_delay(Reg::SGTHRS, uint32_t(cfg.sgthrs) & 0xFFU)) {
        return false;
    }

    _configured = true;
    return true;
}

#endif // AP_PERIPH_BALLBAY_ENABLED