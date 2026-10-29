#pragma once
// ES7210 (Everest Semiconductor) 4ch audio ADC - minimal driver
//   * I2S slave (MCLK / SCLK / LRCK are supplied by the ESP32)
//   * TDM (4 slots on SDOUT1/TDMOUT), I2S (Philips) framing
//   * MCLK = 256 x N x Fs (N = 1, 2), single speed (8k .. 48kHz)
//   * I2C: M5.Ex_I2C (USE_M5UNIFIED) or Arduino Wire (begun by the caller)
// Register sequence follows Espressif esp-codec-dev / ESP-ADF es7210 driver.

#include <stdint.h>

class ES7210 {
public:
    // word_bits: 16 or 32 (TDM slot width = word length), mclk_multiple: MCLK / Fs (256 or 512)
    bool begin(uint8_t i2c_addr, uint32_t sample_rate, uint8_t word_bits, float gain_db, int bias_mv,
               uint16_t mclk_multiple = 256);
    bool present() const { return present_; }
    bool setGain(float gain_db);            // all 4 channels
    uint8_t addr() const { return addr_; }
    uint8_t readReg(uint8_t reg);
    bool writeReg(uint8_t reg, uint8_t val);

    static uint8_t gainCode(float gain_db); // 0..14
    static float gainFromCode(uint8_t code);

private:
    bool updateBits(uint8_t reg, uint8_t mask, uint8_t val);
    uint8_t addr_ = 0x40;
    bool present_ = false;
};
