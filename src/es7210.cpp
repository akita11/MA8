#include "es7210.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "log.h"

#if USE_M5UNIFIED && I2C_USE_M5_EX_I2C
#include <M5Unified.h>
static bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
    return M5.Ex_I2C.writeRegister8(addr, reg, val, I2C_FREQ_HZ);
}
static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* v) {
    return M5.Ex_I2C.readRegister(addr, reg, v, 1, I2C_FREQ_HZ);
}
static bool i2cProbe(uint8_t addr) { return M5.Ex_I2C.scanID(addr, I2C_FREQ_HZ); }
#else
#include <Wire.h>
static bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}
static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* v) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
    *v = Wire.read();
    return true;
}
static bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}
#endif

// ---- register map (subset)
enum : uint8_t {
    REG00_RESET = 0x00,
    REG01_CLK_OFF = 0x01,
    REG02_MAINCLK = 0x02,
    REG04_LRCK_DIVH = 0x04,
    REG05_LRCK_DIVL = 0x05,
    REG06_POWER_DOWN = 0x06,
    REG07_OSR = 0x07,
    REG08_MODE_CFG = 0x08,
    REG09_TIME_CTRL0 = 0x09,
    REG0A_TIME_CTRL1 = 0x0A,
    REG11_SDP_IF1 = 0x11,
    REG12_SDP_IF2 = 0x12,
    REG20_ADC34_HPF2 = 0x20,
    REG21_ADC34_HPF1 = 0x21,
    REG22_ADC12_HPF1 = 0x22,
    REG23_ADC12_HPF2 = 0x23,
    REG40_ANALOG = 0x40,
    REG41_MIC12_BIAS = 0x41,
    REG42_MIC34_BIAS = 0x42,
    REG43_MIC1_GAIN = 0x43,  // 0x43..0x46
    REG47_MIC1_POWER = 0x47, // 0x47..0x4A
    REG4B_MIC12_POWER = 0x4B,
    REG4C_MIC34_POWER = 0x4C,
};

// each write is retried I2C_WRITE_TRIES times (a noisy bus loses a transfer now and then)
bool ES7210::writeReg(uint8_t reg, uint8_t val) {
    if (reg < kNumRegs && reg != REG00_RESET) {
        exp_[reg] = val;
        written_[reg] = true;
    }
    for (int t = 0; t < I2C_WRITE_TRIES; t++) {
        if (i2cWrite(addr_, reg, val)) return true;
        retries_++;
        delayMicroseconds(200);
    }
    logf("  0x%02X REG%02X <- 0x%02X: NG %d times", addr_, reg, val, I2C_WRITE_TRIES);
    return false;
}

// A read is accepted when two successful reads give the same value (a noisy bus loses reads:
// the plain readRegister8() then returns 0x00, seen in the 'R' dumps). Up to I2C_WRITE_TRIES * 2 reads.
bool ES7210::readReg(uint8_t reg, uint8_t* val) {
    bool have = false;
    uint8_t last = 0;
    for (int t = 0; t < I2C_WRITE_TRIES * 2; t++) {
        uint8_t v;
        if (i2cRead(addr_, reg, &v)) {
            if (have && v == last) {
                *val = v;
                return true;
            }
            have = true;
            last = v;
        }
        delayMicroseconds(200);
    }
    return false;
}

// a failed read must not turn into a write of a wrong value
bool ES7210::updateBits(uint8_t reg, uint8_t mask, uint8_t val) {
    uint8_t v;
    if (!readReg(reg, &v)) {
        logf("  0x%02X REG%02X read NG", addr_, reg);
        return false;
    }
    v = (v & ~mask) | (val & mask);
    return writeReg(reg, v);
}

// bits compared by verify(): REG40 bit0 always reads 0 (0x43 written -> 0x42 read on both chips)
static uint8_t verifyMask(uint8_t reg) { return reg == REG40_ANALOG ? 0xFE : 0xFF; }

uint8_t ES7210::gainCode(float db) {
    // 0..11 : 0..33dB (3dB step), 12: 34.5dB, 13: 36dB, 14: 37.5dB
    if (db <= 0) return 0;
    if (db >= 37.5f) return 14;
    float best = 1e9f;
    uint8_t code = 0;
    for (uint8_t c = 0; c <= 14; c++) {
        float d = fabsf(gainFromCode(c) - db);
        if (d < best) { best = d; code = c; }
    }
    return code;
}

float ES7210::gainFromCode(uint8_t c) {
    if (c <= 11) return c * 3.0f;
    return 33.0f + (c - 11) * 1.5f;
}

static uint8_t biasCode(int mv) {
    static const int16_t tbl[8] = {2180, 2260, 2360, 2450, 2550, 2660, 2780, 2870};
    int best = 0;
    for (int i = 1; i < 8; i++)
        if (abs(tbl[i] - mv) < abs(tbl[best] - mv)) best = i;
    return (uint8_t)(best << 4);  // REG41/42 bit[6:4]
}

bool ES7210::setGain(float gain_db) {
    uint8_t g = gainCode(gain_db);
    bool ok = true;
    for (uint8_t i = 0; i < 4; i++) ok &= writeReg(REG43_MIC1_GAIN + i, 0x10 | g);  // bit4: PGA on
    return ok;
}

// log all devices on the bus (when the ES7210 does not answer)
void ES7210::scanBus() {
    char s[160];
    int len = snprintf(s, sizeof(s), "  I2C scan:");
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78 && len < (int)sizeof(s) - 6; a++) {
        if (i2cProbe(a)) {
            len += snprintf(s + len, sizeof(s) - len, " 0x%02X", a);
            found++;
        }
    }
    if (!found) snprintf(s + len, sizeof(s) - len, " no device (check SDA/SCL wiring, pull-ups, 3.3V)");
    logf("%s", s);
}

int ES7210::verify() {
    int bad = 0;
    for (uint8_t r = 0; r < kNumRegs; r++) {
        if (!written_[r]) continue;
        uint8_t v;
        if (!readReg(r, &v)) {
            logf("  0x%02X verify          REG%02X read NG", addr_, r);
            bad++;
        } else if ((v ^ exp_[r]) & verifyMask(r)) {
            logf("  0x%02X verify          REG%02X = 0x%02X (expected 0x%02X) NG", addr_, r, v, exp_[r]);
            bad++;
        }
    }
    return bad;
}

void ES7210::dump() {
    logf("ES7210 0x%02X registers ('*' = differs from the value written by MA8, '--' = read NG)", addr_);
    for (uint8_t r0 = 0; r0 < kNumRegs; r0 += 16) {
        char s[96];
        int len = snprintf(s, sizeof(s), "  %02X:", r0);
        for (uint8_t r = r0; r < r0 + 16; r++) {
            uint8_t v;
            if (!readReg(r, &v)) len += snprintf(s + len, sizeof(s) - len, " -- ");
            else
                len += snprintf(s + len, sizeof(s) - len, " %02X%c", v,
                                (written_[r] && ((v ^ exp_[r]) & verifyMask(r))) ? '*' : ' ');
        }
        logf("%s", s);
    }
    int bad = verify();
    logf("  0x%02X %s", addr_, bad ? "MISMATCH (see above)" : "all written registers ok");
}

bool ES7210::begin(uint8_t i2c_addr, uint32_t sample_rate, uint8_t word_bits, float gain_db, int bias_mv,
                   uint16_t mclk_multiple) {
    (void)sample_rate;  // MCLK = 256 * N * Fs -> same divider setting for every single-speed rate
    addr_ = i2c_addr;
    present_ = false;
    memset(written_, 0, sizeof(written_));
    bool ok = true;
    // one log line per step; stop at the first step with an I2C error
#define STEP(name, body)                                                                    \
    do {                                                                                    \
        ok = true;                                                                          \
        retries_ = 0;                                                                       \
        body;                                                                               \
        if (ok && retries_) logf("  0x%02X %-15s ok (%u retries)", addr_, name, retries_);  \
        else logf("  0x%02X %-15s %s", addr_, name, ok ? "ok" : "I2C write NG");            \
        if (!ok) return false;                                                              \
    } while (0)

    if (!i2cProbe(addr_)) {
        logf("  0x%02X probe          no ACK", addr_);
        scanBus();
        return false;
    }
    logf("  0x%02X probe          ACK", addr_);

    STEP("reset", {
        ok &= writeReg(REG00_RESET, 0xFF);
        delay(5);
        ok &= writeReg(REG00_RESET, 0x41);
        ok &= writeReg(REG01_CLK_OFF, 0x3F);     // all clocks off while configuring
        ok &= writeReg(REG09_TIME_CTRL0, 0x30);  // chip state cycle
        ok &= writeReg(REG0A_TIME_CTRL1, 0x30);  // power-on state cycle
    });

    STEP("HPF", {  // high pass filter (DC cut)
        ok &= writeReg(REG23_ADC12_HPF2, 0x2A);
        ok &= writeReg(REG22_ADC12_HPF1, 0x0A);
        ok &= writeReg(REG20_ADC34_HPF2, 0x0A);
        ok &= writeReg(REG21_ADC34_HPF1, 0x2A);
    });

    STEP("slave mode", ok &= updateBits(REG08_MODE_CFG, 0x01, 0x00));

    uint8_t bias = biasCode(bias_mv);
    STEP("analog/bias/PGA", {
        ok &= writeReg(REG40_ANALOG, 0xC3);
        ok &= writeReg(REG41_MIC12_BIAS, bias);
        ok &= writeReg(REG42_MIC34_BIAS, bias);
        ok &= setGain(gain_db);
        for (uint8_t i = 0; i < 4; i++) ok &= writeReg(REG47_MIC1_POWER + i, 0x08);
    });

    // clock: MCLK = 256N Fs -> adc_div=N, doubler=1, dll=1, OSR=0x20, LRCK div = 256N
    uint8_t adc_div = (mclk_multiple >= 512) ? 2 : 1;
    uint16_t lrck_div = 256 * adc_div;
    STEP("clock", {
        ok &= writeReg(REG02_MAINCLK, 0xC0 | adc_div);
        ok &= writeReg(REG07_OSR, 0x20);
        ok &= writeReg(REG04_LRCK_DIVH, lrck_div >> 8);
        ok &= writeReg(REG05_LRCK_DIVL, lrck_div & 0xFF);
    });

    // serial data port: I2S format + word length, TDM
    uint8_t wl = (word_bits == 16) ? 0x60 : 0x80;  // bit[7:5] 011:16bit 000:24bit 100:32bit
    STEP("TDM format", {
        ok &= writeReg(REG11_SDP_IF1, wl | 0x00);  // bit[1:0] 00: I2S
        ok &= writeReg(REG12_SDP_IF2, 0x02);       // TDM mode (1 LRCK / frame, I2S)
    });

    STEP("power/enable", {
        ok &= writeReg(REG06_POWER_DOWN, 0x04);  // DLL power down
        ok &= writeReg(REG4B_MIC12_POWER, 0x0F);
        ok &= writeReg(REG4C_MIC34_POWER, 0x0F);
        ok &= writeReg(REG00_RESET, 0x71);       // enable device
        ok &= writeReg(REG00_RESET, 0x41);
    });

    STEP("start", {  // all 4 mics
        ok &= writeReg(REG01_CLK_OFF, 0x20);     // enable ADC12/ADC34 clocks
        ok &= writeReg(REG06_POWER_DOWN, 0x00);
        ok &= writeReg(REG40_ANALOG, 0x43);
        for (uint8_t i = 0; i < 4; i++) ok &= writeReg(REG47_MIC1_POWER + i, 0x00);
        ok &= writeReg(REG4B_MIC12_POWER, 0x00);
        ok &= writeReg(REG4C_MIC34_POWER, 0x00);
        ok &= setGain(gain_db);
    });
#undef STEP

    // read back every written register: catches a write that was ACKed but not taken
    // (false -> the caller runs the whole init again)
    if (verify()) return false;
    logf("  0x%02X verify          ok (all written registers)", addr_);

    present_ = true;
    return true;
}
