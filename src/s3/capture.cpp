// StampS3 (ESP32-S3, Arduino 2.0.x / ESP-IDF 4.4 legacy I2S driver)
#include "../capture.h"

#include <Arduino.h>
#include <M5Unified.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>

#include "config.h"
#include "../es7210.h"
#include "../log.h"

namespace capture {

#define SLOT_BYTES       (SLOT_BITS / 8)
#define I2S_FRAME_BYTES  (4 * SLOT_BYTES)                 // 4 TDM slots of one ES7210
#define I2S_BLOCK_BYTES  (I2S_DMA_FRAMES * I2S_FRAME_BYTES)

static ES7210 s_esA, s_esB;
static Status s_st;
static FrameRing s_ring;
static float s_gain = MIC_GAIN_DB;

static volatile bool s_sink = false;
static volatile uint32_t s_dropped = 0;
static volatile uint64_t s_captured = 0;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_peak[NUM_CH];  // |sample| scaled to 32bit full scale

static uint8_t* s_bufA = nullptr;
#ifndef SINGLE_ES7210
static uint8_t* s_bufB = nullptr;
#endif
static uint8_t* s_out = nullptr;

// -------------------------------------------------------------------------------- I2S
static i2s_config_t makeConfig(bool master) {
    i2s_config_t c = {};
    c.mode = (i2s_mode_t)((master ? I2S_MODE_MASTER : I2S_MODE_SLAVE) | I2S_MODE_RX);
    c.sample_rate = SAMPLE_RATE_HZ;
    c.bits_per_sample = (i2s_bits_per_sample_t)SLOT_BITS;
    c.channel_format = I2S_CHANNEL_FMT_MULTIPLE;
    c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    c.intr_alloc_flags = 0;
    c.dma_buf_count = I2S_DMA_BUFS;
    c.dma_buf_len = I2S_DMA_FRAMES;
    c.use_apll = false;
    c.tx_desc_auto_clear = false;
    c.fixed_mclk = 0;
    c.mclk_multiple = I2S_MCLK_MULTIPLE_256;  // MCLK = 256 Fs (ES7210 clock setting assumes this)
    c.bits_per_chan = (i2s_bits_per_chan_t)SLOT_BITS;
    c.chan_mask = (i2s_channel_t)(I2S_TDM_ACTIVE_CH0 | I2S_TDM_ACTIVE_CH1 | I2S_TDM_ACTIVE_CH2 | I2S_TDM_ACTIVE_CH3);
    c.total_chan = 4;
    c.left_align = false;
    c.big_edin = false;
    c.bit_order_msb = false;
    c.skip_msk = false;
    return c;
}

static bool installI2S() {
#ifndef SINGLE_ES7210
    // I2S1 : slave, BCLK/LRCK come from I2S0 through the same pads
    i2s_config_t cb = makeConfig(false);
    if (i2s_driver_install(I2S_NUM_1, &cb, 0, nullptr) != ESP_OK) return false;
    i2s_pin_config_t pb = {};
    pb.mck_io_num = I2S_PIN_NO_CHANGE;
    pb.bck_io_num = PIN_I2S_BCLK;
    pb.ws_io_num = PIN_I2S_LRCK;
    pb.data_out_num = I2S_PIN_NO_CHANGE;
    pb.data_in_num = PIN_I2S_DIN_B;
    if (i2s_set_pin(I2S_NUM_1, &pb) != ESP_OK) return false;
#endif

    // I2S0 : master, outputs MCLK / BCLK / LRCK
    i2s_config_t ca = makeConfig(true);
    if (i2s_driver_install(I2S_NUM_0, &ca, 0, nullptr) != ESP_OK) return false;
    i2s_pin_config_t pa = {};
    pa.mck_io_num = PIN_I2S_MCLK;
    pa.bck_io_num = PIN_I2S_BCLK;
    pa.ws_io_num = PIN_I2S_LRCK;
    pa.data_out_num = I2S_PIN_NO_CHANGE;
    pa.data_in_num = PIN_I2S_DIN_A;
    if (i2s_set_pin(I2S_NUM_0, &pa) != ESP_OK) return false;

#ifndef SINGLE_ES7210
    // i2s_set_pin() made BCLK/LRCK output-only for I2S0 (input disabled).
    // Enable input + output on the pads and route: I2S0 clock out -> pad -> I2S1 clock in.
    const int pins[2] = {PIN_I2S_BCLK, PIN_I2S_LRCK};
    const uint32_t out_sig[2] = {I2S0I_BCK_OUT_IDX, I2S0I_WS_OUT_IDX};
    const uint32_t in_sig[2] = {I2S1I_BCK_IN_IDX, I2S1I_WS_IN_IDX};
    // ES7210_LRCK_INVERT: LRCK is inverted on the pad and inverted back into I2S1,
    // so only the ES7210s see the inverted LRCK (I2S0 and I2S1 stay in step).
    const bool inv[2] = {false, ES7210_LRCK_INVERT != 0};
    for (int i = 0; i < 2; i++) {
        gpio_set_direction((gpio_num_t)pins[i], GPIO_MODE_INPUT_OUTPUT);
        esp_rom_gpio_connect_out_signal(pins[i], out_sig[i], inv[i], false);
        esp_rom_gpio_connect_in_signal(pins[i], in_sig[i], inv[i]);
    }
#elif ES7210_LRCK_INVERT
    esp_rom_gpio_connect_out_signal(PIN_I2S_LRCK, I2S0I_WS_OUT_IDX, true, false);
#endif
    return true;
}

static void drain(i2s_port_t port) {
    size_t n;
    do {
        n = 0;
        i2s_read(port, s_bufA, I2S_BLOCK_BYTES, &n, 0);
    } while (n > 0);
}

#ifdef SINGLE_ES7210
// Only I2S0: no A/B alignment needed, just start with an empty DMA queue.
static void syncRestart() {
    i2s_stop(I2S_NUM_0);
    drain(I2S_NUM_0);
    i2s_start(I2S_NUM_0);
}
#else
static bool bclkToggling() {
    int last = gpio_get_level((gpio_num_t)PIN_I2S_BCLK);
    int edges = 0;
    for (int i = 0; i < 4000; i++) {
        int v = gpio_get_level((gpio_num_t)PIN_I2S_BCLK);
        if (v != last) { edges++; last = v; }
    }
    return edges > 8;
}

// Stop both ports, then start slave first and master second, so that the first frame
// captured by I2S0 and I2S1 is the same LRCK frame (requires BCLK to stop while I2S0 is stopped).
static void syncRestart() {
    i2s_stop(I2S_NUM_0);
    i2s_stop(I2S_NUM_1);
    delayMicroseconds(200);
    s_st.bclk_stops = !bclkToggling();
    drain(I2S_NUM_0);
    drain(I2S_NUM_1);
    i2s_start(I2S_NUM_1);  // slave: waits for clocks
    i2s_start(I2S_NUM_0);  // master: clocks start -> both begin at the same frame
}
#endif

// -------------------------------------------------------------------------------- task
static inline uint32_t uabs32(int32_t v) { return v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v; }

static size_t convert(size_t frames, uint32_t* pk) {
#if SLOT_BITS == 16
    typedef int16_t slot_t;
#else
    typedef int32_t slot_t;
#endif
    const slot_t* a = (const slot_t*)s_bufA;
#ifndef SINGLE_ES7210
    const slot_t* b = (const slot_t*)s_bufB;
#endif
    uint8_t* o = s_out;
    for (size_t f = 0; f < frames; f++) {
        for (int c = 0; c < NUM_CH; c++) {
            const uint8_t src = srcOfChannel(c);  // channel order: see ES7210_SLOT_OF_MIC in config.h
#ifdef SINGLE_ES7210
            int32_t v = a[f * 4 + src];
#else
            int32_t v = (src < 4) ? a[f * 4 + src] : b[f * 4 + (src - 4)];
#endif
#if SAMPLE_BITS == 16
            uint32_t m = uabs32(v) << 16;
            *(int16_t*)o = (int16_t)v;
            o += 2;
#elif SAMPLE_BITS == 24
            uint32_t m = uabs32(v);
            o[0] = (uint8_t)(v >> 8);
            o[1] = (uint8_t)(v >> 16);
            o[2] = (uint8_t)(v >> 24);
            o += 3;
#else
            uint32_t m = uabs32(v);
            memcpy(o, &v, 4);
            o += 4;
#endif
            if (m > pk[c]) pk[c] = m;
        }
    }
    return frames;
}

static void captureTask(void*) {
    for (;;) {
        size_t na = 0;
        i2s_read(I2S_NUM_0, s_bufA, I2S_BLOCK_BYTES, &na, portMAX_DELAY);
#ifndef SINGLE_ES7210
        size_t nb = 0;
        i2s_read(I2S_NUM_1, s_bufB, na, &nb, pdMS_TO_TICKS(50));
        if (nb < na) memset(s_bufB + nb, 0, na - nb);  // slave not receiving -> zeros
#endif
        size_t frames = na / I2S_FRAME_BYTES;
        if (frames == 0) continue;

        uint32_t pk[NUM_CH] = {0};
        convert(frames, pk);

        portENTER_CRITICAL(&s_mux);
        for (int c = 0; c < NUM_CH; c++)
            if (pk[c] > s_peak[c]) s_peak[c] = pk[c];
        portEXIT_CRITICAL(&s_mux);

        if (s_sink) {
            if (s_ring.push(s_out, frames)) s_captured += frames;
            else s_dropped += frames;
        }
    }
}

// -------------------------------------------------------------------------------- API
bool begin() {
    s_bufA = (uint8_t*)heap_caps_malloc(I2S_BLOCK_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#ifndef SINGLE_ES7210
    s_bufB = (uint8_t*)heap_caps_malloc(I2S_BLOCK_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_bufB) return false;
#endif
    s_out = (uint8_t*)heap_caps_malloc(I2S_DMA_FRAMES * FRAME_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_bufA || !s_out) return false;

    uint8_t* mem = nullptr;
    size_t bytes = 0;
    if (psramFound()) {
        bytes = RING_BYTES_PSRAM;
        mem = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!mem) {
        bytes = RING_BYTES_SRAM;
        mem = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!s_ring.init(mem, bytes, FRAME_BYTES)) return false;

    s_st.i2s_ok = installI2S();
    if (!s_st.i2s_ok) return false;
    delay(10);  // MCLK running before ES7210 configuration

    // ES7210 configuration over I2C: repeated until every chip is configured
    for (uint32_t attempt = 1;; attempt++) {
#if I2C_USE_M5_EX_I2C
        logf("I2C init #%lu: SDA=G%d SCL=G%d %d Hz (M5.Ex_I2C, no glitch filter)", (unsigned long)attempt,
             PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
        if (attempt > 1) M5.Ex_I2C.release();  // start again from a free bus
        if (!M5.Ex_I2C.begin(I2C_NUM_0, PIN_I2C_SDA, PIN_I2C_SCL)) logf("  I2C bus begin NG");
#else
        logf("I2C init #%lu: SDA=G%d SCL=G%d %d Hz (Wire, glitch filter %d)", (unsigned long)attempt, PIN_I2C_SDA,
             PIN_I2C_SCL, I2C_FREQ_HZ, I2C_GLITCH_FILTER_CYC);
        if (attempt > 1) Wire.end();  // start again from a free bus
        if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ)) logf("  I2C bus begin NG");
        if (i2c_filter_enable(I2C_NUM_0, I2C_GLITCH_FILTER_CYC) != ESP_OK) logf("  I2C glitch filter NG");
#endif
        if (!s_st.es_a) {
            logf("ES7210 A:");
            s_st.es_a = s_esA.begin(ES7210_ADDR_A, SAMPLE_RATE_HZ, SLOT_BITS, s_gain, MIC_BIAS_MV);
        }
#ifndef SINGLE_ES7210
        if (!s_st.es_b) {
            logf("ES7210 B:");
            s_st.es_b = s_esB.begin(ES7210_ADDR_B, SAMPLE_RATE_HZ, SLOT_BITS, s_gain, MIC_BIAS_MV);
        }
        if (s_st.es_a && s_st.es_b) break;
#else
        if (s_st.es_a) break;
#endif
        logf("I2C init failed, retry in %d ms", I2C_RETRY_MS);
        delay(I2C_RETRY_MS);
    }
    logf("I2C init done");
    delay(20);

    syncRestart();
    xTaskCreatePinnedToCore(captureTask, "capture", 4096, nullptr, CAPTURE_TASK_PRIO, nullptr, CAPTURE_TASK_CORE);
    return true;
}

const Status& status() { return s_st; }
FrameRing& ring() { return s_ring; }
void setSink(bool on) { s_sink = on; }
bool sink() { return s_sink; }
uint32_t droppedFrames() { return s_dropped; }
uint64_t capturedFrames() { return s_captured; }
void resetCounters() {
    s_dropped = 0;
    s_captured = 0;
}

void takePeaks(float* peak) {
    uint32_t p[NUM_CH];
    portENTER_CRITICAL(&s_mux);
    memcpy(p, s_peak, sizeof(p));
    memset(s_peak, 0, sizeof(s_peak));
    portEXIT_CRITICAL(&s_mux);
    for (int c = 0; c < NUM_CH; c++) peak[c] = p[c] / 2147483648.0f;
}

bool setGain(float db) {
    s_gain = ES7210::gainFromCode(ES7210::gainCode(db));
    bool ok = true;
    if (s_st.es_a) ok &= s_esA.setGain(s_gain);
    if (s_st.es_b) ok &= s_esB.setGain(s_gain);
    return ok;
}

float gain() { return s_gain; }

void dumpRegs() {
    s_esA.dump();
#ifndef SINGLE_ES7210
    s_esB.dump();
#endif
}

}  // namespace capture
