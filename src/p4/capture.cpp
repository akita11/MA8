// Stamp-P4 (ESP32-P4, Arduino 3.x / ESP-IDF 5 I2S TDM driver)
//   I2S0 : master, outputs MCLK / BCLK / LRCK, receives ES7210 A (CH1..4)
//   I2S1 : slave, BCLK / LRCK taken from the same pads, receives ES7210 B (CH5..8)
#include "../capture.h"

#include <Arduino.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <driver/i2s_tdm.h>
#include <esp_heap_caps.h>
#include <esp_rom_gpio.h>
#include <soc/i2s_periph.h>

#include "../es7210.h"
#include "config.h"

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

static i2s_chan_handle_t s_rxA = nullptr;
static uint8_t* s_bufA = nullptr;
#ifndef SINGLE_ES7210
static i2s_chan_handle_t s_rxB = nullptr;
static uint8_t* s_bufB = nullptr;
#endif
static uint8_t* s_out = nullptr;

// -------------------------------------------------------------------------------- I2S
static bool newChannel(i2s_port_t port, i2s_role_t role, int din, i2s_chan_handle_t* h) {
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(port, role);
    cc.dma_desc_num = I2S_DMA_BUFS;
    cc.dma_frame_num = I2S_DMA_FRAMES;
    if (i2s_new_channel(&cc, nullptr, h) != ESP_OK) return false;

    const bool master = (role == I2S_ROLE_MASTER);
    i2s_tdm_config_t tc = {
        .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
            (i2s_data_bit_width_t)SLOT_BITS, I2S_SLOT_MODE_STEREO,
            (i2s_tdm_slot_mask_t)(I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3)),
        .gpio_cfg =
            {
                .mclk = master ? (gpio_num_t)PIN_I2S_MCLK : I2S_GPIO_UNUSED,
                .bclk = (gpio_num_t)PIN_I2S_BCLK,
                .ws = (gpio_num_t)PIN_I2S_LRCK,
                .dout = I2S_GPIO_UNUSED,
                .din = (gpio_num_t)din,
                .invert_flags = {},
            },
    };
    tc.clk_cfg.mclk_multiple = (i2s_mclk_multiple_t)MCLK_MULTIPLE;  // ES7210 clock setting assumes this
    return i2s_channel_init_tdm_mode(*h, &tc) == ESP_OK;
}

static bool installI2S() {
#ifndef SINGLE_ES7210
    // I2S1 first (slave, BCLK/LRCK as inputs), then I2S0 (master) takes the pads as outputs
    if (!newChannel(I2S_NUM_1, I2S_ROLE_SLAVE, PIN_I2S_DIN_B, &s_rxB)) return false;
#endif
    if (!newChannel(I2S_NUM_0, I2S_ROLE_MASTER, PIN_I2S_DIN_A, &s_rxA)) return false;

#ifndef SINGLE_ES7210
    // Enable input + output on the pads and route: I2S0 clock out -> pad -> I2S1 clock in.
    const int pins[2] = {PIN_I2S_BCLK, PIN_I2S_LRCK};
    const uint32_t out_sig[2] = {i2s_periph_signal[0].m_rx_bck_sig, i2s_periph_signal[0].m_rx_ws_sig};
    const uint32_t in_sig[2] = {i2s_periph_signal[1].s_rx_bck_sig, i2s_periph_signal[1].s_rx_ws_sig};
    for (int i = 0; i < 2; i++) {
        gpio_input_enable((gpio_num_t)pins[i]);
        esp_rom_gpio_connect_out_signal(pins[i], out_sig[i], false, false);
        esp_rom_gpio_connect_in_signal(pins[i], in_sig[i], false);
    }
#endif
    return true;
}

#ifdef SINGLE_ES7210
// Only I2S0: no A/B alignment needed. i2s_channel_enable() resets DMA and FIFO.
static void syncRestart() {
    i2s_channel_disable(s_rxA);
    i2s_channel_enable(s_rxA);
}
#else
static bool bclkToggling() {
    int last = gpio_get_level((gpio_num_t)PIN_I2S_BCLK);
    int edges = 0;
    for (int i = 0; i < 4000; i++) {
        int v = gpio_get_level((gpio_num_t)PIN_I2S_BCLK);
        if (v != last) {
            edges++;
            last = v;
        }
    }
    return edges > 8;
}

// Stop both ports, then start slave first and master second, so that the first frame
// captured by I2S0 and I2S1 is the same LRCK frame (requires BCLK to stop while I2S0 is stopped).
static void syncRestart() {
    i2s_channel_disable(s_rxA);
    i2s_channel_disable(s_rxB);
    delayMicroseconds(200);
    s_st.bclk_stops = !bclkToggling();
    i2s_channel_enable(s_rxB);  // slave: waits for clocks
    i2s_channel_enable(s_rxA);  // master: clocks start -> both begin at the same frame
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
#ifdef SINGLE_ES7210
            int32_t v = a[f * 4 + c];
#else
            int32_t v = (c < 4) ? a[f * 4 + c] : b[f * 4 + (c - 4)];
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
        i2s_channel_read(s_rxA, s_bufA, I2S_BLOCK_BYTES, &na, portMAX_DELAY);
#ifndef SINGLE_ES7210
        size_t nb = 0;
        i2s_channel_read(s_rxB, s_bufB, na, &nb, 50);
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
    // start the clocks: MCLK must be running before the ES7210 is configured
#ifndef SINGLE_ES7210
    i2s_channel_enable(s_rxB);
#endif
    i2s_channel_enable(s_rxA);
    delay(10);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
    s_st.es_a = s_esA.begin(ES7210_ADDR_A, SAMPLE_RATE_HZ, SLOT_BITS, s_gain, MIC_BIAS_MV, MCLK_MULTIPLE);
#ifndef SINGLE_ES7210
    s_st.es_b = s_esB.begin(ES7210_ADDR_B, SAMPLE_RATE_HZ, SLOT_BITS, s_gain, MIC_BIAS_MV, MCLK_MULTIPLE);
#endif
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

}  // namespace capture
