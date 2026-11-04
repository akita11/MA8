// MA8 : ES7210 x 2, 8ch simultaneous sampling recorder
//   CH1..4 : ES7210 A (I2S0 master)   CH5..8 : ES7210 B (I2S1 slave)
//   SINGLE_ES7210 : ES7210 A only, 4ch (test build)
//   Boards : M5StampS3 in M5Capsule (ESP32-S3)   /   M5Stack Stamp-P4 (ESP32-P4)
//   Output : SD (WAV) / USB serial (binary) / USB Audio (S3: UAC1 FS, P4: UAC2 HS) / debug UART

#include <Arduino.h>

#include "capture.h"
#include "config.h"
#include "serial_stream.h"
#include "usb_audio.h"
#include "wav_writer.h"

#if USE_M5UNIFIED
#include <M5Unified.h>
#endif

#if defined(MA8_BOARD_STAMPP4)
#include <SD_MMC.h>
static fs::FS& g_fs = SD_MMC;
// On-board USB-C = USB Serial/JTAG (Arduino defines HWCDCSerial only with ARDUINO_USB_MODE=1)
static HWCDC JtagSerial;
#else
#include <SD.h>
#include <SPI.h>
static fs::FS& g_fs = SD;
#endif

#if ARDUINO_USB_MODE != 0
#error "Build with -DARDUINO_USB_MODE=0 (TinyUSB). See platformio.ini"
#endif

// ============================================================================ state
static uint8_t g_mode = DEFAULT_OUT_MODE;
static bool g_running = false;           // SD recording / serial streaming / debug UART
static uint32_t g_start_ms = 0;
static char g_msg[64] = "";

static const char* modeName(uint8_t m) {
    switch (m) {
        case OUT_MODE_SD: return "SD";
        case OUT_MODE_SERIAL: return "SERIAL";
        case OUT_MODE_USB_AUDIO: return "USB AUDIO";
        case OUT_MODE_DEBUG_UART: return "UART DBG";
    }
    return "?";
}

// Logs go to the USB CDC, except while binary data is streamed on it.
// Stamp-P4: also to the on-board USB-C (USB Serial/JTAG), which works without the HS connector.
static void logf(const char* fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (!(g_mode == OUT_MODE_SERIAL && g_running)) Serial.println(buf);
#if defined(MA8_BOARD_STAMPP4)
    JtagSerial.println(buf);
#endif
}

static void setMsg(const char* s) {
    strncpy(g_msg, s, sizeof(g_msg) - 1);
    g_msg[sizeof(g_msg) - 1] = 0;
    logf("%s", s);
}

// ============================================================================ SD (WAV)
static bool g_sd_ok = false;
static WavWriter g_wav;
static uint32_t g_session = 0;
static uint32_t g_part = 0;
static uint64_t g_file_frames = 0;
static uint32_t g_last_flush = 0;
static char g_path[40] = "";

static bool sdMount() {
    if (g_sd_ok) return true;
#if defined(MA8_BOARD_STAMPP4)
    // SDMMC slot 1 through the GPIO matrix, 4bit, card powered from 3.3V (no on-chip LDO)
    SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, PIN_SD_D1, PIN_SD_D2, PIN_SD_D3);
    g_sd_ok = SD_MMC.begin("/sdcard", false, false, SD_MMC_KHZ);
#else
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    g_sd_ok = SD.begin(PIN_SD_CS, SPI, SD_SPI_HZ);
#endif
    if (g_sd_ok && !g_fs.exists(SD_DIR)) g_fs.mkdir(SD_DIR);
    return g_sd_ok;
}

static uint32_t nextSession() {
    uint32_t mx = 0;
    File dir = g_fs.open(SD_DIR);
    if (!dir) return 1;
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
        const char* n = f.name();
        const char* b = strrchr(n, '/');
        b = b ? b + 1 : n;
        if (b[0] == 'R' && isdigit((unsigned char)b[1])) {
            uint32_t s = strtoul(b + 1, nullptr, 10);
            if (s > mx) mx = s;
        }
        f.close();
    }
    dir.close();
    return mx + 1;
}

static bool openPart() {
    snprintf(g_path, sizeof(g_path), SD_DIR "/R%04lu_%02lu.WAV", (unsigned long)g_session, (unsigned long)g_part);
    g_file_frames = 0;
    g_last_flush = millis();
    return g_wav.open(g_fs, g_path, SAMPLE_RATE_HZ, NUM_CH, SAMPLE_BITS);
}

static bool sdStart() {
    if (!sdMount()) {
        setMsg("SD mount failed");
        return false;
    }
    g_session = nextSession();
    g_part = 0;
    if (!openPart()) {
        setMsg("WAV open failed");
        return false;
    }
    capture::ring().discard();
    capture::resetCounters();
    capture::setSink(true);
    setMsg(g_path);
    return true;
}

// write up to max_chunks chunks; flush_all: write everything that is left
static void sdService(bool flush_all) {
    FrameRing& r = capture::ring();
    const uint64_t split = (uint64_t)FILE_SPLIT_SEC * SAMPLE_RATE_HZ;
    int chunks = flush_all ? 100000 : 4;
    while (chunks-- > 0) {
        size_t avail = r.available();
        if (avail == 0) break;
        if (!flush_all && avail < SD_WRITE_FRAMES) break;
        const uint8_t* p;
        size_t n = r.peek(&p);
        if (n > SD_WRITE_FRAMES) n = SD_WRITE_FRAMES;
        if (split && g_file_frames + n > split) n = (size_t)(split - g_file_frames);
        size_t w = g_wav.write(p, n * FRAME_BYTES);
        if (w != n * FRAME_BYTES) setMsg("SD write error");
        r.consume(n);
        g_file_frames += n;
        if (split && g_file_frames >= split) {
            g_wav.close();
            g_part++;
            if (!openPart()) setMsg("WAV open failed");
        }
    }
    if (millis() - g_last_flush > FILE_FLUSH_MS) {
        g_wav.updateHeader();
        g_last_flush = millis();
    }
}

static void sdStop() {
    capture::setSink(false);
    sdService(true);
    g_wav.close();
    char m[64];
    snprintf(m, sizeof(m), "saved %s", g_path);
    setMsg(m);
}

// ============================================================================ serial stream
static uint32_t g_seq = 0;
static uint8_t g_pkt_frames[SERIAL_PKT_FRAMES * FRAME_BYTES];

static void serialStart() {
    capture::ring().discard();
    capture::resetCounters();
    g_seq = 0;
    Serial.flush();
    capture::setSink(true);
}

static void serialStop() { capture::setSink(false); }

static void serialService() {
    FrameRing& r = capture::ring();
    int budget = 16;
    while (budget-- > 0 && r.available() >= SERIAL_PKT_FRAMES) {
        size_t n = r.read(g_pkt_frames, SERIAL_PKT_FRAMES);
        if (!serial_stream::sendPacket(g_pkt_frames, n, g_seq++)) break;
    }
}

// ============================================================================ debug UART (text, ~1 kHz)
static HardwareSerial& g_dbg = Serial1;
static int64_t g_dbg_acc[DEBUG_CH];
static uint32_t g_dbg_n = 0;

static inline int32_t sampleAt(const uint8_t* p) {
#if OUT_BYTES == 2
    return *(const int16_t*)p;
#elif OUT_BYTES == 3
    return (int32_t)((uint32_t)p[0] << 8 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 24) >> 8;
#else
    int32_t v;
    memcpy(&v, p, 4);
    return v;
#endif
}

static void dbgStart() {
    static bool begun = false;
    if (!begun) {
        g_dbg.setTxBufferSize(4096);
        g_dbg.begin(DEBUG_UART_BAUD, SERIAL_8N1, PIN_DEBUG_UART_RX, PIN_DEBUG_UART_TX);
        begun = true;
    }
    capture::ring().discard();
    capture::resetCounters();
    memset(g_dbg_acc, 0, sizeof(g_dbg_acc));
    g_dbg_n = 0;
    // label line (Arduino serial plotter etc.)
    for (int c = 0; c < DEBUG_CH; c++) g_dbg.printf("CH%d%c", DEBUG_CH_FIRST + c + 1, c + 1 < DEBUG_CH ? ',' : '\n');
    capture::setSink(true);
    logf("debug UART: TX=G%d %d baud, CH%d-%d, %.1f Hz (Fs %d / %d, %s)", PIN_DEBUG_UART_TX, DEBUG_UART_BAUD,
         DEBUG_CH_FIRST + 1, DEBUG_CH_FIRST + DEBUG_CH, SAMPLE_RATE_HZ / (float)DEBUG_DECIM, SAMPLE_RATE_HZ,
         DEBUG_DECIM, DEBUG_AVERAGE == 2 ? "peak" : DEBUG_AVERAGE ? "mean" : "pick");
}

static void dbgStop() { capture::setSink(false); }

static void dbgService() {
    FrameRing& r = capture::ring();
    char line[512];
    size_t len = 0;
    size_t budget = 4096;  // frames per call
    while (budget > 0) {
        const uint8_t* p;
        size_t n = r.peek(&p);
        if (n == 0) break;
        if (n > budget) n = budget;
        for (size_t i = 0; i < n; i++, p += FRAME_BYTES) {
            const uint8_t* s = p + DEBUG_CH_FIRST * OUT_BYTES;
            g_dbg_n++;
            for (int c = 0; c < DEBUG_CH; c++) {
                int64_t x = sampleAt(s + c * OUT_BYTES);
#if DEBUG_AVERAGE == 2
                if (x < 0) x = -x;
                if (x > g_dbg_acc[c]) g_dbg_acc[c] = x;
#elif DEBUG_AVERAGE
                g_dbg_acc[c] += x;
#else
                if (g_dbg_n == DEBUG_DECIM) g_dbg_acc[c] = x;
#endif
            }
            if (g_dbg_n < DEBUG_DECIM) continue;
            for (int c = 0; c < DEBUG_CH; c++) {
                int64_t v = (DEBUG_AVERAGE == 1) ? g_dbg_acc[c] / DEBUG_DECIM : g_dbg_acc[c];
                if (v > INT32_MAX) v = INT32_MAX;  // peak of -2^31
                len += snprintf(line + len, sizeof(line) - len, "%ld%c", (long)v, c + 1 < DEBUG_CH ? ',' : '\n');
                g_dbg_acc[c] = 0;
            }
            g_dbg_n = 0;
            if (len > sizeof(line) - 128) {  // room for one more line (8ch x 12 chars)
                g_dbg.write((const uint8_t*)line, len);
                len = 0;
            }
        }
        r.consume(n);
        budget -= n;
    }
    if (len) g_dbg.write((const uint8_t*)line, len);
}

// ============================================================================ run control
static void startRun() {
    if (g_running) return;
    bool ok = false;
    if (g_mode == OUT_MODE_SD) ok = sdStart();
    else if (g_mode == OUT_MODE_SERIAL) {
        serialStart();
        ok = true;
    } else if (g_mode == OUT_MODE_DEBUG_UART) {
        dbgStart();
        ok = true;
    }
    if (ok) {
        g_running = true;
        g_start_ms = millis();
    }
}

static void stopRun() {
    if (!g_running) return;
    if (g_mode == OUT_MODE_SD) sdStop();
    else if (g_mode == OUT_MODE_SERIAL) serialStop();
    else if (g_mode == OUT_MODE_DEBUG_UART) dbgStop();
    g_running = false;
}

static void setMode(uint8_t m) {
    if (g_running) stopRun();
    if (g_mode == OUT_MODE_USB_AUDIO && m != OUT_MODE_USB_AUDIO) {
        usb_audio::setActive(false);
        delay(5);                    // let the USB task finish the current packet
        capture::setSink(false);
        capture::ring().discard();   // loop task is the ring consumer again
    }
    g_mode = m;
    if (g_mode == OUT_MODE_USB_AUDIO) {
        capture::setSink(false);
        usb_audio::setActive(true);
        if (!usb_audio::available()) setMsg("USB audio: rate/bits too high");
    }
    char s[48];
    snprintf(s, sizeof(s), "mode: %s", modeName(g_mode));
    setMsg(s);
}

[[maybe_unused]] static void nextMode() { setMode((g_mode + 1) % NUM_OUT_MODES); }

static float g_peak[NUM_CH];

static void printInfo(Print& out) {
    const auto& st = capture::status();
    out.printf(
        "{\"device\":\"MA8\",\"board\":\"%s\",\"rate\":%d,\"bits\":%d,\"channels\":%d,\"bytes\":%d,"
        "\"gain\":%.1f,\"es7210_a\":%s,\"es7210_b\":%s,\"bclk_stops\":%s,\"mode\":\"%s\",\"usb_audio\":%s}\n",
#if defined(MA8_BOARD_STAMPP4)
        "Stamp-P4",
#else
        "StampS3",
#endif
        SAMPLE_RATE_HZ, SAMPLE_BITS, NUM_CH, OUT_BYTES, capture::gain(), st.es_a ? "true" : "false",
        st.es_b ? "true" : "false", st.bclk_stops ? "true" : "false", modeName(g_mode),
        usb_audio::available() ? "true" : "false");
}

// one status line with the peak level of each channel since the last UI update [dBFS]
static void printLevels(Print& out) {
    char s[160];
    int len = snprintf(s, sizeof(s), "%s %s drop:%lu ur:%lu |", modeName(g_mode),
                       g_running ? "RUN" : (usb_audio::streaming() ? "STREAM" : "IDLE"),
                       (unsigned long)capture::droppedFrames(), (unsigned long)usb_audio::underruns());
    for (int c = 0; c < NUM_CH && len < (int)sizeof(s) - 8; c++) {
        float db = g_peak[c] > 1e-6f ? 20.0f * log10f(g_peak[c]) : -120.0f;
        len += snprintf(s + len, sizeof(s) - len, " %.0f", db < -99 ? -99.0f : db);
    }
    out.println(s);
}

// commands from the PC (one character, newline optional):
//   s: SERIAL start   r: SD record start   d: debug UART start   u: USB AUDIO mode   x: stop
//   i: info (JSON)    l: levels             g<dB>\n: gain
static void handleCommands(Stream& in) {
    static char line[24];
    static int len = 0;
    while (in.available()) {
        int ch = in.read();
        if (ch < 0) break;
        if (ch == 's') {
            if (g_mode != OUT_MODE_SERIAL) setMode(OUT_MODE_SERIAL);
            startRun();
            len = 0;
        } else if (ch == 'd') {
            if (g_mode != OUT_MODE_DEBUG_UART) setMode(OUT_MODE_DEBUG_UART);
            startRun();
            len = 0;
        } else if (ch == 'r') {
            if (g_mode != OUT_MODE_SD) setMode(OUT_MODE_SD);
            startRun();
            len = 0;
        } else if (ch == 'u') {
            if (g_mode != OUT_MODE_USB_AUDIO) setMode(OUT_MODE_USB_AUDIO);
            len = 0;
        } else if (ch == 'x') {
            stopRun();
            len = 0;
        } else if (ch == 'i' && !g_running) {
            printInfo(in);
            len = 0;
        } else if (ch == 'l' && !(g_mode == OUT_MODE_SERIAL && g_running)) {
            printLevels(in);
            len = 0;
        } else if (ch == '\n' || ch == '\r') {
            line[len] = 0;
            if (len > 1 && line[0] == 'g') capture::setGain(atof(line + 1));
            len = 0;
        } else if (len < (int)sizeof(line) - 1) {
            line[len++] = (char)ch;
        }
    }
}

static void handleSerialCommands() {
    handleCommands(Serial);
#if defined(MA8_BOARD_STAMPP4)
    handleCommands(JtagSerial);
#endif
    // host closed the port while streaming
    if (g_mode == OUT_MODE_SERIAL && g_running && !serial_stream::connected()) stopRun();
}

// ============================================================================ UI
#if USE_M5UNIFIED  // ----- StampS3: one button + RGB LED
static void drawUI() {
    uint8_t r = 0, g = 0, b = 0;
    bool blink = (millis() / 250) & 1;
    switch (g_mode) {
        case OUT_MODE_SD: g = 40; break;           // green
        case OUT_MODE_SERIAL: b = 40; break;       // blue
        case OUT_MODE_USB_AUDIO: r = 30; b = 30; break;  // purple
        case OUT_MODE_DEBUG_UART: r = 20; g = 20; b = 20; break;  // white
    }
    bool act = g_running || (g_mode == OUT_MODE_USB_AUDIO && usb_audio::streaming());
    if (act) { r = blink ? 80 : 0; }
#ifdef SINGLE_ES7210
    bool es_ok = capture::status().es_a;
#else
    bool es_ok = capture::status().es_a && capture::status().es_b;
#endif
    if (!es_ok) { r = 80; g = 20; b = 0; }  // orange: ES7210 NG
    M5.Led.setAllColor(r, g, b);
    M5.Led.display();
}

static void handleInput() {
    // click: start / stop    hold (1 s): next mode
    if (M5.BtnA.wasHold()) {
        if (!g_running) nextMode();
    } else if (M5.BtnA.wasClicked()) {
        if (g_running) stopRun();
        else if (g_mode != OUT_MODE_USB_AUDIO) startRun();
    }
}
#else  // ----- Stamp-P4: no button / LED, serial commands only
static void drawUI() {}
static void handleInput() {}
#endif

// ============================================================================ setup / loop
void setup() {
#if USE_M5UNIFIED
    auto cfg = M5.config();
    cfg.internal_mic = false;  // I2S0/I2S1 are used by MA8
    cfg.internal_spk = false;
    cfg.internal_imu = false;
    M5.begin(cfg);
    M5.BtnA.setHoldThresh(1000);
#endif
    Serial.begin(115200);
#if defined(MA8_BOARD_STAMPP4)
    JtagSerial.begin(115200);
#endif
    delay(300);

    logf("MA8: %d Hz, %d bit, %d ch, %d bytes/s", SAMPLE_RATE_HZ, SAMPLE_BITS, NUM_CH, BYTES_PER_SEC);
    if (!capture::begin()) logf("capture init failed");
    const auto& st = capture::status();
#ifdef SINGLE_ES7210
    logf("SINGLE_ES7210: ES7210 A only (CH1..4), I2S1 / ES7210 B not used");
    logf("ES7210 A(0x%02X): %s", ES7210_ADDR_A, st.es_a ? "OK" : "NG");
#else
    logf("ES7210 A(0x%02X): %s  B(0x%02X): %s", ES7210_ADDR_A, st.es_a ? "OK" : "NG", ES7210_ADDR_B,
         st.es_b ? "OK" : "NG");
    logf("BCLK stops while I2S0 stopped: %s (A/B start alignment %s)", st.bclk_stops ? "yes" : "no",
         st.bclk_stops ? "guaranteed" : "NOT guaranteed - check with tools/ma8_serial_rx.py --sync-check");
#endif
    logf("MCLK = %d x Fs", MCLK_MULTIPLE);
    logf("ring: %u frames (%.2f s)", (unsigned)capture::ring().capacity(),
         capture::ring().capacity() / (float)SAMPLE_RATE_HZ);
    logf("USB audio: %s (max packet %d bytes)", usb_audio::available() ? "available" : "disabled",
         UAC_MAX_PKT_BYTES);

    if (DEFAULT_OUT_MODE == OUT_MODE_SD) sdMount();
    setMode(DEFAULT_OUT_MODE);
}

void loop() {
#if USE_M5UNIFIED
    M5.update();
#endif
    handleInput();
    handleSerialCommands();

    if (g_running) {
        if (g_mode == OUT_MODE_SD) sdService(false);
        else if (g_mode == OUT_MODE_SERIAL) serialService();
        else if (g_mode == OUT_MODE_DEBUG_UART) dbgService();
    }
    if (g_mode == OUT_MODE_USB_AUDIO) {
        // ring is fed only while the host is recording
        bool want = usb_audio::streaming();
        if (want != capture::sink()) capture::setSink(want);
    }

    static uint32_t last_ui = 0;
    if (millis() - last_ui >= UI_UPDATE_MS) {
        last_ui = millis();
        capture::takePeaks(g_peak);
        drawUI();
    }
    delay(1);
}
