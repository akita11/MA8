#pragma once
// =====================================================================================
// MA8 configuration
//   ES7210 x 2 (4ch + 4ch, TDM) -> I2S0 master + I2S1 slave -> 8ch simultaneous
//   Boards:
//     MA8_BOARD_STAMPS3 : M5StampS3 in M5Capsule (ESP32-S3, Arduino 2.0.x, USB FS, UAC1)
//     MA8_BOARD_STAMPP4 : M5Stack Stamp-P4      (ESP32-P4, Arduino 3.x,  USB HS, UAC2)
//   Output: microSD (WAV) / USB-Serial (binary packets) / USB Audio / debug UART
// =====================================================================================

// ------------------------------------------------------------------ sampling
// ES7210 single-speed range: 8000 .. 48000 Hz
//   8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
#ifndef SAMPLE_RATE_HZ
#define SAMPLE_RATE_HZ      32000
#endif

// (SAMPLE_RATE_HZ / SAMPLE_BITS / DEFAULT_OUT_MODE can also be given with -D in platformio.ini)

// Bits per sample of the OUTPUT data (WAV / serial / USB audio)
//   16 : ES7210 16bit word, I2S slot 16bit
//   24 : ES7210 32bit word (24bit valid), packed to 3 bytes
//   32 : ES7210 32bit word, stored as int32 (lower 8bit are 0)
#ifndef SAMPLE_BITS
#define SAMPLE_BITS         16
#endif

// Test build with only ES7210 A (I2S0 master, CH1..4). I2S1 and ES7210 B are not used.
// (can also be given with -DSINGLE_ES7210 in platformio.ini)
//#define SINGLE_ES7210

#ifdef SINGLE_ES7210
#define NUM_CH              4           // ES7210 A = CH1..4
#else
#define NUM_CH              8           // ES7210 A = CH1..4, ES7210 B = CH5..8
#endif

// ------------------------------------------------------------------ output mode
#define OUT_MODE_SD         0           // record to microSD as WAV
#define OUT_MODE_SERIAL     1           // stream binary packets over USB CDC
#define OUT_MODE_USB_AUDIO  2           // USB Audio microphone (StampS3: UAC1 FS, Stamp-P4: UAC2 HS)
#define OUT_MODE_DEBUG_UART 3           // debug: all ch decimated to ~1 kHz (8ch: 500 Hz), text (CSV) on a hardware UART
#define NUM_OUT_MODES       4

//#define DEFAULT_OUT_MODE OUT_MODE_DEBUG_UART // default output mode at boot (can be changed by button / touch)

#ifndef DEFAULT_OUT_MODE
#if defined(MA8_BOARD_STAMPS3)
#define DEFAULT_OUT_MODE    OUT_MODE_USB_AUDIO  // mode at boot (G0 long press: USB Audio <-> SD)
#else
#define DEFAULT_OUT_MODE    OUT_MODE_SD // mode at boot (can be changed by button / touch)
#endif
#endif

// ------------------------------------------------------------------ ES7210 analog
#define ES7210_ADDR_A       0x40        // AD1=L, AD0=L -> CH1..CH4  (I2S0, master)
#define ES7210_ADDR_B       0x41        // AD1=L, AD0=H -> CH5..CH8  (I2S1, slave; unused with SINGLE_ES7210)
// ---- output channel order (measured on the StampS3, 8ch build, 2026-11)
// One mic at a time gave: A MIC1 -> CH7, A MIC2 -> CH5, B MIC1 -> CH3, B MIC4 -> CH2 (before this mapping)
//  * TDM slot order inside one ES7210 frame as received: slot0 = MIC2, slot1 = MIC4, slot2 = MIC1, slot3 = MIC3
//    (MIC3 -> slot3 is inferred, not measured). ES7210_SLOT_OF_MIC[k] = slot that carries MIC(k+1).
//    (A and B were also swapped in that test: a wiring mistake of DIN_A / DIN_B, not handled here)
// With this mapping: CH1..4 = A MIC1..4 (I2S0 / DIN_A), CH5..8 = B MIC1..4 (I2S1 / DIN_B)
// ---- LRCK polarity seen by the ES7210s (inverted in the ESP GPIO matrix; I2S0/I2S1 timing unchanged)
//  0: as the I2S driver outputs it (the measurement above)
//  1: inverted (fixed spec, verified on hardware 2026-11). The ES7210 sends MIC1/MIC3 in one LRCK half
//     and MIC2/MIC4 in the other; with the inverted LRCK the received order is MIC1, MIC3, MIC2, MIC4
//     and there is no 1-sample skew between the MIC1/3 and MIC2/4 pairs.
#ifndef ES7210_LRCK_INVERT
#define ES7210_LRCK_INVERT  1
#endif
#ifndef ES7210_SLOT_OF_MIC
#if ES7210_LRCK_INVERT
#define ES7210_SLOT_OF_MIC  {0, 2, 1, 3}    // measured: CH1..4 = MIC1..4
#else
#define ES7210_SLOT_OF_MIC  {2, 0, 3, 1}    // measured
#endif
#endif

#define I2C_FREQ_HZ       100000      // 100 kHz: margin for long wires / weak pull-ups
// StampS3 I2C driver for the ES7210s
//   0: Arduino Wire (ESP-IDF driver): SCL/SDA glitch filter on, I2C_GLITCH_FILTER_CYC cycles
//   1: M5.Ex_I2C (M5Unified / M5GFX): glitch filter threshold 0 = spikes on SCL/SDA are not filtered
// (Stamp-P4 always uses Wire)
#ifndef I2C_USE_M5_EX_I2C
#define I2C_USE_M5_EX_I2C   0
#endif
#define I2C_GLITCH_FILTER_CYC 7         // ignore SCL/SDA pulses shorter than this many I2C source clocks (0..7)
#define I2C_WRITE_TRIES     5           // each ES7210 register write is tried up to this many times
#define I2C_RETRY_MS        1000        // ES7210 I2C init is repeated at this interval until it succeeds

// PGA gain [dB]: 0,3,6,...,33, 34.5, 36, 37.5  (nearest step is used)
#define MIC_GAIN_DB         30.0f

// MICBIAS [mV]: 2180, 2260, 2360, 2450, 2550, 2660, 2780, 2870 (nearest step is used)
#define MIC_BIAS_MV         2870

// =====================================================================================
// Pin assignment (select board with -DMA8_BOARD_xxx in platformio.ini)
// =====================================================================================
#if defined(MA8_BOARD_STAMPS3)
// ---- M5StampS3 (in M5Capsule) : only the 2.54mm pitch pins are used for external signals
//   G1, G3, G5, G7, G9, G13, G15, G40, G43, G44
//   G13/G15 : Grove (Port A, I2C)    G43 : debug UART TX    G3, G40 : free
//   (G3 is a strapping pin, G40 is the clock of the M5Capsule internal PDM mic, which is disabled)
//   SD card : M5Capsule internal slot (SPI)
#define PIN_I2S_MCLK        1
#define PIN_I2S_BCLK        5
#define PIN_I2S_LRCK        7
#define PIN_I2S_DIN_A       9
#define PIN_I2S_DIN_B       44          // U0RXD pad, input only here
#define PIN_I2C_SDA         13
#define PIN_I2C_SCL         15
#define PIN_SD_CS           11
#define PIN_SD_SCK          14
#define PIN_SD_MOSI         12
#define PIN_SD_MISO         39
#define PIN_DEBUG_UART_TX   43          // U0TXD pad
#define PIN_RGB_LED         21          // internal WS2812 (status)
#define PIN_DEBUG_SW        0           // switch to GND: UART DBG output on/off (G0 = BOOT strap, internal pull-up)
#define USE_M5UNIFIED       1           // button via M5Unified, LED via FastLED, ES7210 on M5.Ex_I2C

#elif defined(MA8_BOARD_STAMPP4)
// ---- M5Stack Stamp-P4 : external signals only on the 2.54mm DIP pins
//   G0(LPG0) G1(LPG1) G8(LPG8) G10(LPG10) G12(LPG12) G17 G19 G21 G23 G26 G27 G35 G37 G38
//   + USB2 OTG HS D+/D- pads (USB2_HOST_DP / USB2_HOST_DM, bottom edge) for USB Audio + CDC.
//   The on-board USB-C is USB Serial/JTAG (G24/G25): upload + boot log.
//   Strapping / special pins carry only ESP32 outputs into high-Z ES7210 inputs:
//     G35 (BOOT, 10k pull-up) = BCLK, G38 (U0RXD) = LRCK, G37 (U0TXD) = debug UART TX
//   G26/G27 are the USB FS OTG PHY pads (unused as USB) -> slow I2C
#define PIN_I2S_MCLK        0
#define PIN_I2S_BCLK        35
#define PIN_I2S_LRCK        38
#define PIN_I2S_DIN_A       17
#define PIN_I2S_DIN_B       19
#define PIN_I2C_SDA         26
#define PIN_I2C_SCL         27
#define PIN_SD_CLK          12          // SDMMC slot 1 (GPIO matrix), 4bit
#define PIN_SD_CMD          10
#define PIN_SD_D0           8
#define PIN_SD_D1           1
#define PIN_SD_D2           21
#define PIN_SD_D3           23
#define PIN_DEBUG_UART_TX   37          // U0TXD pad
#define USE_M5UNIFIED       0           // no button / LED: control by serial commands only

#else
#error "Define MA8_BOARD_STAMPS3 or MA8_BOARD_STAMPP4 (see platformio.ini, p4/platformio.ini)"
#endif

// ------------------------------------------------------------------ I2S / capture
#define I2S_DMA_FRAMES      160         // frames per DMA buffer (<= 4092 / (4 * slot bytes))
#define I2S_DMA_BUFS        8           // number of DMA buffers per I2S port
#define CAPTURE_TASK_CORE   0
#define CAPTURE_TASK_PRIO   (configMAX_PRIORITIES - 2)

// ------------------------------------------------------------------ ring buffer
#if defined(MA8_BOARD_STAMPP4)
#define RING_BYTES_PSRAM    (8 * 1024 * 1024)   // Stamp-P4: 32MB PSRAM
#else
#define RING_BYTES_PSRAM    (4 * 1024 * 1024)   // used when PSRAM is found
#endif
#define RING_BYTES_SRAM     (128 * 1024)        // used when PSRAM is not found (StampS3)

// ------------------------------------------------------------------ SD
#define SD_SPI_HZ           25000000            // StampS3 (SPI)
#define SD_MMC_KHZ          20000               // Stamp-P4 (SDMMC 4bit, GPIO matrix)
#define SD_DIR              "/MA8"
#define SD_WRITE_FRAMES     2048                // frames per SD write (16bit: 32KB)
#define FILE_SPLIT_SEC      600                 // new WAV file every N seconds (0 = never)
#define FILE_FLUSH_MS       5000                // WAV header update + flush interval

// ------------------------------------------------------------------ USB serial (binary)
#define SERIAL_PKT_FRAMES   128                 // frames per binary packet
#if defined(MA8_BOARD_STAMPS3)
#define SERIAL_USE_RAW_EP   1                   // 1: send directly on the CDC bulk IN endpoint (fast)
#else                                           // 0: use Serial.write() (simple, slow)
#define SERIAL_USE_RAW_EP   0                   // Stamp-P4: CDC is on USB HS, Serial.write() is enough
#endif

// ------------------------------------------------------------------ debug UART (OUT_MODE_DEBUG_UART)
// ES7210 runs at SAMPLE_RATE_HZ (>= 8 kHz); the output is decimated by DEBUG_DECIM to ~DEBUG_RATE_HZ.
// One line per output frame: "v1,v2,...\n" (DEBUG_CH signed integers in SAMPLE_BITS units).
#ifndef DEBUG_CH
#define DEBUG_CH            NUM_CH              // channels per line: all (8 in an 8ch build), or 4 with DEBUG_CH_FIRST
#endif
#ifndef DEBUG_RATE_HZ
#if DEBUG_CH > 4
#define DEBUG_RATE_HZ       500                 // 8ch: keep the text within the UART bandwidth (also at 32bit)
#else
#define DEBUG_RATE_HZ       1000
#endif
#endif
#define DEBUG_AVERAGE       1                   // 0: every DEBUG_DECIM-th sample, 1: mean of DEBUG_DECIM samples,
                                                // 2: peak |x| of DEBUG_DECIM samples (mic level check)
#define DEBUG_CH_FIRST      0                   // first output channel - 1 (4 with DEBUG_CH 4: ES7210 B only)
// Defined: instead of the decimated values, one line "pp1,pp2,..." every DEBUG_PEAK_MS
// (amplitude = maximum - minimum of each channel in that interval; DEBUG_RATE_HZ / DEBUG_AVERAGE are not used)
#define OUT_MODE_DEBUG_UART_PEAK
#define DEBUG_PEAK_MS       500
// Output of OUT_MODE_DEBUG_UART:
//   UART_G43 defined     : hardware UART1, TX = PIN_DEBUG_UART_TX (StampS3: G43, Stamp-P4: G37), DEBUG_UART_BAUD
//   UART_G43 not defined : USB serial (CDC), the same port as the log messages
//#define UART_G43
#define DEBUG_UART_BAUD     921600
#define PIN_DEBUG_UART_RX   -1                  // UART1 RX: not used

// ------------------------------------------------------------------ UI
#define UI_UPDATE_MS        200
#define LONG_PRESS_MS       1000                // StampS3 button / G0: long press = SD <-> USB AUDIO mode

// =====================================================================================
// Derived values (do not edit)
// =====================================================================================
#if SAMPLE_BITS == 16
#define SLOT_BITS           16
#define OUT_BYTES           2
#elif SAMPLE_BITS == 24
#define SLOT_BITS           32
#define OUT_BYTES           3
#elif SAMPLE_BITS == 32
#define SLOT_BITS           32
#define OUT_BYTES           4
#else
#error "SAMPLE_BITS must be 16, 24 or 32"
#endif

// MCLK / Fs. The ES7210 clock setting follows it (adc_div = MCLK_MULTIPLE / 256).
// Stamp-P4 (IDF 5 TDM driver): with MCLK/BCLK <= 2 the driver raises the MCLK multiple by itself while
// receiving multiple slots, so 512 is set for 32bit slots to keep it in step with the ES7210 setting

#if defined(MA8_BOARD_STAMPP4) && SLOT_BITS == 32
#define MCLK_MULTIPLE       512
#else
#define MCLK_MULTIPLE       256
#endif

#define FRAME_BYTES         (NUM_CH * OUT_BYTES)            // bytes of one frame (all channels)
#define BYTES_PER_SEC       (SAMPLE_RATE_HZ * FRAME_BYTES)

#if defined(MA8_BOARD_STAMPP4)
// USB Audio (UAC2, high speed): one iso packet <= 1024 bytes every 2^(bInterval-1) microframes.
// The longest interval whose packet fits is used (fewer interrupts).
#define UAC_PKT_FRAMES_US(us)   ((SAMPLE_RATE_HZ * (us) + 999999) / 1000000 + 1)
#define UAC_HS_MAX_PKT          1024
#if UAC_PKT_FRAMES_US(1000) * FRAME_BYTES <= UAC_HS_MAX_PKT
#define UAC_BINTERVAL           4       // 1 ms
#define UAC_PKTS_PER_SEC        1000
#elif UAC_PKT_FRAMES_US(500) * FRAME_BYTES <= UAC_HS_MAX_PKT
#define UAC_BINTERVAL           3       // 500 us
#define UAC_PKTS_PER_SEC        2000
#elif UAC_PKT_FRAMES_US(250) * FRAME_BYTES <= UAC_HS_MAX_PKT
#define UAC_BINTERVAL           2       // 250 us
#define UAC_PKTS_PER_SEC        4000
#else
#define UAC_BINTERVAL           1       // 125 us
#define UAC_PKTS_PER_SEC        8000
#endif
#define UAC_MAX_FRAMES_PER_PKT  ((SAMPLE_RATE_HZ + UAC_PKTS_PER_SEC - 1) / UAC_PKTS_PER_SEC + 1)
#define UAC_MAX_PKT_BYTES       (UAC_MAX_FRAMES_PER_PKT * FRAME_BYTES)
#define USB_AUDIO_AVAILABLE     1
#else
// USB Audio (UAC1, full speed): the ESP32-S3 USB FIFO is only 1 KB in total.
// About 624 bytes are left for the audio IN FIFO (CDC uses the rest).
#define UAC_MAX_FRAMES_PER_PKT  ((SAMPLE_RATE_HZ + 999) / 1000 + 1)
#define UAC_MAX_PKT_BYTES       (UAC_MAX_FRAMES_PER_PKT * FRAME_BYTES)
#define UAC_FIFO_LIMIT_BYTES    624
#if UAC_MAX_PKT_BYTES <= UAC_FIFO_LIMIT_BYTES
#define USB_AUDIO_AVAILABLE 1
#else
#define USB_AUDIO_AVAILABLE 0   // e.g. 16bit: <= 38kHz, 24bit: <= 25kHz, 32bit: <= 18kHz
#endif
#endif

static_assert(SAMPLE_RATE_HZ >= 8000 && SAMPLE_RATE_HZ <= 48000, "ES7210 single speed: 8k..48kHz");
#define SD_WRITE_CHUNK      (SD_WRITE_FRAMES * FRAME_BYTES)

#ifdef OUT_MODE_DEBUG_UART_PEAK
#define DEBUG_DECIM         (SAMPLE_RATE_HZ * DEBUG_PEAK_MS / 1000)    // frames per max/min line
#else
#define DEBUG_DECIM         ((SAMPLE_RATE_HZ + DEBUG_RATE_HZ / 2) / DEBUG_RATE_HZ)
#endif
static_assert(DEBUG_DECIM >= 1, "DEBUG_RATE_HZ too high");
// worst-case line: "-32768," (16bit) / "-8388608," (24bit) / "-2147483648," (32bit) per channel, 10 bits per byte
#define DEBUG_VALUE_CHARS   (SAMPLE_BITS == 16 ? 7 : SAMPLE_BITS == 24 ? 9 : 12)
#ifdef UART_G43
static_assert(1ULL * DEBUG_CH * DEBUG_VALUE_CHARS * 10 * SAMPLE_RATE_HZ / DEBUG_DECIM <= DEBUG_UART_BAUD,
              "debug UART: DEBUG_RATE_HZ too high for DEBUG_CH / SAMPLE_BITS / DEBUG_UART_BAUD");
#endif
static_assert(DEBUG_CH_FIRST + DEBUG_CH <= NUM_CH, "DEBUG_CH_FIRST out of range");
static_assert(I2S_DMA_FRAMES * 4 * (SLOT_BITS / 8) <= 4092, "I2S DMA buffer too large");
