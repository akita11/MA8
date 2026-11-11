#pragma once
// 8ch capture: ES7210 A -> I2S0 (master, makes MCLK/BCLK/LRCK), ES7210 B -> I2S1 (slave, same BCLK/LRCK)
// The capture task reads both ports block by block, merges them into 8ch frames
// (CH1..4 = A, CH5..8 = B), converts to SAMPLE_BITS and pushes the frames into the ring.
// With SINGLE_ES7210 only ES7210 A / I2S0 is used (4ch frames).
//   src/s3/capture.cpp : StampS3, legacy driver/i2s.h (Arduino 2.0.x / IDF 4.4)
//   src/p4/capture.cpp : Stamp-P4, driver/i2s_tdm.h (Arduino 3.x / IDF 5)

#include <stdint.h>

#include "config.h"
#include "ringbuf.h"

namespace capture {

// Source of output channel c in the merged block: 0..3 = I2S0 slot (A), 4..7 = I2S1 slot (B)
// (ES7210_SLOT_OF_MIC in config.h)
constexpr uint8_t kSlotOfMic[4] = ES7210_SLOT_OF_MIC;
constexpr uint8_t srcOfChannel(int c) { return (uint8_t)((c >> 2) * 4 + kSlotOfMic[c & 3]); }

struct Status {
    bool es_a = false;          // ES7210 A answered on I2C and was configured
    bool es_b = false;
    bool bclk_stops = false;    // BCLK stops while I2S0 is stopped (=> A/B start aligned)
    bool i2s_ok = false;
};

bool begin();                   // I2S + ES7210 init, start capture task
const Status& status();

FrameRing& ring();
void setSink(bool on);          // true: push frames to ring, false: discard
bool sink();

uint32_t droppedFrames();       // frames lost because the ring was full
uint64_t capturedFrames();      // total frames pushed to ring since setSink(true)
void resetCounters();

// peak level per channel since last call (0..1, full scale = 1), peak[NUM_CH]
void takePeaks(float* peak);

bool setGain(float db);         // both ES7210
float gain();

void dumpRegs();                // log the registers of the ES7210(s) and check them (serial command 'R')

}  // namespace capture
