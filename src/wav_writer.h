#pragma once
// Minimal multi-channel PCM WAV writer (16 / 24 / 32 bit, integer).
// The header sizes are rewritten by updateHeader() / close(), so a file stays readable
// up to the last flush even if power is lost.

#include <FS.h>
#include <stdint.h>

class WavWriter {
public:
    bool open(fs::FS& fs, const char* path, uint32_t sample_rate, uint16_t channels, uint16_t bits);
    size_t write(const uint8_t* data, size_t bytes);
    void updateHeader();  // write current sizes into the header and flush
    void close();
    bool isOpen() const { return (bool)file_; }
    uint32_t dataBytes() const { return data_bytes_; }

private:
    void writeHeader();
    File file_;
    uint32_t data_bytes_ = 0;
    uint32_t sample_rate_ = 0;
    uint16_t channels_ = 1;
    uint16_t bits_ = 16;
};
