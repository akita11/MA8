#include "wav_writer.h"

#include <string.h>

static void put16(uint8_t* p, uint16_t v) {
    p[0] = v;
    p[1] = v >> 8;
}
static void put32(uint8_t* p, uint32_t v) {
    p[0] = v;
    p[1] = v >> 8;
    p[2] = v >> 16;
    p[3] = v >> 24;
}

bool WavWriter::open(fs::FS& fs, const char* path, uint32_t sample_rate, uint16_t channels, uint16_t bits) {
    close();
    file_ = fs.open(path, FILE_WRITE);
    if (!file_) return false;
    sample_rate_ = sample_rate;
    channels_ = channels;
    bits_ = bits;
    data_bytes_ = 0;
    writeHeader();
    return true;
}

void WavWriter::writeHeader() {
    uint8_t h[44];
    uint16_t block = channels_ * (bits_ / 8);
    memcpy(h + 0, "RIFF", 4);
    put32(h + 4, 36 + data_bytes_);
    memcpy(h + 8, "WAVE", 4);
    memcpy(h + 12, "fmt ", 4);
    put32(h + 16, 16);
    put16(h + 20, 1);  // PCM
    put16(h + 22, channels_);
    put32(h + 24, sample_rate_);
    put32(h + 28, sample_rate_ * block);
    put16(h + 32, block);
    put16(h + 34, bits_);
    memcpy(h + 36, "data", 4);
    put32(h + 40, data_bytes_);
    file_.seek(0);
    file_.write(h, sizeof(h));
}

size_t WavWriter::write(const uint8_t* data, size_t bytes) {
    if (!file_) return 0;
    size_t n = file_.write(data, bytes);
    data_bytes_ += n;
    return n;
}

void WavWriter::updateHeader() {
    if (!file_) return;
    size_t pos = 44 + data_bytes_;
    writeHeader();
    file_.seek(pos);
    file_.flush();
}

void WavWriter::close() {
    if (!file_) return;
    writeHeader();
    file_.close();
}
