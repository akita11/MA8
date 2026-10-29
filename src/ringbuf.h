#pragma once
// Single-producer / single-consumer lock-free ring buffer of fixed-size frames.
//   producer : capture task (core 0)
//   consumer : SD / serial writer (loop task) or USB audio (TinyUSB task) - only one at a time
// discard() is a consumer-side operation (drops everything that is currently stored).

#include <atomic>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

class FrameRing {
public:
    bool init(uint8_t* mem, size_t bytes, size_t frame_bytes) {
        if (!mem || frame_bytes == 0) return false;
        buf_ = mem;
        frame_bytes_ = frame_bytes;
        capacity_ = bytes / frame_bytes;
        head_.store(0);
        tail_.store(0);
        return capacity_ > 1;
    }

    size_t capacity() const { return capacity_ - 1; }
    size_t frameBytes() const { return frame_bytes_; }

    size_t available() const {
        size_t h = head_.load(std::memory_order_acquire);
        size_t t = tail_.load(std::memory_order_acquire);
        return (h >= t) ? (h - t) : (capacity_ - t + h);
    }

    size_t space() const { return capacity() - available(); }

    // ---- producer: push n frames (all or nothing). returns false if not enough space.
    bool push(const uint8_t* src, size_t n) {
        if (n > space()) return false;
        size_t h = head_.load(std::memory_order_relaxed);
        size_t first = capacity_ - h;
        if (first > n) first = n;
        memcpy(buf_ + h * frame_bytes_, src, first * frame_bytes_);
        if (n > first) memcpy(buf_, src + first * frame_bytes_, (n - first) * frame_bytes_);
        h += n;
        if (h >= capacity_) h -= capacity_;
        head_.store(h, std::memory_order_release);
        return true;
    }

    // ---- consumer: contiguous readable region (up to the end of the buffer)
    size_t peek(const uint8_t** ptr) const {
        size_t h = head_.load(std::memory_order_acquire);
        size_t t = tail_.load(std::memory_order_relaxed);
        *ptr = buf_ + t * frame_bytes_;
        return (h >= t) ? (h - t) : (capacity_ - t);
    }

    // ---- consumer: copy up to n frames into dst, returns copied frames
    size_t read(uint8_t* dst, size_t n) {
        size_t done = 0;
        while (done < n) {
            const uint8_t* p;
            size_t c = peek(&p);
            if (c == 0) break;
            if (c > n - done) c = n - done;
            memcpy(dst + done * frame_bytes_, p, c * frame_bytes_);
            consume(c);
            done += c;
        }
        return done;
    }

    void consume(size_t frames) {
        size_t t = tail_.load(std::memory_order_relaxed) + frames;
        if (t >= capacity_) t -= capacity_;
        tail_.store(t, std::memory_order_release);
    }

    void discard() { tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release); }

private:
    uint8_t* buf_ = nullptr;
    size_t frame_bytes_ = 0;
    size_t capacity_ = 0;
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};
};
