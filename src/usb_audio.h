#pragma once
// USB Audio microphone (NUM_CH, isochronous IN, asynchronous), composite device CDC (Serial) + UAC.
// Implemented as a TinyUSB "application class driver" on top of the Arduino-ESP32 USB stack
// (USB_INTERFACE_CUSTOM).
//
//   src/s3/usb_audio.cpp : StampS3, UAC1, full speed. The ESP32-S3 USB OTG FIFO is 1 KB in total,
//                          so one iso packet must fit in about 624 bytes (UAC_FIFO_LIMIT_BYTES):
//                          16bit: up to 38 kHz (32 kHz recommended), 24bit: up to 25 kHz, 32bit: up to 18 kHz
//   src/p4/usb_audio.cpp : Stamp-P4, UAC2, high speed (USB2 OTG HS). 8ch up to 48 kHz / 32bit.

#include <stdint.h>

namespace usb_audio {

bool available();           // compiled in (USB_AUDIO_AVAILABLE)
bool streaming();           // host selected the streaming alternate setting (recording app is open)
void setActive(bool on);    // true: send ring data, false: send silence (ring is not touched)
bool active();

uint32_t underruns();       // packets with not enough data
uint32_t parityFixes();     // iso IN parity re-sync count (diagnostics, StampS3 only)
uint32_t packets();

}  // namespace usb_audio
