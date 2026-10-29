#pragma once
// Binary streaming over USB CDC (Serial).
//
// Packet (little endian):
//   offset size
//   0      4   magic  'M','A','8','D'  (0x4438414D)
//   4      4   seq    packet sequence number (starts at 0 on every start)
//   8      4   rate   sample rate [Hz]
//   12     2   frames number of frames in this packet
//   14     1   channels (8, or 4 with SINGLE_ES7210)
//   15     1   bytes per sample (2 / 3 / 4)
//   16     N   payload: frames x channels x bytes, interleaved (CH1..CH8), signed LE
//   16+N   4   crc32 (zlib compatible) of bytes [0 .. 16+N)
//
// Text commands from the PC (one character, newline optional):
//   's' start streaming   'x' stop   'i' info (one JSON line, only while stopped)
//   'g' + number : set gain [dB] (e.g. "g24\n")

#include <stddef.h>
#include <stdint.h>

namespace serial_stream {

bool begin();
bool sendPacket(const uint8_t* frames, size_t n_frames, uint32_t seq);  // blocking (with timeout)
bool connected();      // host opened the port (DTR)
uint32_t errors();

}  // namespace serial_stream
