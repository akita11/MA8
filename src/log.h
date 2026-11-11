#pragma once
// Log line to the USB CDC (and the Stamp-P4 USB Serial/JTAG). Defined in main.cpp.
void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
