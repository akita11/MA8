#ifndef Pins_Arduino_h
#define Pins_Arduino_h

// M5Stack Stamp-P4 (ESP32-P4NRW32, 16MB flash, 32MB PSRAM) for MA8.
// Based on the esp32p4 variant, without the ESP32-P4 Function EV board definitions:
//   * no BOARD_HAS_SDMMC -> SD_MMC uses slot 1 through the GPIO matrix with the pins given to setPins()
//     and no on-chip LDO / power pin for the card (external 3.3V)
//   * no ESP-Hosted (Wi-Fi) / Ethernet pins

#include <stdint.h>
#include "soc/soc_caps.h"

static const uint8_t TX = 37;  // U0TXD
static const uint8_t RX = 38;  // U0RXD

static const uint8_t SDA = 26;  // MA8: I2C to ES7210
static const uint8_t SCL = 27;

static const uint8_t SS = 23;
static const uint8_t MOSI = 10;
static const uint8_t MISO = 8;
static const uint8_t SCK = 12;

static const uint8_t A0 = 16;
static const uint8_t A1 = 17;
static const uint8_t A2 = 18;
static const uint8_t A3 = 19;
static const uint8_t A4 = 20;
static const uint8_t A5 = 21;
static const uint8_t A6 = 22;
static const uint8_t A7 = 23;
static const uint8_t A8 = 49;
static const uint8_t A9 = 50;
static const uint8_t A10 = 51;

#endif /* Pins_Arduino_h */
