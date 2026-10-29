#include "serial_stream.h"

#include <Arduino.h>
#include <esp_rom_crc.h>

#include "config.h"

#if SERIAL_USE_RAW_EP && (ARDUINO_USB_MODE == 0)
#include "esp32-hal-tinyusb.h"
#include "device/usbd_pvt.h"
// Arduino-ESP32 2.x USBCDC uses fixed endpoints: notif 0x85, OUT 0x03, IN 0x84
#define CDC_EP_IN 0x84
#endif

namespace serial_stream {

#define PKT_HDR 16
#define PKT_MAX (PKT_HDR + SERIAL_PKT_FRAMES * FRAME_BYTES + 4)

static uint8_t s_buf[2][PKT_MAX] __attribute__((aligned(4)));
static int s_cur = 0;
static uint32_t s_errors = 0;

bool begin() { return true; }

bool connected() { return (bool)Serial; }
uint32_t errors() { return s_errors; }

static size_t build(uint8_t* p, const uint8_t* frames, size_t n, uint32_t seq) {
    const uint32_t magic = 0x4438414D;  // "MA8D"
    const uint32_t rate = SAMPLE_RATE_HZ;
    const uint16_t nf = (uint16_t)n;
    memcpy(p + 0, &magic, 4);
    memcpy(p + 4, &seq, 4);
    memcpy(p + 8, &rate, 4);
    memcpy(p + 12, &nf, 2);
    p[14] = NUM_CH;
    p[15] = OUT_BYTES;
    size_t payload = n * FRAME_BYTES;
    memcpy(p + PKT_HDR, frames, payload);
    uint32_t crc = esp_rom_crc32_le(0, p, PKT_HDR + payload);
    memcpy(p + PKT_HDR + payload, &crc, 4);
    return PKT_HDR + payload + 4;  // = 20 + 16k bytes -> never a multiple of 64 (no ZLP)
}

bool sendPacket(const uint8_t* frames, size_t n, uint32_t seq) {
    if (n > SERIAL_PKT_FRAMES) n = SERIAL_PKT_FRAMES;
    uint8_t* p = s_buf[s_cur];
    size_t len = build(p, frames, n, seq);

#if SERIAL_USE_RAW_EP && (ARDUINO_USB_MODE == 0)
    // Bypass the 64-byte CDC TX FIFO: one transfer of the whole packet on the bulk IN EP.
    // Double buffered: the buffer of the previous transfer is not touched until it completes.
    uint32_t t0 = millis();
    while (!usbd_edpt_claim(0, CDC_EP_IN)) {
        if (!tud_cdc_connected() || millis() - t0 > 500) {
            s_errors++;
            return false;
        }
        vTaskDelay(1);
    }
    if (!usbd_edpt_xfer(0, CDC_EP_IN, p, len)) {
        usbd_edpt_release(0, CDC_EP_IN);
        s_errors++;
        return false;
    }
    s_cur ^= 1;
    return true;
#else
    size_t w = Serial.write(p, len);
    if (w != len) {
        s_errors++;
        return false;
    }
    return true;
#endif
}

}  // namespace serial_stream
