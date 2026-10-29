// Stamp-P4: USB Audio Class 2.0 microphone (NUM_CH), USB2 OTG high speed (TinyUSB rhport 1)
// TinyUSB "application class driver" on top of the Arduino-ESP32 3.x USB stack
// (USB_INTERFACE_CUSTOM). Composite device: CDC (Serial) + UAC2.
//   * clock source: internal fixed clock = SAMPLE_RATE_HZ (GET CUR / RANGE, SET CUR is ignored)
//   * iso IN, asynchronous, one packet every 2^(UAC_BINTERVAL-1) microframes (see config.h)
// (The UAC class built into the Arduino TinyUSB library is compiled for 2 channels, so it is not used.)
#include "../usb_audio.h"

#include <Arduino.h>

#include "../capture.h"
#include "config.h"

#if USB_AUDIO_AVAILABLE && (ARDUINO_USB_MODE == 0)

#include "device/usbd_pvt.h"
#include "esp32-hal-tinyusb.h"

namespace usb_audio {

// ------------------------------------------------------------------ descriptor
// IAD + AC(std, header, clock source, IT, OT) + AS alt0 + AS alt1(general, format, ep, cs ep)
#define UAC_AC_CS_LEN (9 + 8 + 17 + 12)
#define UAC_DESC_LEN  (8 + 9 + UAC_AC_CS_LEN + 9 + 9 + 16 + 6 + 7 + 8)
#define UAC_IT_ID     1
#define UAC_OT_ID     3
#define UAC_CLK_ID    4

static uint8_t s_itf_ac = 0, s_itf_as = 0, s_ep_in = 0;
static tusb_desc_endpoint_t const* s_ep_desc = nullptr;

static uint16_t loadDescriptor(uint8_t* dst, uint8_t* itf) {
    s_itf_ac = *itf;
    s_itf_as = *itf + 1;
    *itf += 2;
    uint8_t ep = tinyusb_get_free_in_endpoint();
    if (ep == 0) return 0;
    s_ep_in = 0x80 | ep;
#ifdef SINGLE_ES7210
    uint8_t str = tinyusb_add_string_descriptor("MA8 4ch Microphone");
#else
    uint8_t str = tinyusb_add_string_descriptor("MA8 8ch Microphone");
#endif

    const uint16_t mps = UAC_MAX_PKT_BYTES;
    const uint8_t d[UAC_DESC_LEN] = {
        // IAD: audio function, UAC 2.0
        8, TUSB_DESC_INTERFACE_ASSOCIATION, s_itf_ac, 2, TUSB_CLASS_AUDIO, 0x00, 0x20, str,
        // AC interface (standard)
        9, TUSB_DESC_INTERFACE, s_itf_ac, 0, 0, TUSB_CLASS_AUDIO, 0x01, 0x20, str,
        // AC header: ADC 2.00, category microphone
        9, 0x24, 0x01, 0x00, 0x02, 0x03, (uint8_t)(UAC_AC_CS_LEN & 0xFF), (uint8_t)(UAC_AC_CS_LEN >> 8), 0x00,
        // Clock source: internal fixed, frequency + validity read-only
        8, 0x24, 0x0A, UAC_CLK_ID, 0x01, 0x05, 0, 0,
        // Input terminal: microphone, NUM_CH, no spatial location
        17, 0x24, 0x02, UAC_IT_ID, 0x01, 0x02, 0, UAC_CLK_ID, NUM_CH, 0, 0, 0, 0, 0, 0x00, 0x00, 0,
        // Output terminal: USB streaming
        12, 0x24, 0x03, UAC_OT_ID, 0x01, 0x01, 0, UAC_IT_ID, UAC_CLK_ID, 0x00, 0x00, 0,
        // AS interface alt 0 (zero bandwidth)
        9, TUSB_DESC_INTERFACE, s_itf_as, 0, 0, TUSB_CLASS_AUDIO, 0x02, 0x20, 0,
        // AS interface alt 1 (streaming)
        9, TUSB_DESC_INTERFACE, s_itf_as, 1, 1, TUSB_CLASS_AUDIO, 0x02, 0x20, 0,
        // AS general: terminal link = OT, format type I, PCM, NUM_CH
        16, 0x24, 0x01, UAC_OT_ID, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, NUM_CH, 0, 0, 0, 0, 0,
        // Format type I: subslot size, bit resolution
        6, 0x24, 0x02, 0x01, OUT_BYTES, SAMPLE_BITS,
        // Standard iso endpoint, asynchronous, data
        7, TUSB_DESC_ENDPOINT, s_ep_in, 0x05, (uint8_t)(mps & 0xFF), (uint8_t)(mps >> 8), UAC_BINTERVAL,
        // Class-specific iso endpoint
        8, 0x25, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    memcpy(dst, d, sizeof(d));
    return sizeof(d);
}

// Register before USB.begin() (Arduino starts USB before setup() when CDC_ON_BOOT=1)
static struct Registrar {
    Registrar() { tinyusb_enable_interface(USB_INTERFACE_CUSTOM, UAC_DESC_LEN, loadDescriptor); }
} s_registrar;

// ------------------------------------------------------------------ state
static volatile uint8_t s_alt = 0;
static volatile bool s_active = false;
static volatile bool s_armed = false;
static volatile bool s_ep_open = false;
static volatile bool s_priming = true;
static volatile bool s_need_discard = true;
static volatile uint32_t s_underruns = 0, s_packets = 0;
static uint32_t s_acc = 0;
static int32_t s_avg_level = 0;  // x16
static uint8_t s_ctrl_buf[16] __attribute__((aligned(4)));
static uint8_t s_alt_buf;
static uint8_t s_pkt[UAC_MAX_PKT_BYTES + 64] __attribute__((aligned(64)));

static const int32_t TARGET_FRAMES = 2 * I2S_DMA_FRAMES + SAMPLE_RATE_HZ / 1000 * 2;

static void queuePacket(uint8_t rhport) {
    s_acc += SAMPLE_RATE_HZ;
    int32_t n = s_acc / UAC_PKTS_PER_SEC;
    s_acc %= UAC_PKTS_PER_SEC;

    bool consume = s_active && capture::sink();
    size_t filled = 0;
    if (consume) {
        FrameRing& r = capture::ring();
        if (s_need_discard) {
            r.discard();
            s_need_discard = false;
            s_priming = true;
            s_avg_level = 0;
        }
        int32_t lvl = (int32_t)r.available();
        if (s_priming) {
            if (lvl >= TARGET_FRAMES) {
                s_priming = false;
                s_avg_level = lvl * 16;
            }
        }
        if (!s_priming) {
            s_avg_level += lvl - s_avg_level / 16;  // IIR, x16
            int32_t avg = s_avg_level / 16;
            if (avg > TARGET_FRAMES + I2S_DMA_FRAMES / 2) n += 1;           // device clock faster
            else if (avg < TARGET_FRAMES - I2S_DMA_FRAMES / 2 && n > 1) n -= 1;
            if (n > UAC_MAX_FRAMES_PER_PKT) n = UAC_MAX_FRAMES_PER_PKT;
            filled = r.read(s_pkt, n);
            if ((int32_t)filled < n) {
                s_underruns = s_underruns + 1;
                s_priming = true;
            }
        }
    }
    if ((int32_t)filled < n) memset(s_pkt + filled * FRAME_BYTES, 0, (n - filled) * FRAME_BYTES);

    s_armed = true;
    if (!usbd_edpt_xfer(rhport, s_ep_in, s_pkt, n * FRAME_BYTES, false)) s_armed = false;
}

static void stopStream(uint8_t rhport) {
    if (s_ep_open) {
        usbd_edpt_close(rhport, s_ep_in);
        s_ep_open = false;
    }
    s_armed = false;
}

// ------------------------------------------------------------------ class driver
static void uacInit(void) {}
static bool uacDeinit(void) { return true; }

static void uacReset(uint8_t rhport) {
    (void)rhport;
    s_alt = 0;
    s_armed = false;
    s_ep_open = false;
}

static uint16_t uacOpen(uint8_t rhport, tusb_desc_interface_t const* itf, uint16_t max_len) {
    if (itf->bInterfaceClass != TUSB_CLASS_AUDIO || itf->bInterfaceSubClass != 0x01 ||
        itf->bInterfaceNumber != s_itf_ac)
        return 0;
    uint8_t const* p = (uint8_t const*)itf;
    uint8_t const* end = p + max_len;
    uint16_t len = 0;
    tusb_desc_endpoint_t const* ep = nullptr;
    while (p < end) {
        uint8_t type = tu_desc_type(p);
        if (type == TUSB_DESC_INTERFACE_ASSOCIATION) break;
        if (type == TUSB_DESC_INTERFACE) {
            uint8_t n = ((tusb_desc_interface_t const*)p)->bInterfaceNumber;
            if (n != s_itf_ac && n != s_itf_as) break;
        }
        if (type == TUSB_DESC_ENDPOINT) ep = (tusb_desc_endpoint_t const*)p;
        len += tu_desc_len(p);
        p = tu_desc_next(p);
    }
    // reserve the iso FIFO once; the endpoint is activated by SET_INTERFACE(alt 1)
    if (ep) {
        if (!usbd_edpt_iso_alloc(rhport, s_ep_in, UAC_MAX_PKT_BYTES)) return 0;
        s_ep_desc = ep;
    }
    s_alt = 0;
    s_armed = false;
    s_ep_open = false;
    return len;
}

static bool clockRequest(uint8_t rhport, tusb_control_request_t const* req) {
    uint8_t cs = tu_u16_high(req->wValue);
    bool in = req->bmRequestType_bit.direction == TUSB_DIR_IN;
    const uint32_t fs = SAMPLE_RATE_HZ;
    if (cs == 0x01) {                        // SAM_FREQ_CONTROL
        if (req->bRequest == 0x01) {         // CUR
            if (!in) return tud_control_xfer(rhport, req, s_ctrl_buf, 4);  // SET: accept and ignore
            memcpy(s_ctrl_buf, &fs, 4);
            return tud_control_xfer(rhport, req, s_ctrl_buf, 4);
        }
        if (req->bRequest == 0x02 && in) {   // RANGE: 1 sub-range (min = max = fs, res = 0)
            const uint16_t num = 1;
            const uint32_t res = 0;
            memcpy(s_ctrl_buf + 0, &num, 2);
            memcpy(s_ctrl_buf + 2, &fs, 4);
            memcpy(s_ctrl_buf + 6, &fs, 4);
            memcpy(s_ctrl_buf + 10, &res, 4);
            return tud_control_xfer(rhport, req, s_ctrl_buf, 14);
        }
        return false;
    }
    if (cs == 0x02 && req->bRequest == 0x01 && in) {  // CLOCK_VALID_CONTROL, CUR
        s_ctrl_buf[0] = 1;
        return tud_control_xfer(rhport, req, s_ctrl_buf, 1);
    }
    return false;
}

static bool uacControl(uint8_t rhport, uint8_t stage, tusb_control_request_t const* req) {
    if (stage != CONTROL_STAGE_SETUP) return true;

    // ---- standard interface requests (alternate setting)
    if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
        req->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE) {
        uint8_t itf = tu_u16_low(req->wIndex);
        if (req->bRequest == TUSB_REQ_SET_INTERFACE) {
            if (itf == s_itf_as) {
                uint8_t alt = tu_u16_low(req->wValue);
                stopStream(rhport);
                s_alt = alt;
                if (alt == 1 && s_ep_desc) {
                    if (usbd_edpt_iso_activate(rhport, s_ep_desc)) {
                        s_ep_open = true;
                        s_need_discard = true;
                        s_priming = true;
                        queuePacket(rhport);
                    }
                }
            }
            return tud_control_status(rhport, req);
        }
        if (req->bRequest == TUSB_REQ_GET_INTERFACE) {
            s_alt_buf = (itf == s_itf_as) ? s_alt : 0;
            return tud_control_xfer(rhport, req, &s_alt_buf, 1);
        }
        return false;
    }

    // ---- class requests to an entity of the AC interface (wIndex = entity << 8 | interface)
    if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
        req->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE) {
        if (tu_u16_low(req->wIndex) != s_itf_ac) return false;
        if (tu_u16_high(req->wIndex) == UAC_CLK_ID) return clockRequest(rhport, req);
        return false;
    }
    return false;
}

static bool uacXfer(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    (void)result;
    (void)xferred_bytes;
    if (ep_addr != s_ep_in) return false;
    s_armed = false;
    s_packets = s_packets + 1;
    if (s_alt == 1 && s_ep_open) queuePacket(rhport);
    return true;
}

static usbd_class_driver_t const s_driver = {
    .name = "UAC2",
    .init = uacInit,
    .deinit = uacDeinit,
    .reset = uacReset,
    .open = uacOpen,
    .control_xfer_cb = uacControl,
    .xfer_cb = uacXfer,
    .xfer_isr = nullptr,
    .sof = nullptr,
};

bool available() { return true; }
bool streaming() { return s_alt == 1; }
void setActive(bool on) {
    if (on && !s_active) s_need_discard = true;
    s_active = on;
}
bool active() { return s_active; }
uint32_t underruns() { return s_underruns; }
uint32_t parityFixes() { return 0; }
uint32_t packets() { return s_packets; }

}  // namespace usb_audio

extern "C" usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count) {
    *driver_count = 1;
    return &usb_audio::s_driver;
}

#else  // ------------------------------------------------------------- not available

namespace usb_audio {
bool available() { return false; }
bool streaming() { return false; }
void setActive(bool) {}
bool active() { return false; }
uint32_t underruns() { return 0; }
uint32_t parityFixes() { return 0; }
uint32_t packets() { return 0; }
}  // namespace usb_audio

#endif
