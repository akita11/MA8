// StampS3 (ESP32-S3): USB Audio Class 1.0, full speed, Arduino 2.0.x TinyUSB (dcd_esp32sx)
#include "../usb_audio.h"

#include <Arduino.h>

#include "../capture.h"
#include "config.h"

#if USB_AUDIO_AVAILABLE && (ARDUINO_USB_MODE == 0)

#include "esp32-hal-tinyusb.h"
#include "device/usbd_pvt.h"
#include "soc/usb_periph.h"

namespace usb_audio {

// ------------------------------------------------------------------ descriptor
// UAC1:  IAD + AC(std, header, IT, OT) + AS alt0 + AS alt1(general, format, ep, cs ep)
#define UAC_DESC_LEN (8 + 9 + 9 + 12 + 9 + 9 + 9 + 7 + 11 + 9 + 7)
#define UAC_IT_ID 1
#define UAC_OT_ID 3

static uint8_t s_itf_ac = 0, s_itf_as = 0, s_ep_in = 0;

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

    const uint16_t ac_total = 9 + 12 + 9;
    const uint16_t mps = UAC_MAX_PKT_BYTES;
    const uint32_t fs = SAMPLE_RATE_HZ;
    const uint8_t d[UAC_DESC_LEN] = {
        // IAD
        8, TUSB_DESC_INTERFACE_ASSOCIATION, s_itf_ac, 2, TUSB_CLASS_AUDIO, 0x01, 0x00, str,
        // AC interface (standard)
        9, TUSB_DESC_INTERFACE, s_itf_ac, 0, 0, TUSB_CLASS_AUDIO, 0x01, 0x00, str,
        // AC header (UAC 1.0)
        9, 0x24, 0x01, 0x00, 0x01, (uint8_t)(ac_total & 0xFF), (uint8_t)(ac_total >> 8), 1, s_itf_as,
        // Input terminal: microphone, 8ch, no spatial location
        12, 0x24, 0x02, UAC_IT_ID, 0x01, 0x02, 0, NUM_CH, 0x00, 0x00, 0, 0,
        // Output terminal: USB streaming
        9, 0x24, 0x03, UAC_OT_ID, 0x01, 0x01, 0, UAC_IT_ID, 0,
        // AS interface alt 0 (zero bandwidth)
        9, TUSB_DESC_INTERFACE, s_itf_as, 0, 0, TUSB_CLASS_AUDIO, 0x02, 0x00, 0,
        // AS interface alt 1 (streaming)
        9, TUSB_DESC_INTERFACE, s_itf_as, 1, 1, TUSB_CLASS_AUDIO, 0x02, 0x00, 0,
        // AS general: terminal link = OT, delay 1, PCM
        7, 0x24, 0x01, UAC_OT_ID, 1, 0x01, 0x00,
        // Format type I: 8ch, subframe, bits, 1 discrete rate
        11, 0x24, 0x02, 0x01, NUM_CH, OUT_BYTES, SAMPLE_BITS, 1,
        (uint8_t)(fs & 0xFF), (uint8_t)(fs >> 8), (uint8_t)(fs >> 16),
        // Standard iso endpoint (UAC1: 9 bytes), asynchronous
        9, TUSB_DESC_ENDPOINT, s_ep_in, 0x05, (uint8_t)(mps & 0xFF), (uint8_t)(mps >> 8), 1, 0, 0,
        // Class-specific endpoint: sampling frequency control
        7, 0x25, 0x01, 0x01, 0, 0, 0,
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
static volatile bool s_priming = true;
static volatile bool s_need_discard = true;
static volatile uint32_t s_last_xfer_ms = 0;
static volatile uint32_t s_underruns = 0, s_parity_fix = 0, s_packets = 0;
static uint32_t s_acc = 0;
static int32_t s_avg_level = 0;  // x16
static uint8_t s_freq_buf[4];
static uint8_t s_alt_buf;
static uint8_t s_pkt[UAC_MAX_PKT_BYTES + 4] __attribute__((aligned(4)));

static const int32_t TARGET_FRAMES = 2 * I2S_DMA_FRAMES + SAMPLE_RATE_HZ / 1000 * 2;

// The S3 DCD (dcd_esp32sx) gives every IN FIFO the same small size (~188 bytes).
// Re-distribute the 1 KB FIFO RAM: other IN endpoints get what they need, the audio EP the rest.
static void fixFifo(uint8_t epnum) {
    const uint32_t total_words = 1024 / 4;
    uint32_t offset = (USB0.grxfsiz & 0xFFFF) + 16;  // RX FIFO + EP0 TX FIFO
    uint8_t my_fifo = (USB0.in_ep_reg[epnum].diepctl >> USB_D_TXFNUM1_S) & USB_D_TXFNUM1_V;
    bool used[8] = {false};
    used[my_fifo] = true;
    for (uint8_t n = 1; n < USB_IN_EP_NUM; n++) {
        if (n == epnum) continue;
        uint32_t ctl = USB0.in_ep_reg[n].diepctl;
        if (!(ctl & USB_D_USBACTEP1_M)) continue;
        uint8_t f = (ctl >> USB_D_TXFNUM1_S) & USB_D_TXFNUM1_V;
        if (f == 0 || used[f]) continue;
        uint32_t mps = ctl & USB_D_MPS1_M;
        uint32_t words = (mps + 3) / 4;
        if (words < 16) words = 16;
        USB0.dieptxf[f - 1] = (words << USB_NPTXFDEP_S) | offset;
        offset += words;
        used[f] = true;
    }
    uint32_t mine = total_words - offset;
    USB0.dieptxf[my_fifo - 1] = (mine << USB_NPTXFDEP_S) | offset;
    // unused FIFOs: zero size at the end
    for (uint8_t f = 1; f < 5; f++)
        if (!used[f]) USB0.dieptxf[f - 1] = (0 << USB_NPTXFDEP_S) | total_words;
    // flush the audio FIFO
    USB0.grstctl = (my_fifo << USB_TXFNUM_S) | USB_TXFFLSH_M;
    while (USB0.grstctl & USB_TXFFLSH_M) {}
}

static void queuePacket(uint8_t rhport) {
    s_acc += SAMPLE_RATE_HZ;
    int32_t n = s_acc / 1000;
    s_acc %= 1000;

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
                s_underruns++;
                s_priming = true;
            }
        }
    }
    if ((int32_t)filled < n) memset(s_pkt + filled * FRAME_BYTES, 0, (n - filled) * FRAME_BYTES);

    s_armed = true;
    s_last_xfer_ms = millis();
    if (!usbd_edpt_xfer(rhport, s_ep_in, s_pkt, n * FRAME_BYTES)) s_armed = false;
}

// iso IN with the old DCD: if a frame was missed, the armed packet waits for the wrong
// (even/odd) frame forever. Re-sync the parity bit to the next frame.
static void watchdogTask(void*) {
    for (;;) {
        vTaskDelay(2);
        if (s_alt != 1 || !s_armed) continue;
        if (millis() - s_last_xfer_ms < 4) continue;
        uint8_t epnum = s_ep_in & 0x7F;
        volatile usb_in_endpoint_t* in = &USB0.in_ep_reg[epnum];
        if (in->diepctl & USB_D_EPENA1_M) {
            uint32_t odd = USB0.dsts & (1u << USB_SOFFN_S);
            in->diepctl |= odd ? USB_DI_SETD0PID1 : USB_DI_SETD1PID1;
            s_parity_fix++;
        }
        s_last_xfer_ms = millis();
    }
}

// ------------------------------------------------------------------ class driver
static void uacInit(void) {
    static bool started = false;
    if (!started) {
        started = true;
        xTaskCreatePinnedToCore(watchdogTask, "uac_wd", 2048, nullptr, 5, nullptr, 1);
    }
}

static void uacReset(uint8_t rhport) {
    (void)rhport;
    s_alt = 0;
    s_armed = false;
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
    // Open the iso endpoint once per configuration (the DCD cannot free FIFOs on close)
    if (ep) {
        if (!usbd_edpt_open(rhport, ep)) return 0;
        fixFifo(s_ep_in & 0x7F);
    }
    s_alt = 0;
    s_armed = false;
    return len;
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
                s_alt = alt;
                if (alt == 1) {
                    s_need_discard = true;
                    s_priming = true;
                    if (!s_armed) queuePacket(rhport);
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

    // ---- class requests to the endpoint: sampling frequency (single fixed rate)
    if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
        req->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT) {
        if (tu_u16_high(req->wValue) != 0x01) return false;  // SAMPLING_FREQ_CONTROL
        if (req->bRequest == 0x01) {                         // SET_CUR: accept and ignore
            return tud_control_xfer(rhport, req, s_freq_buf, 3);
        }
        if (req->bRequest >= 0x81 && req->bRequest <= 0x84) {  // GET_CUR/MIN/MAX/RES
            uint32_t fs = (req->bRequest == 0x84) ? 0 : SAMPLE_RATE_HZ;
            s_freq_buf[0] = fs;
            s_freq_buf[1] = fs >> 8;
            s_freq_buf[2] = fs >> 16;
            return tud_control_xfer(rhport, req, s_freq_buf, 3);
        }
        return false;
    }
    return false;
}

static bool uacXfer(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    (void)result;
    (void)xferred_bytes;
    if (ep_addr != s_ep_in) return false;
    s_armed = false;
    s_packets++;
    if (s_alt == 1) queuePacket(rhport);
    return true;
}

static usbd_class_driver_t const s_driver = {
#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
    .name = "UAC1",
#endif
    .init = uacInit,
    .reset = uacReset,
    .open = uacOpen,
    .control_xfer_cb = uacControl,
    .xfer_cb = uacXfer,
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
uint32_t parityFixes() { return s_parity_fix; }
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
