// StampS3 (ESP32-S3): USB Audio Class 1.0, full speed, Arduino 2.0.x TinyUSB (dcd_esp32sx)
//
// dcd_esp32sx has no real isochronous support, so the audio IN endpoint is driven here directly:
//  - The DCD treats iso IN like bulk (no even/odd frame, DIEPTSIZ.MC = 0, FIFO filled later from
//    its TXFE interrupt): only the first packet ever left. armIn() programs the endpoint itself
//    (size + MC = 1, frame parity, enable) and pushes the whole packet into its TX FIFO.
//  - Completion is taken straight from the DCD interrupt (dcd_event_handler is wrapped, see the
//    end of the file): the next packet - already built - is armed for the next frame right there,
//    then the one after it is built. Re-arming from a task (usbd task, or a task woken from the
//    ISR) missed frames: only ~250..300 of the 1000 packets/s went out.
//  - The DCD prints "Complete but not empty" from that ISR for every such packet (polled UART0,
//    ~4 ms per line -> interrupt watchdog). The only switch is the global IDF log level, so IDF
//    logging is off while the host streams.
//  - uacTask (pinned to the USB interrupt core, works inside s_mux = the ISR cannot run) starts a
//    stream, aborts on alt 0 and re-syncs the frame parity of a packet that missed its frame.
#include "../usb_audio.h"

#include <Arduino.h>

#include "../capture.h"
#include "config.h"

#if USB_AUDIO_AVAILABLE && (ARDUINO_USB_MODE == 0)

#include "esp32-hal-tinyusb.h"
#include "device/dcd.h"
#include "device/usbd_pvt.h"
#include "soc/usb_periph.h"

namespace usb_audio {

// ------------------------------------------------------------------ descriptor
// UAC1:  IAD + AC(std, header, IT, OT) + AS alt0 + AS alt1(general, format, ep, cs ep)
#define UAC_DESC_LEN (8 + 9 + 9 + 12 + 9 + 9 + 9 + 7 + 11 + 9 + 7)
// Channel config (spatial locations). The channels are plain microphones, but Windows WASAPI
// exclusive mode (PortAudio) asks for the standard mask of the channel count - 8: 7.1
// (L R C LFE Ls Rs Lc Rc = 0x00FF), 4: quad (L R Ls Rs = 0x0033) - and refuses the format
// ("Invalid sample rate") if the device reports none (0). Order and data are unchanged.
#if NUM_CH == 8
#define UAC_CH_CONFIG 0x00FF
#elif NUM_CH == 4
#define UAC_CH_CONFIG 0x0033
#else
#define UAC_CH_CONFIG 0x0000
#endif
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
        // Input terminal: microphone, NUM_CH channels, wChannelConfig = UAC_CH_CONFIG
        12, 0x24, 0x02, UAC_IT_ID, 0x01, 0x02, 0, NUM_CH, (uint8_t)(UAC_CH_CONFIG & 0xFF),
        (uint8_t)(UAC_CH_CONFIG >> 8), 0, 0,
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
static volatile bool s_req_abort = false;
static volatile uint32_t s_last_xfer_ms = 0;
static volatile uint32_t s_underruns = 0, s_parity_fix = 0, s_packets = 0, s_aborts = 0;
static uint32_t s_acc = 0;
static int32_t s_avg_level = 0;  // x16
static uint8_t s_freq_buf[4];
static uint8_t s_alt_buf;

// double buffer: one packet in the FIFO / on the wire, the next one already built
static uint8_t s_buf[2][UAC_MAX_PKT_BYTES + 4] __attribute__((aligned(4)));
static uint32_t s_buf_len[2];
static volatile uint8_t s_next = 0;
static volatile bool s_ready = false;
static uint8_t s_zero[UAC_MAX_PKT_BYTES + 4] __attribute__((aligned(4)));  // DCD-primed first packet
static volatile bool s_dcd_primed = false;

static TaskHandle_t s_task = nullptr;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static const int32_t TARGET_FRAMES = 2 * I2S_DMA_FRAMES + SAMPLE_RATE_HZ / 1000 * 2;

static inline uint32_t frameNumber() { return (USB0.dsts >> USB_SOFFN_S) & 0x3FFF; }

static inline void kick() {
    if (s_task) xTaskNotifyGive(s_task);
}

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

// Arm one packet for the frame of parity odd_frame and push it into the EP's TX FIFO.
// Returns false (nothing armed) if the FIFO has no room for the whole packet: writing to a full
// FIFO stalls the bus and would hang the CPU.
static bool armIn(const uint8_t* pkt, uint32_t bytes, bool odd_frame) {
    uint8_t epnum = s_ep_in & 0x7F;
    volatile usb_in_endpoint_t* in = &USB0.in_ep_reg[epnum];
    uint32_t w = (bytes + 3) / 4;
    if ((in->dtxfsts & 0xFFFF) < w) return false;  // free words in this EP's TX FIFO
    in->dieptsiz = (1u << 29) | (1u << USB_D_PKTCNT1_S) | bytes;
    in->diepctl |= (odd_frame ? USB_DI_SETD1PID1 : USB_DI_SETD0PID1) | USB_D_EPENA1_M | USB_D_CNAK1_M;
    volatile uint32_t* fifo = USB0.fifo[epnum];
    const uint32_t* src = (const uint32_t*)pkt;
    for (uint32_t i = 0; i < w; i++) *fifo = src[i];
    // XferSize counts down as the FIFO is written. Wait until it reads 0: the DCD ISR looks at this
    // EP right after us (raw TXFE) and would copy XferSize bytes from its own transfer record.
    for (int i = 0; i < 50 && (in->dieptsiz & 0x7FFFF); i++) esp_rom_delay_us(1);
    return true;
}

// host left the streaming alt setting with a packet still armed: disable the EP and flush it
static void abortIn() {
    uint8_t epnum = s_ep_in & 0x7F;
    volatile usb_in_endpoint_t* in = &USB0.in_ep_reg[epnum];
    if (in->diepctl & USB_D_EPENA1_M) {
        in->diepctl |= USB_DI_SNAK1_M | USB_D_EPDIS1_M;
        for (int i = 0; i < 2000 && (in->diepctl & USB_D_EPENA1_M); i++) esp_rom_delay_us(1);
    }
    uint8_t f = (in->diepctl >> USB_D_TXFNUM1_S) & USB_D_TXFNUM1_V;
    USB0.grstctl = (f << USB_TXFNUM_S) | USB_TXFFLSH_M;
    for (int i = 0; i < 2000 && (USB0.grstctl & USB_TXFFLSH_M); i++) esp_rom_delay_us(1);
    s_armed = false;
    s_aborts++;
}

// build the next packet (ring -> s_buf[s_next])
static void buildPacket() {
    uint8_t* pkt = s_buf[s_next];
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
            filled = r.read(pkt, n);
            if ((int32_t)filled < n) {
                s_underruns++;
                s_priming = true;
            }
        }
    }
    if ((int32_t)filled < n) memset(pkt + filled * FRAME_BYTES, 0, (n - filled) * FRAME_BYTES);
    s_buf_len[s_next] = n * FRAME_BYTES;
    __asm__ __volatile__("" ::: "memory");  // packet complete before the ISR may see s_ready
    s_ready = true;
}

// arm the prepared packet (caller: ISR, or uacTask inside s_mux - both on the USB interrupt core)
// (FIFO without room: nothing armed, the packet stays ready; uacTask flushes / retries)
static void armReady(bool odd_frame) {
    s_last_xfer_ms = millis();
    if (!armIn(s_buf[s_next], s_buf_len[s_next], odd_frame)) return;
    s_armed = true;
    s_next ^= 1;
    s_ready = false;
}

// Packet completion, from the DCD interrupt (__wrap_dcd_event_handler): arm the prepared packet
// for the next frame (the IN token of this frame has just been served), then build the next one
// from the lock-free ring (a few us).
static void onXferComplete(bool in_isr) {
    s_armed = false;
    s_packets++;
    if (in_isr && s_alt == 1 && !s_req_abort) {
        if (!s_ready) buildPacket();
        armReady(!(frameNumber() & 1));
        if (!s_ready) buildPacket();
        if (s_armed) return;
        // FIFO had no room: uacTask flushes it and starts again
    }
    if (!s_task) return;
    if (in_isr) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_task, &woken);
        if (woken) portYIELD_FROM_ISR();
    } else {
        xTaskNotifyGive(s_task);
    }
}

// Starts a stream (first packet), aborts on alt 0, re-syncs the frame parity of a packet that
// missed its frame (it would wait for the wrong even/odd frame forever), and switches IDF logging.
// Pinned to the USB interrupt core: inside s_mux the ISR cannot run, so there is no race on
// s_armed / s_ready / s_next, the ring or the endpoint registers.
static void uacTask(void*) {
    bool quiet = false;
    esp_log_level_t saved_level = ESP_LOG_ERROR;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, 2);
        if ((s_alt == 1) != quiet) {  // no IDF log output while streaming (see top of file)
            quiet = !quiet;
            if (quiet) {
                saved_level = esp_log_level_get("*");
                esp_log_level_set("*", ESP_LOG_NONE);
            } else {
                esp_log_level_set("*", saved_level);
            }
        }
        if (s_req_abort) {
            portENTER_CRITICAL(&s_mux);
            s_req_abort = false;
            if (s_armed) abortIn();
            s_ready = false;
            portEXIT_CRITICAL(&s_mux);
        }
        if (s_alt != 1) continue;

        portENTER_CRITICAL(&s_mux);
        if (!s_armed && s_dcd_primed && (USB0.in_ep_reg[s_ep_in & 0x7F].dtxfsts & 0xFFFF) < UAC_MAX_PKT_BYTES / 4)
            abortIn();  // not armed but the FIFO still holds data: flush it before arming again
        if (!s_armed && !s_dcd_primed) {
            // First packet after the configuration: a silent max-size packet through the DCD
            // itself, so that its transfer record for this EP points at a real buffer of
            // UAC_MAX_PKT_BYTES (its TXFE handling reads from it). MC and frame parity added here,
            // before the ISR fills the FIFO.
            s_dcd_primed = true;
            s_armed = true;
            s_last_xfer_ms = millis();
            if (dcd_edpt_xfer(0, s_ep_in, s_zero, UAC_MAX_PKT_BYTES)) {
                volatile usb_in_endpoint_t* in = &USB0.in_ep_reg[s_ep_in & 0x7F];
                in->dieptsiz = (in->dieptsiz & ~(3u << 29)) | (1u << 29);
                in->diepctl |= (frameNumber() & 1) ? USB_DI_SETD0PID1 : USB_DI_SETD1PID1;
            } else {
                s_armed = false;
            }
            if (!s_ready) buildPacket();
        } else if (!s_armed) {
            if (!s_ready) buildPacket();
            armReady(!(frameNumber() & 1));
            if (!s_ready) buildPacket();
        }
        portEXIT_CRITICAL(&s_mux);

        if (millis() - s_last_xfer_ms < 4) continue;
        portENTER_CRITICAL(&s_mux);
        volatile usb_in_endpoint_t* in = &USB0.in_ep_reg[s_ep_in & 0x7F];
        if (in->diepctl & USB_D_EPENA1_M) {
            uint32_t odd = USB0.dsts & (1u << USB_SOFFN_S);
            in->diepctl |= odd ? USB_DI_SETD0PID1 : USB_DI_SETD1PID1;
            s_parity_fix++;
        }
        portEXIT_CRITICAL(&s_mux);
        s_last_xfer_ms = millis();
    }
}

// ------------------------------------------------------------------ class driver
// Called from tud_init(), in the same task that then installs the USB interrupt (dcd_int_enable)
// -> xPortGetCoreID() here is the core that runs the DCD ISR.
static void uacInit(void) {
    if (!s_task)
        xTaskCreatePinnedToCore(uacTask, "uac", 3072, nullptr, configMAX_PRIORITIES - 1, &s_task,
                                xPortGetCoreID());
}

static void uacReset(uint8_t rhport) {
    (void)rhport;
    s_alt = 0;
    s_req_abort = true;
    kick();
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
    s_dcd_primed = false;
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
                if (alt == 1) {
                    s_need_discard = true;
                    s_priming = true;
                } else {
                    s_req_abort = true;
                }
                s_alt = alt;
                kick();
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
    (void)rhport;
    (void)result;
    (void)xferred_bytes;
    if (ep_addr != s_ep_in) return false;
    onXferComplete(false);  // only without the dcd_event_handler wrap (normally handled in the ISR)
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
uint32_t aborts() { return s_aborts; }

}  // namespace usb_audio

extern "C" usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count) {
    *driver_count = 1;
    return &usb_audio::s_driver;
}

// Linked with -Wl,--wrap=dcd_event_handler (platformio.ini): every event the DCD raises passes
// here first. Completion of the audio endpoint is taken directly in the interrupt; everything
// else goes on to TinyUSB.
extern "C" void __real_dcd_event_handler(dcd_event_t const* event, bool in_isr);
extern "C" void __wrap_dcd_event_handler(dcd_event_t const* event, bool in_isr) {
    if (event->event_id == DCD_EVENT_XFER_COMPLETE && usb_audio::s_ep_in &&
        event->xfer_complete.ep_addr == usb_audio::s_ep_in) {
        usb_audio::onXferComplete(in_isr);
        return;
    }
    __real_dcd_event_handler(event, in_isr);
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
uint32_t aborts() { return 0; }
}  // namespace usb_audio

#endif
