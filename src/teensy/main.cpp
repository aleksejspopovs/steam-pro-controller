#include <Arduino.h>

#include "crashlog.h"
#include "pro.h"
#include "pro_device.h"
#include "puck_host.h"
#include "rumble.h"

namespace {

pro::Controller g_pro;

uint64_t now_us() {
    static uint32_t prior = 0;
    static uint64_t high = 0;
    const uint32_t low = micros();
    if (low < prior)
        high += UINT64_C(1) << 32;
    prior = low;
    return high | low;
}

void send_replies() {
    uint8_t report[pro::REPORT_LEN];
    size_t len = 0;
    while (g_pro.pop_reply(report, &len))
        pro_device::queue_report(report, len);
}

void on_output(const uint8_t* data, size_t len) {
    g_pro.handle_output(data, len, now_us());
    send_replies();
}

// Lowest unmet bring-up stage -> LED blink period (ms). Slower = further along.
// Host side comes first now: we don't present to the Switch until a Steam
// Controller is live, so pro_device::mounted() can't be true before connected().
uint32_t status_blink_ms() {
    if (!puck_host::present())     return 150; // no dongle on the host port
    if (!puck_host::connected())   return 300; // dongle up, no Steam Controller
    if (!pro_device::mounted())    return 500; // SC live; presenting, Switch enumerating
    if (!g_pro.streaming())        return 800; // enumerated, Switch not streaming
    return 1000;                               // live: input flowing both ways
}

} // namespace

void serialEvent() {}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial1.begin(115200);

    // SD + CrashReport BEFORE USB (see crashlog.h): card init must not stall
    // the device-side enumeration.
    crashlog::init();

    // Device side first so the Switch sees us promptly, then the host port.
    pro_device::begin(on_output);
    crashlog::line("usb device stack up");
    puck_host::begin();
    crashlog::line("puck host up");
    Serial1.println("Teensy SC->Switch adapter ready");
}

void loop() {
    const uint64_t now = now_us();
    const uint32_t now_ms = millis();

    // ---- host side: pump the Puck, pull its state into core/ ----
    puck_host::task(now_ms);

    puck_host::Snapshot snap = puck_host::snapshot();

    // ---- connection lifecycle ----
    // The Switch only sees the Pro Controller while a Steam Controller is live
    // on the host port (dongle present + a controller bound). On the rising
    // edge we wipe the handshake state and bring the D+ pullup up; when the SC
    // drops we disconnect, so the Switch sees an unplug and the next reconnect
    // re-enumerates cleanly.
    const bool sc_live = snap.present && snap.connected;
    static bool was_live = false;
    if (sc_live && !was_live) {
        g_pro.reset();
        pro_device::connect();
        crashlog::line("lifecycle: SC connected -> presenting to Switch");
    } else if (!sc_live && was_live) {
        pro_device::disconnect();
        crashlog::line("lifecycle: SC dropped -> Switch link down");
    }
    was_live = sc_live;

    g_pro.set_input(snap.input);
    g_pro.set_battery(snap.bat_level, snap.bat_charging);
    xl::ImuSample s;
    uint64_t t_us;
    while (puck_host::pop_imu(s, t_us))
        g_pro.push_imu(s, t_us);

    // ---- device side: service the Switch, stream 0x30 ----
    pro_device::task();
    send_replies();

    // SPI flash write capture: the Switch saves user calibration via subcmd
    // 0x11. We log it (and flush to SD) here in the main loop -- the write is
    // captured in USB-callback context, but crashlog::flush does SD I/O that
    // must not run from there.
    pro::SpiWrite sw;
    if (g_pro.pop_spi_write(sw)) {
        crashlog::kv("spi write addr", sw.addr);
        crashlog::bytes("  data", sw.data, sw.len);
        crashlog::flush("spi write captured");
    }

    uint8_t report[pro::REPORT_LEN];
    const bool built = g_pro.tick(now, report);
    if (built)
        pro_device::queue_report(report, sizeof(report));

    // ---- rumble back-path: device -> host on change ----
    // -DNO_RUMBLE: diagnostic build that never drives the grips, to tell a
    // genuine IMU bug from the gyro/accel physically picking up an over-strong
    // rumble. We still drain the dirty flag so the device side behaves normally
    // (the Switch's vibration enable is still acked); we just never forward.
#ifndef NO_RUMBLE
    if (g_pro.take_rumble_dirty()) {
        const rumble::State& rs = g_pro.rumble_state();
        puck_host::set_rumble(rs.packet().bytes, rs.active());
    }
#else
    (void)g_pro.take_rumble_dirty();
#endif

    // Milestone once: handshake reached 0x30 streaming.
    static bool was_streaming = false;
    if (g_pro.streaming() && !was_streaming)
        crashlog::line("pro:streaming 0x30 started");
    was_streaming = g_pro.streaming();

    // Bring-up log capture: flush the RAM buffer to SD once host activity
    // settles or the ceiling elapses (captures enumerate + pair + puck init).
    if (!crashlog::flushed() && crashlog::should_flush(now_ms))
        crashlog::flush("host idle / 30s ceiling");

    // ---- status LED ----
    static uint32_t next_led_ms = 0;
    if (static_cast<int32_t>(now_ms - next_led_ms) >= 0) {
        next_led_ms = now_ms + status_blink_ms();
        digitalToggleFast(LED_BUILTIN);
    }
}
