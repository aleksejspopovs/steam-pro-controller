#include "puck_host.h"

#include <Arduino.h>
#include <USBHost_t36.h>
#include <cstring>

#include "crashlog.h"
#include "rumble.h"
#include "sc_proto.h"
#include "translate.h"

namespace puck_host {
namespace {

// ---- command channel constants (== src/linux/puck.cpp / src/pico) ----
constexpr size_t FEATURE_LEN = 64; // incl. report id byte
constexpr uint8_t CH_CONTROLLER = 0x01;
constexpr uint8_t CH_DONGLE = 0x02;
constexpr uint8_t ID_CLEAR_DIGITAL_MAPPINGS = 0x81;
constexpr uint8_t ID_SET_SETTINGS_VALUES = 0x87;
constexpr uint8_t ID_GET_SETTINGS_VALUES = 0x89;
constexpr uint8_t ID_DONGLE_GET_WIRELESS_STATE = 0xB4;
constexpr uint8_t SETTING_LIZARD_MODE = 9;
constexpr uint8_t SETTING_IMU_MODE = 48;
constexpr uint8_t SETTING_STEAM_WATCHDOG_ENABLE = 71;
constexpr uint8_t IMU_ACCEL_GYRO_ORIENT = 0x1C;

constexpr uint16_t PUCK_VID = 0x28de;
constexpr uint16_t PUCK_PID = 0x1304;

// ---- USBHost_t36 stack (host port = USB2) ----
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
// One parser per HID interface the Puck exposes (4 slots + mgmt ~= 6-7).
USBHIDParser hid1(myusb), hid2(myusb), hid3(myusb), hid4(myusb);
USBHIDParser hid5(myusb), hid6(myusb), hid7(myusb), hid8(myusb);
// NOTE: the puck first enumerates as a CDC-ACM "Steam Controller Bootloader"
// (28de:1007), then ~3.6 s later disconnects and re-enumerates as the real
// 7-interface app (28de:1304). Claiming the bootloader CDC with USBSerial
// HUNG USBHost_t36 on the disconnect and did NOT help the handoff, so it's
// gone. The open blocker is that the app re-enumeration isn't completing on
// our host port (try a powered hub between Teensy and puck).

// Enumeration spy: logs every device/interface USBHost_t36 enumerates, then
// declines so the real drivers claim. Distinguishes "dongle never enumerated"
// (no log = wiring/power/D+- swap) from "enumerated but HID claim rejected it".
class LogDriver : public USBDriver {
public:
    LogDriver() { driver_ready_for_device(this); }

protected:
    bool claim(Device_t* dev, int type, const uint8_t* desc, uint32_t len) override {
        crashlog::kv("host: enum vid", dev->idVendor);
        crashlog::kv("  pid", dev->idProduct);
        crashlog::kv("  level 0dev/1itf/2iad", static_cast<uint32_t>(type));
        // Decode this: interface desc = [bLen,04,itfNum,alt,nEP,CLASS,sub,proto,
        // iItf] then endpoint descs [bLen,05,EPaddr,attr,mps_lo,mps_hi,interval].
        // CLASS 3 = HID (USBHIDParser will claim), 0xFF = vendor (it won't).
        crashlog::bytes("  desc", desc, len < 32 ? len : 32);
        return false; // never bind; let USBHIDParser take HID interfaces
    }
    void disconnect() override {}
};
LogDriver g_logdrv;

// ---- claimed interfaces ----
constexpr int MAX_SLOTS = 8;
struct Slot {
    USBHIDParser* drv = nullptr;
    uint8_t itf = 0xFF;
};
Slot g_slots[MAX_SLOTS];
int g_nslots = 0;
int g_active = -1; // index into g_slots, -1 = none

sc::ConnMonitor g_conn;
volatile bool g_present = false;
volatile bool g_connected = false;
volatile bool g_want_probe = false;

// published snapshot: written from USB ISR (hid_process_in_data), read in loop
Snapshot g_snap;

// IMU ring: SPSC, ISR producer (head) / loop consumer (tail)
struct ImuEntry {
    xl::ImuSample s;
    uint64_t t;
};
ImuEntry g_imu[64];
volatile uint8_t g_imu_head = 0, g_imu_tail = 0;

// control channel (async sendControlPacket -> pump-and-wait)
volatile bool g_ctrl_done = false;

// init state machine (delayed + verify-by-effect, gotcha 2026-06-10)
enum class Init { Idle, Wait, Verify };
Init g_init = Init::Idle;
uint32_t g_init_at_ms = 0, g_verify_at_ms = 0;
int g_init_attempts = 0;
volatile int g_imu_changes = 0;
uint8_t g_last_imu[12] = {}; // accel(34..39)+gyro(40..45); NOT the ts counter

// rumble (device -> host): four 0x83 tones -- L grip, R grip, L pad, R pad
uint8_t g_tones[4][10] = {{0x83}, {0x83}, {0x83}, {0x83}};
volatile bool g_tone_active[4] = {false, false, false, false};
// One-shot: set when an actuator goes active->inactive so the next emit pass
// sends its stop packet (g_tones[a] now holds the stop) instead of just letting
// the tone ring out its duration. Poll-context only; not touched by the ISR.
bool g_tone_stop[4] = {false, false, false, false};
volatile bool g_rumble_active = false; // any tone active
volatile bool g_rumble_dirty = false;
uint32_t g_rumble_resend_ms = 0;

uint8_t g_logged_inputs = 0; // throttle 0x45 logging to the first few
volatile bool g_dump45_ready = false; // ISR sets after logging the 0x45 batch

// ---------------------------------------------------------------------------
// Parse a raw Puck input report (ISR context) -> snapshot + IMU ring.

void handle_report(const uint8_t* report, uint16_t len) {
    if (len < 1)
        return;

    if (report[0] == 0x79 && len >= 2) {
        sc::ConnEvent e = g_conn.feed(report[0], report + 1, len - 1);
        if (e == sc::ConnEvent::Connected) {
            g_connected = true;
            g_want_probe = true; // task() resolves which slot
            crashlog::line("puck: 79 02 connect");
        } else if (e == sc::ConnEvent::Disconnected) {
            g_connected = false;
            g_active = -1;
            Snapshot neutral;
            neutral.present = g_present;
            g_snap = neutral;
            crashlog::line("puck: 79 01 disconnect -> neutral");
        }
        return;
    }

    if (report[0] == 0x45) {
        sc::State st;
        if (!sc::parse_45(report, len, st))
            return;
        g_snap.input = xl::map_input(st);
        g_snap.connected = true;
        g_snap.present = g_present;
        ImuEntry e{xl::map_imu(st), static_cast<uint64_t>(micros())};
        uint8_t nh = (g_imu_head + 1) & 63;
        if (nh != g_imu_tail) {
            g_imu[g_imu_head] = e;
            g_imu_head = nh;
        }
        if (len >= 46) { // verify-by-effect: ACCEL+GYRO (34..45) must change.
            // Bytes 30..33 are the imu_ts counter, which can tick on its own --
            // including it here let init falsely "verify" with a dead IMU.
            if (memcmp(report + 34, g_last_imu, 12) != 0)
                g_imu_changes++;
            memcpy(g_last_imu, report + 34, 12);
        }
        if (g_logged_inputs < 8) {
            g_logged_inputs++;
            crashlog::kv("puck: 0x45 len", len);
            crashlog::bytes("  hdr[0..15]", report, len < 16 ? len : 16);
            if (len >= 46) // imu region: ts(30..33) accel(34..39) gyro(40..45)
                crashlog::bytes("  imu[30..45]", report + 30, 16);
            if (g_logged_inputs == 8)
                g_dump45_ready = true; // task() flushes the batch (SD not ISR-safe)
        }
        return;
    }

    if (report[0] == 0x43) {
        sc::Battery b;
        if (sc::parse_43(report, len, b) && b.valid()) {
            g_snap.bat_level = xl::battery_level(b.percent);
            g_snap.bat_charging = b.charging();
        }
    }
}

// ---------------------------------------------------------------------------
// USBHIDInput driver: claim the whole Puck interface, take raw reports.

class PuckHID : public USBHIDInput {
public:
    PuckHID() { USBHIDParser::driver_ready_for_hid_collection(this); }

protected:
    hidclaim_t claim_collection(USBHIDParser* driver, Device_t* dev,
                                uint32_t topusage) override {
        // Log EVERY offer so we see what HID collections exist + their EP sizes,
        // even ones we decline.
        crashlog::kv("puck: offer vid", dev->idVendor);
        crashlog::kv("  pid|topusage", (dev->idProduct << 16) | (topusage & 0xFFFF));
        crashlog::kv("  itf|in|out",
                     (driver->interfaceNumber() << 16) |
                         ((driver->inSize() & 0xFF) << 8) |
                         (driver->outSize() & 0xFF));
        if (dev->idVendor != PUCK_VID) // accept any Valve PID (this unit=0x1007)
            return CLAIM_NO;
        if (driver->inSize() > 64) // would need external RX buffers
            return CLAIM_NO;
        (void)topusage;
        for (int i = 0; i < g_nslots; i++)
            if (g_slots[i].drv == driver) { // already have this interface
                mydevice = dev;
                return CLAIM_INTERFACE;
            }
        if (g_nslots < MAX_SLOTS) {
            uint8_t itf = driver->interfaceNumber();
            g_slots[g_nslots++] = {driver, itf};
            crashlog::kv("puck: claim itf", itf);
            crashlog::kv("  in|out size",
                         (driver->inSize() << 8) | (driver->outSize() & 0xFF));
        }
        mydevice = dev;
        g_present = true;
        return CLAIM_INTERFACE; // whole interface -> raw reports, no usage parse
    }

    bool hid_process_in_data(const Transfer_t* transfer) override {
        handle_report(reinterpret_cast<const uint8_t*>(transfer->buffer),
                      transfer->length);
        return true;
    }

    bool hid_process_control(const Transfer_t*) override {
        g_ctrl_done = true; // our SET/GET_REPORT completed
        return true;
    }

    void disconnect_collection(Device_t* dev) override {
        for (int i = 0; i < g_nslots; i++)
            g_slots[i] = {nullptr, 0xFF};
        g_nslots = 0;
        g_active = -1;
        g_present = false;
        g_connected = false;
        Snapshot neutral;
        g_snap = neutral;
        crashlog::line("puck: disconnect");
        (void)dev;
    }

    // Unused: we claim the whole interface, so no usage-level parsing.
    void hid_input_begin(uint32_t, uint32_t, int, int) override {}
    void hid_input_data(uint32_t, int32_t) override {}
    void hid_input_end() override {}
};

PuckHID g_puck;

// ---------------------------------------------------------------------------
// Command channel (async control xfer pumped to completion).

// Keep the host stack serviced during a wait (never a bare delay).
void host_delay_ms(uint32_t ms) {
    uint32_t start = millis();
    do {
        myusb.Task();
    } while (millis() - start < ms);
}

bool ctrl_xfer(USBHIDParser* drv, uint8_t req_type, uint8_t req, uint16_t value,
               uint8_t itf, uint8_t* buf, uint16_t len) {
    if (!drv)
        return false;
    g_ctrl_done = false;
    if (!drv->sendControlPacket(req_type, req, value, itf, len, buf))
        return false;
    uint32_t start = millis();
    while (!g_ctrl_done) {
        myusb.Task();
        if (millis() - start > 60)
            return false; // STALL/timeout (wireless quirk -> caller retries)
    }
    return true;
}

// SET_REPORT(feature). buf[0] = report_id.
bool set_feature(int slot, uint8_t report_id, const uint8_t* body,
                 size_t body_len) {
    static uint8_t buf[FEATURE_LEN] __attribute__((aligned(32)));
    buf[0] = report_id;
    memset(buf + 1, 0, FEATURE_LEN - 1);
    memcpy(buf + 1, body, body_len < FEATURE_LEN - 1 ? body_len : FEATURE_LEN - 1);
    for (int tries = 0; tries < 50; tries++) {
        if (ctrl_xfer(g_slots[slot].drv, 0x21, 0x09 /*SET_REPORT*/,
                      (0x03 << 8) | report_id, g_slots[slot].itf, buf, FEATURE_LEN))
            return true;
        host_delay_ms(20);
    }
    return false;
}

bool get_feature(int slot, uint8_t report_id, uint8_t* out) {
    for (int tries = 0; tries < 50; tries++) {
        memset(out, 0, FEATURE_LEN);
        out[0] = report_id;
        if (ctrl_xfer(g_slots[slot].drv, 0xA1, 0x01 /*GET_REPORT*/,
                      (0x03 << 8) | report_id, g_slots[slot].itf, out, FEATURE_LEN))
            return true;
        host_delay_ms(20);
    }
    return false;
}

// data = [cmd, payload_len, payload...]; reply echoes cmd in byte[1].
bool command(int slot, uint8_t report_id, uint8_t cmd, const uint8_t* payload,
             size_t payload_len, uint8_t* reply, size_t* reply_len,
             bool expect_reply) {
    uint8_t body[FEATURE_LEN - 1] = {};
    body[0] = cmd;
    body[1] = static_cast<uint8_t>(payload_len);
    if (payload && payload_len)
        memcpy(body + 2, payload, payload_len);
    if (!set_feature(slot, report_id, body, sizeof(body)))
        return false;
    if (!expect_reply) {
        uint8_t scratch[FEATURE_LEN] __attribute__((aligned(32)));
        get_feature(slot, report_id, scratch); // flush
        return true;
    }
    for (int polls = 0; polls < 40; polls++) {
        uint8_t buf[FEATURE_LEN] __attribute__((aligned(32)));
        if (get_feature(slot, report_id, buf) && buf[1] == cmd) {
            size_t n = buf[2];
            if (n > FEATURE_LEN - 3)
                n = FEATURE_LEN - 3;
            if (reply && reply_len) {
                if (n > *reply_len)
                    n = *reply_len;
                memcpy(reply, buf + 3, n);
                *reply_len = n;
            }
            return true;
        }
        host_delay_ms(20);
    }
    return false;
}

bool set_setting(int slot, uint8_t reg, uint16_t val) {
    const uint8_t payload[3] = {reg, static_cast<uint8_t>(val & 0xFF),
                                static_cast<uint8_t>(val >> 8)};
    bool ok = true;
    for (int i = 0; i < 2; i++) { // laggy channel: write twice (sc_hid)
        ok &= command(slot, CH_CONTROLLER, ID_SET_SETTINGS_VALUES, payload, 3,
                      nullptr, nullptr, false);
        host_delay_ms(30);
    }
    return ok;
}

bool enable_gamepad_mode(int slot) {
    // Per-step logging: each control transfer can STALL on the Puck, so record
    // which one (watchdog/lizard/clear/imu) succeeds to pinpoint a hang.
    crashlog::line("puck: gamepad init START");
    bool wd = set_setting(slot, SETTING_STEAM_WATCHDOG_ENABLE, 0);
    crashlog::kv("  watchdog-off ok", wd);
    bool lz = set_setting(slot, SETTING_LIZARD_MODE, 0);
    crashlog::kv("  lizard-off ok", lz);
    bool cm = command(slot, CH_CONTROLLER, ID_CLEAR_DIGITAL_MAPPINGS, nullptr, 0,
                      nullptr, nullptr, false);
    crashlog::kv("  clear-maps ok", cm);
    bool im = set_setting(slot, SETTING_IMU_MODE, IMU_ACCEL_GYRO_ORIENT);
    crashlog::kv("  imu-mode(0x1C) ok", im);
    bool ok = wd && lz && cm && im;
    crashlog::line(ok ? "puck: gamepad-mode init sent"
                      : "puck: gamepad-mode init FAILED");
    return ok;
}

int query_connection_state(int slot) {
    uint8_t reply[8];
    size_t n = sizeof(reply);
    if (!command(slot, CH_DONGLE, ID_DONGLE_GET_WIRELESS_STATE, nullptr, 0,
                 reply, &n, true) ||
        n < 1)
        return -1;
    return reply[0]; // 1 = disconnected, 2 = connected
}

int get_setting(int slot, uint8_t reg) {
    uint8_t reply[8];
    size_t n = sizeof(reply);
    uint8_t pl[1] = {reg};
    if (!command(slot, CH_CONTROLLER, ID_GET_SETTINGS_VALUES, pl, 1, reply, &n,
                 true) ||
        n < 3 || reply[0] != reg)
        return -1;
    return reply[1] | (reply[2] << 8);
}

void set_active(int slot) {
    g_active = slot;
    g_conn.set_connected(true);
    g_connected = true;
    g_init_attempts = 0;
    g_init = Init::Wait;
    g_init_at_ms = millis() + 300; // delayed init (gotcha 2026-06-10)
    crashlog::kv("puck: active slot itf", g_slots[slot].itf);
}

} // namespace

// ---------------------------------------------------------------------------

void begin() {
    myusb.begin();
    crashlog::line("puck: USBHost_t36 up on host port");
}

void task(uint32_t now_ms) {
    myusb.Task();

    // connect probe: find the slot whose dongle reports a bound controller
    if (g_want_probe && g_active < 0) {
        g_want_probe = false;
        for (int i = 0; i < g_nslots; i++) {
            if (query_connection_state(i) == 2) {
                set_active(i);
                break;
            }
        }
    }

    // delayed gamepad-mode init + verify-by-effect re-run
    if (g_init == Init::Wait && g_active >= 0 && now_ms >= g_init_at_ms) {
        enable_gamepad_mode(g_active);
        crashlog::flush("after gamepad init attempt"); // capture even if 0x45 never comes
        g_imu_changes = 0;
        g_verify_at_ms = now_ms + 1500;
        g_init = Init::Verify;
    } else if (g_init == Init::Verify && now_ms >= g_verify_at_ms) {
        if (g_imu_changes < 50) { // ~400 expected in 1.5 s when live
            if (++g_init_attempts < 5) {
                crashlog::kv("puck: IMU not live, re-init #", g_init_attempts);
                g_init = Init::Wait;
                g_init_at_ms = now_ms + 200;
            } else {
                crashlog::line("puck: init failed 5x, await next 79 02");
                g_init = Init::Idle;
            }
        } else {
            int wd = get_setting(g_active, SETTING_STEAM_WATCHDOG_ENABLE);
            crashlog::kv("puck: init verified, imu changes", g_imu_changes);
            crashlog::kv("  watchdog(71) readback (want 0)", wd);
            g_init = Init::Idle;
        }
        // Once init resolves (verified OR gave up), flush so the SD log always
        // captures the decision + the first 0x45 IMU bytes, regardless of when
        // the controller connected relative to the idle/ceiling timers.
        if (g_init == Init::Idle)
            crashlog::flush("imu init resolved");
    }

    // Flush the first batch of 0x45 IMU bytes as soon as the ISR has them.
    if (g_dump45_ready) {
        g_dump45_ready = false;
        crashlog::flush("0x45 imu batch logged");
    }

    // rumble back-path: forward on change, resend <=40 ms while active. Each
    // active actuator is its own 0x83 output report. On the active->inactive
    // edge we send one explicit stop (g_tone_stop[a]) so the side goes quiet at
    // once rather than ringing out its tone duration. NOTE: this fires up to
    // four sendPacket() calls per pass -- confirm the host driver queues them
    // cleanly (vs. dropping/overwriting in-flight) during hardware bring-up.
    if (g_active >= 0 && (g_rumble_dirty ||
                          (g_rumble_active && now_ms >= g_rumble_resend_ms))) {
        g_rumble_dirty = false;
        if (g_slots[g_active].drv) {
            for (int a = 0; a < 4; a++) {
                if (!g_tone_active[a] && !g_tone_stop[a]) continue;
                g_tone_stop[a] = false;  // one-shot: the stop is being sent now
                static uint8_t out[FEATURE_LEN] __attribute__((aligned(32)));
                memset(out, 0, sizeof(out));
                memcpy(out, g_tones[a], sizeof(g_tones[a]));
                g_slots[g_active].drv->sendPacket(out, FEATURE_LEN);
            }
        }
        g_rumble_resend_ms = now_ms + 40;
    }
}

bool present() { return g_present; }
bool connected() { return g_connected; }

Snapshot snapshot() {
    Snapshot s;
    noInterrupts();
    s = g_snap;
    interrupts();
    s.present = g_present;
    s.connected = g_connected;
    return s;
}

bool pop_imu(xl::ImuSample& s, uint64_t& t_us) {
    if (g_imu_tail == g_imu_head)
        return false;
    s = g_imu[g_imu_tail].s;
    t_us = g_imu[g_imu_tail].t;
    g_imu_tail = (g_imu_tail + 1) & 63;
    return true;
}

void set_tones(const uint8_t tones[4][10], const bool active[4]) {
    bool any = false;
    for (int a = 0; a < 4; a++) {
        // active -> inactive this update: queue a one-shot stop so the side goes
        // quiet now instead of waiting out its tone duration. (tones[a] for an
        // inactive actuator already holds the stop packet, from to_tone.)
        if (g_tone_active[a] && !active[a])
            g_tone_stop[a] = true;
        memcpy(g_tones[a], tones[a], 10);
        g_tone_active[a] = active[a];
        any |= active[a];
    }
    g_rumble_active = any;
    g_rumble_dirty = true;
}

} // namespace puck_host
