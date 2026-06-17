// Pro Controller (057e:2009, USB) protocol emulation: 0x80 USB commands,
// 0x01/0x10 output reports (subcommands + rumble), emulated SPI flash, and
// the 0x30 input report builder. Byte-in/byte-out + tick(now_us); every rig
// (uhid harness, Teensy device port) pumps it identically.
//
// hid-nintendo's probe is the acceptance test: handshake 0x02 / baudrate
// 0x03 / handshake / no-timeout 0x04 (no reply), then subcommands 0x02
// device info, SPI cal reads, 0x40 IMU, 0x03 report mode 0x30, 0x48
// vibration, 0x30 player lights, 0x38 home LED.
#pragma once
#include <cstddef>
#include <cstdint>
#include "rumble.h"
#include "translate.h"

namespace pro {

constexpr size_t REPORT_LEN = 64;        // input reports incl. report id
constexpr uint64_t REPORT_PERIOD_US = 15000; // USB Pro Controller cadence

// Emulated SPI flash read; fills `out` (0xFF for unmapped bytes).
// Image: factory stick cal @0x603d/0x6046 (center 2048 +/-1792), IMU cal
// @0x6020 (offsets 0, scales 16384/13371 -> driver passes raw through),
// colors @0x6050, user-cal magics @0x8010/0x801b/0x8026 absent (0xFF).
void spi_read(uint32_t addr, size_t len, uint8_t* out);

// Pack three 5 ms-spaced orientation samples (first/mid/last, each a unit
// quaternion x,y,z,w) into the 36-byte motion region of a 0x30 report, choosing
// the Switch packing mode adaptively (see pro.cpp / tools/sim_quat_packing.py):
//   mode 2 for slow/steady motion (21-bit absolute, tiny delta range),
//   mode 1 for fast-but-smooth motion (16-bit endpoints, larger mid delta),
//   mode 0 for erratic motion (three independent 13-bit quaternions).
// `region` is out+13 of the report; ts_ms is the report timestamp. Exposed for
// the golden-byte / round-trip tests.
void pack_quat_motion(uint8_t region[36], const double qf[4], const double qm[4],
                      const double ql[4], uint32_t ts_ms);

// A captured SPI flash write (subcmd 0x11). We don't persist these -- the
// Switch writes user calibration here (sticks @0x8010/0x801b, 6-axis @0x8026)
// during its calibration UI -- but we hand the payload up so the harness can
// dump it to the log for manual capture. Max 0x1D bytes/write (protocol cap).
struct SpiWrite {
    uint32_t addr;
    uint8_t len;
    uint8_t data[0x1D];
};

class Controller {
public:
    // Clear all per-session handshake state (report mode, IMU/vibration enable,
    // lights, pending replies, orientation). Call when the Switch link drops so
    // the next enumeration starts a fresh handshake instead of resuming a stale
    // one. Externally-fed input/battery/IMU are left alone (re-fed each loop).
    void reset();

    // Feed one output report (incl. report id). now_us for reply pacing.
    void handle_output(const uint8_t* d, size_t len, uint64_t now_us);

    // Queued replies (0x81 / 0x21 input reports). True while one was popped.
    bool pop_reply(uint8_t out[REPORT_LEN], size_t* out_len);

    // Live input state (translated SC values).
    void set_input(const xl::SwInput& in) { input_ = in; }
    void push_imu(const xl::ImuSample& s, uint64_t t_us) { imu_.push(s, t_us); }
    void set_battery(uint8_t level, bool charging) {
        bat_level_ = level > 4 ? 4 : level;
        bat_charging_ = charging;
    }

    // 0x30 streaming pump: returns true and fills `out` when a report is
    // due (call freely; it self-paces to REPORT_PERIOD_US).
    bool tick(uint64_t now_us, uint8_t out[REPORT_LEN]);

    // Rumble back-path: latest decoded Switch rumble -> SC 0x80 packet.
    const rumble::State& rumble_state() const { return rumble_; }
    // True once after each rumble change; harness sends the packet then.
    bool take_rumble_dirty() { bool d = rumble_dirty_; rumble_dirty_ = false; return d; }

    // Pop the most recent captured SPI flash write (subcmd 0x11). Single slot:
    // the Switch serializes writes behind their acks, so the harness draining
    // it once per loop never misses one. Returns false when nothing is pending.
    bool pop_spi_write(SpiWrite& w);

    bool streaming() const { return report_mode_ == 0x30; }
    bool imu_enabled() const { return imu_enabled_; }
    bool vibration_enabled() const { return vibration_enabled_; }
    uint8_t player_lights() const { return player_lights_; }

    // Exposed for golden-byte tests.
    void build_report30(uint8_t out[REPORT_LEN], uint64_t now_us);

private:
    void queue_usb_reply(uint8_t cmd);
    void queue_subcmd_reply(uint8_t ack, uint8_t subcmd,
                            const uint8_t* data, size_t len);
    void fill_input_prefix(uint8_t* b); // timer/battery/buttons/sticks/vib

    xl::SwInput input_;
    xl::ImuResampler imu_;
    rumble::State rumble_;
    bool rumble_dirty_ = false;
    SpiWrite spi_write_;
    bool spi_write_pending_ = false;

    uint8_t bat_level_ = 4; // "no data" maps to full, never 0%
    bool bat_charging_ = false;

    uint8_t report_mode_ = 0x3F;
    bool imu_enabled_ = false;
    // The Switch enables the IMU in "mode 2" (subcmd 0x40 arg 0x02): instead of
    // raw int16 angular rate, the gyro bytes carry a packed unit quaternion of
    // the integrated orientation (see IMU.md). When set, build_report30 emits
    // that format; otherwise (arg 0x01) it streams raw rate as before.
    bool imu_quaternion_ = false;
    double orient_[4] = {0.0, 0.0, 0.0, 1.0}; // integrated orientation x,y,z,w
    bool vibration_enabled_ = false;
    uint8_t player_lights_ = 0;
    uint64_t clock_us_ = 0; // free-running source for the report timer byte
    uint64_t next_report_us_ = 0;

    static constexpr int QN = 8;
    uint8_t queue_[QN][REPORT_LEN];
    size_t qlen_[QN] = {};
    int q_head_ = 0, q_count_ = 0;
};

} // namespace pro
