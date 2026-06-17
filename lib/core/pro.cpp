#include "pro.h"
#include <cstring>
#include <cmath>

namespace pro {

// ---- emulated SPI flash ----

static void pack12(uint8_t* p, uint16_t a, uint16_t b) {
    p[0] = (uint8_t)(a & 0xFF);
    p[1] = (uint8_t)((a >> 8) | ((b & 0xF) << 4));
    p[2] = (uint8_t)(b >> 4);
}

struct SpiRegion { uint32_t addr; uint8_t len; uint8_t data[26]; };

static const SpiRegion* spi_regions(size_t* n) {
    static SpiRegion regions[8];
    static bool init = false;
    if (!init) {
        init = true;
        // IMU factory cal @0x6020. The acc offset is the result of running the
        // Switch's in-console 6-axis calibration against our own emitted IMU
        // data and capturing the user-cal it wrote to 0x8026 (subcmd 0x11):
        // -117,-133,276 vs the genuine unit's -8,-24,276. Hardcoding it as the
        // factory block makes the Switch read us calibrated at rest without the
        // user ever calibrating (the 0x8026 user-cal magic stays absent/0xFF, so
        // this factory block is what's applied). acc scale / gyr offset / gyr
        // scale are unchanged from the genuine unit.
        //   acc offset -117,-133,276  acc scale 16384
        //   gyr offset 0,0,0          gyr scale 13371
        SpiRegion& imu = regions[0];
        imu.addr = 0x6020; imu.len = 24;
        {
            const uint8_t cal[24] = {
                0x8b, 0xff, 0x7b, 0xff, 0x14, 0x01, // acc offset (in-console cal)
                0x00, 0x40, 0x00, 0x40, 0x00, 0x40, // acc scale 16384
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // gyr offset 0 (our gyro is
                                                    // already bias-free; the real
                                                    // unit's -10/-20/-10 would add
                                                    // ~1.5 dps phantom at rest)
                0x3b, 0x34, 0x3b, 0x34, 0x3b, 0x34, // gyr scale 13371
            };
            memcpy(imu.data, cal, sizeof(cal));
        }
        // Left stick factory cal @0x603d: [max_above][center][min_below],
        // each pair 12-bit packed. center 2048, throw 1792 both ways.
        SpiRegion& ls = regions[1];
        ls.addr = 0x603d; ls.len = 9;
        pack12(ls.data + 0, xl::STICK_RANGE, xl::STICK_RANGE);
        pack12(ls.data + 3, xl::STICK_CENTER, xl::STICK_CENTER);
        pack12(ls.data + 6, xl::STICK_RANGE, xl::STICK_RANGE);
        // Right stick @0x6046: [center][min_below][max_above].
        SpiRegion& rs = regions[2];
        rs.addr = 0x6046; rs.len = 9;
        pack12(rs.data + 0, xl::STICK_CENTER, xl::STICK_CENTER);
        pack12(rs.data + 3, xl::STICK_RANGE, xl::STICK_RANGE);
        pack12(rs.data + 6, xl::STICK_RANGE, xl::STICK_RANGE);
        // Colors @0x6050: body RGB, buttons RGB (grey/orange).
        SpiRegion& col = regions[3];
        col.addr = 0x6050; col.len = 6;
        const uint8_t c[6] = {0x32, 0x32, 0x32, 0xCC, 0x78, 0x5c};
        memcpy(col.data, c, 6);
        // User-cal magics: 0xFF = absent (default fill covers them, but be
        // explicit about the probed addresses).
        regions[4] = SpiRegion{0x8010, 2, {0xFF, 0xFF}};
        regions[5] = SpiRegion{0x801b, 2, {0xFF, 0xFF}};
        // 0x6080: 6-axis horizontal offset + factory stick params, VERBATIM
        // from a genuine Pro Controller. The Switch reads this 24 B block and
        // subtracts the horizontal offset from accel (imu_sensor_notes.md:
        // acc_leveled = (raw - horiz_offset) * coeff). Our previous synthetic
        // (0,0,4096) left a ~0.15g sideways residual at rest (our flat accel is
        // ~(-626,-20,4156)); since our raw matches a real unit, the real unit's
        // horiz offset (-688,0,4038) is the right thing to serve. hid-nintendo
        // never reads this region, so Linux never exposed it.
        SpiRegion& hz = regions[6];
        hz.addr = 0x6080; hz.len = 24;
        {
            const uint8_t d[24] = {
                0x50, 0xfd, 0x00, 0x00, 0xc6, 0x0f, // horiz offset -688,0,4038
                0x0f, 0x30, 0x61, 0x96, 0x30, 0xf3, 0xd4, 0x14,
                0x54, 0x41, 0x15, 0x54, 0xc7, 0x79, 0x9c, 0x33, 0x36, 0x63,
            };
            memcpy(hz.data, d, sizeof(d));
        }
        // 0x6098: factory stick device params 2 (same block); Switch reads 18 B.
        SpiRegion& sp2 = regions[7];
        sp2.addr = 0x6098; sp2.len = 18;
        {
            const uint8_t d[18] = {
                0x0f, 0x30, 0x61, 0x96, 0x30, 0xf3, 0xd4, 0x14, 0x54,
                0x41, 0x15, 0x54, 0xc7, 0x79, 0x9c, 0x33, 0x36, 0x63,
            };
            memcpy(sp2.data, d, sizeof(d));
        }
    }
    *n = sizeof(regions) / sizeof(regions[0]);
    return regions;
}

void spi_read(uint32_t addr, size_t len, uint8_t* out) {
    memset(out, 0xFF, len);
    size_t n;
    const SpiRegion* rs = spi_regions(&n);
    for (size_t r = 0; r < n; r++) {
        for (size_t i = 0; i < len; i++) {
            uint32_t a = addr + (uint32_t)i;
            if (a >= rs[r].addr && a < rs[r].addr + rs[r].len)
                out[i] = rs[r].data[a - rs[r].addr];
        }
    }
}

// ---- controller state machine ----

static const uint8_t MAC[6] = {0x7C, 0xBB, 0x8A, 0x12, 0x34, 0x56};

void Controller::reset() {
    report_mode_ = 0x3F;
    imu_enabled_ = false;
    imu_quaternion_ = false;
    orient_[0] = orient_[1] = orient_[2] = 0.0;
    orient_[3] = 1.0;
    vibration_enabled_ = false;
    player_lights_ = 0;
    next_report_us_ = 0;
    q_head_ = 0;
    q_count_ = 0;
    rumble_dirty_ = false;
    spi_write_pending_ = false;
}

void Controller::queue_usb_reply(uint8_t cmd) {
    if (q_count_ >= QN) return;
    int slot = (q_head_ + q_count_++) % QN;
    memset(queue_[slot], 0, REPORT_LEN);
    queue_[slot][0] = 0x81;
    queue_[slot][1] = cmd;
    qlen_[slot] = REPORT_LEN;
}

void Controller::fill_input_prefix(uint8_t* b) {
    // Report timer: a real Pro Controller exposes a free-running ~200 Hz clock
    // here (~1 tick / 5 ms IMU sample, ~3 per 15 ms report), and the Switch uses
    // it to time the 3 IMU samples -- robust to delivery jitter. Our old
    // `timer_++` was a call counter conveying no real time, so any USB jitter
    // made the Switch mis-integrate clean gyro into shake. (hid-nintendo ignores
    // this byte, which is why Linux never showed it.)
    b[1] = (uint8_t)((clock_us_ / 5000u) & 0xFF);
    // Battery byte: level in bits 7-5, charging bit 0x10, "USB powered" 0x01.
    // The Steam Controller is wireless behind the dongle, so its pack is only
    // charging when the SC itself is on the cable -- report the bit the SC sends
    // rather than hardcoding it on (a real wired Pro Controller is always
    // charging; we are not that). bat_charging_ comes from the SC 0x43 battery
    // report via set_battery().
    b[2] = (uint8_t)((bat_level_ << 5) | (bat_charging_ ? 0x10 : 0x00) | 0x01);
    b[3] = (uint8_t)(input_.buttons & 0xFF);
    // Byte-4 bit7 = "charging grip / wired" connection flag. A real USB Pro
    // Controller sets it (we were sending 0). The Switch keys IMU sample timing
    // off wired-vs-wireless (USB 15 ms vs BT 8 ms); without this flag it times
    // our 15 ms stream against the wrong rate -> jumpy gyro + failed "still"
    // calibration. hid-nintendo ignores all this, so Linux never showed it.
    b[4] = (uint8_t)(((input_.buttons >> 8) & 0xFF) | 0x80);
    b[5] = (uint8_t)((input_.buttons >> 16) & 0xFF);
    pack12(b + 6, input_.lx, input_.ly);
    pack12(b + 9, input_.rx, input_.ry);
    b[12] = 0x0C; // vibrator_report: nonzero keeps the rumble worker alive
}

void Controller::queue_subcmd_reply(uint8_t ack, uint8_t subcmd,
                                    const uint8_t* data, size_t len) {
    if (q_count_ >= QN) return;
    int slot = (q_head_ + q_count_++) % QN;
    uint8_t* b = queue_[slot];
    memset(b, 0, REPORT_LEN);
    b[0] = 0x21;
    fill_input_prefix(b);
    b[13] = ack;
    b[14] = subcmd;
    if (len > 35) len = 35;
    if (data && len) memcpy(b + 15, data, len);
    qlen_[slot] = REPORT_LEN;
}

bool Controller::pop_reply(uint8_t out[REPORT_LEN], size_t* out_len) {
    if (q_count_ == 0) return false;
    memcpy(out, queue_[q_head_], REPORT_LEN);
    *out_len = qlen_[q_head_];
    q_head_ = (q_head_ + 1) % QN;
    q_count_--;
    return true;
}

bool Controller::pop_spi_write(SpiWrite& w) {
    if (!spi_write_pending_) return false;
    w = spi_write_;
    spi_write_pending_ = false;
    return true;
}

void Controller::handle_output(const uint8_t* d, size_t len, uint64_t now_us) {
    if (len < 2) return;
    clock_us_ = now_us; // keep the timer byte advancing for 0x21 subcmd replies
    switch (d[0]) {
    case 0x80: // USB command
        switch (d[1]) {
        case 0x01: { // conn status: ack + type + MAC
            if (q_count_ >= QN) break;
            int slot = (q_head_ + q_count_++) % QN;
            uint8_t* b = queue_[slot];
            memset(b, 0, REPORT_LEN);
            b[0] = 0x81; b[1] = 0x01; b[2] = 0x00; b[3] = 0x03; // Pro
            memcpy(b + 4, MAC, 6);
            qlen_[slot] = REPORT_LEN;
            break;
        }
        case 0x02: // handshake
        case 0x03: // baudrate 3M
            queue_usb_reply(d[1]);
            break;
        case 0x04: // no timeout -- no reply by design
        case 0x05: // enable timeout
            break;
        default:
            break;
        }
        break;

    case 0x01: { // rumble + subcommand
        if (len < 11) {
            if (len >= 10) { // rumble-only sized 0x01 is malformed; ignore
            }
            break;
        }
        if (rumble_.update(d + 2)) rumble_dirty_ = true;
        uint8_t sub = d[10];
        const uint8_t* arg = d + 11;
        size_t argn = len - 11;
        switch (sub) {
        case 0x01: { // Bluetooth manual pairing (multi-step; arg[0] = step)
            // The real Switch runs this over USB to register the controller;
            // hid-nintendo never does, so it was untested until the real-Switch
            // crash log (subcmd 0x01 stub-acked -> console re-enumerates and
            // errors). Step 1 (arg 0x01) must return our BD_ADDR. Byte order
            // matches the 0x80/0x01 + device-info MAC the Switch already
            // accepted (forward). Steps 2/3 (LTK exchange / save) are acked so
            // the console advances and the crash log reveals their requests --
            // exact payloads TBD from the next on-hardware capture.
            uint8_t step = argn >= 1 ? arg[0] : 0;
            uint8_t pair[1 + 6] = {0x03,
                                   MAC[0], MAC[1], MAC[2],
                                   MAC[3], MAC[4], MAC[5]};
            (void)step;
            queue_subcmd_reply(0x81, sub, pair, sizeof(pair));
            break;
        }
        case 0x02: { // device info
            // [fw_major, fw_minor, type, 02, mac[6], 01, colors-in-spi]
            uint8_t info[12] = {0x04, 0x21, 0x03, 0x02,
                                MAC[0], MAC[1], MAC[2], MAC[3], MAC[4], MAC[5],
                                0x01, 0x01};
            queue_subcmd_reply(0x82, sub, info, sizeof(info));
            break;
        }
        case 0x03: // report mode
            if (argn >= 1) report_mode_ = arg[0];
            if (streaming() && next_report_us_ == 0)
                next_report_us_ = now_us;
            queue_subcmd_reply(0x80, sub, nullptr, 0);
            break;
        case 0x04: // triggers elapsed
            queue_subcmd_reply(0x83, sub, nullptr, 0);
            break;
        case 0x08: // low power mode
            queue_subcmd_reply(0x80, sub, nullptr, 0);
            break;
        case 0x10: { // SPI flash read
            if (argn < 5) break;
            uint32_t addr = (uint32_t)arg[0] | ((uint32_t)arg[1] << 8) |
                            ((uint32_t)arg[2] << 16) | ((uint32_t)arg[3] << 24);
            uint8_t size = arg[4];
            if (size > 0x1D) size = 0x1D;
            uint8_t reply[5 + 0x1D];
            memcpy(reply, arg, 5);
            spi_read(addr, size, reply + 5);
            queue_subcmd_reply(0x90, sub, reply, 5u + size);
            break;
        }
        case 0x11: { // SPI flash write: capture the payload, ack success.
            // We have no writable flash; the Switch only writes here to save
            // user calibration. Stash the addr+bytes for the harness to log,
            // then reply status 0x00 so the console treats the save as done.
            if (argn < 5) break;
            uint32_t addr = (uint32_t)arg[0] | ((uint32_t)arg[1] << 8) |
                            ((uint32_t)arg[2] << 16) | ((uint32_t)arg[3] << 24);
            size_t n = arg[4];
            size_t avail = argn - 5;       // guard a short/truncated report
            if (n > avail) n = avail;
            if (n > sizeof(spi_write_.data)) n = sizeof(spi_write_.data);
            spi_write_.addr = addr;
            spi_write_.len = (uint8_t)n;
            memcpy(spi_write_.data, arg + 5, n);
            spi_write_pending_ = true;
            uint8_t status = 0x00; // 0x00 = success, 0x01 = write-protected
            queue_subcmd_reply(0x80, sub, &status, 1);
            break;
        }
        case 0x30: // player lights
            if (argn >= 1) player_lights_ = arg[0];
            queue_subcmd_reply(0x80, sub, nullptr, 0);
            break;
        case 0x40: // IMU enable (arg 0x01 = raw rate, arg 0x02 = quaternion mode)
            if (argn >= 1) {
                imu_enabled_ = arg[0] != 0;
                bool quat = (arg[0] == 0x02);
                if (quat && !imu_quaternion_) { // entering mode 2: reset orientation
                    orient_[0] = orient_[1] = orient_[2] = 0.0;
                    orient_[3] = 1.0;
                }
                imu_quaternion_ = quat;
            }
            queue_subcmd_reply(0x80, sub, nullptr, 0);
            break;
        case 0x48: // vibration enable
            if (argn >= 1) vibration_enabled_ = arg[0] != 0;
            queue_subcmd_reply(0x80, sub, nullptr, 0);
            break;
        case 0x50: { // regulated voltage, units of 2.5 mV (1600 = 4.00 V)
            uint8_t v[2] = {0x40, 0x06};
            queue_subcmd_reply(0xD0, sub, v, 2);
            break;
        }
        default: // 0x21 MCU cfg, 0x38 home LED, 0x41 IMU sensitivity, ...
            queue_subcmd_reply(0x80, sub, nullptr, 0);
            break;
        }
        break;
    }

    case 0x10: // rumble only
        if (len >= 10 && rumble_.update(d + 2))
            rumble_dirty_ = true;
        break;

    default:
        break;
    }
}

namespace {

inline void wr16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
inline void wr_s16(uint8_t* p, int16_t v) { wr16(p, (uint16_t)v); }

// Advance the orientation quaternion by one IMU sample over `dt` seconds.
// Ported from ndeadly's QuaternionMotionPacker::UpdateRotationState
// (reference/switch_motion_packing.cpp): convert gyro (in Switch LSB) to the
// incremental rotation quaternion using Nintendo's polynomial sin/cos
// approximations, then q = normalize(q (x) increment).
void integrate_orientation(double q[4], const xl::ImuSample& s, double dt) {
    // Switch gyro units: 14.247 LSB/dps (translate.h). -> radians of rotation.
    constexpr double LSB_TO_RAD = (1.0 / 14.247) * 3.14159265358979323846 / 180.0;
    double ax = s.gx * LSB_TO_RAD * dt;
    double ay = s.gy * LSB_TO_RAD * dt;
    double az = s.gz * LSB_TO_RAD * dt;

    double n2 = ax * ax + ay * ay + az * az;
    double vscale = n2 * n2 / 3840.0 - n2 / 48.0 + 0.5; // ~ sin(|a|/2)/|a|
    double scalar = n2 * n2 / 384.0 - n2 / 8.0 + 1.0;    // ~ cos(|a|/2)
    double ix = ax * vscale, iy = ay * vscale, iz = az * vscale, iw = scalar;

    double sx = q[0], sy = q[1], sz = q[2], sw = q[3]; // state (x) increment
    double rx = sw * ix + sx * iw + sy * iz - sz * iy;
    double ry = sw * iy + sy * iw + sz * ix - sx * iz;
    double rz = sw * iz + sz * iw + sx * iy - sy * ix;
    double rw = sw * iw - sx * ix - sy * iy - sz * iz;

    double inv = 1.0 / sqrt(rx * rx + ry * ry + rz * rz + rw * rw);
    q[0] = rx * inv; q[1] = ry * inv; q[2] = rz * inv; q[3] = rw * inv;
}

// The packed quaternion triplet is one contiguous LSB-first 144-bit
// little-endian stream laid into the three 6-byte gaps the accel samples leave
// at region bytes 6-11 / 18-23 / 30-35. Fields are appended in struct-
// declaration order (reference/switch_motion_packing.hpp); a field that crosses
// a 48-bit gap join is exactly the header's _l/_h split. GyroBitWriter builds
// the 144-bit stream, then scatters it byte-for-byte into the gaps.
struct GyroBitWriter {
    uint8_t buf[18] = {}; // 144 bits, LSB-first contiguous
    int pos = 0;
    void put(uint32_t v, int width) {
        for (int k = 0; k < width; k++) {
            if ((v >> k) & 1u) buf[pos >> 3] |= (uint8_t)(1u << (pos & 7));
            pos++;
        }
    }
    void emit(uint8_t* region) const {
        for (int g = 0; g < 3; g++) memcpy(region + 6 + 12 * g, buf + 6 * g, 6);
    }
};

// Fixed-point unit-scales: a quaternion component of 1.0 maps to 2^N.
constexpr double S20 = 1048576.0; // mode 2 absolute (21-bit field)
constexpr double S15 = 32768.0;   // mode 1 absolute (16-bit field)
constexpr double S12 = 4096.0;    // mode 0 absolute (13-bit field)

// Representable delta ceilings, in component units, that decide the mode.
// Past these the field clamps and the reconstruction degrades (see the
// crossover table in tools/sim_quat_packing.py).
constexpr double M2_DLF = 4095.0 / S20; // mode 2 first->last delta (13-bit)
constexpr double M2_DMA = 63.0 / S20;   // mode 2 mid-vs-chord delta (7-bit)
constexpr double M1_DMA = 508.0 / S15;  // mode 1 mid-vs-chord delta (8-bit, x4)

inline int drop_index(const double q[4]) {
    int mi = 0;
    for (int i = 1; i < 4; i++)
        if (fabs(q[i]) > fabs(q[mi])) mi = i;
    return mi;
}

inline int32_t clampq(double x, int32_t lo, int32_t hi) {
    long v = lrint(x);
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return (int32_t)v;
}

// Choose the packing mode adaptively and write the 36-byte motion region.
// All three modes drop each sample's largest component (rebuildable from
// ||q||=1) and canonicalise it positive; the difference is how the triplet is
// represented (see pro.h / tools/sim_quat_packing.py for the precision/range
// trade-offs). Inverse of the decoders in tools/decode_mode2.py.
} // namespace

void pack_quat_motion(uint8_t region[36], const double qf[4], const double qm[4],
                      const double ql[4], uint32_t ts_ms) {
    // mode 1/2 share one dropped index, taken from the mid sample and sign-
    // canonicalised so its dropped component is positive (the decoder rebuilds
    // it as +sqrt(1-||rest||^2)). first/last inherit the same flip; that holds
    // only while they agree with mid on the dominant axis.
    int mi = drop_index(qm);
    double sign = qm[mi] < 0 ? -1.0 : 1.0;
    double cf[3], cm[3], cl[3];
    for (int i = 0; i < 3; i++) {
        cf[i] = qf[(mi + i + 1) & 3] * sign;
        cm[i] = qm[(mi + i + 1) & 3] * sign;
        cl[i] = ql[(mi + i + 1) & 3] * sign;
    }
    bool shared = (drop_index(qf) == mi) && (drop_index(ql) == mi);

    double dlf_max = 0.0, dma_max = 0.0;
    for (int i = 0; i < 3; i++) {
        double avg = 0.5 * (cf[i] + cl[i]);
        dlf_max = fmax(dlf_max, fabs(cl[i] - cf[i]));
        dma_max = fmax(dma_max, fabs(cm[i] - avg));
    }

    GyroBitWriter w;
    if (shared && dlf_max <= M2_DLF && dma_max <= M2_DMA) {
        // mode 2: last absolute @21-bit, first via 13-bit delta, mid via 7-bit
        // delta-from-chord. Best absolute precision; slow/steady motion only.
        w.put(2, 2);
        w.put((uint32_t)mi, 2);
        for (int i = 0; i < 3; i++)
            w.put((uint32_t)clampq(cl[i] * S20, -(1 << 20), (1 << 20) - 1), 21);
        for (int i = 0; i < 3; i++)
            w.put((uint32_t)clampq((cl[i] - cf[i]) * S20, -(1 << 12), (1 << 12) - 1), 13);
        for (int i = 0; i < 3; i++) {
            double avg = 0.5 * (cf[i] + cl[i]);
            w.put((uint32_t)clampq((cm[i] - avg) * S20, -(1 << 6), (1 << 6) - 1), 7);
        }
        w.put(ts_ms & 0x7FF, 11);
        w.put(3, 6);
    } else if (shared && dma_max <= M1_DMA) {
        // mode 1: first & last absolute @16-bit, mid via 8-bit delta-from-chord
        // (x4 when div4 set). Endpoints unbounded -> fast but smooth motion.
        double dev[3], devmax = 0.0;
        for (int i = 0; i < 3; i++) {
            double avg = 0.5 * (cf[i] + cl[i]);
            dev[i] = (cm[i] - avg) * S15;
            devmax = fmax(devmax, fabs(dev[i]));
        }
        int div4 = devmax > 127.0 ? 1 : 0;
        double dscale = div4 ? 4.0 : 1.0;
        w.put(1, 2);
        w.put((uint32_t)div4, 1);
        w.put((uint32_t)mi, 2);
        for (int i = 0; i < 3; i++)
            w.put((uint32_t)clampq(cf[i] * S15, -(1 << 15), (1 << 15) - 1), 16);
        for (int i = 0; i < 3; i++)
            w.put((uint32_t)clampq(cl[i] * S15, -(1 << 15), (1 << 15) - 1), 16);
        for (int i = 0; i < 3; i++)
            w.put((uint32_t)clampq(dev[i] / dscale, -(1 << 7), (1 << 7) - 1), 8);
        w.put(ts_ms & 0x7FF, 11);
        w.put(3, 6);
    } else {
        // mode 0: three independent 13-bit quaternions, each its own dropped
        // index. Lowest precision but survives erratic motion (reversals/shake)
        // where the shared-index/chord assumptions of modes 1/2 break.
        w.put(0, 2);
        const double* qq[3] = {qf, qm, ql};
        for (int j = 0; j < 3; j++) {
            int mj = drop_index(qq[j]);
            double sgn = qq[j][mj] < 0 ? -1.0 : 1.0;
            w.put((uint32_t)mj, 2);
            for (int i = 0; i < 3; i++) {
                double c = qq[j][(mj + i + 1) & 3] * sgn;
                w.put((uint32_t)clampq(c * S12, -(1 << 12), (1 << 12) - 1), 13);
            }
        }
        w.put(ts_ms & 0x7FF, 11);
        w.put(3, 6);
    }
    w.emit(region);
}

void Controller::build_report30(uint8_t out[REPORT_LEN], uint64_t now_us) {
    memset(out, 0, REPORT_LEN);
    out[0] = 0x30;
    clock_us_ = now_us; // anchor the timer byte to the report's sample time
    fill_input_prefix(out);
    if (imu_enabled_) {
        xl::ImuSample s[3];
        imu_.sample3(now_us, s);
        if (imu_quaternion_) {
            // Quaternion mode: integrate the 3 sub-samples (5 ms grid) into the
            // running orientation, capturing it at each step as the first/mid/
            // last samples. accel stays plain int16 in its 3 slots; the gyro
            // bytes carry the adaptively-packed quaternion triplet.
            constexpr double DT = (double)REPORT_PERIOD_US / 3.0 / 1e6;
            double q3[3][4];
            for (int i = 0; i < 3; i++) {
                integrate_orientation(orient_, s[i], DT);
                memcpy(q3[i], orient_, sizeof(orient_));
                uint8_t* a = out + 13 + 12 * i;
                wr_s16(a + 0, s[i].ax);
                wr_s16(a + 2, s[i].ay);
                wr_s16(a + 4, s[i].az);
            }
            pack_quat_motion(out + 13, q3[0], q3[1], q3[2],
                             (uint32_t)(now_us / 1000));
        } else {
            for (int i = 0; i < 3; i++) {
                uint8_t* p = out + 13 + 12 * i;
                const int16_t v[6] = {s[i].ax, s[i].ay, s[i].az,
                                      s[i].gx, s[i].gy, s[i].gz};
                for (int j = 0; j < 6; j++) {
                    p[2 * j] = (uint8_t)(v[j] & 0xFF);
                    p[2 * j + 1] = (uint8_t)((uint16_t)v[j] >> 8);
                }
            }
        }
    }
}

bool Controller::tick(uint64_t now_us, uint8_t out[REPORT_LEN]) {
    if (!streaming()) return false;
    if (next_report_us_ == 0) next_report_us_ = now_us;
    if (now_us < next_report_us_) return false;
    uint64_t report_us = next_report_us_; // scheduled slot, not jittery loop time
    next_report_us_ += REPORT_PERIOD_US;
    if (next_report_us_ < now_us) { // fell behind; resync
        next_report_us_ = now_us + REPORT_PERIOD_US;
        report_us = now_us;
    }
    // Anchor IMU sampling to the steady 15 ms cadence, NOT the actual loop-fire
    // time. The Switch assumes the 3 samples are an even 5 ms grid contiguous
    // across reports; feeding loop jitter/stalls (USB host + the 0x10 rumble
    // firehose on the Switch) into sample3 makes motion reconstruct as shake,
    // while at rest (gyro~0) it's invisible. report_us keeps samples on a clean
    // grid regardless of when loop() actually serviced us.
    build_report30(out, report_us);
    return true;
}

} // namespace pro
