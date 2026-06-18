// Rig 1, part 3: Pro Controller protocol state machine, SPI image,
// 0x30 report golden bytes, HD-rumble decode (table inversion).
#include <unity.h>

#include <cmath>
#include <cstring>

#include "pro.h"
#include "rumble.h"
#include "translate.h"

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

static pro::Controller* C;
static uint64_t NOW;

static void fresh(void) {
    delete C;
    C = new pro::Controller();
    NOW = 1000000;
}

static void send_usb(uint8_t cmd) {
    const uint8_t buf[2] = {0x80, cmd};
    C->handle_output(buf, sizeof(buf), NOW);
}

static void send_subcmd(uint8_t sub, const uint8_t* arg, size_t argn) {
    uint8_t buf[64] = {};
    buf[0] = 0x01;
    buf[1] = 0x05; // packet counter (don't care)
    // neutral rumble
    static const uint8_t NEUTRAL[8] = {0x00, 0x01, 0x40, 0x40,
                                       0x00, 0x01, 0x40, 0x40};
    memcpy(buf + 2, NEUTRAL, 8);
    buf[10] = sub;
    if (arg && argn) memcpy(buf + 11, arg, argn);
    C->handle_output(buf, 11 + argn, NOW);
}

static bool pop(uint8_t out[pro::REPORT_LEN]) {
    size_t n = 0;
    return C->pop_reply(out, &n);
}

// hid-nintendo's full USB probe sequence in its exact order; wrong answers
// make the real driver give up loudly, so every reply shape is asserted.
static void test_probe_sequence(void) {
    fresh();
    uint8_t r[pro::REPORT_LEN];

    send_usb(0x02); // handshake
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_EQUAL_HEX8(0x81, r[0]);
    TEST_ASSERT_EQUAL_HEX8(0x02, r[1]);

    send_usb(0x03); // baudrate
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_EQUAL_HEX8(0x81, r[0]);
    TEST_ASSERT_EQUAL_HEX8(0x03, r[1]);

    send_usb(0x02); // handshake again
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_EQUAL_HEX8(0x02, r[1]);

    send_usb(0x04); // no-timeout: no reply by design
    TEST_ASSERT_FALSE(pop(r));

    // subcommand 0x02 device info
    send_subcmd(0x02, nullptr, 0);
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_EQUAL_HEX8(0x21, r[0]);
    TEST_ASSERT_TRUE(r[13] & 0x80);           // ACK
    TEST_ASSERT_EQUAL_HEX8(0x02, r[14]);      // echoed subcmd id
    TEST_ASSERT_EQUAL_HEX8(0x03, r[15 + 2]);  // type = Pro Controller
    TEST_ASSERT_EQUAL_HEX8(0x0C, r[12]);      // vibrator_report nonzero

    // user stick cal magics absent -> 0xFF
    const uint8_t magic_l[5] = {0x10, 0x80, 0x00, 0x00, 0x02};
    send_subcmd(0x10, magic_l, 5);
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_EQUAL_HEX8(0x90, r[13]);
    TEST_ASSERT_EQUAL_HEX8(0x10, r[14]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, r[15 + 5]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, r[15 + 6]);

    const uint8_t magic_r[5] = {0x1b, 0x80, 0x00, 0x00, 0x02};
    send_subcmd(0x10, magic_r, 5);
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_EQUAL_HEX8(0xFF, r[15 + 5]);

    // left stick factory cal @0x603d: [throw][center][throw] 12-bit pairs
    const uint8_t cal_l[5] = {0x3d, 0x60, 0x00, 0x00, 0x09};
    send_subcmd(0x10, cal_l, 5);
    TEST_ASSERT_TRUE(pop(r));
    const uint8_t want_l[9] = {0x00, 0x07, 0x70, 0x00, 0x08, 0x80,
                               0x00, 0x07, 0x70};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(want_l, r + 15 + 5, 9);

    // right stick factory cal @0x6046: [center][throw][throw]
    const uint8_t cal_r[5] = {0x46, 0x60, 0x00, 0x00, 0x09};
    send_subcmd(0x10, cal_r, 5);
    TEST_ASSERT_TRUE(pop(r));
    const uint8_t want_r[9] = {0x00, 0x08, 0x80, 0x00, 0x07, 0x70,
                               0x00, 0x07, 0x70};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(want_r, r + 15 + 5, 9);

    // IMU user cal magic @0x8026 absent; IMU factory cal @0x6020 served with the
    // acc offset captured from the Switch's in-console 6-axis calibration.
    const uint8_t magic_imu[5] = {0x26, 0x80, 0x00, 0x00, 0x02};
    send_subcmd(0x10, magic_imu, 5);
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_EQUAL_HEX8(0xFF, r[15 + 5]);

    const uint8_t cal_imu[5] = {0x20, 0x60, 0x00, 0x00, 0x18};
    send_subcmd(0x10, cal_imu, 5);
    TEST_ASSERT_TRUE(pop(r));
    const uint8_t want_imu[24] = {
        0x8b, 0xff, 0x7b, 0xff, 0x14, 0x01, // acc offset (in-console cal)
        0x00, 0x40, 0x00, 0x40, 0x00, 0x40, // acc scale 16384
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // gyr offset 0 (our gyro is bias-free)
        0x3b, 0x34, 0x3b, 0x34, 0x3b, 0x34, // gyr scale 13371
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(want_imu, r + 15 + 5, 24);

    // IMU enable, report mode 0x30, vibration enable, player lights
    const uint8_t on[1] = {0x01};
    send_subcmd(0x40, on, 1);
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_TRUE(C->imu_enabled());

    const uint8_t mode[1] = {0x30};
    send_subcmd(0x03, mode, 1);
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_EQUAL_HEX8(0x03, r[14]);
    TEST_ASSERT_TRUE(C->streaming());

    send_subcmd(0x48, on, 1);
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_TRUE(C->vibration_enabled());

    const uint8_t lights[1] = {0x01};
    send_subcmd(0x30, lights, 1);
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_EQUAL_HEX8(0x01, C->player_lights());

    // home LED (0x38) and IMU sensitivity (0x41) must at least ACK
    send_subcmd(0x38, nullptr, 0);
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_EQUAL_HEX8(0x38, r[14]);
    send_subcmd(0x41, nullptr, 0);
    TEST_ASSERT_TRUE(pop(r));

    // regulated voltage 0x50: 1600 units = 4.00 V
    send_subcmd(0x50, nullptr, 0);
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_EQUAL_HEX8(0xD0, r[13]);
    TEST_ASSERT_EQUAL_UINT16(1600, r[15] | (r[16] << 8));
}

// Subcommand 0x01 (Bluetooth manual pairing) — only the real Switch sends it;
// stub-acking it crashes the console. Step 1 must ACK 0x81 and return our
// BD_ADDR. (Exact step-2/3 payloads are bring-up-on-hardware; this pins step 1
// and the reply shape so it never silently regresses to an empty ack.)
static void test_manual_pairing_subcmd01(void) {
    fresh();
    uint8_t r[pro::REPORT_LEN];

    // Host step 1: its own BD_ADDR + identity (as captured from a real Switch).
    const uint8_t step1[] = {0x01, 0x71, 0xd0, 0x31, 0x55, 0xe2, 0x98,
                             0x3c, 0x04, 0x08, 0x4e, 0x69, 0x6e};
    send_subcmd(0x01, step1, sizeof(step1));
    TEST_ASSERT_TRUE(pop(r));
    TEST_ASSERT_EQUAL_HEX8(0x21, r[0]);
    TEST_ASSERT_EQUAL_HEX8(0x81, r[13]);  // ACK 0x81 (not the 0x80 empty stub)
    TEST_ASSERT_EQUAL_HEX8(0x01, r[14]);  // echoed subcmd id
    TEST_ASSERT_EQUAL_HEX8(0x03, r[15]);  // address-data marker
    const uint8_t mac[6] = {0x7C, 0xBB, 0x8A, 0x12, 0x34, 0x56};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(mac, r + 16, 6);
}

// golden bytes, hand-checked: A + ZL + Dpad-Up, lstick centered, rstick at
// cal max X / cal min Y, battery full discharging, IMU off, fresh timer.
static void test_report30_golden(void) {
    fresh();
    xl::SwInput in;
    in.buttons = xl::SW_A | xl::SW_ZL | xl::SW_UP;
    in.lx = 2048; in.ly = 2048;
    in.rx = 3840; in.ry = 256;
    C->set_input(in);
    C->set_battery(4, false);

    uint8_t r[pro::REPORT_LEN];
    C->build_report30(r, NOW);

    const uint8_t want[13] = {
        0x30,             // report id
        0xC8,             // timer = (NOW=1e6 us / 5000) & 0xFF = 200 (200 Hz clock)
        0x81,             // battery full(4)<<5 | not charging | conn(0x01)
        0x08, 0x80, 0x82, // buttons: A; byte4 = charging-grip bit; (ZL|UP)
        0x00, 0x08, 0x80, // lstick 2048/2048 12-bit packed
        0x00, 0x0F, 0x10, // rstick 3840/256
        0x0C,             // vibrator_report
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(want, r, 13);
    for (int i = 13; i < 49; i++) // IMU off -> zeroed sample area
        TEST_ASSERT_EQUAL_HEX8(0x00, r[i]);

    // timer advances with time: +15 ms / 5 ms = +3 ticks (real-pad 200 Hz clock)
    C->build_report30(r, NOW + 15000);
    TEST_ASSERT_EQUAL_HEX8(0xCB, r[1]); // 203

    // charging bit (0x10) follows the SC's reported state, not a hardcode.
    C->set_battery(4, true);
    C->build_report30(r, NOW);
    TEST_ASSERT_EQUAL_HEX8(0x91, r[2]); // full(4)<<5 | charging(0x10) | conn(0x01)
}

static void test_report30_imu_samples(void) {
    fresh();
    const uint8_t on[1] = {0x01};
    send_subcmd(0x40, on, 1);
    uint8_t r[pro::REPORT_LEN];
    while (pop(r)) {}

    xl::ImuSample s;
    s.ax = 100; s.ay = -200; s.az = 4096;
    s.gx = 10; s.gy = -20; s.gz = 30;
    C->push_imu(s, NOW - 1000);
    C->build_report30(r, NOW);

    // one sample available -> all 3 slots carry it; s16 LE per field
    const uint8_t want[12] = {0x64, 0x00, 0x38, 0xFF, 0x00, 0x10,
                              0x0A, 0x00, 0xEC, 0xFF, 0x1E, 0x00};
    for (int i = 0; i < 3; i++)
        TEST_ASSERT_EQUAL_HEX8_ARRAY(want, r + 13 + 12 * i, 12);
}

// ---- adaptive quaternion packing (modes 0/1/2) ----
//
// Independent decoder for the 36-byte motion region: reads the contiguous
// LSB-first 144-bit gyro stream back out of the three accel gaps and rebuilds
// the first/mid/last orientations, then we check the chosen mode + round-trip
// accuracy against the quaternions we packed. Mirrors tools/sim_quat_packing.py.

namespace {
struct GyroBitReader {
    uint8_t buf[18];
    int pos = 0;
    explicit GyroBitReader(const uint8_t* region) {
        for (int g = 0; g < 3; g++) memcpy(buf + 6 * g, region + 6 + 12 * g, 6);
    }
    uint32_t get(int width) {
        uint32_t v = 0;
        for (int k = 0; k < width; k++) {
            if ((buf[pos >> 3] >> (pos & 7)) & 1u) v |= (1u << k);
            pos++;
        }
        return v;
    }
};
int32_t sext(uint32_t v, int width) {
    return (v & (1u << (width - 1))) ? (int32_t)(v - (1u << width)) : (int32_t)v;
}
void rebuild(const double c[3], int mi, double q[4]) {
    for (int i = 0; i < 3; i++) q[(mi + i + 1) & 3] = c[i];
    double s = 1.0 - (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    q[mi] = sqrt(s < 0 ? 0 : s);
}
int decode_region(const uint8_t* region, double qf[4], double qm[4], double ql[4]) {
    GyroBitReader r(region);
    int mode = r.get(2);
    if (mode == 2) {
        int mi = r.get(2);
        double last[3], dlf[3], dma[3];
        for (int i = 0; i < 3; i++) last[i] = sext(r.get(21), 21) / 1048576.0;
        for (int i = 0; i < 3; i++) dlf[i] = sext(r.get(13), 13) / 1048576.0;
        for (int i = 0; i < 3; i++) dma[i] = sext(r.get(7), 7) / 1048576.0;
        double cf[3], cm[3];
        for (int i = 0; i < 3; i++) {
            cf[i] = last[i] - dlf[i];
            cm[i] = 0.5 * (cf[i] + last[i]) + dma[i];
        }
        rebuild(cf, mi, qf); rebuild(cm, mi, qm); rebuild(last, mi, ql);
    } else if (mode == 1) {
        int div4 = r.get(1), mi = r.get(2);
        double f[3], l[3], dma[3];
        for (int i = 0; i < 3; i++) f[i] = sext(r.get(16), 16) / 32768.0;
        for (int i = 0; i < 3; i++) l[i] = sext(r.get(16), 16) / 32768.0;
        for (int i = 0; i < 3; i++) dma[i] = sext(r.get(8), 8) * (div4 ? 4 : 1) / 32768.0;
        double cm[3];
        for (int i = 0; i < 3; i++) cm[i] = 0.5 * (f[i] + l[i]) + dma[i];
        rebuild(f, mi, qf); rebuild(cm, mi, qm); rebuild(l, mi, ql);
    } else {
        double* qs[3] = {qf, qm, ql};
        for (int j = 0; j < 3; j++) {
            int mj = r.get(2);
            double c[3];
            for (int i = 0; i < 3; i++) c[i] = sext(r.get(13), 13) / 4096.0;
            rebuild(c, mj, qs[j]);
        }
    }
    return mode;
}
double q_angle_deg(const double a[4], const double b[4]) {
    double na = sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]+a[3]*a[3]);
    double nb = sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]+b[3]*b[3]);
    double d = (a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3]) / (na * nb);
    if (d < 0) d = -d;
    if (d > 1) d = 1;
    return 2.0 * acos(d) * 180.0 / 3.14159265358979323846;
}
// one 5 ms rotation increment of `rate` deg/s about a fixed axis
void incr(double rate_dps, const double axis[3], double q[4]) {
    double ang = rate_dps * 0.005 * 3.14159265358979323846 / 180.0;
    double s = sin(ang / 2);
    q[0] = axis[0] * s; q[1] = axis[1] * s; q[2] = axis[2] * s; q[3] = cos(ang / 2);
}
void qmul(const double a[4], const double b[4], double o[4]) {
    double x = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
    double y = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0];
    double z = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3];
    double w = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
    o[0] = x; o[1] = y; o[2] = z; o[3] = w;
}
// build first/mid/last by integrating two 5 ms steps from identity
void triplet(double r1, double r2, const double axis[3],
             double qf[4], double qm[4], double ql[4]) {
    double i1[4], i2[4];
    incr(r1, axis, i1);
    incr(r2, axis, i2);
    qf[0] = qf[1] = qf[2] = 0; qf[3] = 1;   // identity
    qmul(qf, i1, qm);
    qmul(qm, i2, ql);
}
} // namespace

static void test_quat_pack_mode2_slow(void) {
    const double axis[3] = {0.3015, -0.9045, 0.3015}; // unit-ish, mixed
    double qf[4], qm[4], ql[4];
    triplet(20.0, 20.0, axis, qf, qm, ql); // 20 deg/s: well within mode-2 range
    uint8_t region[36] = {};
    pro::pack_quat_motion(region, qf, qm, ql, 1234);
    double df[4], dm[4], dl[4];
    int mode = decode_region(region, df, dm, dl);
    TEST_ASSERT_EQUAL_INT(2, mode);
    TEST_ASSERT_TRUE(q_angle_deg(qf, df) < 0.005);
    TEST_ASSERT_TRUE(q_angle_deg(qm, dm) < 0.005);
    TEST_ASSERT_TRUE(q_angle_deg(ql, dl) < 0.005);
}

static void test_quat_pack_mode1_fast(void) {
    const double axis[3] = {0.0, 1.0, 0.0}; // pure yaw
    double qf[4], qm[4], ql[4];
    triplet(250.0, 250.0, axis, qf, qm, ql); // 250 deg/s overruns mode-2 deltas
    uint8_t region[36] = {};
    pro::pack_quat_motion(region, qf, qm, ql, 0);
    double df[4], dm[4], dl[4];
    int mode = decode_region(region, df, dm, dl);
    TEST_ASSERT_EQUAL_INT(1, mode);
    TEST_ASSERT_TRUE(q_angle_deg(qf, df) < 0.02);
    TEST_ASSERT_TRUE(q_angle_deg(qm, dm) < 0.02);
    TEST_ASSERT_TRUE(q_angle_deg(ql, dl) < 0.02);
}

static void test_quat_pack_mode0_erratic(void) {
    const double axis[3] = {0.0, 1.0, 0.0};
    double qf[4], qm[4], ql[4];
    // full reversal at 1000 deg/s: mid is an extreme, not the chord midpoint ->
    // mode 1's delta (even x4) overflows, so the packer must fall back to mode 0.
    triplet(1000.0, -1000.0, axis, qf, qm, ql);
    uint8_t region[36] = {};
    pro::pack_quat_motion(region, qf, qm, ql, 0);
    double df[4], dm[4], dl[4];
    int mode = decode_region(region, df, dm, dl);
    TEST_ASSERT_EQUAL_INT(0, mode);
    TEST_ASSERT_TRUE(q_angle_deg(qf, df) < 0.1);
    TEST_ASSERT_TRUE(q_angle_deg(qm, dm) < 0.1);
    TEST_ASSERT_TRUE(q_angle_deg(ql, dl) < 0.1);
}

static void test_quat_pack_at_rest(void) {
    // identity triplet -> mode 2, all components zero (byte-compatible with the
    // previously hardware-validated at-rest output).
    double id[4] = {0, 0, 0, 1};
    uint8_t region[36] = {};
    pro::pack_quat_motion(region, id, id, id, 0);
    TEST_ASSERT_EQUAL_INT(2, region[6] & 0x3); // packing_mode field
    double df[4], dm[4], dl[4];
    decode_region(region, df, dm, dl);
    TEST_ASSERT_TRUE(q_angle_deg(id, dl) < 1e-6);
}

static void test_tick_pacing(void) {
    fresh();
    uint8_t r[pro::REPORT_LEN];
    // not streaming yet -> no reports
    TEST_ASSERT_FALSE(C->tick(NOW, r));
    const uint8_t mode[1] = {0x30};
    send_subcmd(0x03, mode, 1);
    while (pop(r)) {}
    TEST_ASSERT_TRUE(C->tick(NOW, r));
    TEST_ASSERT_EQUAL_HEX8(0x30, r[0]);
    TEST_ASSERT_FALSE(C->tick(NOW + 5000, r));   // not due yet
    TEST_ASSERT_TRUE(C->tick(NOW + 15000, r));   // 15 ms cadence
    TEST_ASSERT_FALSE(C->tick(NOW + 16000, r));
}

// ---- HD rumble decode: exact inversion of the encode tables ----

static void test_rumble_neutral(void) {
    const uint8_t neutral[4] = {0x00, 0x01, 0x40, 0x40};
    rumble::Decoded d = rumble::decode_side(neutral);
    TEST_ASSERT_EQUAL_UINT16(0, d.hf_amp);
    TEST_ASSERT_EQUAL_UINT16(0, d.lf_amp);
    TEST_ASSERT_EQUAL_UINT16(160, d.lf_hz);
    TEST_ASSERT_EQUAL_UINT16(320, d.hf_hz);
}

static void test_rumble_amp_inversion(void) {
    // every amplitude the encoder can emit must decode back exactly,
    // independently on both bands (the encoder writes them equal)
    static const uint16_t AMPS[] = {0,  10,  14,  33,  80,  112, 140, 198,
                                    251, 305, 362, 440, 524, 636, 757, 900,
                                    981, 1003};
    for (uint16_t a : AMPS) {
        uint8_t d[4];
        rumble::encode_side(d, 320, 160, a);
        rumble::Decoded out = rumble::decode_side(d);
        TEST_ASSERT_EQUAL_UINT16(a, out.hf_amp);
        TEST_ASSERT_EQUAL_UINT16(a, out.lf_amp);
        TEST_ASSERT_EQUAL_UINT16(160, out.lf_hz);
        TEST_ASSERT_EQUAL_UINT16(320, out.hf_hz);
    }
}

static void test_rumble_band_amps_independent(void) {
    // splice the HB nibble of a strong encode into a weak one: bands differ
    uint8_t strong[4], weak[4], mixed[4];
    rumble::encode_side(strong, 320, 160, 1003);
    rumble::encode_side(weak, 320, 160, 0);
    mixed[0] = strong[0]; // HB freq high
    mixed[1] = strong[1]; // HB amp + freq lsb from the strong side
    mixed[2] = weak[2];   // LB freq + amp lsb from the weak side
    mixed[3] = weak[3];   // LB amp
    rumble::Decoded out = rumble::decode_side(mixed);
    TEST_ASSERT_EQUAL_UINT16(1003, out.hf_amp);
    TEST_ASSERT_EQUAL_UINT16(0, out.lf_amp);
}

static void test_rumble_freq_inversion(void) {
    // low band is invertible across its table span (41..626 Hz)
    static const uint16_t LFS[] = {41, 50, 80, 102, 160, 261, 320, 626};
    for (uint16_t f : LFS) {
        uint8_t d[4];
        rumble::encode_side(d, 320, f, 100);
        uint16_t want = (f == 261) ? 263 : f; // 261 not in table -> next row
        TEST_ASSERT_EQUAL_UINT16(want, rumble::decode_side(d).lf_hz);
    }
    // high band likewise (82..1253 Hz)
    static const uint16_t HFS[] = {82, 160, 320, 640, 1253};
    for (uint16_t f : HFS) {
        uint8_t d[4];
        rumble::encode_side(d, f, 160, 100);
        TEST_ASSERT_EQUAL_UINT16(f, rumble::decode_side(d).hf_hz);
    }
}

static void test_rumble_to_sc(void) {
    rumble::Decoded off{};
    rumble::Decoded weak{};
    weak.lf_amp = 100; weak.lf_hz = 160;
    rumble::Decoded strong{};
    strong.lf_amp = 1003; strong.lf_hz = 160;

    // both off -> all-zero stop packet
    rumble::ScPacket p = rumble::to_sc(off, off);
    TEST_ASSERT_FALSE(p.active);
    for (size_t i = 1; i < rumble::SC_PACKET_LEN; i++)
        TEST_ASSERT_EQUAL_HEX8(0x00, p.bytes[i]);

    // weak left only: left_speed (bytes 4:6) on, right_speed (7:9) clear;
    // amplitude rides in left_gain (byte 6), right_gain (byte 9) stays 0.
    p = rumble::to_sc(weak, off);
    TEST_ASSERT_TRUE(p.active);
    TEST_ASSERT_EQUAL_UINT16(rumble::GRIP_SPEED,
                             (uint16_t)(p.bytes[4] | (p.bytes[5] << 8)));
    TEST_ASSERT_EQUAL_UINT16(0, (uint16_t)(p.bytes[7] | (p.bytes[8] << 8)));
    TEST_ASSERT_EQUAL_INT8(rumble::grip_gain(100), (int8_t)p.bytes[6]);
    TEST_ASSERT_EQUAL_HEX8(0x00, p.bytes[9]);

    // strong both: both speeds on, both gains at the peak cap (full amp -> PEAK)
    p = rumble::to_sc(strong, strong);
    TEST_ASSERT_EQUAL_UINT16(rumble::GRIP_SPEED,
                             (uint16_t)(p.bytes[4] | (p.bytes[5] << 8)));
    TEST_ASSERT_EQUAL_UINT16(rumble::GRIP_SPEED,
                             (uint16_t)(p.bytes[7] | (p.bytes[8] << 8)));
    TEST_ASSERT_EQUAL_INT8(rumble::PEAK_GRIP_GAIN_DB, (int8_t)p.bytes[6]);
    TEST_ASSERT_EQUAL_INT8(rumble::PEAK_GRIP_GAIN_DB, (int8_t)p.bytes[9]);

    // type + intensity fields (bytes 1:4) stay zero like SDL sends them
    TEST_ASSERT_EQUAL_HEX8(0x00, p.bytes[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, p.bytes[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, p.bytes[3]);

    // gain carries the envelope: never above PEAK, monotonic up with amp, and
    // a weaker amp is strictly quieter than full.
    TEST_ASSERT_LESS_THAN_INT8(rumble::PEAK_GRIP_GAIN_DB + 1, rumble::grip_gain(100));
    TEST_ASSERT_LESS_THAN_INT8(rumble::grip_gain(1003), rumble::grip_gain(100));
    int8_t last = -128;
    for (uint16_t a = 1; a <= 1003; a += 100) {
        rumble::Decoded x{};
        x.lf_amp = a; x.lf_hz = 160;
        p = rumble::to_sc(x, off);
        TEST_ASSERT_NOT_EQUAL(0, p.bytes[4] | (p.bytes[5] << 8)); // motor on
        int8_t g = (int8_t)p.bytes[6];
        TEST_ASSERT_GREATER_OR_EQUAL_INT8(last, g);
        last = g;
    }
}

static void test_rumble_pad_tone(void) {
    // amp_to_db: full = 0 dB, half ~ -6 dB, floor -40, off -128
    TEST_ASSERT_EQUAL_INT8(0, rumble::amp_to_db(1003));
    TEST_ASSERT_EQUAL_INT8(-6, rumble::amp_to_db(503));
    TEST_ASSERT_EQUAL_INT8(-20, rumble::amp_to_db(100));
    TEST_ASSERT_EQUAL_INT8(-40, rumble::amp_to_db(1));
    TEST_ASSERT_EQUAL_INT8(-128, rumble::amp_to_db(0));

    // high band off -> inactive tone
    rumble::Decoded d{};
    d.lf_amp = 1003; d.lf_hz = 160; // low band alone never makes a tone
    rumble::PadTonePacket t = rumble::to_pad_tone(0, d);
    TEST_ASSERT_FALSE(t.active);

    // high band on -> 0x83 [side, gain_db, freq, dur, lfo=0, depth=0]
    d.hf_amp = 503; d.hf_hz = 320;
    t = rumble::to_pad_tone(1, d);
    TEST_ASSERT_TRUE(t.active);
    TEST_ASSERT_EQUAL_HEX8(0x83, t.bytes[0]);
    TEST_ASSERT_EQUAL_HEX8(1, t.bytes[1]); // right pad
    TEST_ASSERT_EQUAL_INT8(-6, (int8_t)t.bytes[2]);
    TEST_ASSERT_EQUAL_UINT16(320, (uint16_t)(t.bytes[3] | (t.bytes[4] << 8)));
    TEST_ASSERT_EQUAL_UINT16(rumble::TONE_DURATION_MS,
                             (uint16_t)(t.bytes[5] | (t.bytes[6] << 8)));
    TEST_ASSERT_EQUAL_HEX8(0, t.bytes[7]);
    TEST_ASSERT_EQUAL_HEX8(0, t.bytes[8]);
    TEST_ASSERT_EQUAL_HEX8(0, t.bytes[9]);
}

static void test_rumble_state_via_output_reports(void) {
    fresh();
    // rumble-only output report 0x10 with a strong left effect
    uint8_t buf[10] = {0x10, 0x01};
    rumble::encode_side(buf + 2, 320, 160, 1003); // left
    rumble::encode_side(buf + 6, 320, 160, 0);    // right off
    C->handle_output(buf, sizeof(buf), NOW);
    TEST_ASSERT_TRUE(C->take_rumble_dirty());
    TEST_ASSERT_FALSE(C->take_rumble_dirty()); // edge-triggered
    TEST_ASSERT_TRUE(C->rumble_state().active());
    TEST_ASSERT_EQUAL_UINT16(1003, C->rumble_state().left().lf_amp);
    TEST_ASSERT_EQUAL_UINT16(1003, C->rumble_state().left().hf_amp);
    TEST_ASSERT_EQUAL_UINT16(0, C->rumble_state().right().lf_amp);
    // left high band (320 Hz @ full) -> left pad tone; right pad silent
    TEST_ASSERT_TRUE(C->rumble_state().pad(0).active);
    TEST_ASSERT_FALSE(C->rumble_state().pad(1).active);

    // same data again: no change, no dirty flag
    C->handle_output(buf, sizeof(buf), NOW + 1000);
    TEST_ASSERT_FALSE(C->take_rumble_dirty());

    // neutral via subcommand-carried rumble -> stop packet
    const uint8_t mode[1] = {0x30};
    send_subcmd(0x03, mode, 1);
    TEST_ASSERT_TRUE(C->take_rumble_dirty());
    TEST_ASSERT_FALSE(C->rumble_state().active());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_probe_sequence);
    RUN_TEST(test_manual_pairing_subcmd01);
    RUN_TEST(test_report30_golden);
    RUN_TEST(test_report30_imu_samples);
    RUN_TEST(test_quat_pack_mode2_slow);
    RUN_TEST(test_quat_pack_mode1_fast);
    RUN_TEST(test_quat_pack_mode0_erratic);
    RUN_TEST(test_quat_pack_at_rest);
    RUN_TEST(test_tick_pacing);
    RUN_TEST(test_rumble_neutral);
    RUN_TEST(test_rumble_amp_inversion);
    RUN_TEST(test_rumble_band_amps_independent);
    RUN_TEST(test_rumble_freq_inversion);
    RUN_TEST(test_rumble_to_sc);
    RUN_TEST(test_rumble_pad_tone);
    RUN_TEST(test_rumble_state_via_output_reports);
    return UNITY_END();
}
