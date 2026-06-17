#include "translate.h"

namespace xl {

uint32_t map_buttons(const sc::State& s) {
    const uint32_t b = s.buttons;
    uint32_t out = 0;
    // Positional, not by legend: SC has the Xbox layout (A south, B east,
    // X west, Y north), Switch the mirrored one (B south, A east, Y west,
    // X north).
    if (b & sc::BTN_A) out |= SW_B; // south
    if (b & sc::BTN_B) out |= SW_A; // east
    if (b & sc::BTN_X) out |= SW_Y; // west
    if (b & sc::BTN_Y) out |= SW_X; // north
    if (b & sc::BTN_L1) out |= SW_L;
    if (b & sc::BTN_R1) out |= SW_R;
    if (b & sc::BTN_MENU) out |= SW_PLUS;
    if (b & sc::BTN_VIEW) out |= SW_MINUS;
    if (b & sc::BTN_STEAM) out |= SW_HOME;
    if (b & sc::BTN_QAM) out |= SW_CAP;
    if (b & sc::BTN_L3) out |= SW_LSTICK;
    if (b & sc::BTN_R3) out |= SW_RSTICK;
    if (b & sc::BTN_DPAD_UP) out |= SW_UP;
    if (b & sc::BTN_DPAD_DOWN) out |= SW_DOWN;
    if (b & sc::BTN_DPAD_LEFT) out |= SW_LEFT;
    if (b & sc::BTN_DPAD_RIGHT) out |= SW_RIGHT;
    // Grips/touch/sense have no Pro Controller equivalent; intentionally 0.
    if ((b & sc::BTN_L2) || s.ltrig >= TRIGGER_THRESHOLD) out |= SW_ZL;
    if ((b & sc::BTN_R2) || s.rtrig >= TRIGGER_THRESHOLD) out |= SW_ZR;
    return out;
}

uint16_t map_stick_axis(int16_t v) {
    int32_t x = v;
    if (x < -32767) x = -32767;
    return (uint16_t)(STICK_CENTER + x * STICK_RANGE / 32767);
}

// Axis conventions (PROTOCOL.md vs hid-nintendo):
//   SC:     X right, Y forward (toward triggers), Z up
//   Switch: X toward triggers, Y left, Z up
// so sw.x = sc.y, sw.y = -sc.x, sw.z = sc.z; gyro follows the right-hand
// rule about the same axes, so it remaps identically.
// Gyro output scale 14247/16384 (SC 16.384 LSB/dps -> Switch 14.247 LSB/dps).
static inline int16_t gyro_scale(int16_t v) {
    return (int16_t)((int32_t)v * 14247 / 16384);
}

ImuSample map_imu(const sc::State& s) {
    ImuSample o;
    o.ax = (int16_t)(s.accel[1] / 4);
    o.ay = (int16_t)(-(s.accel[0] / 4));
    o.az = (int16_t)(s.accel[2] / 4);
    o.gx = gyro_scale(s.gyro[1]);
    o.gy = (int16_t)-gyro_scale(s.gyro[0]);
    o.gz = gyro_scale(s.gyro[2]);
    return o;
}

SwInput map_input(const sc::State& s) {
    SwInput o;
    o.buttons = map_buttons(s);
    o.lx = map_stick_axis(s.lstick[0]);
    o.ly = map_stick_axis(s.lstick[1]);
    o.rx = map_stick_axis(s.rstick[0]);
    o.ry = map_stick_axis(s.rstick[1]);
    return o;
}

uint8_t battery_level(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint8_t level = (uint8_t)((percent + 12) / 25); // 100->4, 75->3, ... <=12->0
    return level > 4 ? 4 : level;
}

void ImuResampler::push(const ImuSample& s, uint64_t t_us) {
    buf_[head_] = s;
    t_[head_] = t_us;
    head_ = (head_ + 1) % N;
    if (count_ < N) count_++;
}

ImuSample ImuResampler::interp_at(uint64_t t) const {
    // Bracket t: a = latest sample at/before t, b = earliest sample after t.
    int ai = -1, bi = -1;
    uint64_t at = 0, bt = ~0ull;
    for (int i = 0; i < count_; i++) {
        int idx = (head_ - 1 - i + 2 * N) % N;
        uint64_t ti = t_[idx];
        if (ti <= t) {
            if (ai < 0 || ti > at) { ai = idx; at = ti; }
        } else {
            if (bi < 0 || ti < bt) { bi = idx; bt = ti; }
        }
    }
    if (ai < 0) return bi >= 0 ? buf_[bi] : ImuSample{}; // all newer: oldest
    if (bi < 0 || bt == at) return buf_[ai];             // all older / no span
    const int32_t span = (int32_t)(bt - at);
    const int32_t num = (int32_t)(t - at);
    const ImuSample& A = buf_[ai];
    const ImuSample& B = buf_[bi];
    auto lerp = [&](int16_t a, int16_t b) -> int16_t {
        return (int16_t)(a + (int32_t)(b - a) * num / span); // |.|<6.6e8, fits i32
    };
    ImuSample o;
    o.ax = lerp(A.ax, B.ax); o.ay = lerp(A.ay, B.ay); o.az = lerp(A.az, B.az);
    o.gx = lerp(A.gx, B.gx); o.gy = lerp(A.gy, B.gy); o.gz = lerp(A.gz, B.gz);
    return o;
}

void ImuResampler::sample3(uint64_t now_us, ImuSample out[3]) const {
    if (count_ == 0) {
        // No Puck sample yet (the Switch enables IMU and starts polling 0x30
        // before the Puck has streamed anything). Emit a plausible "flat at
        // rest" frame -- gravity down on +Z (4096 LSB/g), no rotation --
        // rather than all-zero, which reads as freefall and seeds the Switch's
        // orientation fusion with an impossible attitude on connect.
        ImuSample rest;
        rest.az = 4096;
        out[0] = out[1] = out[2] = rest;
        return;
    }
    for (int i = 0; i < 3; i++)
        out[i] = interp_at(now_us - (uint64_t)(2 - i) * 5000);
}

} // namespace xl
