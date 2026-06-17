// SC state -> Switch Pro Controller values: buttons, sticks (s16 -> 12-bit
// against the emulated SPI calibration), IMU (rescale + axis remap), battery.
#pragma once
#include <cstdint>
#include "sc_proto.h"

namespace xl {

// Switch button bits, laid out exactly as 0x30 report bytes 3..5 read LE
// (matches hid-nintendo's JC_BTN_* u32).
enum SwButton : uint32_t {
    SW_Y      = 1u << 0,
    SW_X      = 1u << 1,
    SW_B      = 1u << 2,
    SW_A      = 1u << 3,
    SW_R      = 1u << 6,
    SW_ZR     = 1u << 7,
    SW_MINUS  = 1u << 8,
    SW_PLUS   = 1u << 9,
    SW_RSTICK = 1u << 10,
    SW_LSTICK = 1u << 11,
    SW_HOME   = 1u << 12,
    SW_CAP    = 1u << 13,
    SW_DOWN   = 1u << 16,
    SW_UP     = 1u << 17,
    SW_RIGHT  = 1u << 18,
    SW_LEFT   = 1u << 19,
    SW_L      = 1u << 22,
    SW_ZL     = 1u << 23,
};

// Emulated factory stick calibration served from SPI (see pro_spi.cpp):
// center 2048, full throw +/-1792 -> raw range 256..3840 hit exactly at
// SC +/-32767 (firmware pre-calibrates SC sticks to full scale).
constexpr int32_t STICK_CENTER = 2048;
constexpr int32_t STICK_RANGE  = 1792;

// SC analog triggers (0..32767) assert ZL/ZR at half pull; the SC's own
// digital full-pull bits OR in on top.
constexpr uint16_t TRIGGER_THRESHOLD = 16384;

// Switch IMU units: accel 4096 LSB/g (SC 16384 -> /4), gyro 14.247 LSB/dps
// (SC 16.384 -> *14247/16384).
struct ImuSample {
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;
};

struct SwInput {
    uint32_t buttons = 0;
    uint16_t lx = STICK_CENTER, ly = STICK_CENTER; // 12-bit raw
    uint16_t rx = STICK_CENTER, ry = STICK_CENTER;
};

uint32_t map_buttons(const sc::State& s);
uint16_t map_stick_axis(int16_t v);
ImuSample map_imu(const sc::State& s);
SwInput map_input(const sc::State& s);

// 0x43 percent -> Switch 3-bit level (0 empty .. 4 full).
uint8_t battery_level(uint8_t percent);

// Resamples the ~268 Hz SC IMU stream to the 3 samples / 5 ms grid of a
// Switch 0x30 report. Timestamps come from report arrival, close enough at
// 4 ms cadence (the SC's imu_ts u32 us clock is the higher-fidelity option).
class ImuResampler {
public:
    void push(const ImuSample& s, uint64_t t_us);
    // Fills out[0..2] with interpolated samples at (now-10ms, now-5ms, now).
    void sample3(uint64_t now_us, ImuSample out[3]) const;
    bool empty() const { return count_ == 0; }

private:
    static constexpr int N = 32; // ~120 ms of history at 268 Hz
    ImuSample buf_[N];
    uint64_t t_[N] = {};
    int head_ = 0; // next write slot
    int count_ = 0;

    // Linear interpolation to time t (vs the old nearest-neighbor, which
    // duplicated samples / aliased the ~268 Hz SC stream onto the 200 Hz grid
    // and juddered in motion). Clamps at the ends (no extrapolation).
    ImuSample interp_at(uint64_t t) const;
};

} // namespace xl
