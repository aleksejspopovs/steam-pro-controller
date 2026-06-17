// Steam Controller (2026) wire protocol: input report parsers + connection
// lifecycle. Portable (no OS, no Arduino). Layouts per PROTOCOL.md.
#pragma once
#include <cstddef>
#include <cstdint>

namespace sc {

constexpr size_t REPORT45_LEN = 46; // report id + 45 data bytes
constexpr size_t REPORT43_LEN = 15;

// Button mask bit = (report_byte - 2) * 8 + bit, i.e. bytes 2..5 read LE.
enum Button : uint32_t {
    BTN_A           = 1u << 0,
    BTN_B           = 1u << 1,
    BTN_X           = 1u << 2,
    BTN_Y           = 1u << 3,
    BTN_QAM         = 1u << 4,
    BTN_R3          = 1u << 5,
    BTN_MENU        = 1u << 6,
    BTN_GRIP_R4     = 1u << 7,
    BTN_GRIP_R5     = 1u << 8,
    BTN_R1          = 1u << 9,
    BTN_DPAD_DOWN   = 1u << 10,
    BTN_DPAD_RIGHT  = 1u << 11,
    BTN_DPAD_LEFT   = 1u << 12,
    BTN_DPAD_UP     = 1u << 13,
    BTN_VIEW        = 1u << 14,
    BTN_L3          = 1u << 15,
    BTN_STEAM       = 1u << 16,
    BTN_GRIP_L4     = 1u << 17,
    BTN_GRIP_L5     = 1u << 18,
    BTN_L1          = 1u << 19,
    BTN_RSTICK_TOUCH = 1u << 20,
    BTN_RPAD_TOUCH  = 1u << 21,
    BTN_RPAD_CLICK  = 1u << 22,
    BTN_R2          = 1u << 23, // full-pull digital
    BTN_LSTICK_TOUCH = 1u << 24,
    BTN_LPAD_TOUCH  = 1u << 25,
    BTN_LPAD_CLICK  = 1u << 26,
    BTN_L2          = 1u << 27, // full-pull digital
    BTN_RGRIP_SENSE = 1u << 28,
    BTN_LGRIP_SENSE = 1u << 29,
};

struct State {
    uint8_t  seq = 0;
    uint32_t buttons = 0;
    uint16_t ltrig = 0, rtrig = 0;        // 0..32767
    int16_t  lstick[2] = {0, 0};          // x right+, y up+
    int16_t  rstick[2] = {0, 0};
    int16_t  lpad[2] = {0, 0};
    uint16_t lpad_pressure = 0;
    int16_t  rpad[2] = {0, 0};
    uint16_t rpad_pressure = 0;
    uint32_t imu_ts = 0;                  // bytes 30:33, us, frozen w/ IMU off
    int16_t  accel[3] = {0, 0, 0};        // ~16384 LSB/g
    int16_t  gyro[3]  = {0, 0, 0};        // ~16.384 LSB/dps
};

bool parse_45(const uint8_t* d, size_t len, State& out);

struct Battery {
    // EChargeState: 0 reset, 1 discharging, 2 charging, 3 src-validate
    // (transient), 4 charging done
    uint8_t  state = 0;
    uint8_t  percent = 0; // 0..100
    uint16_t battery_mv = 0;
    uint16_t system_mv = 0;
    uint16_t input_mv = 0; // USB input voltage; 0 on battery
    bool charging() const { return state == 2 || state == 3; }
    bool full() const { return state == 4; }
    // Disconnect flushes zero-payload 0x43 records (state 0): "no data",
    // never to be read as 0%.
    bool valid() const { return state != 0; }
};

// Returns false for unparseable records (wrong id/short) -- the all-zero
// flush records in lifecycle captures land here ("no data", never 0%).
bool parse_43(const uint8_t* d, size_t len, Battery& out);

const char* button_name(uint32_t single_bit_mask); // nullptr if unknown

// Connection lifecycle: feeds slot input reports, emits exactly one
// Connected per `79 02` (adapter must re-run gamepad-mode init on each).
enum class ConnEvent { None, Connected, Disconnected };

class ConnMonitor {
public:
    ConnEvent feed(uint8_t report_id, const uint8_t* payload, size_t len) {
        if (report_id != 0x79 || len < 1)
            return ConnEvent::None;
        if (payload[0] == 0x02) {
            connected_ = true;
            return ConnEvent::Connected;
        }
        if (payload[0] == 0x01) {
            connected_ = false;
            return ConnEvent::Disconnected;
        }
        return ConnEvent::None;
    }
    bool connected() const { return connected_; }
    void set_connected(bool c) { connected_ = c; } // from a 0xB4 startup query

private:
    bool connected_ = false;
};

} // namespace sc
