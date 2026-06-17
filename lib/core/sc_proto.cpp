#include "sc_proto.h"

namespace sc {

static inline uint16_t rd_u16(const uint8_t* p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}
static inline int16_t rd_s16(const uint8_t* p) {
    return (int16_t)rd_u16(p);
}

bool parse_45(const uint8_t* d, size_t len, State& out) {
    if (len < 30 || d[0] != 0x45)
        return false;
    out = State{};
    out.seq = d[1];
    out.buttons = (uint32_t)d[2] | ((uint32_t)d[3] << 8) |
                  ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 24);
    out.ltrig = rd_u16(d + 6);
    out.rtrig = rd_u16(d + 8);
    out.lstick[0] = rd_s16(d + 10);
    out.lstick[1] = rd_s16(d + 12);
    out.rstick[0] = rd_s16(d + 14);
    out.rstick[1] = rd_s16(d + 16);
    out.lpad[0] = rd_s16(d + 18);
    out.lpad[1] = rd_s16(d + 20);
    out.lpad_pressure = rd_u16(d + 22);
    out.rpad[0] = rd_s16(d + 24);
    out.rpad[1] = rd_s16(d + 26);
    out.rpad_pressure = rd_u16(d + 28);
    if (len >= 46) {
        out.imu_ts = (uint32_t)rd_u16(d + 30) | ((uint32_t)rd_u16(d + 32) << 16);
        for (int i = 0; i < 3; i++) {
            out.accel[i] = rd_s16(d + 34 + 2 * i);
            out.gyro[i]  = rd_s16(d + 40 + 2 * i);
        }
    }
    return true;
}

bool parse_43(const uint8_t* d, size_t len, Battery& out) {
    if (len < 9 || d[0] != 0x43)
        return false;
    out = Battery{};
    out.state = d[1];
    out.percent = d[2];
    out.battery_mv = rd_u16(d + 3);
    out.system_mv = rd_u16(d + 5);
    out.input_mv = rd_u16(d + 7);
    return true;
}

const char* button_name(uint32_t m) {
    switch (m) {
    case BTN_A: return "A";
    case BTN_B: return "B";
    case BTN_X: return "X";
    case BTN_Y: return "Y";
    case BTN_QAM: return "QAM";
    case BTN_R3: return "R3";
    case BTN_MENU: return "Menu";
    case BTN_GRIP_R4: return "grip-R4";
    case BTN_GRIP_R5: return "grip-R5";
    case BTN_R1: return "R1";
    case BTN_DPAD_DOWN: return "Dpad-Down";
    case BTN_DPAD_RIGHT: return "Dpad-Right";
    case BTN_DPAD_LEFT: return "Dpad-Left";
    case BTN_DPAD_UP: return "Dpad-Up";
    case BTN_VIEW: return "View";
    case BTN_L3: return "L3";
    case BTN_STEAM: return "Steam";
    case BTN_GRIP_L4: return "grip-L4";
    case BTN_GRIP_L5: return "grip-L5";
    case BTN_L1: return "L1";
    case BTN_RSTICK_TOUCH: return "RStick-touch";
    case BTN_RPAD_TOUCH: return "RPad-touch";
    case BTN_RPAD_CLICK: return "RPad-click";
    case BTN_R2: return "R2";
    case BTN_LSTICK_TOUCH: return "LStick-touch";
    case BTN_LPAD_TOUCH: return "LPad-touch";
    case BTN_LPAD_CLICK: return "LPad-click";
    case BTN_L2: return "L2";
    case BTN_RGRIP_SENSE: return "RGrip-sense";
    case BTN_LGRIP_SENSE: return "LGrip-sense";
    }
    return nullptr;
}

} // namespace sc
