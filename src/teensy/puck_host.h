// Host side on the Teensy 4.1 USB host port (USB2 / USBHost_t36): claims the
// Steam Controller dongle (28de:1304) and runs the proven Puck protocol
// (src/linux/puck.cpp + src/pico/puck_host.cpp) so live input/IMU/battery flow
// to core/ and rumble flows back. Unlike the Pico this is SINGLE-CORE: this
// module and pro_device both run in one loop(); no cross-core bus.
//
// USBHost_t36 specifics vs the Pico's Adafruit_TinyUSB host:
//   - a HID interface is claimed via USBHIDInput::claim_collection (return
//     CLAIM_INTERFACE to take the whole interface and get RAW reports through
//     hid_process_in_data, bypassing usage parsing -- the RawHIDController
//     pattern). The Puck exposes several HID interfaces; we claim them all.
//   - feature reports = USBHIDParser::sendControlPacket (SET/GET_REPORT). It is
//     ASYNC (completion -> hid_process_control), so the command channel is a
//     pump-and-wait wrapper, not the Pico's blocking control xfer.
//   - rumble OUT = USBHIDParser::sendPacket on the interrupt OUT endpoint.
//
// Input completions fire in USB ISR context, so the published snapshot + IMU
// ring are guarded / lock-free for the loop()-side reader.
#pragma once

#include <cstddef>
#include <cstdint>

#include "translate.h"

namespace puck_host {

// Bring up USBHost_t36 on the Teensy host port. Call from setup().
void begin();

// Pump the host stack + connection probe + delayed/verified gamepad-mode init
// + rumble resend. Call every loop().
void task(uint32_t now_ms);

bool present();    // dongle enumerated on the host port
bool connected();  // a controller is bound to the dongle

// Latest translated input (buttons + 12-bit sticks) + battery, guarded copy.
struct Snapshot {
    xl::SwInput input;
    uint8_t bat_level = 4;
    bool bat_charging = false;
    bool present = false;
    bool connected = false;
};
Snapshot snapshot();

// Drain one IMU sample (host -> device resampler). Returns false when empty.
bool pop_imu(xl::ImuSample& s, uint64_t& t_us);

// Device -> host: latest HD-rumble as four 0x83 tones (L grip, R grip, L pad,
// R pad), each a 10-byte output report incl. id. Call on change; task() resends
// each active tone <=40 ms and lets silent ones expire by duration.
void set_tones(const uint8_t tones[4][10], const bool active[4]);

} // namespace puck_host
