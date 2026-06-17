// Switch HD-rumble decode (inverse of hid-nintendo's joycon_encode_rumble,
// tables from dekuNukem's rumble_data_table.md) and the two-band mapping
// onto the SC haptics (PROTOCOL.md):
//
//   low band  -> grip rumble 0x80 (MsgHapticRumble): per-side speed_u16;
//                the grips are resonance-bound (~65 Hz), so the low band
//                carries body weight only, amplitude via speed.
//   high band -> trackpad tone 0x83 (MsgHapticLfoTone): per-side tone at
//                the true HD frequency, amplitude as gain_db. Verified live:
//                pads are wideband (100..1600+ Hz), 40 ms re-sends glide
//                smoothly, tones coexist with 0x80.
//
// Real Switch games send independent band amplitudes (the kernel encoder
// writes them equal), so both bands are decoded separately.
#pragma once
#include <cstddef>
#include <cstdint>

namespace rumble {

constexpr uint16_t MAX_AMP = 1003; // joycon_max_rumble_amp

struct Decoded {
    uint16_t hf_hz = 0;   // 0 = none/below table
    uint16_t lf_hz = 0;
    uint16_t hf_amp = 0;  // 0..1003, high-band amplitude
    uint16_t lf_amp = 0;  // 0..1003, low-band amplitude
};

// d = 4 bytes of one side's rumble data (left = data[0..3], right = data[4..7])
Decoded decode_side(const uint8_t d[4]);

// Reference encoder (same tables, equal band amps like the kernel) -- used
// by tests to verify the inversion.
void encode_side(uint8_t d[4], uint16_t hf_hz, uint16_t lf_hz, uint16_t amp);

// amp 0..1003 -> 0x83 gain_db (~20*log10(amp/MAX_AMP)), clamped to -40;
// -128 ("off") for amp 0.
int8_t amp_to_db(uint16_t amp);

// Grip-rumble strength calibration (the 0x80 grip path -- the only one we
// actually forward; pad tones are computed but unused, see main.cpp).
//
// The complaint "SC rumble is much stronger and longer than the Switch" comes
// from how the old to_sc() drove 0x80: it scaled the Switch amplitude into
// `left_speed`/`right_speed` and left `gain` at 0 dB. But on this hardware
// (PROTOCOL.md, measured): `speed`'s steady RMS is ~constant -- its high byte
// only sets a *throb rate* (~0.30*hi + 3.8 Hz) -- while `gain` (dB) is the real
// loudness knob. So the old path pinned full-scale speed (~80 Hz throb, max
// buzz) at a fixed 0 dB and threw the amplitude envelope into a parameter that
// barely changes felt strength: always loud, no decay (= "too strong, too long").
//
// Now we drive a fixed gentle throb and map the Switch low-band amplitude into
// `gain` (already a dB/log axis, like amp_to_db), so the envelope comes through
// and PEAK_GRIP_GAIN_DB caps peak loudness. Calibrate the cap against a real
// Pro Controller with tools/40_rumble_strength.py.
constexpr int8_t PEAK_GRIP_GAIN_DB = -10; // gain at full Switch amplitude (TUNE)
constexpr int8_t MIN_GRIP_GAIN_DB = -40;  // envelope floor (matches amp_to_db)
constexpr uint16_t GRIP_SPEED = 0x1000;   // motor-on drive; hi byte ~= 8.6 Hz throb

// Switch low-band amp (1..1003) -> SC grip gain (dB): PEAK at full amp, falling
// with the amplitude envelope down to MIN. Undefined for amp 0 (caller stops).
int8_t grip_gain(uint16_t lf_amp);

constexpr size_t SC_PACKET_LEN = 10; // incl. report id 0x80
constexpr size_t SC_TONE_LEN = 10;   // incl. report id 0x83

// 0x83 tone duration; outlives the 40 ms resend cadence, dies fast on stop.
constexpr uint16_t TONE_DURATION_MS = 100;

struct ScPacket {
    uint8_t bytes[SC_PACKET_LEN] = {0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool active = false; // false = all-zero stop packet
};

struct PadTonePacket {
    uint8_t bytes[SC_TONE_LEN] = {0x83, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool active = false; // false = don't send; tone expires by duration
};

// Grip packet from the low bands of both sides.
ScPacket to_sc(const Decoded& left, const Decoded& right);

// Pad tone from one side's high band; side: 0 = left pad, 1 = right pad.
PadTonePacket to_pad_tone(uint8_t side, const Decoded& d);

// Decodes a full 8-byte Switch rumble payload and tracks the latest state.
class State {
public:
    // Returns true if any resulting SC packet changed.
    bool update(const uint8_t data[8]);
    const ScPacket& packet() const { return pkt_; }
    const PadTonePacket& pad(int side) const { return pads_[side & 1]; }
    bool active() const {
        return pkt_.active || pads_[0].active || pads_[1].active;
    }
    const Decoded& left() const { return l_; }
    const Decoded& right() const { return r_; }

private:
    Decoded l_, r_;
    ScPacket pkt_;
    PadTonePacket pads_[2];
};

} // namespace rumble
