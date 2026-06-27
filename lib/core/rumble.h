// Switch HD-rumble decode: the stateful AM/FM codec the Switch puts on the
// wire (port of reference/switch_rumble_decoder.*). Each 4-byte side packs
// 1..3 samples; each sample carries a low and a high band, given as absolute
// values or differential deltas over per-side state:
//   low band  centered 160 Hz (range ~40..640 Hz)
//   high band centered 320 Hz (range ~80..1280 Hz)
//   amplitude 0..1 linear (2^lin, lin in [-8,0]; <= -7.9375 -> off)
// Decoded amplitude is exposed on a 0..MAX_AMP linear scale.
#pragma once
#include <cstddef>
#include <cstdint>

namespace rumble {

constexpr uint16_t MAX_AMP = 1003; // full-scale linear amplitude in Decoded

struct Decoded {
    uint16_t hf_hz = 0;   // high-band frequency, Hz (0 = off)
    uint16_t lf_hz = 0;   // low-band frequency, Hz
    uint16_t hf_amp = 0;  // 0..MAX_AMP, high-band amplitude (linear)
    uint16_t lf_amp = 0;  // 0..MAX_AMP, low-band amplitude (linear)
};

// Stateful per-controller-side decoder. The codec is differential, so one
// instance must persist per side and be fed each 4-byte side in arrival order
// (left = data[0..3], right = data[4..7]).
class SideDecoder {
public:
    SideDecoder() { reset(); }
    void reset();                          // back to silent/centered defaults
    Decoded decode(const uint8_t d[4]);    // returns the latest sample
private:
    Decoded current() const;
    void apply5(uint8_t code, bool high);  // 5-bit am+fm command on one band
    float la_, lf_, ha_, hf_;              // linear amp/freq, low & high bands
};

// Convenience: decode one 4-byte side from a fresh (default) decoder. Only
// meaningful for absolute packets; stateful streams must use SideDecoder/State.
Decoded decode_side(const uint8_t d[4]);

// amp 0..MAX_AMP -> 0x83 gain_db (~20*log10(amp/MAX_AMP)), clamped to -40;
// -128 ("off") for amp 0.
int8_t amp_to_db(uint16_t amp);

// ---- output: HD rumble as four independent 0x83 tones ----
//
// 0x83 MsgHapticLfoTone is a per-actuator (frequency, gain) tone generator,
// matching the codec's per-band (frequency, amplitude). Each decoded channel
// drives one actuator's own 0x83 stream (grips take frequency via 0x83, not
// just the pads):
//
//   left  low band  -> left  GRIP (side 3)    right low band  -> right GRIP (side 4)
//   left  high band -> left  PAD  (side 0)    right high band -> right PAD  (side 1)
//
// gain comes from the band amplitude plus a per-band trim (GRIP_GAIN_DB for the
// low band on the grips, PAD_GAIN_DB for the high band on the pads).
// side 5 (both grips at once) is avoided -- same-frequency beating.

constexpr uint8_t N_TONE = 4; // four actuators driven independently

// Actuator order (index into State::tone()); ACT_SIDE maps to the 0x83 `side`.
enum Actuator : uint8_t { ACT_L_GRIP = 0, ACT_R_GRIP = 1, ACT_L_PAD = 2, ACT_R_PAD = 3 };

constexpr size_t TONE_LEN = 10; // 0x83 report incl. id

// 0x83 tone duration; outlives the 40 ms resend cadence, dies fast on stop.
constexpr uint16_t TONE_DURATION_MS = 100;

// Tuning. Each band adds a trim (dB) on top of the per-tone amp_to_db (which is
// <= 0 dB, 0 at full amp); grips and pads differ in output, so they trim
// independently. Freq clamp keeps us in the codec's range and off the very low
// end where grips get drummy. Calibrate with tools/rumble_calibrate.py.
constexpr int8_t GRIP_GAIN_DB = -3;      // low band -> grips trim (TUNE on hw)
constexpr int8_t PAD_GAIN_DB = 4;        // high band -> pads trim (TUNE on hw)
constexpr int8_t GAIN_MAX_DB = 6;        // clip ceiling (SDL allows positive)
constexpr int8_t GAIN_MIN_DB = -40;      // inaudible floor
constexpr uint16_t FREQ_MIN_HZ = 40;     // codec low-band floor
constexpr uint16_t FREQ_MAX_HZ = 1280;   // codec high-band ceiling

struct TonePacket {
    uint8_t bytes[TONE_LEN] = {0x83, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool active = false; // false = don't send; a playing tone expires by duration
};

// Build one actuator's 0x83 tone from a band's (freq, amp). side = 0x83 `side`
// selector; trim_db adds to the amplitude-derived gain (GRIP_GAIN_DB/PAD_GAIN_DB).
// amp 0 or freq 0 -> inactive (no tone).
TonePacket to_tone(uint8_t side, uint16_t freq_hz, uint16_t amp, int8_t trim_db);

// Decodes a full 8-byte Switch rumble payload and tracks the four output tones.
class State {
public:
    // Returns true if any output tone changed.
    bool update(const uint8_t data[8]);
    const TonePacket& tone(int actuator) const { return tones_[actuator & 3]; }
    bool active() const {
        return tones_[0].active || tones_[1].active ||
               tones_[2].active || tones_[3].active;
    }
    const Decoded& left() const { return l_; }
    const Decoded& right() const { return r_; }

private:
    SideDecoder ldec_, rdec_;
    Decoded l_, r_;
    TonePacket tones_[N_TONE];
};

} // namespace rumble
