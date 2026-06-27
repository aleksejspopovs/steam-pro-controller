#include "rumble.h"

#include <cmath>

namespace rumble {

// ---- Switch AM/FM rumble codec (see reference/switch_rumble_decoder.*) ----
namespace {

constexpr float AMP_LIN_MIN = -8.0f, AMP_LIN_MAX = 0.0f, AMP_LIN_DEF = -8.0f;
constexpr float FRQ_LIN_MIN = -2.0f, FRQ_LIN_MAX = 2.0f, FRQ_LIN_DEF = 0.0f;
constexpr float CENTER_LO = 160.0f, CENTER_HI = 320.0f;
constexpr float AMP_THRESH = -7.9375f; // below this, amplitude reads as off

enum Action : uint8_t { ACT_IGNORE = 0, ACT_DEFAULT = 1, ACT_SUBSTITUTE = 2, ACT_SUM = 3 };

struct Cmd { uint8_t am_a, fm_a; float am_o, fm_o; };

// 32-entry 5-bit command table (am/fm action + offset), CommandTable[] in ref.
constexpr Cmd CMD[32] = {
    {ACT_DEFAULT, ACT_DEFAULT, 0.0f, 0.0f},
    {ACT_SUBSTITUTE, ACT_IGNORE, 0.0f, 0.0f},  {ACT_SUBSTITUTE, ACT_IGNORE, -0.5f, 0.0f},
    {ACT_SUBSTITUTE, ACT_IGNORE, -1.0f, 0.0f}, {ACT_SUBSTITUTE, ACT_IGNORE, -1.5f, 0.0f},
    {ACT_SUBSTITUTE, ACT_IGNORE, -2.0f, 0.0f}, {ACT_SUBSTITUTE, ACT_IGNORE, -2.5f, 0.0f},
    {ACT_SUBSTITUTE, ACT_IGNORE, -3.0f, 0.0f}, {ACT_SUBSTITUTE, ACT_IGNORE, -3.5f, 0.0f},
    {ACT_SUBSTITUTE, ACT_IGNORE, -4.0f, 0.0f}, {ACT_SUBSTITUTE, ACT_IGNORE, -4.5f, 0.0f},
    {ACT_SUBSTITUTE, ACT_IGNORE, -5.0f, 0.0f},
    {ACT_IGNORE, ACT_SUBSTITUTE, 0.0f, -0.375f},  {ACT_IGNORE, ACT_SUBSTITUTE, 0.0f, -0.1875f},
    {ACT_IGNORE, ACT_SUBSTITUTE, 0.0f, 0.0f},     {ACT_IGNORE, ACT_SUBSTITUTE, 0.0f, 0.1875f},
    {ACT_IGNORE, ACT_SUBSTITUTE, 0.0f, 0.375f},
    {ACT_SUM, ACT_SUM, 0.125f, 0.03125f},   {ACT_SUM, ACT_IGNORE, 0.125f, 0.0f},
    {ACT_SUM, ACT_SUM, 0.125f, -0.03125f},  {ACT_SUM, ACT_SUM, 0.03125f, 0.03125f},
    {ACT_SUM, ACT_IGNORE, 0.03125f, 0.0f},  {ACT_SUM, ACT_SUM, 0.03125f, -0.03125f},
    {ACT_IGNORE, ACT_SUM, 0.0f, 0.03125f},  {ACT_IGNORE, ACT_IGNORE, 0.0f, 0.0f},
    {ACT_IGNORE, ACT_SUM, 0.0f, -0.03125f}, {ACT_SUM, ACT_SUM, -0.03125f, 0.03125f},
    {ACT_SUM, ACT_IGNORE, -0.03125f, 0.0f}, {ACT_SUM, ACT_SUM, -0.03125f, -0.03125f},
    {ACT_SUM, ACT_SUM, -0.125f, 0.03125f},  {ACT_SUM, ACT_IGNORE, -0.125f, 0.0f},
    {ACT_SUM, ACT_SUM, -0.125f, -0.03125f},
};

// 7-bit absolute amplitude / frequency lookups (Am7BitLookup / Fm7BitLookup).
float am7(uint8_t i) {
    if (i == 0) return -8.0f;
    if (i < 16) return 0.25f * i - 7.75f;
    if (i < 32) return 0.0625f * i - 4.9375f;
    return 0.03125f * i - 3.96875f;
}
float fm7(uint8_t i) { return 0.03125f * i - 2.0f; }

// 2^lin quantized to the reference's 1/32 grid from -8.0; off below threshold.
float exp2q(float lin) {
    int idx = (int)((lin - AMP_LIN_MIN) * 32.0f); // lin >= -8, truncation = floor
    float q = AMP_LIN_MIN + idx * (1.0f / 32.0f);
    return q >= AMP_THRESH ? exp2f(q) : 0.0f;
}

float apply(uint8_t action, float offset, float cur, float defv, float lo, float hi) {
    switch (action) {
        case ACT_SUBSTITUTE: return offset;
        case ACT_SUM: { float v = cur + offset; return v < lo ? lo : (v > hi ? hi : v); }
        case ACT_IGNORE: return cur;
        default: return defv;
    }
}

inline uint32_t bits(uint32_t v, int off, int width) {
    return (v >> off) & ((1u << width) - 1u);
}

} // namespace

void SideDecoder::reset() {
    la_ = AMP_LIN_DEF; lf_ = FRQ_LIN_DEF;
    ha_ = AMP_LIN_DEF; hf_ = FRQ_LIN_DEF;
}

void SideDecoder::apply5(uint8_t code, bool high) {
    const Cmd& c = CMD[code & 31];
    float& a = high ? ha_ : la_;
    float& f = high ? hf_ : lf_;
    a = apply(c.am_a, c.am_o, a, AMP_LIN_DEF, AMP_LIN_MIN, AMP_LIN_MAX);
    f = apply(c.fm_a, c.fm_o, f, FRQ_LIN_DEF, FRQ_LIN_MIN, FRQ_LIN_MAX);
}

Decoded SideDecoder::current() const {
    Decoded d;
    d.lf_hz = (uint16_t)lroundf(exp2q(lf_) * CENTER_LO);
    d.hf_hz = (uint16_t)lroundf(exp2q(hf_) * CENTER_HI);
    d.lf_amp = (uint16_t)lroundf(exp2q(la_) * (float)MAX_AMP);
    d.hf_amp = (uint16_t)lroundf(exp2q(ha_) * (float)MAX_AMP);
    return d;
}

// Decode one 4-byte side, advancing per-side state; return the final sample.
// (The codec emits 1..3 sub-samples per packet meant to play in sequence at
// ~1 ms; we forward at a coarse cadence, so the latest commanded value wins.)
Decoded SideDecoder::decode(const uint8_t d[4]) {
    uint32_t v = (uint32_t)d[0] | ((uint32_t)d[1] << 8) |
                 ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    switch (bits(v, 30, 2)) {
    case 0: // no samples -> hold current state
        break;
    case 1:
        if (bits(v, 0, 20) == 0) {            // one5bit
            apply5(bits(v, 25, 5), false);
            apply5(bits(v, 20, 5), true);
        } else if (bits(v, 0, 2) == 0) {      // one7bit (absolute)
            la_ = am7(bits(v, 23, 7)); lf_ = fm7(bits(v, 16, 7));
            ha_ = am7(bits(v, 9, 7));  hf_ = fm7(bits(v, 2, 7));
        } else {                              // three7bit
            uint8_t c7 = bits(v, 23, 7);
            bool hs = bits(v, 0, 1), fs = bits(v, 2, 1);
            if (hs) { if (fs) hf_ = fm7(c7); else ha_ = am7(c7); }
            else    { if (fs) lf_ = fm7(c7); else la_ = am7(c7); }
            apply5(bits(v, 18, 5), false); apply5(bits(v, 13, 5), true);
            apply5(bits(v, 8, 5), false);  apply5(bits(v, 3, 5), true);
        }
        break;
    case 2:
        if (bits(v, 0, 10) == 0) {            // two5bit
            apply5(bits(v, 25, 5), false); apply5(bits(v, 20, 5), true);
            apply5(bits(v, 15, 5), false); apply5(bits(v, 10, 5), true);
        } else {                              // two7bit
            bool hs = bits(v, 0, 1);
            uint8_t fm_xx = bits(v, 1, 7), am_xx = bits(v, 23, 7);
            if (hs) { ha_ = am7(am_xx); hf_ = fm7(fm_xx); apply5(bits(v, 18, 5), false); }
            else    { la_ = am7(am_xx); lf_ = fm7(fm_xx); apply5(bits(v, 18, 5), true); }
            apply5(bits(v, 13, 5), false); apply5(bits(v, 8, 5), true);
        }
        break;
    default: // case 3: three5bit
        apply5(bits(v, 25, 5), false); apply5(bits(v, 20, 5), true);
        apply5(bits(v, 15, 5), false); apply5(bits(v, 10, 5), true);
        apply5(bits(v, 5, 5), false);  apply5(bits(v, 0, 5), true);
        break;
    }
    return current();
}

Decoded decode_side(const uint8_t d[4]) {
    SideDecoder dec;
    return dec.decode(d);
}

static void wr_u16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

int8_t amp_to_db(uint16_t amp) {
    if (amp == 0) return -128; // off
    // thresholds: MAX_AMP * 10^(-i/20) for i = 0..40 dB of attenuation
    static const uint16_t THR[] = {
        1003, 894, 797, 710, 633, 564, 503, 448, 400, 356,
        317,  283, 252, 225, 200, 178, 159, 142, 127, 113,
        100,  89,  80,  71,  63,  57,  50,  45,  40,  36,
        32,   28,  25,  23,  20,  18,  16,  14,  13,  11,
        10,
    };
    for (int i = 0; i < (int)(sizeof(THR) / sizeof(THR[0])); i++) {
        if (amp >= THR[i]) return (int8_t)-i;
    }
    return -40;
}

float corr_db(const CorrPoint* tbl, size_t n, uint16_t hz) {
    if (!tbl || n == 0) return 0.0f;
    if (hz <= tbl[0].hz) return tbl[0].db;            // hold flat past the ends
    if (hz >= tbl[n - 1].hz) return tbl[n - 1].db;
    for (size_t i = 1; i < n; i++) {
        if (hz <= tbl[i].hz) {                        // interpolate in log-freq
            float l0 = logf((float)tbl[i - 1].hz), l1 = logf((float)tbl[i].hz);
            float u = (logf((float)hz) - l0) / (l1 - l0);
            return tbl[i - 1].db + u * (tbl[i].db - tbl[i - 1].db);
        }
    }
    return tbl[n - 1].db;
}

TonePacket to_tone(uint8_t side, uint16_t freq_hz, uint16_t amp, int8_t trim_db,
                   const CorrPoint* corr, size_t corr_n) {
    TonePacket p;
    p.bytes[1] = side;
    if (amp == 0 || freq_hz == 0)
        return p; // inactive; a playing tone expires by its duration

    // MsgHapticLfoTone: [0x83, side, gain_db, freq, duration_ms, lfo, depth].
    p.active = true;
    uint16_t f = freq_hz;                             // clamp first; EQ uses the
    if (f < FREQ_MIN_HZ) f = FREQ_MIN_HZ;             // frequency we actually emit
    if (f > FREQ_MAX_HZ) f = FREQ_MAX_HZ;
    // gain = band amplitude (dB, <=0) + loudness trim + freq-response EQ; lfo off.
    int g = (int)amp_to_db(amp) + (int)trim_db + (int)lroundf(corr_db(corr, corr_n, f));
    if (g > (int)GAIN_MAX_DB) g = (int)GAIN_MAX_DB;
    if (g < (int)GAIN_MIN_DB) g = (int)GAIN_MIN_DB;
    p.bytes[2] = (uint8_t)(int8_t)g;
    wr_u16(p.bytes + 3, f);
    wr_u16(p.bytes + 5, TONE_DURATION_MS);
    return p;
}

bool State::update(const uint8_t data[8]) {
    l_ = ldec_.decode(data);
    r_ = rdec_.decode(data + 4);
    // low bands -> grips (side 3/4), high bands -> pads (side 0/1)
    const TonePacket next[N_TONE] = {
        to_tone(3, l_.lf_hz, l_.lf_amp, GRIP_GAIN_DB, GRIP_CORR, GRIP_CORR_N), // ACT_L_GRIP
        to_tone(4, r_.lf_hz, r_.lf_amp, GRIP_GAIN_DB, GRIP_CORR, GRIP_CORR_N), // ACT_R_GRIP
        to_tone(0, l_.hf_hz, l_.hf_amp, PAD_GAIN_DB, PAD_CORR, PAD_CORR_N),    // ACT_L_PAD
        to_tone(1, r_.hf_hz, r_.hf_amp, PAD_GAIN_DB, PAD_CORR, PAD_CORR_N),    // ACT_R_PAD
    };
    bool changed = false;
    for (size_t a = 0; a < N_TONE && !changed; a++) {
        if (next[a].active != tones_[a].active) { changed = true; break; }
        for (size_t i = 0; i < TONE_LEN && !changed; i++)
            changed = next[a].bytes[i] != tones_[a].bytes[i];
    }
    for (size_t a = 0; a < N_TONE; a++)
        tones_[a] = next[a];
    return changed;
}

} // namespace rumble
