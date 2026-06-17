#include "rumble.h"

namespace rumble {

struct FreqData { uint16_t high; uint8_t low; uint16_t freq; };
struct AmpData { uint8_t high; uint16_t low; uint16_t amp; };

// https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/rumble_data_table.md
// (same tables as hid-nintendo.c)
static const FreqData FREQ_TABLE[] = {
    { 0x0000, 0x01,   41 }, { 0x0000, 0x02,   42 }, { 0x0000, 0x03,   43 },
    { 0x0000, 0x04,   44 }, { 0x0000, 0x05,   45 }, { 0x0000, 0x06,   46 },
    { 0x0000, 0x07,   47 }, { 0x0000, 0x08,   48 }, { 0x0000, 0x09,   49 },
    { 0x0000, 0x0A,   50 }, { 0x0000, 0x0B,   51 }, { 0x0000, 0x0C,   52 },
    { 0x0000, 0x0D,   53 }, { 0x0000, 0x0E,   54 }, { 0x0000, 0x0F,   55 },
    { 0x0000, 0x10,   57 }, { 0x0000, 0x11,   58 }, { 0x0000, 0x12,   59 },
    { 0x0000, 0x13,   60 }, { 0x0000, 0x14,   62 }, { 0x0000, 0x15,   63 },
    { 0x0000, 0x16,   64 }, { 0x0000, 0x17,   66 }, { 0x0000, 0x18,   67 },
    { 0x0000, 0x19,   69 }, { 0x0000, 0x1A,   70 }, { 0x0000, 0x1B,   72 },
    { 0x0000, 0x1C,   73 }, { 0x0000, 0x1D,   75 }, { 0x0000, 0x1e,   77 },
    { 0x0000, 0x1f,   78 }, { 0x0000, 0x20,   80 }, { 0x0400, 0x21,   82 },
    { 0x0800, 0x22,   84 }, { 0x0c00, 0x23,   85 }, { 0x1000, 0x24,   87 },
    { 0x1400, 0x25,   89 }, { 0x1800, 0x26,   91 }, { 0x1c00, 0x27,   93 },
    { 0x2000, 0x28,   95 }, { 0x2400, 0x29,   97 }, { 0x2800, 0x2a,   99 },
    { 0x2c00, 0x2b,  102 }, { 0x3000, 0x2c,  104 }, { 0x3400, 0x2d,  106 },
    { 0x3800, 0x2e,  108 }, { 0x3c00, 0x2f,  111 }, { 0x4000, 0x30,  113 },
    { 0x4400, 0x31,  116 }, { 0x4800, 0x32,  118 }, { 0x4c00, 0x33,  121 },
    { 0x5000, 0x34,  123 }, { 0x5400, 0x35,  126 }, { 0x5800, 0x36,  129 },
    { 0x5c00, 0x37,  132 }, { 0x6000, 0x38,  135 }, { 0x6400, 0x39,  137 },
    { 0x6800, 0x3a,  141 }, { 0x6c00, 0x3b,  144 }, { 0x7000, 0x3c,  147 },
    { 0x7400, 0x3d,  150 }, { 0x7800, 0x3e,  153 }, { 0x7c00, 0x3f,  157 },
    { 0x8000, 0x40,  160 }, { 0x8400, 0x41,  164 }, { 0x8800, 0x42,  167 },
    { 0x8c00, 0x43,  171 }, { 0x9000, 0x44,  174 }, { 0x9400, 0x45,  178 },
    { 0x9800, 0x46,  182 }, { 0x9c00, 0x47,  186 }, { 0xa000, 0x48,  190 },
    { 0xa400, 0x49,  194 }, { 0xa800, 0x4a,  199 }, { 0xac00, 0x4b,  203 },
    { 0xb000, 0x4c,  207 }, { 0xb400, 0x4d,  212 }, { 0xb800, 0x4e,  217 },
    { 0xbc00, 0x4f,  221 }, { 0xc000, 0x50,  226 }, { 0xc400, 0x51,  231 },
    { 0xc800, 0x52,  236 }, { 0xcc00, 0x53,  241 }, { 0xd000, 0x54,  247 },
    { 0xd400, 0x55,  252 }, { 0xd800, 0x56,  258 }, { 0xdc00, 0x57,  263 },
    { 0xe000, 0x58,  269 }, { 0xe400, 0x59,  275 }, { 0xe800, 0x5a,  281 },
    { 0xec00, 0x5b,  287 }, { 0xf000, 0x5c,  293 }, { 0xf400, 0x5d,  300 },
    { 0xf800, 0x5e,  306 }, { 0xfc00, 0x5f,  313 }, { 0x0001, 0x60,  320 },
    { 0x0401, 0x61,  327 }, { 0x0801, 0x62,  334 }, { 0x0c01, 0x63,  341 },
    { 0x1001, 0x64,  349 }, { 0x1401, 0x65,  357 }, { 0x1801, 0x66,  364 },
    { 0x1c01, 0x67,  372 }, { 0x2001, 0x68,  381 }, { 0x2401, 0x69,  389 },
    { 0x2801, 0x6a,  397 }, { 0x2c01, 0x6b,  406 }, { 0x3001, 0x6c,  415 },
    { 0x3401, 0x6d,  424 }, { 0x3801, 0x6e,  433 }, { 0x3c01, 0x6f,  443 },
    { 0x4001, 0x70,  453 }, { 0x4401, 0x71,  462 }, { 0x4801, 0x72,  473 },
    { 0x4c01, 0x73,  483 }, { 0x5001, 0x74,  494 }, { 0x5401, 0x75,  504 },
    { 0x5801, 0x76,  515 }, { 0x5c01, 0x77,  527 }, { 0x6001, 0x78,  538 },
    { 0x6401, 0x79,  550 }, { 0x6801, 0x7a,  562 }, { 0x6c01, 0x7b,  574 },
    { 0x7001, 0x7c,  587 }, { 0x7401, 0x7d,  600 }, { 0x7801, 0x7e,  613 },
    { 0x7c01, 0x7f,  626 }, { 0x8001, 0x00,  640 }, { 0x8401, 0x00,  654 },
    { 0x8801, 0x00,  668 }, { 0x8c01, 0x00,  683 }, { 0x9001, 0x00,  698 },
    { 0x9401, 0x00,  713 }, { 0x9801, 0x00,  729 }, { 0x9c01, 0x00,  745 },
    { 0xa001, 0x00,  761 }, { 0xa401, 0x00,  778 }, { 0xa801, 0x00,  795 },
    { 0xac01, 0x00,  812 }, { 0xb001, 0x00,  830 }, { 0xb401, 0x00,  848 },
    { 0xb801, 0x00,  867 }, { 0xbc01, 0x00,  886 }, { 0xc001, 0x00,  905 },
    { 0xc401, 0x00,  925 }, { 0xc801, 0x00,  945 }, { 0xcc01, 0x00,  966 },
    { 0xd001, 0x00,  987 }, { 0xd401, 0x00, 1009 }, { 0xd801, 0x00, 1031 },
    { 0xdc01, 0x00, 1053 }, { 0xe001, 0x00, 1076 }, { 0xe401, 0x00, 1100 },
    { 0xe801, 0x00, 1124 }, { 0xec01, 0x00, 1149 }, { 0xf001, 0x00, 1174 },
    { 0xf401, 0x00, 1199 }, { 0xf801, 0x00, 1226 }, { 0xfc01, 0x00, 1253 }
};
static const size_t N_FREQ = sizeof(FREQ_TABLE) / sizeof(FREQ_TABLE[0]);

static const AmpData AMP_TABLE[] = {
    { 0x00, 0x0040,    0 },
    { 0x02, 0x8040,   10 }, { 0x04, 0x0041,   12 }, { 0x06, 0x8041,   14 },
    { 0x08, 0x0042,   17 }, { 0x0a, 0x8042,   20 }, { 0x0c, 0x0043,   24 },
    { 0x0e, 0x8043,   28 }, { 0x10, 0x0044,   33 }, { 0x12, 0x8044,   40 },
    { 0x14, 0x0045,   47 }, { 0x16, 0x8045,   56 }, { 0x18, 0x0046,   67 },
    { 0x1a, 0x8046,   80 }, { 0x1c, 0x0047,   95 }, { 0x1e, 0x8047,  112 },
    { 0x20, 0x0048,  117 }, { 0x22, 0x8048,  123 }, { 0x24, 0x0049,  128 },
    { 0x26, 0x8049,  134 }, { 0x28, 0x004a,  140 }, { 0x2a, 0x804a,  146 },
    { 0x2c, 0x004b,  152 }, { 0x2e, 0x804b,  159 }, { 0x30, 0x004c,  166 },
    { 0x32, 0x804c,  173 }, { 0x34, 0x004d,  181 }, { 0x36, 0x804d,  189 },
    { 0x38, 0x004e,  198 }, { 0x3a, 0x804e,  206 }, { 0x3c, 0x004f,  215 },
    { 0x3e, 0x804f,  225 }, { 0x40, 0x0050,  230 }, { 0x42, 0x8050,  235 },
    { 0x44, 0x0051,  240 }, { 0x46, 0x8051,  245 }, { 0x48, 0x0052,  251 },
    { 0x4a, 0x8052,  256 }, { 0x4c, 0x0053,  262 }, { 0x4e, 0x8053,  268 },
    { 0x50, 0x0054,  273 }, { 0x52, 0x8054,  279 }, { 0x54, 0x0055,  286 },
    { 0x56, 0x8055,  292 }, { 0x58, 0x0056,  298 }, { 0x5a, 0x8056,  305 },
    { 0x5c, 0x0057,  311 }, { 0x5e, 0x8057,  318 }, { 0x60, 0x0058,  325 },
    { 0x62, 0x8058,  332 }, { 0x64, 0x0059,  340 }, { 0x66, 0x8059,  347 },
    { 0x68, 0x005a,  355 }, { 0x6a, 0x805a,  362 }, { 0x6c, 0x005b,  370 },
    { 0x6e, 0x805b,  378 }, { 0x70, 0x005c,  387 }, { 0x72, 0x805c,  395 },
    { 0x74, 0x005d,  404 }, { 0x76, 0x805d,  413 }, { 0x78, 0x005e,  422 },
    { 0x7a, 0x805e,  431 }, { 0x7c, 0x005f,  440 }, { 0x7e, 0x805f,  450 },
    { 0x80, 0x0060,  460 }, { 0x82, 0x8060,  470 }, { 0x84, 0x0061,  480 },
    { 0x86, 0x8061,  491 }, { 0x88, 0x0062,  501 }, { 0x8a, 0x8062,  512 },
    { 0x8c, 0x0063,  524 }, { 0x8e, 0x8063,  535 }, { 0x90, 0x0064,  547 },
    { 0x92, 0x8064,  559 }, { 0x94, 0x0065,  571 }, { 0x96, 0x8065,  584 },
    { 0x98, 0x0066,  596 }, { 0x9a, 0x8066,  609 }, { 0x9c, 0x0067,  623 },
    { 0x9e, 0x8067,  636 }, { 0xa0, 0x0068,  650 }, { 0xa2, 0x8068,  665 },
    { 0xa4, 0x0069,  679 }, { 0xa6, 0x8069,  694 }, { 0xa8, 0x006a,  709 },
    { 0xaa, 0x806a,  725 }, { 0xac, 0x006b,  741 }, { 0xae, 0x806b,  757 },
    { 0xb0, 0x006c,  773 }, { 0xb2, 0x806c,  790 }, { 0xb4, 0x006d,  808 },
    { 0xb6, 0x806d,  825 }, { 0xb8, 0x006e,  843 }, { 0xba, 0x806e,  862 },
    { 0xbc, 0x006f,  881 }, { 0xbe, 0x806f,  900 }, { 0xc0, 0x0070,  920 },
    { 0xc2, 0x8070,  940 }, { 0xc4, 0x0071,  960 }, { 0xc6, 0x8071,  981 },
    { 0xc8, 0x0072, MAX_AMP }
};
static const size_t N_AMP = sizeof(AMP_TABLE) / sizeof(AMP_TABLE[0]);

static const FreqData& find_freq(uint16_t freq) {
    size_t i = 0;
    if (freq > FREQ_TABLE[0].freq) {
        for (i = 1; i < N_FREQ - 1; i++) {
            if (freq > FREQ_TABLE[i - 1].freq && freq <= FREQ_TABLE[i].freq)
                break;
        }
    }
    return FREQ_TABLE[i];
}

static const AmpData& find_amp(uint16_t amp) {
    size_t i = 0;
    if (amp > AMP_TABLE[0].amp) {
        for (i = 1; i < N_AMP - 1; i++) {
            if (amp > AMP_TABLE[i - 1].amp && amp <= AMP_TABLE[i].amp)
                break;
        }
    }
    return AMP_TABLE[i];
}

void encode_side(uint8_t d[4], uint16_t hf_hz, uint16_t lf_hz, uint16_t amp) {
    const FreqData& fh = find_freq(hf_hz);
    const FreqData& fl = find_freq(lf_hz);
    const AmpData& a = find_amp(amp);
    d[0] = (uint8_t)((fh.high >> 8) & 0xFF);
    d[1] = (uint8_t)((fh.high & 0xFF) + a.high);
    d[2] = (uint8_t)(fl.low + ((a.low >> 8) & 0xFF));
    d[3] = (uint8_t)(a.low & 0xFF);
}

Decoded decode_side(const uint8_t d[4]) {
    Decoded out;

    // high-band amp: AMP_TABLE[i].high == 2*i, exact in d[1]
    size_t ai = (size_t)(d[1] & 0xFE) >> 1;
    if (ai >= N_AMP) ai = N_AMP - 1;
    out.hf_amp = AMP_TABLE[ai].amp;

    // low-band amp: AMP_TABLE[i].low = (i&1 ? 0x8000 : 0) | (0x40 + i/2),
    // split as d[3] = low byte, d[2] bit7 = the index lsb
    if (d[3] >= 0x40) {
        size_t li = (size_t)(d[3] - 0x40) * 2 + ((d[2] >> 7) & 1);
        if (li >= N_AMP) li = N_AMP - 1;
        out.lf_amp = AMP_TABLE[li].amp;
    }

    // low band: 7-bit code, FREQ_TABLE[c-1].low == c for c in 1..0x7F
    uint8_t lc = d[2] & 0x7F;
    if (lc >= 1 && lc <= 0x7F)
        out.lf_hz = FREQ_TABLE[lc - 1].freq;

    // high band: u16 split across d[0] and bit0 of d[1]
    uint16_t hf = (uint16_t)((d[0] << 8) | (d[1] & 0x01));
    if (hf != 0) {
        for (size_t i = 0; i < N_FREQ; i++) {
            if (FREQ_TABLE[i].high == hf) {
                out.hf_hz = FREQ_TABLE[i].freq;
                break;
            }
        }
    }
    return out;
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

int8_t grip_gain(uint16_t lf_amp) {
    // amp_to_db(lf_amp) is <= 0 (0 dB at full amp), so this rides PEAK down with
    // the Switch amplitude envelope. Clamp to the floor; never above PEAK.
    int g = (int)PEAK_GRIP_GAIN_DB + (int)amp_to_db(lf_amp);
    if (g < (int)MIN_GRIP_GAIN_DB) g = (int)MIN_GRIP_GAIN_DB;
    if (g > (int)PEAK_GRIP_GAIN_DB) g = (int)PEAK_GRIP_GAIN_DB;
    return (int8_t)g;
}

ScPacket to_sc(const Decoded& left, const Decoded& right) {
    ScPacket p;
    if (left.lf_amp == 0 && right.lf_amp == 0)
        return p; // all-zero = stop

    // MsgHapticRumble layout: [0x80, type, intensity_u16, left_speed_u16,
    // left_gain_s8, right_speed_u16, right_gain_s8]. type/intensity stay 0.
    // speed must be nonzero for the motor to run (gain on a speed=0 channel does
    // nothing); we use a fixed gentle throb. Loudness + envelope ride in gain.
    p.active = true;
    if (left.lf_amp) {
        wr_u16(p.bytes + 4, GRIP_SPEED);
        p.bytes[6] = (uint8_t)grip_gain(left.lf_amp);
    }
    if (right.lf_amp) {
        wr_u16(p.bytes + 7, GRIP_SPEED);
        p.bytes[9] = (uint8_t)grip_gain(right.lf_amp);
    }
    return p;
}

PadTonePacket to_pad_tone(uint8_t side, const Decoded& d) {
    PadTonePacket p;
    p.bytes[1] = side; // 0 = left pad, 1 = right pad
    if (d.hf_amp == 0 || d.hf_hz == 0)
        return p; // inactive; a playing tone expires by its duration

    // MsgHapticLfoTone: [0x83, side, gain_db, freq, duration_ms, lfo, depth]
    p.active = true;
    p.bytes[2] = (uint8_t)amp_to_db(d.hf_amp);
    wr_u16(p.bytes + 3, d.hf_hz);
    wr_u16(p.bytes + 5, TONE_DURATION_MS);
    return p;
}

bool State::update(const uint8_t data[8]) {
    l_ = decode_side(data);
    r_ = decode_side(data + 4);
    ScPacket next = to_sc(l_, r_);
    PadTonePacket nl = to_pad_tone(0, l_);
    PadTonePacket nr = to_pad_tone(1, r_);
    bool changed = next.active != pkt_.active ||
                   nl.active != pads_[0].active ||
                   nr.active != pads_[1].active;
    for (size_t i = 0; i < SC_PACKET_LEN && !changed; i++)
        changed = next.bytes[i] != pkt_.bytes[i];
    for (size_t i = 0; i < SC_TONE_LEN && !changed; i++)
        changed = nl.bytes[i] != pads_[0].bytes[i] ||
                  nr.bytes[i] != pads_[1].bytes[i];
    pkt_ = next;
    pads_[0] = nl;
    pads_[1] = nr;
    return changed;
}

} // namespace rumble
