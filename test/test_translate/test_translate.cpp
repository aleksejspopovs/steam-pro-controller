// Rig 1, part 2: SC -> Switch translation against fixtures + known scalars.
#include <unity.h>

#include "../common/fixtures.hpp"
#include "translate.h"

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

// ---- IMU ----

// Known-value remap: SC (X right, Y fwd, Z up) -> Switch (X fwd, Y left, Z up);
// accel /4 (16384 -> 4096 LSB/g), gyro *14247/16384 (16.384 -> 14.247 LSB/dps).
static void test_imu_known_values(void) {
    sc::State s;
    s.accel[0] = 4000; s.accel[1] = 8000; s.accel[2] = 16384;
    s.gyro[0] = 1000; s.gyro[1] = 2000; s.gyro[2] = 3000;
    xl::ImuSample m = xl::map_imu(s);
    TEST_ASSERT_EQUAL_INT16(2000, m.ax);   //  sc_ay/4
    TEST_ASSERT_EQUAL_INT16(-1000, m.ay);  // -sc_ax/4
    TEST_ASSERT_EQUAL_INT16(4096, m.az);   //  sc_az/4
    TEST_ASSERT_EQUAL_INT16(1739, m.gx);   //  2000*14247/16384
    TEST_ASSERT_EQUAL_INT16(-869, m.gy);   // -1000*14247/16384
    TEST_ASSERT_EQUAL_INT16(2608, m.gz);   //  3000*14247/16384
}

// neutral fixture: flat at rest -> gravity must land on Switch +Z at
// ~4096 LSB/g (= the SC's ~16400 / 4), other axes near zero.
static void test_imu_neutral_gravity(void) {
    auto ns = load45("neutral");
    TEST_ASSERT_FALSE(ns.empty());
    double ax = 0, ay = 0, az = 0;
    for (auto& s : ns) {
        xl::ImuSample m = xl::map_imu(s);
        ax += m.ax; ay += m.ay; az += m.az;
    }
    ax /= (double)ns.size(); ay /= (double)ns.size(); az /= (double)ns.size();
    TEST_ASSERT_TRUE(3750 < az && az < 4500); // 15000/4 .. 18000/4
    TEST_ASSERT_TRUE(-1000 < ax && ax < 1000);
    TEST_ASSERT_TRUE(-1000 < ay && ay < 1000);
}

// imu_motion: the dominant SC gyro axes must survive the remap with the
// 0.8696 rescale (max |out| == max |in| * 14247/16384 within rounding).
static void test_imu_motion_scaling(void) {
    auto im = load45("imu_motion");
    TEST_ASSERT_FALSE(im.empty());
    int in_max[3] = {0, 0, 0};  // sc axes x,y,z
    int out_max[3] = {0, 0, 0}; // sw axes x,y,z
    for (auto& s : im) {
        xl::ImuSample m = xl::map_imu(s);
        const int gi[3] = {s.gyro[0], s.gyro[1], s.gyro[2]};
        const int go[3] = {m.gx, m.gy, m.gz};
        for (int i = 0; i < 3; i++) {
            int a = gi[i] < 0 ? -gi[i] : gi[i];
            int b = go[i] < 0 ? -go[i] : go[i];
            if (a > in_max[i]) in_max[i] = a;
            if (b > out_max[i]) out_max[i] = b;
        }
    }
    // sw x <- sc y, sw y <- sc x, sw z <- sc z
    TEST_ASSERT_INT_WITHIN(2, in_max[1] * 14247 / 16384, out_max[0]);
    TEST_ASSERT_INT_WITHIN(2, in_max[0] * 14247 / 16384, out_max[1]);
    TEST_ASSERT_INT_WITHIN(2, in_max[2] * 14247 / 16384, out_max[2]);
}

static void test_imu_resampler(void) {
    xl::ImuResampler rs;
    // ~268 Hz: a sample every 4 ms with a recognizable ramp
    for (int i = 0; i < 10; i++) {
        xl::ImuSample s;
        s.ax = (int16_t)(100 * i);
        rs.push(s, (uint64_t)(1000000 + i * 4000));
    }
    xl::ImuSample out[3];
    rs.sample3(1036000, out); // targets t=1026/1031/1036 ms
    // linear interpolation between the 4 ms ramp samples:
    //  t=1026: between i6(1024,600) and i7(1028,700), frac 0.5 -> 650
    //  t=1031: between i7(1028,700) and i8(1032,800), frac 0.75 -> 775
    //  t=1036: exactly i9(1036,900) -> 900 (clamped, no extrapolation)
    TEST_ASSERT_EQUAL_INT16(650, out[0].ax);
    TEST_ASSERT_EQUAL_INT16(775, out[1].ax);
    TEST_ASSERT_EQUAL_INT16(900, out[2].ax);
}

// Before the Puck pushes any sample, sample3 must emit a "flat at rest" frame
// (gravity down on +Z, no rotation), NOT all-zero freefall, so the Switch's
// orientation fusion isn't seeded with an impossible attitude on connect.
static void test_imu_resampler_rest_seed(void) {
    xl::ImuResampler rs;
    TEST_ASSERT_TRUE(rs.empty());
    xl::ImuSample out[3];
    rs.sample3(1000000, out);
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_EQUAL_INT16(4096, out[i].az); // 1 g down
        TEST_ASSERT_EQUAL_INT16(0, out[i].ax);
        TEST_ASSERT_EQUAL_INT16(0, out[i].ay);
        TEST_ASSERT_EQUAL_INT16(0, out[i].gx);
        TEST_ASSERT_EQUAL_INT16(0, out[i].gy);
        TEST_ASSERT_EQUAL_INT16(0, out[i].gz);
    }
}

// ---- sticks ----

static void test_stick_axis_known_values(void) {
    TEST_ASSERT_EQUAL_UINT16(2048, xl::map_stick_axis(0));
    TEST_ASSERT_EQUAL_UINT16(3840, xl::map_stick_axis(32767));  // cal max
    TEST_ASSERT_EQUAL_UINT16(256, xl::map_stick_axis(-32767));  // cal min
    TEST_ASSERT_EQUAL_UINT16(256, xl::map_stick_axis(-32768));  // clamped
}

// sticks_circles hits SC +/-32767 exactly -> mapped extremes must hit the
// served SPI cal min/max (256/3840) exactly.
static void test_sticks_circles_extremes(void) {
    auto ss = load45("sticks_circles");
    TEST_ASSERT_FALSE(ss.empty());
    uint16_t mn[4] = {4095, 4095, 4095, 4095};
    uint16_t mx[4] = {0, 0, 0, 0};
    for (auto& s : ss) {
        xl::SwInput in = xl::map_input(s);
        const uint16_t v[4] = {in.lx, in.ly, in.rx, in.ry};
        for (int i = 0; i < 4; i++) {
            if (v[i] < mn[i]) mn[i] = v[i];
            if (v[i] > mx[i]) mx[i] = v[i];
        }
    }
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_UINT16(256, mn[i]);
        TEST_ASSERT_EQUAL_UINT16(3840, mx[i]);
    }
}

// neutral: rest offsets (<=~1000 SC units) must map inside the dead zone
// around center (1500 SC units ~ 82 raw -> use 90).
static void test_sticks_neutral_centered(void) {
    auto ns = load45("neutral");
    TEST_ASSERT_FALSE(ns.empty());
    for (auto& s : ns) {
        xl::SwInput in = xl::map_input(s);
        for (uint16_t v : {in.lx, in.ly, in.rx, in.ry})
            TEST_ASSERT_INT_WITHIN(90, (int)xl::STICK_CENTER, (int)v);
    }
}

// ---- buttons ----

static void test_button_table(void) {
    static const struct { uint32_t sc_bit; uint32_t sw_mask; } TABLE[] = {
        // ABXY by position, not legend (SC = Xbox layout, Switch mirrored)
        {sc::BTN_A, xl::SW_B},          {sc::BTN_B, xl::SW_A},
        {sc::BTN_X, xl::SW_Y},          {sc::BTN_Y, xl::SW_X},
        {sc::BTN_L1, xl::SW_L},         {sc::BTN_R1, xl::SW_R},
        {sc::BTN_L2, xl::SW_ZL},        {sc::BTN_R2, xl::SW_ZR},
        {sc::BTN_MENU, xl::SW_PLUS},    {sc::BTN_VIEW, xl::SW_MINUS},
        {sc::BTN_STEAM, xl::SW_HOME},   {sc::BTN_QAM, xl::SW_CAP},
        {sc::BTN_L3, xl::SW_LSTICK},    {sc::BTN_R3, xl::SW_RSTICK},
        {sc::BTN_DPAD_UP, xl::SW_UP},   {sc::BTN_DPAD_DOWN, xl::SW_DOWN},
        {sc::BTN_DPAD_LEFT, xl::SW_LEFT}, {sc::BTN_DPAD_RIGHT, xl::SW_RIGHT},
        // no Pro Controller equivalent -> intentionally unmapped:
        {sc::BTN_GRIP_L4, 0}, {sc::BTN_GRIP_L5, 0},
        {sc::BTN_GRIP_R4, 0}, {sc::BTN_GRIP_R5, 0},
        {sc::BTN_LSTICK_TOUCH, 0}, {sc::BTN_RSTICK_TOUCH, 0},
        {sc::BTN_LPAD_TOUCH, 0}, {sc::BTN_RPAD_TOUCH, 0},
        {sc::BTN_LPAD_CLICK, 0}, {sc::BTN_RPAD_CLICK, 0},
        {sc::BTN_LGRIP_SENSE, 0}, {sc::BTN_RGRIP_SENSE, 0},
    };
    for (auto& row : TABLE) {
        sc::State s;
        s.buttons = row.sc_bit;
        TEST_ASSERT_EQUAL_UINT32(row.sw_mask, xl::map_buttons(s));
    }
}

static void test_trigger_threshold(void) {
    sc::State s;
    s.ltrig = xl::TRIGGER_THRESHOLD - 1;
    TEST_ASSERT_EQUAL_UINT32(0, xl::map_buttons(s));
    s.ltrig = xl::TRIGGER_THRESHOLD;
    TEST_ASSERT_EQUAL_UINT32(xl::SW_ZL, xl::map_buttons(s));
    s.ltrig = 0;
    s.rtrig = 32767;
    TEST_ASSERT_EQUAL_UINT32(xl::SW_ZR, xl::map_buttons(s));
}

// fixtures drive every mapped button: replay press orders through the
// translation and watch the *Switch* bits appear in fixture order.
static void test_buttons_fixture_order(void) {
    static const struct {
        const char* name;
        uint32_t sw[4];
    } CASES[] = {
        // SC presses A,B,X,Y -> positionally south,east,west,north
        {"buttons_abxy", {xl::SW_B, xl::SW_A, xl::SW_Y, xl::SW_X}},
        {"dpad", {xl::SW_UP, xl::SW_RIGHT, xl::SW_DOWN, xl::SW_LEFT}},
        {"menu_cluster", {xl::SW_MINUS, xl::SW_PLUS, xl::SW_HOME, xl::SW_CAP}},
    };
    for (auto& c : CASES) {
        std::vector<uint32_t> seen;
        for (auto& s : load45(c.name)) {
            uint32_t sw = xl::map_buttons(s);
            for (uint32_t m : c.sw) {
                bool already = false;
                for (uint32_t v : seen) already |= v == m;
                if (!already && (sw & m)) seen.push_back(m);
            }
        }
        TEST_ASSERT_EQUAL_size_t(4, seen.size());
        for (int i = 0; i < 4; i++)
            TEST_ASSERT_EQUAL_UINT32(c.sw[i], seen[i]);
    }
}

// bumpers fixture through the full translation
static void test_bumpers_translate(void) {
    uint32_t got = 0;
    for (auto& s : load45("bumpers"))
        got |= xl::map_buttons(s);
    TEST_ASSERT_EQUAL_UINT32(xl::SW_L | xl::SW_R,
                             got & (xl::SW_L | xl::SW_R));
}

// ---- battery ----

static void test_battery_level_mapping(void) {
    TEST_ASSERT_EQUAL_UINT8(4, xl::battery_level(100));
    TEST_ASSERT_EQUAL_UINT8(4, xl::battery_level(90));
    TEST_ASSERT_EQUAL_UINT8(3, xl::battery_level(75));
    TEST_ASSERT_EQUAL_UINT8(2, xl::battery_level(50));
    TEST_ASSERT_EQUAL_UINT8(1, xl::battery_level(25));
    TEST_ASSERT_EQUAL_UINT8(0, xl::battery_level(5));
    TEST_ASSERT_EQUAL_UINT8(0, xl::battery_level(0));
}

// The 9 zero-payload records flushed at disconnect (state 0) must read as
// "no data" via Battery::valid(), never as 0%.
static void test_battery_no_data_records(void) {
    auto recs = load_records("lifecycle_battery.raw43", sc::REPORT43_LEN);
    TEST_ASSERT_EQUAL_size_t(341, recs.size());
    int invalid = 0;
    for (auto& r : recs) {
        sc::Battery b;
        TEST_ASSERT_TRUE(sc::parse_43(r.data(), r.size(), b));
        if (!b.valid()) {
            invalid++;
            TEST_ASSERT_EQUAL_UINT8(0, b.percent); // zero record, not 0% data
        } else {
            TEST_ASSERT_GREATER_THAN_UINT8(0, b.percent);
        }
    }
    TEST_ASSERT_EQUAL_INT(9, invalid);
}

// ---- lifecycle ----

// Feed the recorded event order into the connection state machine: must
// emit re-init exactly once per `79 02` (8 in this capture), 9 disconnects,
// and nothing for the 0x42 flushes / 0x44 chatter.
static void test_lifecycle_reinit(void) {
    auto evs = load_lifecycle_events();
    TEST_ASSERT_EQUAL_size_t(50, evs.size());
    sc::ConnMonitor mon;
    int reinit = 0, lost = 0;
    for (auto& ev : evs) {
        TEST_ASSERT_FALSE(ev.raw.empty());
        TEST_ASSERT_EQUAL_INT(ev.id, ev.raw[0]);
        sc::ConnEvent e = mon.feed(ev.raw[0], ev.raw.data() + 1,
                                   ev.raw.size() - 1);
        reinit += e == sc::ConnEvent::Connected;
        lost += e == sc::ConnEvent::Disconnected;
        if (ev.id != 0x79)
            TEST_ASSERT_TRUE(e == sc::ConnEvent::None);
    }
    TEST_ASSERT_EQUAL_INT(8, reinit);
    TEST_ASSERT_EQUAL_INT(9, lost);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_imu_known_values);
    RUN_TEST(test_imu_neutral_gravity);
    RUN_TEST(test_imu_motion_scaling);
    RUN_TEST(test_imu_resampler);
    RUN_TEST(test_imu_resampler_rest_seed);
    RUN_TEST(test_stick_axis_known_values);
    RUN_TEST(test_sticks_circles_extremes);
    RUN_TEST(test_sticks_neutral_centered);
    RUN_TEST(test_button_table);
    RUN_TEST(test_trigger_threshold);
    RUN_TEST(test_buttons_fixture_order);
    RUN_TEST(test_bumpers_translate);
    RUN_TEST(test_battery_level_mapping);
    RUN_TEST(test_battery_no_data_records);
    RUN_TEST(test_lifecycle_reinit);
    return UNITY_END();
}
