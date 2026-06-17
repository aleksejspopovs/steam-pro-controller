// Rig 1, part 1: the 14 checks of tools/36_validate_fixtures.py ported 1:1
// against the same fixture files. Same names, same assertions.
#include <unity.h>

#include "../common/fixtures.hpp"

using sc::State;

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

static void test_neutral(void) {
    auto ns = load45("neutral");
    TEST_ASSERT_FALSE(ns.empty());
    uint32_t allbtn = 0;
    int maxstick = 0;
    double az = 0;
    for (auto& s : ns) {
        allbtn |= s.buttons;
        for (int v : {s.lstick[0], s.lstick[1], s.rstick[0], s.rstick[1]}) {
            int a = v < 0 ? -v : v;
            if (a > maxstick) maxstick = a;
        }
        az += s.accel[2];
    }
    az /= (double)ns.size();
    TEST_ASSERT_EQUAL_UINT32(0, allbtn);
    TEST_ASSERT_LESS_THAN_INT(1500, maxstick);
    TEST_ASSERT_TRUE(15000 < az && az < 18000);
}

static void check_press_order(const char* name,
                              const std::vector<uint32_t>& expect) {
    auto order = press_order(load45(name), expect);
    TEST_ASSERT_EQUAL_size_t(expect.size(), order.size());
    for (size_t i = 0; i < expect.size(); i++)
        TEST_ASSERT_EQUAL_UINT32(expect[i], order[i]);
}

static void test_buttons_abxy(void) {
    check_press_order("buttons_abxy",
                      {sc::BTN_A, sc::BTN_B, sc::BTN_X, sc::BTN_Y});
}

static void test_dpad(void) {
    check_press_order("dpad", {sc::BTN_DPAD_UP, sc::BTN_DPAD_RIGHT,
                               sc::BTN_DPAD_DOWN, sc::BTN_DPAD_LEFT});
}

static void test_menu_cluster(void) {
    check_press_order("menu_cluster",
                      {sc::BTN_VIEW, sc::BTN_MENU, sc::BTN_STEAM, sc::BTN_QAM});
}

static void test_grips(void) {
    check_press_order("grips", {sc::BTN_GRIP_L4, sc::BTN_GRIP_L5,
                                sc::BTN_GRIP_R4, sc::BTN_GRIP_R5});
}

static void test_bumpers(void) {
    uint32_t got = 0;
    for (auto& s : load45("bumpers"))
        got |= s.buttons;
    TEST_ASSERT_EQUAL_UINT32(sc::BTN_L1 | sc::BTN_R1,
                             got & (sc::BTN_L1 | sc::BTN_R1));
}

static void test_triggers_slow(void) {
    auto ts = load45("triggers_slow");
    TEST_ASSERT_FALSE(ts.empty());
    uint16_t lmax = 0, rmax = 0;
    bool lmid = false, rmid = false, l2 = false, r2 = false;
    for (auto& s : ts) {
        if (s.ltrig > lmax) lmax = s.ltrig;
        if (s.rtrig > rmax) rmax = s.rtrig;
        lmid |= (8000 < s.ltrig && s.ltrig < 24000);
        rmid |= (8000 < s.rtrig && s.rtrig < 24000);
        l2 |= (s.buttons & sc::BTN_L2) != 0;
        r2 |= (s.buttons & sc::BTN_R2) != 0;
    }
    TEST_ASSERT_EQUAL_UINT16(32767, lmax);
    TEST_ASSERT_EQUAL_UINT16(32767, rmax);
    TEST_ASSERT_TRUE(lmid && rmid && l2 && r2);
}

static void test_sticks_circles(void) {
    auto ss = load45("sticks_circles");
    TEST_ASSERT_FALSE(ss.empty());
    int mn[4] = {32767, 32767, 32767, 32767};
    int mx[4] = {-32768, -32768, -32768, -32768};
    for (auto& s : ss) {
        const int v[4] = {s.lstick[0], s.lstick[1], s.rstick[0], s.rstick[1]};
        for (int i = 0; i < 4; i++) {
            if (v[i] < mn[i]) mn[i] = v[i];
            if (v[i] > mx[i]) mx[i] = v[i];
        }
    }
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_LESS_OR_EQUAL_INT(-32000, mn[i]);
        TEST_ASSERT_GREATER_OR_EQUAL_INT(32000, mx[i]);
    }
}

static void test_stick_clicks(void) {
    uint32_t got = 0;
    for (auto& s : load45("stick_clicks"))
        got |= s.buttons;
    TEST_ASSERT_EQUAL_UINT32(sc::BTN_L3 | sc::BTN_R3,
                             got & (sc::BTN_L3 | sc::BTN_R3));
}

static void test_pads_swipe_click(void) {
    auto ps = load45("pads_swipe_click");
    TEST_ASSERT_FALSE(ps.empty());
    uint32_t got = 0;
    int lspan[2] = {0, 0}, rspan[2] = {0, 0}, maxp = 0;
    for (auto& s : ps) {
        got |= s.buttons;
        for (int i = 0; i < 2; i++) {
            int la = s.lpad[i] < 0 ? -s.lpad[i] : s.lpad[i];
            int ra = s.rpad[i] < 0 ? -s.rpad[i] : s.rpad[i];
            if (la > lspan[i]) lspan[i] = la;
            if (ra > rspan[i]) rspan[i] = ra;
        }
        if (s.lpad_pressure > maxp) maxp = s.lpad_pressure;
        if (s.rpad_pressure > maxp) maxp = s.rpad_pressure;
    }
    const uint32_t want = sc::BTN_LPAD_TOUCH | sc::BTN_RPAD_TOUCH |
                          sc::BTN_LPAD_CLICK | sc::BTN_RPAD_CLICK;
    TEST_ASSERT_EQUAL_UINT32(want, got & want);
    TEST_ASSERT_GREATER_THAN_INT(20000, lspan[0]);
    TEST_ASSERT_GREATER_THAN_INT(20000, lspan[1]);
    TEST_ASSERT_GREATER_THAN_INT(20000, rspan[0]);
    TEST_ASSERT_GREATER_THAN_INT(20000, rspan[1]);
    TEST_ASSERT_GREATER_THAN_INT(20000, maxp);
}

static void test_grips_sense(void) {
    uint32_t got = 0;
    for (auto& s : load45("grips"))
        got |= s.buttons;
    TEST_ASSERT_EQUAL_UINT32(sc::BTN_LGRIP_SENSE | sc::BTN_RGRIP_SENSE,
                             got & (sc::BTN_LGRIP_SENSE | sc::BTN_RGRIP_SENSE));
}

static void test_imu_motion(void) {
    auto im = load45("imu_motion");
    TEST_ASSERT_FALSE(im.empty());
    int gmax[3] = {0, 0, 0};
    int amin[3] = {32767, 32767, 32767}, amax[3] = {-32768, -32768, -32768};
    for (auto& s : im) {
        for (int i = 0; i < 3; i++) {
            int g = s.gyro[i] < 0 ? -s.gyro[i] : s.gyro[i];
            if (g > gmax[i]) gmax[i] = g;
            if (s.accel[i] < amin[i]) amin[i] = s.accel[i];
            if (s.accel[i] > amax[i]) amax[i] = s.accel[i];
        }
    }
    int gyro_axes = 0, span = 0;
    for (int i = 0; i < 3; i++) {
        gyro_axes += gmax[i] > 3000;
        if (amax[i] - amin[i] > span) span = amax[i] - amin[i];
    }
    TEST_ASSERT_GREATER_OR_EQUAL_INT(2, gyro_axes);
    TEST_ASSERT_GREATER_THAN_INT(20000, span);
}

// IMU off does NOT zero the fields -- it FREEZES them at the last sample
// (and the [30:33] us timestamp stops). "Off" = frozen, not zero.
static void test_rest_imu_off(void) {
    auto ro = load45("rest_imu_off");
    TEST_ASSERT_GREATER_THAN_size_t(1, ro.size());
    int changes = 0;
    for (size_t i = 1; i < ro.size(); i++) {
        bool same = true;
        for (int j = 0; j < 3; j++)
            same = same && ro[i - 1].accel[j] == ro[i].accel[j] &&
                   ro[i - 1].gyro[j] == ro[i].gyro[j];
        changes += !same;
    }
    // the us timestamp may advance a sample or two before the IMU stops
    // (one tick = 4032 us); a live capture would advance ~2.4e6 us here
    uint32_t adv_us = ro.back().imu_ts - ro.front().imu_ts;
    TEST_ASSERT_LESS_OR_EQUAL_INT(2, changes);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(2 * 4032, adv_us);
}

static void test_battery_parse(void) {
    auto recs = load_records("lifecycle_battery.raw43", sc::REPORT43_LEN);
    TEST_ASSERT_EQUAL_size_t(341, recs.size());
    for (auto& r : recs) {
        sc::Battery b;
        TEST_ASSERT_TRUE(sc::parse_43(r.data(), r.size(), b));
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_neutral);
    RUN_TEST(test_buttons_abxy);
    RUN_TEST(test_dpad);
    RUN_TEST(test_menu_cluster);
    RUN_TEST(test_grips);
    RUN_TEST(test_bumpers);
    RUN_TEST(test_triggers_slow);
    RUN_TEST(test_sticks_circles);
    RUN_TEST(test_stick_clicks);
    RUN_TEST(test_pads_swipe_click);
    RUN_TEST(test_grips_sense);
    RUN_TEST(test_imu_motion);
    RUN_TEST(test_rest_imu_off);
    RUN_TEST(test_battery_parse);
    return UNITY_END();
}
