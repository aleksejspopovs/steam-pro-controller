#include "pro_device.h"

#include <Arduino.h>
#include <cstring>
#include <tusb.h>

#include "crashlog.h"
#include "pro.h"

extern "C" void usb_isr();

namespace pro_device {
namespace {

OutputFn g_on_output = nullptr;
bool g_link_up = false; // intended D+ pullup state (see connect/disconnect)

constexpr size_t TX_COUNT = 16;
uint8_t g_tx[TX_COUNT][pro::REPORT_LEN];
size_t g_tx_head = 0;
size_t g_tx_count = 0;

void drain_tx() {
    while (g_tx_count > 0 && tud_hid_ready()) {
        uint8_t const* report = g_tx[g_tx_head];
        if (!tud_hid_report(report[0], report + 1, pro::REPORT_LEN - 1))
            break;
        g_tx_head = (g_tx_head + 1) % TX_COUNT;
        --g_tx_count;
    }
}

void init_usb_hardware() {
    PMU_REG_3P0 = PMU_REG_3P0_OUTPUT_TRG(0x0F) | PMU_REG_3P0_BO_OFFSET(6) |
                   PMU_REG_3P0_ENABLE_LINREG;
    CCM_CCGR6 |= CCM_CCGR6_USBOH3(CCM_CCGR_ON);
    USB1_BURSTSIZE = 0x0404;

    // A Teensy bootloader or previous sketch may leave USB1 active. Reset the
    // controller and PHY before handing ownership to TinyUSB.
    USBPHY1_CTRL_SET = USBPHY_CTRL_SFTRST;
    USB1_USBCMD |= USB_USBCMD_RST;
    while (USB1_USBCMD & USB_USBCMD_RST) {}
    NVIC_CLEAR_PENDING(IRQ_USB1);
    USBPHY1_CTRL_CLR = USBPHY_CTRL_SFTRST | USBPHY_CTRL_CLKGATE;
    USBPHY1_PWD = 0;

    attachInterruptVector(IRQ_USB1, ::usb_isr);
}

} // namespace

void begin(OutputFn on_output) {
    g_on_output = on_output;
    init_usb_hardware();

    tusb_rhport_init_t init = {};
    init.role = TUSB_ROLE_DEVICE;
    init.speed = TUSB_SPEED_FULL;
    tusb_init(0, &init);

    // Stay off the bus until a Steam Controller is live: tusb_init() enables the
    // pullup, so explicitly drop it here. The Switch won't enumerate us until
    // connect() is called (main.cpp lifecycle).
    tud_disconnect();
    g_link_up = false;
}

void task() {
    tud_task();
    drain_tx();
}

bool mounted() {
    return tud_mounted();
}

void connect() {
    if (g_link_up)
        return;
    g_link_up = true;
    tud_connect();
    crashlog::line("usb:link up -> present Pro Controller to Switch");
}

void disconnect() {
    if (!g_link_up)
        return;
    g_link_up = false;
    g_tx_head = 0;
    g_tx_count = 0; // drop reports queued for the old session
    tud_disconnect();
    crashlog::line("usb:link down -> hidden from Switch");
}

bool linked() {
    return g_link_up;
}

bool queue_report(const uint8_t* data, size_t len) {
    if (len > pro::REPORT_LEN || !mounted() || g_tx_count == TX_COUNT)
        return false;

    const size_t tail = (g_tx_head + g_tx_count) % TX_COUNT;
    memset(g_tx[tail], 0, pro::REPORT_LEN);
    memcpy(g_tx[tail], data, len);
    ++g_tx_count;
    drain_tx();
    return true;
}

} // namespace pro_device

extern "C" void usb_isr() {
    tusb_int_handler(0, true);
}

// USB lifecycle. The suspend/resume pair is unhandled in firmware today even
// though the descriptor advertises remote wakeup; logging it tells us if the
// Switch suspends the port mid-pair (a candidate for the fault).
extern "C" void tud_mount_cb() {
    crashlog::line("usb:mount (configured)");
}

extern "C" void tud_umount_cb() {
    crashlog::line("usb:umount");
}

extern "C" void tud_suspend_cb(bool remote_wakeup_en) {
    crashlog::kv("usb:suspend remote_wakeup_en", remote_wakeup_en);
}

extern "C" void tud_resume_cb() {
    crashlog::line("usb:resume");
}

// Fires on every successful interrupt-IN to the host (~66x/s while streaming):
// our truest "host is still polling us" heartbeat. Bump activity, don't log.
extern "C" void tud_hid_report_complete_cb(uint8_t, uint8_t const*, uint16_t) {
    crashlog::mark_activity();
}

extern "C" uint16_t tud_hid_get_report_cb(uint8_t, uint8_t report_id,
                                            hid_report_type_t report_type,
                                            uint8_t*, uint16_t reqlen) {
    // We answer 0 (STALL). If the Switch issues a GET_REPORT Linux never does,
    // this is where it would wedge.
    crashlog::kv("hid:get_report id", report_id);
    crashlog::kv("  type", report_type);
    crashlog::kv("  reqlen", reqlen);
    return 0;
}

extern "C" void tud_hid_set_report_cb(uint8_t, uint8_t report_id,
                                        hid_report_type_t,
                                        uint8_t const* buffer,
                                        uint16_t size) {
    if (!size || !pro_device::g_on_output)
        return;

    if (report_id == 0) {
        pro_device::g_on_output(buffer, size);
        return;
    }

    uint8_t report[pro::REPORT_LEN] = {};
    report[0] = report_id;
    const size_t copy = size < pro::REPORT_LEN - 1 ? size : pro::REPORT_LEN - 1;
    memcpy(report + 1, buffer, copy);
    pro_device::g_on_output(report, copy + 1);
}
