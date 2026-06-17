#include <tusb.h>

#include "pro_descriptor.h"

namespace {

constexpr tusb_desc_device_t DEVICE = {
    sizeof(tusb_desc_device_t),
    TUSB_DESC_DEVICE,
    0x0200,
    0x00,
    0x00,
    0x00,
    64,
    pro::kVendorId,
    pro::kProductId,
    pro::kBcdDevice,
    1,
    2,
    3,
    1,
};

// Exact genuine Pro Controller configuration descriptor captured locally.
constexpr uint8_t CONFIG[] = {
    9, TUSB_DESC_CONFIGURATION, 41, 0, 1, 1, 0, 0xA0, 250,
    9, TUSB_DESC_INTERFACE, 0, 0, 2, TUSB_CLASS_HID, 0, 0, 0,
    9, HID_DESC_TYPE_HID, 0x11, 0x01, 0, 1, HID_DESC_TYPE_REPORT,
       sizeof(pro::REPORT_DESCRIPTOR) & 0xFF, sizeof(pro::REPORT_DESCRIPTOR) >> 8,
    7, TUSB_DESC_ENDPOINT, 0x81, TUSB_XFER_INTERRUPT, 64, 0, 8,
    7, TUSB_DESC_ENDPOINT, 0x01, TUSB_XFER_INTERRUPT, 64, 0, 8,
};

constexpr char const* STRINGS[] = {
    nullptr,
    "Nintendo Co., Ltd.",
    "Pro Controller",
    "000000000001",
};

uint16_t string_buf[32];

} // namespace

extern "C" uint8_t const* tud_descriptor_device_cb() {
    return reinterpret_cast<uint8_t const*>(&DEVICE);
}

extern "C" uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    return CONFIG;
}

extern "C" uint8_t const* tud_hid_descriptor_report_cb(uint8_t) {
    return pro::REPORT_DESCRIPTOR;
}

extern "C" uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    uint8_t count = 0;
    if (index == 0) {
        string_buf[1] = 0x0409;
        count = 1;
    } else {
        if (index >= sizeof(STRINGS) / sizeof(STRINGS[0])) {
            return nullptr;
        }
        char const* s = STRINGS[index];
        while (s[count] && count < 31) {
            string_buf[1 + count] = static_cast<uint8_t>(s[count]);
            ++count;
        }
    }
    string_buf[0] = static_cast<uint16_t>((TUSB_DESC_STRING << 8) | (2 * count + 2));
    return string_buf;
}
