// Pro Controller USB HID report descriptor, from dekuNukem's
// Nintendo_Switch_Reverse_Engineering (USB-HID-Notes). hid-nintendo binds
// hidraw-only and parses none of the axes, but the report IDs (in 0x30,
// 0x21, 0x81; out 0x01, 0x10, 0x80, 0x82) must be declared.
#pragma once
#include <cstdint>

namespace pro {

// Named with a k-prefix on purpose: the bare USB_VENDOR/USB_PRODUCT/
// USB_VERSION tokens are #defined as string macros by some Arduino cores
// (e.g. arduino-pico), and the preprocessor would clobber these before the
// `pro::` namespace ever applies.
constexpr uint16_t kVendorId = 0x057e;
constexpr uint16_t kProductId = 0x2009; // Pro Controller
constexpr uint16_t kBcdDevice = 0x0200;

inline constexpr uint8_t REPORT_DESCRIPTOR[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x15, 0x00,        // Logical Minimum (0)
    0x09, 0x04,        // Usage (Joystick)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x30,        //   Report ID (0x30)
    0x05, 0x01,        //   Usage Page (Generic Desktop)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (1)
    0x29, 0x0A,        //   Usage Maximum (10)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x0A,        //   Report Count (10)
    0x55, 0x00,        //   Unit Exponent (0)
    0x65, 0x00,        //   Unit (None)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x0B,        //   Usage Minimum (11)
    0x29, 0x0E,        //   Usage Maximum (14)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x03,        //   Input (Const,Var,Abs)
    0x0B, 0x01, 0x00, 0x01, 0x00, //   Usage (Generic Desktop:Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x0B, 0x30, 0x00, 0x01, 0x00, //     Usage (Generic Desktop:X)
    0x0B, 0x31, 0x00, 0x01, 0x00, //     Usage (Generic Desktop:Y)
    0x0B, 0x32, 0x00, 0x01, 0x00, //     Usage (Generic Desktop:Z)
    0x0B, 0x35, 0x00, 0x01, 0x00, //     Usage (Generic Desktop:Rz)
    0x15, 0x00,        //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00, //     Logical Maximum (65534)
    0x75, 0x10,        //     Report Size (16)
    0x95, 0x04,        //     Report Count (4)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0xC0,              //   End Collection
    0x0B, 0x39, 0x00, 0x01, 0x00, //   Usage (Generic Desktop:Hat Switch)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x07,        //   Logical Maximum (7)
    0x35, 0x00,        //   Physical Minimum (0)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x65, 0x14,        //   Unit (Eng Rot: Degree)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x0F,        //   Usage Minimum (15)
    0x29, 0x12,        //   Usage Maximum (18)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x34,        //   Report Count (52)
    0x81, 0x03,        //   Input (Const,Var,Abs)
    0x06, 0x00, 0xFF,  //   Usage Page (Vendor Defined)
    0x85, 0x21,        //   Report ID (0x21)
    0x09, 0x01,        //   Usage (1)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x3F,        //   Report Count (63)
    0x81, 0x03,        //   Input (Const,Var,Abs)
    0x85, 0x81,        //   Report ID (0x81)
    0x09, 0x02,        //   Usage (2)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x3F,        //   Report Count (63)
    0x81, 0x03,        //   Input (Const,Var,Abs)
    0x85, 0x01,        //   Report ID (0x01)
    0x09, 0x03,        //   Usage (3)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x3F,        //   Report Count (63)
    0x91, 0x83,        //   Output (Const,Var,Abs,Vol)
    0x85, 0x10,        //   Report ID (0x10)
    0x09, 0x04,        //   Usage (4)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x3F,        //   Report Count (63)
    0x91, 0x83,        //   Output (Const,Var,Abs,Vol)
    0x85, 0x80,        //   Report ID (0x80)
    0x09, 0x05,        //   Usage (5)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x3F,        //   Report Count (63)
    0x91, 0x83,        //   Output (Const,Var,Abs,Vol)
    0x85, 0x82,        //   Report ID (0x82)
    0x09, 0x06,        //   Usage (6)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x3F,        //   Report Count (63)
    0x91, 0x83,        //   Output (Const,Var,Abs,Vol)
    0xC0,              // End Collection
};

} // namespace pro
