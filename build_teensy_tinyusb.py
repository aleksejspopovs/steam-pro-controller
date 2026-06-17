"""Build the upstream TinyUSB device stack for Teensy 4.1.

PlatformIO treats TinyUSB as a generic library and would otherwise compile
every MCU/host backend. Select only the device core, HID class, and official
ChipIdea HS i.MX RT device-controller driver used by this firmware.
"""

Import("env")

from pathlib import Path


project = Path(env["PROJECT_DIR"])
tinyusb = Path(env["PROJECT_LIBDEPS_DIR"]) / env["PIOENV"] / "TinyUSB"
if not tinyusb.exists():
    raise RuntimeError(f"TinyUSB dependency not installed at {tinyusb}")

env.AppendUnique(CPPPATH=[
    str(project / "src" / "teensy"),
    str(tinyusb / "src"),
])

tinyusb_lib = env.BuildLibrary(
    str(Path(env.subst("$BUILD_DIR")) / "TinyUSBDevice"),
    str(tinyusb),
    src_filter=[
        "+<src/tusb.c>",
        "+<src/common/tusb_fifo.c>",
        "+<src/device/usbd.c>",
        "+<src/device/usbd_control.c>",
        "+<src/class/hid/hid_device.c>",
        "+<src/portable/chipidea/ci_hs/dcd_ci_hs.c>",
    ],
)
env.Prepend(LIBS=[tinyusb_lib])
