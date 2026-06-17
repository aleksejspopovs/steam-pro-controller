# Vendor a patched copy of USBHost_t36 into the project's lib/ instead of
# editing the shared Teensy framework package in place.
#
# Why patch at all: the Steam Controller puck's *application* firmware
# (28de:1304) STALLs USBHost_t36's Language-ID string request (enumeration
# state 3, a GET_DESCRIPTOR string with wLength = sizeof(enumbuf)-4 = 2044).
# USBHost_t36 treats that as a fatal enumeration error and retries forever, so
# the device never reaches claim() and our driver never sees its HID
# interfaces. (The bootloader 28de:1007 answers the same request, and Linux
# tolerates string failures -- so it's the app firmware + USBHost_t36's no-skip
# behavior.) String descriptors are purely cosmetic (manufacturer()/product()
# accessors); we bind by VID/PID. So skip them: after the device descriptor
# (state 2) jump straight to the config descriptor (state 7).
#
# Why a project-local copy: the previous version of this script rewrote
# enumeration.cpp directly under ~/.platformio/packages, mutating global state
# shared by every PlatformIO project on the machine (and changing string
# enumeration for ALL host devices, not just ours). Instead we copy the library
# out of the framework package into lib/USBHost_t36/, which PlatformIO's LDF
# resolves with higher priority than the framework's LIBSOURCE_DIRS -- so the
# patched copy transparently shadows the bundled one, no lib_ignore needed.
#
# The copy is gitignored and re-created on demand. A stamp file records the
# framework version + patch marker it was built from; if that still matches the
# installed framework, we skip the copy+patch entirely (cheap no-op rebuild).
# Bump the framework and the stamp goes stale, so the copy is refreshed.
import glob
import json
import os
import shutil

Import("env")  # noqa: F821  (injected by PlatformIO)

MARKER = "PATCH_PUCK_SKIPSTR"
TARGET = "\t\t\tdev->enum_state = 3;\n"
REPLACEMENT = (
    "\t\t\tdev->enum_state = 7; /* " + MARKER + ": app STALLs langid "
    "(wLength 2044) and USBHost_t36 retries forever; strings are cosmetic, "
    "skip straight to config so enumeration reaches claim() */\n"
)
STAMP_NAME = ".vendor_stamp"

pkgdir = env.subst("$PROJECT_PACKAGES_DIR")  # noqa: F821
projdir = env.subst("$PROJECT_DIR")  # noqa: F821
dest = os.path.join(projdir, "lib", "USBHost_t36")


def framework_version(fw_dir):
    try:
        with open(os.path.join(fw_dir, "package.json")) as f:
            return json.load(f).get("version", "unknown")
    except (OSError, ValueError):
        return "unknown"


srcs = glob.glob(
    os.path.join(pkgdir, "framework-arduinoteensy*", "libraries", "USBHost_t36")
)
if not srcs:
    raise RuntimeError(
        "vendor_usbhost_t36: USBHost_t36 not found under " + pkgdir
    )
src = srcs[0]
fw_dir = os.path.dirname(os.path.dirname(src))
stamp = "framework-arduinoteensy {} {}\n".format(framework_version(fw_dir), MARKER)

stamp_path = os.path.join(dest, STAMP_NAME)
current = None
if os.path.isfile(stamp_path):
    with open(stamp_path) as f:
        current = f.read()

if current == stamp:
    print("vendor_usbhost_t36: up to date:", dest)
else:
    print("vendor_usbhost_t36: vendoring patched USBHost_t36 ->", dest)
    if os.path.isdir(dest):
        shutil.rmtree(dest)
    shutil.copytree(src, dest)

    enum_path = os.path.join(dest, "enumeration.cpp")
    with open(enum_path) as f:
        text = f.read()
    if MARKER in text:
        pass  # already patched upstream somehow; leave it
    elif TARGET not in text:
        raise RuntimeError(
            "vendor_usbhost_t36: patch target not found in " + enum_path
            + " (USBHost_t36 enumeration changed upstream?)"
        )
    else:
        with open(enum_path, "w") as f:
            f.write(text.replace(TARGET, REPLACEMENT, 1))

    with open(stamp_path, "w") as f:
        f.write(stamp)
    print("vendor_usbhost_t36: done")
