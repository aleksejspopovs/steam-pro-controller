# Steam/Pro Controller

_The contents of this repo are not authored by, endorsed by, or in any way affiliated with Nintendo or Valve. Use at yor own risk._

This is a hardware adapter that allows a Steam Controller (2026, codename Triton) to be used with a Nintendo Switch, by emulating a wired Switch Pro Controller.

It supports accel/gyro and HD rumble.

It does not currently make any use of the capacitive touch pads, grips, and sticks. It cannot emulate the GL/GR buttons or the C button, because it emulates the orginal Pro Controller, not the Switch 2 Pro Controller. Likewise, it cannot emulate the Switch 2 Joy-Con mouse functionality.

## Hardware

You'll need:

- a Steam Controller
- a Steam Controller Puck
- a [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) with a [USB host cable](https://www.pjrc.com/store/cable_usb_host_t36.html) (or any other way to connect the Puck to the USB host port the Teensy)

## Build and install

`pio run -e teensy41 -t upload`

## Attribution and references

The code in this repo is entirely AI-generated.

Thanks to:

- Nintendo and Valve for making great hardware;
- [dekuNukem](https://github.com/dekuNukem/) for reverse engineering most of the Switch controller protocol in [dekuNukem/Nintendo_Switch_Reverse_Engineering](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering);
- [ndeadly](https://github.com/ndeadly/) for reverse engineering the [rumble structs](https://github.com/ndeadly/MissionControl/blob/master/mc_mitm/source/controllers/switch_rumble_decoder.hpp) and the newer [IMU mode 2 structs](https://github.com/ndeadly/MissionControl/blob/master/mc_mitm/source/controllers/switch_motion_packing.hpp) in [ndeadly/MissionControl](https://github.com/ndeadly/MissionControl/);
- [SDL](https://github.com/libsdl-org/SDL) contributors for [implementing](https://github.com/libsdl-org/SDL/blob/main/src/joystick/hidapi/SDL_hidapi_steam_triton.c) the Steam Controller protocol;
- Linux kernel contributors for implementing [hid-steam](https://github.com/torvalds/linux/blob/master/drivers/hid/hid-steam.c) and [hid-nintendo](https://github.com/torvalds/linux/blob/master/drivers/hid/hid-nintendo.c) kernel modules, which served as useful references and testbeds.
