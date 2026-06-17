#pragma once

// Teensyduino 1.60's yield.cpp references serialEvent even for USB_DISABLED,
// while Arduino.h omits its declaration in that mode.
#ifdef __cplusplus
void serialEvent();
#endif
