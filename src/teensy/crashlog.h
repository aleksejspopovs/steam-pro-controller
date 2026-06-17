#pragma once

// On-Teensy crash logger for the real-Switch bring-up (no UART available).
//
// The Switch faults our device with a generic USB error and parks on its
// "press power to turn off" screen WITHOUT cutting VBUS, so the Teensy keeps
// running on bus power. We therefore log every USB milestone into a RAM
// buffer during the timing-critical enumeration/handshake window (these calls
// run in the USB ISR and must never block), then flush the whole buffer to the
// built-in microSD once the host goes quiet. main.cpp signals "safe to power
// off" on the LED after the flush completes.
//
// init() MUST be called before USB is brought up (pro_device::begin): the SD
// card init can block for hundreds of ms, and doing it before D+ is pulled up
// just looks like a device that took a moment to appear, instead of a stalled
// tud_task() during the first control transfers.

#include <cstddef>
#include <cstdint>

namespace crashlog {

// SD + open log file + dump any prior-boot CrashReport. Call before USB init.
void init();

// Append a record to the RAM buffer. ISR-safe (brief PRIMASK critical
// section, no allocation, no blocking). Each call also marks host activity.
void line(const char* tag);
void kv(const char* tag, uint32_t value);
void bytes(const char* tag, const uint8_t* data, size_t n);

// Bump the host-liveness timestamp without logging (for the IN-transfer
// completion heartbeat, which fires ~66x/s and would flood the buffer).
void mark_activity();

// True once the host has been quiet past the idle window, or the hard ceiling
// since first activity has elapsed. False before any activity or after flush.
bool should_flush(uint32_t now_ms);

// Write the RAM buffer to microSD and latch flushed(). No-op if SD failed or
// already flushed. Safe to call from loop() (host is idle by definition here).
void flush(const char* reason);

bool flushed();

} // namespace crashlog
