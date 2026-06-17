#pragma once

#include <cstddef>
#include <cstdint>

namespace pro_device {

using OutputFn = void (*)(const uint8_t* data, size_t len);

void begin(OutputFn on_output);
void task();
bool mounted();
bool queue_report(const uint8_t* data, size_t len);

// Connection lifecycle (D+ pullup). begin() leaves us disconnected so the
// Switch never enumerates the adapter until a Steam Controller is live on the
// host port; call connect() to present the Pro Controller and disconnect() to
// drop the link (the Switch sees an unplug). Both are idempotent.
void connect();
void disconnect();
bool linked(); // our intended link state (pullup enabled)

} // namespace pro_device
