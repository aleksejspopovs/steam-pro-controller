#include "crashlog.h"

#include <Arduino.h>
#include <SD.h>

namespace crashlog {
namespace {

constexpr size_t BUF_SIZE = 256 * 1024; // headroom for verbose USBHost trace
constexpr uint32_t IDLE_MS = 30000;  // host quiet this long => assume crash
constexpr uint32_t CEILING_MS = 120000; // hard upper bound from first activity
const char* const LOG_PATH = "crash.log";

// OCRAM-resident so the 64 KB does not crowd the fast DTCM. Not zero-init'd by
// the C runtime (DMAMEM is .noinit); g_len gates what is valid.
DMAMEM char g_buf[BUF_SIZE];
volatile uint32_t g_len = 0;
volatile bool g_trunc = false;

volatile uint32_t g_first_ms = 0;
volatile uint32_t g_last_ms = 0;
volatile bool g_seen = false;
bool g_flushed = false;
bool g_sd_ok = false;
uint32_t g_flush_offset = 0; // bytes already written; enables incremental flush

// PRIMASK save/restore: correct whether or not we are already in an ISR or a
// nested-disabled context (plain __enable_irq() would wrongly re-enable).
// Inline asm avoids a CMSIS header dependency.
struct Crit {
    uint32_t pm;
    Crit() {
        __asm__ volatile("MRS %0, primask" : "=r"(pm));
        __asm__ volatile("CPSID i" ::: "memory");
    }
    ~Crit() { __asm__ volatile("MSR primask, %0" ::"r"(pm) : "memory"); }
};

inline void put(char c) {
    if (g_len < BUF_SIZE)
        g_buf[g_len++] = c;
    else
        g_trunc = true;
}

void put_str(const char* s) {
    while (*s)
        put(*s++);
}

void put_u32(uint32_t v) {
    if (v == 0) {
        put('0');
        return;
    }
    char tmp[10];
    int n = 0;
    while (v) {
        tmp[n++] = static_cast<char>('0' + (v % 10));
        v /= 10;
    }
    while (n)
        put(tmp[--n]);
}

void put_hex8(uint8_t b) {
    static const char* hex = "0123456789abcdef";
    put(hex[b >> 4]);
    put(hex[b & 0x0F]);
}

void stamp() {
    put('[');
    put_u32(micros());
    put_str("] ");
}

// Live UART mirror of a just-appended record (g_buf[start..end)). Pin 1 =
// Serial1 TX, 115200 8N1 (begun in setup()). NON-BLOCKING: writes only what
// fits in the TX FIFO right now and drops the rest, so it never spins with
// interrupts off -- safe from the USB ISR. Build with -DLOG_UART; the SD path
// is unaffected. High-volume USBHost trace can be truncated under load; the
// low-rate diagnostic records are not.
void uart_tee(uint32_t start, uint32_t end) {
#ifdef LOG_UART
    if (end <= start) return;
    int avail = Serial1.availableForWrite();
    if (avail <= 0) return;
    uint32_t n = end - start;
    if (n > (uint32_t)avail) n = (uint32_t)avail;
    Serial1.write(reinterpret_cast<const uint8_t*>(g_buf) + start, n);
#else
    (void)start;
    (void)end;
#endif
}

} // namespace

void mark_activity() {
    const uint32_t now = millis();
    if (!g_seen) {
        g_seen = true;
        g_first_ms = now;
    }
    g_last_ms = now;
}

void line(const char* tag) {
    const uint32_t s = g_len;
    {
        Crit c;
        stamp();
        put_str(tag);
        put('\r');
        put('\n');
    }
    uart_tee(s, g_len);
    mark_activity();
}

void kv(const char* tag, uint32_t value) {
    const uint32_t s = g_len;
    {
        Crit c;
        stamp();
        put_str(tag);
        put('=');
        put_u32(value);
        put('\r');
        put('\n');
    }
    uart_tee(s, g_len);
    mark_activity();
}

void bytes(const char* tag, const uint8_t* data, size_t n) {
    const uint32_t s = g_len;
    {
        Crit c;
        stamp();
        put_str(tag);
        put(' ');
        for (size_t i = 0; i < n; ++i) {
            put_hex8(data[i]);
            put(' ');
        }
        put('\r');
        put('\n');
    }
    uart_tee(s, g_len);
    mark_activity();
}

void init() {
    g_len = 0;
    g_trunc = false;
    g_flushed = false;
    g_seen = false;

#ifdef LOG_UART
    // Bigger Serial1 TX ring so a full hex-dump record fits and the
    // non-blocking uart_tee writes it whole instead of clipping at the
    // default 64-byte FIFO. (Serial1.begin() ran in setup() before this.)
    static uint8_t uart_tx[1024];
    Serial1.addMemoryForWrite(uart_tx, sizeof(uart_tx));
#endif

    g_sd_ok = SD.begin(BUILTIN_SDCARD);
    if (g_sd_ok) {
        SD.remove(LOG_PATH); // fresh log every boot
        File f = SD.open(LOG_PATH, FILE_WRITE);
        if (f) {
            f.println("=== boot ===");
            if (CrashReport) // a fault from the previous run, if any
                f.print(CrashReport);
            f.close();
        } else {
            g_sd_ok = false;
        }
    }
    line(g_sd_ok ? "sd ok" : "sd FAIL");
}

bool should_flush(uint32_t now_ms) {
    if (g_flushed || !g_seen)
        return false;
    return (now_ms - g_last_ms > IDLE_MS) || (now_ms - g_first_ms > CEILING_MS);
}

// Incremental: appends only bytes logged since the last flush, so it can be
// called at several checkpoints (init attempt, 0x45 dump, verify) without
// losing or duplicating anything. The idle/ceiling auto-flush is still gated
// one-shot via should_flush(); explicit checkpoint flushes are not.
void flush(const char* reason) {
    if (!g_sd_ok)
        return;

    File f = SD.open(LOG_PATH, FILE_WRITE); // FILE_WRITE appends on Teensy SD
    if (f) {
        if (g_flush_offset == 0)
            f.println("=== log ===");
        const uint32_t len = g_len; // snapshot (ISR keeps appending)
        if (len > g_flush_offset)
            f.write(reinterpret_cast<const uint8_t*>(g_buf) + g_flush_offset,
                    len - g_flush_offset);
        if (g_trunc)
            f.println("\n*** BUFFER TRUNCATED ***");
        f.print("=== flush: ");
        f.print(reason);
        f.println(" ===");
        f.close();
        g_flush_offset = len;
    }
    g_flushed = true;
}

bool flushed() {
    return g_flushed;
}

} // namespace crashlog
