// Fixture loading shared by the native test suites. Plain fread, fixed
// record sizes (46 B .raw45 / 15 B .raw43). FIXTURES_DIR comes from
// platformio.ini (tests run with CWD = adapter/).
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "sc_proto.h"

#ifndef FIXTURES_DIR
#define FIXTURES_DIR "../tools/fixtures"
#endif

inline std::string fixture_path(const char* file) {
    return std::string(FIXTURES_DIR) + "/" + file;
}

inline std::vector<std::vector<uint8_t>> load_records(const char* file,
                                                      size_t rec_len) {
    std::vector<std::vector<uint8_t>> out;
    FILE* f = fopen(fixture_path(file).c_str(), "rb");
    if (!f) return out;
    std::vector<uint8_t> rec(rec_len);
    while (fread(rec.data(), 1, rec_len, f) == rec_len)
        out.push_back(rec);
    fclose(f);
    return out;
}

inline std::vector<sc::State> load45(const char* name) {
    std::vector<sc::State> out;
    auto recs = load_records((std::string(name) + ".raw45").c_str(),
                             sc::REPORT45_LEN);
    for (auto& r : recs) {
        sc::State s;
        if (sc::parse_45(r.data(), r.size(), s))
            out.push_back(s);
    }
    return out;
}

// First-seen order of the given button masks across a fixture stream.
inline std::vector<uint32_t> press_order(const std::vector<sc::State>& states,
                                         const std::vector<uint32_t>& masks) {
    std::vector<uint32_t> seen;
    for (const auto& s : states) {
        for (uint32_t m : masks) {
            bool already = false;
            for (uint32_t v : seen) already |= (v == m);
            if (!already && (s.buttons & m))
                seen.push_back(m);
        }
    }
    return seen;
}

// Minimal lifecycle_events.jsonl reader: {"t": "...", "id": N, "raw": "hex"}.
struct LifecycleEvent {
    int id;
    std::vector<uint8_t> raw; // full report incl. id byte
};

inline std::vector<LifecycleEvent> load_lifecycle_events() {
    std::vector<LifecycleEvent> out;
    FILE* f = fopen(fixture_path("lifecycle_events.jsonl").c_str(), "r");
    if (!f) return out;
    char line[1024];
    while (fgets(line, sizeof(line), f)) {
        const char* idp = strstr(line, "\"id\":");
        const char* rawp = strstr(line, "\"raw\":");
        if (!idp || !rawp) continue;
        LifecycleEvent ev;
        ev.id = atoi(idp + 5);
        const char* h = strchr(rawp + 6, '"');
        if (!h) continue;
        h++;
        while (h[0] && h[1] && h[0] != '"') {
            unsigned b;
            if (sscanf(h, "%2x", &b) != 1) break;
            ev.raw.push_back((uint8_t)b);
            h += 2;
        }
        out.push_back(ev);
    }
    fclose(f);
    return out;
}
