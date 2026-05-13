#pragma once
// HITACHI AC power frames captured from the user's original remote via the
// recorder env (build with -DBUDDY_IR_RECORDER=1). HITACHI AC isn't a
// toggle — each press encodes the full state (power + mode + temp + fan
// + ...). Pressing POWER while the AC is OFF yields one frame; pressing
// it while the AC is ON yields a different frame. We store both.
//
// Arrays are declared `extern const uint16_t[]`; the definitions live in
// hitachi_ac_codes.cpp. On ESP32 const arrays land in .rodata which is
// directly addressable from flash — no AVR-style PROGMEM needed.
//
// The whole AC remote module is gated behind BUDDY_HAS_HITACHI_AC. This
// header is safe to include unconditionally (declarations only); callers
// must still guard their usage.

#include <stdint.h>
#include <stddef.h>     // size_t — .cpp does not get this implicitly the way .ino does

constexpr uint16_t HITACHI_AC_CARRIER_HZ = 38;

extern const uint16_t HITACHI_AC_POWER_ON_RAW[];
extern const size_t   HITACHI_AC_POWER_ON_LEN;

extern const uint16_t HITACHI_AC_POWER_OFF_RAW[];
extern const size_t   HITACHI_AC_POWER_OFF_LEN;
