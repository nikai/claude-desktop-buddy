#pragma once
// HITACHI AC power-toggle IR remote. Pure logic; no UI side-effects —
// callers (main.cpp) are responsible for beep / toast / serial UX. All
// declarations gated by BUDDY_HAS_HITACHI_AC so non-StickS3 builds
// (which don't have IRremoteESP8266 in lib_deps) won't see them.

#ifdef BUDDY_HAS_HITACHI_AC

void hitachiAcInit();              // Call after M5.begin(). Powers EXT_5V, sets up IRsend on GPIO46, reads last state from NVS.
bool hitachiAcSendOn();            // Force-send the POWER_ON frame. Returns true. Updates NVS.
bool hitachiAcSendOff();           // Force-send the POWER_OFF frame. Returns false. Updates NVS.
bool hitachiAcLastState();         // Last persisted state (true = we last sent ON).
bool hitachiAcToggleAndSend();     // Flip persisted state and send matching raw. Returns the new state.

#endif  // BUDDY_HAS_HITACHI_AC
