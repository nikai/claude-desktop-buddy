// HITACHI AC remote — see hitachi_ac_remote.h.
//
// **No recording**: this file uses IRremoteESP8266's HITACHI AC protocol
// encoder (`IRHitachiAc`) to synthesize full-state IR frames directly,
// bypassing the broken-on-StickS3 internal IR receiver entirely. HITACHI
// AC remotes don't use a "power toggle" code — every press is a full
// state frame (power + mode + temp + fan + ...). We pick sensible
// MVP defaults: cool / 25°C / auto fan, and flip just the power bit.
//
// HITACHI variant: defaults to `IRHitachiAc` (the 28-byte / 224-bit
// "classic" variant — most common across HITACHI residential models).
// If your AC doesn't respond, the library also ships `IRHitachiAc1`,
// `IRHitachiAc2`, `IRHitachiAc3`, `IRHitachiAc264`, `IRHitachiAc344`
// for newer variants. Swap the class below and rebuild.

#ifdef BUDDY_HAS_HITACHI_AC

#include "hitachi_ac_remote.h"

#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <ir_Hitachi.h>

constexpr uint8_t HITACHI_IR_TX_PIN = 46;

static IRHitachiAc  ac(HITACHI_IR_TX_PIN);
static Preferences  prefs;
static bool         lastState = false;

static const char* kPrefsNamespace = "buddy";
static const char* kPrefsKey       = "hitachi_ac";

static void persistState(bool s) {
  lastState = s;
  prefs.begin(kPrefsNamespace, /*readonly=*/false);
  prefs.putBool(kPrefsKey, s);
  prefs.end();
}

// Sensible MVP defaults baked into every frame. HITACHI is stateful, so
// each send fully overrides the AC's current settings — picking cool /
// 25°C / auto fan makes the "AC ON" press behave predictably regardless
// of the AC's prior state.
static void configureFrame(bool on) {
  if (on) ac.on();
  else    ac.off();
  ac.setMode(kHitachiAcCool);
  ac.setTemp(25);
  ac.setFan(kHitachiAcFanAuto);
  ac.setSwingVertical(false);
  ac.setSwingHorizontal(false);
}

void hitachiAcInit() {
  // EXT_5V on so the internal IR LED's supply rail is energized.
  M5.Power.setExtOutput(true);

  ac.begin();

  prefs.begin(kPrefsNamespace, /*readonly=*/true);
  lastState = prefs.getBool(kPrefsKey, false);
  prefs.end();

  Serial.printf("[ir] hitachiAcInit done, last ac_state=%s, protocol=HITACHI_AC %d-byte\n",
                lastState ? "ON" : "OFF",
                kHitachiAcStateLength);
}

bool hitachiAcSendOn() {
  configureFrame(true);
  ac.send();
  Serial.println("[ir] HITACHI sent: ON  (cool / 25 C / auto fan)");
  persistState(true);
  return true;
}

bool hitachiAcSendOff() {
  configureFrame(false);
  ac.send();
  Serial.println("[ir] HITACHI sent: OFF");
  persistState(false);
  return false;
}

bool hitachiAcLastState() {
  return lastState;
}

bool hitachiAcToggleAndSend() {
  bool target = !lastState;
  return target ? hitachiAcSendOn() : hitachiAcSendOff();
}

#endif  // BUDDY_HAS_HITACHI_AC
