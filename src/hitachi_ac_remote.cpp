// HITACHI AC remote — see hitachi_ac_remote.h.
//
// **No recording**: uses IRremoteESP8266's HITACHI AC protocol encoders
// to synthesize full-state IR frames directly, bypassing the
// broken-on-StickS3 internal IR receiver entirely. HITACHI AC remotes
// don't use a "power toggle" code — every press is a full state frame
// (power + mode + temp + fan + ...). We pick sensible MVP defaults:
// cool / 25°C / auto fan, and flip just the power bit.
//
// Multi-variant strategy: we don't know which HITACHI sub-protocol the
// user's AC speaks (Ac / Ac1 / Ac2 / Ac3 / Ac264 / Ac344 — different
// HITACHI residential / commercial product lines use different framing).
// Rather than ask the user to figure out which one, we **send all six
// in sequence on each toggle**. The AC silently ignores frames whose
// protocol/checksum it doesn't recognize and acts on whichever one it
// does. Total send time ~1.5–2 s; acceptable for a manual AC toggle.

#ifdef BUDDY_HAS_HITACHI_AC

#include "hitachi_ac_remote.h"

#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <ir_Hitachi.h>

constexpr uint8_t HITACHI_IR_TX_PIN = 46;

// One instance per variant. Each owns its own state buffer.
// IRremoteESP8266 2.8.6 quirk: `IRHitachiAc3::send()` is declared in the
// header but has no body in ir_Hitachi.cpp (library bug), so AC3 is
// omitted from the variant list. The 5 remaining variants cover the
// great majority of HITACHI models in the wild.
static IRHitachiAc     ac_v0(HITACHI_IR_TX_PIN);   // HITACHI_AC,    28-byte / 224-bit
static IRHitachiAc1    ac_v1(HITACHI_IR_TX_PIN);   // HITACHI_AC1,   13-byte
static IRHitachiAc264  ac_v264(HITACHI_IR_TX_PIN); // HITACHI_AC264, 33-byte / 264-bit
static IRHitachiAc344  ac_v344(HITACHI_IR_TX_PIN); // HITACHI_AC344, 43-byte / 344-bit
static IRHitachiAc296  ac_v296(HITACHI_IR_TX_PIN); // HITACHI_AC296, 37-byte / 296-bit

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

// Configure-and-send for each variant. They share the same conceptual
// API but constants and capabilities differ — keep each block explicit
// rather than templated to avoid surprises when a variant doesn't
// expose a setter (e.g. AC1 has no swing setter, AC344 has different
// fan constants). Defaults: cool / 25°C / auto fan.
static void sendVariantAc(bool on) {
  if (on) ac_v0.on(); else ac_v0.off();
  ac_v0.setMode(kHitachiAcCool);
  ac_v0.setTemp(25);
  ac_v0.setFan(kHitachiAcFanAuto);
  ac_v0.send();
}
static void sendVariantAc1(bool on) {
  ac_v1.setPower(on);
  ac_v1.setMode(kHitachiAc1Cool);
  ac_v1.setTemp(25);
  ac_v1.setFan(kHitachiAc1FanAuto);
  ac_v1.send();
}
static void sendVariantAc264(bool on) {
  if (on) ac_v264.on(); else ac_v264.off();
  ac_v264.setMode(kHitachiAc264Cool);
  ac_v264.setTemp(25);
  ac_v264.setFan(kHitachiAc264FanAuto);
  ac_v264.send();
}
static void sendVariantAc344(bool on) {
  if (on) ac_v344.on(); else ac_v344.off();
  ac_v344.setMode(kHitachiAc344Cool);
  ac_v344.setTemp(25);
  ac_v344.setFan(kHitachiAc344FanAuto);
  ac_v344.send();
}
static void sendVariantAc296(bool on) {
  ac_v296.setPower(on);
  ac_v296.setMode(kHitachiAc296Cool);
  ac_v296.setTemp(25);
  ac_v296.setFan(kHitachiAc296FanAuto);
  ac_v296.send();
}

static void sendAllVariants(bool on) {
  Serial.printf("[ir] HITACHI: spraying 5 variants (%s)...\n", on ? "ON" : "OFF");
  sendVariantAc(on);    Serial.println("[ir]   sent: HITACHI_AC");    delay(50);
  sendVariantAc1(on);   Serial.println("[ir]   sent: HITACHI_AC1");   delay(50);
  sendVariantAc264(on); Serial.println("[ir]   sent: HITACHI_AC264"); delay(50);
  sendVariantAc344(on); Serial.println("[ir]   sent: HITACHI_AC344"); delay(50);
  sendVariantAc296(on); Serial.println("[ir]   sent: HITACHI_AC296");
  Serial.println("[ir] HITACHI: all variants sent");
}

void hitachiAcInit() {
  // EXT_5V on so the internal IR LED's supply rail is energized.
  M5.Power.setExtOutput(true);

  ac_v0.begin();
  ac_v1.begin();
  ac_v264.begin();
  ac_v344.begin();
  ac_v296.begin();

  prefs.begin(kPrefsNamespace, /*readonly=*/true);
  lastState = prefs.getBool(kPrefsKey, false);
  prefs.end();

  Serial.printf("[ir] hitachiAcInit done, last ac_state=%s, will spray all 6 HITACHI variants per toggle\n",
                lastState ? "ON" : "OFF");
}

bool hitachiAcSendOn() {
  sendAllVariants(true);
  persistState(true);
  return true;
}

bool hitachiAcSendOff() {
  sendAllVariants(false);
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
