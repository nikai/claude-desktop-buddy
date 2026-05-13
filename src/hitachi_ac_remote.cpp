// HITACHI AC remote — see hitachi_ac_remote.h.
//
// The whole translation unit is gated behind BUDDY_HAS_HITACHI_AC because
// the repository's `build_src_filter = +<*>` includes every .cpp in every
// env, but StickC Plus env doesn't depend on IRremoteESP8266 (it's a
// StickS3-only feature). Without this guard, the StickC Plus build would
// fail to find IRremoteESP8266.h.

#ifdef BUDDY_HAS_HITACHI_AC

#include "hitachi_ac_remote.h"
#include "hitachi_ac_codes.h"

#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <IRsend.h>

// StickS3 internal IR transmitter is on GPIO46. The pin coexists with
// M5Unified's gpio_hi(46) in M5.begin() (Power Hold for Capsule / Dial /
// DinMeter) — hardware compatibility for HITACHI-length raw frames must
// be verified by Phase A.5 in the plan (~450-symbol stress test).
constexpr uint8_t HITACHI_IR_TX_PIN = 46;

static IRsend     irsend(HITACHI_IR_TX_PIN);
static Preferences prefs;
static bool       lastState = false;   // last persisted ac_state: false=OFF, true=ON

// NVS namespace shared with stats/settings/owner/petname (see stats.h).
// Direct use of `Preferences` (not stats.h::_prefs) — stats.h is
// header-static and would dupe state if a second TU included it.
static const char* kPrefsNamespace = "buddy";
static const char* kPrefsKey       = "hitachi_ac";

static bool sendRawSafe(const uint16_t* buf, size_t len, const char* label) {
  if (len == 0 || buf == nullptr) {
    Serial.printf("[ir] HITACHI %s: codes not recorded yet (len=0), skipping send\n", label);
    return false;
  }
  irsend.sendRaw(buf, len, HITACHI_AC_CARRIER_HZ);
  Serial.printf("[ir] HITACHI sent: %s (%u symbols @ %u kHz)\n",
                label, (unsigned)len, (unsigned)HITACHI_AC_CARRIER_HZ);
  return true;
}

static void persistState(bool s) {
  lastState = s;
  prefs.begin(kPrefsNamespace, /*readonly=*/false);
  prefs.putBool(kPrefsKey, s);
  prefs.end();
}

void hitachiAcInit() {
  // EXT_5V on. On StickS3 the M5PM1 power IC routes the IR transmitter
  // through the external rail; on other boards this is a no-op.
  M5.Power.setExtOutput(true);

  irsend.begin();

  prefs.begin(kPrefsNamespace, /*readonly=*/true);
  lastState = prefs.getBool(kPrefsKey, false);
  prefs.end();

  Serial.printf("[ir] hitachiAcInit done, last ac_state=%s, on_len=%u off_len=%u\n",
                lastState ? "ON" : "OFF",
                (unsigned)HITACHI_AC_POWER_ON_LEN,
                (unsigned)HITACHI_AC_POWER_OFF_LEN);
}

bool hitachiAcSendOn() {
  bool sent = sendRawSafe(HITACHI_AC_POWER_ON_RAW, HITACHI_AC_POWER_ON_LEN, "ON");
  if (sent) persistState(true);
  return true;   // contract: returns the intended new state, regardless of len==0 placeholder
}

bool hitachiAcSendOff() {
  bool sent = sendRawSafe(HITACHI_AC_POWER_OFF_RAW, HITACHI_AC_POWER_OFF_LEN, "OFF");
  if (sent) persistState(false);
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
