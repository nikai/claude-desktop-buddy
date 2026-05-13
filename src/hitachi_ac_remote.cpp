// HITACHI AC remote — see hitachi_ac_remote.h.
//
// **Bit-bang implementation**: bypasses all IR libraries' RMT layers
// (IRremoteESP8266 uses legacy <driver/rmt.h> which has known issues
// with carrier modulation on ESP32-S3; ESP-IDF v5 driver/rmt_tx.h isn't
// available in this Arduino core). Instead, hand-toggle GPIO46 at 38kHz
// for marks, hold LOW for spaces. We use IRremoteESP8266 only to
// generate the protocol state byte arrays — the actual transmission is
// our own.
//
// Pin sharing note: GPIO46 on StickS3 is also the speaker amp enable.
// We must M5.Speaker.end() + gpio_reset_pin(46) before driving it as
// IR, then restore speaker after.

#ifdef BUDDY_HAS_HITACHI_AC

#include "hitachi_ac_remote.h"

#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <ir_Hitachi.h>
#include <driver/gpio.h>

constexpr uint8_t HITACHI_IR_TX_PIN = 46;

// State-byte generators only — we DON'T call .send() on these.
static IRHitachiAc     ac_v0(HITACHI_IR_TX_PIN);   // 28-byte / 224-bit
static IRHitachiAc1    ac_v1(HITACHI_IR_TX_PIN);   // 13-byte
static IRHitachiAc264  ac_v264(HITACHI_IR_TX_PIN); // 33-byte / 264-bit
static IRHitachiAc344  ac_v344(HITACHI_IR_TX_PIN); // 43-byte / 344-bit
static IRHitachiAc296  ac_v296(HITACHI_IR_TX_PIN); // 37-byte / 296-bit

static Preferences  prefs;
static bool         lastState = false;

static const char* kPrefsNamespace = "buddy";
static const char* kPrefsKey       = "hitachi_ac";

// =========================================================================
// Bit-bang 38kHz IR carrier.
// 38 kHz period = 26.32 us. Tight loop: 8us high + 18us low ≈ 38.46 kHz,
// 30% duty cycle (matches most TSOP receivers).
// =========================================================================
static inline void irMark(uint32_t us) {
  uint32_t start = micros();
  while ((int32_t)(micros() - start) < (int32_t)us) {
    digitalWrite(GPIO_NUM_46, HIGH);
    delayMicroseconds(8);
    digitalWrite(GPIO_NUM_46, LOW);
    delayMicroseconds(18);
  }
}

static inline void irSpace(uint32_t us) {
  digitalWrite(GPIO_NUM_46, LOW);
  if (us <= 16383) delayMicroseconds(us);
  else             delay((us + 999) / 1000);
}

// Send a state-byte array using the given protocol timing. byte_order_lsb_first
// matches HITACHI variants which transmit each byte LSB first.
struct HitachiTiming {
  uint16_t hdrMark;
  uint16_t hdrSpace;
  uint16_t bitMark;
  uint16_t oneSpace;
  uint16_t zeroSpace;
  uint32_t gap;        // inter-frame gap (microseconds)
};

static void sendHitachiBitBang(const uint8_t* state, size_t len,
                               const HitachiTiming& t,
                               bool lsb_first = true) {
  irMark(t.hdrMark);
  irSpace(t.hdrSpace);
  for (size_t b = 0; b < len; b++) {
    uint8_t byte = state[b];
    for (int bit = 0; bit < 8; bit++) {
      uint8_t bit_val = lsb_first ? ((byte >> bit) & 1)
                                  : ((byte >> (7 - bit)) & 1);
      irMark(t.bitMark);
      irSpace(bit_val ? t.oneSpace : t.zeroSpace);
    }
  }
  // Trailing mark to terminate frame
  irMark(t.bitMark);
  irSpace(t.gap);
}

// Protocol timing constants pulled from IRremoteESP8266's ir_Hitachi.cpp.
// IMPORTANT (verified against the library's IRsend::sendHitachiAC):
//   - AC, AC264, AC296, AC344 all share the SAME timing (kHitachiAc*).
//     They differ only in byte-order (MSB-first for AC/AC1, LSB-first
//     for AC264/AC296/AC344). The AC424 timing in the library is for a
//     DIFFERENT protocol (HITACHI_AC424, 53 bytes with leader) which we
//     do not need to support here.
//   - AC1 has its own header timing (kHitachiAc1Hdr*) but same bit timing.
static const HitachiTiming TIMING_AC  = { 3300, 1700, 400, 1250, 500, 100000 };
static const HitachiTiming TIMING_AC1 = { 3400, 3400, 400, 1250, 500, 100000 };

// =========================================================================
// Public API
// =========================================================================
static void persistState(bool s) {
  lastState = s;
  prefs.begin(kPrefsNamespace, /*readonly=*/false);
  prefs.putBool(kPrefsKey, s);
  prefs.end();
}

static void configureFrames(bool on) {
  // CRITICAL: IRremoteESP8266's 424-series (AC264 / AC296 / AC344)
  // encodes a "Button" field in the state. Each setter call rewrites
  // that field to "the button corresponding to this setter" — setTemp
  // → TempUp, setFan → FanSpeed, setPower → Power/Mode. The LAST
  // setter call wins. We want the AC to interpret these frames as a
  // **power toggle**, so setPower / on() / off() MUST be the final
  // call. Otherwise the AC sees "TempUp 25C" and ignores power state.
  // For AC and AC1 (which don't have a Button field) the order is
  // harmless either way, but we apply the same convention uniformly.

  // AC (28-byte)
  ac_v0.setMode(kHitachiAcCool);
  ac_v0.setTemp(25);
  ac_v0.setFan(kHitachiAcFanAuto);
  if (on) ac_v0.on(); else ac_v0.off();    // last → no Button to set, harmless

  // AC1: stateReset() leaves SwingToggle / SwingV bits set by default.
  // The library's IRHitachiAc1::send() clears the toggle bits after
  // transmission so subsequent frames don't keep flipping swing; our
  // hand bit-bang skips that cleanup. Explicitly clear both swing
  // states so our power-only frame won't make the AC oscillate its
  // vertical louvers as a side effect.
  ac_v1.setMode(kHitachiAc1Cool);
  ac_v1.setTemp(25);
  ac_v1.setFan(kHitachiAc1FanAuto);
  ac_v1.setSwingV(false);
  ac_v1.setSwingToggle(false);
  ac_v1.setPower(on);

  // AC264
  ac_v264.setMode(kHitachiAc264Cool);
  ac_v264.setTemp(25);
  ac_v264.setFan(kHitachiAc264FanAuto);
  if (on) ac_v264.on(); else ac_v264.off();  // last → Button = Power/Mode

  // AC344
  ac_v344.setMode(kHitachiAc344Cool);
  ac_v344.setTemp(25);
  ac_v344.setFan(kHitachiAc344FanAuto);
  if (on) ac_v344.on(); else ac_v344.off();  // last → Button = Power/Mode

  // AC296
  ac_v296.setMode(kHitachiAc296Cool);
  ac_v296.setTemp(25);
  ac_v296.setFan(kHitachiAc296FanAuto);
  ac_v296.setPower(on);                       // last
}

static void sendAllVariantsBitBang(bool on) {
  Serial.printf("[ir] HITACHI: bit-banging 5 variants (%s)...\n", on ? "ON" : "OFF");

  // Reclaim GPIO46 from M5Unified's speaker driver / boot Power-Hold.
  M5.Speaker.end();
  gpio_reset_pin(GPIO_NUM_46);
  M5.Power.setExtOutput(true);
  delay(50);
  pinMode(GPIO_NUM_46, OUTPUT);
  digitalWrite(GPIO_NUM_46, LOW);

  configureFrames(on);

  // Per IRsend::sendHitachiAC source: AC + AC1 are MSB-first per byte,
  // while AC264 / AC296 / AC344 are LSB-first per byte. All non-AC1
  // variants share the same TIMING_AC. AC1 alone uses the wider
  // 3400/3400 header.
  sendHitachiBitBang(ac_v0.getRaw(),   kHitachiAcStateLength,    TIMING_AC,  /*lsb_first=*/false);
  Serial.println("[ir]   sent: HITACHI_AC (bit-bang, MSB-first)");
  delay(50);

  sendHitachiBitBang(ac_v1.getRaw(),   kHitachiAc1StateLength,   TIMING_AC1, /*lsb_first=*/false);
  Serial.println("[ir]   sent: HITACHI_AC1 (bit-bang, MSB-first)");
  delay(50);

  sendHitachiBitBang(ac_v264.getRaw(), kHitachiAc264StateLength, TIMING_AC,  /*lsb_first=*/true);
  Serial.println("[ir]   sent: HITACHI_AC264 (bit-bang, LSB-first)");
  delay(50);

  sendHitachiBitBang(ac_v344.getRaw(), kHitachiAc344StateLength, TIMING_AC,  /*lsb_first=*/true);
  Serial.println("[ir]   sent: HITACHI_AC344 (bit-bang, LSB-first)");
  delay(50);

  sendHitachiBitBang(ac_v296.getRaw(), kHitachiAc296StateLength, TIMING_AC,  /*lsb_first=*/true);
  Serial.println("[ir]   sent: HITACHI_AC296 (bit-bang, LSB-first)");

  digitalWrite(GPIO_NUM_46, LOW);

  // Restore speaker so beep() works
  M5.Speaker.begin();
  M5.Speaker.setVolume(80);
  Serial.println("[ir] HITACHI: bit-bang done, speaker restored");
}

void hitachiAcInit() {
  M5.Power.setExtOutput(true);
  delay(50);
  bool extOn  = M5.Power.getExtOutput();
  int  boardN = (int)M5.getBoard();
  const int expectedBoard = (int)m5::board_t::board_M5StickS3;
  Serial.printf("[ir] EXT_5V check: M5.getBoard()=%d (StickS3 expected=%d, match=%s), "
                "M5.Power.getExtOutput()=%d\n",
                boardN, expectedBoard,
                boardN == expectedBoard ? "YES" : "NO",
                (int)extOn);

  // Library state generators initialization
  ac_v0.begin();
  ac_v1.begin();
  ac_v264.begin();
  ac_v344.begin();
  ac_v296.begin();

  prefs.begin(kPrefsNamespace, /*readonly=*/true);
  lastState = prefs.getBool(kPrefsKey, false);
  prefs.end();

  Serial.printf("[ir] hitachiAcInit done, last ac_state=%s, mode=bit-bang HITACHI 5 variants\n",
                lastState ? "ON" : "OFF");
}

bool hitachiAcSendOn() {
  sendAllVariantsBitBang(true);
  persistState(true);
  return true;
}

bool hitachiAcSendOff() {
  sendAllVariantsBitBang(false);
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
