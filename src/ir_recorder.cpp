// IR recorder for the original HITACHI remote — see ir_recorder.h.
//
// Run flow (also documented on-device via screen prompts):
//   Phase 1: user confirms AC is physically OFF, presses BtnA to arm, then
//            presses POWER on the HITACHI remote. We capture the "turn
//            ON" frame and dump it to Serial as a resultToSourceCode() block.
//   Phase 2: AC is now ON (the captured frame did its job in the real
//            world too). User presses BtnA to re-arm, then presses POWER
//            again. We capture the "turn OFF" frame.
//   Done:    screen says copy serial output to hitachi_ac_codes.cpp.
//            Device sits in a death-loop until power-cycled into normal
//            firmware.
//
// Wait-loop discipline: this whole routine runs from setup() and never
// returns, so loop() never executes. Every wait point MUST call
// M5.update() to drive M5Unified's button state machine, plus a short
// delay() / yield() to let the BLE stack / IDF tasks breathe.

#if defined(BUDDY_HAS_HITACHI_AC) && defined(BUDDY_IR_RECORDER)

#include "ir_recorder.h"

#include <Arduino.h>
#include <M5Unified.h>
#include <IRrecv.h>
#include <IRutils.h>

// StickS3 internal IR receiver GPIO. RMT-driven via IRremoteESP8266.
constexpr uint8_t HITACHI_IR_RX_PIN = 42;

// HITACHI AC frames can run up to ~700 raw symbols (AC344 variant), AND
// HITACHI remotes typically transmit the same frame 2-3 times per button
// press as redundancy. 2048 covers up to ~3 long-variant repeats; if a
// real capture still hits this ceiling, bump to 4096 and reflash.
constexpr uint16_t kRecorderBufSize    = 2048;
// Inter-pulse timeout (ms) deciding "no more pulses, frame complete".
// HITACHI's between-repeat gap is around 40-50ms, so 50 ms (library
// default) tends to concatenate multiple repeats into one giant blob and
// hide the protocol-detector. 15 ms cleanly separates repeats so the
// decoder sees a single frame.
constexpr uint8_t  kRecorderTimeoutMs  = 15;

static IRrecv      irrecv(HITACHI_IR_RX_PIN, kRecorderBufSize, kRecorderTimeoutMs, /*save_buffer=*/true);
static decode_results results;

// Render a centered single-line message in the upper part of the screen,
// clearing the area first. Keeps the recorder UI minimal.
static void showLine(const char* line1, const char* line2 = nullptr) {
  M5.Display.fillScreen(0x0000);
  M5.Display.setTextDatum(MC_DATUM);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(0xFFFF, 0x0000);
  M5.Display.drawString(line1, M5.Display.width() / 2, M5.Display.height() / 2 - 8);
  if (line2) {
    M5.Display.drawString(line2, M5.Display.width() / 2, M5.Display.height() / 2 + 8);
  }
  M5.Display.setTextDatum(TL_DATUM);
}

// Block until BtnA short-click event. M5Unified needs M5.update() to
// progress; with no loop() this routine must drive it manually.
static void waitForBtnAClick() {
  // Drain any stale state first.
  M5.update();
  while (true) {
    M5.update();
    if (M5.BtnA.wasClicked() || M5.BtnA.wasReleased()) return;
    delay(10);
  }
}

// Block until IRrecv decodes a frame, then return. Caller owns the
// receiver and is expected to disableIRIn() afterwards.
static void waitForIrFrame() {
  while (true) {
    M5.update();   // keep button state machine alive in case user wants to abort (future)
    if (irrecv.decode(&results)) return;
    delay(2);
  }
}

static void dumpFrame(const char* label) {
  Serial.println();
  Serial.printf("[recorder] === %s capture ===\n", label);
  Serial.printf("[recorder] protocol      : %s\n", typeToString(results.decode_type).c_str());
  Serial.printf("[recorder] decode_type   : %d (0=UNKNOWN)\n", (int)results.decode_type);
  Serial.printf("[recorder] bits          : %u\n", results.bits);
  Serial.printf("[recorder] rawlen        : %u\n", results.rawlen);
  Serial.printf("[recorder] overflow      : %s\n", results.overflow ? "YES (raise kCaptureBufferSize)" : "no");
  Serial.println("[recorder] --- BEGIN resultToSourceCode (paste into hitachi_ac_codes.cpp, rename arrays) ---");
  // resultToSourceCode produces a self-contained C++ snippet with the
  // raw timing array, length, and a sendRaw() example. It correctly
  // skips rawbuf[0] (leading gap) — do not try to extract rawbuf by hand.
  Serial.print(resultToSourceCode(&results));
  Serial.println("[recorder] --- END resultToSourceCode ---");
  Serial.println();
}

void irRecorderRun() {
  // Speaker amplifier (StickS3 has a real I2S DAC + ES8311) can leak
  // noise into the IR receiver via shared peripheral timing. Shut it
  // down for the duration of recording.
  M5.Speaker.end();

  // EXT_5V on — gives the internal IR receiver its supply rail. Harmless
  // on boards that don't need it.
  M5.Power.setExtOutput(true);

  Serial.println();
  Serial.println("================================================================");
  Serial.println("[recorder] HITACHI AC raw-frame recorder");
  Serial.println("[recorder] IR_RX pin: GPIO42, buffer: 1024 symbols, timeout: 50ms");
  Serial.println("================================================================");

  // ---------------- Phase 1: capture POWER_ON frame ----------------
  showLine("AC must be OFF", "Press BtnA to arm");
  Serial.println("[recorder] Phase 1: ensure AC is OFF, press BtnA on device");
  waitForBtnAClick();

  showLine("Aim remote, press", "POWER on remote");
  Serial.println("[recorder] Phase 1: armed. Press POWER on the original remote now.");
  irrecv.enableIRIn();
  waitForIrFrame();
  irrecv.disableIRIn();
  dumpFrame("PHASE 1 / POWER_ON");

  // ---------------- Phase 2: capture POWER_OFF frame ---------------
  // The AC is now ON in the real world (the Phase 1 frame both got
  // captured here AND reached the AC unit). The remote's internal state
  // also flipped to ON because of that button press. So we go straight
  // to the OFF capture — do NOT ask the user to press the remote between
  // phases, that would flip the remote state back to OFF and the next
  // recorded frame would be the wrong one.
  showLine("AC should be ON now", "Press BtnA to arm");
  Serial.println("[recorder] Phase 2: AC is now ON. Press BtnA on device.");
  waitForBtnAClick();

  showLine("Press POWER now", "(AC will turn OFF)");
  Serial.println("[recorder] Phase 2: armed. Press POWER on the original remote now.");
  irrecv.enableIRIn();
  waitForIrFrame();
  irrecv.disableIRIn();
  dumpFrame("PHASE 2 / POWER_OFF");

  // ---------------- Done ----------------
  showLine("Done.", "Copy serial output");
  Serial.println("[recorder] ALL DONE. Copy both resultToSourceCode blocks above into");
  Serial.println("[recorder] src/hitachi_ac_codes.cpp, renaming the arrays as documented");
  Serial.println("[recorder] in that file. Then rebuild & flash the normal m5sticks3 env.");
  Serial.println("[recorder] Sitting in a death-loop now. Replug USB or hold KEY1 + replug");
  Serial.println("[recorder] to enter download mode for the next flash.");

  while (true) {
    M5.update();
    delay(100);
  }
}

#endif  // BUDDY_HAS_HITACHI_AC && BUDDY_IR_RECORDER
