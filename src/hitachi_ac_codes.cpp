// Placeholder. Real raw frames will be pasted here after running the
// `m5sticks3-recorder` env once with the original HITACHI remote. Until
// then both arrays are length-0 and the send functions early-return.
//
// To populate:
//   1. Build & flash recorder env: pio run -e m5sticks3-recorder -t upload
//   2. pio device monitor -e m5sticks3-recorder
//   3. Follow on-screen prompts to capture POWER_ON then POWER_OFF
//   4. Replace the two empty initializer blocks below with the
//      resultToSourceCode() output (renamed to *_RAW arrays + *_LEN sizes)
//   5. Rebuild normal env: pio run -e m5sticks3 -t upload

#include "hitachi_ac_codes.h"

const uint16_t HITACHI_AC_POWER_ON_RAW[] = {};
const size_t   HITACHI_AC_POWER_ON_LEN  = sizeof(HITACHI_AC_POWER_ON_RAW) / sizeof(HITACHI_AC_POWER_ON_RAW[0]);

const uint16_t HITACHI_AC_POWER_OFF_RAW[] = {};
const size_t   HITACHI_AC_POWER_OFF_LEN  = sizeof(HITACHI_AC_POWER_OFF_RAW) / sizeof(HITACHI_AC_POWER_OFF_RAW[0]);
