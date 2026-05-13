#pragma once
// Temporary IR recorder. Built into the firmware only when both
// BUDDY_HAS_HITACHI_AC and BUDDY_IR_RECORDER are defined (m5sticks3-recorder
// env). irRecorderRun() is called once from setup() and never returns —
// the device sits in a record-and-dump loop until power-cycled into the
// normal m5sticks3 firmware.

#if defined(BUDDY_HAS_HITACHI_AC) && defined(BUDDY_IR_RECORDER)

void irRecorderRun();

#endif
