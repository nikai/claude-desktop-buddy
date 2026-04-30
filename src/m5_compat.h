#pragma once
// M5Unified replaces the M5StickCPlus library across both StickC Plus
// and StickS3 builds. The codebase still uses the older TFT_eSPI /
// TFT_eSprite type names — alias them to M5Unified types.
//
// Original TFT_eSPI hierarchy: TFT_eSprite : public TFT_eSPI, so an
// upcast TFT_eSprite* → TFT_eSPI* worked. M5Unified's M5GFX and
// M5Canvas are siblings (both derive from lgfx::LGFXBase via different
// paths), so we alias TFT_eSPI to the common base LovyanGFX. M5.Lcd is
// LovyanGFX-compatible and so is M5Canvas.
#include <M5Unified.h>

using TFT_eSPI    = LovyanGFX;
using TFT_eSprite = M5Canvas;
