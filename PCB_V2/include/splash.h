#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Arduino.h>

// ── Boot splash animations ───────────────────────────────────────────
// User-selectable (menu → Settings → Splash, `splash_style` in config.json).
// Every style ends on a still frame carrying the DiscoX name, the BLE name
// suffix and the firmware version; that frame stays up while setup() finishes
// initialising. Each frame is a pure function of elapsed time, so the
// animation runs at whatever rate the I2C panel refresh allows (~20 fps).
namespace Splash {

enum Style : uint8_t {
    CLASSIC = 0, // device silhouette, laser flashes twice (the original)
    ENGRAVE,     // laser burns the name on in a scanning sweep
    MIRROR_BALL, // spinning disco ball drops in, letters light up
    COMPASS,     // needle settles on north, name decodes from noise
    DROP,        // letters fall and bounce, the X lands with a shockwave
    CAVE,        // fly down a cave passage into the light
    FLOOR,       // light-up dance floor, letters hop on the beat
    COUNT
};

/// Menu label for a style (out-of-range → CLASSIC's).
const char *name(uint8_t style);

/// Animation length including the final hold, in ms.
uint16_t durationMs(uint8_t style);

/// Draw one frame at time t (ms) into the buffer — no clear, no display().
void draw(Adafruit_GFX &g, uint8_t style, uint32_t t, const char *nameSuffix, const char *version);

/// Play a style start to finish (blocking, ~1.5-2.7 s), leaving the final
/// frame on screen.
void play(Adafruit_SH1107 &display, uint8_t style, const char *nameSuffix);

} // namespace Splash
