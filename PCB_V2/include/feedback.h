#pragma once
// LED + buzzer feedback for shot-like events, shared by normal survey use,
// calibration and the F/B check so a calibration point or an F/B shot looks
// and sounds exactly like a survey reading. (Calibration used to carry its
// own V1-era sequences — a click for a taken point, the LED simply going out
// on a failed one — which drifted from the survey flow once sounds.cpp
// arrived.) Change feedback here, not at the call sites.

#include "disco_manager.h"
#include "laser_manager.h"

namespace Feedback {

/// Shot requested and waiting for a steady hold: red + shot click.
/// keepLed leaves the LED alone (a splay taken while a leg's purple is latched).
void shotStart(DiscoManager &disco, bool keepLed = false);

/// Reading taken: green + the loud reading bleep. The LED is left green;
/// the caller turns it off (or latches purple) when the shot cycle ends.
void readingOk(DiscoManager &disco);

/// Splay (quick shot) taken: green + the flat splay beep, so it is told
/// apart from a leg shot by ear. LED left green, as for readingOk.
void splayOk(DiscoManager &disco);

/// Three agreeing shots: white flash under the rising fanfare, optional laser
/// wibble, then purple — which the caller keeps until the next FIRE press.
void legComplete(DiscoManager &disco, LaserManager &laser, bool wibble);

/// Shot failed (laser error, or a calibration point that never settled):
/// four red flashes, then the falling womp. LED ends off.
void failed(DiscoManager &disco);

} // namespace Feedback
