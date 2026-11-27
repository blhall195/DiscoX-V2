#pragma once

#include <stdint.h>

// The WS2812 status LED: shot feedback only (red = measuring, green = got a
// reading, red flashes = failed). The colours and the 0.3 brightness match
// the full firmware's DiscoManager static colours, so a shot looks the same.
namespace Led {
void begin();
void red();
void green();
void off();
} // namespace Led
