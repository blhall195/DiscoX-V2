#include "led.h"

#include "config.h"
#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel pixel(NEOPIXEL_COUNT, PIN_NEOPIXEL_DIN, NEO_GRB + NEO_KHZ800);
static constexpr float BRIGHTNESS = 0.3f; // = Disco::BASE_BRIGHTNESS in ../PCB_V2

static void set(uint8_t r, uint8_t g, uint8_t b) {
    pixel.setPixelColor(0, pixel.Color((uint8_t)(r * BRIGHTNESS), (uint8_t)(g * BRIGHTNESS),
                                       (uint8_t)(b * BRIGHTNESS)));
    pixel.show();
}

namespace Led {
void begin() {
    pixel.begin();
    pixel.setBrightness(255); // scaled in software, as DiscoManager does
    pixel.show();
}
void red() { set(255, 0, 0); }
void green() { set(0, 255, 0); }
void off() { set(0, 0, 0); }
} // namespace Led
