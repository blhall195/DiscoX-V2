#pragma once

#include "config.h"
#include <Arduino.h>

constexpr uint8_t NUM_BUTTONS = 4;

// Button indices — PCB V2 has 4 GPIO buttons (H3 header) plus a dedicated
// hardware power toggle into the LTC2954 (handled by the power driver, not
// here). Roles per the V2 UI scheme:
enum class Button : uint8_t {
    FIRE = 0,     // Button 1 (P0.27) — take measurement / select
    UP_DISCO = 1, // Button 2 (P1.03) — menu up / disco toggle / hold for snake
    DOWN = 2,     // Button 3 (P1.05) — menu down
    MENU = 3,     // Button 4 (P1.07) — menu open / select
};

class ButtonManager {
  public:
    void begin();
    void update();

    bool isPressed(Button btn) const;
    bool wasPressed(Button btn);
    bool wasReleased(Button btn);

    static const char *name(Button btn);

  private:
    struct State {
        uint8_t pin;
        bool debounced;          // true = pressed (active-low inverted)
        bool previous;           // debounced state from last update()
        bool raw;                // last raw reading (true = pressed)
        uint32_t lastChangeTime; // millis() when raw last changed
        bool fell;               // edge: just pressed this tick
        bool rose;               // edge: just released this tick
    };

    State btn_[NUM_BUTTONS];
};
