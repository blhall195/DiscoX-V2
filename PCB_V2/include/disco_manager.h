#pragma once
// Disco Manager — NeoPixel LED effects (non-blocking state machine)
// Session 9: static colors, disco rainbow (with gyro-driven wild mode), purple
// pulse.

#include "config.h"
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

/// Active LED effect
enum class DiscoEffect : uint8_t {
    OFF,
    STATIC,      // solid colour, no animation
    DISCO,       // HSV rainbow cycling, shake → wild mode
    PURPLE_PULSE // sine-wave brightness modulation
};

class DiscoManager {
  public:
    /// Call once in setup() after pin init
    void begin();

    /// Call every loop() iteration — drives animations non-blockingly
    void update(float accelX, float accelY, float accelZ);

    // ── Static colours ──────────────────────────────────────────────
    void setRed();
    void setGreen();
    void setBlue();
    void setWhite();
    void turnOff();

    // ── Animated effects ────────────────────────────────────────────
    void turnOn();
    void setPurple(); // starts purple pulse

    // ── Music sync (the Mario easter egg) ───────────────────────────
    /// Punch the LED for one note of the tune: full brightness, hue picked
    /// from the pitch, then a fade until the next note lands. freqHz 0 (a
    /// rest) blacks it out, which is what turns the fade into a strobe.
    /// Takes over the disco's own rainbow until clearMusicSync(); does
    /// nothing unless the disco is running.
    void noteHit(uint16_t freqHz);
    /// Hand the disco back its rainbow when the tune stops.
    void clearMusicSync();

    // ── State queries ───────────────────────────────────────────────
    bool isDiscoActive() const { return effect_ == DiscoEffect::DISCO; }
    bool isPulseActive() const { return effect_ == DiscoEffect::PURPLE_PULSE; }
    bool isActive() const { return effect_ != DiscoEffect::OFF; }
    DiscoEffect effect() const { return effect_; }

  private:
    // V2: WS2812B on PIN_NEOPIXEL_DIN (pins_v2.h). No power-gate pin — the
    // LED rail is hardware-gated by ENA.
    Adafruit_NeoPixel pixel_{NEOPIXEL_COUNT, PIN_NEOPIXEL_DIN, NEO_GRB + NEO_KHZ800};
    DiscoEffect effect_ = DiscoEffect::OFF;

    // ── Brightness ──────────────────────────────────────────────────
    float brightness_ = Disco::BASE_BRIGHTNESS;

    // ── Disco state ─────────────────────────────────────────────────
    uint16_t hueStep_ = 0; // rotating hue (0-359)
    bool wildLatched_ = false;
    uint32_t lastShakeTime_ = 0;
    uint32_t lastFrameTime_ = 0;

    // ── Music sync state ────────────────────────────────────────────
    bool musicSync_ = false;
    float noteHue_ = 0.0f;    // 0-1, from the pitch of the note
    float noteBright_ = 0.0f; // envelope, 1.0 on the attack down to 0
    uint32_t lastEnvTime_ = 0;
    // The transposed tune spans E6-C8; mapping that span across the whole
    // colour wheel is what makes Mario's leaps throw the colour around.
    static constexpr uint16_t MUSIC_MIN_HZ = 1319; // E6
    static constexpr uint16_t MUSIC_MAX_HZ = 4186; // C8
    // Fade per second. Notes are 150 ms slots, so ~0.75 of the envelope is
    // spent by the next attack: every note reads as its own flash, and the
    // long rests go fully dark.
    static constexpr float NOTE_DECAY_PER_SEC = 5.0f;
    static constexpr uint32_t ENVELOPE_MS = 16; // ~60 Hz envelope updates

    // ── Purple pulse state ──────────────────────────────────────────
    float pulsePhase_ = 0.0f;
    uint32_t lastPulseTime_ = 0;

    // ── Helpers ─────────────────────────────────────────────────────
    void setPixel(uint8_t r, uint8_t g, uint8_t b);
    void stopAll();

    /// HSV → RGB (h 0.0-1.0, s 0.0-1.0, v 0.0-1.0) → 0-255 per channel
    static void hsvToRgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b);
};
