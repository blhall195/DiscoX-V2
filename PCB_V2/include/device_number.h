#pragma once

#include <cstddef>
#include <cstdint>

// ── Device (unit) number ─────────────────────────────────────────────
// The number written on the unit when it is built, shown on the splash
// next to the firmware version ("v2.0.10 #49"). The same firmware goes on
// every unit, so it lives on the chip: in the nRF52840's UICR customer
// registers, which firmware updates, storage reformats and the factory
// reset never touch (only a full SWD chip erase clears them).
//
// Set over serial with `>DEVNUM 49` (normal mode). Flash bits can only be
// cleared, so a number is never rewritten in place: each set takes the next
// blank slot and the newest slot wins, which allows a mistyped number to be
// corrected SLOTS-1 times.
namespace DeviceNumber {

constexpr uint16_t MAX = 9999;
constexpr uint8_t SLOTS = 8; // UICR CUSTOMER[0..7]

/// The unit's number, or 0 if none has been set.
uint16_t get();

/// Blank slots left (0 = the number can no longer be changed).
uint8_t slotsLeft();

/// "v2.0.10 #49", or just the version when no number is set.
void versionLabel(char *buf, size_t len);

/// Store a number (1..MAX) and reboot. Returns only on failure: number out
/// of range, or no blank slot left. The caller must have synced storage
/// first: this stops the SoftDevice and resets.
void setAndReboot(uint16_t number);

} // namespace DeviceNumber
