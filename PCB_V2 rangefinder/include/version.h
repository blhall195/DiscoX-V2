#pragma once

// Rangefinder firmware version — independent of ../PCB_V2's numbering.
// Shown on the boot screen (with the unit number, see device_number.h) and
// on the boot serial line. Max 21 characters (size-1 line on a 128 px panel).
constexpr char FIRMWARE_VERSION[] = "RF v1.0.0";
