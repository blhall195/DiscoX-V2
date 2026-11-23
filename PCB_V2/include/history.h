#pragma once

#include "history_log.h"

// The device's reading history, on the internal-flash region
// FlashLayout::HISTORY_START..HISTORY_END (see history_log.h for the format).
// Call device().begin() once in setup(); readings go in through add() from
// handleMeasurementSuccess() and reach flash with sync() while idle and
// before power-off/standby.
namespace History {
Log &device();
} // namespace History
