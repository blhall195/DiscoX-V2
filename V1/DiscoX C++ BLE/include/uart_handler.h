#pragma once
#include <Arduino.h>

// Initialise UART handler state — call once in setup()
void uartInit();

// Non-blocking poll: read UART when DRDY is asserted, parse lines.
// Call every loop() iteration.
void uartPoll();
