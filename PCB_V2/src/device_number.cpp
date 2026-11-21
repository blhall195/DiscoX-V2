#include "device_number.h"
#include "version.h"
#include <Arduino.h>
#include <cstdio>
#include <nrf_sdm.h>

namespace DeviceNumber {

// A slot holds TAG in the top half and the number in the bottom half, so a
// register something else has written is not mistaken for a number.
static constexpr uint32_t TAG = 0x444E0000; // "DN"
static constexpr uint32_t BLANK = 0xFFFFFFFF;

/// Index of the first blank slot (SLOTS if none).
static uint8_t firstBlank() {
    uint8_t i = 0;
    while (i < SLOTS && NRF_UICR->CUSTOMER[i] != BLANK) {
        i++;
    }
    return i;
}

uint16_t get() {
    uint8_t used = firstBlank();
    if (used == 0) {
        return 0;
    }
    uint32_t v = NRF_UICR->CUSTOMER[used - 1];
    uint32_t n = v & 0xFFFF;
    return ((v & 0xFFFF0000) == TAG && n >= 1 && n <= MAX) ? (uint16_t)n : 0;
}

uint8_t slotsLeft() {
    return SLOTS - firstBlank();
}

void versionLabel(char *buf, size_t len) {
    uint16_t n = get();
    if (n) {
        snprintf(buf, len, "%s #%u", FIRMWARE_VERSION, (unsigned)n);
    } else {
        snprintf(buf, len, "%s", FIRMWARE_VERSION);
    }
}

void setAndReboot(uint16_t number) {
    uint8_t slot = firstBlank();
    if (number < 1 || number > MAX || slot >= SLOTS) {
        return;
    }

    // The SoftDevice owns the flash controller while it is enabled and its
    // flash API refuses addresses outside the application area, so the UICR
    // can only be written with it stopped. Nothing else may run between
    // stopping it and the reset: the BLE tasks would find it gone.
    vTaskSuspendAll();
    sd_softdevice_disable();
    __disable_irq();

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
    }
    NRF_UICR->CUSTOMER[slot] = TAG | number;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
    }
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
    }

    NVIC_SystemReset();
}

} // namespace DeviceNumber
