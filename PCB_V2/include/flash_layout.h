#pragma once

#include <stdint.h>

// nRF52840 internal-flash partition map (1 MB total, 4 KB erase pages).
// Must stay in sync with linker/nrf52840_s140_v6_usbfat.ld, which caps the
// application region so it can never grow into the FAT partition.
//
//   0x00000 - 0x26000   SoftDevice S140
//   0x26000 - 0xCD000   application (668 KB; ~360 KB used as of 2026-07)
//   0xCD000 - 0xED000   FAT12 USB-drive partition (128 KB) ← FlashLayout
//   0xED000 - 0xF4000   InternalFS LittleFS (28 KB, fixed by the core)
//   0xF4000 - 0x100000  Adafruit UF2 bootloader + settings
//
// The FAT partition survives firmware updates: both UF2 and nrfutil DFU only
// touch the pages of the application image itself.

namespace FlashLayout {
constexpr uint32_t FAT_START = 0xCD000;                 // first byte of FAT partition
constexpr uint32_t FAT_SIZE = 128 * 1024;               // 32 × 4 KB pages
constexpr uint32_t FAT_END = FAT_START + FAT_SIZE;      // == LittleFS start (0xED000)
constexpr uint32_t SECTOR_SIZE = 512;                   // FAT sector size
constexpr uint32_t SECTOR_COUNT = FAT_SIZE / SECTOR_SIZE;
constexpr uint32_t PAGE_SIZE = 4096;                    // nRF52 erase page

static_assert(FAT_END == 0xED000, "FAT partition must butt up against InternalFS LittleFS");
static_assert(FAT_START % PAGE_SIZE == 0, "FAT partition must be page-aligned");
static_assert(FAT_SIZE % PAGE_SIZE == 0, "FAT partition must be a whole number of pages");
} // namespace FlashLayout
