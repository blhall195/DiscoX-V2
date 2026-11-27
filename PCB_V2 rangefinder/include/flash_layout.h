#pragma once

#include <stdint.h>

// nRF52840 internal-flash partition map (1 MB total, 4 KB erase pages).
// Must stay in sync with linker/nrf52840_s140_v6_usbfat.ld, which caps the
// application region so it can never grow into the history ring.
//
//   0x00000 - 0x26000   SoftDevice S140
//   0x26000 - 0xC5000   application (636 KB; ~450 KB used as of 2026-11)
//   0xC5000 - 0xCD000   reading history ring (32 KB, 8 pages) ← HISTORY_*
//   0xCD000 - 0xED000   FAT12 USB-drive partition (128 KB) ← FAT_*
//   0xED000 - 0xF4000   InternalFS LittleFS (28 KB, fixed by the core)
//   0xF4000 - 0x100000  Adafruit UF2 bootloader + settings
//
// The history ring and the FAT partition survive firmware updates: both UF2
// and nrfutil DFU only touch the pages of the application image itself. The
// ring was carved from the app region's unused tail on 2026-11-23; on a
// device updated from older firmware it starts out erased (or holding stale
// image bytes, which fail the record CRC — see history_log.h).

namespace FlashLayout {
constexpr uint32_t PAGE_SIZE = 4096; // nRF52 erase page

constexpr uint32_t HISTORY_START = 0xC5000;
constexpr uint32_t HISTORY_SIZE = 32 * 1024; // 8 × 4 KB pages
constexpr uint32_t HISTORY_END = HISTORY_START + HISTORY_SIZE;

constexpr uint32_t FAT_START = 0xCD000;            // first byte of FAT partition
constexpr uint32_t FAT_SIZE = 128 * 1024;          // 32 × 4 KB pages
constexpr uint32_t FAT_END = FAT_START + FAT_SIZE; // == LittleFS start (0xED000)
constexpr uint32_t SECTOR_SIZE = 512;              // FAT sector size
constexpr uint32_t SECTOR_COUNT = FAT_SIZE / SECTOR_SIZE;

static_assert(HISTORY_END == FAT_START, "history ring must butt up against the FAT partition");
static_assert(HISTORY_START % PAGE_SIZE == 0, "history ring must be page-aligned");
static_assert(HISTORY_SIZE % PAGE_SIZE == 0, "history ring must be a whole number of pages");
static_assert(FAT_END == 0xED000, "FAT partition must butt up against InternalFS LittleFS");
static_assert(FAT_START % PAGE_SIZE == 0, "FAT partition must be page-aligned");
static_assert(FAT_SIZE % PAGE_SIZE == 0, "FAT partition must be a whole number of pages");
} // namespace FlashLayout
