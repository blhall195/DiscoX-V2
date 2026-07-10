/* Linker script to configure memory regions.
 *
 * Copy of the core's nrf52840_s140_v6.ld with the application FLASH region
 * shortened to reserve a 128 KB FAT12 partition for the USB-drive settings
 * feature (see include/flash_layout.h — keep the two in sync):
 *
 *   0x00000 - 0x26000   SoftDevice S140
 *   0x26000 - 0xCD000   application (this region, 668 KB)
 *   0xCD000 - 0xED000   FAT12 USB-drive partition (128 KB)
 *   0xED000 - 0xF4000   InternalFS LittleFS (28 KB, core-defined)
 *   0xF4000 - 0x100000  Adafruit UF2 bootloader + settings
 *
 * If the firmware outgrows 668 KB the link FAILS here instead of the app
 * silently overlapping the FAT partition.
 */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx)     : ORIGIN = 0x26000, LENGTH = 0xCD000 - 0x26000

  /* SRAM required by Softdevice depend on
   * - Attribute Table Size (Number of Services and Characteristics)
   * - Vendor UUID count
   * - Max ATT MTU
   * - Concurrent connection peripheral + central + secure links
   * - Event Len, HVN queue, Write CMD queue
   */
  RAM (rwx) :  ORIGIN = 0x20006000, LENGTH = 0x20040000 - 0x20006000
}

SECTIONS
{
  . = ALIGN(4);
  .svc_data :
  {
    PROVIDE(__start_svc_data = .);
    KEEP(*(.svc_data))
    PROVIDE(__stop_svc_data = .);
  } > RAM

  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf52_common.ld"
