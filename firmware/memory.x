MEMORY
{
  /* Executable application space ends at the container descriptor at 0x433fc,
   * before the checksum at 0x4340c. Packaging generates zero fill and footer
   * metadata; the payload ends at 0x43640 and its erase envelope at 0x44000.
   * Bootloader below 0x10000 and settings at [0x72000, 0x80000) are untouched.
   */
  FLASH : ORIGIN = 0x00010000, LENGTH = 0x333fc
  /* SRAMH (32 KiB) upper half + SRAM1 (64 KiB). The origin leaves the original
   * firmware's clock bookkeeping words at 0x1FFF82A4/0x1FFF8C54 outside this image's
   * .bss, and the length stops 256 bytes short of SRAM1's exclusive end so the
   * initial stack pointer is strictly inside SRAM1. The 64.8 KiB frame buffer
   * plus stack fit comfortably; SRAM3 is left untouched.
   */
  RAM : ORIGIN = 0x1FFF9000, LENGTH = 0x15F00
  /* SRAM2 (64 KiB) holds only the 64.8 KiB frame buffer, so .bss shrinks and
   * the stack at the top of RAM gets ~72 KiB instead of ~7.9 KiB (the deepest
   * PD paths overflowed into the timer queue; see docs/SAFETY.md). SRAM1 and
   * SRAM2 share the SRAM12 wait-state field (SRAM_WTCR), so the timing is the
   * same as SRAM1. SRAM3 (ECC errata) stays unused.
   */
  SRAM2 : ORIGIN = 0x20010000, LENGTH = 0x10000
}

SECTIONS
{
  .sram2 (NOLOAD) : ALIGN(4)
  {
    *(.sram2 .sram2.*);
  } > SRAM2
} INSERT AFTER .uninit;

/* cortex-m-rt takes these from the location counter after .uninit, which is
 * now in SRAM2: keep the stack (and the unused heap start) in RAM. */
_stack_end = __euninit;
__sheap = __euninit;
