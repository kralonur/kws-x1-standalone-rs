/* The unpacker (`src/bin/unpacker.rs`) rewrites the flash it is loaded from,
 * so all its code and constants run from RAM. Only the reset path stays in
 * flash: cortex-m-rt's Reset, then `__pre_init`, which copies .ramcode to
 * RAM before cortex-m-rt initializes .bss and .data.
 */
MEMORY
{
  /* The same application range as memory.x. */
  FLASH : ORIGIN = 0x00010000, LENGTH = 0x333fc
  /* SRAM1 + SRAM2 (128 KiB, contiguous): inside the Cortex-M SRAM region, so
   * no access straddles its 0x20000000 boundary with SRAMH. The length stops
   * 256 bytes short of SRAM2's end so the initial stack pointer is strictly
   * inside it; SRAM3 (ECC errata) stays unused.
   */
  RAM : ORIGIN = 0x20000000, LENGTH = 0x1FF00
}

/* Inserted before .text so these patterns claim the sections first. */
SECTIONS
{
  .boot : ALIGN(4)
  {
    *(.text.__pre_init);
  } > FLASH

  .ramcode : ALIGN(4)
  {
    __sramcode = .;
    *(.text .text.* .rodata .rodata.*);
    . = ALIGN(4);
    __eramcode = .;
  } > RAM AT> FLASH
} INSERT BEFORE .text;

__siramcode = LOADADDR(.ramcode);
/* cortex-m-rt places .text at `_stext`, by default right after the vector
 * table: move it past the RAM code's load image. 8 covers any alignment its
 * reset and fault handlers need. */
_stext = ALIGN(__siramcode + SIZEOF(.ramcode), 8);
