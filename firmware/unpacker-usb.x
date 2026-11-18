/* unpacker.x with the RAM code in SRAMH, so SRAM1 + SRAM2 keep room for the stack. */
MEMORY
{
  FLASH : ORIGIN = 0x00010000, LENGTH = 0x333fc
  RAM : ORIGIN = 0x20000000, LENGTH = 0x1FF00
  RAMCODE : ORIGIN = 0x1FFF9000, LENGTH = 0x7000
}

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
  } > RAMCODE AT> FLASH
} INSERT BEFORE .text;

__siramcode = LOADADDR(.ramcode);
_stext = ALIGN(__siramcode + SIZEOF(.ramcode), 8);
