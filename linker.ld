/* STM32F103C8/CB: 64K flash, 20K RAM.

   The last 1K page is not in the FLASH region. store.c keeps the saved
   settings there, and the one way to be sure a save never erases a piece of
   the running code is for the linker to refuse to put code there: a build
   that grows past 63K fails to link rather than failing in the field. */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 63K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Where store.c writes. Kept here so that the address and the region it is
   carved out of are stated in one place. */
_store_page = ORIGIN(FLASH) + 63K;

ENTRY(Reset_Handler)

/* Top of stack: goes into the first word of the vector table. */
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  /* The vector table must be the first thing in flash and must survive
     --gc-sections, hence its own section plus KEEP. If it ever gets
     stripped, the CPU loads a garbage stack pointer at reset and dies
     before main() - while the host still sees a USB device because of the
     board's D+ pull-up, so the symptom looks like a USB bug. */
  .isr_vector :
  {
    KEEP(*(.isr_vector))
  } > FLASH

  .text :
  {
    *(.text*)
    *(.rodata*)
    . = ALIGN(4);
    _sidata = .;          /* flash image of .data, copied by Reset_Handler */
  } > FLASH

  .data : AT(_sidata)
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) }
}
