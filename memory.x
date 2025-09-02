MEMORY
{
    FLASH    : ORIGIN = 0x08000000, LENGTH =  512K /* BANK_1 */
    RAM      : ORIGIN = 0x20000000, LENGTH =   80K
    SRAM2    : ORIGIN = 0x20014000, LENGTH =   16K
    CCMRAM   : ORIGIN = 0x10000000, LENGTH =   32K
}


/* # Sections */
SECTIONS
{
  /* ## Sections in RAM */
  /* ### .data */
  .data2 : ALIGN(4)
  {
    . = ALIGN(4);
    __sdata2 = .;
    *(.data2 .data2.*);
    . = ALIGN(4); /* 4-byte align the end (VMA) of this section */
      __edata2 = .;
  } > SRAM2 AT>FLASH

  /* LMA of .data */
  __sidata2 = LOADADDR(.data2);

    .ccmdata : ALIGN(4)
  {
    . = ALIGN(4);
    __sccmdata = .;
    *(.ccmdata .ccmdata.*);
    . = ALIGN(4); /* 4-byte align the end (VMA) of this section */
  __eccmdata = .;
  } > CCMRAM AT>FLASH

  /* LMA of .data */
  __siccmdata = LOADADDR(.ccmdata);
}
INSERT AFTER .uninit;