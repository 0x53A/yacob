MEMORY
{
  /* Application slot. Linked at the address the bootloader jumps to; the
     vector table lands at ORIGIN, which is 512-byte aligned as VTOR needs. */
  FLASH : ORIGIN = 0x0800E800, LENGTH = 70K
  RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}
