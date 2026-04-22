MEMORY
{
  /* Bootloader lives in first 16K, application starts at 0x0800_4000 */
  FLASH : ORIGIN = 0x08004000, LENGTH = 112K
  RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}
