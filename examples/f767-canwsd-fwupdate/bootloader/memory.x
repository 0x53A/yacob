MEMORY
{
  /* Sector 3 (0x08018000..0x08020000) is the metadata journal. */
  FLASH : ORIGIN = 0x08000000, LENGTH = 96K
  RAM   : ORIGIN = 0x20000000, LENGTH = 512K
}
