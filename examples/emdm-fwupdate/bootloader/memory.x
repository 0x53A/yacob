MEMORY
{
  /* Immutable updater. The metadata page (0x0800_E000) and the application
     slot (0x0800_E800) are deliberately outside this region — see
     common/src/lib.rs, which is the single source of truth for the map. */
  FLASH : ORIGIN = 0x08000000, LENGTH = 56K
  RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}
