MEMORY
{
  /* The top 4 KiB (0x2007F000..0x20080000) is the shared IPC mailbox,
     reachable from the network core. */
  FLASH : ORIGIN = 0x00000000, LENGTH = 1M
  RAM : ORIGIN = 0x20000000, LENGTH = 508K
}
