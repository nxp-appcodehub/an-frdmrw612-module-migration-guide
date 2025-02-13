/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright 2008-2013 Code Red Technologies Ltd,
 * Copyright 2013-2025 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 * Generated linker script file for RW612
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v24.12 [Build 148] [2025-01-10] on Feb 13, 2025, 4:49:06 PM
 */

MEMORY
{
  /* Define each memory region */
  QSPI_FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 0x800000 /* 8M bytes (alias Flash) */  
  SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x130000 /* 1216K bytes (alias RAM) */  
}

  /* Define a symbol for the top of each memory region */
  __base_QSPI_FLASH = 0x8000000  ; /* QSPI_FLASH */  
  __base_Flash = 0x8000000 ; /* Flash */  
  __top_QSPI_FLASH = 0x8000000 + 0x800000 ; /* 8M bytes */  
  __top_Flash = 0x8000000 + 0x800000 ; /* 8M bytes */  
  __base_SRAM = 0x20000000  ; /* SRAM */  
  __base_RAM = 0x20000000 ; /* RAM */  
  __top_SRAM = 0x20000000 + 0x130000 ; /* 1216K bytes */  
  __top_RAM = 0x20000000 + 0x130000 ; /* 1216K bytes */  
