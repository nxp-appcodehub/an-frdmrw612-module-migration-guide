/*
 * Copyright 2017-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MFLASH_DRV_H__
#define __MFLASH_DRV_H__

#include "mflash_common.h"
#include "board.h"

/* Device specific settings */
#ifndef MFLASH_FLEXSPI
#define MFLASH_FLEXSPI FLEXSPI
#endif

#ifndef MFLASH_BASE_ADDRESS
#define MFLASH_BASE_ADDRESS (FlexSPI_AMBA_PC_CACHE_BASE)
#endif

#if (BOARD_FLASH == W25Q512JVFIQ) || (BOARD_FLASH == MX25U51245G)
/* Flash constants */
#ifndef MFLASH_SECTOR_SIZE
#define MFLASH_SECTOR_SIZE (4096U)
#endif

#ifndef MFLASH_PAGE_SIZE
#define MFLASH_PAGE_SIZE (256U)
#endif

#define FLASH_SIZE 0x04000000U

#elif (BOARD_FLASH == W25Q128JWYIQ) || (BOARD_FLASH == FM25M4AA_1AIBD)

/* Flash constants */
#ifndef MFLASH_SECTOR_SIZE
#define MFLASH_SECTOR_SIZE (4096U)
#endif

#ifndef MFLASH_PAGE_SIZE
#define MFLASH_PAGE_SIZE (256U)
#endif

#define FLASH_SIZE 0x01000000U

#elif (BOARD_FLASH == MX25U6432FBBI02) || (BOARD_FLASH == FM25M64C_13IBE)

/* Flash constants */
#ifndef MFLASH_SECTOR_SIZE
#define MFLASH_SECTOR_SIZE (4096U)
#endif

#ifndef MFLASH_PAGE_SIZE
#define MFLASH_PAGE_SIZE (256U)
#endif

#define FLASH_SIZE 0x00800000U

#endif

#endif
