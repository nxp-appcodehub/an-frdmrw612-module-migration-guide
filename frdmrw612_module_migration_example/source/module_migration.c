/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "lfs.h"
#include "lfs_mflash.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    char ch;
    status_t status;
    lfs_t lfs;
    struct lfs_config cfg;
    lfs_file_t file;

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    PRINTF("Board migration test.\r\n");
    PRINTF("Board selected: %s\r\n", BOARD_NAME);

    CLOCK_EnableClock(kCLOCK_Flexspi);
    RESET_ClearPeripheralReset(kFLEXSPI_RST_SHIFT_RSTn);
    /* Use aux0_pll_clk / 2 */
    BOARD_SetFlexspiClock(FLEXSPI, 2U, 2U);

    lfs_get_default_config(&cfg);

    status = lfs_storage_init(&cfg);
    if (status != kStatus_Success)
    {
        PRINTF("LFS storage init failed: %i\r\n", status);
        return status;
    }

    PRINTF("Flash and littleFS drivers initialization complete.\r\n");

    // mount the filesystem
    int err = lfs_mount(&lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        lfs_format(&lfs, &cfg);
        lfs_mount(&lfs, &cfg);
    }
    // read current count
    uint32_t boot_count = 0;
    lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

    // update boot count
    boot_count += 1;
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));

    // remember the storage is not updated until the file is closed successfully
    lfs_file_close(&lfs, &file);

    // release any resources we were using
    lfs_unmount(&lfs);

    // print the boot count
    PRINTF("boot_count: %d (Read from Flash)\r\n", boot_count);

    while (1)
    {
        ch = GETCHAR();
        PUTCHAR(ch);
    }
}
