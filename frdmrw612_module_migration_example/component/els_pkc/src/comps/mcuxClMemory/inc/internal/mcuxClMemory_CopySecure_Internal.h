/*--------------------------------------------------------------------------*/
/* Copyright 2023 NXP                                                       */
/*                                                                          */
/* NXP Proprietary. This software is owned or controlled by NXP and may    */
/* only be used strictly in accordance with the applicable license terms.   */
/* By expressly accepting such terms or by downloading, installing,         */
/* activating and/or otherwise using the software, you are agreeing that    */
/* you have read, and that you agree to comply with and are bound by, such  */
/* license terms. If you do not agree to be bound by the applicable license */
/* terms, then you may not retain, install, activate or otherwise use the   */
/* software.                                                                */
/*--------------------------------------------------------------------------*/

/** @file  mcuxClMemory_CopySecure_Internal.h
 *  @brief Memory header for copy functions.
 * This header exposes functions that enable secure memory copy function.
 */

/**
 * @defgroup mcuxClMemory_Copy_Secure_Internal mcuxClMemory_Copy_Secure_Internal
 * @brief This function securely copies a memory region from @p src to @p dst when a secure copy is available.
 * @ingroup mcuxClMemory
 * @{
 */

#ifndef MCUXCLMEMORY_COPYSECURE_INTERNAL_H_
#define MCUXCLMEMORY_COPYSECURE_INTERNAL_H_

#include <mcuxClConfig.h>  // Exported features flags header

#include <mcuxClCore_Platform.h>
#include <mcuxClMemory_Constants.h>
#include <mcuxClMemory_Types.h>
#include <mcuxClToolchain.h>
#include <mcuxCsslFlowProtection.h>
#include <internal/mcuxClMemory_Copy_Internal.h>

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************
 * FUNCTIONS
 **********************************************/

/**
 * Copies a memory buffer to another location with security against fault and SPA.
 *
 * The two buffers must not overlap.
 *
 * * Data Integrity: Record(pSrc + pDst + length)
 *
 * @param[out] pDst        pointer to the buffer to be copied to.
 * @param[in]  pSrc        pointer to the buffer to copy.
 * @param[in]  length      size (in bytes) to be copied.
 *
 * @return A status code encapsulated in a flow-protection type.
 * @retval #MCUXCLMEMORY_STATUS_OK                 If @p length bytes copied at @p pDst.
 * @retval #MCUXCLMEMORY_STATUS_FAULT
 */

MCUX_CSSL_FP_FUNCTION_DEF(mcuxClMemory_copy_secure_int)
static inline MCUX_CSSL_FP_PROTECTED_TYPE(mcuxClMemory_Status_t) mcuxClMemory_copy_secure_int
(
    uint8_t * pDst,
    uint8_t const * pSrc,
    uint32_t length
)
{
    MCUX_CSSL_FP_FUNCTION_ENTRY(mcuxClMemory_copy_secure_int);

        MCUX_CSSL_FP_FUNCTION_CALL(retval, mcuxClMemory_copy_int(pDst, pSrc, length));
        MCUX_CSSL_FP_FUNCTION_EXIT(mcuxClMemory_copy_secure_int, retval, MCUX_CSSL_FP_FUNCTION_CALLED(mcuxClMemory_copy_int));
}


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MCUXCLMEMORY_COPYSECURE_INTERNAL_H_ */

/**
 * @}
 */
