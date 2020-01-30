/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef OAL_KALLOC_H
#define OAL_KALLOC_H

/* Unified interface for Kernel memory allocation */

#include "priv_oal_kalloc.h"
#include "common_stringify_macros.h"
#include XSTR(../libs/utils/oal/kernel/include/internal/OSDIR/os_oal_kalloc.h)

__BEGIN_DECLS
/**
 * @brief OAL_Kalloc Allocate kernel memory
 *
 * @param size[in]  Size of memory area
 * @param flags[in] Allocation flags
 * @see kflags
 *
 * @return Pointer to the allocated memory or NULL
 * on error.
 */
void *OAL_Kalloc(size_t size, unsigned flags);

/**
 * @brief OAL_Kzalloc Allocate memory and set it to zero.
 *
 * @param size[in]  Size of memory area
 * @param flags[in] Allocation flags
 * @see kflags
 *
 * @return Pointer to the allocated memory or NULL
 * on error.
 */
void *OAL_Kzalloc(size_t size, unsigned flags);

/**
 * @brief OAL_Kfree Free the memory allocated with one of the
 * above functions.
 *
 * @param addr Pointer to the allocated memory
 * @param flags[in] Allocation flags used for allocation
 */
void OAL_Kfree(void *addr, unsigned flags);

__END_DECLS
#endif /* OAL_KALLOC_H */


