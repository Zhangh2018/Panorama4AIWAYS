/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017 NXP
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/
#ifndef __ALLOCATION_KERNEL_OAL_H__
#define __ALLOCATION_KERNEL_OAL_H__

#include "oal_allocator.h"
#include "utils.h"

#ifndef ULONG_MAX
#define ULONG_MAX 0xFFFFFFFFFFFFFFFFUL
#endif

__BEGIN_DECLS

/**
 * @brief Allocates a contiguous memory block.
 *
 * Allocates a block of memory with specified size and alignment
 * and add an item into management list.
 *
 * @param   size    [in] Amount of memory to allocate in bytes
 * @param   align   [in] Alignment of allocated block
 * @param   chunk_id [in] Memory region where the allocation should take place (see oal_chunk_ids.h)
 * @param   pid     [in] Process id
 * @returns allocated HW address if success, 0 otherwise.
 */
uint64_t oal_alloc(uint64_t size, uint64_t align, uint8_t chunk_id, int32_t pid);


/**
 * @brief Removes a contiguous memory block.
 *
 * Function removes a contiguous memory block and
 * removes an entry from the management list.
 * Does not free virtual mappings.
 *
 * @param   handle_pointer  [in] handle to memory block
 * @returns 0 if success and ULONG_MAX if fail.
 */
uint64_t oal_free (uint64_t handle_pointer);

uint64_t oal_free_phys (uint64_t physical_pointer);
uint64_t oal_unmap (uint64_t handle_pointer);

/**
 * Function returns number of allocations.
 */
uint64_t oal_get_num_allocations (void);

/**
 * Function returns number of mappings.
 */
uint64_t oal_get_num_mappings (void);

/**
 * Function returns a size of allocated memory block.
 */
uint64_t oal_getsize(uint64_t handle_pointer);

void oal_getmappings(uint64_t handle_pointer, uint64_t mappings[6]);
void oal_getmappingshw(uint64_t handle_pointer, uint64_t mappings[6]);

/**
 * Function returns a specified buffer based on handle pointer.
 */
uint64_t oal_getbuffer(uint64_t handle_pointer, uint64_t type);

/**
 * Function adds a new memory mapping into the management list.
 */
uint64_t oal_map(uint64_t handle_pointer, uint64_t type, uint64_t virtual_pointer);
uint64_t oal_map_phys(uint64_t physical_pointer, uint64_t type, uint64_t virtual_pointer);
uint64_t oal_map_new(uint64_t type, uint64_t virtual_pointer, uint64_t size, int32_t pid);

/* Function returns total size in bytes being managed by OAL Memory */
int64_t oal_memorysizetotal(void);
/* Function returns free size in bytes remaining in OAL Memory */
int64_t oal_memorysizefree(void);

/* Function to get memory information */
uint8_t oal_get_chunkid(uint64_t handle_pointer);

__END_DECLS

#endif
