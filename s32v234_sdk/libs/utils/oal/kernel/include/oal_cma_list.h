/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2014 Freescale Semiconductor
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
*****************************************************************************/
#ifndef __CMA_LIST_H__
#define __CMA_LIST_H__

#include "oal_struct_defs.h"
#include "utils.h"

/**
 * Operation OK flag
 */
#define CMA_OP_OK 1

/**
 * Operation error flag
 */
#define CMA_OP_ERROR -1

__BEGIN_DECLS

/*=============================================================================*/
/* Functions managing physical contiguous allocations                          */
/*=============================================================================*/
int32_t cma_list_init(uint32_t id, uint64_t base_address, uint64_t end_address, uint32_t alignment);

/**
 * Frees all allocated CMA lists
 */
int32_t cma_list_deinit(void);

/**
 * Pushes the new area into managed physical allocations.
 * Does not map into user space!
 * @returns CMA_OP_OK on success, CMA_OP_ERROR otherwise.
 */
int32_t cma_list_area_push_back(t_MemArea *area);

/**
 * Finds appropriate t_MemArea structure by physical address.
 * @returns pointer to this structure or NULL if not found.
 */
t_MemArea *cma_list_area_find(uint64_t addr);

/**
 * Removes a managed physical allocation from the list.
 * Successful only if there is no virtual map.
 * Does not free *area pointer!
 * @returns CMA_OP_OK on success, CMA_OP_ERROR otherwise.
 *
 */
int32_t cma_list_area_remove(t_MemArea *area);

int32_t cma_list_free_pid(uint32_t pid);

int32_t cma_list_free_all(void);

#if 0
void cma_list_reset(void);

t_MemArea * cma_list_begin(void);
#endif

__END_DECLS

#endif
