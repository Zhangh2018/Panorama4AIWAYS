/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright 2015-2016 CogniVue Corporation
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

#ifndef __APEX_ALLOCATOR_H__
#define __APEX_ALLOCATOR_H__

#include "utils.h"

#if 0
 SASHBY: hard coded value bad!!!
 #define MAX_ADDRSLIST   0x8000
 #define MAX_CHUNKLIST   0x400
#endif
#define MAX_CHUNKLIST   0x8000
#define MAX_ADDRSLIST   ((MAX_CHUNKLIST -2) / 2)
#define OAL_MAX_ALLOCATOR_NUM 8

#define EXPORT extern

__BEGIN_DECLS

/* Functions */
EXPORT void         apex_allocator_mmap_init(uint8_t space_id, uint64_t base, uint64_t size);
EXPORT void         apex_allocator_mmap_deinit(uint8_t space_id);
EXPORT void*        apex_allocator_malloc(uint8_t space_id, uint64_t size, const uint64_t align);
EXPORT void         apex_allocator_free(uint8_t space_id, void *addr);

EXPORT uint64_t     apex_allocator_get_physical_base(uint8_t space_id);
EXPORT int64_t      apex_allocator_get_total_size(uint8_t space_id);
EXPORT int64_t      apex_allocator_get_free_size(uint8_t space_id);

__END_DECLS

#endif /* __APEX_ALLOCATOR_H__ */
