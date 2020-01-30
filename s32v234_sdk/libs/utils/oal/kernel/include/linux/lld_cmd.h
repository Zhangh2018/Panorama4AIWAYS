/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2017-2018 NXP;
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
#ifndef __HEADER_LLDCMD_H__
#define __HEADER_LLDCMD_H__

#include "utils.h"
#include "oal_types.h"

__BEGIN_DECLS

/*======================================================================*/
/* Driver global functions */
/*======================================================================*/

typedef struct
{
  uint64_t   size;
  uint64_t   align;
  uint64_t   ret_phys_pointer;
  uint8_t   chunk_id;
} CMD_ALLOC_TYPE;
/**
 * Add new allocation into the list
 */
#define CMD_ALLOC 0U

/**
 * Remove the allocation from the list
 */
#define CMD_FREE 1U
#define CMD_FREE_PHYS 2U

/**
 * Get the size of the allocation
 */
#define CMD_SIZE 3U

typedef struct
{
  uint64_t   handle_pointer;
  uint64_t   type;
  uint64_t   ret_virt_pointer;
} CMD_BUFFER_TYPE;
/**
 * Get specific buffer
 */
#define CMD_BUFFER 4U

typedef struct
{
  uint64_t   handle_pointer;
  uint64_t   type;
  uint64_t   virtual_pointer;
} CMD_MAP_TYPE;
/**
 * Create a new mapping for the current process
 */
#define CMD_MAP 5U
#define CMD_MAP_PHYS 6U

typedef struct
{
  uint64_t   type;
  uint64_t   virtual_pointer;
  uint64_t   size;
} CMD_MAP_NEW_TYPE;
#define CMD_MAP_NEW 7U

typedef struct
{
  uint64_t   virtual_pointer;
  uint64_t   size;
} CMD_FLUSH_SPECIFIC_TYPE;
#define CMD_FLUSH 8U
#define CMD_FLUSH_SPECIFIC 9U
#define CMD_INVALIDATE_SPECIFIC 10U
#define CMD_FLUSHINVALIDATE_SPECIFIC 11U

#define CMD_INFO 12U
#define CMD_INFO_MAP 13U

typedef struct
{
  uint64_t address[6];
} CMD_MAPPINGS_TYPE;
#define CMD_MAPPINGS 14U
#define CMD_MAPPINGSHW 15U

#define CMD_UNMAP 16U

#define CMD_MUTEX 17U
#define CMD_UNMUTEX 18U

#define CMD_MEMORY_SIZE_FREE_GET 19U
#define CMD_MEMORY_SIZE_TOTAL_GET 20U

#define CMD_MEMORY_GET_DEVICES 21U
#define CMD_MEMORY_GET_AUTOBALANCE 22U
#define CMD_MEMORY_GET_BASE 23U
#define CMD_MEMORY_GET_SIZE 24U

#if 0
typedef struct
{
  uint32_t irq_num;
  uint32_t mode;
  int32_t pid;
} CMD_INTERRUPT_SET_TYPE;
#define CMD_INTERRUPT_SET 25U
#define CMD_INTERRUPT_REMOVE 26U

#define CMD_INTERRUPT_ENABLE 27U
#define CMD_INTERRUPT_DISABLE 28U
#endif	/* if 0 */

#define CMD_AREA_INFO_GET 29U
#define CMD_MEMORY_ZONES_GET 30U

__END_DECLS

#endif	/* __HEADER_LLDCMD_H__ */
