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
#ifndef __HEADER_DEVCTLCMD_H__
#define __HEADER_DEVCTLCMD_H__

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
#define CMD_ALLOC 					_IOWR('m',  0U, CMD_ALLOC_TYPE)

/**
 * Remove the allocation from the list
 */
#define CMD_FREE 					_IOWR('m',  1U, uint64_t)
#define CMD_FREE_PHYS 				_IOW('m',  2U, uint64_t)

/**
 * Get the size of the allocation
 */
#define CMD_SIZE 					_IOWR('m',  3U, uint64_t)

typedef struct
{
  uint64_t   handle_pointer;
  uint64_t   type;
  uint64_t   ret_virt_pointer;
} CMD_BUFFER_TYPE;
/**
 * Get specific buffer
 */
#define CMD_BUFFER					_IOWR('m',  4U, CMD_BUFFER_TYPE)

typedef struct
{
  uint64_t   handle_pointer;
  uint64_t   type;
  uint64_t   virtual_pointer;
} CMD_MAP_TYPE;
/**
 * Create a new mapping for the current process
 */
#define CMD_MAP 					_IOWR('m',  5U, CMD_MAP_TYPE)
#define CMD_MAP_PHYS 				_IOWR('m',  6U, CMD_MAP_TYPE)

typedef struct
{
  uint64_t   type;
  uint64_t   virtual_pointer;
  uint64_t   size;
} CMD_MAP_NEW_TYPE;
#define CMD_MAP_NEW 				_IOWR('m',  7U, CMD_MAP_NEW_TYPE)

typedef struct
{
  uint64_t   virtual_pointer;
  uint64_t   size;
} CMD_FLUSH_SPECIFIC_TYPE;
#define CMD_FLUSH					_IOW('m',  8U, int)
#define CMD_FLUSH_SPECIFIC 			_IOW('m',  9U, CMD_FLUSH_SPECIFIC_TYPE)
#define CMD_INVALIDATE_SPECIFIC 	_IOW('m', 10U, CMD_FLUSH_SPECIFIC_TYPE)
#define CMD_FLUSHINVALIDATE_SPECIFIC _IOW('m', 11U, CMD_FLUSH_SPECIFIC_TYPE)

#define CMD_INFO					_IOWR('m', 12U, int64_t)
#define CMD_INFO_MAP				_IOWR('m', 13U, int64_t)

typedef struct
{
  uint64_t address[6];
} CMD_MAPPINGS_TYPE;
#define CMD_MAPPINGS				_IOWR('m', 14U, CMD_MAPPINGS_TYPE)
#define CMD_MAPPINGSHW				_IOWR('m', 15U, CMD_MAPPINGS_TYPE)

#define CMD_UNMAP 					_IOWR('m', 16U, int64_t)

#define CMD_MUTEX					_IO('m', 17U)
#define CMD_UNMUTEX					_IO('m', 18U)

#define CMD_MEMORY_SIZE_FREE_GET	_IOWR('m', 19U, int64_t)
#define CMD_MEMORY_SIZE_TOTAL_GET	_IOWR('m', 20U, int64_t)

#define CMD_MEMORY_GET_DEVICES		_IOWR('m', 21U, uint8_t)
#define CMD_MEMORY_GET_AUTOBALANCE	_IOWR('m', 22U, uint8_t)
#define CMD_MEMORY_GET_BASE			_IOWR('m', 23U, uint64_t)
#define CMD_MEMORY_GET_SIZE			_IOWR('m', 24U, uint64_t)

#if 0
typedef struct
{
  uint32_t irq_num;
  uint32_t mode;
  int32_t pid;
} CMD_INTERRUPT_SET_TYPE;
#define CMD_INTERRUPT_SET			_IOR('m', 25U, int)
#define CMD_INTERRUPT_REMOVE		_IOR('m', 26U, int)

#define CMD_INTERRUPT_ENABLE		_IOR('m', 27U, int)
#define CMD_INTERRUPT_DISABLE		_IOR('m', 28U, int)
#endif	/* if 0 */

#define CMD_AREA_INFO_GET			_IOWR('m', 29U, CMD_ALLOC_TYPE)
#define CMD_MEMORY_ZONES_GET		_IOWR('m', 30U, oal_mem)

__END_DECLS

#endif	/* __HEADER_DEVCTLCMD_H__ */
