/******************************************************************************
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
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <devctl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/cache.h>

#include "oal.h"
#include "qnx/devctl_cmd.h"
#include "oal_allocator.h"
#include "oal_memory_driver_if.h"
#include "os_oal_check.h"
#include "vdb_log.h"

int32_t oal_fd = -1;

/* memory zones file descriptors */
static int32_t memory_zone_fd[OAL_MAX_ALLOCATOR_NUM] = {0};

/* memory zone info */
static oal_mem memory_zones[OAL_MAX_ALLOCATOR_NUM];

/* cache info structure */
static struct cache_ctrl cinfo;

uint64_t OAL_MemoryGetBase(uint8_t chunk_id);

static void wait_for_mutex(void)
{
    if (devctl (oal_fd, CMD_MUTEX, NULL, 0, NULL) != EOK)
    {
        VDB_LOG_WARNING("OAL: failed to wait for mutex\n");
    }
}

static void release_mutex(void)
{
    if (devctl(oal_fd, CMD_UNMUTEX, NULL, 0, NULL) != EOK)
    {
        VDB_LOG_WARNING("OAL: failed to release mutex\n");
    }
}

/* Initializes OAL */
int32_t OAL_Initialize(void)
{
    uint8_t i;
    int32_t retval = OAL_SUCCESS;

    do {
        if (oal_fd < 0)
        {
            oal_fd = open("/dev/oal", O_RDWR);
            if (oal_fd < 0)
            {
                retval = OAL_FAILURE;
                continue;
            }
        }
        vdb_LogInit();      /* enable logging */
        if (devctl(oal_fd, CMD_MEMORY_ZONES_GET, memory_zones, sizeof(memory_zones), NULL) != EOK)
        {
            VDB_LOG_ERROR("OAL: failed to get memory zones\n");
            retval = OAL_FAILURE;
            continue;
        }

        /* open each as */
        for (i=0; i<OAL_MAX_ALLOCATOR_NUM; i++)
        {
            if (!is_memory_zone_empty(memory_zones[i]))
            {
                if (memory_zone_fd[i] <= 0)
                {
                    memory_zone_fd[i] = posix_typed_mem_open(memory_zones[i].name, O_RDWR,
                                                             POSIX_TYPED_MEM_MAP_ALLOCATABLE);
                    if (memory_zone_fd[i] < 0)
                    {
                        retval = OAL_FAILURE;
                        break;
                    }
                }
            }
        }
        if (retval != OAL_FAILURE) {
            /* initialize cache info structure */
            if (cache_init(0, &cinfo, NULL) == -1)
            {
                VDB_LOG_ERROR("OAL: Failed to initialize cache info\n");
                retval = OAL_FAILURE;
            }
        }
    } while (0);

    return retval;
}

/* Deinitializes OAL */
int32_t OAL_Deinitialize(void)
{
    uint8_t i;
    int32_t retval = OAL_SUCCESS;
    if (oal_fd >= 0)
    {
        uint64_t num = OAL_ContAllocGetNumAllocations();
        if (num != 0)
        {
            VDB_LOG_WARNING("OAL: Close while %d allocations remain not freed!\n",
                            (int32_t)num);
            retval = OAL_FAILURE;
        }
        else
        {
            num = OAL_ContAllocGetNumMappings();
            if (num != 0)
            {
                VDB_LOG_WARNING("OAL: Close while %d user space mappings remain not freed!\n",
                                (int32_t)num);
                retval = OAL_FAILURE;
            }
        }

        /* close memory zones */
        for (i=0; i<OAL_MAX_ALLOCATOR_NUM; i++)
        {
            if (memory_zone_fd[i] > 0)
            {
                close(memory_zone_fd[i]);
                memory_zone_fd[i] = -1;
            }
        }

        close(oal_fd);
        oal_fd = -1;
        vdb_LogStop();
    }

    /* clear cache info */
    cache_fini(&cinfo);

    return retval;
}

/* Returns a number of current allocations from all processes */
uint64_t OAL_ContAllocGetNumAllocations(void)
{
    uint64_t retval;
    if (oal_fd >= 0)
    {

        if(devctl (oal_fd, CMD_INFO, &retval, sizeof(retval), NULL) != EOK) {
            retval = 0xFFFFFFFFFFFFFFFF;
        }
    }
    else
    {
        retval = 0xFFFFFFFFFFFFFFFF;
    }
    return retval;
}

/* Returns a number of current mappings from all processes */
uint64_t OAL_ContAllocGetNumMappings(void)
{
    uint64_t retval;
    if (oal_fd >= 0)
    {
        if(devctl (oal_fd, CMD_INFO_MAP, &retval, sizeof(retval), NULL) != EOK) {
            retval = 0xFFFFFFFFFFFFFFFF;
        }
    }
    else
    {
        retval = 0xFFFFFFFFFFFFFFFF;
    }
    return retval;
}

/* Allocates a block of memory with specified size and alignment.
   Uses underlying oal kernel module to do so */
void * OAL_ContAllocAlloc(uint64_t size, uint64_t align, uint8_t chunk_id)
{
    void *retval = NULL_ADDRESS;
    if (oal_fd >= 0)
    {
        CMD_ALLOC_TYPE aux;
        aux.size = size;
        aux.align = align;
        aux.chunk_id = chunk_id;

        if(devctl (oal_fd, CMD_ALLOC, &aux, sizeof(aux), NULL) == EOK)
        {
            retval = (void *)aux.ret_phys_pointer;
        }
    }
    return retval;
}

/* Free mapped memory from underlying oal kernel module
   If memory lacks the physical pointer (allocated externally),
   it's freed using free() */
int32_t OAL_ContAllocFree(void *handle_pointer)
{
    int32_t retval = OAL_SUCCESS;
    if (oal_fd >= 0)
    {
        wait_for_mutex();

        CMD_MAPPINGS_TYPE aux;
        aux.address[ACCESS_PHY] = (uint64_t)handle_pointer;

        /* continue only if get mappings is OK */
        if(devctl (oal_fd, CMD_MAPPINGS, &aux, sizeof(aux), NULL) == EOK)
        {
            /* 1..4 are mappings (0 is physical, ACCESS_NUM is size) */
            for (int32_t i = 1; i < ACCESS_NUM; ++i)
            {
                if (aux.address[i] != 0)
                {
                    if (munmap((void *)aux.address[i], aux.address[ACCESS_NUM]) != 0)
                    {
                        VDB_LOG_ERROR("OAL: Failed to unmap memory\n");
                        retval = OAL_FAILURE;
                    }
                }
            }

            /* free the memory */
            if(devctl (oal_fd, CMD_FREE, &handle_pointer, sizeof(uint64_t), NULL) != EOK)
            {
                retval = OAL_FAILURE;
            }

            if (handle_pointer != 0)
            {
                retval = OAL_FAILURE;
            }
        }
        else
        {
            retval = OAL_FAILURE;
        }

        release_mutex();
    }
    else
    {
        retval = OAL_FAILURE;
    }

    return retval;
}

int32_t OAL_ContAllocUnmap(void *handle_pointer)
{
    int32_t retval = OAL_SUCCESS;
    if (oal_fd >= 0)
    {
        wait_for_mutex();

        CMD_MAPPINGS_TYPE aux;
        aux.address[ACCESS_PHY] = (uint64_t)handle_pointer;

        /* continue only if get mappings is OK */
        if(devctl (oal_fd, CMD_MAPPINGS, &aux, sizeof(aux), NULL) == EOK)
        {
            /* 1..4 are mappings (0 is physical, ACCESS_NUM is size) */
            for (int32_t i = 1; i < ACCESS_NUM; ++i)
            {
                if (aux.address[i] != 0)
                {
                    if (munmap((void *)aux.address[i], aux.address[ACCESS_NUM]) != 0)
                    {
                        VDB_LOG_ERROR("OAL: Failed to unmap memory\n");
                        retval = OAL_FAILURE;
                    }
                }
            }

            if(devctl (oal_fd, CMD_UNMAP, &handle_pointer, sizeof(uint64_t), NULL) != EOK)
            {
                retval = OAL_FAILURE;
            }
        }
        else
        {
            retval = OAL_FAILURE;
        }

        release_mutex();
    }
    else
    {
        retval = OAL_FAILURE;
    }

    return retval;
}

/* Returns a buffer of specified type.
   If type is not specified or cannot be mapped, returns NULL
*/
void *OAL_ContAllocGetBuffer(uint64_t handle_pointer, uint64_t type)
{
    void *vadr = NULL_ADDRESS;
    if (oal_fd >= 0)
    {
        wait_for_mutex();

        CMD_MAPPINGS_TYPE aux;
        CMD_ALLOC_TYPE area;
        area.ret_phys_pointer = handle_pointer;
        aux.address[ACCESS_PHY] = handle_pointer;

        if(devctl (oal_fd, CMD_MAPPINGSHW, &aux, sizeof(aux), NULL) == EOK)
        {
            /* if NCH_NB wasn't mapped yet and we want it (and we can, ACCESS_PHY exists),
             * we map it as non cache */
            if ((aux.address[type] == NULL_ADDRESS) && (aux.address[ACCESS_PHY] != NULL_ADDRESS))
            {
                bool ok = true;
                for (int32_t i = 1; i < ACCESS_NUM; ++i)
                {
                    if (aux.address[i] != 0)
                    {
                        VDB_LOG_WARNING("OAL: Multiple memory mappings detected. "
                                "Please consider revision of the code to prevent "
                                "different caching settings.\n");
                        ok = false;
                        break;
                    }
                }
                if (ok)
                {
                    if (devctl (oal_fd, CMD_AREA_INFO_GET, &area, sizeof(area), NULL) == EOK)
                    {
                        /* get virtual address */
                        if ((type == ACCESS_NCH_NB) || (type == ACCESS_NCH_B))
                        {
                            vadr = mmap(NULL, aux.address[ACCESS_NUM],
                                        PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_SHARED,
                                        memory_zone_fd[area.chunk_id], aux.address[ACCESS_PHY]);
                        }
                        else if  ((type == ACCESS_CH_WB) || (type == ACCESS_CH_WT))
                        {
                            vadr = mmap(NULL, aux.address[ACCESS_NUM],
                                        PROT_READ | PROT_WRITE, MAP_SHARED,
                                        memory_zone_fd[area.chunk_id], aux.address[ACCESS_PHY]);
                        }
                        else
                        {
                            vadr = MAP_FAILED;
                        }

                        if (vadr != MAP_FAILED)
                        {
                            /* create an input structure */
                            CMD_MAP_TYPE map_aux;
                            map_aux.handle_pointer = aux.address[ACCESS_PHY];
                            map_aux.type = type;
                            map_aux.virtual_pointer = (uint64_t)vadr;

                            /* insert new mapping to kernel module */
                            if(devctl (oal_fd, CMD_MAP_PHYS, &map_aux, sizeof(map_aux), NULL) != EOK)
                            {
                                vadr = NULL_ADDRESS;
                            }
                            else
                            {
                                /* if all ok, we return virtual address */
                                if (!map_aux.virtual_pointer)
                                {
                                    munmap((void *)map_aux.virtual_pointer, 
                                           aux.address[ACCESS_NUM]);
                                    vadr = NULL_ADDRESS;
                                }
                            }
                        }
                        else
                        {
                            VDB_LOG_ERROR("OAL: Failed to get virtual address\n");
                            vadr = NULL_ADDRESS;
                        }
                    }
                    else
                    {
                        VDB_LOG_ERROR("OAL: Failed to get memory zone id\n");
                        vadr = NULL_ADDRESS;
                    }
                }
            }
            else
            {
                vadr = (void *)aux.address[type];
            }
        }
        release_mutex();
    }
    return vadr;
}

/* Function returns total size in bytes being managed by OAL Memory.
 * return:  total size in bytes being managed by OAL Memory */
int64_t OAL_ContAllocGetTotalSize(void)
{
  int64_t retval = -1;
  if (oal_fd >= 0)
  {
    if (devctl (oal_fd, CMD_MEMORY_SIZE_TOTAL_GET, &retval, sizeof(retval), NULL) != EOK) {
      retval = -1;
    }
  }
  return retval;
}

/* Function returns free size in bytes remaining in OAL Memory.
* return:	free size in bytes remaining in OAL Memory */
int64_t OAL_ContAllocGetFreeSize(void)
{
  int64_t retval = -1;
  if (oal_fd >= 0)
  {
    if (devctl (oal_fd, CMD_MEMORY_SIZE_FREE_GET, &retval, sizeof(retval), NULL) != EOK) {
      retval = -1;
    }
  }
  return retval;
}

uint8_t OAL_MemoryGetDevicesMask(void)
{
  uint8_t retval = 0;
  int32_t retinit = OAL_SUCCESS;

  CHECK_INIT(retinit)

  if (retinit == OAL_SUCCESS)
  {
    if (devctl (oal_fd, CMD_MEMORY_GET_DEVICES, &retval, sizeof(retval), NULL) != EOK) {
      retval = 0;
    }
  }
  return retval;
}

uint8_t OAL_MemoryGetAutobalanceMask(void)
{
  uint8_t retval = 0;
  if (oal_fd >= 0)
  {
    if (devctl (oal_fd, CMD_MEMORY_GET_AUTOBALANCE, &retval, sizeof(retval), NULL) != EOK) {
      retval = 0;
    }
  }
  return retval;
}

uint64_t OAL_MemoryGetBase(uint8_t chunk_id)
{
  uint64_t retval = 0;
  if (oal_fd >= 0)
  {
    retval = chunk_id;
    if (devctl (oal_fd, CMD_MEMORY_GET_BASE, &retval, sizeof(retval), NULL) != EOK) {
      retval = 0;
    }
  }
  return retval;
}

uint64_t OAL_MemoryGetsize(uint8_t chunk_id)
{
  uint64_t retval = 0;
  if (oal_fd >= 0)
  {
    retval = chunk_id;
    if (devctl (oal_fd, CMD_MEMORY_GET_SIZE, &retval, sizeof(retval), NULL) != EOK) {
      retval = 0;
    }
  }
  return retval;
}

/* Flushes and invalidates the whole cache */
void CleanAndInvalidateDCacheAll(void)
{
    if ((msync((void *)memory_zones[0].start_address, memory_zones[0].size, MS_SYNC) != 0) ||
        (msync((void *)memory_zones[1].start_address, memory_zones[1].size, MS_SYNC) != 0))
    {
         VDB_LOG_WARNING("OAL: Failed to flush and invalidate whole cache\n");
    }
}

/* Flushes and invalidates the specified cache */
void CleanAndInvalidateDCache(void *ptr, uint32_t size)
{
    off64_t phys_addr;
    if (mem_offset64((void *)ptr, NOFD, size, &phys_addr, NULL) != 0)
    {
        VDB_LOG_WARNING("OAL: Failed to get physical address\n");
    }

    CACHE_FLUSH(&cinfo, ptr, phys_addr, size);
    CACHE_INVAL(&cinfo, ptr, phys_addr, size);
}

/* Invalidates the specified cache */
void InvalidateDCache(void *ptr, uint32_t size)
{
    off64_t phys_addr;
    if (mem_offset64((void *)ptr, NOFD, size, &phys_addr, NULL) != 0)
    {
        VDB_LOG_WARNING("OAL: Failed to get physical address\n");
    }

    CACHE_INVAL(&cinfo, ptr, phys_addr, size);
}

/* Flushes the specified cache */
void CleanDCache(void *ptr, uint32_t size)
{
    off64_t phys_addr;
    if (mem_offset64((void *)ptr, NOFD, size, &phys_addr, NULL) != 0)
    {
        VDB_LOG_WARNING("OAL: Failed to get physical address\n");
    }

    CACHE_FLUSH(&cinfo, ptr, phys_addr, size);
}
