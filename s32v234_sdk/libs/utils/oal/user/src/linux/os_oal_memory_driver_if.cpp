/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2017 Freescale Semiconductor;
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
/**
* \file oal_memory_cont_alloc.cpp
* \brief OAL memory allocation implementation for Linux (contiguous allocator)
* \author Rostislav Hulik
* \version
* \date 3-March-2016
****************************************************************************/
#include <oal.h>
#include "oal.h"
#include "oal_memory_driver_if.h"
#include "linux/lld_cmd.h"
#include "os_oal_check.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "vdb_log.h"

int32_t oal_fd = -1;
static int32_t oal_fd_nc = -1;
static pthread_mutex_t oal_init_mutex = PTHREAD_MUTEX_INITIALIZER;

/////////////////////////////////////////////////////////////////////////
/// Initializes OAL
/////////////////////////////////////////////////////////////////////////
int32_t OAL_Initialize(void)
{
  int32_t retval = OAL_SUCCESS;

  pthread_mutex_lock(&oal_init_mutex);
  if (oal_fd < 0)
  {
    oal_fd = open("/dev/oal_cached", O_RDWR);
    if (oal_fd < 0)
    {
        retval = OAL_FAILURE;
    }
    else
    {
        oal_fd_nc = open("/dev/oal_noncached", O_RDWR);
        if (oal_fd_nc < 0)
        {
            retval = OAL_FAILURE;
        }
    }
  }
  vdb_LogInit();      /* enable logging */
  pthread_mutex_unlock(&oal_init_mutex);
  return retval;
}

/////////////////////////////////////////////////////////////////////////
/// Deinitializes OAL
/////////////////////////////////////////////////////////////////////////
int32_t OAL_Deinitialize(void)
{
  int32_t retval = OAL_SUCCESS;
  if (oal_fd >= 0)
  {
    uint64_t num = OAL_ContAllocGetNumAllocations();
    if (num != 0)
    {
      VDB_LOG_WARNING("OAL: Close while %d allocations remain not freed!\n", (int32_t)num);
      retval = OAL_FAILURE;
    }
    else
    {
      num = OAL_ContAllocGetNumMappings();
      if (num != 0)
      {
        VDB_LOG_WARNING("OAL: Close while %d user space mappings remain not freed!\n", (int32_t)num);
        retval = OAL_FAILURE;
      }
    }
    close(oal_fd);
    close(oal_fd_nc);
    oal_fd = -1;
    oal_fd_nc = -1;
    vdb_LogStop();
  }
  return retval;
}

/////////////////////////////////////////////////////////////////////////
/// Returns a number of current allocations from all processes
/////////////////////////////////////////////////////////////////////////
uint64_t OAL_ContAllocGetNumAllocations(void)
{
  uint64_t retval;
  if (oal_fd >= 0)
  {
    OAL_LldCmd_t lCmd = {CMD_INFO, (uintptr_t)&retval};
    if(OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) < 0)
    {
      retval = 0xFFFFFFFFFFFFFFFF;
    }
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
    retval = 0xFFFFFFFFFFFFFFFF;
  }
  return retval;
}

/////////////////////////////////////////////////////////////////////////
/// Returns a number of current mappings from all processes
/////////////////////////////////////////////////////////////////////////
uint64_t OAL_ContAllocGetNumMappings(void)
{
  uint64_t retval;
  if (oal_fd >= 0)
  {
    OAL_LldCmd_t lCmd = {CMD_INFO_MAP, (uintptr_t)&retval};
    if(OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) < 0)
    {
      retval = 0xFFFFFFFFFFFFFFFF;
    }
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
    retval = 0xFFFFFFFFFFFFFFFF;
  }
  return retval;
}

/////////////////////////////////////////////////////////////////////////
/// Allocates a block of memory with specified size and alignment
/// Uses underlying oal kernel module to do so
/////////////////////////////////////////////////////////////////////////
void * OAL_ContAllocAlloc(uint64_t size, uint64_t align, uint8_t chunk_id)
{
  void *retval = NULL_ADDRESS;
  if (oal_fd >= 0)
  {
    CMD_ALLOC_TYPE aux;
    aux.size = size;
    aux.align = align;
    aux.chunk_id = chunk_id;
    // alloc
    OAL_LldCmd_t lCmd = {CMD_ALLOC, (uintptr_t)&aux};
    if(OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) >= 0)
    {
      retval = (void *)aux.ret_phys_pointer;
    }
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
  }
  return retval;
}

/////////////////////////////////////////////////////////////////////////
/// Free mapped memory from underlying oal kernel module
/// If memory lacks the physical pointer (allocated externally), it's freed
/// using free()
/////////////////////////////////////////////////////////////////////////
int32_t OAL_ContAllocFree(void *handle_pointer)
{
  int32_t retval = OAL_SUCCESS;
  if (oal_fd >= 0)
  {
    ///////////////////////////////////////////////
    // SAFE SECTION
    CMD_MAPPINGS_TYPE aux;
    aux.address[ACCESS_PHY] = (uint64_t)handle_pointer;

    // Continue only if get mappings is OK
    OAL_LldCmd_t lCmd = {CMD_MAPPINGS, (uintptr_t)&aux};
    if(OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) >= 0)
    {
      // 1..4 are mappings (0 is physical, ACCESS_NUM is size)
      for (int32_t i = 1; i < ACCESS_NUM; ++i) {
        if (aux.address[i] != 0)
        {
          munmap((void *)aux.address[i], aux.address[ACCESS_NUM]);
        }
      }
      // Free the memory
      lCmd.mCommandId = CMD_FREE;
      lCmd.mArg       = (uintptr_t)&handle_pointer;
      if(OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) < 0)
      {
        retval = OAL_FAILURE;
      }

      if (handle_pointer != 0) {
        retval = OAL_FAILURE;
      }
    }
    else
    {
      retval = OAL_FAILURE;
    }
    ///////////////////////////////////////////////
    // END SAFE SECTION
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
    retval = OAL_FAILURE;
  }
  return retval;
}

int32_t OAL_ContAllocUnmap(void *handle_pointer)
{
  int32_t retval = OAL_SUCCESS;
  if (oal_fd >= 0)
  {
    ///////////////////////////////////////////////
    // SAFE SECTION
    CMD_MAPPINGS_TYPE aux;
    aux.address[ACCESS_PHY] = (uint64_t)handle_pointer;

    // Continue only if get mappings is OK
    OAL_LldCmd_t lCmd = {CMD_MAPPINGS, (uintptr_t)&aux};
    if(OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) >= 0)
    {
      // 1..4 are mappings (0 is physical, ACCESS_NUM is size)
      for (int32_t i = 1; i < ACCESS_NUM; ++i) {
        if (aux.address[i] != 0)
        {
          munmap((void *)aux.address[i], aux.address[ACCESS_NUM]);
        }
      }
      lCmd.mCommandId = CMD_UNMAP;
      lCmd.mArg       = (uintptr_t)&handle_pointer;
      if(OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) < 0)
      {
        retval = OAL_FAILURE;
      }
    }
    else
    {
      retval = OAL_FAILURE;
    }
    ///////////////////////////////////////////////
    // END SAFE SECTION
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
    retval = OAL_FAILURE;
  }
  return retval;
}

/////////////////////////////////////////////////////////////////////////
/// Returns a buffer of specified type.
/// If type is not specified or cannot be mapped, returns NULL
/////////////////////////////////////////////////////////////////////////
void *OAL_ContAllocGetBuffer(uint64_t handle_pointer, uint64_t type)
{
  uint64_t vadr = NULL_ADDRESS;
  if (oal_fd >= 0)
  {
    ///////////////////////////////////////////////
    // SAFE SECTION
    CMD_MAPPINGS_TYPE aux;
    aux.address[ACCESS_PHY] = (uint64_t)handle_pointer;
    OAL_LldCmd_t lCmd = {CMD_MAPPINGSHW, (uintptr_t)&aux};
    if(OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) >= 0)
    {
      if (aux.address[ACCESS_PHY] != NULL_ADDRESS)
      {
        /* if NCH_NB wasn't mapped yet and we want it (and we can, ACCESS_PHY exists),
           we map it from non-cached file */
        if (aux.address[type] == NULL_ADDRESS)
        {
          bool ok = true;
        for (uint32_t i = 1; i < ACCESS_NUM; ++i)
          {
            if (((uint64_t)i != type) && (aux.address[i] != 0))
            {
              #if 0
              printf("%d, %d\n", i, type);
              printf("OAL WARNING: Multiple memory mappings detected. "
                      "Please consider revision of the code to prevent different caching settings.\n");
              #endif  /* #if 0 */
              ok = false;
              break;
            }
          }
          if (ok)
          {
            if ((type == ACCESS_NCH_NB) || (type == ACCESS_NCH_B))
            {
                vadr = (uint64_t)mmap(0, aux.address[ACCESS_NUM],
                                      PROT_READ | PROT_WRITE | PROT_EXEC, MAP_SHARED,
                                      oal_fd_nc, aux.address[ACCESS_PHY]);
            }
            else if ((type == ACCESS_CH_WB) || (type == ACCESS_CH_WT))
            {
                vadr = (uint64_t)mmap(0, aux.address[ACCESS_NUM],
                                      PROT_READ | PROT_WRITE | PROT_EXEC, MAP_SHARED,
                                      oal_fd, aux.address[ACCESS_PHY]);
            }
            else
            {
                vadr = NULL_ADDRESS;
            }

            if (vadr > NULL_ADDRESS)
            {
              // Create an input structure and open file
              CMD_MAP_TYPE map_aux;
              map_aux.handle_pointer = aux.address[ACCESS_PHY];
              map_aux.type = type;
              map_aux.virtual_pointer = (uint64_t)vadr;

              // insert new mapping to kernel module
              lCmd.mCommandId = CMD_MAP_PHYS;
              lCmd.mArg       = (uintptr_t)&map_aux;
              if(OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) < 0)
              {
                vadr = NULL_ADDRESS;
              }
              else
              {
                // If all ok, we return virtual address
                if (!(bool)map_aux.virtual_pointer)
                {
                  munmap((void *)map_aux.virtual_pointer, aux.address[ACCESS_NUM]);
                  vadr = NULL_ADDRESS;
                }
              }
            }
          }
        }
        else
        {
        vadr = (uint64_t)aux.address[type];
        }
      }
    }
    ///////////////////////////////////////////////
    // SAFE SECTION
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
  }
  return (void *)vadr;
}

/////////////////////////////////////////////////////////////////////////
///
/// Function returns total size in bytes being managed by OAL Memory.
///
/// \return
///    total size in bytes being managed by OAL Memory
/////////////////////////////////////////////////////////////////////////
int64_t OAL_ContAllocGetTotalSize(void)
{
  int64_t retval = -1;
  if (oal_fd >= 0)
  {
    OAL_LldCmd_t lCmd = {CMD_MEMORY_SIZE_TOTAL_GET, (uintptr_t)&retval};
    if (OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) < 0)
    {
      retval = -1;
    }
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
  }
  return retval;
}

/////////////////////////////////////////////////////////////////////////
///
/// Function returns free size in bytes remaining in OAL Memory.
///
/// \return
///    free size in bytes remaining in OAL Memory
/////////////////////////////////////////////////////////////////////////
int64_t OAL_ContAllocGetFreeSize(void)
{
  int64_t retval = -1;
  if (oal_fd >= 0)
  {
    OAL_LldCmd_t lCmd = {CMD_MEMORY_SIZE_FREE_GET, (uintptr_t)&retval};
    if (OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) < 0)
    {
      retval = -1;
    }
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
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
    OAL_LldCmd_t lCmd = {CMD_MEMORY_GET_DEVICES, (uintptr_t)&retval};
    if (OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) < 0)
    {
      retval = 0;
    }
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
  }
  return retval;
}

uint8_t OAL_MemoryGetAutobalanceMask(void)
{
  uint8_t retval = 0;
  if (oal_fd >= 0)
  {
    OAL_LldCmd_t lCmd = {CMD_MEMORY_GET_AUTOBALANCE, (uintptr_t)&retval};
    if (OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) < 0)
    {
      retval = 0;
    }
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
  }
  return retval;
}

uint64_t OAL_MemoryGetBase(uint8_t chunk_id)
{
  uint64_t retval = 0;
  if (oal_fd >= 0)
  {
    retval = chunk_id;
    OAL_LldCmd_t lCmd = {CMD_MEMORY_GET_BASE, (uintptr_t)&retval};
    if (OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) < 0)
    {
      retval = 0;
    }
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
  }
  return retval;
}

uint64_t OAL_MemoryGetsize(uint8_t chunk_id)
{
  uint64_t retval = 0;
  if (oal_fd >= 0)
  {
    retval = chunk_id;
    OAL_LldCmd_t lCmd = {CMD_MEMORY_GET_SIZE, (uintptr_t)&retval};
    if (OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd) < 0)
    {
      retval = 0;
    }
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
  }
  return retval;
}

/////////////////////////////////////////////////////////////////////////
/// Flushes and invalidates the whole cache
/////////////////////////////////////////////////////////////////////////
void CleanAndInvalidateDCacheAll(void)
{
  if (oal_fd >= 0)
  {
    OAL_LldCmd_t lCmd = {CMD_FLUSH, 0};
    OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd);
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
  }
}

/////////////////////////////////////////////////////////////////////////
/// Flushes and invalidates the specified cache
/////////////////////////////////////////////////////////////////////////
void CleanAndInvalidateDCache(void *ptr, uint32_t size)
{
  if (oal_fd >= 0)
  {
    CMD_FLUSH_SPECIFIC_TYPE aux;
    aux.virtual_pointer = (uint64_t)ptr;
    aux.size = size;
    OAL_LldCmd_t lCmd = {CMD_FLUSHINVALIDATE_SPECIFIC, (uintptr_t)&aux};
    OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd);
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
  }
}

/////////////////////////////////////////////////////////////////////////
/// Invalidates the specified cache
/////////////////////////////////////////////////////////////////////////
void InvalidateDCache(void *ptr, uint32_t size)
{
  if (oal_fd >= 0)
  {
    CMD_FLUSH_SPECIFIC_TYPE aux;
    aux.virtual_pointer = (uint64_t)ptr;
    aux.size = size;

    OAL_LldCmd_t lCmd = {CMD_INVALIDATE_SPECIFIC, (uintptr_t)&aux};
    OAL_LldCall((OAL_LldHandle_t)oal_fd, &lCmd);
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
  }
}

/////////////////////////////////////////////////////////////////////////
/// Flushes the specified cache
/////////////////////////////////////////////////////////////////////////
void CleanDCache(void *ptr, uint32_t size)
{
  if (oal_fd >= 0)
  {
    CMD_FLUSH_SPECIFIC_TYPE aux;
    aux.virtual_pointer = (uint64_t)ptr;
    aux.size = size;

    OAL_LldCmd_t lCmd = {CMD_FLUSH_SPECIFIC, (uintptr_t)&aux};
    OAL_LldCall(oal_fd, &lCmd);
  }
  else
  {
    VDB_LOG_ERROR("OAL: Initialization failed. Please check the OAL kernel module.\n");
  }
}
