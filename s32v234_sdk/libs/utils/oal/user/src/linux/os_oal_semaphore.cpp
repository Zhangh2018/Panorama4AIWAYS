/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013 Freescale Semiconductor;
* Copyright (c) 2017 NXP;
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
* \file Oal_semaphore.cpp
* \brief OAL semaphore implementation
* \author Igor Aleksandrowicz and Rostislav Hulik
* \version
* \date 3-March-2016
****************************************************************************/

#include <string.h>
#include <semaphore.h>
#include <cerrno>
#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include "oal.h"
#include "os_oal_check.h"
#include "common_helpers.h"     /* UNUSED_ARG macro */

////////////////////////////////////////////////////////////////////////
/// Create an OS semaphore. If succeeds, the created Semaphore ID
/// will be subsequently referred to in later Semaphore calls.
/// NOTE: For OS specific stuff see S32V234 Platform user guide
////////////////////////////////////////////////////////////////////////
int32_t OAL_SemaphoreCreate(OAL_SEMAPHORE *pSem, const char *pName, OAL_SEMAPHORE_TYPE type, unsigned int count)
{
  UNUSED_ARG (pName);
  UNUSED_ARG (type);

  int32_t libResult = OAL_SUCCESS;

  CHECK_INIT(libResult)
  if (libResult == OAL_SUCCESS)
  {
    libResult = OAL_FAILURE;

    *pSem = (OAL_SEMAPHORE)malloc(sizeof(sem_t));

    if(*pSem != NULL)
    {                                             // [0] Not shared between processes
        int32_t intResult = sem_init((sem_t*)*pSem, 0, count);
        if(intResult == 0)
        {
            libResult = OAL_SUCCESS;
        }
    }
  }
  return libResult;
}

////////////////////////////////////////////////////////////////////////
/// Try to obtain a semaphore until it times out. If the Semaphore is
/// not available, the calling task will be blocked until the Semaphore
/// becomes available or times out.
////////////////////////////////////////////////////////////////////////
int32_t OAL_SemaphoreObtain(OAL_SEMAPHORE sem, unsigned int time)
{
  int32_t intResult = OAL_SUCCESS;
  int32_t libResult = OAL_FAILURE;

  CHECK_INIT(intResult)
  if (intResult == OAL_SUCCESS)
  {
      if(time == 0xFFFFFFFF)
      {
        intResult = sem_wait((sem_t*)sem);
        while ((-1 == intResult) && (EINTR == errno))
        {
          intResult = sem_wait((sem_t*)sem);
        }
      }
      else
      {
        timespec lTimespec;
        clock_gettime(CLOCK_REALTIME, &lTimespec);
        uint64_t lNanoSecTotal = ((uint64_t)lTimespec.tv_sec)*1000000000 + ((uint64_t)lTimespec.tv_nsec) + ((uint64_t)time)*1000;
        lTimespec.tv_sec  = lNanoSecTotal / 1000000000;
        lTimespec.tv_nsec = (lNanoSecTotal % 1000000000);

        intResult = sem_timedwait((sem_t*)sem, &lTimespec);
        while ((-1 == intResult) && (EINTR == errno)) {
          intResult = sem_timedwait((sem_t*)sem, &lTimespec);
        }
      }
      if(intResult == 0)
      {
        libResult = OAL_SUCCESS;
      }
  }
  return libResult;
}

////////////////////////////////////////////////////////////////////////
///  Wait on a semaphore until it is obtained.
////////////////////////////////////////////////////////////////////////
int32_t OAL_SemaphoreObtain(OAL_SEMAPHORE sem)
{
  int32_t intResult = OAL_SUCCESS;
  int32_t libResult = OAL_FAILURE;

  CHECK_INIT(intResult)
  if (intResult == OAL_SUCCESS)
  {
      intResult = sem_wait((sem_t*)sem);
      while ((-1 == intResult) && (EINTR == errno))
      {
        intResult = sem_wait((sem_t*)sem);
      }
      if(intResult == 0)
      {
        libResult = OAL_SUCCESS;
      }
  }
  return libResult;
}

////////////////////////////////////////////////////////////////////////
/// Try to obtain a semaphore. If the Semaphore is not available, the
/// function will return immediately with an error. If it is available
/// a SUCCESS return code will be returned.
////////////////////////////////////////////////////////////////////////
int32_t OAL_SemaphoreTryObtain(OAL_SEMAPHORE sem)
{
    int32_t ret= OAL_SUCCESS;

    CHECK_INIT(ret)
    if (ret == OAL_SUCCESS)
    {
        ret = OAL_SemaphoreObtain(sem, 0);
    }
    return ret;
}

////////////////////////////////////////////////////////////////////////
/// Signal or release a semaphore assuming the passed Semaphore ID has
/// already been created.
////////////////////////////////////////////////////////////////////////
int32_t OAL_SemaphoreRelease(OAL_SEMAPHORE sem)
{
    int32_t intResult = OAL_SUCCESS;
    int32_t libResult = OAL_FAILURE;

    CHECK_INIT(intResult)
    if (intResult == OAL_SUCCESS)
    {
        intResult = sem_post((sem_t*)sem);

        if(intResult == 0)
        {
        libResult = OAL_SUCCESS;
        }
    }
    return libResult;
}

////////////////////////////////////////////////////////////////////////
/// Delete a semaphore assuming the Semaphore ID argument has been
/// already created.
////////////////////////////////////////////////////////////////////////
int32_t OAL_SemaphoreDelete(OAL_SEMAPHORE sem)
{
    int32_t intResult = OAL_SUCCESS;
    int32_t libResult = OAL_FAILURE;

    CHECK_INIT(intResult)
    if (intResult == OAL_SUCCESS)
    {
        intResult = sem_destroy((sem_t*)sem);

        if(intResult == 0)
        {
            free(sem);
            libResult = OAL_SUCCESS;
        }
    }
    return libResult;
}
