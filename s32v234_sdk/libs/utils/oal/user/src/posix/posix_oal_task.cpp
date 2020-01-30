/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013 Freescale Semiconductor;
* Copyright (c) 2017 NXP
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
* \file OAL_Task.cpp
* \brief OAL task implementation
* \author Rostislav Hulik
* \version
****************************************************************************/

#include <stdlib.h>
#include <pthread.h>
#include "utils.h"
#include "oal.h"
#include "posix_oal_task.h"
#include "os_oal_check.h"

/////////////////////////////////////////////////////////////////////////
/// Forward declarations
/////////////////////////////////////////////////////////////////////////
const uint32_t DEFAULT_TASK_PRIORITY = 127;
const uint32_t MAX_TASK_PRIORITY = 255;

/////////////////////////////////////////////////////////////////////////
/// Function transforms priority number to possible priority range
/////////////////////////////////////////////////////////////////////////
int MapPriority(uint32_t aOALpriority)
{
  int pmax = sched_get_priority_max( SCHED_FIFO );
  int pmin = sched_get_priority_min( SCHED_FIFO );
  int range = abs((pmax - pmin)  +1);

  // Calculate the number of bins to map the OAL priorities into (large range
  // to small range).
  int nBins = int(MAX_TASK_PRIORITY)/range;

  // Add one bin if more than half a range is left.
  if (((int)MAX_TASK_PRIORITY % range) > (range / 2))
  {
    ++nBins;
  }

  // revert the priority (descending vs ascending)
  int pReverted = int(MAX_TASK_PRIORITY) - aOALpriority;
  int offset = pReverted / nBins ;

  return pmin + offset;
}

////////////////////////////////////////////////////////////////////////
/// Waits for the task to terminate.
////////////////////////////////////////////////////////////////////////
int32_t OAL_TaskJoin(OAL_TCB task)
{
    int32_t retval = OAL_SUCCESS;

    CHECK_INIT(retval)
    if (retval == OAL_SUCCESS)
    {
        if (pthread_join(ThreadGetFromTCB(task), NULL)) {
            retval = OAL_FAILURE;
        }
    }
    return retval;
}

////////////////////////////////////////////////////////////////////////
/// Delete the specified task assuming the specified task ID has
/// already been created.
////////////////////////////////////////////////////////////////////////
int32_t OAL_TaskDelete(OAL_TCB task)
{
    int32_t ret = OAL_SUCCESS;

    CHECK_INIT(ret)
    pthread_t t = ThreadGetFromTCB(task);

    if (ret == OAL_SUCCESS)
    {
        if (pthread_cancel(t))
        {
            ret = OAL_SUCCESS;
        }
        else
        {
            if (pthread_join(t, NULL))
            {
                ret = OAL_FAILURE;
            }
        }
    }
    return ret;
}
