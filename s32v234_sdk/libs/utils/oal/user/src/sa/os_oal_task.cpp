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
* \file OAL_Task.cpp
* \brief OAL task implementation
* \author Rostislav Hulik
* \version
* \date 4-March-2016
****************************************************************************/

#include "oal.h"
#include "os_oal_check.h"

///////////////////////////////////////////////////////////////////////////////////////
/// Create an OS Task.  If succeeded, the created Task ID
/// will be subsequently referred to in later OS Task calls.
///////////////////////////////////////////////////////////////////////////////////////
int32_t OAL_TaskInit(OAL_TASK_ENTRY task_entry, void* argument,
                        unsigned int stack_size, unsigned int priority, char* task_name,
                        OAL_TCB *task)
{
    int32_t retval = OAL_SUCCESS;

    CHECK_INIT(retval)
    if (retval == OAL_SUCCESS)
    {
        if (task_entry(argument) == 0)
        {
            retval = OAL_SUCCESS;
        }
        else
        {
            retval = OAL_FAILURE;
        }
    }
    return retval;
}

////////////////////////////////////////////////////////////////////////
/// Waits for the task to terminate.
////////////////////////////////////////////////////////////////////////
int32_t OAL_TaskJoin(OAL_TCB task)
{
    int32_t retval = OAL_SUCCESS;

    CHECK_INIT(retval)
    return retval;
}

////////////////////////////////////////////////////////////////////////
/// Delete the specified task assuming the specified task ID has
/// already been created.
////////////////////////////////////////////////////////////////////////
int32_t OAL_TaskDelete(OAL_TCB task)
{
    int32_t retval = OAL_SUCCESS;

    CHECK_INIT(retval)
    return retval;
}
