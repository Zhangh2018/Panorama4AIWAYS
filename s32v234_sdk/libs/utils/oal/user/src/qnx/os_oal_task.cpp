/*****************************************************************************
*
* NXP Confidential Proprietary
*
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

#include <pthread.h>
#include "oal.h"
#include "posix_oal_task.h"
#include "os_oal_check.h"

///////////////////////////////////////////////////////////////////////////////////////
/// Create an OS Task.  If succeeded, the created Task ID
/// will be subsequently referred to in later OS Task calls.
///////////////////////////////////////////////////////////////////////////////////////
int32_t OAL_TaskInit(OAL_TASK_ENTRY task_entry, void* argument,
                        unsigned int stack_size, unsigned int priority, char* task_name,
                        OAL_TCB *task)
{
	pthread_attr_t attr;
	int32_t retval = OAL_SUCCESS;

	do {
		CHECK_INIT(retval)
		if (retval != OAL_SUCCESS)
		{
			continue;
		}

		retval = OAL_FAILURE;
		// init pthread attributes
		if (pthread_attr_init(&attr)) {
			continue;
		}
		// set stack size to attributes
		if (pthread_attr_setstacksize(&attr, (size_t)stack_size)) {
			continue;
		}
		// set priority to attributes
		sched_param sp;
		priority = MapPriority(priority);
		sp.sched_priority = priority;
		if (pthread_attr_setschedpolicy(&attr, SCHED_RR)) {
			continue;
		}
		if (pthread_attr_setschedparam(&attr, &sp)) {
			continue;
		}
		// create a thread and set the name
		if (pthread_create(ThreadPGetFromTCBP(task), &attr, task_entry, argument)) {
			continue;
		}
		pthread_setname_np(*ThreadPGetFromTCBP(task), task_name);
		(void)pthread_attr_destroy(&attr);

		retval = OAL_SUCCESS;
	} while (0);

	return retval;
}
