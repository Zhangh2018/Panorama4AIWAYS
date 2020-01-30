/******************************************************************************
 *
 * NXP Confidential Proprietary
 *
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

#include <stdio.h>
#include <semaphore.h>
#include <pthread.h>
#include <time.h>
#include "oal.h"
#include "os_oal_check.h"
#include "vdb_log.h"
#include "common_helpers.h"     /* UNUSED_ARG macro */

#define GETMILISEC 1000000
#define GETNANOSEC 1000

/* QNX-specific semaphore type hidden by the OAL_SEMAPHORE */
struct tOalSemaphore
{
    sem_t mSemaphore;               //POSIX semaphore
    pthread_mutex_t mMutex;         //POSIX mutex
    OAL_SEMAPHORE_TYPE type;

    /* Simple constructor */
    tOalSemaphore(void): type(OAL_SEMAPHORE_INVALID)
    {
    }
};


/* Converter from microseconds to timespec */
static inline timespec MicrosecondsToTimeSpec(unsigned int aTime)
{
    timespec ts;
    ts.tv_sec = time_t(aTime / GETMILISEC);
    ts.tv_nsec = time_t((aTime % GETMILISEC) * GETNANOSEC);
    return ts;
}

/* Is the OAL_SEMAPHORE a mutex? */
static inline bool IsMutex(const OAL_SEMAPHORE aOS)
{
    return aOS->type == OAL_SEMAPHORE_MUTEX;
}

/* Create an OS semaphore. If succeeds, the created Semaphore ID
   will be subsequently referred to in later Semaphore calls.
*/
int32_t OAL_SemaphoreCreate(OAL_SEMAPHORE *pSem, const char *pName,
                            OAL_SEMAPHORE_TYPE type, unsigned int count)
{
    UNUSED_ARG(pName);      /* silence unused variable */
    int32_t ret = OAL_SUCCESS;

    do {
        CHECK_INIT(ret)
        if (ret != OAL_SUCCESS)
        {
            continue;
        }

        if (type == OAL_SEMAPHORE_INVALID)
        {
            VDB_LOG_WARNING("OAL: Invalid semaphore type\n");
            ret = OAL_SEM_EBADTYPE;
            continue;
        }

        tOalSemaphore* tos = new tOalSemaphore();

        if (type == OAL_SEMAPHORE_MUTEX)
        {
            tos->mMutex = PTHREAD_MUTEX_INITIALIZER;

            if (pthread_mutex_init(&tos->mMutex, NULL) != EOK)
            {
                VDB_LOG_ERROR("OAL: Failed to initialize mutex\n");
                delete tos;
                ret = OAL_FAILURE;
                continue;
            }
        }
        else
        {
            if (type == OAL_SEMAPHORE_BINARY)
            {
                    count = 1;
            }

            if (sem_init(&tos->mSemaphore, 1, count) != 0)
            {
                VDB_LOG_ERROR("OAL: Failed to initialize semaphore\n");
                delete tos;
                ret = OAL_FAILURE;
                continue;
            }
        }
        tos->type = type;
        *pSem = tos;
        ret = OAL_SUCCESS;
    } while (0);

    return ret;
}

/* Try to obtain a semaphore until it times out. If the Semaphore is
   not available, the calling task will be blocked until the Semaphore
   becomes available or times out
*/
int32_t OAL_SemaphoreObtain(OAL_SEMAPHORE sem, unsigned int time)
{
    int32_t ret= OAL_SUCCESS;

    CHECK_INIT(ret)
    if (ret == OAL_SUCCESS)
    {
        int intRet;
        timespec now;

        clock_gettime(CLOCK_REALTIME, &now);
        timespec ts = MicrosecondsToTimeSpec(time);
        //absolute time needed
        ts.tv_sec += now.tv_sec;
        ts.tv_nsec += now.tv_nsec;
        ts.tv_sec += ts.tv_nsec / (GETMILISEC * GETNANOSEC);
        ts.tv_nsec %= GETMILISEC * GETNANOSEC;

        if (!(bool)sem)
        {
            VDB_LOG_ERROR("OAL: Invalid OAL semaphore\n");
            ret = OAL_SEM_EBADSEM;
        }
        else
        {
            if (IsMutex(sem))
            {
                if (pthread_mutex_timedlock(&sem->mMutex, &ts) != 0)
                {
                    VDB_LOG_ERROR("OAL: Failed to acquire mutex\n");
                    ret = OAL_FAILURE;
                }
            }
            else
            {
                do
                {
                    intRet = sem_timedwait(&sem->mSemaphore, &ts);
                } while ((intRet == -1) && (errno == EINTR));

                if (intRet != 0)
                {
                    VDB_LOG_ERROR("OAL: Failed to get semaphore\n");
                    ret = OAL_FAILURE;
                }
            }
        }
    }
    return ret;
}

/*
	Try to obtain a semaphore. If the Semaphore is not available, the
	function will return immediately with an error. If it is available
	a SUCCESS return code will be returned.
*/
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

/* Signal or release a semaphore assuming the passed Semaphore ID
   has already been created.
*/
int32_t OAL_SemaphoreRelease(OAL_SEMAPHORE sem)
{
    int32_t ret = OAL_SUCCESS;

    CHECK_INIT(ret)
    if (ret == OAL_SUCCESS)
    {
        if (!(bool)sem)
        {
            VDB_LOG_ERROR("OAL: Invalid OAL semaphore\n");
            ret = OAL_SEM_EBADSEM;
        }
        else
        {
            if (IsMutex(sem))
            {
                if (pthread_mutex_unlock(&sem->mMutex) != 0)
                {
                    VDB_LOG_ERROR("OAL: Failed to unlock mutex\n");
                    ret = OAL_FAILURE;
                }
            }
            else
            {
                if (sem_post(&sem->mSemaphore) != 0)
                {
                    VDB_LOG_ERROR("OAL: Failed to release semaphore\n");
                    ret = OAL_FAILURE;
                }
            }
        }
    }
    return ret;
}

/* Delete a semaphore assuming the Semaphore ID argument
   has been already created.
*/
int32_t OAL_SemaphoreDelete(OAL_SEMAPHORE sem)
{
    int32_t ret = OAL_SUCCESS;

    CHECK_INIT(ret)
    if (ret == OAL_SUCCESS)
    {
        if (!(bool)sem)
        {
            VDB_LOG_ERROR("OAL: Invalid OAL semaphore\n");
            ret = OAL_SEM_EBADSEM;
        }
        else
        {
            if (IsMutex(sem))
            {
              if (pthread_mutex_destroy(&sem->mMutex))
              {
                  VDB_LOG_ERROR("OAL: Cannot destroy mutex\n");
                  ret = OAL_FAILURE;
              }
            }
            else
            {
              if (sem_destroy(&sem->mSemaphore))
              {
                  VDB_LOG_ERROR("OAL: Cannot destroy semaphore\n");
                  ret = OAL_FAILURE;
              }
            }
        }
    }
    return ret;
}
