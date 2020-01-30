/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP
* All Rights Reserved
*
******************************************************************************
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
* \file APU_ACF.h
* \brief APU ACF framework simulation
* \author Igor Aleksandrowicz
* \version
* \date 12-February-2014
****************************************************************************/

#include "acf_lib.hpp"


namespace APEX2 {
#ifdef USE_MULTITHREADING
	pthread_mutex_t apex_Mutex;
#endif

	void APEX_Init(){
#ifdef USE_MULTITHREADING
		//Init global semaphores
		pthread_mutexattr_t attr;
		int err = 0;
		err = pthread_mutexattr_init(&attr);
		err |= pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_DEFAULT); // PTHREAD_MUTEX_RECURSIVE);
		err |= pthread_mutex_init(&apex_Mutex, &attr);
		if (err) {
			fprintf(stderr, "ERROR %d in creating mutex for the emulator's critical section \n", err);
		}
#endif
	}


	void APEX_Deinit(){
#ifdef USE_MULTITHREADING
		pthread_mutex_lock(&apex_Mutex);
		pthread_mutex_destroy(&apex_Mutex);
#endif
	}

	int StartCriticalSection() {
#ifdef USE_MULTITHREADING
		// int err = sem_wait(&mSem);
		int err = pthread_mutex_lock(&apex_Mutex);

		if (err)  {
			fprintf(stderr, "ERROR in StartCriticalSection: mutex lock interrupted with error %d\n", errno);
			fflush(stderr);
		}
		return err;
#endif
		return 0;
	}

	int EndCriticalSection() {
#ifdef USE_MULTITHREADING
		return pthread_mutex_unlock(&apex_Mutex);
#endif
		return 0;
	}


} // end of namespace
