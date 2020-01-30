/*******************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright 2017 NXP
 * All Rights Reserved
 *
 *******************************************************************************
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
 ******************************************************************************/

/*M/////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this
//  license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2014, Itseez Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright
//     notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote
//     products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is"
// and any express or implied warranties, including, but not limited to, the
// implied warranties of merchantability and fitness for a particular purpose
// are disclaimed. In no event shall the Intel Corporation or contributors be
// liable for any direct, indirect, incidental, special, exemplary, or
// consequential damages (including, but not limited to, procurement of
// substitute goods or services; loss of use, data, or profits; or business
// interruption) however caused and on any theory of liability, whether in
// contract, strict liability, or tort (including negligence or otherwise)
// arising in any way out of the use of this software, even if advised of the
// possibility of such damage.
//
//M*/

#include "sumat_defines.hpp"

#if !defined(__STANDALONE__) && !defined(__INTEGRITY__) && !defined(APEX2_EMULATE)
#include <pthread.h>
#endif

#include "sumat_mutex.hpp"
#include "oal.h"

/*******************************************************************************
 * Mutex implementation class - for pthread
 ******************************************************************************/
#ifdef WIN32
#include <windows.h>
#include <synchapi.h>

struct vsdk::Mutex::Impl
{
  Impl()
  {
#if (_WIN32_WINNT >= 0x0600)
    ::InitializeCriticalSectionEx(&cs, 1000, 0);
#else
    ::InitializeCriticalSection(&cs);
#endif
    refcount = 1;
  }
  ~Impl() { DeleteCriticalSection(&cs); }

  void lock() { EnterCriticalSection(&cs); }
  bool trylock() { return TryEnterCriticalSection(&cs) != 0; }
  void unlock() { LeaveCriticalSection(&cs); }

  CRITICAL_SECTION cs;
  int refcount;
};
#else


struct vsdk::Mutex::Impl
{
  /*****************************************************************************
   * Constructor creates the mutex and initializes it
   ****************************************************************************/
  Impl()
  {
    #if !defined(__STANDALONE__)
      /* Transparent implementation of the pthread mutex */
      OAL_SemaphoreCreate(&mt, "", OAL_SEMAPHORE_BINARY, 0);
      OAL_SemaphoreRelease(mt);
    #endif
    refcount = 1;
  }

  /*****************************************************************************
   * Destructor destroys underlying pthread mutex
   ****************************************************************************/
  ~Impl()
  {
    #if !defined(__STANDALONE__)
      OAL_SemaphoreDelete(mt);
    #endif
  }

  /*****************************************************************************
   * Locks underlying pthread mutex
   ****************************************************************************/
  void lock()
  {
    #if !defined(__STANDALONE__)
      OAL_SemaphoreObtain(mt);
    #endif
  }

  /*****************************************************************************
   * Tries to lock the underlying pthread mutex
   ****************************************************************************/
  bool trylock()
  {
    #if !defined(__STANDALONE__)
      if(OAL_SemaphoreTryObtain(mt) == OAL_SUCCESS)
      {
        return true;
      }
      else
      {
        return false;
      }
    #else
      return true;
    #endif
  }

  /*****************************************************************************
   * Unlocks the underlying pthread mutex
   ****************************************************************************/
  void unlock()
  {
    #if !defined(__STANDALONE__)
      OAL_SemaphoreRelease(mt);
    #endif
  }

  /*******************************************************************************
   * Pthread mutec
   ******************************************************************************/
  OAL_SEMAPHORE mt;

  /*****************************************************************************
   * Reference count
   ****************************************************************************/
  int32_t refcount;
};
#endif


/*******************************************************************************
 * Constructor, creates and initializes the mutex
 ******************************************************************************/
vsdk::Mutex::Mutex()
{
  mImpl = new vsdk::Mutex::Impl;
}

/*******************************************************************************
 * Destructor
 * Destroys the Mutex instance.
 ******************************************************************************/
vsdk::Mutex::~Mutex()
{
  /* Lower the reference count, delete if needed */
  if(VSDK_CV_XADD(&mImpl->refcount, -1) == 1)
  {
    delete mImpl;
  }
  mImpl = static_cast< vsdk::Mutex::Impl* >(NULL);
}

/*******************************************************************************
 * Constructor, creates and initializes the mutex based on existing one
 ******************************************************************************/
vsdk::Mutex::Mutex(const vsdk::Mutex& aMatrix)
{
  mImpl = aMatrix.mImpl;
  VSDK_CV_XADD_VOID(&mImpl->refcount, 1);
}

/*******************************************************************************
 * Assignment operator
 * Explicit assgnment operator for mutex class.
 ******************************************************************************/
vsdk::Mutex& vsdk::Mutex::operator=(const vsdk::Mutex& aMatrix)
{
  /* Raise the reference count of the object in matrix */
  VSDK_CV_XADD_VOID(&aMatrix.mImpl->refcount, 1);

  /* Lower the reference count of standalone implementation */
  if(VSDK_CV_XADD(&mImpl->refcount, -1) == 1)
  {
    delete mImpl;
  }
  mImpl = aMatrix.mImpl;
  return *this;
}

/*******************************************************************************
 * Obtain the mutex
 ******************************************************************************/
void vsdk::Mutex::lock() const
{
  mImpl->lock();
}

/*******************************************************************************
 * Free the mutex
 ******************************************************************************/
void vsdk::Mutex::unlock() const
{
  mImpl->unlock();
}

/*******************************************************************************
 * Try to obtain the mutex.
 ******************************************************************************/
bool vsdk::Mutex::trylock() const
{
  return mImpl->trylock();
}
