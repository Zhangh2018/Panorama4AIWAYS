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

#ifndef __POSIX_OAL_TASK_H__
#define __POSIX_OAL_TASK_H__

/////////////////////////////////////////////////////////////////////////
/// Function transforms pointer to opaque handle pointer to pthread pointer
/////////////////////////////////////////////////////////////////////////
static inline pthread_t* ThreadPGetFromTCBP(OAL_TCB* apTcb)
{
  return (pthread_t*)apTcb;
}

/////////////////////////////////////////////////////////////////////////
/// Function transforms pointer to opaque handle to pthread pointer
/////////////////////////////////////////////////////////////////////////
static inline pthread_t ThreadGetFromTCB(OAL_TCB aTcb)
{
  return (pthread_t)aTcb;
}

/////////////////////////////////////////////////////////////////////////
/// Function transforms priority number to possible priority range
/////////////////////////////////////////////////////////////////////////
extern int MapPriority(uint32_t aOALpriority);

#endif
