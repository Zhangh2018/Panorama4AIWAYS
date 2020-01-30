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
* \file oal_lldcmd.cpp
* \brief OAL low level driver command functionality
* \author Tomas Babinec
* \version
* \date 24-Nov-2017
****************************************************************************/

#include <oal.h>
#include <stdio.h>
#include <unistd.h>

////////////////////////////////////////////////////////////////////////
/// Initialize an interrupt and add a service routine to it.
////////////////////////////////////////////////////////////////////////
OAL_RESULT OAL_LldCall(const OAL_LldHandle_t acHandle, 
                   OAL_LldCmd_t*         apCmd)
{
  OAL_RESULT lRet = OAL_SUCCESS;
  ssize_t    lRes = 0;
  
  lRes = write((int)acHandle, apCmd, sizeof(OAL_LldCmd_t));
  
  if(lRes != sizeof(OAL_LldCmd_t)) 
  {
    lRet = OAL_FAILURE;
  } // if error returned
  
  return lRet;
} // LldCall()