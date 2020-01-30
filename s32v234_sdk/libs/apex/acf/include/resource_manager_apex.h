/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017 NXP 
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

#ifndef RESOURCEMANAGERAPEX_H
#define RESOURCEMANAGERAPEX_H

#include <stdint.h>
#include <global_errors.h>

//resource type enumeration
enum
{  
   //RES__<set type>__<res type>
   RES__APEX__APU = 0,
   RES__APEX__CU_32,
   RES__SMEM_64K
};

//user type enumeration (no association with a set)
enum
{
   USER__ACF = 0
};


LIB_RESULT ResourceManagerInit_APEX(uint32_t lHwVersion);
LIB_RESULT ResourceManagerDeinit_APEX(uint32_t lHwVersion);

uint32_t InitApexResReq(uint32_t lArray[4], uint32_t lApexId, uint32_t lResType, uint32_t lNumOrInstance);
uint32_t ApexSetTypeId();
uint32_t APEX_ResTypeId(int32_t lResEnum);
uint32_t UserTypeId(int32_t lUserEnum);

#endif /* RESOURCEMANAGERAPEX_H */
