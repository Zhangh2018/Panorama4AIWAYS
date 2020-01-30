/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2018 NXP Semiconductor;
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

#include <stdint.h>
#include <common.hpp>
#include <vector_map_desc.hpp>

int32_t RetNumCusInGoc(VEC_MODE lVecMode)
{
   int32_t lNumCus = 0;

   switch (lVecMode)
   {
   case VEC_MODE_GOC4:  lNumCus = 4;  break;
   case VEC_MODE_GOC8:  lNumCus = 8;  break;
   default:                           break;
   }

   return lNumCus;
}

int32_t IsGocMode(VEC_MODE lVecMode)
{
   int32_t lGocModeFlag = 0;
   if (VEC_MODE_GOC4 == lVecMode  ||
       VEC_MODE_GOC8 == lVecMode)
   {
      lGocModeFlag = 1;
   }

   return lGocModeFlag;
}

void VectorMapDesc::Init(VEC_MODE lVecMode, int32_t lPhysVecSize, int32_t lVecPad0, int32_t lVecPad1)
{
   mVecMode = lVecMode;
   mPhysVecSize = lPhysVecSize;
   mVecPad[0] = lVecPad0;
   mVecPad[1] = lVecPad1;
}
