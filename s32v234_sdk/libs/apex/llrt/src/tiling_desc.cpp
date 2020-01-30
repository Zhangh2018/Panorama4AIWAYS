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
#include <tiling_desc.hpp>

/*NOTES:
- size in tiling/vectorization desc should match size of APEX CB that will contain the tile/vchunk.  step may be different in each case, but size should be the same.
there is no data 'shape' transformation here.  data will move assuming the same physical layout at src and dst (albeit with allowance for different steps/spans)
*/

int32_t TilingDesc::InitND(int32_t lNumDim, int32_t lSize[], int32_t lStepSize[], int32_t lStepDim[])
{  
   int lRetVal = 0;
   
   mNumDim = lNumDim;

   if (mNumDim <= DESC_MAX_NUM_DIM)
   {
      for (int32_t i=0; i<mNumDim; i++)
      {
         mSize[i] = lSize[i];
         mStepSize[i] = lStepSize[i];
         mStepDim[i] = lStepDim[i];
      } 
   }
   else
   {
      //lNumDim is out of range
      lRetVal = 1;
   }

   mNumTileUserDef = 0;
   mTileOffsetArrayUserDef = 0;
   
   return lRetVal;
}

int32_t TilingDesc::Init2D(int32_t lSize0, int32_t lSize1,
                           int32_t lStepSize0, int32_t lStepSize1,
                           int32_t lStepDim0, int32_t lStepDim1)
{
   mNumDim = 2;
   mSize[0] = lSize0;
   mSize[1] = lSize1;
   mStepSize[0] = lStepSize0;
   mStepSize[1] = lStepSize1;
   mStepDim[0] = lStepDim0;
   mStepDim[1] = lStepDim1;
   mNumTileUserDef = 0;
   mTileOffsetArrayUserDef = 0;

   return 0;
}

#if 0
int32_t TilingDesc::InitUserDefND(int32_t lNumDim, int32_t lSize[], int32_t lNumTile, int32_t lTileOffsetArray[])
{
   mNumDim = lNumDim;

   if (mNumDim <= DESC_MAX_NUM_DIM)
   {
      for (int32_t i = 0; i<mNumDim; i++)
      {
         mSize[i] = lSize[i];
         mStepSize[i] = 0;
         mStepDim[i] = 0;
      }
   }

   mNumTileUserDef = lNumTile;
   mTileOffsetArrayUserDef = lTileOffsetArray;

   return 0;
}
#endif
