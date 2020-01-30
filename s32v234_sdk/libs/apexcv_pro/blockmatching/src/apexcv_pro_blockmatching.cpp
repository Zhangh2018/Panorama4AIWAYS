/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2015-2017 NXP
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
/*!*********************************************************************************
*  \file pro_blockmatching.cpp
*  \brief ApexCV Host wrapper for block matching algorithm
***********************************************************************************/

// ACF HOST LIB
#include <icp_data.h>
#include <icp_types.h>
#include <oal.h>

// USER DEFINED HOST ICL API
#include <apexcv_pro_blockmatching.h>
#include <apexcv_error_codes.hpp>


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>


// GENERATED ACF PROCESS
#include <BLOCKMATCHING.hpp>

namespace apexcv
{

Blockmatching::Blockmatching()
: mInit(false), mNumPoints(0), mSadThreshold(0),
  mTX(0), mTY(0), mWX(0), mWY(0), mNX(0), mNY(0),
  mMaxPoints(0), mpProcess(nullptr)
{
}


Blockmatching::~Blockmatching()
{
  Release();
}


APEXCV_LIB_RESULT Blockmatching::AllocateBuffers(int aMaxPoints)
{
  int lRetVal = APEXCV_SUCCESS;
  if (aMaxPoints > 0 && aMaxPoints > mMaxPoints)
  {
    mOffTemplate.~SUMat();
    mOffWindow.~SUMat();
    mScore.~SUMat();
    mOutput.~SUMat();

    mMaxPoints      = aMaxPoints;

    mOffTemplate = vsdk::SUMat(1, mMaxPoints, VSDK_CV_32SC1);
    mOffWindow   = vsdk::SUMat(1, mMaxPoints, VSDK_CV_32SC1);
    mScore       = vsdk::SUMat(1, mMaxPoints, VSDK_CV_16UC1);
    mOutput      = vsdk::SUMat(1, mMaxPoints, VSDK_CV_16SC2);

    // We only need the temporary output buffer if more than 1 CU is used for each tracked point
    if ((mOffTemplate.empty() == true) ||
        (mOffWindow.empty()   == true) ||
        (mScore.empty()       == true) ||
        (mOutput.empty()      == true))
    {
      Release();
      lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
  }
  return lRetVal;
}

APEXCV_LIB_RESULT Blockmatching::Initialize(
            vsdk::SUMat& aOutputPoints,
            vsdk::SUMat& aOutputStatus,
      const vsdk::SUMat& acInputTemplate,
      const vsdk::SUMat& acInputWindow,
      const vsdk::SUMat& acInputPoints,
                    int aSad_threshold,
                    int aTracked_points,
                    int aTsx, int aTsy,
                    int aWsx, int aWsy,
                   int aNcu_x, int aNcu_y)
{
  int lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;

  if (!mInit)
  {
    // Perform parameter checking
    // Need to manually check lOutputPoints and lOutputStatus since ACF does not use them,
    // therefore it is not checked for us
    if (  (aTsx&0x3) || (aTsy&0x3) || (aWsx&0x3) || (aWsy&0x3) || (aTsx >= aWsx)
       || (aTsy >= aWsy) || (aTsx <=0) || (aTsy <=0) || (aWsx <=0) || (aTsy<=0)
       || aWsx > acInputWindow.cols   || aWsy > acInputWindow.rows
       || aTsx > acInputTemplate.cols || aTsy > acInputTemplate.rows
       || aTracked_points>acInputPoints.cols
       || acInputTemplate.type() != VSDK_CV_8UC1
       || acInputWindow.type()   != VSDK_CV_8UC1
       || aOutputStatus.type()  != VSDK_CV_8UC1
       // Don't really care about the e0 size, all we care is if there is enough memory
       || aOutputStatus.step[0] < (uint)aTracked_points
       || aOutputPoints.step[0] < (uint)aTracked_points)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    BLOCKMATCHING* lProcess = new BLOCKMATCHING;
    if (nullptr == lProcess)
    {
       return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mpProcess = lProcess;

    if (aTracked_points > 0)
    {
      mNumPoints = aTracked_points;
    }
    else
    {
      mNumPoints = acInputPoints.cols;
    }

    mSadThreshold = aSad_threshold;
    if (mSadThreshold > 0xFFFF)
    {
      mSadThreshold = 0xFFFF;
    }
    mNX = aNcu_x;
    mNY = aNcu_y;
    mNX = mNX <= 0 ? 1:mNX;
    mNY = mNY <= 0 ? 1:mNY;
    mWX = aWsx;
    mWY = aWsy;
    mTX = aTsx;
    mTY = aTsy;

    mOutputPoints  = aOutputPoints;
    mOutputStatus  = aOutputStatus;
    mInputTemplate = acInputTemplate;
    mInputWindow   = acInputWindow;
    mInputPoints   = acInputPoints;

    lRetVal = lProcess->Initialize();
    {
      ApexcvHostBaseBaseClass::InfoCluster lInfo;
      lInfo.set_ACF("BLOCKMATCHING", lProcess);
      lInfo.push_PortName("INPUT_TEMPLATE");
      mvInfoClusters.push_back(lInfo);
    }
    if (!lRetVal)
    {
      mInit = true;
    }
    else
    {
      mInit = false;
      Release();
    }
  }

  return lRetVal;
}

APEXCV_LIB_RESULT Blockmatching::SetParameters(
          int ,
          int aTsx, int aTsy,
          int aWsx, int aWsy,
          int aNcu_x, int aNcu_y)
{
  int lRetVal = APEXCV_SUCCESS;
  if (  (aTsx&0x3) || (aTsy&0x3) || (aWsx&0x3) || (aWsy&0x3) || (aTsx >= aWsx)
     || (aTsy >= aWsy) || (aTsx <=0) || (aTsy <=0) || (aWsx <=0) || (aTsy<=0))
  {
    lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
  }
  else
  {
    mNX = aNcu_x;
    mNY = aNcu_y;
    mNX = mNX <= 0 ? 1:mNX;
    mNY = mNY <= 0 ? 1:mNY;
    mWX = aWsx;
    mWY = aWsy;
    mTX = aTsx;
    mTY = aTsy;
  }
  return lRetVal;
}

APEXCV_LIB_RESULT Blockmatching::ReconnectIO(
            vsdk::SUMat& aOutputPoints,
            vsdk::SUMat& aOutputStatus,
      const vsdk::SUMat& acInputTemplate,
      const vsdk::SUMat& acInputWindow,
      const vsdk::SUMat& acInputPoints,
                    int aTracked_points
)
{
  int lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  if (mInit)
  {
    if(   mWX > acInputWindow.cols   || mWY > acInputWindow.rows
       || mTX > acInputTemplate.cols || mTY > acInputTemplate.rows
       || aTracked_points       >  acInputPoints.cols
       || acInputTemplate.type() != VSDK_CV_8UC1
       || acInputWindow.type()   != VSDK_CV_8UC1
       || aOutputStatus.type()  != VSDK_CV_8UC1
       || aOutputStatus.step[0] <(uint) aTracked_points
       || aOutputPoints.step[0] <(uint) aTracked_points)
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
    }
    else
    {
      lRetVal = APEXCV_SUCCESS;
      if (aTracked_points > 0)
      {
        mNumPoints = aTracked_points;
      }
      else
      {
        mNumPoints = acInputPoints.cols;
      }
      mOutputPoints  = aOutputPoints;
      mOutputStatus  = aOutputStatus;
      mInputTemplate = acInputTemplate;
      mInputWindow   = acInputWindow;
      mInputPoints   = acInputPoints;
    }
  }
  return lRetVal;
}

APEXCV_LIB_RESULT Blockmatching::Process(int aTracked_points)
{
  int32_t lRetVal = APEXCV_SUCCESS;

  if(mInit)
  {
    if (aTracked_points < 0)
    {
      aTracked_points = mNumPoints;
    }
    const int ww = mInputWindow.cols;
    const int wh = mInputWindow.rows;
    const int ws = mInputWindow.step[0];
    const int tw = mInputTemplate.cols;
    const int th = mInputTemplate.rows;
    const int ts = mInputTemplate.step[0];

    const int off_x = mWX - mTX + 1;
    const int off_y = mWY - mTY + 1;
    const int eff_x = off_x*(mNX - 1) + mWX;
    const int eff_y = off_y*(mNY - 1) + mWY;

    int nx = mNX;
    int ny = mNY;
    int ex = eff_x;
    int ey = eff_y;

    if (ex > ww)
    {
      nx = (ww - mWX)/off_x + 1;
      ex = off_x*(nx - 1) + mWX;
    }

    if (ey > wh)
    {
      ny = (wh - mWY)/off_y + 1;
      ey = off_y*(ny - 1) + mWY;
    }

    // For indirect inputs, we need a muliple of 4
    int num_offsets = aTracked_points*nx*ny;
    num_offsets = (num_offsets+3)& (~0x3);

    if (!AllocateBuffers(num_offsets))
    {
      vsdk::SMat mInputPointsMat = mInputPoints.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      ICP_Point_16S* pPoints = (ICP_Point_16S*)mInputPointsMat.data;
      { /* setup offset table */
        vsdk::SMat offTemplateMat = mOffTemplate.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        vsdk::SMat offWindowMat = mOffWindow.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

        int32_t* src_offsets = (int32_t*)offTemplateMat.data;
        int32_t* win_offsets = (int32_t*)offWindowMat.data;

        // Generate offsets for each point
        for (int i = 0; i < aTracked_points; ++i)
        {
          // Assuming the point co-ordinates given are center of feature
          int start_x = pPoints[i].x - ex / 2;
          int start_y = pPoints[i].y - ey / 2;

          // Validate parameters
          if (start_x < 0)
          {
            start_x = 0;
          }
          if (start_y < 0)
          {
            start_y = 0;
          }

          if ((start_x + ex) > ww)
          {
            start_x = ww - ex;
          }
          if ((start_y + ey) > wh)
          {
            start_y = wh - ey;
          }

          // Setup the search windows
          int32_t point_off = (pPoints[i].y - mTY/2)*ts + pPoints[i].x - mTX/2;
          if (point_off < 0 || point_off > (th-1)*ts+tw-1)
          {
            point_off = 0;
          }
          for (int y = 0; y < ny; ++y)
          {
            for (int x = 0; x < nx; ++x)
            {
              win_offsets[y*nx + x] = (start_y + y*off_y)*ws + (start_x + x*off_x);
              src_offsets[y*nx + x] = point_off;
            }
          }
          src_offsets += nx*ny;
          win_offsets += nx*ny;
        } /* for (int i = 0; i < tracked_points; ++i)  */

        // Adjust for any extra padded points
        src_offsets = (int32_t*)offTemplateMat.data;
        win_offsets = (int32_t*)offWindowMat.data;
        for (int i = aTracked_points*nx*ny; i < num_offsets; ++i)
        {
          win_offsets[i] = win_offsets[aTracked_points*nx*ny-1];
          src_offsets[i] = src_offsets[aTracked_points*nx*ny-1];
        }
      } /* setup offset table */

      // PROCESSING

      BLOCKMATCHING* lProcess = (BLOCKMATCHING*) mpProcess;

      vsdk::CustomPointerSUMat lOffTemplate = vsdk::CustomPointerSUMat(1, num_offsets, VSDK_CV_32SC1, (uint8_t*)(mOffTemplate.u->handle) + mOffTemplate.offset, 0);
      vsdk::CustomPointerSUMat lOffWindow   = vsdk::CustomPointerSUMat(1, num_offsets, VSDK_CV_32SC1, (uint8_t*)(mOffWindow.u->handle) + mInputWindow.offset, 0);
      vsdk::CustomPointerSUMat lScore       = vsdk::CustomPointerSUMat(1, num_offsets, VSDK_CV_16UC1, (uint8_t*)(mScore.u->handle) + mScore.offset, 0);

      lRetVal |= lProcess->ConnectIndirectInput("INPUT_TEMPLATE", mInputTemplate, lOffTemplate);
      lRetVal |= lProcess->ConnectIndirectInput("INPUT_WINDOW", mInputWindow, lOffWindow);
      lRetVal |= lProcess->ConnectIO("OUTPUT_SCORE", lScore);

      if (nx != 1 || ny != 1)
      {
        vsdk::CustomPointerSUMat lOutput(1, num_offsets, VSDK_CV_16SC2, (uint8_t*)(mOutput.u->handle) + mOutput.offset, 0);
  	    lRetVal |= lProcess->ConnectIO("OUTPUT_POINTS", lOutput);
      }
      else
      {
        vsdk::CustomPointerSUMat outputPointsCustom(1, num_offsets, VSDK_CV_16SC2, (uint8_t*)(mOutputPoints.u->handle) + mOutputPoints.offset, 0);
        lRetVal |= lProcess->ConnectIO("OUTPUT_POINTS", outputPointsCustom);
      }
      lRetVal |= lProcess->SelectScenario("", 0, 0);
      
      // Caveat: Need to override the chunk size every time since this modifies the graph properties
      //         instead of just this specific process.
      lRetVal |= lProcess->OverridePortChunkSize("INPUT_TEMPLATE", mTX, mTY);
      lRetVal |= lProcess->OverridePortChunkSize("INPUT_WINDOW", mWX, mWY);

      lRetVal |= lProcess->Start();
      lRetVal |= lProcess->Wait();

      if (!lRetVal)
      {
        vsdk::SMat mOutputPointsMat = mOutputPoints.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        vsdk::SMat mOutputStatusMat = mOutputStatus.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        vsdk::SMat mInputPointsMat  = mInputPoints.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

        uint16_t*      pScore      = (uint16_t*)(mScore.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED)).data;
        int32_t*       pOffWin     = (int32_t*) (mOffWindow.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED)).data;
        ICP_Point_16S* pPoints     = (ICP_Point_16S*)mInputPointsMat.data;
        uint8_t*       pStatus     = (uint8_t*)mOutputStatusMat.data;
        ICP_Point_16S* pOutPoints  = (ICP_Point_16S*)mOutputPointsMat.data;

        if (nx != 1 || ny != 1)
        {
          ICP_Point_16S* pOutput   = (ICP_Point_16S*)(mOutput.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED)).data;

          for (int i = 0; i < aTracked_points; ++i)
          {
            if ( pPoints[i].x - mTX/2 < 0 || pPoints[i].x + mTX/2 > tw ||
                 pPoints[i].y - mTY/2 < 0 || pPoints[i].y + mTY/2 > th)
            {
              pOutPoints[i].x = 0;
              pOutPoints[i].y = 0;
              pStatus[i] = 0;
            }
            else
            {
              // We need to aggregate the results from the multiple search windows
              int min = 0x7FFFFFFF;
              int min_x = 0;
              int min_y = 0;
              for (int j = 0; j < nx*ny; ++j)
              {
                if (pScore[j] < min)
                {
                  // The output diff_x and diff_y are based on the search window start,
                  // and the search window start is based on the input point
                  int off_win = pOffWin[j];
                  int off_y = off_win / ws;
                  int off_x = off_win - off_y*ws;

                  min = pScore[j];
                  min_x = off_x + pOutput[j].x;
                  min_y = off_y + pOutput[j].y;
                }
              } /* for (int j = 0; j < nx*ny; ++j) */
              
              if (min < mSadThreshold)
              {
                pStatus[i] = 1;
                pOutPoints[i].x = min_x + mTX/2;
                pOutPoints[i].y = min_y + mTY/2;
              }
              else
              {
                pStatus[i] = 0;
                pOutPoints[i].x = 0;
                pOutPoints[i].y = 0;
              }
            } /* else */
            pOutput += nx*ny;
            pScore += nx*ny;
            pOffWin += nx*ny;
          } /*  for (int i = 0; i < tracked_points; ++i) */
        } /* if (nx != 1 || ny != 1)  */
        else
        {
          for (int i = 0; i < aTracked_points; ++i)
          {
            if (  pPoints[i].x - mTX/2 < 0 || pPoints[i].x + mTX/2 > tw
               || pPoints[i].y - mTY/2 < 0 || pPoints[i].y + mTY/2 > th)
            {
              pOutPoints[i].x = 0;
              pOutPoints[i].y = 0;
              pStatus[i] = 0;
            }
            else
            {
              if (pScore[i] < mSadThreshold)
              {
                int start_y = pOffWin[i] / ws;
                int start_x = pOffWin[i] - ws*start_y;
                pOutPoints[i].x += start_x + mTX/2;
                pOutPoints[i].y += start_y + mTY/2;
                pStatus[i] = 1;
              }
              else
              {
                pStatus[i] = 0;
                pOutPoints[i].x = 0;
                pOutPoints[i].y = 0;
              }
            } /* else */
          } /* for (int i = 0; i < tracked_points; ++i) */
        } /* else */
      } /* if (!lRetVal) */
      else
      {
        lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
  } /* if(mInit) */
  else
  {
    lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }
  return lRetVal;
}

APEXCV_LIB_RESULT Blockmatching::SetSadThreshold(int aSadThreshold)
{
  if (aSadThreshold > 0xFFFF)
  {
    aSadThreshold = 0xFFFF;
  }
  else if (aSadThreshold <= 0)
  {
    aSadThreshold = 1;
  }
  mSadThreshold = aSadThreshold;
  return APEXCV_SUCCESS;
}

void Blockmatching::Release()
{
  if (nullptr != mpProcess)
  {
    delete (BLOCKMATCHING*) mpProcess;
  }
  mpProcess = nullptr;

  mOffTemplate.~SUMat();
  mOffWindow.~SUMat();
  mScore.~SUMat();
  mOutput.~SUMat();
  mMaxPoints   = 0;
}

} /* namespace apexcv */
