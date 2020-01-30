/*****************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2014-2017 NXP
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

#include <cstring>
#include <oal.h>
#include "apexcv_pro_hough.h"
#include "hough_config_acf.h"
#include <math.h>

// GENERATED ACF PROCESS
#include <HOUGH_6X4.hpp>
#include <HOUGH_10X4.hpp>
#include <HOUGH_20X2.hpp>
#include <HOUGH_40X1.hpp>

#if (HOUGH_PERF_PROB != 0)
#include <common_time_measure.h>
#include "math.h"

#ifdef __STANDALONE__
const uint32_t cTimeScale = 1;
#else
const uint32_t cTimeScale = 1000;
#endif

volatile int64_t gHoughTime[HOUGH_PROFILED_FUNC_CNT][PROBES_CNT];
#endif

/* Returns the base address of a DataDescriptor */
static void * AddrDataDesc(icp::DataDescriptor &aIn)
{
  void *pData = aIn.GetDataPtr();

  return pData;
}

/* Force the flush of cache granules to DDR */
static void FlushInvalidateDataDesc(icp::DataDescriptor &aIn)
{
  void *pData              = aIn.GetDataPtr();
  unsigned int sizeInBytes = aIn.GetHeight() * aIn.GetSpan();

  if(pData != NULL)
  {
    OAL_MemoryFlushAndInvalidate(pData, sizeInBytes);
  }
}

/* Rounding function from double to int */
static inline int RoundDbl(double aX)
{
  return (aX < 0) ? -(int)(0.5 - aX) : (int)(aX + 0.5);
}

/* Force aTheta to be inside [0, 2*pi] */
static double WrapTheta(double aTheta)
{
  /* The function can be seen as a modulo 2*pi of aTheta */
  /* Example: aTheta = 2.58*pi => output will be 0.58*pi */
  double pi = 3.1415926535897932384626433832795;  
  int n     = aTheta < 0 ? (int)ceil(-aTheta / pi) : -(int)(aTheta / pi);
  aTheta += n * pi;

  return aTheta;
}

/* Returns the number of Rho discreet values inside the accumulator */
static inline int ReadRhoCount(int aImageCols, int aImageRows)
{
  return RoundDbl(sqrt((double)(aImageCols * aImageCols + aImageRows * aImageRows))) + 1;
}

/* Returns the Sin/Cos table for polar coordinate lines */
static void GetCosSinTableNoNms(int16_t *apCosTable,
                                int16_t *apSinTable,
                                int aTableCount,
                                int aThetaCount,
                                float *apTheta)
{
  // Get the cosine and sine lookup tables for no NMS between angles  
  // one_Q14 = 1 * 2^14
  const int16_t one_Q14 = (int16_t)0x4000;
  int thetaLast   = aThetaCount - 1;
  
  for(int i = 0; i < thetaLast; ++i)
  {
    // Read the Theta value
    double t = apTheta[i];

    // The output representation is 2Q14
    apCosTable[i] = (int16_t)RoundDbl(cos(t) * one_Q14);
    apSinTable[i] = (int16_t)RoundDbl(sin(t) * one_Q14);
  }

  // Read the last valid angle
  double t         = apTheta[thetaLast];

  // Convert double to fixed point: signed 2Q14
  int16_t cosTheta = (int16_t)RoundDbl(cos(t) * one_Q14);
  int16_t sinTheta = (int16_t)RoundDbl(sin(t) * one_Q14);

  // Duplicate the last Theta value in the remaining free spots from the input buffers
  for(int i = thetaLast; i < aTableCount; ++i)
  {
    apCosTable[i] = cosTheta;
    apSinTable[i] = sinTheta;
  }
}

/* Returns the Sin/Cos table for polar coordinate lines */
static void GetCosSinTable(int16_t *apCosTable,
                           int16_t *apSinTable,
                           int aTableCount,
                           float *apTheta,
                           int aThetaCount,
                           int aCuCount)
{
  // Get the cosine and sine lookup tables for no NMS between angles  
  // one_Q14 = 1 * 2^14
  int16_t one_Q14 = (int16_t) 0x4000;

  // First angle padding
  double t = apTheta[-1];
  apCosTable[0] = (int16_t)RoundDbl(cos(t) * one_Q14);
  apSinTable[0] = (int16_t)RoundDbl(sin(t) * one_Q14);

  int tableID = 1;

  for(int thetaID = 0; thetaID < aThetaCount; ++thetaID, ++tableID)
  {
    if(tableID % aCuCount == 0)
    {
      thetaID -= 2;
    }

    double t = apTheta[thetaID];
    apCosTable[tableID] = (int16_t)RoundDbl(cos(t) * one_Q14);
    apSinTable[tableID] = (int16_t)RoundDbl(sin(t) * one_Q14);
  }

  // Read the last valid angle
  t = apTheta[aThetaCount];

  // Convert double to fixed point: signed 2Q14
  int16_t cosTheta = (int16_t)RoundDbl(cos(t) * one_Q14);
  int16_t sinTheta = (int16_t)RoundDbl(sin(t) * one_Q14);

  // Duplicate the last Theta value in the remaining free spots from the input buffers
  for(int i = tableID; i < aTableCount; ++i)
  {
    apCosTable[i] = cosTheta;
    apSinTable[i] = sinTheta;
  }
}

namespace apexcv
{
  int HoughLineDetector::mCuCnt         = HOUGH_CU_COUNT;
  int HoughLineDetector::mMaxRhoCnt     = HOUGH_ACCUM_BUFFER_SIZE - 2;
  int HoughLineDetector::mMaxLinesPerIt = HOUGH_MAX_LINES_PER_CU * HOUGH_CU_COUNT;
  double HoughLineDetector::mcDeg2Rad   = (1.745329251994329576923690768489e-2);
  double HoughLineDetector::mcRad2Deg   = (5.7295779513082320876798154814105e+1);
  double HoughLineDetector::mcPi        = (3.1415926535897932384626433832795e+0);
  
  /* Unpack the line information */
  int HoughLineDetector::GetAccumulator(PackedLine aLine)
  {
    /* Return accumulator value for the specified line  */
    return (uint16_t)(aLine >> 20);
  }

  /* Unpack the line information */
  int HoughLineDetector::GetRhoId(PackedLine aLine)
  {
    /* Return the rho value for the specified line  */
    aLine <<= 12;
    aLine >>= 20;

    return (int)aLine;
  }

  /* Unpack the line information */
  int HoughLineDetector::GetThetaId(PackedLine aLine)
  {
    /* Return the theta value for the specified line  */
    return (uint8_t)aLine;
  }

  /* Algorithm parameters check */
  APEXCV_LIB_RESULT HoughLineDetector::CheckParameters(int aImageCols,
                                                       int aImageRows,
                                                       int aPixelThreshold,
                                                       int aAccThreshold,
                                                       int aThetaCount)
  {
    /* Just make sure the compiler doesn't complain */
    aPixelThreshold = aPixelThreshold;
    aAccThreshold   = aAccThreshold;

    /* Hough was designed to work on a few chunk sizes: 6x4, 10x4, 20x2 and 40x1 */
    /* This is where the image is checked such that it can be divided in the predifined chunk lists  */
    if((aImageCols == (HOUGH_CU_COUNT * 40) || aImageCols == (HOUGH_CU_COUNT * 20) || aImageCols == (HOUGH_CU_COUNT * 10) || aImageCols == (HOUGH_CU_COUNT * 6)) &&
       0 < aImageRows &&       
       (0 < aThetaCount) && (aThetaCount <= 256))
    {
      /* Calculate the number of discreet Rho's */
      int rhoCount = ReadRhoCount(aImageCols, aImageRows);

      /* Check if you exceed the maximum number for RhoCnt */
      if(rhoCount <= mMaxRhoCnt)
      {
        return APEXCV_SUCCESS;
      }
    }

    return APEXCV_ERROR_INVALID_ARGUMENT;
  }

/*!*********************************************************************************
*  \brief Constructor with initialization
*  Allocate resources for the Hough transform ACF process.
***********************************************************************************/
  HoughLineDetector::HoughLineDetector()
    : mInit(0), mpSelectedProcess(0),
    mImageCols(0), mImageRows(0), mRhoStart(0), mThetaCount(0), mIterations(0), mLineCount(0),
    mpRhoCountOAL(0), mpCuCountOAL(0), mpPixelThresholdOAL(0), mpPixelOffsetParamsOAL(0), mpCuIndexParamsOAL(0), mpLineThresholdParamsOAL(0), mpLineCountOAL(0),
    mpThetaOAL(0), mpTheta(0), mpCosTableOAL(0), mpCosTable(0), mpSinTableOAL(0), mpSinTable(0), mpLinesOAL(0), mpLines(0)
  {
  }

  HoughLineDetector::~HoughLineDetector()
  {
    Release();
  }

  APEXCV_LIB_RESULT HoughLineDetector::Allocate()
  {
    APEXCV_LIB_RESULT status = APEXCV_SUCCESS;

    status = Release();

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }
    
    /* Dynamic allocation of the DDR buffers that will be used by APEX */    
    mpCuCountOAL             = OAL_MemoryAllocFlag(2, OAL_MEMORY_FLAG_CONTIGUOUS);
    mpRhoCountOAL            = OAL_MemoryAllocFlag(2, OAL_MEMORY_FLAG_CONTIGUOUS);
    mpPixelThresholdOAL      = OAL_MemoryAllocFlag(2, OAL_MEMORY_FLAG_CONTIGUOUS);
    mpPixelOffsetParamsOAL   = OAL_MemoryAllocFlag(8, OAL_MEMORY_FLAG_CONTIGUOUS);
    mpLineThresholdParamsOAL = OAL_MemoryAllocFlag(12, OAL_MEMORY_FLAG_CONTIGUOUS);
    mpCuIndexParamsOAL       = OAL_MemoryAllocFlag(4, OAL_MEMORY_FLAG_CONTIGUOUS);
    mpLineCountOAL           = OAL_MemoryAllocFlag(4, OAL_MEMORY_FLAG_CONTIGUOUS);

    /* If all pointers are valid and the memory was allocated correctly, proceed */
    if(mpCuCountOAL && mpRhoCountOAL && mpPixelThresholdOAL && mpPixelOffsetParamsOAL && mpLineThresholdParamsOAL)
    {
      /* Manual linkage between Apex buffers and the preallocated buffers */
      mLineCountDesc.InitManual(1, OAL_MemoryReturnAddress(mpLineCountOAL, ACCESS_CH_WB), OAL_MemoryReturnAddress(mpLineCountOAL, ACCESS_PHY), icp::DATATYPE_32S, 1);
      mCuCount.InitManual(1, OAL_MemoryReturnAddress(mpCuCountOAL, ACCESS_CH_WB), OAL_MemoryReturnAddress(mpCuCountOAL, ACCESS_PHY), icp::DATATYPE_16S, 1);
      mRhoCount.InitManual(1, OAL_MemoryReturnAddress(mpRhoCountOAL, ACCESS_CH_WB), OAL_MemoryReturnAddress(mpRhoCountOAL, ACCESS_PHY), icp::DATATYPE_16S, 1);
      mPixelThreshold.InitManual(1, OAL_MemoryReturnAddress(mpPixelThresholdOAL, ACCESS_CH_WB), OAL_MemoryReturnAddress(mpPixelThresholdOAL, ACCESS_PHY), icp::DATATYPE_16S, 1);
      mPixelOffsetParams.InitManual(4, OAL_MemoryReturnAddress(mpPixelOffsetParamsOAL, ACCESS_CH_WB), OAL_MemoryReturnAddress(mpPixelOffsetParamsOAL, ACCESS_PHY), icp::DATATYPE_16S, 1);
      mCuIndexParams.InitManual(2, OAL_MemoryReturnAddress(mpCuIndexParamsOAL, ACCESS_CH_WB), OAL_MemoryReturnAddress(mpCuIndexParamsOAL, ACCESS_PHY), icp::DATATYPE_16S, 1);
      mLineThresholdParams.InitManual(6, OAL_MemoryReturnAddress(mpLineThresholdParamsOAL, ACCESS_CH_WB), OAL_MemoryReturnAddress(mpLineThresholdParamsOAL, ACCESS_PHY), icp::DATATYPE_16S, 1);

      /* Read the base addresses */
      int16_t *pCuCount             = (int16_t *)AddrDataDesc(mCuCount);
      int16_t *pPixelOffsetParams   = (int16_t *)AddrDataDesc(mPixelOffsetParams);
      int16_t *pCuIndexParams       = (int16_t *)AddrDataDesc(mCuIndexParams);
      int16_t *pLineThresholdParams = (int16_t *)AddrDataDesc(mLineThresholdParams);
      *pCuCount               = *pPixelOffsetParams = pCuIndexParams[1] = pLineThresholdParams[5] = (int16_t)mCuCnt;
      pLineThresholdParams[3] = HOUGH_MAX_LINES_PER_CU;
    }
    else
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    return APEXCV_SUCCESS;
  }

  /* Reasource deallocation */
  APEXCV_LIB_RESULT HoughLineDetector::Release()
  {
    /* Freeing all the preallocated buffers at HoughLineDetector::Allocate() */
    if(mpLineCountOAL)
    {
      OAL_MemoryFree(mpLineCountOAL);
    }

    if(mpCosTableOAL)
    {
      OAL_MemoryFree(mpCosTableOAL);
    }

    if(mpSinTableOAL)
    {
      OAL_MemoryFree(mpSinTableOAL);
    }

    if(mpCuCountOAL)
    {
      OAL_MemoryFree(mpCuCountOAL);
    }

    if(mpRhoCountOAL)
    {
      OAL_MemoryFree(mpRhoCountOAL);
    }

    if(mpPixelThresholdOAL)
    {
      OAL_MemoryFree(mpPixelThresholdOAL);
    }

    if(mpPixelOffsetParamsOAL)
    {
      OAL_MemoryFree(mpPixelOffsetParamsOAL);
    }

    if(mpCuIndexParamsOAL)
    {
      OAL_MemoryFree(mpCuIndexParamsOAL);
    }

    if(mpLineThresholdParamsOAL)
    {
      OAL_MemoryFree(mpLineThresholdParamsOAL);
    }

    if(mpThetaOAL)
    {
      OAL_MemoryFree(mpThetaOAL);
    }

    if(mpLinesOAL)
    {
      OAL_MemoryFree(mpLinesOAL);
    }
    
    /* Freeing the memory allocated to the process descriptors */
    if(NULL != mpSelectedProcess)
    {
      switch(mProcessType)
      {
        /* Chunk size: 40x1 */
        case PROC_40X1:
        {
          delete (HOUGH_40X1 *) mpSelectedProcess;
        }
        break;

        /* Chunk size: 20x2 */
        case PROC_20X2:
        {
          delete (HOUGH_20X2 *) mpSelectedProcess;
        }
        break;

        /* Chunk size: 10x4 */
        case PROC_10X4:
        {
          delete (HOUGH_10X4 *) mpSelectedProcess;
        }
        break;

        /* Chunk size: 6x4 */
        case PROC_6X4:
        {
          delete (HOUGH_6X4 *) mpSelectedProcess;
        }
        break;
      } 
    }

    /* Zero out all the class parameters */
    mInit = 0;
    mpSelectedProcess = 0;
    mProcessType = PROC_NONE;
    mImageCols = 0;
    mImageRows = 0;
    mRhoStart = 0;
    mThetaCount = 0;
    mIterations = 0;
    mLineCount = 0;
    mpRhoCountOAL = 0;
    mpCuCountOAL = 0;
    mpPixelThresholdOAL = 0;
    mpPixelOffsetParamsOAL = 0;
    mpCuIndexParamsOAL = 0;
    mpLineThresholdParamsOAL = 0;
    mpLineCountOAL = 0;
    mpThetaOAL = 0;
    mpTheta = 0;
    mpCosTableOAL = 0;
    mpCosTable = 0;
    mpSinTableOAL = 0;
    mpSinTable = 0;
    mpLinesOAL = 0;
    mpLines = 0;

    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT HoughLineDetector::SelectProcess(int aProcessID)
  {
    /* The function allocates memory for the process id read as a parameter */
    int blockCols;

    switch(aProcessID)
    {
      /* Chunk size: 40x1 */
      case PROC_40X1:
      {
        mpSelectedProcess = new HOUGH_40X1;
        mProcessType = PROC_40X1;
        blockCols    = 40;
      }
      break;

      /* Chunk size: 20x2 */
      case PROC_20X2:
      {
        mpSelectedProcess = new HOUGH_20X2;
        mProcessType = PROC_20X2;
        blockCols    = 20;
      }
      break;

      /* Chunk size: 10x4 */
      case PROC_10X4:
      {
        mpSelectedProcess = new HOUGH_10X4;
        mProcessType = PROC_10X4;
        blockCols = 10;
      }
      break;

      /* Chunk size: 6x4 */
      case PROC_6X4:
      {
        mpSelectedProcess = new HOUGH_6X4;
        mProcessType = PROC_6X4;
        blockCols = 6;
      }
      break;

      /* Return an error on default */
      default:
        mpSelectedProcess = 0;
        return APEXCV_ERROR_INVALID_ARGUMENT;
    }   

    /* Set the block column */
    int16_t *pPixelOffsetParams = (int16_t *)AddrDataDesc(mPixelOffsetParams);
    pPixelOffsetParams[1] = blockCols;

    return APEXCV_SUCCESS;
  }

  /* Set-up pixel threshold */
  /* Minimum value for the pixel to be qualified for the accumulator */
  APEXCV_LIB_RESULT HoughLineDetector::SetPixelThreshold(int aPixelThreshold)
  {
    int16_t *pPixelThreshold = (int16_t *)AddrDataDesc(mPixelThreshold);

    if(pPixelThreshold)
    {
      // Clip the pixel threshold
      *pPixelThreshold = (int16_t)aPixelThreshold;

      return APEXCV_SUCCESS;
    }
    else
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
  }

  /* Set-up accumulator threshold */
  /* Minimum value for the pair <rho, theta> to be considered a line */
  APEXCV_LIB_RESULT HoughLineDetector::SetAccumThreshold(int aAccThreshold)
  {
    int16_t *pLineThresholdParams = (int16_t *)AddrDataDesc(mLineThresholdParams);

    if(pLineThresholdParams)
    {
      pLineThresholdParams[2] = (int16_t)aAccThreshold;

      return APEXCV_SUCCESS;
    }
    else
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
  }
  
  /* Overrides the number of rho counts that will be used for generating lines */
  APEXCV_LIB_RESULT HoughLineDetector::SetRhoCount(int aRhoCount)
  {
    int16_t *pRhoCount            = (int16_t *)AddrDataDesc(mRhoCount);
    int16_t *pLineThresholdParams = (int16_t *)AddrDataDesc(mLineThresholdParams);

    if(pRhoCount && pLineThresholdParams)
    {
      *pRhoCount = *pLineThresholdParams = (int16_t)aRhoCount;
      mRhoStart  = -(aRhoCount >> 1);

      return APEXCV_SUCCESS;
    }
    else
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
  }

  /* Overrides the number of rho counts that will be used for generating lines */
  APEXCV_LIB_RESULT HoughLineDetector::SetThetaCount(int aThetaCount, int aNonMaxSupp)
  {
    APEXCV_LIB_RESULT status = APEXCV_ERROR_INVALID_ARGUMENT;

    int16_t *pLineThresholdParams = (int16_t *)AddrDataDesc(mLineThresholdParams);

    if(pLineThresholdParams == NULL)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    if(pLineThresholdParams && aThetaCount > 0 && aThetaCount <= 256)
    {
      /* If the flag is invalid, use non-maxima suppression in both dimensions by default */
      int defaultFlag = NMS_RHO | NMS_THETA;

      if(aNonMaxSupp < NMS_NONE || aNonMaxSupp > (NMS_RHO | NMS_THETA))
      {
        aNonMaxSupp = defaultFlag;
      }

      if(aThetaCount == 1)
      {
        /* Disable NMS between angles if there is only one angle */
        aNonMaxSupp &= NMS_RHO;
      }

      /* If the code is here everything should be ok */
      status = APEXCV_SUCCESS;

      if(aThetaCount != mThetaCount || pLineThresholdParams[4] != aNonMaxSupp)
      {
        pLineThresholdParams[4] = (int16_t)aNonMaxSupp;

        /* Get the number of process iterations */
        mThetaCount = aThetaCount;
        int anglesPerIteration = (aNonMaxSupp & NMS_THETA) ? mCuCnt - 2 : mCuCnt;
        mIterations = (mThetaCount + anglesPerIteration - 1) / anglesPerIteration;

        /* Release the old buffers */
        if(mpCosTableOAL)
        {
          OAL_MemoryFree(mpCosTableOAL);
        }

        if(mpSinTableOAL)
        {
          OAL_MemoryFree(mpSinTableOAL);
        }

        if(mpLinesOAL)
        {
          OAL_MemoryFree(mpLinesOAL);
        }

        if(mpThetaOAL)
        {
          OAL_MemoryFree(mpThetaOAL);
        }

        /* Allocate the new buffers */
        int tableSize       = mCuCnt * mIterations;
        int tableBufferSize = sizeof(int16_t) * tableSize;

        /* Add 2 for padding (used with non-maxima suppression */
        int thetaBufferSize = sizeof(float) * (mThetaCount + 2);

        /* AnglesPerIteration * mIterations */
        int lineBufferSize = sizeof(PackedLine) * HOUGH_MAX_LINES_PER_CU * mThetaCount;

        mpCosTableOAL = OAL_MemoryAllocFlag(tableBufferSize, OAL_MEMORY_FLAG_CONTIGUOUS);
        mpSinTableOAL = OAL_MemoryAllocFlag(tableBufferSize, OAL_MEMORY_FLAG_CONTIGUOUS);
        mpLinesOAL    = OAL_MemoryAllocFlag(lineBufferSize, OAL_MEMORY_FLAG_CONTIGUOUS);
        mpThetaOAL    = OAL_MemoryAllocFlag(thetaBufferSize, OAL_MEMORY_FLAG_CONTIGUOUS);

        mpCosTable = (int16_t *)OAL_MemoryReturnAddress(mpCosTableOAL, ACCESS_CH_WB);

        if(mpCosTable == NULL)
        {
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }

        mpSinTable = (int16_t *)OAL_MemoryReturnAddress(mpSinTableOAL, ACCESS_CH_WB);

        if(mpSinTable == NULL)
        {
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }

        mpLines = (PackedLine *)OAL_MemoryReturnAddress(mpLinesOAL, ACCESS_CH_WB);

        if(mpLines == NULL)
        {
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }

        mpTheta = ((float *)OAL_MemoryReturnAddress(mpThetaOAL, ACCESS_CH_WB));

        if(mpTheta == NULL)
        {
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpTheta += 1;

        mCosTable.InitManual(mCuCnt, mpCosTable, OAL_MemoryReturnAddress(mpCosTableOAL, ACCESS_PHY), icp::DATATYPE_16S, 1);
        mSinTable.InitManual(mCuCnt, mpSinTable, OAL_MemoryReturnAddress(mpSinTableOAL, ACCESS_PHY), icp::DATATYPE_16S, 1);
        mLines.InitManual(mMaxLinesPerIt, mpLines, OAL_MemoryReturnAddress(mpLinesOAL, ACCESS_PHY), icp::DATATYPE_32U, 1);
      }
    }

    return status;
  }

  APEXCV_LIB_RESULT HoughLineDetector::SetTheta(int aThetaCount, float *apThetaData, int aNonMaxSupp)
  {
    /* The function can generate the rotation coefficients in two ways:
       1. User defined data - this is done if (apThetaData != NULL), it will copy
          the external data to the internal buffer that Apex will read
       2. Linear generated angles - divides the [0, 2*pi] interval in equally
          spaced values, the angle step per iteration is PI/aThetaCount
      
      The function will also generate the sinus and cosinus look-up tables for
      each angle value.

      The interval of angle values is [0, 2*pi]

    */

    APEXCV_LIB_RESULT status = APEXCV_ERROR_INVALID_ARGUMENT;

    status = SetThetaCount(aThetaCount, aNonMaxSupp);

    if(status == APEXCV_SUCCESS)
    {
      /* Initialize the angle values */
      if(apThetaData)
      {
        /* Linearly extrapolate first two values to get the first padded angle */
        /* Copy float values */
        for(int i = 0; i < mThetaCount; ++i)
        {
          mpTheta[i] = (float)WrapTheta(apThetaData[i]);
        }

        if(mThetaCount > 1)
        {
          /* Linearly extrapolate for the padded values */
          mpTheta[-1]          = (float)WrapTheta(2 * apThetaData[0] - apThetaData[1]);
          mpTheta[mThetaCount] = (float)WrapTheta(2 * apThetaData[mThetaCount - 1] - apThetaData[mThetaCount - 2]);
        }
        else
        {
          /* Use a dummy value (0) since NMS is disabled */
          mpTheta[-1]          = 0;
          mpTheta[mThetaCount] = 0;
        }
      }
      else
      {
        /* Divide the angle range evenly */
        double thetaStep = mcPi / mThetaCount;

        for(int i = -1; i < mThetaCount + 1; ++i)
        {
          mpTheta[i] = (float)WrapTheta(thetaStep * i);
        }
      }

      int16_t *pLineThresholdParams = (int16_t *)AddrDataDesc(mLineThresholdParams);

      /* Initialize the fixed point cosine and sine lookup tables */
      if(pLineThresholdParams[4] & NMS_THETA)
      {
        GetCosSinTable(mpCosTable, mpSinTable, mIterations * mCuCnt, mpTheta, mThetaCount, mCuCnt);
      }
      else
      {
        GetCosSinTableNoNms(mpCosTable, mpSinTable, mIterations * mCuCnt, mThetaCount, mpTheta);
      }
    }

    return status;
  }
  
  APEXCV_LIB_RESULT HoughLineDetector::SetTheta(int aThetaCount, double aThetaStart, double aThetaStep, int aNonMaxSupp)
  {
    /* This is another method of generating the angle values besides:
       HoughLineDetector::SetTheta(int aThetaCount, float *apThetaData, int aNonMaxSupp)

      The second variant of SetTheta() one can restrict the angle values between:

      aThetaStop = aThetaStart + aThetaStep * aThetaCount
      [aThetaStart, aThetaStop]    
    
    */
    APEXCV_LIB_RESULT status = SetThetaCount(aThetaCount, aNonMaxSupp);

    if(status == APEXCV_SUCCESS)
    {
      // Initialize the angle values
      for(int i = -1; i < aThetaCount + 1; ++i)
      {
        mpTheta[i] = (float)WrapTheta(aThetaStep * i + aThetaStart);
      }

      int16_t *pLineThresholdParams = (int16_t *)AddrDataDesc(mLineThresholdParams);

      // Initialize the fixed point cosine and sine lookup tables
      if(pLineThresholdParams[4] & NMS_THETA)
      {
        GetCosSinTable(mpCosTable, mpSinTable, mIterations * mCuCnt, mpTheta, mThetaCount, mCuCnt);
      }
      else
      {
        GetCosSinTableNoNms(mpCosTable, mpSinTable, mIterations * mCuCnt, mThetaCount, mpTheta);
      }
    }

    return status;
  }

/*!*****************************************************************************
*  \brief Set the image size the detector will receive.
*  \return Error code for initialization (zero on success).
*******************************************************************************/
  APEXCV_LIB_RESULT HoughLineDetector::SetImageSize(int aImageCols,
                                                    int aImageRows)
  {
    const int nProcess        = 4;
    int processCols[nProcess] =
    {
      HOUGH_CU_COUNT * 40,
      HOUGH_CU_COUNT * 20,
      HOUGH_CU_COUNT * 10,
      HOUGH_CU_COUNT * 6
    };

    APEXCV_LIB_RESULT status = APEXCV_ERROR_FAILURE;

    // Do this until subsizes are supported
    if((aImageCols == processCols[0] ||
        aImageCols == processCols[1] ||
        aImageCols == processCols[2] ||
        aImageCols == processCols[3]) &&
       (aImageRows > 0))
    {
      if((aImageCols != mImageCols) &&
         (aImageRows != mImageRows))
      {
        int rhoCount = ReadRhoCount(aImageCols, aImageRows);

        // Check all necessary conditions for initialization
        if(rhoCount <= mMaxRhoCnt) // the rho count is not too large
        {
          mImageCols = aImageCols;
          mImageRows = aImageRows;
          int16_t *pPixelOffsetParams = (int16_t *)AddrDataDesc(mPixelOffsetParams);
          pPixelOffsetParams[2] = (int16_t)(aImageCols >> 1);
          pPixelOffsetParams[3] = (int16_t)(aImageRows >> 1);
          SetRhoCount(rhoCount);

          /* Get the optimal ACF process */
          int optimalProcess = PROC_NONE;

          /* For now, only exact image widths are supported */
          for(int p = 0; p < nProcess; ++p)
          {
            if(aImageCols == processCols[p])
            {
              optimalProcess = (1 << (p + 1)) >> 1;
              break;
            }
          }

          status = SelectProcess(optimalProcess);

          if(status != APEXCV_SUCCESS)
          {
            return status;
          }
        }
      }
      else
      {
        status = APEXCV_SUCCESS;
      }
    }

    return status;
  }

  APEXCV_LIB_RESULT HoughLineDetector::ConnectIO(vsdk::SUMat &aImage)
  {
    /* Similar to other libs inside ApexCV Base/Pro.
    *    
    *   The basic functionality is to initialize and connect all the external
    *   buffers to the Hough Graphs.
    *   
    *   The process is selected based on the desired chunk size
    *   
    */ 
    APEXCV_LIB_RESULT status = APEXCV_SUCCESS;
    
    ACF_Process * pProcess = ((ACF_Process *) mpSelectedProcess);
    
    status = pProcess->Initialize();

    if(status != APEXCV_SUCCESS)
    {
      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }

    status = pProcess->ConnectIO("PIXEL_THRESHOLD", mPixelThreshold);

    if(status != APEXCV_SUCCESS)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    status = pProcess->ConnectIO("PIXEL_OFFSET_PARAMS", mPixelOffsetParams);

    if(status != APEXCV_SUCCESS)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    status = pProcess->ConnectIO("CU_COUNT", mCuCount);

    if(status != APEXCV_SUCCESS)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    status = pProcess->ConnectIO("RHO_COUNT", mRhoCount);

    if(status != APEXCV_SUCCESS)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    status = pProcess->ConnectIO("CU_INDEX_PARAMS", mCuIndexParams);

    if(status != APEXCV_SUCCESS)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    status = pProcess->ConnectIO("LINE_THRESHOLD_PARAMS", mLineThresholdParams);

    if(status != APEXCV_SUCCESS)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    status = pProcess->ConnectIO("LINE_COUNT", mLineCountDesc);

    if(status != APEXCV_SUCCESS)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    /* Link the input image to the pointer inside the class */
    mpInputImage = &aImage;

    status = pProcess->ConnectIO("IMAGE", aImage);

    if(status != APEXCV_SUCCESS)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }
    {
      ApexcvHostBaseBaseClass::InfoCluster lInfo;
      lInfo.set_ACF("HOUGH_PROCESS", pProcess);
      lInfo.push_PortName("IMAGE");
      mvInfoClusters.push_back(lInfo);
    }
    return APEXCV_SUCCESS;
  }

/*!*****************************************************************************
 *  \brief Allocate resources for the Hough transform ACF process.
 *  \return Error code for the ACF initialization (zero on success).
 ******************************************************************************/

  /* The initialization are customized for the type of theta generation */
  APEXCV_LIB_RESULT HoughLineDetector::Initialize(vsdk::SUMat &aImage,
                                                  int aPixelThreshold,
                                                  int aAccThreshold,
                                                  int aThetaCount,
                                                  float *apTheta,
                                                  int aNonMaxSupp)
  {
#if (HOUGH_PERF_PROB != 0)
    gHoughTime[HOUGH_INIT][Hough_Init_Ref] = FSL_Ticks();
#endif
    
   /* Theta can be generated externally of linearly based on  aThetaCount*/
    
    /* 1. Allocates all the internal buffers for the algorithm to work
     * 2. Sets the pixel threshold for a pixel to be take into account
     * 3. Sets the accumulator threshold that defines the minim accumulator value
     *    for a pair <rho, theta> to be considered a line
     * 4. Generates all the angles and calculates sin/cos for the line rotations
     * 5. Links the image to the process
     * 6. Connects all the graph ports
     */

    APEXCV_LIB_RESULT status = APEXCV_SUCCESS;

    if(mInit == false)
    {      
      status = Allocate();

      if(status != APEXCV_SUCCESS)
      {
        Release();

        return status;
      }

#if (HOUGH_PERF_PROB != 0)
      gHoughTime[HOUGH_INIT][Hough_Init_Mem_Alloc] = abs(FSL_Ticks() - gHoughTime[HOUGH_INIT][Hough_Init_Ref]) / cTimeScale;
      gHoughTime[HOUGH_INIT][Hough_Init_Ref]       = FSL_Ticks();
#endif

      status = SetPixelThreshold(aPixelThreshold);

      if(status != APEXCV_SUCCESS)
      {
        Release();

        return status;
      }

      status = SetAccumThreshold(aAccThreshold);

      if(status != APEXCV_SUCCESS)
      {
        Release();

        return status;
      }

      status = SetTheta(aThetaCount, apTheta, aNonMaxSupp);

      if(status != APEXCV_SUCCESS)
      {
        Release();

        return status;
      }

      status = SetImageSize(aImage.cols, aImage.rows);

      if(status != APEXCV_SUCCESS)
      {
        Release();

        return status;
      }

      status = ConnectIO(aImage);

      if(status != APEXCV_SUCCESS)
      {
        Release();

        return status;
      }

      mInit = true;
    }
    else
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

#if (HOUGH_PERF_PROB != 0)
    gHoughTime[HOUGH_INIT][Hough_Init_Process_Init] = abs(FSL_Ticks() - gHoughTime[HOUGH_INIT][Hough_Init_Ref]) / cTimeScale;
#endif

    return APEXCV_SUCCESS;
  }

/*!*****************************************************************************
*  \brief Allocate resources for the Hough transform ACF process.
*  \return Error code for the ACF initialization (zero on success).
*******************************************************************************/

  /* Theta is generated based on aThetaStart, aThetaStep and aThetaCount */
  APEXCV_LIB_RESULT HoughLineDetector::Initialize(vsdk::SUMat &aImage,
                                                  int aPixelThreshold,
                                                  int aAccThreshold,
                                                  int aThetaCount,
                                                  double aThetaStart,
                                                  double aThetaStep,
                                                  int aNonMaxSupp)
  {
#if (HOUGH_PERF_PROB != 0)
    gHoughTime[HOUGH_INIT][Hough_Init_Ref] = FSL_Ticks();
#endif

    /* 1. Allocates all the internal buffers for the algorithm to work
     * 2. Sets the pixel threshold for a pixel to be take into account
     * 3. Sets the accumulator threshold that defines the minim accumulator value
     *    for a pair <rho, theta> to be considered a line
     * 4. Generates all the angles and calculates sin/cos for the line rotations
     * 5. Links the image to the process
     * 6. Connects all the graph ports
     */

    APEXCV_LIB_RESULT status = APEXCV_SUCCESS;

    if(mInit == false)
    {
      status = Allocate();

      if(status != APEXCV_SUCCESS)
      {
        Release();

        return status;
      }

#if (HOUGH_PERF_PROB != 0)
      gHoughTime[HOUGH_INIT][Hough_Init_Mem_Alloc] = abs(FSL_Ticks() - gHoughTime[HOUGH_INIT][Hough_Init_Ref]) / cTimeScale;
      gHoughTime[HOUGH_INIT][Hough_Init_Ref]       = FSL_Ticks();
#endif

      status = SetPixelThreshold(aPixelThreshold);

      if(status != APEXCV_SUCCESS)
      {
        Release();

        return status;
      }

      status = SetAccumThreshold(aAccThreshold);

      if(status != APEXCV_SUCCESS)
      {
        Release();

        return status;
      }

      status = SetTheta(aThetaCount, aThetaStart, aThetaStep, aNonMaxSupp);

      if(status != APEXCV_SUCCESS)
      {
        Release();

        return status;
      }

      status = SetImageSize(aImage.cols, aImage.rows);

      if(status != APEXCV_SUCCESS)
      {
        Release();

        return status;
      }

      status = ConnectIO(aImage);

      if(status != APEXCV_SUCCESS)
      {
        Release();

        return status;
      }

      mInit = true;
    }
    else
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

#if (HOUGH_PERF_PROB != 0)
    gHoughTime[HOUGH_INIT][Hough_Init_Process_Init] = abs(FSL_Ticks() - gHoughTime[HOUGH_INIT][Hough_Init_Ref]) / cTimeScale;
#endif

    return APEXCV_SUCCESS;
  }


  /* The ReConnect IO functions is very similar to Initialize() 
  * Its API flavors are dependent of the style of generating the theta samples !  
  */
  APEXCV_LIB_RESULT HoughLineDetector::ReconnectIO(vsdk::SUMat &aImage,
                                                   int aPixelThreshold,
                                                   int aAccThreshold,
                                                   int aThetaCount,
                                                   float *apTheta,
                                                   int aNonMaxSupp)
  {
#if (HOUGH_PERF_PROB != 0)
    gHoughTime[HOUGH_RECONN][Hough_Reconn_Ref] = FSL_Ticks();
#endif

    /* The functions first deallocates old resources and then calls the appropriate Initialize() */    

    APEXCV_LIB_RESULT status = APEXCV_SUCCESS;

    if(mInit == true)
    {
      status = Release();

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }

#if (HOUGH_PERF_PROB != 0)
    gHoughTime[HOUGH_RECONN][Hough_Reconn_Dealloc] = abs(FSL_Ticks() - gHoughTime[HOUGH_RECONN][Hough_Reconn_Ref]) / cTimeScale;
    gHoughTime[HOUGH_RECONN][Hough_Reconn_Ref]     = FSL_Ticks();
#endif

    status = Initialize(aImage,
                        aPixelThreshold,
                        aAccThreshold,
                        aThetaCount,
                        apTheta,
                        aNonMaxSupp);

#if (HOUGH_PERF_PROB != 0)
    gHoughTime[HOUGH_RECONN][Hough_Reconn_Run] = abs(FSL_Ticks() - gHoughTime[HOUGH_RECONN][Hough_Reconn_Ref]) / cTimeScale;
#endif

    return status;
  }

 /* The ReConnect IO functions is very similar to Initialize() 
  * Its API flavors are dependent of the style of generating the theta samples !  
  */
  APEXCV_LIB_RESULT HoughLineDetector::ReconnectIO(vsdk::SUMat &aImage,
                                                   int aPixelThreshold,
                                                   int aAccThreshold,
                                                   int aThetaCount,
                                                   double aThetaStart,
                                                   double aThetaStep,
                                                   int aNonMaxSupp)
  {
#if (HOUGH_PERF_PROB != 0)
    gHoughTime[HOUGH_RECONN][Hough_Reconn_Ref] = FSL_Ticks();
#endif

    /* The functions first deallocates old resources and then calls the appropriate Initialize() */    

    APEXCV_LIB_RESULT status = APEXCV_SUCCESS;

    if(mInit == true)
    {
      status = Release();

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }

#if (HOUGH_PERF_PROB != 0)
    gHoughTime[HOUGH_RECONN][Hough_Reconn_Dealloc] = abs(FSL_Ticks() - gHoughTime[HOUGH_RECONN][Hough_Reconn_Ref]) / cTimeScale;
    gHoughTime[HOUGH_RECONN][Hough_Reconn_Ref]     = FSL_Ticks();
#endif

    status = Initialize(aImage,
                        aPixelThreshold,
                        aAccThreshold,
                        aThetaCount,
                        aThetaStart,
                        aThetaStep,
                        aNonMaxSupp);


#if (HOUGH_PERF_PROB != 0)
    gHoughTime[HOUGH_RECONN][Hough_Reconn_Run] = abs(FSL_Ticks() - gHoughTime[HOUGH_RECONN][Hough_Reconn_Ref]) / cTimeScale;
#endif

    return status;
  }

  APEXCV_LIB_RESULT HoughLineDetector::Process()
  {
#if (HOUGH_PERF_PROB != 0)
    gHoughTime[HOUGH_PROCESS][Hough_Process_Ref] = FSL_Ticks();
#endif

    APEXCV_LIB_RESULT status = APEXCV_SUCCESS;
    
    /* Connects the sin/cos precalculated look-up tables and the starts the APEX processing */

    if(mInit == true)
    {
      mLineCount = 0;

      int32_t *pLineCount     = (int32_t *)AddrDataDesc(mLineCountDesc);
      int16_t *pCuIndexParams = (int16_t *)AddrDataDesc(mCuIndexParams);
      ACF_Process_APU &rProcess = *((ACF_Process_APU *) mpSelectedProcess);

      int16_t *pLineThresholdParams = (int16_t *)AddrDataDesc(mLineThresholdParams);
      int anglesPerIteration;
      int thetaOffset;

      if(pLineThresholdParams[4] & NMS_THETA)
      {
        anglesPerIteration = mCuCnt - 2;
        thetaOffset        = -1;
      }
      else
      {
        anglesPerIteration = mCuCnt;
        thetaOffset        = 0;
      }

      int thetaRemainder = mThetaCount;
      FlushInvalidateDataDesc(mPixelThreshold);
      FlushInvalidateDataDesc(mPixelOffsetParams);
      FlushInvalidateDataDesc(mCuCount);
      FlushInvalidateDataDesc(mRhoCount);

      int16_t *pCosTablePhyAdd = (int16_t *)OAL_MemoryReturnAddress(mpCosTableOAL, ACCESS_PHY);
      int16_t *pSinTablePhyAdd = (int16_t *)OAL_MemoryReturnAddress(mpSinTableOAL, ACCESS_PHY);
      uint32_t *pLinesPhyAdd   = (uint32_t *)OAL_MemoryReturnAddress(mpLinesOAL, ACCESS_PHY);

      for(int i = 0; i < mIterations; ++i, thetaRemainder -= anglesPerIteration)
      {
        // Connect the Cos and Sin tables for this iteration
        pLineThresholdParams[1] = thetaRemainder > anglesPerIteration ? anglesPerIteration : thetaRemainder;
        int indexStart      = mCuCnt * i;
        int thetaIndexStart = anglesPerIteration * i + thetaOffset;
        *pCuIndexParams = thetaIndexStart;

        mCosTable.InitManual(mCuCnt, mpCosTable + indexStart, pCosTablePhyAdd + indexStart, icp::DATATYPE_16S, 1);
        mSinTable.InitManual(mCuCnt, mpSinTable + indexStart, pSinTablePhyAdd + indexStart, icp::DATATYPE_16S, 1);
        mLines.InitManual(mMaxLinesPerIt, mpLines + mLineCount, pLinesPhyAdd + mLineCount, icp::DATATYPE_32U, 1);

        status = rProcess.ConnectIO("COS_TABLE", mCosTable);

        if(status != APEXCV_SUCCESS)
        {
          Release();

          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }

        status = rProcess.ConnectIO("SIN_TABLE", mSinTable);

        if(status != APEXCV_SUCCESS)
        {
          Release();

          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }

        status = rProcess.ConnectIO("LINES", mLines);

        if(status != APEXCV_SUCCESS)
        {
          Release();

          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
                
        FlushInvalidateDataDesc(mLineThresholdParams);
        FlushInvalidateDataDesc(mCuIndexParams);
        FlushInvalidateDataDesc(mLineCountDesc);
        FlushInvalidateDataDesc(mCosTable);
        FlushInvalidateDataDesc(mSinTable);
        FlushInvalidateDataDesc(mLines);

        // Starting the process
        status = rProcess.Start();

        if(status != APEXCV_SUCCESS)
        {
          Release();

          return APEXCV_ERROR_ACF_PROC_EXEC;
        }

        status = rProcess.Wait();

        if(status != APEXCV_SUCCESS)
        {
          Release();

          return APEXCV_ERROR_ACF_PROC_EXEC;
        }

        if(status == APEXCV_SUCCESS)
        {
          mLineCount += (*pLineCount >> 2);
        }
        else
        {
          mLineCount = -1;
          break;
        }
      }
    }
    else
    {
      /* Release resources if an error is detected */
      Release();

      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

#if (HOUGH_PERF_PROB != 0)
    gHoughTime[HOUGH_PROCESS][Hough_Process_Run] = abs(FSL_Ticks() - gHoughTime[HOUGH_PROCESS][Hough_Process_Ref]) / cTimeScale;
#endif

    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT HoughLineDetector::SelectApuConfiguration(ACF_APU_CFG aApuConfig,
                                                              int32_t aApexId)
  {
    APEXCV_LIB_RESULT status;
    
    /* Select the device and configuration on which the kernel will run */
    if(mInit == true)
    {
      ACF_Process_APU &rProcess = *((ACF_Process_APU *) mpSelectedProcess);
      status = rProcess.SelectApuConfiguration(aApuConfig, (int32_t)aApexId);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }
    }
    else
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    return APEXCV_SUCCESS;
  }

  /* Helper methods for getting various information from the class */ 
  int HoughLineDetector::GetRhoCount()
  {
    return *((int16_t *)AddrDataDesc(mRhoCount));
  }

  int HoughLineDetector::GetRhoStart()
  {
    return mRhoStart;
  }

  int HoughLineDetector::GetThetaCount()
  {
    return mThetaCount;
  }

  float *HoughLineDetector::GetThetaData()
  {
    return mpTheta;
  }

  int HoughLineDetector::GetNmsFlag()
  {
    return ((int16_t *)AddrDataDesc(mLineThresholdParams))[3];
  }

  int HoughLineDetector::GetLineCount()
  {
    return mLineCount;
  }

  HoughLineDetector::PackedLine *HoughLineDetector::GetPackedLineData()
  {
    return mpLines;
  }

  HoughLineDetector::Line HoughLineDetector::GetLine(int aIndex)
  {
    if(aIndex >= 0 && aIndex < mLineCount)
    {
      PackedLine packedLine = mpLines[aIndex];

      return Line(GetRhoId(packedLine) + mRhoStart, mpTheta[GetThetaId(packedLine)]);
    }
    else
    {
      return Line();
    }
  }

  HoughLineDetector::Line HoughLineDetector::GetLine(PackedLine aPackedLine)
  {
    return Line(GetRhoId(aPackedLine) + mRhoStart, mpTheta[GetThetaId(aPackedLine)]);
  }
} /* namespace apex */
