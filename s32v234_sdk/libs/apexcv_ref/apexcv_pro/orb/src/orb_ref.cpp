/*******************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2016 - 2017, NXP Semiconductors
* All Rights Reserved
*
********************************************************************************
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
*******************************************************************************/

#include "orb_ref.h"

#ifdef __STANDALONE__
#include "t32term.h"
#endif

#define MAX_RADIUS 25
#define nDEBUG
using namespace std;

static const int gSinCosExp = 8;

static const unsigned char gTapsInsideCircle[MAX_RADIUS][18] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 4, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 5, 5, 4, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 6, 6, 5, 4, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 7, 7, 6, 6, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 8, 8, 7, 7, 6, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 9, 9, 8, 8, 7, 7, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 10, 10, 10, 9, 9, 8, 7, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 11, 11, 11, 10, 10, 9, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 12, 12, 12, 11, 11, 10, 10, 9, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 13, 13, 13, 12, 12, 12, 11, 10, 9, 8, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 14, 14, 14, 13, 13, 13, 12, 11, 11, 10, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 10, 0, 0, 0, 0, 0, 0, 0 },
  { 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 12, 11, 0, 0, 0, 0, 0, 0 },
  { 17, 17, 17, 17, 16, 16, 15, 15, 14, 14, 13, 12, 11, 0, 0, 0, 0, 0 },
  { 18, 18, 18, 18, 17, 17, 17, 16, 16, 15, 14, 13, 12, 0, 0, 0, 0, 0 },
  { 19, 19, 19, 19, 18, 18, 18, 17, 17, 16, 15, 15, 14, 13, 0, 0, 0, 0 },
  { 20, 20, 20, 20, 19, 19, 19, 18, 18, 17, 17, 16, 15, 14, 13, 0, 0, 0 },
  { 21, 21, 21, 21, 20, 20, 20, 19, 19, 18, 18, 17, 16, 16, 15, 0, 0, 0 },
  { 22, 22, 22, 22, 21, 21, 21, 20, 20, 20, 19, 18, 18, 17, 16, 15, 0, 0 },
  { 23, 23, 23, 23, 22, 22, 22, 22, 21, 21, 20, 20, 19, 18, 17, 17, 15, 0 },
  { 24, 24, 24, 24, 23, 23, 23, 23, 22, 22, 21, 21, 20, 19, 19, 18, 17, 0 },
  { 25, 25, 25, 25, 24, 24, 24, 24, 23, 23, 22, 22, 21, 21, 20, 19, 18, 17 }
};

void DumpPatch(vsdk::SUMat& aImg, int aCenterX, int aCenterY, int aPatchSize, char *apFileName)
{
#ifdef __STANDALONE__
  printf("Function did not print !\n");
#else
  vsdk::SMat imgMat                = aImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  int span                        = aImg.step[0];
  int halfPatch                   = aPatchSize >> 1;
  unsigned char *pBase            = (unsigned char *)imgMat.data;
  unsigned char *pCenter          = (unsigned char *)(pBase + aCenterX + aCenterY * span);
  unsigned char *pUpperLeftCorner = (unsigned char *)(pCenter - halfPatch - halfPatch * span);

  // Open the file in text mode for simplicity
  FILE *fp = fopen(apFileName, "wt");

  for(int y = 0; y < aPatchSize; y++)
  {
    for(int x = 0; x < aPatchSize; x++)
    {
      fprintf(fp, "%.2x ", *(pUpperLeftCorner + x + y * span));
    }

    // New line
    fprintf(fp, "\n");
  }

  // Close the file
  fclose(fp);
#endif
}

void PrintArrayHex(unsigned char *apData, int aSizeInBytes)
{
  printf("0x");

  for(int index = 0; index < aSizeInBytes; index++)
  {
    printf("%.2x", apData[index]);
  }

  printf("\n");

  return;
}

void SinFixedPoint256Levels(int16_t aX, int16_t *apMagnitude)
{
  // x is inside [0, 255]
  int16_t in;
  int32_t in_3;
  int32_t out;
  int16_t sign = 1;

  if(aX <= 128)
  {
    if(aX > 64)
    {
      aX = 128 - aX;
    }

    sign = 1;
  }
  else
  {
    if(aX <= 192)
    {
      aX = aX - 128;
    }
    else
    {
      aX = 256 - aX;
    }

    sign = -1;
  }

  in           = aX;
  in_3         = in * in * in;
  out          = (6 * in - ((in_3) >> 11));
  out          = out * sign;
  *apMagnitude = out;
}

void CosFixedPoint256Levels(int16_t aX, int16_t *apMagnitude)
{
  //x is: [0, 2*pi] and mapped to [0, 255]
  aX = 64 - aX;

  if(aX < 0)
  {
    aX = 256 + aX;
  }

  SinFixedPoint256Levels(aX, apMagnitude);
}

uint8_t Atan2FixedPoint256Levels(int32_t aY, int32_t aX)
{
  // The functions calculates fixed point atan2(y, x) and maps it from [0, 2*pi] to [0, 255]
  unsigned char a0       = 201;
  unsigned char a1       = 251;
  unsigned char a2       = 50;
  unsigned char rad2Taps = 41;

  int32_t out;
  int32_t absY = abs(aY);
  int32_t sum;
  int32_t r;
  int32_t r3;
  int32_t arctg;
  int32_t offset;

  sum = (aX + absY);
  int32_t diff;

  if(aX >= 0)
  {
    diff   = (aX - absY) * 256;
    r      = (diff / sum);
    offset = 0;
  }
  else
  {
    sum    = sum * 256;
    diff   = absY - aX;
    r      = sum / diff;
    offset = 64;
  }

  r3    = ((r * r * r) >> 16);
  arctg = (a0 - ((a1 * r) >> 8) + ((a2 * r3) >> 8));
  arctg = arctg * rad2Taps;

  out = arctg >> 8;
  out = out + offset;

  if(aY < 0)
  {
    out = -out;
  }

  if(out < 0)
  {
    out = out + 256;
  }

  if(aX == 0)
  {
    if(aY > 0)
    {
      // pi/2
      out = 64;
    }
    else
    if(aY < 0)
    {
      // 3/2 * pi
      out = 192;
    }
    else
    {
      out = 0;
    }
  }

  return (uint8_t)out;
}

int DumpUmat(const char *apFileName, vsdk::SUMat& aData)
{
  unsigned int elements = aData.rows * aData.step[0];
  unsigned int elemSize = 1;
  unsigned int wrBytes;

  // Open the desired file
#ifdef __STANDALONE__
  int fhandle = T32_fopen(apFileName, T32_TERM_O_CREATE_TRUNC | T32_TERM_O_WRONLY | T32_TERM_O_BINARY);
#else
  FILE *fHandler = fopen(apFileName, "wb");
  assert(NULL != fHandler);
#endif
#ifdef __STANDALONE__
  wrBytes = T32_fwrite(aData.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data, elemSize, elements, fhandle);
#else
  wrBytes = fwrite(aData.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data, elemSize, elements, fHandler);
#endif
  assert(wrBytes > 0);

#ifdef __STANDALONE__
  T32_fclose(fhandle);
#else
  fflush(fHandler);
  fclose(fHandler);
#endif

  return EXIT_SUCCESS;
}

void RemoveKeypointsCloseToBorder(vsdk::SUMat& aOut,
                                  vsdk::SUMat& aIn,
                                  unsigned int aW,
                                  unsigned int aH,
                                  unsigned int aBorderDistance,
                                  unsigned int *apNrOfKeypoints)
{
  vsdk::SMat outMat      = aOut.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  vsdk::SMat inMat       = aIn.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  uint16_t *pInData     = (uint16_t *)inMat.data;
  uint16_t *pOutData    = (uint16_t *)outMat.data;
  uint32_t newKeypoints = 0;

  int nrOfElements = (int)(*apNrOfKeypoints);
  // Clear the output data first !
  memset(pOutData, 0, aOut.step[0] * aOut.rows);

  for(int idx = 0; idx < nrOfElements; idx++)
  {
    uint16_t x = pInData[2 * idx + 0];
    uint16_t y = pInData[2 * idx + 1];

    if((x > aBorderDistance) && (y > aBorderDistance) && (x < (aW - aBorderDistance)) && (y < (aH - aBorderDistance)))
    {
      pOutData[2 * newKeypoints + 0] = x;
      pOutData[2 * newKeypoints + 1] = y;
      newKeypoints++;
    }
  }

  *apNrOfKeypoints = newKeypoints;

  return;
}

void CreateBinBriefDescriptor(vsdk::SUMat& aOut, vector<DescriptorInfo>& aInDescr)
{
  vsdk::SMat outMat    = aOut.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  uint8_t *pData      = (uint8_t *)outMat.data;
  int nrOfDescriptors = (int)aInDescr.size();
  int aSizeInBytes;

  for(int descId = 0; descId < nrOfDescriptors; descId++)
  {
    DescriptorInfo data = aInDescr[descId];
    aSizeInBytes = data.size;

    for(int byte = 0; byte < aSizeInBytes; byte++)
    {
      *pData++ = data.getByte(byte);
    }
  }

  return;
}

void CreateVectorOfKeypoints(vsdk::SUMat& aIn,
                             int aNrOfKeypoints,
                             vector<Keypoint>& aKeypnts)
{
  vsdk::SMat inMat   = aIn.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  uint16_t *pInData = (uint16_t *)inMat.data;

  // Clear the output data first !
  for(int idx = 0; idx < aNrOfKeypoints; idx++)
  {
    uint16_t x = pInData[2 * idx + 0];
    uint16_t y = pInData[2 * idx + 1];

    // Push a keypoint into the vector
    Keypoint newKeypoint;
    newKeypoint.location.x = x;
    newKeypoint.location.y = y;
    newKeypoint.angle      = 0;
    aKeypnts.push_back(newKeypoint);
  }
}

int ReadUmatGray(vsdk::SUMat& aImg, char * apFileName)
{
#ifdef __STANDALONE__
  int fhandle = T32_fopen(apFileName, T32_TERM_O_OPEN_EXISTING | T32_TERM_O_RDONLY | T32_TERM_O_BINARY);
  
  if(fhandle == 0)
  {
    printf("File: %s was not found !\n", apFileName);
    fflush(stdout);
    return EXIT_FAILURE;
  }
#else
  FILE *pFile = fopen(apFileName, "rb");

  if(pFile == NULL)
  {
    printf("File: %s was not found !\n", apFileName);
    fflush(stdout);
    return EXIT_FAILURE;
  }
#endif
#ifdef __STANDALONE__
  T32_fseek(fhandle, 0, T32_TERM_SEEK_END);
  int sizeBytes = T32_ftell(fhandle);
#else
  fseek(pFile, 0, SEEK_END);
  int sizeBytes = ftell(pFile);
#endif

#ifdef __STANDALONE__
  T32_fseek(fhandle, 0, T32_TERM_SEEK_SET);
  T32_fread(aImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data, 1, sizeBytes, fhandle);
  T32_fclose(fhandle);
#else
  rewind(pFile);
  fread(aImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data, 1, sizeBytes, pFile);
  fclose(pFile);
#endif

  return EXIT_SUCCESS;
}

void DumpAnglesFromVector(vsdk::SUMat& aOut, vector<Keypoint>& aKeypnts)
{
  // Clear the output data first !
  vsdk::SMat outMat   = aOut.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  uint16_t *pOutData = (uint16_t *)outMat.data;

  for(uint32_t idx = 0; idx < aKeypnts.size(); idx++)
  {
    pOutData[idx] = aKeypnts[idx].angle;
  }
}

int CalcIntensityCentroidAngle(vsdk::SUMat& aInputImg,
                               vector<Keypoint>& aKeypnts,
                               uint16_t aRadius)
{
  vsdk::SMat inputImgMat = aInputImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  unsigned char *pData  = (unsigned char *)inputImgMat.data;
  int span              = aInputImg.step[0];
  int aNrOfKeypoints    = aKeypnts.size();

  for(int keypntIdx = 0; keypntIdx < aNrOfKeypoints; keypntIdx++)
  {
    int m_01 = 0, m_10 = 0;

    Point centerCoordinates = aKeypnts[keypntIdx].location;
    int x                   = centerCoordinates.x;
    int y                   = centerCoordinates.y;
    int index               = y * span + x;
    unsigned char *center   = &pData[index];

    for(int u = -aRadius; u <= aRadius; ++u)
    {
      m_10 += u * center[u];
    }

    // Go line by line in the circular patch
    for(int v = 1; v <= aRadius; ++v)
    {
      // Proceed over the two lines
      int v_sum = 0;
      int d     = (int)(gTapsInsideCircle[aRadius - 1][v - 1]);

      for(int u = -d; u <= d; ++u)
      {
        int val_plus = center[u + v * span], val_minus = center[u - v * span];
        v_sum += (val_plus - val_minus);
        m_10  += u * (val_plus + val_minus);
      }

      m_01 += v * v_sum;
    }

    aKeypnts[keypntIdx].angle = Atan2FixedPoint256Levels(m_01, m_10);
  }

  return EXIT_SUCCESS;
}

unsigned int CalcHammingDistance(unsigned char *aLeft, unsigned char *aRight, int aSizeInBytes)
{
  int out = 0;

  for(int index = 0; index < aSizeInBytes; index += 64)
  {
    uint8x8_t in0        = vld1_u8((uint8_t const *)&aLeft[index]);
    uint8x8_t in1        = vld1_u8((uint8_t const *)&aRight[index]);
    uint8x8_t population = vcnt_u8(veor_u8(in0, in1));
    out += vaddv_u8(population);
  }

  return out;
}

static unsigned char RotatePatternAndReturnData(unsigned char *aImg, int aSpan, Point *apPattern, int16_t aRadIdx)
{
  int16_t cosMant;
  int16_t sinMant;

  // Fixed point rotator
  SinFixedPoint256Levels(aRadIdx, &sinMant);
  CosFixedPoint256Levels(aRadIdx, &cosMant);

  int x      = ((cosMant * apPattern->x) - (apPattern->y * sinMant)) >> gSinCosExp;
  int y      = ((sinMant * apPattern->x) + (apPattern->y * cosMant)) >> gSinCosExp;
  int offset = x + y * aSpan;
#ifdef DEBUG
  printf("Pattern coordinates <%d, %d>\n", apPattern->x, apPattern->y);
  printf("Rotated pattern coordinates <%d, %d>\n", x, y);
  printf("Offset: %x \n", offset);
#endif
  unsigned char out = (unsigned char)(*(aImg + offset));

  return out;
}

static unsigned char CalcDescriptorRotated(unsigned char *imgCenter, int aSpan, Point *apPattern, int16_t aAngleInRadians)
{
  uint8_t partialDescriptor = 0;
  uint8_t tmp0, tmp1;

  tmp0              = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[0], aAngleInRadians);
  tmp1              = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[1], aAngleInRadians);
  partialDescriptor = tmp0 < tmp1;

  tmp0               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[2], aAngleInRadians);
  tmp1               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[3], aAngleInRadians);
  partialDescriptor |= ((tmp0 < tmp1) << 1);

  tmp0               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[4], aAngleInRadians);
  tmp1               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[5], aAngleInRadians);
  partialDescriptor |= ((tmp0 < tmp1) << 2);

  tmp0               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[6], aAngleInRadians);
  tmp1               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[7], aAngleInRadians);
  partialDescriptor |= ((tmp0 < tmp1) << 3);

  tmp0               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[8], aAngleInRadians);
  tmp1               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[9], aAngleInRadians);
  partialDescriptor |= ((tmp0 < tmp1) << 4);

  tmp0               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[10], aAngleInRadians);
  tmp1               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[11], aAngleInRadians);
  partialDescriptor |= ((tmp0 < tmp1) << 5);

  tmp0               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[12], aAngleInRadians);
  tmp1               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[13], aAngleInRadians);
  partialDescriptor |= ((tmp0 < tmp1) << 6);

  tmp0               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[14], aAngleInRadians);
  tmp1               = RotatePatternAndReturnData(imgCenter, aSpan, &apPattern[15], aAngleInRadians);
  partialDescriptor |= ((tmp0 < tmp1) << 7);

  return partialDescriptor;
}

int RBrief(vsdk::SUMat& aGrayImg,
           vector<Keypoint>& aKeypnts,
           vector<DescriptorInfo>& aOutDescr,
           const Point *apPattern,
           int aDescriptorSizeBytes,
           int aWtaK)
{
  uint32_t span  = aGrayImg.step[0];
  int nrOfKeypnt = 0;
  int keypntIdx;
  unsigned char *pCenter;
  char outBuffer[50];

  // Get the number of keypoints
  nrOfKeypnt = aKeypnts.size();

  if(nrOfKeypnt > 0)
  {
    vsdk::SMat grayImgMat    = aGrayImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    unsigned char *tmpImage = (unsigned char *)grayImgMat.data;

    for(keypntIdx = 0; keypntIdx < nrOfKeypnt; keypntIdx++)
    {
      // Debug out data
      memset(outBuffer, 0, 50);
      sprintf(outBuffer, "patch_%d.txt", keypntIdx);

      // Local data
      Keypoint *data = &aKeypnts[keypntIdx];
      Point center   = data->location;
      int16_t angle  = data->angle;

      // Allocate and clear memory, aligned to 64b
      uint64_t *memBuffer            = new uint64_t [aDescriptorSizeBytes / sizeof(uint64_t)];
      unsigned char *descOutBaseAddr = (unsigned char *)memBuffer;
      memset(descOutBaseAddr, 0, aDescriptorSizeBytes);

      switch(aWtaK)
      {
        case 2:
        {
          pCenter = (unsigned char *)(tmpImage + center.x + center.y * span);
          int16_t cosMant;
          int16_t sinMant;

          // Fixed point rotator
          SinFixedPoint256Levels(angle, &sinMant);
          CosFixedPoint256Levels(angle, &cosMant);
#ifdef DEBUG
          DumpPatch(aGrayImg, center.x, center.y, 36, outBuffer);
          printf("**************************************************\n");
          printf("ID: %d\n", keypntIdx);
          printf("Radians: %d\n", angle);
          printf("Keypoint coordinates: <%d, %d> \n", center.x, center.y);
          printf("Center offset: %x \n", center.x + center.y * span);
          printf("Sin Mant: %d\n", sinMant);
          printf("Cos Mant: %d\n", cosMant);
#endif

          for(int byte = 0; byte < aDescriptorSizeBytes; byte++)
          {
            Point *patternForDescriptor = (Point *)&apPattern[16 * byte];
#ifdef DEBUG
            printf("\n\nByte%d\n\n", byte);
#endif
            descOutBaseAddr[byte] = CalcDescriptorRotated(pCenter, span, (Point *)patternForDescriptor, angle);
          }

#ifdef DEBUG
          // Debug purpose only
          PrintArrayHex(descOutBaseAddr, aDescriptorSizeBytes);
#endif

          // The base address of the descriptor is stored in the output vector
          DescriptorInfo newDescriptor;
          newDescriptor.addr     = (uint64_t)descOutBaseAddr;
          newDescriptor.size     = aDescriptorSizeBytes;
          newDescriptor.location = center;
          aOutDescr.push_back(newDescriptor);
        }
        break;

        default:
          assert(aWtaK == 2);

          return EXIT_FAILURE;
      } /* switch aWtaK */
    }
  }
  else
  {
    printf(" rBRIEF did not run !\n");

    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

unsigned int BruteForceMatching(vector<DescriptorInfo>& aInput,
                                vector<DescriptorInfo>& aRef,
                                vector<MatchMetric>& aMatchedData,
                                int aDescriptorSizeInBytes,
                                unsigned int aValidDist,
                                unsigned int aRange)
{
  Point center;
  unsigned int posInRefSet;
  unsigned int posInDataSet;
  unsigned int idx0, idx1;
  unsigned int distance;
  unsigned int minDistFirst;
  unsigned int minDistSecond;
  bool distIsValid;

  assert((aDescriptorSizeInBytes * 8) % 64 == 0);

  for(idx0 = 0; idx0 < aInput.size(); idx0++)
  {
    unsigned char *firstDescriptor = (unsigned char *)aInput[idx0].addr;
    center        = aInput[idx0].location;
    minDistFirst  = UINT_MAX;
    minDistSecond = UINT_MAX;
    posInRefSet   = 0;
    posInDataSet  = 0;
    distIsValid   = false;

    for(idx1 = 0; idx1 < aRef.size(); idx1++)
    {
      unsigned char *secondDescriptor = (unsigned char *)aRef[idx1].addr;
      distance = CalcHammingDistance(firstDescriptor, secondDescriptor, aDescriptorSizeInBytes);

      if(distance < aValidDist)
      {
        distIsValid = true;

        if(distance < minDistFirst)
        {
          minDistFirst = distance;
          posInRefSet  = idx1;
          posInDataSet = idx0;
        }
        else
        if(distance < minDistSecond)
        {
          minDistSecond = distance;
        }
      }
      else
      if(distance < minDistSecond)
      {
        minDistSecond = distance;
      }
    }

    if((distIsValid == true) && ((minDistSecond - minDistFirst) > aRange))
    {
      // Remove the resources allocated to input
      delete ((unsigned char *)(aInput[idx0].addr));

      // Metric for the match
      MatchMetric output;
      output.hammingDistance = minDistFirst;
      output.indexInDataSet  = posInDataSet;
      output.indexInRefSet   = posInRefSet;
      output.locInDataSet    = center;
      aMatchedData.push_back(output);
    }
  }

  return EXIT_SUCCESS;
}
