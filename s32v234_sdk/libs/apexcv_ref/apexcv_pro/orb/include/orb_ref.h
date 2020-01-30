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


#ifndef ORBREF_H
#define ORBREF_H

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <stdint.h>
#include "icp_data.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "arm_neon.h"
#include <assert.h>
#include <vector>
#include <time.h>
#include <limits.h>
#include <sumat.hpp>
#include <oal.h>

using namespace icp;
using namespace std;

extern int sinCosExp;

struct __attribute__((__packed__)) Point
{
  int x;
  int y;
};

struct Keypoint
{
  Point location;
  uint16_t angle;
};

class DescriptorInfo
{
public:
Point location;
uint64_t addr;
int size;

uint8_t getByte(uint8_t location)
{
  volatile uint8_t *pData = (volatile uint8_t *)addr;

  return pData[location];
}
};

struct MatchMetric
{
  // Metric for the match
  Point locInDataSet;
  int hammingDistance;
  int indexInDataSet;
  int indexInRefSet;
};

// Helper functions
void DumpPatch(vsdk::SUMat& aImg,
               int aCenterX,
               int aCenterY,
               int aPatchSize,
               char *apFileName);

void PrintArrayHex(unsigned char *apData,
                   int aSizeInBytes);

int DumpUmat(const char *apFileName,
             vsdk::SUMat& aData);

void SinFixedPoint256Levels(int16_t aX,
                            int16_t *apMagnitude);

void CosFixedPoint256Levels(int16_t aX,
                            int16_t *apMagnitude);

uint8_t Atan2FixedPoint256Levels(int32_t aY,
                                 int32_t aX);

void RemoveKeypointsCloseToBorder(vsdk::SUMat& aOut,
                                  vsdk::SUMat& aIn,
                                  unsigned int aW,
                                  unsigned int aH,
                                  unsigned int aBorderDistance,
                                  unsigned int *apNrOfKeypoints);

void CreateBinBriefDescriptor(vsdk::SUMat& aOut, vector<DescriptorInfo>& aInDescr);
void CreateVectorOfKeypoints(vsdk::SUMat& aIn,
                             int aNrOfKeypoints,
                             vector<Keypoint>& aKeypnts);

int ReadUmatGray(vsdk::SUMat& aImg,
                 char *apFileName);

void DumpAnglesFromVector(vsdk::SUMat& aOut,
                          vector<Keypoint>& aKeypnts);

// Library emulation
int CalcIntensityCentroidAngle(vsdk::SUMat& aInputImg,
                               vector<Keypoint>& aKeypnts,
                               uint16_t aRadius);

int RBrief(vsdk::SUMat& aGrayImg,
           vector<Keypoint>& aKeypnts,
           vector<DescriptorInfo>& aOutDescr,
           const Point *apPattern,
           int aDescriptorSizeBytes,
           int aWtaK);

// Extras
unsigned int CalcHammingDistance(unsigned char *aLeft,
                                 unsigned char *aRight,
                                 int aSizeInBytes);

unsigned int BruteForceMatching(vector<DescriptorInfo>& aInput,
                                vector<DescriptorInfo>& aRef,
                                vector<MatchMetric>& aMatchedData,
                                int aDescriptorSizeInBytes,
                                unsigned int aValidDist,
                                unsigned int aRange);
#endif /* ORBREF_H */
