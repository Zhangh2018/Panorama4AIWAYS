/******************************************************************************
*  (C) Copyright NXP Semiconductos 2017 All right reserved.
*
*  Confidential Information
*
*  All parts of the NXP Semiconductos 2017 Program Source are protected by copyright law
*  and all rights are reserved.
*  This documentation may not, in whole or in part, be copied, photocopied,
*  reproduced, translated, or reduced to any electronic medium or machine
*  readable form without prior consent, in writing, from NXP Semiconductos 2017.
******************************************************************************/
#include "brief_ref.h"

static bool IsInBounds(int aX, int aLeft, int aRight)
{
  if((aX >= aLeft) && (aX <= aRight))
  {
    return true;
  }
  else
  {
    return false;
  }
}

unsigned char Gaussian9x9(unsigned char *apCenter,
                          int aStr)
{
  // Compute the GAUSSSIAN filter
  unsigned char *line0 = &apCenter[-4 * aStr - 4];
  unsigned char *line1 = &apCenter[-3 * aStr - 4];
  unsigned char *line2 = &apCenter[-2 * aStr - 4];
  unsigned char *line3 = &apCenter[-1 * aStr - 4];
  unsigned char *line4 = &apCenter[-4];
  unsigned char *line5 = &apCenter[-1 * aStr - 4];
  unsigned char *line6 = &apCenter[-2 * aStr - 4];
  unsigned char *line7 = &apCenter[-3 * aStr - 4];
  unsigned char *line8 = &apCenter[-4 * aStr - 4];

  // Accmulate lines
  unsigned int out;

  out  = 25 * line0[0] + 60 * line0[1] + 112 * line0[2] + 163 * line0[3] + 185 * line0[4] + 163 * line0[5] + 112 * line0[6] + 60 * line0[7] + 25 * line0[8];
  out += 60 * line1[0] + 144 * line1[1] + 269 * line1[2] + 391 * line1[3] + 443 * line1[4] + 391 * line1[5] + 269 * line1[6] + 144 * line1[7] + 60 * line1[8];
  out += 112 * line2[0] + 269 * line2[1] + 502 * line2[2] + 731 * line2[3] + 828 * line2[4] + 731 * line2[5] + 502 * line2[6] + 269 * line2[7] + 112 * line2[8];
  out += 163 * line3[0] + 391 * line3[1] + 731 * line3[2] + 1064 * line3[3] + 1205 * line3[4] + 1064 * line3[5] + 731 * line3[6] + 391 * line3[7] + 163 * line3[8];
  out += 185 * line4[0] + 443 * line4[1] + 828 * line4[2] + 1205 * line4[3] + 1366 * line4[4] + 1205 * line4[5] + 828 * line4[6] + 443 * line4[7] + 185 * line4[8];
  out += 163 * line5[0] + 391 * line5[1] + 731 * line5[2] + 1064 * line5[3] + 1205 * line5[4] + 1064 * line5[5] + 731 * line5[6] + 391 * line5[7] + 163 * line5[8];
  out += 112 * line6[0] + 269 * line6[1] + 502 * line6[2] + 731 * line6[3] + 828 * line6[4] + 731 * line6[5] + 502 * line6[6] + 269 * line6[7] + 112 * line6[8];
  out += 60 * line7[0] + 144 * line7[1] + 269 * line7[2] + 391 * line7[3] + 443 * line7[4] + 391 * line7[5] + 269 * line7[6] + 144 * line7[7] + 60 * line7[8];
  out += 25 * line8[0] + 60 * line8[1] + 112 * line8[2] + 163 * line8[3] + 185 * line8[4] + 163 * line8[5] + 112 * line8[6] + 60 * line8[7] + 25 * line8[8];

  return out >> 15;
}

unsigned char Box9x9(unsigned char *apCenter,
                     int aStr)
{
  // Compute the BOX filter
  int out;

  unsigned char *line0 = &apCenter[-4 * aStr - 4];
  unsigned char *line1 = &apCenter[-3 * aStr - 4];
  unsigned char *line2 = &apCenter[-2 * aStr - 4];
  unsigned char *line3 = &apCenter[-1 * aStr - 4];
  unsigned char *line4 = &apCenter[-4];
  unsigned char *line5 = &apCenter[-1 * aStr - 4];
  unsigned char *line6 = &apCenter[-2 * aStr - 4];
  unsigned char *line7 = &apCenter[-3 * aStr - 4];
  unsigned char *line8 = &apCenter[-4 * aStr - 4];

  // Accmulate lines
  out  = line0[0] + line0[1] + line0[2] + line0[3] + line0[4] + line0[5] + line0[6] + line0[7] + line0[8];
  out += line1[0] + line1[1] + line1[2] + line1[3] + line1[4] + line1[5] + line1[6] + line1[7] + line1[8];
  out += line2[0] + line2[1] + line2[2] + line2[3] + line2[4] + line2[5] + line2[6] + line2[7] + line2[8];
  out += line3[0] + line3[1] + line3[2] + line3[3] + line3[4] + line3[5] + line3[6] + line3[7] + line3[8];
  out += line4[0] + line4[1] + line4[2] + line4[3] + line4[4] + line4[5] + line4[6] + line4[7] + line4[8];
  out += line5[0] + line5[1] + line5[2] + line5[3] + line5[4] + line5[5] + line5[6] + line5[7] + line5[8];
  out += line6[0] + line6[1] + line6[2] + line6[3] + line6[4] + line6[5] + line6[6] + line6[7] + line6[8];
  out += line7[0] + line7[1] + line7[2] + line7[3] + line7[4] + line7[5] + line7[6] + line7[7] + line7[8];
  out += line8[0] + line8[1] + line8[2] + line8[3] + line8[4] + line8[5] + line8[6] + line8[7] + line8[8];

  return ((out * 405) >> 15);
}

static unsigned char filter(unsigned char *img, int span, int x, int y, int filterType)
{
  int offset             = x - y * span;
  unsigned char *pCenter = &img[offset];

  switch(filterType)
  {
    case 0:
    {
      return pCenter[0];
    }
    break;

    case 1:
    {
      return Box9x9(pCenter, span);
    }
    break;

    case 2:
    {
      return Gaussian9x9(pCenter, span);
    }
    break;
  }

  return 0xFF;
}

static unsigned char calcDescriptor(unsigned char *imgCenter, int span, int filterType, int8_t *pBitPattern)
{
  uint8_t partialDescriptor;
  uint8_t tmp0, tmp1;
  int x0, y0, x1, y1;

  x0 = pBitPattern[0];
  y0 = pBitPattern[1];
  x1 = pBitPattern[2];
  y1 = pBitPattern[3];

  tmp0              = filter(imgCenter, span, x0, y0, filterType);
  tmp1              = filter(imgCenter, span, x1, y1, filterType);
  partialDescriptor = (uint8_t)(tmp0 < tmp1);

  x0                 = pBitPattern[4];
  y0                 = pBitPattern[5];
  x1                 = pBitPattern[6];
  y1                 = pBitPattern[7];
  tmp0               = filter(imgCenter, span, x0, y0, filterType);
  tmp1               = filter(imgCenter, span, x1, y1, filterType);
  partialDescriptor |= ((uint8_t)(tmp0 < tmp1) << 1);

  x0                 = pBitPattern[8];
  y0                 = pBitPattern[9];
  x1                 = pBitPattern[10];
  y1                 = pBitPattern[11];
  tmp0               = filter(imgCenter, span, x0, y0, filterType);
  tmp1               = filter(imgCenter, span, x1, y1, filterType);
  partialDescriptor |= ((uint8_t)(tmp0 < tmp1) << 2);

  x0                 = pBitPattern[12];
  y0                 = pBitPattern[13];
  x1                 = pBitPattern[14];
  y1                 = pBitPattern[15];
  tmp0               = filter(imgCenter, span, x0, y0, filterType);
  tmp1               = filter(imgCenter, span, x1, y1, filterType);
  partialDescriptor |= ((uint8_t)(tmp0 < tmp1) << 3);

  x0                 = pBitPattern[16];
  y0                 = pBitPattern[17];
  x1                 = pBitPattern[18];
  y1                 = pBitPattern[19];
  tmp0               = filter(imgCenter, span, x0, y0, filterType);
  tmp1               = filter(imgCenter, span, x1, y1, filterType);
  partialDescriptor |= ((uint8_t)(tmp0 < tmp1) << 4);

  x0                 = pBitPattern[20];
  y0                 = pBitPattern[21];
  x1                 = pBitPattern[22];
  y1                 = pBitPattern[23];
  tmp0               = filter(imgCenter, span, x0, y0, filterType);
  tmp1               = filter(imgCenter, span, x1, y1, filterType);
  partialDescriptor |= ((uint8_t)(tmp0 < tmp1) << 5);

  x0                 = pBitPattern[24];
  y0                 = pBitPattern[25];
  x1                 = pBitPattern[26];
  y1                 = pBitPattern[27];
  tmp0               = filter(imgCenter, span, x0, y0, filterType);
  tmp1               = filter(imgCenter, span, x1, y1, filterType);
  partialDescriptor |= ((uint8_t)(tmp0 < tmp1) << 6);

  x0                 = pBitPattern[28];
  y0                 = pBitPattern[29];
  x1                 = pBitPattern[30];
  y1                 = pBitPattern[31];
  tmp0               = filter(imgCenter, span, x0, y0, filterType);
  tmp1               = filter(imgCenter, span, x1, y1, filterType);
  partialDescriptor |= ((uint8_t)(tmp0 < tmp1) << 7);

  return partialDescriptor;
}

void brief_ref(vsdk::SUMat &aInputImg,
               std::vector<signed char> &aBitPattern,
               std::vector<unsigned int> &aKeypoints,
               uint8_t *apDescrOut,
               int32_t aFilterType,
               int32_t aBorderSize,
               int32_t aDescrSizeB)
{
  int nrOfKeypoints = (int)aKeypoints.size();
  std::vector<unsigned int> validKeypoints;

  int imgW          = aInputImg.cols;
  int imgH          = aInputImg.rows;
  int span          = aInputImg.step[0];
  uint8_t *pBaseImg = (uint8_t *)aInputImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;

  // Generate bit pattern
  int size            = aDescrSizeB * 8 * 4;
  int8_t *pBitPattern = new int8_t[size];

  for(int idx = 0; idx < size; idx++)
  {
    unsigned int data = aBitPattern[idx];
    pBitPattern[idx] = data;
  }

  // Remove aKeypoints close to border
  for(int kpntId = 0; kpntId < nrOfKeypoints; kpntId++)
  {
    unsigned int data = aKeypoints[kpntId];
    unsigned long x   = (data >> 16) & 0xFFFF;
    unsigned long y   = (data >> 0) & 0xFFFF;

    if(IsInBounds(x, aBorderSize, imgW - aBorderSize) &&
       IsInBounds(y, aBorderSize, imgH - aBorderSize))
    {
      validKeypoints.push_back(data);
    }
  }

  nrOfKeypoints = (int)validKeypoints.size();

  for(int kpntId = 0; kpntId < nrOfKeypoints; kpntId++)
  {
    // Fetch the position
    unsigned int data = validKeypoints[kpntId];
    unsigned int x    = ((data >> 16) & 0xFFFF);
    unsigned int y    = ((data >> 0) & 0xFFFF);

    // Point to the center of the patch
    unsigned int offset    = x + (y * span);
    unsigned char *pCenter = (unsigned char *)&pBaseImg[offset];

    for(int byte = 0; byte < aDescrSizeB; byte++)
    {
      apDescrOut[kpntId * aDescrSizeB + byte] = calcDescriptor(pCenter, span, aFilterType, &pBitPattern[32 * byte]);
    }
  }

  // Free the memory
  delete pBitPattern;
}
