/******************************************************************************
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2015, Digital Media Professionals.
 * Copyright 2017 NXP
 * All Rights Reserved
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
 *****************************************************************************/
#include <stdint.h>
#include <apexcv_pro_util.h>
#include <lbp_ref.h>

// fixed parameter
static const int RADIUS         = 2;
static const int NEIGHBORS      = 8;
static const int GRID_X         = 8;
static const int GRID_Y         = 8;
static const int HISTOGRAM_SIZE = 256;

static inline unsigned char CalculateLocalBinaryPattern(unsigned char *apSrc, unsigned short aSstr);
static inline int CalculateDistance(unsigned char aSrc0, unsigned char aSrc1);

/*****************************************************************************/
void ExtractDescriptor(unsigned char *apSrc,
                       unsigned short aSstr,
                       unsigned char *apDst,
                       unsigned short aChunkWidth,
                       unsigned short aChunkHeight)
{
  int i, h, w;

  for(i = 0; i < HISTOGRAM_SIZE; i++)
  {
    apDst[i] = 0;
  }

  apSrc += aSstr * RADIUS + RADIUS;

  for(h = 0; h < aChunkHeight; h++)
  {
    for(w = 0; w < aChunkWidth; w++)
    {
      unsigned char lbp = CalculateLocalBinaryPattern(&apSrc[w], aSstr);
      apDst[lbp]++;
    }
    apSrc += aSstr;
  }
}

/*****************************************************************************/
void CompareDescriptor(unsigned char *apSrc0,
                       unsigned char *apSrc1,
                       int *apDst,
                       unsigned short aChunkSize)
{
  for(int w = 0; w < aChunkSize; w++)
  {
    apDst[0] += CalculateDistance(apSrc0[w], apSrc1[w]);
  }
}

/*****************************************************************************/
void AccumulateDistance(int *apSrc,
                        int *apDst)
{
  int32_t APU_VSIZE3;
  int result;

  result = apexcv::RetNumCU(0, APU_VSIZE3);

  if(result)
  {
    return;
  }

  apDst[0] = 0;

  for(int i = 0; i < APU_VSIZE3; i++)
  {
    apDst[0] += apSrc[i];
  }
}

/*****************************************************************************/
static inline unsigned char CalculateLocalBinaryPattern(unsigned char *apSrc,
                                                        unsigned short aSstr)
{
  unsigned char center = apSrc[0];
  short shifted_center = apSrc[0] << 7;

  short t1 = 31 * (short)apSrc[-2 * aSstr + 1] +
             22 * (short)apSrc[-2 * aSstr + 2] +
             44 * (short)apSrc[-aSstr + 1] +
             31 * (short)apSrc[-aSstr + 2];

  short t3 = 22 * (short)apSrc[-2 * aSstr - 2] +
             31 * (short)apSrc[-2 * aSstr - 1] +
             31 * (short)apSrc[-aSstr - 2] + 44 * (short)apSrc[-aSstr - 1];

  short t5 = 31 * (short)apSrc[aSstr - 2] +
             44 * (short)apSrc[aSstr - 1] +
             22 * (short)apSrc[2 * aSstr - 2] +
             31 * (short)apSrc[2 * aSstr - 1];

  short t7 = 44 * (short)apSrc[aSstr + 1] +
             31 * (short)apSrc[aSstr + 2] +
             31 * (short)apSrc[2 * aSstr + 1] +
             22 * (short)apSrc[2 * aSstr + 2];

  unsigned char dst = (unsigned char)(apSrc[2] >= center) |
                      ((unsigned char)(t1 >= shifted_center) << 1) |
                      ((unsigned char)(apSrc[-2 * aSstr] >= center) << 2) |
                      ((unsigned char)(t3 >= shifted_center) << 3) |
                      ((unsigned char)(apSrc[-2] >= center) << 4) |
                      ((unsigned char)(t5 >= shifted_center) << 5) |
                      ((unsigned char)(apSrc[2 * aSstr] >= center) << 6) |
                      ((unsigned char)(t7 >= shifted_center) << 7);

  return dst;
}

/*****************************************************************************/
static inline int CalculateDistance(unsigned char aSrc0, unsigned char aSrc1)
{
  int dist   = 0;
  short diff = (short)aSrc0 - (short)aSrc1;
  short deno = (short)aSrc0 + (short)aSrc1;

  if(deno != 0)
  {
    dist = ((int)diff * (int)diff) / (int)deno;
  }

  return dist;
}
