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
/*!*********************************************************************************
*  \file gftt_corners_ref.cpp
*  \brief Reference kernels for gftt corners algorithm
***********************************************************************************/
#include <oal.h>
#include "gftt_corners_ref.h"
#include "icp_feature.h"
#include "icp_types.h"
#include "apexcv_pro_util.h"

#define MAX_CORNERS  4096
#define MIN_INT16 -32767
/* Rescale value for the filters */
struct Shift_values
{
   uint8_t covariance;
   uint8_t box;
};
static const Shift_values shifts[]=
{
   {6, 3}, //Covariance, box 3x3
   {6, 4}, //Covariance, box 5x5
   {6, 5} //Covariance, box 7x7
};
static void ref_nms_x(int16_t* src, int sstr,
                      int16_t* dst, int dstr,
                      int iw, int ih, int radius)
{
   for (int i = 0; i < ih; ++i)
   {
      for (int j = 0; j < iw; ++j)
      {
         int16_t val = src[i*sstr + j]-1;
         for (int c = 1; c <= radius / 2; ++c)
         {
            val = val > src[i*sstr + j + c] ? val : src[i*sstr + j + c];
            val = val > src[i*sstr + j - c] ? val : src[i*sstr + j - c];
         }
         dst[i*dstr + j] = val;
      }
   }
}



static void ref_nms_y(int16_t* score, int scstr,
                      int16_t* srcx,  int sstr,
                      int16_t* dst,   int dstr,
                      int iw, int ih, int radius)
{
   for (int i = 0; i < ih; ++i)
   {
      for (int j = 0; j < iw; ++j)
      {
         int16_t val = score[i*scstr + j];
         for (int k = 1; k <= radius / 2; ++k)
         {
            val = val > srcx[(i + k)*sstr + j] ? val : 0;
            val = val > srcx[(i - k)*sstr + j] ? val : 0;
         }
         val = val > srcx[i*sstr + j] ? val : 0;
         dst[i*dstr + j] = val;
      }
   }
}



static void ref_box_filter_generic_y(      int32_t* __restrict dst, int dstr,
                                     const int16_t* __restrict src, int sstr,
                                     int bw, int bh, int /*flt_x*/, int flt_y)
{
   for (int i = 0; i < bh; ++i)
   {
      for (int j = 0; j < bw; ++j)
      {
         int32_t sum = 0;
         for (int r = -(flt_y >> 1); r <= (flt_y >> 1); ++r)
         {
            sum += src[(i + r)*sstr + j];
         }
         dst[i*dstr + j] = sum;
      }
   }
}



static void ref_box_filter_generic_x(      int16_t* __restrict dst, int dstr,
                                     const int32_t* __restrict src, int sstr,
                                     int bw, int bh, int flt_x, int /*flt_y*/, int shift)
{
   for (int i = 0; i < bh; ++i)
   {
      for (int j = 0; j < bw; ++j)
      {
         int32_t sum = 1 << (shift - 1);
         for (int c = -(flt_x >> 1); c <= (flt_x >> 1); ++c)
         {
            sum += src[i*sstr + j + c];
         }
         dst[i*dstr + j] = (int16_t)(sum >> shift);
      }
   }
}

static inline short _sqrt_int(unsigned int val)
{
   unsigned int rem = 0;
   unsigned int root = 0;
   for (int i = 0; i < 16; i++)
   {
      root <<= 1;
      rem = ((rem << 2) + (val >> 30));
      val <<= 2;
      if (root < rem)
      {
         root++;
         rem -= root;
         root++;
      }
   }

   unsigned int buf0 = root >> 1;
   short buf1 = (short)buf0;
   if ((unsigned int)buf1 != buf0)
   {
      printf("overflow _sqrt_int\n");
   }
   return buf1;
}

static inline int16_t _get_min_eigen_val(short xx, short yy, short xy)
{
   short a = xx >> 1;
   short b = xy;
   short c = yy >> 1;
   short ac = a - c;
   return (a + c) - _sqrt_int((unsigned int)(ac*ac) + (unsigned int)(b*b));
}

static void gftt_calc_score(  int16_t* axx, int16_t* axy, int16_t* ayy, int16_t* dst, 
                              int16_t* highest_strength, int iw, int ih)
{
   for (int i = 0; i < ih; ++i)
   {
      for (int j = 0; j < iw; ++j)
      {
         int16_t score = _get_min_eigen_val(axx[i*iw+j], ayy[i*iw+j], axy[i*iw+j]);
         //score = score > 32767 ? 32767 : score;
         //score = score < -32767 ? -32767 : score;
         dst[i*iw + j] = (int16_t)score;
         highest_strength[0] = highest_strength[0] > dst[i*iw + j] ? highest_strength[0] : dst[i*iw + j];
      }
   }
}
static void harris_calc_score(int16_t* axx, int16_t* axy, int16_t* ayy,
                       int16_t* dst,
                       int k, int iw, int ih)
{
   for (int i = 0; i < ih; ++i)
   {
      for (int j = 0; j < iw; ++j)
      {
         int32_t xx_yy = axx[i*iw + j] * ayy[i*iw + j];
         int32_t xy_xy = axy[i*iw + j] * axy[i*iw + j];
         int32_t det = xx_yy - xy_xy;
         int32_t sum = axx[i*iw + j] + ayy[i*iw + j];
         int32_t trace = sum*sum;
         trace += 128;
         trace >>= 8;
         trace *= k;
         int32_t score = det - trace;
         score = score > 32767 ? 32767 : score;
         score = score < -32767 ? -32767 : score;
         dst[i*iw + j] = (int16_t)score;
      }
   }
}
#define LT(x, y)((x)>(y))

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

static inline void swap(short *ptr1, short *ptr2)
{
   int tmp;
   tmp = *ptr1;
   *ptr1 = *ptr2;
   *ptr2 = tmp;
}

struct
{
  short *lb;
  short *ub;
}stack[48];

static int sort(short* vec, short* idx, int total)
{
   int isort_thresh = 7;
   int sp = 0;
   int lRetVal = 0;
   short* arr;

   if( total <= 1 )
   {
     lRetVal = -1;
     goto EXIT;
   }

   arr = &idx[0];
   stack[0].lb = arr;
   stack[0].ub = arr + (total - 1);

   while( sp >= 0 )
   {
      short* left = stack[sp].lb;
      short* right = stack[sp--].ub;

      for(;;)
      {
         int i, n = (int)(right - left) + 1, m;
         short* ptr;
         short* ptr2;

         if( n <= isort_thresh )
         {
            insert_sort:
            for( ptr = left + 1; ptr <= right; ptr++ )
            {
               for( ptr2 = ptr; ptr2 > left && LT(vec[ptr2[0]], vec[ptr2[-1]]); ptr2--)
               {
                  swap(&ptr2[0], &ptr2[-1] );
               }
            }
            break;
         }
         else
         {
            short* left0;
            short* left1;
            short* right0;
            short* right1;
            short* pivot;
            short* a;
            short* b;
            short* c;
            int swap_cnt = 0;

            left0 = left;
            right0 = right;
            pivot = left + (n/2);

            if( n > 40 )
            {
               int d = n / 8;
               a = left, b = left + d, c = left + 2*d;
               vec[*a] = vec[*a];
               vec[*b] = vec[*b];
               vec[*c] = vec[*c];
               left = LT(vec[*a], vec[*b]) ? (LT(vec[*b], vec[*c]) ? b : (LT(vec[*a], vec[*c]) ? c : a))
                                           : (LT(vec[*c], vec[*b]) ? b : (LT(vec[*a], vec[*c]) ? a : c));

               a = pivot - d, b = pivot, c = pivot + d;
               pivot = LT(vec[*a], vec[*b]) ? (LT(vec[*b], vec[*c]) ? b : (LT(vec[*a], vec[*c]) ? c : a))
                                            : (LT(vec[*c], vec[*b]) ? b : (LT(vec[*a], vec[*c]) ? a : c));

               a = right - 2*d, b = right - d, c = right;
               right = LT(vec[*a], vec[*b]) ? (LT(vec[*b], vec[*c]) ? b : (LT(vec[*a], vec[*c]) ? c : a))
                                            : (LT(vec[*c], vec[*b]) ? b : (LT(vec[*a], vec[*c]) ? a : c));
            }

            a = left, b = pivot, c = right;
            pivot = LT(vec[*a], vec[*b]) ? (LT(vec[*b], vec[*c]) ? b : (LT(vec[*a], vec[*c]) ? c : a))
                                         : (LT(vec[*c], vec[*b]) ? b : (LT(vec[*a], vec[*c]) ? a : c));

            if( pivot != left0 )
            {
               swap(pivot, left0);
               pivot = left0;
            }

            left = left1 = left0 + 1;
            right = right1 = right0;

            for(;;)
            {
               while( left <= right && !LT(vec[*pivot], vec[*left]) )
               {
                  if( !LT(vec[*left], vec[*pivot]) )
                  {
                     if( left > left1 )
                     {
                        swap(left1, left );
                     }
                     swap_cnt = 1;
                     left1++;
                  }
                  left++;
               }

               while( left <= right && !LT(vec[*right], vec[*pivot]) )
               {
                  if( !LT(vec[*pivot], vec[*right]) )
                  {
                     if( right < right1 )
                     {
                        swap(right1, right );
                     }
                     swap_cnt = 1;
                     right1--;
                  }
                  right--;
               }

               if( left > right )
                  break;

               swap(left, right );
               swap_cnt = 1;
               left++;
               right--;
            }

            if( swap_cnt == 0 )
            {
               left = left0, right = right0;
               goto insert_sort;
            }

            n = MIN( (left1 - left0), (left - left1) );
            for( i = 0; i < n; i++ )
            {
               swap(&left0[i], &left[i-n] );
            }

            n = MIN( (right0 - right1), (right1 - right) );
            for( i = 0; i < n; i++ )
            {
               swap(&left[i], &right0[i-n+1] );
            }

            n = (left - left1);
            m = (right1 - right);
            if( n > 1 )
            {
               if( m > 1 )
               {
                  // check sp index out of range
                  if(sp>=47)
                  {
                    printf("Error: Stack overflow\n");
                    lRetVal = -1;
                    goto EXIT;
                  }
                  if( n > m )
                  {
                     stack[++sp].lb = left0;
                     stack[sp].ub = left0 + n - 1;
                     left = right0 - m + 1, right = right0;
                  }
                  else
                  {
                     stack[++sp].lb = right0 - m + 1;
                     stack[sp].ub = right0;
                     left = left0, right = left0 + n - 1;
                  }
               }
               else
               {
                  left = left0, right = left0 + n - 1;
               }
            }
            else if( m > 1 )
            {
               left = right0 - m + 1, right = right0;
            }
            else
              break;
         }
      }
   }
   EXIT:
   return lRetVal;
}
static int sortingRef(icp::DataDescriptor& src             /*!< Source corner score buffer. */,
                      icp::FeatureDescriptor& corners      /*!< Destination corner feature description buffer. */,
                      int highest_strength                 /*!< Highest score value .*/,
                      int qualityLevel                     /*!< Quality Level */,
                      int maxCorners                      /*!< Maximum number of corners to return. */,
                      int useHarrisDetector,
                      int harrisThreshold)
{
    int lRetVal = 0;
    int width            = src.GetWidth();
    int height           = src.GetHeight();
    int sstr             = src.GetSpan()>>1;
    short* pSrc          = (short*)src.GetDataPtr();
    int cnt              = 0;

    int16_t* stren       = (int16_t*)malloc(sizeof(int16_t)*width*height);
    int16_t* coord_list  = (int16_t*)malloc(sizeof(int16_t)*width*height*2);
    int16_t* idx         = (int16_t*)malloc(sizeof(int16_t)*width*height);

    //Threshold
    int threshold;
    if (!useHarrisDetector)
      threshold = (((highest_strength*qualityLevel)>>7)+1)>>1;
    else
      threshold = harrisThreshold;
    // Enumerate the corner and stren list
    for (int16_t i = 0; i < height; ++i)
    {
      for (int16_t j = 0; j < width; ++j)
      {
        if (pSrc[i*sstr+j] >= threshold)
        {
          stren[cnt] = pSrc[i*sstr+j]; //strength
          coord_list[cnt*2 + 0] = j; //width
          coord_list[cnt*2 + 1] = i; //height
          idx[cnt] = cnt;
          cnt++;
        }
        if (cnt > MAX_CORNERS) break;
      }
      if (cnt > MAX_CORNERS) break;
   }
  
    int ncorners = 0;
    //sort corners
    lRetVal = sort(stren, idx, cnt);
    if(lRetVal != 0)
    {
      printf("Error: sort() function execute failed!!!\n");
      goto EXIT;
    }
    for (int i = 0; i < cnt; ++i) {
      if (ncorners >= maxCorners) break;
      int x = coord_list[idx[i] * 2 + 0];
      int y = coord_list[idx[i] * 2 + 1];
      short tstren = stren[idx[i]];
      if (tstren < 0) continue;
      //compute distance with previous corners and compare with min distance
	  corners.Add(x, y, tstren);
	  ncorners++;
    }

    EXIT:
    free(stren);
    free(coord_list);
    free(idx);
   return lRetVal;
}



template<typename T>
static void create_padded_image(T *src                     /*!< Source memory buffer. */,
                                T *dst                     /*!< Destination memory buffer. */,
                                int32_t width              /*!< Source image width. */,
                                int32_t height             /*!< Source image height. */,
                                int32_t top                /*!< Padding size in +y direction. */,
                                int32_t bottom             /*!< Padding size in -y direction. */,
                                int32_t left               /*!< Padding size in -x direction. */,
                                int32_t right)             /*!< Padding size in +x direction. */
{
   int sstr = width;
   int dstr = width + left + right;
   T *buf = dst + top*dstr;

   for (int h = 0; h < height; h++)
   {
      int w;
      for (w = 0; w < left; w++)     // left size padding
      {
         buf[h*dstr + w] = src[h*sstr];
      }
      for (; w < left + width; w++)  // copy image
      {
         buf[h*dstr + w] = src[h*sstr + (w - left)];
      }
      for (; w < dstr; w++)         // right size padding
      {
         buf[h*dstr + w] = src[h*sstr + width - 1];
      }
   }

   for (int w = 0; w < dstr; w++)
   {
	  // copy top
      for (int h = 0; h < top; h++)
      {
         dst[h*dstr + w] = buf[w];
      }
      // copy bottom
      for (int h = top + height; h < top + height + bottom; h++)
      {
         dst[h*dstr + w] = buf[(height - 1)*dstr + w];
      }
   }
}



static void sobel_dx_filter(uint8_t* src, int sstr,
                      int16_t* dst, int dstr,
                      int iw, int ih)
{
   for (int i = 0; i < ih; ++i)
   {
      for (int j = 0; j < iw; ++j)
      {
        dst[j] = src[-sstr + j + 1] - src[-sstr + j - 1] + 2 * src[j + 1] - 2 * src[j - 1] + src[sstr + j + 1] - src[sstr + j - 1];
      }
      dst += dstr;
      src += sstr;
   }
}



static void sobel_dy_filter(uint8_t* src, int sstr,
                      int16_t* dst, int dstr,
                      int iw, int ih)
{
   for (int i = 0; i < ih; ++i)
   {
      for (int j = 0; j < iw; ++j)
      {
        dst[j] = (src[sstr + j - 1] + 2 * src[sstr + j] + src[sstr + j + 1]) - (src[-sstr + j - 1] + 2 * src[-sstr + j] + src[-sstr + j + 1]);
      }
      dst += dstr;
      src += sstr;
   }
}



static void covariance(int16_t* src_dx,  int16_t* src_dy,
                int16_t* dst_dxx, int16_t* dst_dxy, int16_t* dst_dyy,
                int iw, int ih, int shift)
{
   for (int i = 0; i < ih; ++i)
   {
      for (int j = 0; j < iw; ++j)
      {
         dst_dxx[j] = (int16_t)((((int32_t)src_dx[j] * (int32_t)src_dx[j]) + (1 << (shift - 1))) >> shift);
         dst_dxy[j] = (int16_t)((((int32_t)src_dx[j] * (int32_t)src_dy[j]) + (1 << (shift - 1))) >> shift);
         dst_dyy[j] = (int16_t)((((int32_t)src_dy[j] * (int32_t)src_dy[j]) + (1 << (shift - 1))) >> shift);
      }
      dst_dxx += iw;
      dst_dxy += iw;
      dst_dyy += iw;
      src_dx += iw;
      src_dy += iw;
   }
}

namespace ref{



GFTTCorners::GFTTCorners():
  mCornerImagePtr(0)
{}

GFTTCorners::~GFTTCorners()
{
  if (mCornerImagePtr)  OAL_MemoryFree(mCornerImagePtr);
  mCornerImagePtr = 0;
}
icp::DataDescriptor GFTTCorners::retCornerImage()
{
  return mCornerImage;
}
int GFTTCorners::process(  vsdk::SMat rSrcDd,
                           icp::FeatureDescriptor& rFeatureDescriptor,
                           const int cMinDistance,
                           const int cQualityLevel,
                           const int cBoxSize,
                           const int cNmsRadius,
                           const int cMaxTotalCorners,
                           const int cUseHarrisDetector,
                           const int cHarrisK,
                           const int cHarrisThreshold)
{
   int intResult = -1;

   unsigned char* src = (unsigned char*)rSrcDd.data;
   int width          = rSrcDd.cols;
   int height         = rSrcDd.rows;
   int minDistance    = cMinDistance;
   int qualityLevel   = cQualityLevel;
   int box_size       = cBoxSize;
   int nms_radius     = cNmsRadius;
   int max_corners    = cMaxTotalCorners;
   int useHarrisDetector = cUseHarrisDetector;
   int harrisK = cHarrisK;
   int harrisThreshold = cHarrisThreshold;

   icp::FeatureDescriptor lCornersRef;

   int pad_x = box_size >> 1;
   int pad_y = box_size >> 1;

   //figure out shift values for the filters
   int8_t covariance_shift;
   int8_t box_shift;
   switch (box_size)
   {
      case 3: //box 3x3
        covariance_shift = shifts[0].covariance;
        box_shift = shifts[0].box;
        break;
      case 5: //box 5x5
        covariance_shift = shifts[1].covariance;
        box_shift = shifts[1].box;
        break;
      case 7: //box 7x7
        covariance_shift = shifts[2].covariance;
        box_shift = shifts[2].box;
        break;
      default:
        covariance_shift = shifts[0].covariance;
        box_shift = shifts[0].box;
        break;
   }

    uint8_t* src_pad = (uint8_t*)malloc(sizeof(uint8_t)*(width+2)*(height+2));

    int16_t* dst_dx = (int16_t*)malloc(sizeof(int16_t)*width*height);
    int16_t* dst_dy = (int16_t*)malloc(sizeof(int16_t)*width*height);

    int16_t* dst_dxx = (int16_t*)malloc(sizeof(int16_t)*width*height);
    int16_t* dst_dxy = (int16_t*)malloc(sizeof(int16_t)*width*height);
    int16_t* dst_dyy = (int16_t*)malloc(sizeof(int16_t)*width*height);

    int16_t* dxx = (int16_t*)malloc(sizeof(int16_t)*width*(height+2*pad_y));
    int16_t* dxy = (int16_t*)malloc(sizeof(int16_t)*width*(height+2*pad_y));
    int16_t* dyy = (int16_t*)malloc(sizeof(int16_t)*width*(height+2*pad_y));

    int32_t* dst_bxx_y = (int32_t*)malloc(sizeof(int32_t)*width*height);
    int32_t* dst_bxy_y = (int32_t*)malloc(sizeof(int32_t)*width*height);
    int32_t* dst_byy_y = (int32_t*)malloc(sizeof(int32_t)*width*height);

    int32_t* bxx_y = (int32_t*)malloc(sizeof(int32_t)*(width+2*pad_x)*height);
    int32_t* bxy_y = (int32_t*)malloc(sizeof(int32_t)*(width+2*pad_x)*height);
    int32_t* byy_y = (int32_t*)malloc(sizeof(int32_t)*(width+2*pad_x)*height);

    int16_t* dst_bxx = (int16_t*)malloc(sizeof(int16_t)*width*height);
    int16_t* dst_bxy = (int16_t*)malloc(sizeof(int16_t)*width*height);
    int16_t* dst_byy = (int16_t*)malloc(sizeof(int16_t)*width*height);

    int16_t* dst_score = (int16_t*)malloc(sizeof(int16_t)*width*height);

    int16_t* score = (int16_t*)malloc(sizeof(int16_t)*(width+nms_radius-1)*height);  

    int16_t* dst_nms_x = (int16_t*)malloc(sizeof(int16_t)*width*height);

    int16_t* d_nms_x = (int16_t*)malloc(sizeof(int16_t)*width*(height+nms_radius-1));

    if (mCornerImagePtr)
    {
      OAL_MemoryFree(mCornerImagePtr);
      mCornerImagePtr = 0;
    }

    mCornerImagePtr = OAL_MemoryAllocFlag(sizeof(int16_t)*width*height,
                                          OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
    mCornerImage.InitManual(width, height,
                           OAL_MemoryReturnAddress(mCornerImagePtr, ACCESS_CH_WB), 
                           OAL_MemoryReturnAddress(mCornerImagePtr, ACCESS_PHY),
                           icp::DATATYPE_16S, 1, 1, 0, 0, 1, icp::ALLOC_CONTIGUOUS);

   void *lpRefOal        = OAL_MemoryReturnAddress(mCornerImagePtr, ACCESS_CH_WB);

   if (src_pad   && dst_dx    && dst_dy    && dst_dxx   &&
       dst_dxy   && dst_dyy   && dxx      && dxy      &&
       dyy      && dst_bxx_y  && dst_bxy_y  && dst_byy_y  &&
       bxx_y     && bxy_y     && byy_y     && dst_bxx   &&
       dst_bxy   && dst_byy   && dst_score && score    &&
       dst_nms_x  && d_nms_x    && lpRefOal)
   {
      int16_t *ref       = (int16_t*)lpRefOal;

      // Do the padding
      create_padded_image<uint8_t>(src, src_pad, width, height, 1, 1, 1, 1);

      int offset = width+3;

      sobel_dx_filter(&src_pad[offset], width + 2, dst_dx, width, width, height);
      sobel_dy_filter(&src_pad[offset], width + 2, dst_dy, width, width, height);
      covariance(dst_dx, dst_dy, dst_dxx, dst_dxy, dst_dyy, width, height, covariance_shift);

      create_padded_image<int16_t>(dst_dxx, dxx, width, height, pad_y, pad_y, 0, 0);
      create_padded_image<int16_t>(dst_dxy, dxy, width, height, pad_y, pad_y, 0, 0);
      create_padded_image<int16_t>(dst_dyy, dyy, width, height, pad_y, pad_y, 0, 0);

      ref_box_filter_generic_y(dst_bxx_y, width, dxx + pad_y*width, width, width, height, box_size, box_size);
      ref_box_filter_generic_y(dst_bxy_y, width, dxy + pad_y*width, width, width, height, box_size, box_size);
      ref_box_filter_generic_y(dst_byy_y, width, dyy + pad_y*width, width, width, height, box_size, box_size);

      create_padded_image<int32_t>(dst_bxx_y, bxx_y, width, height, 0, 0, pad_x, pad_x);
      create_padded_image<int32_t>(dst_bxy_y, bxy_y, width, height, 0, 0, pad_x, pad_x);
      create_padded_image<int32_t>(dst_byy_y, byy_y, width, height, 0, 0, pad_x, pad_x);

      ref_box_filter_generic_x(dst_bxx, width, bxx_y + pad_x, width + 2 * pad_x, width, height, box_size, box_size, box_shift);
      ref_box_filter_generic_x(dst_bxy, width, bxy_y + pad_x, width + 2 * pad_x, width, height, box_size, box_size, box_shift);
      ref_box_filter_generic_x(dst_byy, width, byy_y + pad_x, width + 2 * pad_x, width, height, box_size, box_size, box_shift);

      int16_t highest_strength = MIN_INT16;
      if (!useHarrisDetector)
         gftt_calc_score(dst_bxx, dst_bxy, dst_byy, dst_score, &highest_strength, width, height);
      else
         harris_calc_score(dst_bxx, dst_bxy, dst_byy, dst_score, harrisK, width, height);

      memset(score, 0, sizeof(uint16_t)*height*(width+nms_radius-1));

      for (int i = 0; i < height; ++i)
      {
         memcpy(score + i*(width + nms_radius - 1) + (nms_radius >> 1), dst_score + i*width, sizeof(int16_t)*width);
      }

      ref_nms_x(score + (nms_radius >> 1),
                width + nms_radius - 1,
                dst_nms_x,
                width,
                width,
                height,
                nms_radius);

      memset(d_nms_x, 0, sizeof(uint16_t)*(height+nms_radius-1)*width);

      for (int i = 0; i < height; ++i)
      {
         memcpy(d_nms_x + (i + (nms_radius >> 1))*width, dst_nms_x + i*width, sizeof(int16_t)*width);
      }

      ref_nms_y(score + (nms_radius >> 1), width + nms_radius - 1, d_nms_x + (nms_radius >> 1)*width, width, ref, width, width, height, nms_radius);

      sortingRef(mCornerImage,
                 rFeatureDescriptor,
                 highest_strength,
                 qualityLevel,
                 max_corners,
                 useHarrisDetector,
                 harrisThreshold);

      intResult = 0;
   }


   if (src_pad)     free(src_pad);
   if (dst_dx)      free(dst_dx);
   if (dst_dy)      free(dst_dy);
   if (dst_dxx)     free(dst_dxx);
   if (dst_dxy)     free(dst_dxy);
   if (dst_dyy)     free(dst_dyy);
   if (dxx)         free(dxx);
   if (dxy)         free(dxy);
   if (dyy)         free(dyy);
   if (dst_bxx_y)   free(dst_bxx_y);
   if (dst_bxy_y)   free(dst_bxy_y);
   if (dst_byy_y)   free(dst_byy_y);
   if (bxx_y)       free(bxx_y);
   if (bxy_y)       free(bxy_y);
   if (byy_y)       free(byy_y);
   if (dst_bxx)     free(dst_bxx);
   if (dst_bxy)     free(dst_bxy);
   if (dst_byy)     free(dst_byy);
   if (dst_score)   free(dst_score);
   if (score)       free(score);
   if (dst_nms_x)   free(dst_nms_x);
   if (d_nms_x)     free(d_nms_x);
   return intResult;
}

}
