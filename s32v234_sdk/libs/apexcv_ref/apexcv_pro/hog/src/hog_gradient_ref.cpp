/**************************************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright 2013-2017 NXP
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
**************************************************************************************************/

#include "hog_gradient_ref.h"
#include "stdlib.h"   // abs()

#ifndef INT16_MAX
#define INT16_MAX  0x7fff
#endif
#ifndef UINT16_MAX
#define UINT16_MAX 0xffff
#endif

/******** HOG Gradient C reference code *********/

uchar getAngleBin(int16_t x, int16_t y)
{
  //typedef double fix;
  const int16_t s_num = 15;
  const int16_t s_den = 35;

  // rotate vector forward by PI/16
  const int16_t c_den = 5;

  HOG_ASSERT((int16_t)(c_den * x) == (int32_t)c_den * (int32_t)x);
  int16_t tmp = (int16_t)(c_den * x) - y;
  y *= c_den;
  y += x;
  x = tmp;

  y >>= 1;
  x >>= 1;

  int16_t u, v;

  if(x >= 0)
  {
    u = x;
  }
  else
  {
    u = -x;
    y = -y;
  }

  uchar id;

  if(y >= 0)
  {
    v = y;
    id = 0;
  }
  else
  {
    v = u;
    u = -y;
    id = 4;
  }

  if(v >= u)
  {
    id += 2;
    tmp = -v;
    v = -u;
    u = tmp;
  }

  HOG_ASSERT(((int32_t)s_num * (int32_t)u) == ((int32_t)s_num * (int32_t)u));
  HOG_ASSERT(((int32_t)s_den * (int32_t)v) == ((int32_t)s_den * (int32_t)v));

  if((int16_t)(s_num * u) <= (int16_t)(s_den * v))
  {
    id += 1;
  }

  // Zero cases
  if(x == 0)
  {
    id = 4;
  }

  if(y == 0)
  {
    id = 0;
  }

  return id;
}

#ifdef __cplusplus
void HOG_allocateGradientCache(HOG_GradientCache& cache, const Size& gradSize)
{
  const int gradientBufferSize = AREA(gradSize);
  MATRIX_RESERVE(cache.magnitude, gradientBufferSize, uchar);
  MATRIX_RESERVE(cache.qangle, gradientBufferSize, uchar);
}

void HOG_allocateGradientCache(HOG_GradientCache& cache, const Rect& imageROI)
{
  Size gradSize;
  gradSize.width = imageROI.width;
  gradSize.height = imageROI.height;
  HOG_allocateGradientCache(cache, gradSize);
}

void HOG_deleteGradientCache(HOG_GradientCache& cache)
{
  MATRIX_DELETE(cache.magnitude);
  MATRIX_DELETE(cache.qangle);
}

void HOG_computeGradientC1(HOG_GradientCache& cache,
                           const Mat8u& img,
                           const Rect& imgROI)
{
  // Compute the interpolated gradient and quantized angle bin IDs
  // for a grayscale (single channel) image

  // Define useful constants
  const int lineStep = img.step - imgROI.width;

  // Define pointers
  const uchar* imgPtr = img.data + imgROI.y * img.step + imgROI.x;
  uchar*       gradPtr = cache.magnitude.data;
  uchar*       angleBinPtr = cache.qangle.data;

  for(int y = 0; y < cache.magnitude.rows; ++y, imgPtr += lineStep)
  {
    for(int x = 0; x < cache.magnitude.cols; ++x, ++imgPtr, ++gradPtr, ++angleBinPtr)
    {
      // Get the derivatives
      int16_t dx = imgPtr[1] - imgPtr[-1];
      int16_t dy = imgPtr[img.step] - imgPtr[-img.step];

      // Get the angle bin
      *angleBinPtr = getAngleBin(dx, dy);

      // Compute the L1 norm
      int16_t L1_norm = (int16_t)(abs(dx) + abs(dy));
      *gradPtr = L1_norm > 255 ? (uchar)255 : (uchar)L1_norm;
    }
  }
}

void HOG_computeGradientC3(HOG_GradientCache& cache,
                           const Mat8u& img,
                           const Rect& imgROI)
{
  // Compute the interpolated gradient and quantized angle bin IDs
  // for a RGB or LAB (3 channel) image

  // Define useful constants
  const int lineStep = img.step - imgROI.width * 3;

  // Define pointers
  const uchar* imgPtr = img.data + imgROI.y * img.step + imgROI.x * 3;
  uchar   *   gradPtr = cache.magnitude.data;
  uchar*   angleBinPtr = cache.qangle.data;

  // Up and down pixel shifts for channels 2 and 3 (to avoid adding for each pixel)
  const int up2 = 1 + img.step, down2 = 1 - img.step;
  const int up3 = up2 + 1, down3 = down2 + 1;

  for(int y = 0; y < cache.magnitude.rows; ++y, imgPtr += lineStep)
  {
    for(int x = 0; x < cache.magnitude.cols; ++x, imgPtr += 3, ++gradPtr, ++angleBinPtr)
    {
      // Use the channel with the largest derivative magnitude

      // Channel 3
      int16_t dx = imgPtr[5] - imgPtr[-1];
      uint16_t mag = (uint16_t)abs(dx);
      mag *= mag;

      int16_t dy = imgPtr[up3] - imgPtr[down3];
      uint16_t dy2 = (uint16_t)abs(dy);
      dy2 *= dy2;

      if(mag > UINT16_MAX - dy2)
      {
        mag = UINT16_MAX;
      }
      else
      {
        mag += dy2;
      }

      // Channel 2
      int16_t dx_ = imgPtr[4] - imgPtr[-2];
      uint16_t mag_ = (uint16_t)abs(dx_);
      mag_ *= mag_;

      int16_t dy_ = imgPtr[up2] - imgPtr[down2];
      dy2 = (uint16_t)abs(dy_);
      dy2 *= dy2;

      if(mag_ > UINT16_MAX - dy2)
      {
        mag_ = UINT16_MAX;
      }
      else
      {
        mag_ += dy2;
      }

      if(mag < mag_)
      {
        dx = dx_;
        dy = dy_;
        mag = mag_;
      }

      // Channel 1
      dx_ = imgPtr[3] - imgPtr[-3];
      mag_ = (uint16_t)abs(dx_);
      mag_ *= mag_;

      dy_ = imgPtr[img.step] - imgPtr[-img.step];
      dy2 = (uint16_t)abs(dy_);
      dy2 *= dy2;

      if(mag_ > UINT16_MAX - dy2)
      {
        mag_ = UINT16_MAX;
      }
      else
      {
        mag_ += dy2;
      }

      if(mag < mag_)
      {
        dx = dx_;
        dy = dy_;
        mag = mag_;
      }

      // Get the angle bin
      *angleBinPtr = getAngleBin(dx, dy);

      // Compute the gradient magnitude
      int16_t L1_norm = (int16_t)(abs(dx) + abs(dy));
      *gradPtr = L1_norm > 255 ? (uchar)255 : (uchar)L1_norm;
    }
  }
}

#endif

void HOG_computeGradient(HOG_GradientCache& cache,
                         const Mat8u& img,
                         const Rect& imgROI)
{
  // Assert all necessary conditions

  // Assert the buffers are large enough
  Size gradSize;
  gradSize.width = imgROI.width;
  gradSize.height = imgROI.height;

  HOG_ASSERT(cache.magnitude.data && static_cast<unsigned int>(cache.magnitude.capacity) >= AREA(gradSize));
  HOG_ASSERT(cache.qangle.data && static_cast<unsigned int>(cache.qangle.capacity) >= AREA(gradSize));

  // Assert the image ROI is within range
  // NOTE: Because we do not use the ymap and xmap (used in OpenCV implementation to deal with padding and image boundaries),
  //       we must have a ROI that leaves at least one pixel of padding on all sides.
  HOG_ASSERT((imgROI.x > 0) && (imgROI.y > 0) &&
             (imgROI.width > 0) && (imgROI.height > 0) &&
             (imgROI.x + imgROI.width < img.cols) &&
             (imgROI.y + imgROI.height < img.rows));

  // Initialize the outputs (2-channel gradient image and quantized angles)
  MATRIX_RESIZE(cache.magnitude, gradSize, 1);
  MATRIX_RESIZE(cache.qangle, gradSize, 1);

  if(1 == img.channels)
  {
    // Grayscale image
    HOG_computeGradientC1(cache, img, imgROI);
  }
  else if(3 == img.channels)
  {
    // RGB or LAB colour image
    HOG_computeGradientC3(cache, img, imgROI);
  }
  else
  {
    // Not supported
    HOG_ASSERT(false);
  }
}
