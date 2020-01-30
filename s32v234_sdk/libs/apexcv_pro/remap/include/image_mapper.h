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

#ifndef IMAGEMAPPER_H
#define IMAGEMAPPER_H

#ifdef _OPENCV_TYPES

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "highgui/highgui.hpp"


class ImageMapper
{
public:
   typedef cv::Mat Mat;
   typedef cv::Size Size;

   ImageMapper() {}

   template <class MAPPER>
   ImageMapper(const MAPPER& mapper, const Size& outputImageSize)
   {
      createLUT(mapper, outputImageSize);
   }

   void remap(const Mat & src, Mat & dst) const
   {
       cv::remap(src, dst, mapX, mapY, cv::INTER_LINEAR);
   }

   template <class MAPPER>
   void createLUT(const MAPPER& mapper, const Size& outputImageSize)
   {
      // Create a mapping between the pixels of the two planes

      // Allocate the mapping matrices
      mapX.create(outputImageSize, CV_32FC1);
      mapY.create(outputImageSize, CV_32FC1);

      // Define the mapping matrices
      float *px, *py;
      MAPPER::Point src, dst;

      for (int y=0; y<outputImageSize.height; y++)
      {
         px = mapX.ptr<float>(y),  py = mapY.ptr<float>(y);
         src.y = y;
         for (int x=0; x<outputImageSize.width; x++, px++, py++)
         {
            src.x = x;
            mapper.map(src, dst);
            *px = (float)dst.x;
            *py = (float)dst.y;
         }
      }
   }

   Size getOutputImageSize() const
   {
      return Size(mapX.cols, mapX.rows);
   }

protected:
   Mat mapX, mapY;
};

#else

#include "icp_types.hpp"
#include "icp_remap_protected.hpp"


void mergeRemapLUTs(ICP_Remap_LUT& mergedLUT,
               const ICP_Remap_LUT& lut1, const ICP_Size& imageSize1, ICP_Image_ROI& roi1,
               const ICP_Remap_LUT& lut2, const ICP_Size& imageSize2, ICP_Image_ROI& roi2,
               ICP_Image_ROI& roiFull);

class ImageMapper
{
public:
   ImageMapper(){}

   template <class MAPPER>
   ImageMapper(const MAPPER& mapper, const ICP_Size& inputImageSize, const ICP_Size& outputImageSize, ICP_COLOR_MODEL srcColorModel, ICL_MAP_MODE mapMode, ICL_HW_MAP_MODE hwMode)
   {
      createLUT(mapper, inputImageSize, outputImageSize, srcColorModel, mapMode, hwMode);
   }

   void remap(const ICP_Image& src, ICP_Image &dst, ICP_COLOR_MODEL dstColorModel) const
   {
      SHOW_ERROR_STOP(dstColorModel == ICP_COLOR_GRAYSCALE || dstColorModel == ICP_COLOR_YCbCr422_CbYCrY);    // only grayscale and yuv422 are currently supported
      SHOW_ERROR_STOP( ((int)src.ReturnWidth() == mLut.srcImageSize().width) && ((int)src.ReturnHeight() == mLut.srcImageSize().height) );    // assert the source size is correct

      ICP_Image_Descriptor srcDescriptor;
      src.ReturnDescriptor(&srcDescriptor);
      SHOW_ERROR_STOP( (srcDescriptor.colorModel == mLut.srcColorModel()) );

      //Get the properties of the destination image
      const ICP_Size& dstSize = mLut.dstImageSize();     // The size of the lookup table determines the output size
      ICP_Image_Descriptor dstDesc;
      dstDesc.Configure(dstSize.width, dstSize.height, dstColorModel);
      dst.Create(&dstDesc);       // create the image, if necessary

      const ICP_Image_ROI dstROI(dstSize.width, dstSize.height, 0, 0);
      const ICP_Image_ROI& fullROI = dstROI;
      const ICP_Image_ROI emptyROI(0,0,0,0);
      ICP_Image emptyImage;

      if (mLut.srcColorModel() == ICP_COLOR_YCbCr422_CbYCrY)
      {
         if (dstColorModel == ICP_COLOR_GRAYSCALE)
         {
            ::remap(emptyImage, emptyROI, dst, dstROI, src, fullROI, mLut);
         }
         else if (dstColorModel == ICP_COLOR_YCbCr422_CbYCrY)
         {
            ::remap(dst, dstROI, emptyImage, emptyROI, src, fullROI, mLut);
         }
      }
      else
      {
         ::remap(dst, dstROI, emptyImage, emptyROI, src, fullROI, mLut);
      }
   }

   template <class MAPPER>
   void createLUT(const MAPPER& mapper, const ICP_Size& srcImageSize, const ICP_Size& dstImageSize, ICP_COLOR_MODEL srcColorModel, ICL_MAP_MODE mapMode, ICL_HW_MAP_MODE hwMode)
   {
      MatQ1616 map;
      createMapQ1616(map, mapper, dstImageSize);
      mLut.create(map, srcImageSize, srcColorModel, mapMode, hwMode, true);
   }

   const ICP_Remap_LUT& lut() const {return mLut;}

protected:
   ICP_Remap_LUT mLut;
};

#endif   // _OPENCV_TYPES

#endif   /* IMAGEMAPPER_H */
