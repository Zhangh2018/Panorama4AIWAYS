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

#ifndef DEWARPTOOLS_H
#define DEWARPTOOLS_H

#include "camera_mapper.h"

#ifdef _OPENCV_TYPES
#include "core/core.hpp"
#define Rect_ cv::Rect_
using cv::Size;
using cv::Point_;
#else
#include "icp_types.h"
#define Rect_ ICP_Rectangle_
#define Size ICP_Size
#define Point_ ICP_Point2
#endif

template <class REAL, class DISTORTMAPPER>
class DewarpROI_
{
public:
   typedef Rect_<REAL> Roi;
   typedef CameraParams_<REAL> CameraParams;
   typedef typename DISTORTMAPPER::Params DistortParams;
   typedef Point_<REAL> Point2r;
   typedef DewarpMapper_<REAL, DISTORTMAPPER> DewarpMapper;
   
   DewarpROI_()
   {
   }

   DewarpROI_(const CameraParams& cameraParams, const DistortParams& distortParams, const Size& inputImageSize)
   {
      init(cameraParams, distortParams, inputImageSize);
   }

   template <class REAL2, class DISTORTMAPPER2>
   DewarpROI_(const DewarpROI_<REAL2, DISTORTMAPPER2>& other) : mInner(other.inner()), mOuter(other.outer())
   {
   }

   void init(const CameraParams& cameraParams, const DistortParams& distortParams, const Size& inputImageSize)
   {
      const int sampleStep = 4;     // The number of pixels that are skipped when sampling the image edge

      const int horizontalSampleCount = inputImageSize.width  / sampleStep;
      const int verticalSampleCount   = inputImageSize.height / sampleStep;

#ifdef _WIN32
      Point2r* const left   = new Point2r[verticalSampleCount];
      Point2r* const right  = new Point2r[verticalSampleCount];
      Point2r* const top    = new Point2r[horizontalSampleCount];
      Point2r* const bottom = new Point2r[horizontalSampleCount];
#else
      Point2r* const pointBuffer = (Point2r*)OAL_MemoryAllocFlag(2 * (verticalSampleCount + horizontalSampleCount) * sizeof(Point2r),
                                   OAL_MEMORY_FLAG_CONTIGUOUS | OAL_MEMORY_FLAG_ZERO | OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE));
      Point2r* const left   = pointBuffer;
      Point2r* const right  = left  + verticalSampleCount;
      Point2r* const top    = right + verticalSampleCount;
      Point2r* const bottom = top   + horizontalSampleCount;
#endif

      // 1. Sample the countour of the dewarped boundary
      sampleImageBoundary(left, right, top, bottom, sampleStep, cameraParams, distortParams, inputImageSize);
  
      REAL xMin, yMin, xMax, yMax;

      // 2. Get the outer ROI and initialize the inner ROI
      //    (the inner ROI takes two passes)

      // Initialize the min/max values as the top-left corner
      xMin = xMax = left[0].x;
      yMin = yMax = left[0].y;

      // Get the global min/max x values for the left boundary
      for (int s=1; s<verticalSampleCount; ++s)
      {
         Point2r& p = left[s];
         if (p.x<xMin) xMin = p.x;
         if (p.x>xMax) xMax = p.x;
      }
      mOuter.x = xMin;
      mInner.x = xMax;

      // Get the global min/max y values for the top boundary
      for (int s=1; s<horizontalSampleCount; ++s)
      {
         Point2r& p = top[s];
         if (p.y<yMin) yMin = p.y;
         if (p.y>yMax) yMax = p.y;
      }
      mOuter.y = yMin;
      mInner.y = yMax;

      // Initialize the min/max values as the bottom-right corner
      xMin = xMax = right[verticalSampleCount-1].x;
      yMin = yMax = right[verticalSampleCount-1].y;

      // Get the global min/max x values for the right boundary
      for (int s=0; s<verticalSampleCount; ++s)
      {
         Point2r& p = right[s];
         if (p.x<xMin) xMin = p.x;
         if (p.x>xMax) xMax = p.x;
      }
      mOuter.width = xMax - mOuter.x;
      mInner.width = xMin - mInner.x;

      // Get the global min/max y values for the bottom boundary
      for (int s=0; s<horizontalSampleCount; ++s)
      {
         Point2r& p = bottom[s];
         if (p.y<yMin) yMin = p.y;
         if (p.y>yMax) yMax = p.y;
      }
      mOuter.height = yMax - mOuter.y;
      mInner.height = yMin - mInner.y;

      // 3. Refine the inner ROI such that the x-range is found using values 
      //    within the y-range and vice versa

      // Refine the left boundary
      int s = 0;
      // Run through the samples until the y-range is found
      for (; s<verticalSampleCount; ++s){if (left[s].y > mInner.y) break;}
      // Initialize the x max value
      xMax = left[s++].x;
      // Run through the y-range
      yMax = mInner.y + mInner.height;
      for (; s<verticalSampleCount; ++s)
      {
         Point2r& p = left[s];
         if (p.y > yMax) break;
         if (p.x > xMax) xMax = p.x;
      }
      mInner.x = xMax;

      // Refine the right column
      s = 0;
      // Run through the samples until the y-range is found
      for (; s<verticalSampleCount; ++s){if (right[s].y > mInner.y) break;}
      // Initialize the x min value
      xMin = right[s++].x;
      // Run through the y-range
      for (; s<verticalSampleCount; ++s)
      {
         Point2r& p = right[s];
         if (p.y > yMax) break;
         if (p.x < xMin) xMin = p.x;
      }
      mInner.width = xMin - mInner.x;

      // Refine the top row
      s = 0;
      // Run through the samples until the x-range is found
      for (; s<horizontalSampleCount; ++s){if (top[s].x > mInner.x) break;}
      // Initialize the y max value
      yMax = top[s++].y;
      // Run through the x-range
      xMax = mInner.x + mInner.width;
      for (; s<horizontalSampleCount; ++s)
      {
         Point2r& p = top[s];
         if (p.x > xMax) break;
         if (p.y > yMax) yMax = p.y;
      }
      mInner.y = yMax;

      // Bottom Row
      s = 0;
      // Run through the samples until the x-range is found
      for (; s<horizontalSampleCount; ++s){if (bottom[s].x > mInner.x) break;}
      // Initialize the y min value
      yMin = bottom[s++].y;
      // Run through the x-range
      for (; s<horizontalSampleCount; ++s)
      {
         Point2r& p = bottom[s];
         if (p.x > xMax) break;
         if (p.y < yMin) yMin = p.y;
      }
      mInner.height = yMin - mInner.y;


#ifdef _WIN32
      delete[] left;
      delete[] right;
      delete[] top;
      delete[] bottom;
#else
      OAL_MemoryFree((void*)pointBuffer);
#endif
   }

   const Roi& inner() const {return mInner;}
   const Roi& outer() const {return mOuter;}

protected:

//   static REAL radialDistanceToImageBoundary(Real angle, Reak sampleStep, Real maxDistance,
//               const CameraParams& cameraParams, const DistortParams& distortParams, const Size& inputImageSize)
//   {
//      // Initialize the min/max values for the top row and left column
//      CameraMapper_<REAL> cameraMapperInv;
//      cameraMapperInv.invert(cameraParams);
//      const DISTORTMAPPER distortMapper(distortParams);
//
//      Point2r upperLeft, lowerRight;
//      cameraMapperInv(Point2r(0,0), upperLeft);
//      cameraMapperInv(Point2r(inputImageSize.width, inputImageSize.height), lowerRight);
//
//      Point2r directionVector(cos(angle), sin(angle));
//      Real distance = 0;
//      for (; distance < maxDistance; distance += sampleStep)
//      {
//         Point2r point = directionVector * distance;
//         distortMapper.map(point, point);
//         if (point.x < upperLeft.x || point.y < upperLeft.y || point.x > lowerRight.x || point.y > lowerRight.y)
//         {
//            break;
//         }
//      }
//
//      return distance;
//   }
//
//   static void sampleImageBoundary(Point2r* const left, Point2r* const right, Point2r* const top, Point2r* const bottom, int sampleCount,
//               const CameraParams& cameraParams, const DistortParams& distortParams, const Size& inputImageSize)
//   {
//      // Initialize the min/max values for the top row and left column
//      CameraMapper_<REAL> cameraMapperInv;
//      cameraMapperInv.invert(cameraParams);
//      const DISTORTMAPPER distortMapper(distortParams);
//
//      Point2r upperLeft, lowerRight;
//      cameraMapperInv(Point2r(0,0), upperLeft);
//      cameraMapperInv(Point2r(inputImageSize.width, inputImageSize.height), lowerRight);
//
//      const REAL PI = 3.1415926535897932384626433832795;
//
//      // Top Boundary
//      REAL angle = atan2(upperLeft.y, lowerRight.x);
//      REAL angleEnd = atan2(upperLeft.y, upperLeft.x);
//      LIB_ASSERT(angle < angleEnd);
//
//      const REAL angleStep = (angleEnd - angle) / sampleCount;
//
//      // Top Row : Determines the inner and outer rectangle y coordinates
//      for (int s=0; s<sampleCount; ++s, angle+=angleStep)
//      {
//         Point2r& p = top[s];
//         p.x = 0;
//         p.y = y;
//         cameraMapperInv.map(p,p);
//         distortMapper.map_inv(p,p);
//      }
//   }

   static void sampleImageBoundary(Point2r* const left, Point2r* const right, Point2r* const top, Point2r* const bottom, int sampleStep,
               const CameraParams& cameraParams, const DistortParams& distortParams, const Size& inputImageSize)
   {
      // Initialize the min/max values for the top row and left column
      CameraMapper_<REAL> cameraMapperInv;
      cameraMapperInv.invert(cameraParams);
      const DISTORTMAPPER distortMapper(distortParams);

      const int horizontalSampleCount = inputImageSize.width  / sampleStep;
      const int verticalSampleCount   = inputImageSize.height / sampleStep;

      // Top Row : Determines the inner and outer rectangle y coordinates
      for (int s=0, x=0; s<horizontalSampleCount; ++s, x+=sampleStep)
      {
         Point2r& p = top[s];
         p.x = x;
         p.y = 0;
         cameraMapperInv.map(p,p);
         distortMapper.map_inv(p,p);
      }

      // Left Column : Determines the inner and outer rectangle x coordinates
      for (int s=0, y=0; s<verticalSampleCount; ++s, y+=sampleStep)
      {
         Point2r& p = left[s];
         p.x = 0;
         p.y = y;
         cameraMapperInv.map(p,p);
         distortMapper.map_inv(p,p);
      }

      // Bottom Row : Determines the inner and outer rectangle heights
      const REAL h_minus_1 = inputImageSize.height-1;
      for (int s=0, x=0; s<horizontalSampleCount; ++s, x+=sampleStep)
      {
         Point2r& p = bottom[s];
         p.x = x;
         p.y = h_minus_1;
         cameraMapperInv.map(p,p);
         distortMapper.map_inv(p,p);
      }

      // Right Column : Determines the inner and outer rectangle widths
      const REAL w_minus_1 = inputImageSize.width -1;
      for (int s=0, y=0; s<verticalSampleCount; ++s, y+=sampleStep)
      {
         Point2r&  p = right[s];
         p.x = w_minus_1;
         p.y = y;
         cameraMapperInv.map(p,p);
         distortMapper.map_inv(p,p);
      }
   }

   Roi mInner,   // An ROI in ideal image coordinates (not pixels!) that is the largest rectangular region containing only mapped pixels.
       mOuter;   // An ROI in ideal image coordinates (not pixels!) that is the smallest rectangular region containing all the mapped pixels.
};

template <class REAL, class DISTORTMAPPER>
class DewarpParamTool_
{
public:
   enum OffsetMode {OFFSET_NONE, CENTER_OPTICAL, CENTER_IMAGE, CENTER_INNER_ROI, CENTER_OUTER_ROI};
   enum ScaleMode  {SCALE_NONE, SCALE_INPUT_FX, SCALE_INPUT_FY, SCALE_INNER_ROI, SCALE_OUTER_ROI};

   typedef CameraParams_<REAL> CameraParams;
   typedef typename DISTORTMAPPER::Params DistortParams;
   typedef Rect_<REAL> Roi;
   typedef Point_<REAL> Point2r;
   typedef DewarpROI_<REAL, DISTORTMAPPER> DewarpROI;
   
   DewarpParamTool_()
   {
   }

   DewarpParamTool_(const CameraParams& cameraParams, const DistortParams& distortParams, const Size& cInputImageSize)
   {
      init(cameraParams, distortParams, cInputImageSize);
   }

   template <class REAL2, class DISTORTMAPPER2>
   DewarpParamTool_(const DewarpParamTool_<REAL2, DISTORTMAPPER2>& other) :
      mDewarpROI(other.dewarpROI()), mInputImageSize(other.inputImageSize()), mInputOffset(other.inputOffset())
   {
   }

   
   void init(const CameraParams& cameraParams, const DistortParams& distortParams, const Size& cInputImageSize)
   {
      // Get the inner and outter clipping ROI of the dewarped image
      // This requires dewarping the edge pixels of the warped image, an iterative process,
      // which generates the dewarped image edge.
      mDewarpROI.init(cameraParams, distortParams, cInputImageSize);

      mInputImageSize = cInputImageSize;
      mInputOffset.x  = cameraParams.cx;
      mInputOffset.y  = cameraParams.cy;
   }

   void configure(CameraParams& dewarpCameraParams, const Size& dewarpImageSize,
                  REAL scale = 1, REAL aspectRatio = 1,
                  ScaleMode scaleMode = SCALE_INNER_ROI, OffsetMode offsetMode = CENTER_OPTICAL)
   {
      const Roi& innerROI = mDewarpROI.inner();
      const Roi& outerROI = mDewarpROI.outer();
      const Point2r innerROICenter(innerROI.x + innerROI.width*(REAL)0.5, innerROI.y + innerROI.height*(REAL)0.5);
      const Point2r outerROICenter(outerROI.x + outerROI.width*(REAL)0.5, outerROI.y + outerROI.height*(REAL)0.5);
      const Point2r dewarpImageCenter(dewarpImageSize.width*(REAL)0.5, dewarpImageSize.height*(REAL)0.5);

      // 1. The trivial Offset Modes

      if(offsetMode == CENTER_OPTICAL)
      {
         // set the optical axis (i.e. the princpal point) at the center of the image
         dewarpCameraParams.cx = (REAL)dewarpImageSize.width  * (REAL)0.5;
         dewarpCameraParams.cy = (REAL)dewarpImageSize.height * (REAL)0.5;
      }
      else if (offsetMode == CENTER_IMAGE)
      {
         // Preserve the image center: the center of the dewarped image is also the center of the warped image
         dewarpCameraParams.cx = mInputOffset.x + (REAL)(dewarpImageSize.width  - mInputImageSize.width ) * (REAL)0.5;
         dewarpCameraParams.cy = mInputOffset.y + (REAL)(dewarpImageSize.height - mInputImageSize.height) * (REAL)0.5;
      }

      // 2. The trivial Scale Modes

      if (scaleMode == SCALE_INPUT_FX)
      {
         // scale fx, and modify f to have the desired aspect ratio
         dewarpCameraParams.fx *= scale;
         dewarpCameraParams.fy = dewarpCameraParams.fx;
         dewarpCameraParams.fx *= aspectRatio;
      }
      else if (scaleMode == SCALE_INPUT_FY)
      {
         // scale fy, and modify fx to have the desired aspect ratio
         dewarpCameraParams.fy *= scale;
         dewarpCameraParams.fx = dewarpCameraParams.fy;
         dewarpCameraParams.fx *= aspectRatio;
      }

      // 3.  Scale w.r.t. to a dewarp ROI

      else if (scaleMode == SCALE_INNER_ROI || scaleMode == SCALE_OUTER_ROI)
      {
         // Select the appropriate ideal center and image offset
         const Point2r nullCenter(0,0);
         const Point2r imageCenter(dewarpImageSize.width*(REAL)0.5, dewarpImageSize.height*(REAL)0.5);
         const Point2r imageOffset(dewarpCameraParams.cx, dewarpCameraParams.cy);
         const Point2r* center, *offset;
         if      (offsetMode == CENTER_INNER_ROI) {center = &innerROICenter; offset = &dewarpImageCenter;}
         else if (offsetMode == CENTER_OUTER_ROI) {center = &outerROICenter; offset = &dewarpImageCenter;}
         else                                       {center = &nullCenter;     offset = &imageOffset;}

         // Select the appropriate roi and min/max setting
         const Roi* roi;
         int flag;
         if    (scaleMode == SCALE_INNER_ROI)  {roi = &innerROI; flag = USE_MAX;}
         else /*scaleMode == SCALE_OUTER_ROI*/ {roi = &outerROI; flag = USE_MIN;}

         // Multiply the normalized scale by the scale determined by the ROI
         scale *= roiScale(*roi, *center, *offset, dewarpImageSize, aspectRatio, flag);
         
         // Set the dewarp image scales
         dewarpCameraParams.fx = aspectRatio * scale;
         dewarpCameraParams.fy = scale;
      }

      // 4. Offset w.r.t. a dewarp ROI

      if (offsetMode == CENTER_INNER_ROI)
      {
         // Center the image on the inner ROI
         dewarpCameraParams.cx = dewarpImageCenter.x - dewarpCameraParams.fx * innerROICenter.x;
         dewarpCameraParams.cy = dewarpImageCenter.y - dewarpCameraParams.fy * innerROICenter.y;
      }
      else if (offsetMode == CENTER_OUTER_ROI)
      {
         // Center the image on the outer ROI
         dewarpCameraParams.cx = dewarpImageCenter.x - dewarpCameraParams.fx * outerROICenter.x;
         dewarpCameraParams.cy = dewarpImageCenter.y - dewarpCameraParams.fy * outerROICenter.y;
      }
   }

   // Getters
   const DewarpROI& dewarpROI() const {return mDewarpROI;}
   const Size& inputImageSize() const {return mInputImageSize;}
   const Point2r& inputOffset() const {return mInputOffset;}

protected:
   enum {USE_MAX, USE_MIN};

   static REAL roiScale(const Roi& rect, const Point2r& center, const Point2r& offset, const Size& dewarpImageSize, REAL aspectRatio, int flag)
   {
      // This function takes a real-valued ROI (in ideal image coordinates (not pixels!)) and returns the either
      // (1) the maximum scale that can be applied before an edge (any edge) of the rectangle is reached, or
      // (2) the minimum scale that can be applied such that includes the entire ROI

      // Map the upper-left and bottom-right image points to their corresponding locations on the ideal image plane
      Point2r upperLeft(-offset.x/aspectRatio, -offset.y);
      Point2r bottomRight( (dewarpImageSize.width  - offset.x) / aspectRatio, dewarpImageSize.height - offset.y );

      // Assert the center is within the ROI
      REAL distanceLeft = rect.x - center.x;
      if (distanceLeft  >= 0) return 0;
      REAL distanceRight = rect.width + rect.x - center.x;
      if (distanceRight <= 0) return 0;
      REAL distanceUp = rect.y - center.y;
      if (distanceUp   >= 0) return 0;
      REAL distanceDown = rect.height + rect.y - center.y;
      if (distanceDown <= 0) return 0;
   
      // Get the distance ratio for each direction
      REAL scaleLeft  = upperLeft.x   / distanceLeft;
      REAL scaleRight = bottomRight.x / distanceRight;
      REAL scaleUp    = upperLeft.y   / distanceUp;
      REAL scaleDown  = bottomRight.y / distanceDown;
   
      // Initialize the scale
      REAL scale = scaleLeft;
   
      if (flag == USE_MAX)
      {
         // use the maximum value
         if (scaleRight > scale) scale = scaleRight;
         if (scaleUp    > scale) scale = scaleUp;
         if (scaleDown  > scale) scale = scaleDown;
      }
      else
      {
         // use the minimum value
         if (scaleRight < scale) scale = scaleRight;
         if (scaleUp    < scale) scale = scaleUp;
         if (scaleDown  < scale) scale = scaleDown;
      }

      return scale;
   }

   DewarpROI_<REAL, DISTORTMAPPER> mDewarpROI;     // the inner and outer dewarp ROIs
   Size mInputImageSize;
   Point2r mInputOffset;
};

#undef Rect_
#undef Size
#undef Point_

#endif   /* DEWARPTOOLS_H */
