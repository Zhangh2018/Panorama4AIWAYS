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

#ifndef PERSPECTIVEMAPPER_H
#define PERSPECTIVEMAPPER_H

#ifdef _OPENCV_TYPES
#include "core/core.hpp"
#else
#include "icp_types.h"
#endif

#include "camera_mapper.h"
#include "homography.h"

template<class REAL, class DISTORTMAPPER>
class PerspectiveMapper_
{
public:
#ifdef _OPENCV_TYPES
   typedef cv::Point_<REAL> Point;
#else
   typedef ICP_Point2<REAL> Point;
#endif
   typedef REAL                    Real;
   typedef CameraMapper_<REAL>     CameraMapper;
   typedef DISTORTMAPPER           DistortMapper;
   typedef Homography_<REAL>       Homography;
   typedef CameraParams_<REAL>     CameraParams;
   typedef typename DISTORTMAPPER::Params DistortParams;
   typedef HomographyParams_<REAL> HomographyParams;

   // Default constructor
   PerspectiveMapper_(){}

   // Constructor from parameters
   PerspectiveMapper_(const CameraParams&     cameraParams,
                      const DistortParams&    distortParams,
                      const HomographyParams& homographyParams)
      : mCameraMapper(cameraParams),
        mDistortMapper(distortParams),
        mHomography(homographyParams)
   {
   }

   // Constructor from another perspective mapper of any type
   template<class REAL2, class DISTORTMAPPER2>
   PerspectiveMapper_(const PerspectiveMapper_<REAL2, DISTORTMAPPER2>& other)
      : mCameraMapper(other.getCameraMapper()),
        mDistortMapper(other.getDistortMapper()),
        mHomography(other.getHomography())
   {
   }

   // Perspective Correction Mapping
   void map(const Point& src, Point& dst) const
   {
      // 1.  Map the perspective-corrected pixel to the ideal image
      mHomography.map(src, dst);

      // 3.  Map the ideal image plane to the warped ideal image plane.
      mDistortMapper.map(dst, dst);

      // 4.  Map the warped ideal image plane to the warped image
      mCameraMapper.map(dst, dst);
   }

   // Setters
   void setParams(const CameraParams& cameraParams,
                  const DistortParams& distortParams,
                  const HomographyParams& homographyParams)
   {
      mCameraMapper.setParams(cameraParams);
      mDistortMapper.setParams(distortParams);
      mHomography.setParams(homographyParams);
   }

   void setCameraParams(const CameraParams& cameraParams) {mCameraMapper.setParams(cameraParams);}
   void setDistortParams(const DistortParams& distortParams) {mDistortMapper.setParams(distortParams);}
   void setHomographyParams(const HomographyParams& homographyParams) {mHomography.setParams(homographyParams);}

   // Getters
   const CameraMapper&  getCameraMapper()  const {return mCameraMapper;}
   const DistortMapper& getDistortMapper() const {return mDistortMapper;}
   const Homography&    getHomography()    const {return mHomography;}

protected:
    // Mappers
    CameraMapper  mCameraMapper;      // The orignal camera mapper
    DistortMapper mDistortMapper;     // The distortion mapper
    Homography    mHomography;        // The homography from the perspective-corrected image to the world plane
};

#endif   /* PERSPECTIVEMAPPER_H */
