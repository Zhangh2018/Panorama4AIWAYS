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

#ifndef DEWARPMAPPER_H
#define DEWARPMAPPER_H

#ifdef _OPENCV_TYPES
#include "core/core.hpp"
#else
#include "icp_types.h"
#endif

#include "camera_mapper.h"

template<class REAL, class DISTORTMAPPER>
class DewarpMapper_
{
public:
#ifdef _OPENCV_TYPES
   typedef cv::Point_<REAL> Point;
#else
   typedef ICP_Point2<REAL> Point;
#endif
   typedef REAL Real;
   typedef CameraMapper_<REAL> CameraMapper;
   typedef DISTORTMAPPER DistortMapper;
   typedef CameraParams_<REAL> CameraParams;
   typedef typename DistortMapper::Params DistortParams;

   // Default constructor
   DewarpMapper_(){}

   // Constructor from parameters
   DewarpMapper_(const CameraParams&  inputCameraParams,
                 const DistortParams& distortParams,
                 const CameraParams&  outputCameraParams)
      : mInputCameraMapper(inputCameraParams),
        mDistortMapper(distortParams), 
        mOutputCameraMapper(outputCameraParams)
   {
      mOutputCameraMapperInverse.invert(mOutputCameraMapper);
   }

   // Constructor from another dewarp mapper of any type
   template<class REAL2, class DISTORTMAPPER2>
   DewarpMapper_(const DewarpMapper_<REAL2, DISTORTMAPPER2>& other)
      : mInputCameraMapper(other.getInputCameraMapper()),
        mDistortMapper(other.getDistortMapper()),
        mOutputCameraMapper(other.getOutputCameraMapper())
   {
      mOutputCameraMapperInverse.invert(mOutputCameraMapper);
   }

   // Dewarp Mapping
   void map(const Point& src, Point& dst) const
   {
      // 1. Map the undistorted pixel to the undistorted image plane
      mOutputCameraMapperInverse.map(src, dst);

      // 2. Map the undistorted image plane to the distorted image plane
      mDistortMapper.map(dst,dst);

      // 3. Map the distorted image plane to the distorted pixel
      mInputCameraMapper.map(dst,dst);
   }

   // Setters
   void setParams(const CameraParams&  inputCameraParams,
                  const DistortParams& distortParams,
                  const CameraParams&  outputCameraParams)
   {
      mInputCameraMapper.setParams(inputCameraParams);
      mDistortMapper.setParams(distortParams);
      setOutputCameraParams(outputCameraParams);
   }

   void setInputCameraParams(const CameraParams& params) {mInputCameraMapper.setParams(params);}
   void setDistortParams(const DistortParams& distortParams) {mDistortMapper.setParams(distortParams);}
   void setOutputCameraParams(const CameraParams& params)
   {
      // Set the output camera mapper and define the inverse output camera mapper
      mOutputCameraMapper.setParams(params);
      mOutputCameraMapperInverse.invert(mOutputCameraMapper);
   }

   // Getters
   const CameraMapper&  getInputCameraMapper()  const {return mInputCameraMapper;}
   const DistortMapper& getDistortMapper()      const {return mDistortMapper;}
   const CameraMapper&  getOutputCameraMapper() const {return mOutputCameraMapper;}

protected:
   // Mappers
   CameraMapper  mInputCameraMapper;     // The orignal camera mapper
   DistortMapper mDistortMapper;         // The distortion mapper
   CameraMapper  mOutputCameraMapper;    // The dewarp camera mapper (scales and offsets the dewarped image)

   // The output camera mapper inverse is what is actually used in the mapping, 
   // not the output camera mapper itself.
   CameraMapper mOutputCameraMapperInverse;
};


#endif   /* DEWARPMAPPER_H */
