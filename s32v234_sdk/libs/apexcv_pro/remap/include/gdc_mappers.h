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

#ifndef CGVMAPPERS_H
#define CGVMAPPERS_H

#include "perspective_mapper.h"

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

/**** Homography Helper Functions ****************************************************/

// Set parameters from intrinsic, extrinsic, and view parameters
template <class REAL>
void configureHomographyParams(HomographyParams_<REAL>& homographyParams, const REAL* rotationMatrix, const REAL* translationVector)
{
   const REAL& z = translationVector[2];
   
   homographyParams.h1 = rotationMatrix[0]/z;   homographyParams.h2 = rotationMatrix[1]/z;   homographyParams.h3 = translationVector[0]/z;
   homographyParams.h4 = rotationMatrix[3]/z;   homographyParams.h5 = rotationMatrix[4]/z;   homographyParams.h6 = translationVector[1]/z;
   homographyParams.h7 = rotationMatrix[6]/z;   homographyParams.h8 = rotationMatrix[7]/z;   // h9 = 1 by definition
}

template <class REAL>
void configureHomographyParams(HomographyParams_<REAL>& homographyParams, const CameraParams_<REAL>& cameraParams, const REAL* rotationMatrix, const REAL* translationVector, const CameraParams_<REAL>& viewParams)
{
   const REAL* r = rotationMatrix;     // for convenience of notation
   const REAL* t = translationVector;     // for convenience of notation

   // The following computation is equivalent to H = C * [r1, r2, t ] * V
   // where C is the camera matrix defined by cameraParams,
   //       ri is the ith column of the rotation matrix
   //       V is the camera matrix defined by viewParams

   homographyParams.h1 = viewParams.fx * r[0];  homographyParams.h2 = viewParams.fy * r[1];  homographyParams.h3 = viewParams.cx * r[0] + viewParams.cy * r[1] + t[0];
   homographyParams.h4 = viewParams.fx * r[3];  homographyParams.h5 = viewParams.fy * r[4];  homographyParams.h6 = viewParams.cx * r[3] + viewParams.cy * r[4] + t[1];
   homographyParams.h7 = viewParams.fx * r[6];  homographyParams.h8 = viewParams.fy * r[7];  const REAL h9 = viewParams.cx * r[6] + viewParams.cy * r[7] + t[2];

   homographyParams.h1 = cameraParams.fx * homographyParams.h1 + cameraParams.cx * homographyParams.h7;
   homographyParams.h2 = cameraParams.fx * homographyParams.h2 + cameraParams.cx * homographyParams.h8;
   homographyParams.h3 = cameraParams.fx * homographyParams.h3 + cameraParams.cx * h9;
   homographyParams.h4 = cameraParams.fy * homographyParams.h4 + cameraParams.cy * homographyParams.h7;
   homographyParams.h5 = cameraParams.fy * homographyParams.h5 + cameraParams.cy * homographyParams.h8;
   homographyParams.h6 = cameraParams.fy * homographyParams.h6 + cameraParams.cy * h9;

   // Divide the homography matrix by the last element
   const REAL h9_inv = (REAL)1 / h9;
   homographyParams.h1 *= h9_inv;
   homographyParams.h2 *= h9_inv;
   homographyParams.h3 *= h9_inv;
   homographyParams.h4 *= h9_inv;
   homographyParams.h5 *= h9_inv;
   homographyParams.h6 *= h9_inv;
   homographyParams.h7 *= h9_inv;
   homographyParams.h8 *= h9_inv;
}

template <class REAL>
void configureHomographyParams(HomographyParams_<REAL>& homographyParams, const CameraParams_<REAL>& cameraParams, const REAL* rotationMatrix, REAL distance, const CameraParams_<REAL>& viewParams)
{
   const REAL* r = rotationMatrix;     // for convenience of notation

   // The following computation is equivalent to H = C * [r1, r2, distance * r3 ] * V
   // where C is the camera matrix defined by cameraParams,
   //       ri is the ith column of the rotation matrix
   //       V is the camera matrix defined by viewParams

   homographyParams.h1 = viewParams.fx * r[0];  homographyParams.h2 = viewParams.fy * r[1];  homographyParams.h3 = viewParams.cx * r[0] + viewParams.cy * r[1] + distance * r[2];
   homographyParams.h4 = viewParams.fx * r[3];  homographyParams.h5 = viewParams.fy * r[4];  homographyParams.h6 = viewParams.cx * r[3] + viewParams.cy * r[4] + distance * r[5];
   homographyParams.h7 = viewParams.fx * r[6];  homographyParams.h8 = viewParams.fy * r[7];  const REAL h9 = viewParams.cx * r[6] + viewParams.cy * r[7] + distance * r[8];

   homographyParams.h1 = cameraParams.fx * homographyParams.h1 + cameraParams.cx * homographyParams.h7;
   homographyParams.h2 = cameraParams.fx * homographyParams.h2 + cameraParams.cx * homographyParams.h8;
   homographyParams.h3 = cameraParams.fx * homographyParams.h3 + cameraParams.cx * h9;
   homographyParams.h4 = cameraParams.fy * homographyParams.h4 + cameraParams.cy * homographyParams.h7;
   homographyParams.h5 = cameraParams.fy * homographyParams.h5 + cameraParams.cy * homographyParams.h8;
   homographyParams.h6 = cameraParams.fy * homographyParams.h6 + cameraParams.cy * h9;

   // Divide the homography matrix by the last element
   const REAL h9_inv = (REAL)1 / h9;
   homographyParams.h1 *= h9_inv;
   homographyParams.h2 *= h9_inv;
   homographyParams.h3 *= h9_inv;
   homographyParams.h4 *= h9_inv;
   homographyParams.h5 *= h9_inv;
   homographyParams.h6 *= h9_inv;
   homographyParams.h7 *= h9_inv;
   homographyParams.h8 *= h9_inv;
}

template <class REAL>
HomographyParams_<REAL> configureHomographyParams(const CameraParams_<REAL>& cameraParams, const REAL* rotationMatrix, REAL distance, const CameraParams_<REAL>& viewParams)
{
   HomographyParams_<REAL> params;
   configureHomographyParams(params, cameraParams, rotationMatrix, distance, viewParams);
   return params;
}


/***** Top View Mapper Helper Functions **********************************************/

template <class REAL>
struct RotationAngles_
{
   // ALL ANGLES ARE IN RADIANS
   REAL tilt,     // angle of rotation about the world x-axis
        swing,    // angle of rotation about the world z-axis
        pan;      // angle of rotation about the camera z-axis

   RotationAngles_(REAL _tilt = 0, REAL _swing = 0, REAL _pan = 0) : tilt(_tilt), swing(_swing), pan(_pan)
   {
   }

   template <class OTHER>
   RotationAngles_(const RotationAngles_<OTHER>& other)
   {
      tilt = (REAL)other.tilt;
      swing = (REAL)other.swing;
      pan = (REAL)other.pan;
   }
};

template <class REAL>
CameraParams_<REAL> configureViewParams(const Rect_<REAL>& roi, const Size& imageSize, REAL scale = 1)
{
   const REAL offsetAdjustFactor = (REAL)0.5 * ( (REAL)1 - scale);

   return CameraParams_<REAL>(scale * roi.width  / (REAL)imageSize.width,  roi.x + roi.width  * offsetAdjustFactor,
                              scale * roi.height / (REAL)imageSize.height, roi.y + roi.height * offsetAdjustFactor);
}

template <class REAL>
void rotateX_(REAL angle, REAL* matrixPtr)
{
   // angle in radians
   REAL _sin = sin(angle);
   REAL _cos = cos(angle);

   matrixPtr[0] = 1;      matrixPtr[1] = 0;      matrixPtr[2] = 0;
   matrixPtr[3] = 0;      matrixPtr[4] = _cos;   matrixPtr[5] = -_sin;
   matrixPtr[6] = 0;      matrixPtr[7] = _sin;   matrixPtr[8] =  _cos;
}

template <class REAL>
void rotateY_(REAL angle, REAL* matrixPtr)
{
   // angle in radians
   REAL _sin = sin(angle);
   REAL _cos = cos(angle);

   matrixPtr[0] = _cos;   matrixPtr[1] = 0;   matrixPtr[2] = _sin;
   matrixPtr[3] = 0;      matrixPtr[4] = 1;   matrixPtr[5] = 0;
   matrixPtr[6] = -_sin;  matrixPtr[7] = 0;   matrixPtr[8] = _cos;
}

template <class REAL>
void rotateZ_(REAL angle, REAL* matrixPtr)
{
   // angle in radians
   REAL _sin = sin(angle);
   REAL _cos = cos(angle);

   matrixPtr[0] = _cos;   matrixPtr[1] = -_sin;  matrixPtr[2] = 0;
   matrixPtr[3] = _sin;   matrixPtr[4] = _cos;   matrixPtr[5] = 0;
   matrixPtr[6] = 0;      matrixPtr[7] = 0;      matrixPtr[8] = 1;
}

#ifdef _OPENCV_TYPES

void createRotationMatrix(cv::Mat& R, const RotationAngles_<double> angles)
{
    // angles must be in radians
    cv::Mat panMatrix(3,3,CV_64FC1);
    rotateZ_(-angles.pan, (double*)panMatrix.data);
    cv::Mat tiltMatrix(3,3,CV_64FC1);
    rotateX_(-angles.tilt, (double*)tiltMatrix.data);
    cv::Mat swingMatrix(3,3,CV_64FC1);
    rotateZ_(-angles.swing, (double*)swingMatrix.data);

    R = swingMatrix * tiltMatrix * panMatrix;
}

#else

template <class REAL>
void rotateX_(REAL angle, ICP_Mat<REAL>& R)
{
   // angle in radians
   R.Create(3, 3);
   rotateX_(angle, R.ReturnPtr());
}

template <class REAL>
void rotateY_(REAL angle, ICP_Mat<REAL>& R)
{
   // angle in radians
   R.Create(3, 3);
   rotateY_(angle, R.ReturnPtr());
}

template <class REAL>
void rotateZ_(REAL angle, ICP_Mat<REAL>& R)
{
   // angle in radians
   R.Create(3, 3);
   rotateZ_(angle, R.ReturnPtr());
}

template <class REAL>
void createRotationMatrix(ICP_Mat<REAL>& R, const RotationAngles_<REAL> angles)
{
    // angles must be in radians
    ICP_Mat<REAL> panMatrix;
    rotateZ_(-angles.pan, panMatrix);
    ICP_Mat<REAL> tiltMatrix;
    rotateX_(-angles.tilt, tiltMatrix);
    ICP_Mat<REAL> swingMatrix;
    rotateZ_(-angles.swing, swingMatrix);

    R = swingMatrix * tiltMatrix * panMatrix;
}

template <class REAL>
void createWorldToCameraRotationMatrix(ICP_Mat<REAL>& R, const RotationAngles_<REAL> angles)
{
    // angles must be in radians
    createRotationMatrix(R, angles);
}

template <class REAL>
void createCameraToWorldRotationMatrix(ICP_Mat<REAL>& R, const RotationAngles_<REAL> angles)
{
    // angles must be in radians
    ICP_Mat<REAL> panMatrix;
    rotateZ_(angles.pan, panMatrix);
    ICP_Mat<REAL> tiltMatrix;
    rotateX_(angles.tilt, tiltMatrix);
    ICP_Mat<REAL> swingMatrix;
    rotateZ_(angles.swing, swingMatrix);

    R = panMatrix * tiltMatrix * swingMatrix;
}

#endif

template <class REAL, class DISTORTMAPPER>
void configureTopMapper(PerspectiveMapper_<REAL, DISTORTMAPPER>& topMapper,
                  const CameraParams_<REAL>& cameraParams,
                  const typename DISTORTMAPPER::Params& distortParams,
                  const REAL* rotationMatrix,
                  const REAL* translationVector,
                  const Rect_<REAL>& groundROI,
                  const Size& topImageSize,
                  REAL scale = 1)
{
   // 1. Define the top mapper viewing parameters
   CameraParams_<REAL> topViewParams = configureViewParams(groundROI, topImageSize, scale);

   // 2. Define the top mapper homography
   HomographyParams_<REAL> topHomographyParams;
   configureHomographyParams(topHomographyParams, CameraParams_<REAL>(), rotationMatrix, translationVector, topViewParams);

   // 3. Set the top mapper parameteres
   topMapper.setParams(cameraParams, distortParams, topHomographyParams);
}

template <class REAL, class DISTORTMAPPER>
void configureTopMapper(PerspectiveMapper_<REAL, DISTORTMAPPER>& topMapper,
                  const CameraParams_<REAL>& cameraParams,
                  const typename DISTORTMAPPER::Params& distortParams,
                  const REAL* rotationMatrix,
                  const REAL& height,
                  const Rect_<REAL>& groundROI,
                  const Size& topImageSize,
                  REAL scale = 1)
{
   // 1. Define the top mapper viewing parameters
   CameraParams_<REAL> topViewParams = configureViewParams(groundROI, topImageSize, scale);

   // 2. Define the top mapper homography
   HomographyParams_<REAL> topHomographyParams;
   configureHomographyParams(topHomographyParams, CameraParams_<REAL>(), rotationMatrix, height, topViewParams);

   // 3. Set the top mapper parameteres
   topMapper.setParams(cameraParams, distortParams, topHomographyParams);
}

template <class REAL, class DISTORTMAPPER>
void configureTopMapper(PerspectiveMapper_<REAL, DISTORTMAPPER>& topMapper,
                  const CameraParams_<REAL>& cameraParams,
                  const typename DISTORTMAPPER::Params& distortParams,
                  const RotationAngles_<REAL>& angles,
                  const REAL& height,
                  const Rect_<REAL>& groundROI,
                  const Size& topImageSize,
                  REAL scale = 1)
{
   ICP_Mat<REAL> R;
   createRotationMatrix(R, angles);
   configureTopMapper(topMapper, cameraParams, distortParams, R.ReturnPtr(), height, groundROI, topImageSize, scale);
}

/***** Split (left and right) View Mapper Helper Functions **********************************************/

template <class REAL>
struct SplitAngles_
{
   // ALL ANGLES ARE IN RADIANS
   REAL theta1,   // minimum horizontal angle   (between 0 and PI/2)
        theta2,   // maximum horizontal angle   (between theta1 and PI/2)
        phi1,     // minimum vertical angle     (-ve, unless you want the minimum angle above the horizon)
        phi2;     // maximum vertical angle     (+ve, unless you want the maximum angle below the horizon)

   SplitAngles_(REAL _theta1 = (REAL)0.25, REAL _theta2 = (REAL)0.75, REAL _phi1 = (REAL)-0.5, REAL _phi2 = (REAL)0.5) : theta1(_theta1), theta2(_theta2), phi1(_phi1), phi2(_phi2)
   {
   }

   template <class OTHER>
   SplitAngles_(const CameraParams_<OTHER>& other)
   {
      theta1 = (REAL)other.theta1;
      theta2 = (REAL)other.theta2;
      phi1 = (REAL)other.phi1;
      phi2 = (REAL)other.phi2;
   }
};

template <class REAL>
Size getSplitSize(int desiredWidth, const SplitAngles_<REAL>& angles)
{
   // Returns the image width and height that most closely preserves the world aspect ratio while enforcing both width and height to be factors of 16, based on the desired image width
   const int FACTOR = 16;
   const int HALF_FACTOR = FACTOR >> 1;

   // Width Adjustment (so that it is divisible by FACTOR)
   const int wMod = desiredWidth % FACTOR;
   int widthAdjust = wMod < HALF_FACTOR ? wMod : wMod - FACTOR;
   int width = desiredWidth - widthAdjust;
   
   // Get the height that preserves the aspect ratio
   REAL H_over_W = ( tan(angles.phi2) - tan(angles.phi1) ) / ( tan(angles.theta1) - tan(angles.theta2) );    // world height over width ratio
   const int height = int(width * H_over_W);

   // Height Adjustment (so that it is divisible by FACTOR)
   const int hMod = height % FACTOR;
   int heightAdjust = hMod < HALF_FACTOR ? hMod : hMod - FACTOR;

   return Size(width, height - heightAdjust);
}

template <class REAL>
void setPhi2ToPreserveAspectRatio(SplitAngles_<REAL>& angles, const Size& sideImageSize)
{
   // Sets Phi2 to preserve the aspect ratio of the side plane, given the side image size
   REAL aspectRatio =  (REAL)(sideImageSize.height) / (REAL)sideImageSize.width;      // Divide width by two since each view is only half the width of the splitview image

   // Get the height that preserves the aspect ratio
   REAL tanTheta1 = tan(angles.theta1);
   REAL tanTheta2 = tan(angles.theta2);
   REAL tanPhi1   = tan(angles.phi1);
   REAL tanPhi2   = ( tanPhi1 - aspectRatio * ( tanTheta2 - tanTheta1 ) );    // world height over width ratio
   
   angles.phi2 = atan(tanPhi2);

   // Check
   //REAL fx = (tanTheta2 - tanTheta1) / sideImageSize.width;
   //REAL fy = (-tanPhi2 + tanPhi1)    / sideImageSize.height;
}

template <class REAL, class DISTORTMAPPER>
void configureSplitMappers(PerspectiveMapper_<REAL, DISTORTMAPPER>& leftMapper,
                        PerspectiveMapper_<REAL, DISTORTMAPPER>& rightMapper,
                        const CameraParams_<REAL>& cameraParams,
                        const typename DISTORTMAPPER::Params& distortParams,
                        const REAL* rotationMatrix,                  // rotationMatrix is of the vehicle coordinate system w.r.t. the camera
                        const SplitAngles_<REAL>& splitAngles,       // the 4 angles defining the field of view for both splitviews
                        const Size& sideImageSize)                   // the size of both side images
{
   const REAL tanTheta1 = tan(splitAngles.theta1);
   const REAL tanTheta2 = tan(splitAngles.theta2);
   const REAL tanPhi1 = -tan(splitAngles.phi1);
   const REAL tanPhi2 = -tan(splitAngles.phi2);

   CameraParams_<REAL> viewParams( (tanTheta2 - tanTheta1) / sideImageSize.width,  tanTheta1,
                                   (tanPhi2 - tanPhi1)     / sideImageSize.height, tanPhi1);
   
   const REAL *r = rotationMatrix;      // for ease of notation
   REAL r_[9];

   // 1.  The left side view
   // Define the left plane rotation: [-r2, r3, -r1]
   r_[0] = -r[1];    r_[1] = r[2];    r_[2] = -r[0];
   r_[3] = -r[4];    r_[4] = r[5];    r_[5] = -r[3];
   r_[6] = -r[7];    r_[7] = r[8];    r_[8] = -r[6];

   HomographyParams_<REAL> homographyParams;
   configureHomographyParams(homographyParams, CameraParams_<REAL>(), r_, (REAL)1, viewParams);

   leftMapper.setParams(cameraParams, distortParams, homographyParams);

   // 2.  The right side view
   viewParams.cx = -tanTheta2;      // only the x offset differs from the left view

   // Define the right plane rotation: [r2, r3, r1]
   // Second column remains the same
   r_[0] = r[1];    /*r_[1] = r[2];*/    r_[2] = r[0];
   r_[3] = r[4];    /*r_[4] = r[5];*/    r_[5] = r[3];
   r_[6] = r[7];    /*r_[7] = r[8];*/    r_[8] = r[6];

   configureHomographyParams(homographyParams, CameraParams_<REAL>(), r_, (REAL)1, viewParams);

   rightMapper.setParams(cameraParams, distortParams, homographyParams);
}

#undef Rect_
#undef Size
#undef Point_

#endif   /* CGVMAPPERS_H */
