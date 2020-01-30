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

#ifndef CAMERAMAPPER_H
#define CAMERAMAPPER_H

#ifdef _OPENCV_TYPES
#include "core/core.hpp"
#else
#include "icp_types.h"
#endif

template <class REAL>
struct CameraParams_
{
   REAL fx,     // horizontal scaling (pixels/meter)
        cx,     // horizontal offset  (pixels)
        fy,     // vertical   scaling (pixels/meter)
        cy;     // vertical   offset  (pixels)

   // Default constructor
   CameraParams_(REAL _fx=1, REAL _cx=0, REAL _fy=1, REAL _cy=0)
      : fx(_fx), cx(_cx), fy(_fy), cy(_cy)
   {
   }

   // Construct from another type of CameraParams_
   template <class REAL2>
   CameraParams_(const CameraParams_<REAL2>& other)
      : fx(other.fx), cx(other.cx), fy(other.fy), cy(other.cy)
   {
   }

   // Construct from a camera matrix of any type
   template <class REAL2>
   CameraParams_(const REAL2* matrixData)
      : fx(matrixData[0]), cx(matrixData[2]), fy(matrixData[4]), cy(matrixData[5])
   {
   }

   // Define a camera matrix from the parameters
   template <class REAL2>
   inline void toMatrix(REAL2* matrixData) const
   {
	   matrixData[0] = (REAL2)fx;
	   matrixData[1] = 0;
	   matrixData[2] = (REAL2)cx;
	   matrixData[3] = 0;
	   matrixData[4] = (REAL2)fy;
	   matrixData[5] = (REAL2)cy;
	   matrixData[6] = 0;
	   matrixData[7] = 0;
	   matrixData[8] = 1;
   }

#ifndef _OPENCV_TYPES
   // Create a camera matrix from the parameters
   template <class REAL2>
   inline void toMatrix(ICP_Mat<REAL2>& matrix) const
   {
   	   matrix.Create(3,3);
   	   toMatrix(matrix.ReturnPtr());
   }
#endif
};

template <class REAL>
class CameraMapper_
{
public:
   
   typedef REAL Real;
   typedef CameraParams_<REAL> Params;
#ifdef _OPENCV_TYPES
   typedef cv::Point_<REAL> Point;
#else
   typedef ICP_Point2<REAL> Point;
#endif
   
   // Default constructor
   CameraMapper_(){}

   // Construct from a set of camera parameters
   CameraMapper_(const Params& params) : mParams(params) {}

   // Construct from a camera mapper of another type
   template<class REAL2>
   CameraMapper_(const CameraMapper_<REAL2>& other) : mParams(other.getParams()) {}

   // Construct from a camera matrix of any type
   template <class REAL2>
   CameraMapper_(const REAL2* matrixData) : mParams(matrixData)
   {
   }

   // Camera mapping
   inline void map(const Point& src, Point& dst) const
   {
      dst.x = mParams.fx*src.x + mParams.cx;
      dst.y = mParams.fy*src.y + mParams.cy;
   }

   // Invert w.r.t. a set of camera parameters
   void invert(const Params& params) {invert(params, mParams);}

   // Invert w.r.t. another camera mapper
   void invert(const CameraMapper_<REAL>& other) {invert(other.mParams, mParams);}

   // Set parameters
   void setParams(const Params& params){mParams = params;}

   // Get parameters
   const Params& getParams() const {return mParams;}

   // Invert camera parameters
   static inline void invert(const Params& src, Params& dst)
   {
      dst.fx = 1 / src.fx;
      dst.fy = 1 / src.fy;
      dst.cx = -src.cx * dst.fx;
      dst.cy = -src.cy * dst.fy;
   }

protected:
    Params mParams;
};


#endif   /* CAMERAMAPPER_H */
