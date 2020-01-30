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

#ifndef HOMOGRAPHY_H
#define HOMOGRAPHY_H

#ifdef _OPENCV_TYPES
#include "core/core.hpp"
#else
#include "icp_types.h"
#endif

template <class REAL>
struct HomographyParams_
{
   REAL h1,h2,h3,h4,h5,h6,h7,h8;    // The 8 independent elements of the homography matrix
                                    // (h9 is set to 1)

   // Default constructor
   HomographyParams_(REAL _h1=1, REAL _h2=0, REAL _h3=0,
                     REAL _h4=0, REAL _h5=1, REAL _h6=0,
                     REAL _h7=0, REAL _h8=0)
      : h1(_h1), h2(_h2), h3(_h3), 
        h4(_h4), h5(_h5), h6(_h6),
        h7(_h7), h8(_h8) {}

   // Construct from another type of HomographyParams_
   template <class REAL2>
   HomographyParams_(const HomographyParams_<REAL2>& other)
      : h1(other.h1), h2(other.h2), h3(other.h3),
        h4(other.h4), h5(other.h5), h6(other.h6),
        h7(other.h7), h8(other.h8) {}

   // Construct from a homography matrix (last element cannot be zero)
   template<class REAL2>
   HomographyParams_(const REAL2* matrixData)
      : h1(matrixData[0]), h2(matrixData[1]), h3(matrixData[2]),
        h4(matrixData[3]), h5(matrixData[4]), h6(matrixData[5]),
        h7(matrixData[6]), h8(matrixData[7])
   {
      const REAL h9_inv = 1 / (REAL)matrixData[8];
      h1 *= h9_inv;
      h2 *= h9_inv;
      h3 *= h9_inv;
      h4 *= h9_inv;
      h5 *= h9_inv;
      h6 *= h9_inv;
      h7 *= h9_inv;
      h8 *= h9_inv;
   }

   // Define a homography matrix from the parameters
   template <class REAL2>
   inline void toMatrix(REAL2* matrixData) const
   {
      matrixData[0] = (REAL2)h1;
   	matrixData[1] = (REAL2)h2;
   	matrixData[2] = (REAL2)h3;
   	matrixData[3] = (REAL2)h4;
   	matrixData[4] = (REAL2)h5;
   	matrixData[5] = (REAL2)h6;
   	matrixData[6] = (REAL2)h7;
   	matrixData[7] = (REAL2)h8;
      matrixData[8] = 1;
   }

#ifndef _OPENCV_TYPES
   // Create a homography matrix from the parameters
   template <class REAL2>
   inline void toMatrix(ICP_Mat<REAL2>& matrix) const
   {
      	matrix.Create(3,3);
      	toMatrix(matrix.ReturnPtr());
   }
#endif
};


template<class REAL>
class Homography_
{
public:

   typedef REAL Real;
   typedef HomographyParams_<REAL> Params;
#ifdef _OPENCV_TYPES
   typedef cv::Point_<REAL> Point;
#else
   typedef ICP_Point2<REAL> Point;
#endif
   
   // Default constructor
   Homography_(){}

   // Construct from a set of homography parameters
   Homography_(const Params& params) : mParams(params) {}

   // Construct from another homography of any type
   template<class REAL2>
   Homography_(const Homography_<REAL2>& other) : mParams(other.getParams()) {}

   // Homographic mapping
   inline void map(const Point& src, Point& dst) const
   {
      // Multiply homographic matrix and homogeneous 2D point
      REAL x = mParams.h1*src.x + mParams.h2*src.y + mParams.h3;
      REAL y = mParams.h4*src.x + mParams.h5*src.y + mParams.h6;
      REAL z_inv = 1 / ( mParams.h7*src.x + mParams.h8*src.y + 1 );     // h9 = 1

      // Perspective division
      dst.x = x*z_inv;
      dst.y = y*z_inv;
   }

   // Invert w.r.t. a set of homography parameters
   void invert(const Params& params) {invert(params, mParams);}

   // Invert w.r.t. to another homography
   void invert(const Homography_<REAL>& other) {invert(other.mParams, mParams);}

   // Set parameters
   void setParams(const Params& params){mParams = params;}

   // Get parameters
   const Params& getParams() const {return mParams;}

   // Invert homography parameters
   static inline void invert(const Params& src, Params& dst)
   {
      REAL s = 1 / ( src.h1 * src.h5 - src.h2 * src.h4 );
      dst.h1 = s * ( src.h5 - src.h6 * src.h8 );
      dst.h2 = s * ( src.h3 * src.h8 - src.h2 );
      dst.h3 = s * ( src.h2 * src.h6 - src.h3 * src.h5 );
      dst.h4 = s * ( src.h6 * src.h7 - src.h4 );
      dst.h5 = s * ( src.h1 - src.h3 * src.h7 );
      dst.h6 = s * ( src.h3 * src.h4 - src.h1 * src.h6 );
      dst.h7 = s * ( src.h4 * src.h8 - src.h5 * src.h7 );
      dst.h8 = s * ( src.h2 * src.h7 - src.h1 * src.h8 );
   }

protected:
   Params mParams;
};

#endif   /* HOMOGRAPHY_H */
