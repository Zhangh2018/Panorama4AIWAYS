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

#ifndef POLYNOMIALRATIONALDISTORTMAPPER_H
#define POLYNOMIALRATIONALDISTORTMAPPER_H


#ifdef _OPENCV_TYPES
#include "core/core.hpp"
#else
#include "icp_types.h"
#endif

template <class REAL>
struct PolynomialRationalParams_
{
   REAL p1,p2,                // tangential distortion coefficients
        k1,k2,k3,k4,k5,k6;    // radial distortion coefficients

   // Default constructor
   PolynomialRationalParams_(REAL _p1=0, REAL _p2=0, REAL _k1=0, REAL _k2=0, REAL _k3=0,
                                                     REAL _k4=0, REAL _k5=0, REAL _k6=0)
      : p1(_p1), p2(_p2), k1(_k1), k2(_k2), k3(_k3), k4(_k4), k5(_k5), k6(_k6)
   {
   }

   // Construct from another type of PolynomialRationalParams_
   template <class REAL2>
   PolynomialRationalParams_(const PolynomialRationalParams_<REAL2>& other)
      : p1(other.p1), p2(other.p2), k1(other.k1), k2(other.k2), k3(other.k3),
                                    k4(other.k4), k5(other.k5), k6(other.k6)
   {
   }

   // Construct from an array of any type.  Array order: {p1, p2, k1, k2, k3, k4, k5, k6}
   template<class REAL2>
   PolynomialRationalParams_(const REAL2* ptr)
      : p1(ptr[0]), p2(ptr[1]), k1(ptr[2]), k2(ptr[3]), k3(ptr[4]),
                                k4(ptr[5]), k5(ptr[6]), k6(ptr[7])
   {
   }
};

template<class REAL>
class PolynomialRationalDistortMapper_
{
public:
   typedef REAL Real;
   typedef PolynomialRationalParams_<REAL> Params;
#ifdef _OPENCV_TYPES
   typedef cv::Point_<REAL> Point;
#else
   typedef ICP_Point2<REAL> Point;
#endif
   
   // Default constructor
   PolynomialRationalDistortMapper_(){}

   // Construct from a set of polynomial-rational distortion parameterse
   PolynomialRationalDistortMapper_(const Params& params) : mParams(params) {}

   // Construct from another PolynomialRationalDistortMapper_ of any type
   template<class REAL2>
   PolynomialRationalDistortMapper_(const PolynomialRationalDistortMapper_<REAL2>& other)
      : mParams(other.getParams()) {}

   // Construct from an array of any type.  Array order: {p1, p2, k1, k2, k3, k4, k5, k6}
   template<class REAL2>
   PolynomialRationalDistortMapper_(const REAL2* ptr) : mParams(ptr) {}

   // Distortion mapping in ideal image plane coordinates (NOT PIXEL COORDINATES)
   inline void map(const Point& src, Point& dst) const
   {
      // this function maps an undistorted point (src) to its corresponding distorted point (dst)
      // src and dst are NOT in pixel coordinates.  They are in ideal image plane coordinates
      // that are independent of any camera matrix.

      REAL x2   = src.x*src.x;
      REAL y2   = src.y*src.y;
      REAL r2   = x2 + y2;
      REAL _2xy = 2*src.x*src.y;
      REAL kr   = (1 + ((mParams.k3*r2 + mParams.k2)*r2 + mParams.k1)*r2) / 
                  (1 + ((mParams.k6*r2 + mParams.k5)*r2 + mParams.k4)*r2);

      dst.x = src.x*kr + mParams.p1*_2xy + mParams.p2*(r2 + 2*x2);
      dst.y = src.y*kr + mParams.p1*(r2 + 2*y2) + mParams.p2*_2xy;
   }

   // Iterative inverse distortion mapping (undistortion)   
   void map_inv(const Point& src, Point& dst) const
   {
      // This function uses the Newton-Raphson method to map the disorted
      // point (src) to the undistorted point (dst)

      REAL j1,j2,j3,j4;      // elements of the Jacobian matrix
      REAL res;              // residual value in distorted image pixels
      REAL det_inv;          // inverse determinant of the Jacobian
      int count=0;           // iteration counter

      // Use the source point as the initial guess
      Point tmp = src;
      Point f;

      // Start Newton-Raphson Method
      do
      {
         // Get the mapped point
         map(tmp, f);
        
         f.x -= src.x;
         f.y -= src.y;

         // Get the Jacobian
         // J = | j1  j2 |
         //     | j3  j4 |
         getJacobian(j1, j2, j3, j4, tmp);

         // Get the next point: Pnext = P - Jacobian_inv * F
         det_inv = 1. / (j1*j4 - j2*j3);
         tmp.x += det_inv * (j2 * f.y - j4 * f.x);
         tmp.y += det_inv * (j3 * f.x - j1 * f.y);

         // Determine the residual (in distorted image coordinates)
         res = f.x*f.x + f.y*f.y;
      }
      while (res > minInverseResidual && ++count < maxInverseIterations);
    
      dst.x = tmp.x;
      dst.y = tmp.y;
   }



   // Set parameters
   void setParams(const Params& params){mParams = params;}

   // Get parameters
   const Params& getParams() const {return mParams;}

   static const REAL minInverseResidual;
   static const int maxInverseIterations = 30;

protected:

   // Get the Jacobian (j1, j2, j3, j4) for the point p
   inline void getJacobian(REAL& j1, REAL& j2, REAL& j3, REAL& j4, const Point& p) const
   {
      REAL r   = p.x*p.x + p.y*p.y;
      REAL r2  = r*r;
      REAL r3  = r2*r;
      REAL _2x = 2*p.x; 
      REAL _2y = 2*p.y;

      REAL G    = mParams.k1*r + mParams.k2*r2  + mParams.k3*r3   + 1;
      REAL G_dr = mParams.k1   + mParams.k2*2*r + mParams.k3*3*r2;
      REAL H    = mParams.k4*r + mParams.k5*r2  + mParams.k6*r3   + 1;
      REAL H_dr = mParams.k4   + mParams.k5*2*r + mParams.k6*3*r2;
      
      REAL kr = G/H;
      REAL kr_dr = (G_dr*H - G*H_dr) / (H*H);

      j1 = p.x*(6*mParams.p2 + kr_dr*_2x) + mParams.p1*_2y + kr;    // j1 = du/dx
      j2 = mParams.p1*_2x + mParams.p2*_2y;                         // j2 = du/dy
      j3 = j2;                                  // j3 = dv/dx  (and dv/dx = du/dy)
      j4 = p.y*(6*mParams.p1 + kr_dr*_2y) + mParams.p2*_2x + kr;    // j4 = dv/dy
   }

   // Distortion Parameters
   Params mParams;
};

template<class REAL>
const REAL PolynomialRationalDistortMapper_<REAL>::minInverseResidual = 1.0E-9;

#endif   /* POLYNOMIALRATIONALDISTORTMAPPER_H */
