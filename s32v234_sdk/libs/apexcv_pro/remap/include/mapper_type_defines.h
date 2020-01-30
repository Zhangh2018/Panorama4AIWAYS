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

#ifndef CGVTYPES_H
#define CGVTYPES_H

//
#include "icp_types.h"
#include "icp_fixed.hpp"
#include "mapper.h"
#include "param_loader.h"

//
typedef float                    Real;
typedef ICP_Point2<Real>      Point2;
typedef ICP_Point3<Real>      Point3;
typedef ICP_Mat<Real>         Matrix;
typedef ICP_Rectangle_<Real>  Rect;
typedef ICP_Fixed<18>            Fixed;
typedef ICP_Point2<Fixed>     FixedPoint2;
typedef ICP_Point3<Fixed>     FixedPoint3;
typedef ICP_Mat<Fixed>        FixedMat;

// Mapper types
typedef Real                                          MapperReal;
typedef CameraMapper_<MapperReal>                     CameraMapper;
typedef PolynomialRationalDistortMapper_<MapperReal>  DistortMapper;
typedef Homography_<MapperReal>                       Homography;
typedef DewarpMapper_<MapperReal, DistortMapper>      DewarpMapper;
typedef PerspectiveMapper_<MapperReal, DistortMapper> PerspectiveMapper;
typedef ParamLoader_<MapperReal>                      ParamLoader;
//typedef DewarpParamTool_<MapperReal, DistortMapper>   DewarpParamTool;

#endif /* CGVTYPES_H */
