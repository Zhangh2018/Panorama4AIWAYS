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

#ifndef PARAMLOADER_H
#define PARAMLOADER_H

#include "icp_types.h"
#include "mapper.h"
#include "polynomial_rational_distort_mapper.h"
#include "oal.h"
#include <stdlib.h>
#include "iniparser.h"
#include "t32term.h"

enum
{
  LOCAL_FILE = 0,
  REMOTE_FILE
};



/*
#define PRINT_FLOAT(A)\
{\
   char buffer[64];\
   floatToString((float)(A), buffer, 64, 8);\
   printf(#A);\
   printf(" = %s\n", buffer);\
}
 */


typedef unsigned int FILE_LOCATION_TYPE;

template <class REAL>
class ParamLoader_
{
public:
  typedef REAL                 Real;
  typedef ICP_Rectangle_<Real> Rect;
  typedef ICP_Size             Size;
  typedef CameraParams_<Real>                      CameraParams;
  typedef PolynomialRationalParams_<Real>          DistortParams;
  typedef RotationAngles_<Real>                    RotationAngles;
  typedef SplitAngles_<Real>                       SplitAngles;
  typedef PolynomialRationalDistortMapper_<Real>   DistortMapper;
  typedef DewarpMapper_<Real, DistortMapper>       DewarpMapper;
  typedef PerspectiveMapper_<Real, DistortMapper>  PerspectiveMapper;
  typedef HomographyParams_<Real>                  HomographyParams;
  typedef Homography_<Real>                        Homography;

  // Intrinsic Parameters
  CameraParams  cameraParams;
  DistortParams distortParams;

  // Extrinsic Parameters
  RotationAngles rotationAngles;
  Real           cameraHeight;

  // Dewarp Parameters
  CameraParams  dewarpCameraParams;
  Size          dewarpImageSize;

  // Topview Parameters
  Rect    topGroundROI;
  Size    topImageSize;

  // Splitview Parameters
  SplitAngles splitViewAngles;

  ParamLoader_() {}

  ParamLoader_(const char* filename, FILE_LOCATION_TYPE fileLoc)
  {
    load(filename, fileLoc);
  }

  int load(const char* filename, FILE_LOCATION_TYPE /*fileLoc*/)
  {
    int lRetVal = 9999;
    uint32_t fileSizeInBytes = 0;
    char* pBufferInChar = 0;
    {
#ifdef __STANDALONE__
      int pFile = T32_fopen(filename, T32_TERM_O_RDONLY);

      if (pFile != 0)
      {
        T32_fseek(pFile, 0, T32_TERM_SEEK_END);
        fileSizeInBytes = T32_ftell(pFile);
        //pBufferInChar = (char*)OAL_MemoryAllocFlag(fileSizeInBytes, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
        pBufferInChar = (char*)malloc(fileSizeInBytes);
        T32_fseek(pFile, 0, T32_TERM_SEEK_SET);
        T32_fread((void*)pBufferInChar, fileSizeInBytes, 1, pFile);
        T32_fclose(pFile);
      }
      else
      {
#if DEBUG_PRINT
        printf("ERROR: Could not open %s    in %s (%d)\n", filename, __FILE__, __LINE__);
#endif
        lRetVal = 1;
        goto EXIT;
    }
#else  // linux
      FILE* pFile = fopen(filename, "r");

      if (nullptr != pFile)
      {
        fseek(pFile, 0, SEEK_END);
        fileSizeInBytes = ftell(pFile);
        //pBufferInChar = (char*)OAL_MemoryAllocFlag(fileSizeInBytes, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
        pBufferInChar = (char*)malloc(fileSizeInBytes);
        fseek(pFile, 0, SEEK_SET);
        fread((void*)pBufferInChar, fileSizeInBytes, 1, pFile);
        fclose(pFile);
        //printf("open %s\n", filename);
      }
      else
      {
#if DEBUG_PRINT
        printf("ERROR: Could not open %s    in %s (%d)\n", filename, __FILE__, __LINE__);
#endif
        lRetVal = 1;
        goto EXIT;
      }
#endif
    }
    load(pBufferInChar);

    if (nullptr != pBufferInChar)
    {
      free(pBufferInChar);
    }

    lRetVal = 0;// return success
  EXIT:
    return lRetVal;
  }

  void load(const char* pBufferInChar)
  {
    if (nullptr != pBufferInChar)
    {
      char* pCurrentChar = (char*)pBufferInChar;
      const double cDEG2RAD = 1.745329251994329576923690768489E-2;

      //char* end;

      // *** Intrinsic Camera Parameters *****************************

      // *** Camera Parameters ***
      cameraParams.fx = std::strtod(pCurrentChar, &pCurrentChar);
      cameraParams.cx = std::strtod(pCurrentChar, &pCurrentChar);
      cameraParams.fy = std::strtod(pCurrentChar, &pCurrentChar);
      cameraParams.cy = std::strtod(pCurrentChar, &pCurrentChar);

      //             PRINT_FLOAT(cameraParams.fx);
      //             PRINT_FLOAT(cameraParams.cx);
      //             PRINT_FLOAT(cameraParams.fy);
      //             PRINT_FLOAT(cameraParams.cy);

      // *** Distortion Parameters ***
      distortParams.p1 = std::strtod(pCurrentChar, &pCurrentChar);
      distortParams.p2 = std::strtod(pCurrentChar, &pCurrentChar);
      distortParams.k1 = std::strtod(pCurrentChar, &pCurrentChar);
      distortParams.k2 = std::strtod(pCurrentChar, &pCurrentChar);
      distortParams.k3 = std::strtod(pCurrentChar, &pCurrentChar);
      distortParams.k4 = std::strtod(pCurrentChar, &pCurrentChar);
      distortParams.k5 = std::strtod(pCurrentChar, &pCurrentChar);
      distortParams.k6 = std::strtod(pCurrentChar, &pCurrentChar);

      //             PRINT_FLOAT(distortParams.k1);
      //             PRINT_FLOAT(distortParams.k2);
      //             PRINT_FLOAT(distortParams.k3);
      //             PRINT_FLOAT(distortParams.k4);
      //             PRINT_FLOAT(distortParams.k5);
      //             PRINT_FLOAT(distortParams.k6);
      //             PRINT_FLOAT(distortParams.p1);
      //             PRINT_FLOAT(distortParams.p2);

      // *** Extrinsic Camera Parameters *****************************

      // *** Vehicle Coordinate Rotation Angles ***
      // Angles in file are in degrees, so we must convert to radians
      rotationAngles.tilt = std::strtod(pCurrentChar, &pCurrentChar) * cDEG2RAD;
      rotationAngles.swing = std::strtod(pCurrentChar, &pCurrentChar) * cDEG2RAD;
      rotationAngles.pan = std::strtod(pCurrentChar, &pCurrentChar) * cDEG2RAD;

      //             PRINT_FLOAT(rotationAngles.tilt);
      //             PRINT_FLOAT(rotationAngles.swing);
      //             PRINT_FLOAT(rotationAngles.pan);

      // *** Camera Height ***
      cameraHeight = std::strtod(pCurrentChar, &pCurrentChar);

      //             PRINT_FLOAT(cameraHeight);


      // *** Dewarped Image Parameters *******************************

      // *** Dewarped Camera Parameters ***
      dewarpCameraParams.fx = std::strtod(pCurrentChar, &pCurrentChar);
      dewarpCameraParams.cx = std::strtod(pCurrentChar, &pCurrentChar);
      dewarpCameraParams.fy = std::strtod(pCurrentChar, &pCurrentChar);
      dewarpCameraParams.cy = std::strtod(pCurrentChar, &pCurrentChar);

      // *** Dewarped Image Size ***
      dewarpImageSize.width = strtoul(pCurrentChar, &pCurrentChar, 10);
      dewarpImageSize.height = strtoul(pCurrentChar, &pCurrentChar, 10);


      // *** TopView Image Parameters *******************************

      // *** TopView Ground ROI ***
      topGroundROI.x = std::strtod(pCurrentChar, &pCurrentChar);
      topGroundROI.y = std::strtod(pCurrentChar, &pCurrentChar);
      topGroundROI.width = std::strtod(pCurrentChar, &pCurrentChar);
      topGroundROI.height = std::strtod(pCurrentChar, &pCurrentChar);

      // *** TopView Image Size ***
      topImageSize.width = std::strtoul(pCurrentChar, &pCurrentChar, 10);
      topImageSize.height = std::strtoul(pCurrentChar, &pCurrentChar, 10);

      //printf("topview: (%d,%d)\n", topImageSize.width, topImageSize.height);

      // *** SplitView Image Parameters *******************************

      // *** SplitView Angles ***
      // Angles in file are in degrees, so we must convert to radians
      splitViewAngles.theta1 = std::strtod(pCurrentChar, &pCurrentChar) * cDEG2RAD;
      splitViewAngles.theta2 = std::strtod(pCurrentChar, &pCurrentChar) * cDEG2RAD;
      splitViewAngles.phi1 = std::strtod(pCurrentChar, &pCurrentChar) * cDEG2RAD;
      splitViewAngles.phi2 = std::strtod(pCurrentChar, &pCurrentChar) * cDEG2RAD;

      //PRINT_FLOAT(-splitViewAngles.phi2);
    }
  }

  //void configureDewarpMapper(DewarpMapper& dewarpMapper)
  //{
  //   dewarpMapper.setParams(cameraParams, distortParams, dewarpCameraParams);
  //}

  void configureTopMapper(PerspectiveMapper& topMapper)
  {
    // The rotation of the vehicle coordinate system w.r.t. the camera is generated from the three camera angles (tilt, swing, pan)
    ICP_Mat<Real> R;
    createRotationMatrix(R, rotationAngles);

    // Configure the top mapper
    ::configureTopMapper(topMapper, cameraParams, distortParams, R.ReturnPtr(), cameraHeight, topGroundROI, topImageSize);
  }

  void configureEBCMappers(DewarpMapper& dewarpMapper,
    PerspectiveMapper& topMapper,
    PerspectiveMapper& leftMapper,
    PerspectiveMapper& rightMapper,
    Homography& groundToDewarp,
    PerspectiveMapper& groundToWarp) const
  {
    // The rotation of the vehicle coordinate system w.r.t. the camera is generated from the three camera angles (tilt, swing, pan)
    ICP_Mat<Real> R;
    createRotationMatrix(R, rotationAngles);

    // 1. Configure the dewarp mapper
    dewarpMapper.setParams(cameraParams, distortParams, dewarpCameraParams);

    // 2. Configure the top mapper
    ::configureTopMapper(topMapper, cameraParams, distortParams, /*R.ReturnPtr()*/ rotationAngles, cameraHeight, topGroundROI, topImageSize);

    // 3. Configure the split mappers
    Size sideImageSize(dewarpImageSize.width / 2, dewarpImageSize.height);
    configureSplitMappers(leftMapper, rightMapper, cameraParams, distortParams, R.ReturnPtr(), splitViewAngles, sideImageSize);

    // 4. Configure the ground-to-dewarped-image homography
    const CameraParams identityParams(1, 0, 1, 0);
    HomographyParams homographyParams;
    configureHomographyParams(homographyParams, dewarpCameraParams, R.ReturnPtr(), cameraHeight, identityParams);
    groundToDewarp.setParams(homographyParams);

    // 5. Configure the ground-to-warped-image perspective mapper
    configureHomographyParams(homographyParams, identityParams, R.ReturnPtr(), cameraHeight, identityParams);
    groundToWarp.setParams(cameraParams, distortParams, homographyParams);
  }
};

#endif /* PARAMLOADER_H */
