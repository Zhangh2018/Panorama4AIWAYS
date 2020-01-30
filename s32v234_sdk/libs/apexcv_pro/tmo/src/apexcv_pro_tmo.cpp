/**************************************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2014-2017 NXP
 * All Rights Reserved
 *
 *****************************************************************************
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
 **************************************************************************************************/


#include "apexcv_pro_tmo.h"
#include <oal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// GENERATED ACF PROCESS
#include <TMO_CALC_GEO_MEAN_RGBE.hpp>
#include <TMO_CALC_GEO_MEAN_OPENEXR.hpp>
#include <TMO_LDR_TRANSFORM.hpp>


#define SHOW_ERROR(expr) if(!(expr)) { printf("%s : %s [%d]\n", __FILE__, __PRETTY_FUNCTION__, __LINE__); }
//#define SHOW_ERROR(expr)

static const uint16_t gsDivTableData[] = {
      4,  0, 5461, 12, 6553, 13, 4681, 13, 7281, 14, 5957, 14, 5041, 14, 4369, 14,
   3855, 14, 3449, 14, 6241, 15, 2849, 14, 2621, 14, 2427, 14, 4519, 15, 1057, 13,
   7943, 16, 7489, 16, 1771, 14, 6721, 16, 6393, 16,  381, 12, 5825, 16, 5577, 16,
   5349, 16, 1285, 14, 2473, 15, 2383, 15, 4599, 16, 4443, 16, 4297, 16, 4161, 16,
   8065, 17, 7825, 17, 3799, 16,  923, 14, 3591, 16, 3495, 16,  851, 14, 1659, 15,
    809, 14, 1579, 15,  771, 14, 3013, 16, 2945, 16, 5761, 17, 5637, 17, 2759, 16,
   5405, 17, 5295, 17, 2595, 16, 2545, 16, 4993, 17, 4899, 17, 4809, 17, 4723, 17,
   4639, 17, 4559, 17, 4481, 17, 4405, 17, 1083, 15, 2131, 16, 2097, 16,  129, 12,
    127, 12, 2001, 16, 1971, 16, 7767, 18, 7653, 18, 7543, 18, 1859, 16, 1833, 16,
   7231, 18, 7133, 18, 7037, 18,  217, 13, 6853, 18, 6765, 18, 3339, 17, 3297, 17,
    407, 14,  201, 13, 6355, 18, 3139, 17, 1551, 16, 1533, 16, 6061, 18, 5991, 18,
   1481, 16, 5857, 18, 5793, 18, 5729, 18, 5667, 18, 5607, 18, 1387, 16, 5489, 18,
   5433, 18, 5377, 18, 2661, 17, 5269, 18,  163, 13, 5165, 18, 5115, 18, 5065, 18,
   5017, 18, 4969, 18, 2461, 17, 4877, 18,  151, 13, 1197, 16,  593, 15, 2351, 17,
   1165, 16, 4619, 18, 2289, 17, 4539, 18, 1125, 16, 2231, 17,  553, 15, 4387, 18,
   2175, 17, 4315, 18, 4279, 18, 4245, 18, 4211, 18, 4177, 18,  259, 14,  257, 14,
    255, 14, 8097, 19, 8035, 19, 7973, 19, 7913, 19, 3927, 18, 1949, 17, 3869, 18,
   7681, 19, 3813, 18, 3785, 18, 1879, 17, 7463, 19, 3705, 18, 3679, 18, 7307, 19,
    907, 16, 3603, 18, 7157, 19, 1777, 17, 7061, 19, 7013, 19, 6967, 19, 6921, 19,
   6875, 19, 6831, 19, 3393, 18, 6743, 19, 1675, 17, 6657, 19, 6615, 19, 3287, 18,
   6533, 19, 1623, 17, 1613, 17, 6413, 19, 3187, 18, 6335, 19, 6297, 19, 1565, 17,
   6223, 19, 3093, 18, 3075, 18, 3057, 18, 3039, 18, 6043, 19, 6009, 19, 2987, 18,
   1485, 17, 5907, 19, 2937, 18, 5841, 19, 5809, 19, 5777, 19, 5745, 19, 2857, 18,
   5683, 19, 1413, 17, 2811, 18,  699, 16, 2781, 18, 5533, 19,   43, 12, 5475, 19,
   5447, 19, 2709, 18, 5391, 19, 5363, 19,  667, 16, 5309, 19, 2641, 18,  657, 16,
   5229, 19, 5203, 19, 2589, 18,  161, 14, 5127, 19, 2551, 18, 5077, 19, 5053, 19,
   5029, 19, 5005, 19, 4981, 19, 4957, 19, 2467, 18, 4911, 19,  611, 16, 4865, 19,
   4843, 19, 4821, 19, 2399, 18, 4777, 19, 4755, 19, 4733, 19,  589, 16, 4691, 19,
   2335, 18, 2325, 18, 4629, 19, 4609, 19, 1147, 17,  571, 16, 4549, 19, 4529, 19,
   2255, 18, 2245, 18, 4471, 19, 1113, 17, 4433, 19, 4415, 19, 1099, 17, 2189, 18,
   4359, 19, 4341, 19, 1081, 17, 2153, 18,   67, 13, 4271, 19, 4253, 19, 1059, 17,
   4219, 19, 2101, 18, 4185, 19, 4169, 19,  519, 16,  517, 16,  515, 16,  513, 16
};



namespace apexcv
{
Tmo::Tmo()
   :  mIsInitialized(false),
      mHdrImageFormat(HDR_IMAGE_FORMAT_INVALID),
      mApexId(0),
      mProcessGeoMeanRgbe(nullptr),
      mProcessGeoMeanExr(nullptr),
      mProcessLdrTransform(nullptr)
      // vsdk::SUMat mDivValue;     /*!< Pointer to the buffer for division value. */
      // vsdk::SUMat mIntermExp;    /*!< Pointer to the buffer for intermediate exponent. */
      // vsdk::SUMat mIntermMan;    /*!< Pointer to the buffer for intermediate mantissa. */
      // vsdk::SUMat mWorldExp;     /*!< Pointer to the buffer for world luminance exponent. */
      // vsdk::SUMat mWorldMan;     /*!< Pointer to the buffer for world luminance mantissa. */
      // vsdk::SUMat mGeoMeanExp;   /*!< Pointer to the buffer for geometric mean exponent. */
      // vsdk::SUMat mGeoMeanMan;   /*!< Pointer to the buffer for geometric mean mantissa. */
      // TMO_CALC_GEO_MEAN_RGBE    mProcessGeoMeanRgbe;
      // TMO_CALC_GEO_MEAN_OPENEXR mProcessGeoMeanExr;
      // TMO_LDR_TRANSFORM    mProcessLdrTransform;
{
}



Tmo::~Tmo()
{
   Release();
}



void Tmo::Release()
{
   if (nullptr != mProcessGeoMeanRgbe)
   {
      delete (TMO_CALC_GEO_MEAN_RGBE*) mProcessGeoMeanRgbe;
   }
   if (nullptr != mProcessGeoMeanExr)
   {
      delete (TMO_CALC_GEO_MEAN_OPENEXR*) mProcessGeoMeanExr;
   }
   if (nullptr != mProcessLdrTransform)
   {
      delete (TMO_LDR_TRANSFORM*) mProcessLdrTransform;
   }

   mProcessGeoMeanRgbe  = nullptr;
   mProcessGeoMeanExr   = nullptr;
   mProcessLdrTransform = nullptr; 
}


APEXCV_LIB_RESULT Tmo::Initialize(vsdk::SUMat&            arHdrImage,       /*!< Input image. Datatype must correspond to "acHdrImageFormat" */
                                  HDR_IMAGE_FORMAT const acHdrImageFormat, /*!< Input image format. "arHdrImage" datatype must correspond to "acHdrImageFormat" */
                                  vsdk::SUMat&            arLdrImage,       /*!< Output image. Datatype is 08U. e0 size is (3, 1). */
                                  vsdk::SUMat&            arLdrTransformKey /*!< Key value used during LDR transform. Datatype is 08U. */
                                 )
{
   APEXCV_LIB_RESULT lLibResult = APEXCV_ERROR_FAILURE;

   if (mIsInitialized == false &&
       (acHdrImageFormat == HDR_IMAGE_FORMAT_RGBE ||
        acHdrImageFormat == HDR_IMAGE_FORMAT_OPENEXR))
   {

      TMO_CALC_GEO_MEAN_RGBE* lProcessRgbe = new TMO_CALC_GEO_MEAN_RGBE;
      if (nullptr == lProcessRgbe)
      {
         return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mProcessGeoMeanRgbe = lProcessRgbe;

      TMO_CALC_GEO_MEAN_OPENEXR* lProcessExr = new TMO_CALC_GEO_MEAN_OPENEXR;
      if (nullptr == lProcessExr)
      {
         return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mProcessGeoMeanExr = lProcessExr;

      TMO_LDR_TRANSFORM* lProcessLdr = new TMO_LDR_TRANSFORM;
      if (nullptr == lProcessLdr)
      {
         return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mProcessLdrTransform = lProcessLdr;      
      
      const int32_t lcHdrImageType       = (acHdrImageFormat == HDR_IMAGE_FORMAT_RGBE) ?
                                            VSDK_CV_8UC4 : // HDR_IMAGE_FORMAT_RGBE
                                            VSDK_CV_16UC3; // HDR_IMAGE_FORMAT_OPENEXR
      const int32_t lcLdrTransformKeyType = VSDK_CV_8UC1;

      if (arHdrImage.type()        == lcHdrImageType &&
          arLdrTransformKey.type() == lcLdrTransformKeyType)
      {
         const int32_t lcSrcWidth  = arHdrImage.cols;
         const int32_t lcSrcHeight = arHdrImage.rows;

         // allocate intermediate data buffer
         mDivValue   = vsdk::SUMat(          1,          1, VSDK_CV_16UC2);
         mIntermExp  = vsdk::SUMat(lcSrcHeight, lcSrcWidth, VSDK_CV_8UC3);
         mIntermMan  = vsdk::SUMat(lcSrcHeight, lcSrcWidth, VSDK_CV_8UC3);
         mWorldExp   = vsdk::SUMat(lcSrcHeight, lcSrcWidth, VSDK_CV_8UC1);
         mWorldMan   = vsdk::SUMat(lcSrcHeight, lcSrcWidth, VSDK_CV_8UC1);
         mGeoMeanExp = vsdk::SUMat(          1,          1, VSDK_CV_8UC1);
         mGeoMeanMan = vsdk::SUMat(          1,          1, VSDK_CV_8UC1);

         if (!mDivValue.empty()   &&
             !mIntermExp.empty()  &&
             !mIntermMan.empty()  &&
             !mWorldExp.empty()   &&
             !mWorldMan.empty()   &&
             !mGeoMeanExp.empty() &&
             !mGeoMeanMan.empty())
         {
            int32_t lIntResult = 0; // Must set to 0 because of all the "|="

            // initialize processes and connect ports to buffers
            if(acHdrImageFormat == HDR_IMAGE_FORMAT_RGBE)
            {
               lIntResult |= lProcessRgbe->Initialize();                                SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessRgbe->ConnectIO(           "INPUT", arHdrImage);   SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessRgbe->ConnectIO("INTERMEDIATE_EXP", mIntermExp);   SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessRgbe->ConnectIO("INTERMEDIATE_MAN", mIntermMan);   SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessRgbe->ConnectIO(       "WORLD_EXP", mWorldExp);    SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessRgbe->ConnectIO(       "WORLD_MAN", mWorldMan);    SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessRgbe->ConnectIO(    "GEO_MEAN_EXP", mGeoMeanExp);  SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessRgbe->ConnectIO(    "GEO_MEAN_MAN", mGeoMeanMan);  SHOW_ERROR(lIntResult == 0);
               {
                  ApexcvHostBaseBaseClass::InfoCluster lInfo;
                  lInfo.set_ACF("TMO_CALC_GEO_MEAN_RGBE", lProcessRgbe);
                  lInfo.push_PortName("INPUT");
                  mvInfoClusters.push_back(lInfo);
               }
            }
            else
            {
               lIntResult |= lProcessExr->Initialize();                                SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessExr->ConnectIO(           "INPUT", arHdrImage);   SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessExr->ConnectIO("INTERMEDIATE_EXP", mIntermExp);   SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessExr->ConnectIO("INTERMEDIATE_MAN", mIntermMan);   SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessExr->ConnectIO(       "WORLD_EXP", mWorldExp);    SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessExr->ConnectIO(       "WORLD_MAN", mWorldMan);    SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessExr->ConnectIO(    "GEO_MEAN_EXP", mGeoMeanExp);  SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessExr->ConnectIO(    "GEO_MEAN_MAN", mGeoMeanMan);  SHOW_ERROR(lIntResult == 0);
               {
                  ApexcvHostBaseBaseClass::InfoCluster lInfo;
                  lInfo.set_ACF("TMO_CALC_GEO_MEAN_OPENEXR", lProcessExr);
                  lInfo.push_PortName("INPUT");
                  mvInfoClusters.push_back(lInfo);
               }
            }

            lIntResult |= lProcessLdr->Initialize();
            lIntResult |= lProcessLdr->ConnectIO("INTERMEDIATE_EXP", mIntermExp);        SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO("INTERMEDIATE_MAN", mIntermMan);        SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO(       "WORLD_EXP", mWorldExp);         SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO(       "WORLD_MAN", mWorldMan);         SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO(    "GEO_MEAN_EXP", mGeoMeanExp);       SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO(       "DIV_VALUE", mDivValue);         SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO(       "KEY_VALUE", arLdrTransformKey); SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO(          "OUTPUT", arLdrImage);        SHOW_ERROR(lIntResult == 0);
            {
               ApexcvHostBaseBaseClass::InfoCluster lInfo;
               lInfo.set_ACF("TMO_LDR_TRANSFORM", lProcessLdr);
               lInfo.push_PortName("OUTPUT");
               mvInfoClusters.push_back(lInfo);
            }

            if (lIntResult == 0)
            {
               mHdrImageFormat = acHdrImageFormat;
               mIsInitialized = true;
               lLibResult = APEXCV_SUCCESS;
            }
            else
            {
               SHOW_ERROR(false);
               lLibResult = APEXCV_ERROR_INTERNAL_ERROR;
            }

            if (lIntResult == 0)
            {
               if(acHdrImageFormat == HDR_IMAGE_FORMAT_RGBE)
               {
                  lIntResult |= lProcessRgbe->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, mApexId);
               }
               else
               {
                  lIntResult |= lProcessExr->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, mApexId);
               }

               lIntResult |= lProcessLdr->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, mApexId);

               if (lIntResult != 0)
               {
                  SHOW_ERROR(false);
                  lLibResult = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
               }
            }
         }
         else
         {
            SHOW_ERROR(false);
            lLibResult = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
         }
      }
      else
      {
         SHOW_ERROR(false);
         lLibResult = APEXCV_ERROR_INVALID_ARGUMENT;
      }
   }
   else if (mIsInitialized == true)
   {
      SHOW_ERROR(false);
      lLibResult = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
   }
   else if (!(acHdrImageFormat == HDR_IMAGE_FORMAT_RGBE ||
              acHdrImageFormat == HDR_IMAGE_FORMAT_OPENEXR))
   {
      lLibResult = APEXCV_ERROR_INVALID_ARGUMENT;
   }


   return lLibResult;
}


APEXCV_LIB_RESULT Tmo::ReconnectIO(vsdk::SUMat&           arHdrImage,       /*!< Input image. Datatype must correspond to "acHdrImageFormat" */
                                   HDR_IMAGE_FORMAT const acHdrImageFormat, /*!< Input image format. "arHdrImage" datatype must correspond to "acHdrImageFormat" */
                                   vsdk::SUMat&            arLdrImage,       /*!< Output image. Datatype is 08U. e0 size is (3, 1). */
                                   vsdk::SUMat&            arLdrTransformKey /*!< Key value used during LDR transform. Datatype is 08U. */
                                  )
{
   APEXCV_LIB_RESULT lLibResult = APEXCV_ERROR_FAILURE;

   TMO_CALC_GEO_MEAN_RGBE*    lProcessRgbe = (TMO_CALC_GEO_MEAN_RGBE*) mProcessGeoMeanRgbe;
   TMO_CALC_GEO_MEAN_OPENEXR* lProcessExr  = (TMO_CALC_GEO_MEAN_OPENEXR*) mProcessGeoMeanExr;
   TMO_LDR_TRANSFORM*         lProcessLdr  = (TMO_LDR_TRANSFORM*) mProcessLdrTransform;

   if (mIsInitialized == true &&
       (acHdrImageFormat == HDR_IMAGE_FORMAT_RGBE ||
        acHdrImageFormat == HDR_IMAGE_FORMAT_OPENEXR))
   {
      const int32_t lcHdrImageType       = (acHdrImageFormat == HDR_IMAGE_FORMAT_RGBE) ?
                                            VSDK_CV_8UC4 : // HDR_IMAGE_FORMAT_RGBE
                                            VSDK_CV_16UC3; // HDR_IMAGE_FORMAT_OPENEXR
      const int32_t lcLdrTransformKeyType = VSDK_CV_8UC1;

      if (arHdrImage.type()        == lcHdrImageType &&
          arLdrTransformKey.type() == lcLdrTransformKeyType)
      {
         const int32_t lcSrcWidth  = arHdrImage.cols;
         const int32_t lcSrcHeight = arHdrImage.rows;

         // allocate intermediate data buffer
         mIntermExp  = vsdk::SUMat(lcSrcHeight, lcSrcWidth, VSDK_CV_8UC3);
         mIntermMan  = vsdk::SUMat(lcSrcHeight, lcSrcWidth, VSDK_CV_8UC3);
         mWorldExp   = vsdk::SUMat(lcSrcHeight, lcSrcWidth, VSDK_CV_8UC1);
         mWorldMan   = vsdk::SUMat(lcSrcHeight, lcSrcWidth, VSDK_CV_8UC1);

         if (!mIntermExp.empty()  &&
             !mIntermMan.empty()  &&
             !mWorldExp.empty()   &&
             !mWorldMan.empty())
         {
            int32_t lIntResult = 0; // Must set to 0 because of all the "|="

            // initialize processes and connect ports to buffers
            if (acHdrImageFormat == HDR_IMAGE_FORMAT_RGBE)
            {
               lIntResult |= lProcessRgbe->ConnectIO(           "INPUT", arHdrImage); SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessRgbe->ConnectIO("INTERMEDIATE_EXP", mIntermExp); SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessRgbe->ConnectIO("INTERMEDIATE_MAN", mIntermMan); SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessRgbe->ConnectIO(       "WORLD_EXP", mWorldExp);  SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessRgbe->ConnectIO(       "WORLD_MAN", mWorldMan);  SHOW_ERROR(lIntResult == 0);
            }
            else
            {
               lIntResult |= lProcessExr->ConnectIO(           "INPUT", arHdrImage); SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessExr->ConnectIO("INTERMEDIATE_EXP", mIntermExp); SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessExr->ConnectIO("INTERMEDIATE_MAN", mIntermMan); SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessExr->ConnectIO(       "WORLD_EXP", mWorldExp);  SHOW_ERROR(lIntResult == 0);
               lIntResult |= lProcessExr->ConnectIO(       "WORLD_MAN", mWorldMan);  SHOW_ERROR(lIntResult == 0);
            }

            lIntResult |= lProcessLdr->ConnectIO("INTERMEDIATE_EXP", mIntermExp);        SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO("INTERMEDIATE_MAN", mIntermMan);        SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO(       "WORLD_EXP", mWorldExp);         SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO(       "WORLD_MAN", mWorldMan);         SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO(    "GEO_MEAN_EXP", mGeoMeanExp);       SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO(       "KEY_VALUE", arLdrTransformKey); SHOW_ERROR(lIntResult == 0);
            lIntResult |= lProcessLdr->ConnectIO(          "OUTPUT", arLdrImage);        SHOW_ERROR(lIntResult == 0);

            if (lIntResult == 0)
            {
               mHdrImageFormat = acHdrImageFormat;
               lLibResult = APEXCV_SUCCESS;
            }
            else
            {
               SHOW_ERROR(false);
               lLibResult = APEXCV_ERROR_INTERNAL_ERROR;
            }

            if (lIntResult == 0)
            {
               if(acHdrImageFormat == HDR_IMAGE_FORMAT_RGBE)
               {
                  lIntResult |= lProcessRgbe->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, mApexId);
               }
               else
               {
                  lIntResult |= lProcessExr->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, mApexId);
               }

               lIntResult |= lProcessLdr->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, mApexId);

               if (lIntResult != 0)
               {
                  SHOW_ERROR(false);
                  lLibResult = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
               }
            }
         }
         else
         {
            SHOW_ERROR(false);
            lLibResult = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
         }
      }
      else
      {
         SHOW_ERROR(false);
         lLibResult = APEXCV_ERROR_INVALID_ARGUMENT;
      }
   }
   else if (mIsInitialized == false)
   {
      SHOW_ERROR(false);
      lLibResult = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }
   else if (!(acHdrImageFormat == HDR_IMAGE_FORMAT_RGBE ||
              acHdrImageFormat == HDR_IMAGE_FORMAT_OPENEXR))
   {
      lLibResult = APEXCV_ERROR_INVALID_ARGUMENT;
   }

   return lLibResult;
}



APEXCV_LIB_RESULT Tmo::Process(void)
{
   APEXCV_LIB_RESULT lLibResult = APEXCV_ERROR_FAILURE;

   if (mIsInitialized == true &&
       (mHdrImageFormat == HDR_IMAGE_FORMAT_RGBE ||
        mHdrImageFormat == HDR_IMAGE_FORMAT_OPENEXR))
   {
      int32_t lIntResult = 0; // Must set to 0 because of all the "|="

      TMO_CALC_GEO_MEAN_RGBE*    lProcessRgbe = (TMO_CALC_GEO_MEAN_RGBE*) mProcessGeoMeanRgbe;
      TMO_CALC_GEO_MEAN_OPENEXR* lProcessExr  = (TMO_CALC_GEO_MEAN_OPENEXR*) mProcessGeoMeanExr;
      TMO_LDR_TRANSFORM*         lProcessLdr  = (TMO_LDR_TRANSFORM*) mProcessLdrTransform;

      if (mHdrImageFormat == HDR_IMAGE_FORMAT_RGBE)
      {
         lIntResult |= lProcessRgbe->Start(); SHOW_ERROR(lIntResult == 0);
         lIntResult |= lProcessRgbe->Wait();  SHOW_ERROR(lIntResult == 0);
      }
      else
      {
         lIntResult |= lProcessExr->Start(); SHOW_ERROR(lIntResult == 0);
         lIntResult |= lProcessExr->Wait();  SHOW_ERROR(lIntResult == 0);
      }

      if (lIntResult == 0)
      {
         // New scope create to force the SUMat/SMat destructors to be called
         {
            vsdk::SMat lGeoMeanManMat = mGeoMeanMan.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
            vsdk::SMat lDivValueMat   = mDivValue.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

            // division value is determined by geometric mean mantissa calculated in the previous process
            uint8_t*  pGeoMeanMan = (uint8_t*)lGeoMeanManMat.data;
            uint16_t* pDivValue   = (uint16_t*)lDivValueMat.data;

            pDivValue[0] = gsDivTableData[pGeoMeanMan[0] * 2];
            pDivValue[1] = gsDivTableData[pGeoMeanMan[0] * 2 + 1];
         }

         lIntResult |= lProcessLdr->Start(); SHOW_ERROR(lIntResult == 0);
         lIntResult |= lProcessLdr->Wait();  SHOW_ERROR(lIntResult == 0);
      }

      if (lIntResult == 0)
      {
         lLibResult = APEXCV_SUCCESS;
      }
      else
      {
         SHOW_ERROR(false);
         lLibResult = APEXCV_ERROR_INTERNAL_ERROR;
      }
   }
   else if (mIsInitialized == false)
   {
      SHOW_ERROR(false);
      lLibResult = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }
   else if (!(mHdrImageFormat == HDR_IMAGE_FORMAT_RGBE ||
              mHdrImageFormat == HDR_IMAGE_FORMAT_OPENEXR))
   {
      lLibResult = APEXCV_ERROR_INTERNAL_ERROR;
   }

   return lLibResult;
}



/************************************
 * Select APEX to run
 ***********************************/
APEXCV_LIB_RESULT Tmo::SelectApexCore(const int32_t acApexId)
{
   TMO_CALC_GEO_MEAN_RGBE*    lProcessRgbe = (TMO_CALC_GEO_MEAN_RGBE*) mProcessGeoMeanRgbe;
   TMO_CALC_GEO_MEAN_OPENEXR* lProcessExr  = (TMO_CALC_GEO_MEAN_OPENEXR*) mProcessGeoMeanExr;
   TMO_LDR_TRANSFORM*         lProcessLdr  = (TMO_LDR_TRANSFORM*) mProcessLdrTransform;

   if (mIsInitialized == false)
   {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   if (mHdrImageFormat == HDR_IMAGE_FORMAT_RGBE)
   {
      if (lProcessRgbe->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, acApexId) ||
          lProcessLdr->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, acApexId))
      {
         return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      }
   }
   else
   {
      if (lProcessExr->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, acApexId)  ||
          lProcessLdr->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, acApexId))
      {
         return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      }
   }

   mApexId = acApexId;
   return APEXCV_SUCCESS;
} 

} // namespace apecv 

