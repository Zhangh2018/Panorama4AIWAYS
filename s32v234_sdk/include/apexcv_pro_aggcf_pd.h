/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2016-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP 
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

#ifndef APEXCVPROAGGCFPD_H
#define APEXCVPROAGGCFPD_H

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "float.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <iterator>
#include "math.h"

#include <apexcv_error_codes.hpp> 
#include "apexcv_host_base_class.h"
#include "apexcv_pro_util.h"
#include <sumat.hpp>
#include <icp_data.h>
#include <oal.h>


namespace apexcv{


// Bounding Box Structure [x y w h score]
struct bbs
{
   int x, y, w, h;
   float score;
};


// detector paramters
struct opts
{
   // detector is trained with setting of follwing modelIDs and modelIDsPad are equal
   // so no extra padding needed
   float*   modelDs;           // model height+width without padding (eg [100 41])
   float*   modelDsPad;        // model height + width with padding (eg[128 64])
   int      stride;            // spatial stride between detection windows
   float    cascThr;           // constant cascade threshold(affects speed / accuracy)
   float    cascCal;           // cascade calibration(affects speed / accuracy)
   int      nPerOct;           // number of scales per octave
   int      nOctUp;            // number of upsampled octaves to compute
   int      nApprox;           // number of approx.scales(if - 1 nApprox = nPerOct - 1)
   float*   lambdas;           // coefficients for power law scaling(see BMVC10)
   int*     pad;               // amount to pad channels(along T / B and L / R)
   float*   minDs;             // minimum image size for channel computation
   float    smooth;            // radius for channel smoothing(using convTri)
   int      shrink;            // Channel subsampling
   int      normRad;           // Normalizing Radius
   float    normConst;         // Normalizing Constant
   bool     full;              // range of orientation [0 PI] or [0 2*PI]  
   int      nOrients;          // Number of gradient orientations to compute
   int      softBin;           // control histogram calculation method (nearest neighborhood, bilinear, trilinear interpolation)      
   double   overlap;           // non-maximal suppression overlap
   bool     luvSmooth;         // flag to control if apply low pass filtering to LUV 
   uint32_t *fxlambdas;        // Fixed Point conversion of lambda to calculation approximate scales
   uint32_t fxnormConst;       // Fixed Point conversion of normconst to apply magnitude normizlation
};

typedef struct scale_par
{
    int width;
    int height;
    int isRealscale;    // 1: real scale, 0:approximate scale
    int realScaleIndex; // realScale index used to calculate approxmiate scales
    float scaleRatio;   // scale ratio used to resize bounding box to correct size
 
    // image span       // to track the change of image span, update FidsArray if changed
    int spanSize;

}scalePar, *pScalePar;


/*!*********************************************************************************
* \brief apexcv::aggcf_detector is the host-ACF interface for aggregated channel 
* feature (aggCF) based pedestrian detection.
*
* This class provides interfaces to load pre-trained detector model, calculate aggregated channel  
* features including LUV, magnitude and histogram of gradient, then perform pedestrian
* detection and output bounding boxes of detected pedestrians  
* 
***********************************************************************************/
class AggCFDetector
{
public:

/***********************************************************************************
*  \brief Default constructor.
*  Allocate resources for aggCF.
***********************************************************************************/
   AggCFDetector();

/***********************************************************************************
*  \brief Destructor.
*   Release resources for aggCF.
***********************************************************************************/
   virtual ~AggCFDetector();

/*!*********************************************************************************
*  \brief aggCF load pre-trained detector model
***********************************************************************************/
   void InitDetectorModel(const char*);

   void InitDetectorModelBuf( char *apDetectorBuf, int aBufSize);

/*!*********************************************************************************
*  \brief aggCF based on input image size and detector model (octaves/scales/bin)
*  calculate size of each scale
***********************************************************************************/
   void CalcScaleParameters(int aInWidth, int aInHight);

/*!*********************************************************************************
*  \brief aggCF calculate feature pyramid on default APEX  
***********************************************************************************/
   int CalcChannelsPyramid(vsdk::SUMat &aDstLUV, vsdk::SUMat *apOutputPy);

/*!*********************************************************************************
*  \brief aggCF calculate feature pyramid and assign the task to specified APEX
***********************************************************************************/
   int CalcChannelsPyramid(vsdk::SUMat &aDstLUV, vsdk::SUMat *apOutputPy, int aApexID);

/*!*********************************************************************************
*  \brief aggCF perform pedestrian detection on extracted feature pyramid using
*  detector model and return bounding boxes of detected pedestrians 
***********************************************************************************/
   std::vector<apexcv::bbs> ApplyPedDetectionDET(vsdk::SUMat *apChannelFeatures);

/*!*********************************************************************************
*  \brief aggCF display loaded detector model parameters 
***********************************************************************************/
   void ShowDetectorParameters();

/*!*********************************************************************************
*  \brief aggCF perform non maximum suppression (NMS) on bounding boxes
***********************************************************************************/
   void ApplyPedDetectionNMS(std::vector<apexcv::bbs> &aBbs, int aGreedy );

/*!*********************************************************************************
*  \brief aggCF init feature pyramid structure and allocate feature data buffer
***********************************************************************************/
   vsdk::SUMat *InitPyramidBuf( );

/*!*********************************************************************************
*  \brief aggCF release feature data buffer
***********************************************************************************/
   void DeInitPyramidBuf( vsdk::SUMat *&apOutPy );

/// @private
   vsdk::SUMat *createImage_desc_helper(char *apInput_name, int aWidth, int aHeight);

/*!*********************************************************************************
*  \brief aggCF calculate one feature octave and assign the task to specified APEX.
*  using this function, APP has full control on how to calculate feature pyramid including
*  the order of each octave calculation within the pyramid, the assignment of octave calculationi task
*  to different APU to achieve optimal performance.
***********************************************************************************/
   int CalcChannelsOctave( vsdk::SUMat &aDstLUV, vsdk::SUMat *apOutputPy, uint32_t aRealScaleIdx, int aApexID);

/*!*********************************************************************************
*  \brief aggCF calculate feature octaves with APU using specified octave and APEX array.
*  APEX code will decide how to dispatch octave calculation tasks to different APEX
*  for performance and load balance purpose. 
***********************************************************************************/
   int CalcChannelsOctave( vsdk::SUMat &aDstLUV, vsdk::SUMat *apOutputPy, std::vector<uint32_t> &aRealScaleIdx, std::vector<uint32_t> &aApexID);

   //AcfProfilingInfo retProfilingInfo();
/// @private   
   APEXCV_LIB_RESULT retPyramidScalePars( std::vector<scalePar> & aPar )  
   {
      APEXCV_LIB_RESULT rval = APEXCV_SUCCESS;
      aPar = scalePars;

      if( true == this->mfailed )
      {
         rval = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
      }
      return rval; 
   }
/// @private   
   APEXCV_LIB_RESULT retShrinkparameter(int &aShrinkParam) 
   { 
      APEXCV_LIB_RESULT rval = APEXCV_SUCCESS;
      aShrinkParam = mdOpts.shrink;
      
      if( true == this->mfailed )
      {
         rval = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
      }    
       
      return rval;
   }

/*!*********************************************************************************
*  \brief aggCF return the flag to show if a detector model is loaded successfully 
***********************************************************************************/
   bool IsDetectorModelFailToLoaded() const { return mfailed; }

/// @private
   APEXCV_LIB_RESULT ConvertRGBtoLUV(vsdk::SUMat &aImageData, vsdk::SUMat &aDstLUV, int aWidth, int aHeight, int aApexID);

private:
   
   // init feature index array 
   void initFidsArray();
   
   // update feature index array if feature images width/stride is changed
   void updateScaleFidsArray(uint32_t aScaleindex, uint32_t aScaleSpan);
  
   // calculate one octave of feature pyramid, private member function called by aggcf_channelsPyramid
   int CalcChannelsOctave( vsdk::SUMat &aDst_luv, vsdk::SUMat *apOutput_py, uint32_t aRealScaleIdx);

   // calculate real scale
   int CalcChannelsRealScale( vsdk::SUMat &aDst_luv, vsdk::SUMat *apOutput_py, int32_t aScaleIdx, int aApexID);
   
   // calculate approximate scale
   int CalcChannelsApproxScale(vsdk::SUMat *apOutput_py, int32_t aScaleIdx, int aApexID);
   
   // apply low pass filtering to real scale 
   int ApplyChannelsRealScaleFiltering(vsdk::SUMat *apOutput_py, int32_t aScaleIdx, int aApexID);

   // kernel calculator
   int32_t      mApexID;
    
   void *mAggcfMagoriApexP[2];
   void *mAggcfFlt3x3ApexP[2];
   void *mAggcfLUVApexP[2];
   void *mAggcfHistApexP[2];
   void *mAggcfScaleDown8P[2];
   void *mAggcfScaleDown4P[2];
   void *mAggcfScaleDown2P[2];

   
   // channel pyramid parameters
   std::vector<scalePar> scalePars; // scale parameters
   uint32_t *mpFids;                   // feature index array, each node of decision tree has one feature index
                                     // use to locate feature value in channel feature data
   double   *mpThrs;                   // decision tree threshold (decision stomp to decide go left or right along decision tree)
   uint32_t *mpFixThrs;                // converted fixed point threshold
   uint32_t *mpChild;                  // decision tree child index
   double   *mpHs;                     // log ratio at each node to accumulate and compare with constant cascade threshold
   int      mTreeDepth;               // decision tree depth
   int      mnTreeNodes;              // number of nodes in decision tree (define by fixed or maximum tree depth setting)
   int      mnTrees;                  // number of decision trees
   opts     mdOpts;                   // aggcf detection parameters
   bool     mfailed;                  // flag to indicate if detector model is loaded successfully
   uint32_t **mpChannelIds;            // pre-calculated relative offset of feature index to each searching windows
   uint32_t mNumFeatures;            // number of features


   APEXCV_LIB_RESULT AGGCFResize16CalcCon( vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH, vsdk::SUMat &aOut, int aOutOffset, int aOutW, int aOutH, int aNum, uint32_t aRatio, int aApexID);

   APEXCV_LIB_RESULT AGGCFMagOriCalc(vsdk::SUMat &aIn, int aInW, int aInH, int aApexID);

   APEXCV_LIB_RESULT AGGCFfltCalc( vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH,
                       vsdk::SUMat &aOut, int aOutOffset, int aOutW, int aOutH, int aNum, uint32_t aP, uint32_t aNormConst, int aApexID);

   APEXCV_LIB_RESULT AGGCFfltCalcCon(vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH,
                       vsdk::SUMat &aOut, int aOutOffset, int aOutW, int aOutH, int aNum, uint32_t aP, uint32_t aNormConst, int aApexID);

   APEXCV_LIB_RESULT AGGCFhistCalc(vsdk::SUMat &aIn, int aInW, int aInH,
                       vsdk::SUMat &aOut, int aOutW, int aOutH, int aBin, int aSoftbin, int aFull, int aNum, int aApexID);

   APEXCV_LIB_RESULT AGGCFResize16fltCalcCon( vsdk::SUMat &aIn, int aInoffset, int aInw, int aInh, vsdk::SUMat &aOut, int aOutoffset, int aOutw, int aOuth, int aNum, uint32_t aRatio, int aApexID );
   
   APEXCV_LIB_RESULT AGGCFScaleDown8Calc( vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH,
                        vsdk::SUMat &aOut, int aOutOffset, int aOutW, int aOutH, int aNum, int aApexID );

   APEXCV_LIB_RESULT AGGCFScaleDown4Calc( vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH,
                        vsdk::SUMat &aOut, int aOutOffset, int aOutW, int aOutH, int aNum, int aApexID );

   APEXCV_LIB_RESULT AGGCFScaleDown2Calc( vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH,
                        vsdk::SUMat &aOut, int aOutOffset, int aOutW, int aOutH, int aNum, int aApexID );



};


}

#endif /* APEXCVPROAGGCFPD_H */

