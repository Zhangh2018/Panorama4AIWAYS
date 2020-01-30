/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2016-2017 NXP
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
****************************************************************************/

#include "apexcv_pro_aggcf_pd.h"
#define __APEXCV_PROFILE__
#include "swt_arm_report.h"

namespace apexcv{


vsdk::SUMat *AggCFDetector::InitPyramidBuf( )
{

    vsdk::SUMat *outputPY = new vsdk::SUMat[scalePars.size()];

    int32_t bin             = mdOpts.shrink;
    int32_t numChannels     = mdOpts.nOrients + 4;
    int total_mem = 0;

    for (uint32_t i = 0; i < scalePars.size(); i ++ )
    {
        int32_t resH = scalePars[i].height;
        int32_t resW = scalePars[i].width;
        
        int32_t resWStride = (resW/bin+127)/128*128;

        vsdk::SUMat output = vsdk::SUMat(resH/bin*numChannels+64, resWStride+64, VSDK_CV_16UC1);
	vsdk::Rect lRect(32, 32, resW/bin, resH/bin*numChannels);
        outputPY[i] =  vsdk::SUMat(output, lRect); 

        total_mem += resWStride/bin*resH/bin*numChannels*sizeof(uint16_t);

    }

    return outputPY;
}

void AggCFDetector::DeInitPyramidBuf( vsdk::SUMat *&apOutPY )
{
    delete[] apOutPY;
}

vsdk::SUMat *AggCFDetector::createImage_desc_helper(char *apInputName, int aWidth, int aHeight)
{
    vsdk::SUMat *In_desp = new vsdk::SUMat(aHeight, aWidth, VSDK_CV_32SC1);
    vsdk::SMat tempMat = In_desp->getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    int32_t *inBuf = (int32_t *)tempMat.data;
  
    FILE *fp = NULL;
    fp = fopen(apInputName, "rb");
    if( fp != NULL)
    {
        fread(inBuf, sizeof(int32_t)*aWidth*aHeight, 1, fp);
        fclose(fp);
	OAL_MemoryFlushAndInvalidate( inBuf, aWidth*aHeight*sizeof(uint32_t));
        return In_desp;
    }
    else
    {
        delete In_desp;
        printf(" test RGB data not existi %s \n", apInputName);
        return NULL;
    }

}

// calculate one octave feature data in feature pyramid
int AggCFDetector::CalcChannelsOctave( vsdk::SUMat &aDstLUV, vsdk::SUMat *apOutputPY, uint32_t aRealScaleIdx)
{
    int ret = 0;
    uint32_t nPerOct = mdOpts.nPerOct;
    uint32_t scaleLen = scalePars.size();

    for(uint32_t i = aRealScaleIdx; i < std::min(scaleLen, aRealScaleIdx+nPerOct); i++)
    {
        if( i == aRealScaleIdx)
        {
            ret |= CalcChannelsRealScale( aDstLUV, apOutputPY, i, mApexID);    
        }
        else
        {
            ret |= CalcChannelsApproxScale(apOutputPY, i, mApexID); 
        }
    }

    ret |= ApplyChannelsRealScaleFiltering(apOutputPY, aRealScaleIdx, mApexID);

    return ret;
}

void dumpdata_desc( vsdk::SUMat &aDesc, std::string aName, int aNum, int aOffset, int aTotal )
{
     int width  = aDesc.cols;
     int height = aDesc.rows/aTotal;
     int stride = aDesc.step[0]/sizeof(uint16_t);
     uint16_t *temp = new uint16_t[width*height];

     FILE *fp = NULL;
     for(int i = 0; i < aNum; i++)
     {
         vsdk::SMat matTemp = aDesc.getMat(vsdk::ACCESS_RW | OAL_USAGE_NONCACHED);
	 uint16_t *inVaddr = (uint16_t *)matTemp.data;

         std::ostringstream ss, sw, sh;
         ss<<i;
         sw<<width;
         sh<<height;
         std::string filename = "./dump/" + aName + "." + ss.str() + "wh" + sw.str() + sh.str() + ".dat";
         uint16_t *dst = inVaddr + (i+aOffset)*stride*height;

         for(int j = 0; j < height; j++)
         {
             for(int k = 0; k < width; k++)
             {
                temp[j*width +k] = dst[j*stride+k]; 
             } 
         }
        
         fp = fopen(filename.c_str(), "wb"); 
         if(fp != NULL)
         {
             fwrite(temp, sizeof(uint16_t)*width*height, 1, fp);
             fclose(fp);
         }

     }

     delete[] temp;
}


// calcuate one real scale feature data 
int AggCFDetector::CalcChannelsRealScale( vsdk::SUMat &aDstLUV, vsdk::SUMat *apOutputPY, int32_t aScaleIdx, int aApexID)
{
    int ret = 0;
    int bin = mdOpts.shrink; 
    int softbin = mdOpts.softBin;
    int full = mdOpts.full;
    int32_t numOrientations = mdOpts.nOrients;
    int width  = aDstLUV.cols;
    int height = aDstLUV.rows/3;
    int resH = scalePars[aScaleIdx].height;
    int resW = scalePars[aScaleIdx].width;
    int stride = (resW+127)/128*128;

    // LUV + magnitude filtered + magnitude + orientation
    vsdk::SUMat dstTempMat = vsdk::SUMat(resH*6+64, stride+64, VSDK_CV_16UC1 );
    vsdk::Rect lRect(32, 32, resW, resH*6);
    vsdk::SUMat dstTemp =  vsdk::SUMat(dstTempMat, lRect);
 
    // resize LUV to scale size
    // resize starting from source at offset 0, output at offset 0, 3 images on scale ratio 1
    // resize from width/height to resW/resH 
    if (( resW == (width>>1) ) && (resH == (height>>1)) )
    {
       ret |= AGGCFScaleDown2Calc(aDstLUV, 0, width, height, dstTemp, 0, width/2, height/2, 3, aApexID); 
    }
    else if (( resW == (width>>2) ) && (resH == (height>>2)) )
    {
       ret |= AGGCFScaleDown4Calc(aDstLUV, 0, width, height, dstTemp, 0, width/4, height/4, 3, aApexID); 
    }
    else 
    {
       ret |= AGGCFResize16CalcCon(aDstLUV, 0, width, height, dstTemp, 0, resW, resH, 3, (1U<<16), aApexID);
    }

    // calculate magnitude and orientation
    // dst_temp 0/1/2 L/U/V, 4/5 M/O
    ret |= AGGCFMagOriCalc( dstTemp, resW, resH, aApexID);

    // low pass filtering magnitude
    // dst_temp filtered_m source dst_temp offset 4 to dst_temp offset 3, filter on 1 image [1 2 1] filtering
    uint32_t fxnormConst = (uint32_t)(mdOpts.normConst*pow(2,14));

    ret |= AGGCFfltCalc( dstTemp, 4, resW, resH, dstTemp, 3, resW, resH, 1, 2, fxnormConst, aApexID);

    // hist calculation
    ret |= AGGCFhistCalc( dstTemp, resW, resH, apOutputPY[aScaleIdx], resW/bin, resH/bin, bin, softbin, full, numOrientations, aApexID);

    // resize LUV and filtered magnitude based on bin setting
    if( 2 == bin)
    {

       ret |= AGGCFScaleDown2Calc( dstTemp, 0, resW, resH, apOutputPY[aScaleIdx], 0, resW/2, resH/2, 4, aApexID);
    }
    else if (4 == bin)
    {
       ret |= AGGCFScaleDown4Calc( dstTemp, 0, resW, resH, apOutputPY[aScaleIdx], 0, resW/2, resH/2, 4, aApexID);
    } 
    else
    {
       ret |= AGGCFResize16CalcCon( dstTemp, 0, resW, resH, apOutputPY[aScaleIdx], 0, resW/bin, resH/bin, 4, (1U<<16), aApexID);
    } 
    return ret;
}

// calculate one approximate scale feature data
int AggCFDetector::CalcChannelsApproxScale(vsdk::SUMat *apOutputPY, int32_t aScaleIdx, int aApexID)
{
    int ret = 0;
    int32_t bin = mdOpts.shrink;
    int realScaleIndex = scalePars[ aScaleIdx ].realScaleIndex;
    int resH = scalePars[aScaleIdx].height/bin;
    int resW = scalePars[aScaleIdx].width/bin;
    int realScaleH = scalePars[realScaleIndex].height/bin;
    int realScaleW = scalePars[realScaleIndex].width/bin;

    ret |= AGGCFResize16fltCalcCon( apOutputPY[realScaleIndex], 0, realScaleW, realScaleH, apOutputPY[aScaleIdx], 0, resW, resH, 10,  (1u<<16), aApexID);

    return ret;
}

// apply low pass filtering to real scale feature once all approximate scale finished calculation
int AggCFDetector::ApplyChannelsRealScaleFiltering(vsdk::SUMat *apOutputPY, int32_t aScaleIdx, int aApexID)
{
    int ret = 0;
    uint32_t numChannels     = mdOpts.nOrients + 4;
    vsdk::SUMat scale = apOutputPY[aScaleIdx];
    int32_t width  = scale.cols;
    int32_t height = scale.rows;
    int32_t spanSize = scale.step[0]/sizeof(uint16_t);

    vsdk::SUMat tempUMat = vsdk::SUMat(height, spanSize, VSDK_CV_16UC1 );
    vsdk::Rect lRect(0, 0, width, height);
    vsdk::SUMat tempUMatROI =  vsdk::SUMat(tempUMat, lRect);

    ret |= AGGCFfltCalcCon(apOutputPY[aScaleIdx], 0, width, height/numChannels, tempUMatROI, 0, width, height/numChannels, numChannels, 2, 0, aApexID);
    apOutputPY[aScaleIdx] = tempUMatROI;

    return ret;
}

int AggCFDetector::CalcChannelsPyramid( vsdk::SUMat &aDstLUV, vsdk::SUMat *apOutputPY )
{
    int ret = 0;

    for (uint32_t i = 0; i < scalePars.size(); i += mdOpts.nApprox + 1)
    {
       ret |=  CalcChannelsOctave(aDstLUV, apOutputPY, i);  
    }
    return ret;
} 

int AggCFDetector::CalcChannelsPyramid( vsdk::SUMat &aDstLUV, vsdk::SUMat *apOutputPY, int aApexID)
{
    int ret = 0;

    for (uint32_t i = 0; i < scalePars.size(); i += mdOpts.nApprox + 1)
    {
        ret |= CalcChannelsOctave(aDstLUV, apOutputPY, i, aApexID);
    }

    return ret;
}

int AggCFDetector::CalcChannelsOctave( vsdk::SUMat &aDstLUV, vsdk::SUMat *apOutputPY, uint32_t aRealScaleIdx, int aApexID)
{
    int ret = 0;
    uint32_t nPerOct = mdOpts.nPerOct;
    uint32_t scale_len = scalePars.size();

    for(uint32_t i = aRealScaleIdx; i < std::min(scale_len, aRealScaleIdx+nPerOct); i++)
    {
        if( i == aRealScaleIdx)
        {
            ret |= CalcChannelsRealScale( aDstLUV, apOutputPY, i, aApexID);
        }
        else
        {
            ret |= CalcChannelsApproxScale(apOutputPY, i, aApexID);
        }
    }

    ret |= ApplyChannelsRealScaleFiltering(apOutputPY, aRealScaleIdx, aApexID);

    return ret;

}

}



