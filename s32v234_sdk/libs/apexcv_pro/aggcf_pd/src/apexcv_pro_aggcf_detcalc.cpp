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
#include "apexcv_pro_aggcf.h"
#include "apexcv_pro_resize.h"

namespace apexcv{

APEXCV_LIB_RESULT AggCFDetector::ConvertRGBtoLUV(vsdk::SUMat &aImageData, vsdk::SUMat &aDstluv, int aWidth, int aHeight, int aApexID)
{
    int ApexIndex = aApexID;
    if( aApexID < 0 || aApexID > 1)
    {
        printf( "LUV: invalid apex assignment, use default apexID %d \n", mApexID );
        ApexIndex = mApexID;       
    }
    AGGCFluv *pProcess = (AGGCFluv *)mAggcfLUVApexP[ApexIndex];
    if(nullptr == pProcess)
    {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
   
    APEXCV_LIB_RESULT ret = APEXCV_SUCCESS;

    aDstluv = vsdk::SUMat(aHeight*3, aWidth, VSDK_CV_16UC1);
    vsdk::Rect lRect1(0, 0, aWidth, aHeight);
    vsdk::SUMat outLMat = vsdk::SUMat( aDstluv, lRect1);

    vsdk::Rect lRect2(0, aHeight, aWidth, aHeight);
    vsdk::SUMat outUMat = vsdk::SUMat( aDstluv, lRect2);
   
    vsdk::Rect lRect3(0, aHeight*2, aWidth, aHeight);
    vsdk::SUMat outVMat = vsdk::SUMat( aDstluv, lRect3);

    ret = pProcess->ReconnectIO( aImageData, outLMat, outUMat, outVMat );

    if(APEXCV_SUCCESS == ret)
    {
        ret = pProcess->Process();
    }
    else
    {
        printf("Unable to init aggcf_luv, error ocde %d \n", ret );
    } 

    if ( APEXCV_SUCCESS != ret)
    {
        printf("Unable to finish aggcf_luv calculation %d \n", ret );
    } 

    return ret; 

}


APEXCV_LIB_RESULT AggCFDetector::AGGCFResize16CalcCon( vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH, vsdk::SUMat &aOut, int aOutOffset, int aOutW, int aOutH, int aNum, uint32_t , int aApexID )
{
    int ApexIndex = aApexID;
    if( aApexID < 0 || aApexID > 1)
    {
        printf( "Resize16con: invalid apex assignment, use default apexID %d \n", mApexID );
        ApexIndex = mApexID;
    }

    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

    uint32_t inStride  = (uint32_t)aIn.step[0]/sizeof(uint16_t);
    uint32_t outStride = (uint32_t)aOut.step[0]/sizeof(uint16_t);

    if ( aInW == aOutW && aInH == aOutH )
    {
        vsdk::SMat inMat  = aIn.getMat(vsdk::ACCESS_RW|OAL_USAGE_CACHED);
        vsdk::SMat outMat = aOut.getMat(vsdk::ACCESS_RW|OAL_USAGE_CACHED);

        uint16_t *inVaddr  = (uint16_t *)inMat.data + aInOffset*aInH*inStride;
        uint16_t *outVaddr = (uint16_t *)outMat.data + aOutOffset*aOutH*outStride;

        uint16_t *outVaddrTemp = outVaddr;

        for(int j = 0; j < aInH*aNum; j++)
        {
            memcpy( outVaddrTemp, inVaddr, aInW*sizeof(uint16_t) );
            inVaddr  += inStride;
            outVaddrTemp += outStride;
        }
        return lRetVal;
    }
    Resize aggcfResize;
    aggcfResize.SelectApuConfiguration( ACF_APU_CFG__DEFAULT, ApexIndex );

    //int aligned_span = (aOutW + 0xf)&~0xf;
    //vsdk::SUMat tempMat = vsdk::SUMat(aOutH*aNum, aligned_span, CV_16UC1);
    vsdk::Rect oRect(0, aOutOffset*aOutH, aOutW, aOutH*aNum);
    vsdk::SUMat outMat = vsdk::SUMat( aOut, oRect);
    
    vsdk::Rect iRect(0, aInH*aInOffset, aInW, aInH*aNum);
    vsdk::SUMat inMat = vsdk::SUMat( aIn, iRect );
    lRetVal = aggcfResize.Initialize(inMat, outMat);
   
    if( APEXCV_SUCCESS == lRetVal)
    { 
        lRetVal = aggcfResize.Process( );
    }
    return lRetVal;
}


APEXCV_LIB_RESULT AggCFDetector::AGGCFResize16fltCalcCon( vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH, vsdk::SUMat &aOut, int aOutOffset, int aOutW, int aOutH, int aNum, uint32_t , int aApexID )
{
    int ApexIndex = aApexID;
    if( aApexID < 0 || aApexID > 1)
    {
        printf( " Resize16fltcon: invalid apex assignment, use default apexID %d \n", mApexID );
        ApexIndex = mApexID;
    }

    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

    apexcv::AGGCFResizeFlt aggcfResizeflt;
    aggcfResizeflt.SelectApuConfiguration( ACF_APU_CFG__DEFAULT, ApexIndex );

    vsdk::Rect lRect(0, aOutOffset*aOutH, aOutW, aOutH*aNum);
    vsdk::SUMat outT = vsdk::SUMat( aOut, lRect);

    vsdk::Rect iRect(0, aInH*aInOffset, aInW, aInH*aNum);
    vsdk::SUMat inT = vsdk::SUMat( aIn, iRect );
    lRetVal = aggcfResizeflt.Initialize(inT, outT, 2);

    if ( APEXCV_SUCCESS == lRetVal)
    {
       lRetVal = aggcfResizeflt.Process( inT, outT );
    }
    return lRetVal;
}


APEXCV_LIB_RESULT AggCFDetector::AGGCFMagOriCalc(vsdk::SUMat &aIn, int aInW, int aInH, int aApexID )
{
    int ApexIndex = aApexID;
    if( aApexID < 0 || aApexID > 1)
    {
        printf( " Magori: invalid apex assignment, use default apexID %d \n", mApexID );
        ApexIndex = mApexID;
    }

    AGGCFMagOri *pProcess = (AGGCFMagOri *)mAggcfMagoriApexP[ApexIndex];
    if(nullptr == pProcess)
    {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    APEXCV_LIB_RESULT ret = APEXCV_SUCCESS;

    vsdk::Rect lRectL(0, 0, aInW, aInH);
    vsdk::SUMat inLMat = vsdk::SUMat( aIn, lRectL);

    vsdk::Rect lRectU(0, aInH, aInW, aInH);
    vsdk::SUMat inUMat = vsdk::SUMat( aIn, lRectU);

    vsdk::Rect lRectV(0, aInH*2, aInW, aInH);
    vsdk::SUMat inVMat = vsdk::SUMat( aIn, lRectV);

    vsdk::Rect lRectM(0, aInH*4, aInW, aInH);
    vsdk::SUMat outMMat = vsdk::SUMat( aIn, lRectM);

    vsdk::Rect lRectO(0, aInH*5, aInW, aInH);
    vsdk::SUMat outOMat = vsdk::SUMat( aIn, lRectO);
  
    ret = pProcess->ReconnectIO(inLMat, inUMat, inVMat, outMMat, outOMat, 0);

    if(APEXCV_SUCCESS == ret)
    {
        ret = pProcess->Process();
    } 
    else
    {
        printf( " AggCFDetector::aggcf_magori_calc %d \n ", ret );
    }   
    return ret;
}


APEXCV_LIB_RESULT AggCFDetector::AGGCFfltCalc(vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH,
                   vsdk::SUMat &aOut, int aOutOffset, int , int , int aNum, uint32_t aP, uint32_t aNormConst,int aApexID)
{
    int ApexIndex = aApexID;
    if( aApexID < 0 || aApexID > 1)
    {
        printf( " Flt3x3: invalid apex assignment, use default apexID %d \n", mApexID );
        ApexIndex = mApexID;
    }
    AGGCFflt3x3 *pProcess = (AGGCFflt3x3 *)mAggcfFlt3x3ApexP[ApexIndex];
    if(nullptr == pProcess)
    {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    APEXCV_LIB_RESULT ret = APEXCV_SUCCESS;

    for(int i = 0; i < aNum; i++)
    {
        vsdk::Rect lRectIn(0, (i+aInOffset)*aInH, aInW, aInH);
        vsdk::SUMat inMat = vsdk::SUMat( aIn, lRectIn);

        vsdk::Rect lRectOut(0, (i+aOutOffset)*aInH, aInW, aInH);
        vsdk::SUMat outMat = vsdk::SUMat( aOut, lRectOut);

        ret = pProcess->ReconnectIO(inMat, outMat, aP, aNormConst);

        if ( APEXCV_SUCCESS == ret) 
        {  
            ret = pProcess->Process();
        }
        if( APEXCV_SUCCESS != ret)
        {
            printf( " AggCFDetector::aggcf_flt_calc %d \n ", ret );
            break; 
        }       
    }

    return ret;
}


APEXCV_LIB_RESULT AggCFDetector::AGGCFfltCalcCon( vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH,
                       vsdk::SUMat &aOut, int aOutOffset, int, int, int aNum, uint32_t aP, uint32_t aNormConst, int aApexID )
{
    int ApexIndex = aApexID;
    if( aApexID < 0 || aApexID > 1)
    {
       printf( " Flt3x3con: invalid apex assignment, use default apexID %d \n", mApexID );
       ApexIndex = mApexID;
    }
    AGGCFflt3x3 *pProcess = (AGGCFflt3x3 *)mAggcfFlt3x3ApexP[ApexIndex];
    if(nullptr == pProcess)
    {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    APEXCV_LIB_RESULT ret = APEXCV_SUCCESS;

    vsdk::Rect lRectIn(0, aInOffset*aInH, aInW, aInH*aNum);
    vsdk::SUMat inMat = vsdk::SUMat( aIn, lRectIn);

    vsdk::Rect lRectOut(0, aOutOffset*aInH, aInW, aInH*aNum);
    vsdk::SUMat outMat = vsdk::SUMat( aOut, lRectOut);

    ret = pProcess->ReconnectIO(inMat, outMat, aP, aNormConst);

    if ( APEXCV_SUCCESS == ret)
    {
       ret = pProcess->Process();
    }
    else
    {
       printf( " AggCFDetector::aggcf_flt_calc_con %d \n ", ret );
    }


    return ret;
}

APEXCV_LIB_RESULT AggCFDetector::AGGCFScaleDown2Calc( vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH,
                       vsdk::SUMat &aOut, int aOutOffset, int aOutW, int aOutH, int aNum, int aApexID )
{
    int ApexIndex = aApexID;
    if( aApexID < 0 || aApexID > 1)
    {
       printf( " ScaleDown2: invalid apex assignment, use default apexID %d \n", mApexID );
       ApexIndex = mApexID;
    }
    AGGCFScaleDown2 *pProcess = (AGGCFScaleDown2 *)mAggcfScaleDown2P[ApexIndex];
    if(nullptr == pProcess)
    {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    APEXCV_LIB_RESULT ret = APEXCV_SUCCESS;

    vsdk::Rect lRectIn(0, aInOffset*aInH, aInW, aInH*aNum);
    vsdk::SUMat inMat = vsdk::SUMat( aIn, lRectIn);

    vsdk::Rect lRectOut(0, aOutOffset*aInH, aOutW, aOutH*aNum);
    vsdk::SUMat outMat = vsdk::SUMat( aOut, lRectOut);

    ret = pProcess->ReconnectIO(inMat, outMat);

    if ( APEXCV_SUCCESS == ret)
    {
       ret = pProcess->Process();
    }
    else
    {
       printf( " AggCFDetector::AGGCFScaleDown2Calc %d \n ", ret );
    }


    return ret;
}

APEXCV_LIB_RESULT AggCFDetector::AGGCFScaleDown4Calc( vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH,
                       vsdk::SUMat &aOut, int aOutOffset, int aOutW, int aOutH, int aNum, int aApexID )
{
    int ApexIndex = aApexID;
    if( aApexID < 0 || aApexID > 1)
    {
       printf( " ScaleDown4: invalid apex assignment, use default apexID %d \n", mApexID );
       ApexIndex = mApexID;
    }
    AGGCFScaleDown4 *pProcess = (AGGCFScaleDown4 *)mAggcfScaleDown4P[ApexIndex];
    if(nullptr == pProcess)
    {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    APEXCV_LIB_RESULT ret = APEXCV_SUCCESS;

    vsdk::Rect lRectIn(0, aInOffset*aInH, aInW, aInH*aNum);
    vsdk::SUMat inMat = vsdk::SUMat( aIn, lRectIn);

    vsdk::Rect lRectOut(0, aOutOffset*aInH, aOutW, aOutH*aNum);
    vsdk::SUMat outMat = vsdk::SUMat( aOut, lRectOut);

    ret = pProcess->ReconnectIO(inMat, outMat);

    if ( APEXCV_SUCCESS == ret)
    {
       ret = pProcess->Process();
    }
    else
    {
       printf( " AggCFDetector::AGGCFScaleDown4Calc %d \n ", ret );
    }


    return ret;
}


APEXCV_LIB_RESULT AggCFDetector::AGGCFScaleDown8Calc( vsdk::SUMat &aIn, int aInOffset, int aInW, int aInH,
                       vsdk::SUMat &aOut, int aOutOffset, int aOutW, int aOutH, int aNum, int aApexID )
{
    int ApexIndex = aApexID;
    if( aApexID < 0 || aApexID > 1)
    {
       printf( " ScaleDown8: invalid apex assignment, use default apexID %d \n", mApexID );
       ApexIndex = mApexID;
    }
    AGGCFScaleDown8 *pProcess = (AGGCFScaleDown8 *)mAggcfScaleDown8P[ApexIndex];
    if(nullptr == pProcess)
    {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    APEXCV_LIB_RESULT ret = APEXCV_SUCCESS;

    vsdk::Rect lRectIn(0, aInOffset*aInH, aInW, aInH*aNum);
    vsdk::SUMat inMat = vsdk::SUMat( aIn, lRectIn);

    vsdk::Rect lRectOut(0, aOutOffset*aInH, aOutW, aOutH*aNum);
    vsdk::SUMat outMat = vsdk::SUMat( aOut, lRectOut);

    ret = pProcess->ReconnectIO(inMat, outMat);

    if ( APEXCV_SUCCESS == ret)
    {
       ret = pProcess->Process();
    }
    else
    {
       printf( " AggCFDetector::AGGCFScaleDown8Calc %d \n ", ret );
    }


    return ret;
}

APEXCV_LIB_RESULT AggCFDetector::AGGCFhistCalc(vsdk::SUMat &aIn, int aInW, int aInH, 
                     vsdk::SUMat &aOut, int aOutW, int aOutH, 
                     int bin, int softbin, int full, int aNum, int aApexID )
{
    int ApexIndex = aApexID;
    if( aApexID < 0 || aApexID > 1)
    {
       printf( " hist: invalid apex assignment, use default apexID %d \n", mApexID );
       ApexIndex = mApexID;
    }
    AGGCFhist *pProcess = (AGGCFhist *)mAggcfHistApexP[ApexIndex]; 
    if(nullptr == pProcess)
    {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    APEXCV_LIB_RESULT ret = APEXCV_SUCCESS;

    vsdk::Rect lRectM(0, 3*aInH, aInW, aInH);
    vsdk::SUMat inMMat = vsdk::SUMat( aIn, lRectM);

    vsdk::Rect lRectO(0, 5*aInH, aInW, aInH);
    vsdk::SUMat inOMat = vsdk::SUMat( aIn, lRectO);

    vsdk::Rect lRecth1(0, 4*aOutH, aOutW, aOutH);
    vsdk::SUMat outh1Mat = vsdk::SUMat( aOut, lRecth1);

    vsdk::Rect lRecth2(0, 5*aOutH, aOutW, aOutH);
    vsdk::SUMat outh2Mat = vsdk::SUMat( aOut, lRecth2);
    
    vsdk::Rect lRecth3(0, 6*aOutH, aOutW, aOutH);
    vsdk::SUMat outh3Mat = vsdk::SUMat( aOut, lRecth3);

    vsdk::Rect lRecth4(0, 7*aOutH, aOutW, aOutH);
    vsdk::SUMat outh4Mat = vsdk::SUMat( aOut, lRecth4);
  
    vsdk::Rect lRecth5(0, 8*aOutH, aOutW, aOutH);
    vsdk::SUMat outh5Mat = vsdk::SUMat( aOut, lRecth5);

    vsdk::Rect lRecth6(0, 9*aOutH, aOutW, aOutH);
    vsdk::SUMat outh6Mat = vsdk::SUMat( aOut, lRecth6);

    ret  = pProcess->ReconnectIO(inMMat, inOMat, outh1Mat, outh2Mat, outh3Mat, outh4Mat, outh5Mat, outh6Mat,
                                    full, aNum, bin, softbin  );

    if( APEXCV_SUCCESS == ret)
    {
       ret = pProcess->Process();
    }
    else
    {
       printf( " AggCFDetector::aggcf_hist_calc %d \n ", ret );
    }


    return ret;

}

}



