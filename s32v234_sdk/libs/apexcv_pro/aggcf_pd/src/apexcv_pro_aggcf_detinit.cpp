/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2016-2018 NXP
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
//#define DEBUG_OUTPUT

// load detector and calculate configuration parameters such as scales/octaves

namespace apexcv{


AggCFDetector::~AggCFDetector()
{

   {
   
      for(uint32_t s = 0; s < scalePars.size(); s++)
      {
         if(mpChannelIds[s])
            delete[] mpChannelIds[s];

      }
      if( mpChannelIds) 
         delete[] mpChannelIds;

      if(mpFids)             delete mpFids;
      if(mpThrs)             delete mpThrs;
      if(mpChild)            delete mpChild;
      if(mpHs)               delete mpHs;
      if(mdOpts.modelDs)    delete mdOpts.modelDs;
      if(mdOpts.modelDsPad) delete mdOpts.modelDsPad;
      if(mdOpts.lambdas)    delete mdOpts.lambdas;
      if(mdOpts.pad)        delete mdOpts.pad;
      if(mdOpts.minDs)      delete mdOpts.minDs;
   }  

   if(nullptr != mAggcfMagoriApexP[0] ) 
   {
      delete (AGGCFMagOri *)mAggcfMagoriApexP[0];
      mAggcfMagoriApexP[0]  = nullptr; 
   }
 
   if(nullptr != mAggcfMagoriApexP[1] ) 
   {
      delete (AGGCFMagOri *)mAggcfMagoriApexP[1];
      mAggcfMagoriApexP[1]  = nullptr; 
   }

   if(nullptr != mAggcfFlt3x3ApexP[0])
   {
      delete (AGGCFflt3x3 *)mAggcfFlt3x3ApexP[0];
      mAggcfFlt3x3ApexP[0] = nullptr; 
   } 

   if(nullptr != mAggcfFlt3x3ApexP[1])
   {
      delete (AGGCFflt3x3 *)mAggcfFlt3x3ApexP[1];
      mAggcfFlt3x3ApexP[1] = nullptr; 
   }

   if(nullptr != mAggcfHistApexP[0])
   {
      delete (AGGCFhist *)mAggcfHistApexP[0];
      mAggcfHistApexP[0] = nullptr; 
   }
 
   if(nullptr != mAggcfHistApexP[1])
   {
      delete (AGGCFhist *)mAggcfHistApexP[1];
      mAggcfHistApexP[1] = nullptr; 
   }
  
   if(nullptr != mAggcfScaleDown8P[0])
   {  
      delete (AGGCFScaleDown8 *)mAggcfScaleDown8P[0];
      mAggcfScaleDown8P[0] = nullptr;
   }

   if(nullptr != mAggcfScaleDown8P[1])
   {  
      delete (AGGCFScaleDown8 *)mAggcfScaleDown8P[1];
      mAggcfScaleDown8P[1] = nullptr;
   }

   if(nullptr != mAggcfScaleDown4P[0])
   {
      delete (AGGCFScaleDown4 *)mAggcfScaleDown4P[0];
      mAggcfScaleDown4P[0] = nullptr;
   }

   if(nullptr != mAggcfScaleDown4P[1])
   {
      delete (AGGCFScaleDown4 *)mAggcfScaleDown4P[1];
      mAggcfScaleDown4P[1] = nullptr;
   }

   if(nullptr != mAggcfScaleDown2P[0])
   {
      delete (AGGCFScaleDown2 *)mAggcfScaleDown2P[0];
      mAggcfScaleDown2P[0] = nullptr;
   }

   if(nullptr != mAggcfScaleDown2P[1])
   {
      delete (AGGCFScaleDown2 *)mAggcfScaleDown2P[1];
      mAggcfScaleDown2P[1] = nullptr;
   }
   
   if(nullptr != mAggcfLUVApexP[0])
   {
      delete (AGGCFluv *)mAggcfLUVApexP[0];
      mAggcfLUVApexP[0] = nullptr;
   }
   
   if(nullptr != mAggcfLUVApexP[1])
   {
      delete (AGGCFluv *)mAggcfLUVApexP[1];
      mAggcfLUVApexP[1] = nullptr;
   }
}

AggCFDetector::AggCFDetector()
 :
   mTreeDepth(0),
   mnTreeNodes(0),
   mnTrees(0),
   mfailed(true),
   mpChannelIds(nullptr)
{
   mpThrs    = nullptr;
   mpFixThrs = nullptr;
   mpHs      = nullptr;
   mpFids    = nullptr;
   mpChild   = nullptr;

   mdOpts.modelDs = nullptr; mdOpts.modelDsPad = nullptr; mdOpts.minDs = nullptr;
   mdOpts.pad = nullptr; mdOpts.lambdas = nullptr;

   scalePars  = std::vector<scalePar>(0);
   AGGCFMagOri     *aggcfMagOriP0    = new AGGCFMagOri; 
   AGGCFMagOri     *aggcfMagOriP1    = new AGGCFMagOri; 
   AGGCFflt3x3     *aggcfFlt0        = new AGGCFflt3x3;  
   AGGCFflt3x3     *aggcfFlt1        = new AGGCFflt3x3;  
   AGGCFluv        *aggcfLuv0        = new AGGCFluv;
   AGGCFluv        *aggcfLuv1        = new AGGCFluv;
   AGGCFhist       *aggcfHist0       = new AGGCFhist;
   AGGCFhist       *aggcfHist1       = new AGGCFhist;
   AGGCFScaleDown8 *aggcfScaledown80 = new AGGCFScaleDown8; 
   AGGCFScaleDown8 *aggcfScaledown81 = new AGGCFScaleDown8; 
   AGGCFScaleDown4 *aggcfScaledown40 = new AGGCFScaleDown4; 
   AGGCFScaleDown4 *aggcfScaledown41 = new AGGCFScaleDown4; 
   AGGCFScaleDown2 *aggcfScaledown20 = new AGGCFScaleDown2; 
   AGGCFScaleDown2 *aggcfScaledown21 = new AGGCFScaleDown2; 

   mAggcfMagoriApexP[0] = aggcfMagOriP0;
   mAggcfMagoriApexP[1] = aggcfMagOriP1;
   mAggcfFlt3x3ApexP[0] = aggcfFlt0;
   mAggcfFlt3x3ApexP[1] = aggcfFlt1;
   mAggcfLUVApexP[0]    = aggcfLuv0;
   mAggcfLUVApexP[1]    = aggcfLuv1;
   mAggcfHistApexP[0]   = aggcfHist0;
   mAggcfHistApexP[1]   = aggcfHist1;
   mAggcfScaleDown8P[0] = aggcfScaledown80;
   mAggcfScaleDown8P[1] = aggcfScaledown81;
   mAggcfScaleDown4P[0] = aggcfScaledown40;
   mAggcfScaleDown4P[1] = aggcfScaledown41;
   mAggcfScaleDown2P[0] = aggcfScaledown20;
   mAggcfScaleDown2P[1] = aggcfScaledown21;

   mApexID = 0;

   if ( mAggcfMagoriApexP[0] == nullptr || mAggcfMagoriApexP[1] == nullptr ||
        mAggcfFlt3x3ApexP[0] == nullptr || mAggcfFlt3x3ApexP[1] == nullptr ||
        mAggcfLUVApexP[0]    == nullptr || mAggcfLUVApexP[1]    == nullptr ||
        mAggcfHistApexP[0]   == nullptr || mAggcfHistApexP[1]   == nullptr ||
        mAggcfScaleDown8P[0] == nullptr || mAggcfScaleDown8P[1] == nullptr ||
        mAggcfScaleDown4P[0] == nullptr || mAggcfScaleDown4P[1] == nullptr ||
        mAggcfScaleDown2P[0] == nullptr || mAggcfScaleDown2P[1] == nullptr    )
   {
       this->mfailed = true;
       return;
   }

   AGGCFMagOri *mAggcfMagoriApex_0, *mAggcfMagoriApex_1;
   mAggcfMagoriApex_0 = (AGGCFMagOri *)mAggcfMagoriApexP[0];
   mAggcfMagoriApex_1 = (AGGCFMagOri *)mAggcfMagoriApexP[1];

   AGGCFflt3x3 *mAggcfFlt3x3Apex_0, *mAggcfFlt3x3Apex_1;
   mAggcfFlt3x3Apex_0 = (AGGCFflt3x3 *)mAggcfFlt3x3ApexP[0];
   mAggcfFlt3x3Apex_1 = (AGGCFflt3x3 *)mAggcfFlt3x3ApexP[1];

   AGGCFluv    *mAggcfLUVApex_0, *mAggcfLUVApex_1;
   mAggcfLUVApex_0 = (AGGCFluv *)mAggcfLUVApexP[0];
   mAggcfLUVApex_1 = (AGGCFluv *)mAggcfLUVApexP[1];

   AGGCFhist   *mAggcfHistApex_0, *mAggcfHistApex_1;
   mAggcfHistApex_0 = (AGGCFhist *)mAggcfHistApexP[0];
   mAggcfHistApex_1 = (AGGCFhist *)mAggcfHistApexP[1];

   AGGCFScaleDown8 *mAggcfScaleDown8_0, *mAggcfScaleDown8_1;
   mAggcfScaleDown8_0 = (AGGCFScaleDown8 *)mAggcfScaleDown8P[0];
   mAggcfScaleDown8_1 = (AGGCFScaleDown8 *)mAggcfScaleDown8P[1];

   AGGCFScaleDown4 *mAggcfScaleDown4_0, *mAggcfScaleDown4_1;
   mAggcfScaleDown4_0 = (AGGCFScaleDown4 *)mAggcfScaleDown4P[0];
   mAggcfScaleDown4_1 = (AGGCFScaleDown4 *)mAggcfScaleDown4P[1];

   AGGCFScaleDown2 *mAggcfScaleDown2_0, *mAggcfScaleDown2_1;
   mAggcfScaleDown2_0 = (AGGCFScaleDown2 *)mAggcfScaleDown2P[0];
   mAggcfScaleDown2_1 = (AGGCFScaleDown2 *)mAggcfScaleDown2P[1];

   // default processes APEX init
   mAggcfMagoriApex_0->SetApuConfiguration(ACF_APU_CFG__DEFAULT, mApexID);
   mAggcfFlt3x3Apex_0->SetApuConfiguration(ACF_APU_CFG__DEFAULT, mApexID);
   mAggcfHistApex_0->SetApuConfiguration(ACF_APU_CFG__DEFAULT, mApexID);
   mAggcfLUVApex_0->SetApuConfiguration(ACF_APU_CFG__DEFAULT,mApexID);

   mAggcfScaleDown2_0->SetApuConfiguration(ACF_APU_CFG__DEFAULT,mApexID);
   mAggcfScaleDown4_0->SetApuConfiguration(ACF_APU_CFG__DEFAULT,mApexID);
   mAggcfScaleDown8_0->SetApuConfiguration(ACF_APU_CFG__DEFAULT,mApexID);

   int apexId = (mApexID == 0)?1:0;

   mAggcfMagoriApex_1->SetApuConfiguration(ACF_APU_CFG__DEFAULT, apexId);
   mAggcfFlt3x3Apex_1->SetApuConfiguration(ACF_APU_CFG__DEFAULT, apexId);
   mAggcfHistApex_1->SetApuConfiguration(ACF_APU_CFG__DEFAULT, apexId);
   mAggcfLUVApex_1->SetApuConfiguration(ACF_APU_CFG__DEFAULT, apexId);

   mAggcfScaleDown2_1->SetApuConfiguration(ACF_APU_CFG__DEFAULT,mApexID);
   mAggcfScaleDown4_1->SetApuConfiguration(ACF_APU_CFG__DEFAULT,mApexID);
   mAggcfScaleDown8_1->SetApuConfiguration(ACF_APU_CFG__DEFAULT,mApexID);

}

// convert fid array from column major to row major
uint32_t *convertFid(uint32_t * fidsArray, uint32_t num, 
                     uint32_t window_width, uint32_t window_height )
{
    //printf( " convertFid, %d, %d, %d \n", num, window_width, window_height  );
  
    uint32_t windows_size = window_width*window_height;

    for(uint32_t i = 0; i < num; i++)
    {
        uint32_t r = fidsArray[i]/windows_size; uint32_t m = fidsArray[i]%windows_size;

        uint32_t m_tilda = (m%window_height)*window_width + m/window_height;

        fidsArray[i] = m_tilda + r*windows_size;
    }
    return fidsArray;
}

void AggCFDetector::InitDetectorModelBuf( char *aDetectorBuf, int aBufSize)
{
    if( nullptr == aDetectorBuf || aBufSize <= 0)
    {
       return;
    }

    char *linePtr = aDetectorBuf;
    int lineIndex = 0;
    int nTreeNode = 0, nTree = 0;

    this->mfailed = false;

    while (linePtr - aDetectorBuf < aBufSize)
    {
        char *eol = strchr(linePtr, '\n');
        if( 0 == lineIndex)
        {
            sscanf(linePtr, "%d %d %d", &(this->mTreeDepth), &(this->mnTreeNodes), &(this->mnTrees));
         
            nTreeNode     = this->mnTreeNodes; nTree = this->mnTrees;
            this->mpThrs    = new double[nTreeNode*nTree];
            this->mpFixThrs = new uint32_t[nTreeNode*nTree];
            this->mpHs      = new double[nTreeNode*nTree];
            this->mpFids    = new uint32_t[nTreeNode*nTree];
            this->mpChild   = new uint32_t[nTreeNode*nTree];
        } 
        else if( 1 == lineIndex)
        {
           char *endPtr = linePtr; 

           for( int i = 0; i < nTreeNode*nTree; i++)
           { 
               this->mpThrs[i] = strtod(linePtr, &endPtr);
               linePtr = endPtr;
           } 
        } 
        else if( 2 == lineIndex)
        {
           char *endPtr = linePtr;

           for( int i = 0; i < nTreeNode*nTree; i++)
           {
               this->mpHs[i] = strtod(linePtr, &endPtr);
               linePtr = endPtr;
           }
        }
        else if( 3 == lineIndex)
        {
           char *endPtr = linePtr;

           for( int i = 0; i < nTreeNode*nTree; i++)
           {
               this->mpFids[i] = strtol(linePtr, &endPtr, 10);
               linePtr = endPtr;
           }
        } 
        else if( 4 == lineIndex)
        {
           char *endPtr = linePtr;

           for( int i = 0; i < nTreeNode*nTree; i++)
           {
               this->mpChild[i] = strtol(linePtr, &endPtr, 10);
               linePtr = endPtr;
           }
        }
        else
        {
           mdOpts.modelDs = new float[2]; mdOpts.modelDsPad = new float[2]; mdOpts.minDs = new float[2];
           mdOpts.pad = new int[2]; mdOpts.lambdas = new float[3];

           sscanf( linePtr, "%f %f %f %f %d %f %f %d %d %d %f %f %f %d %d %f %f %f %d %d %f %d %d %d %lf",  
                &mdOpts.modelDs[0], &mdOpts.modelDs[1], &mdOpts.modelDsPad[0], &mdOpts.modelDsPad[1], 
                &mdOpts.stride, &mdOpts.cascThr, &mdOpts.cascCal, &mdOpts.nPerOct, &mdOpts.nOctUp, 
                &mdOpts.nApprox, &mdOpts.minDs[0], &mdOpts.minDs[1], &mdOpts.smooth, &mdOpts.pad[0], 
                &mdOpts.pad[1], &mdOpts.lambdas[0], &mdOpts.lambdas[1], &mdOpts.lambdas[2], 
                &mdOpts.shrink, &mdOpts.normRad, &mdOpts.normConst, &mdOpts.full, &mdOpts.nOrients, 
                &mdOpts.softBin, &mdOpts.overlap);
    
        }      

        while ((*eol == '\r' || *eol == '\n') && ( eol - aDetectorBuf < aBufSize))
        {
            eol++;
        } 

        linePtr = eol;  
        lineIndex ++;
    }

    for(int i = 0; i < nTreeNode*nTree; i++)
    {
        this->mpFixThrs[i] = (uint32_t)(this->mpThrs[i]*pow(2, 14));
    }

}


void AggCFDetector::InitDetectorModel(const char *aDetectorName)
{
    if (nullptr == aDetectorName)
    {
       return;
    }  
    this->mfailed = false;
    int index = 0;
    int nTreeNode = 0, nTree = 0;    // local mnTreeNodes and mnTrees


    if (!std::ifstream(aDetectorName))
    {
        this->mfailed = true;
        std::cout << "Detector File Does Not Exist: " << aDetectorName << std::endl;
        return;
    }
    std::ifstream detectorFile(aDetectorName);

    void *detectorArr = NULL;
    std::string inArray;

    while (getline(detectorFile, inArray))
    {
        std::istringstream iss(inArray);

        switch (index)
        {
            case 0:
                if (!(iss >> this->mTreeDepth >> this->mnTreeNodes >> this->mnTrees))
                {
                    this->mfailed = true;  
                    std::cout << "Failed to read decision tree structure parameters to detector" << std::endl;
                    return; 
                }
               
                // read decision tree numbers and tree nodes 
                nTreeNode     = this->mnTreeNodes; nTree = this->mnTrees;
                this->mpThrs    = new double[nTreeNode*nTree];
                this->mpFixThrs = new uint32_t[nTreeNode*nTree];
                this->mpHs      = new double[nTreeNode*nTree];
                this->mpFids    = new uint32_t[nTreeNode*nTree];
                this->mpChild   = new uint32_t[nTreeNode*nTree];  
                break;    

            case 1:
                detectorArr = static_cast <double*>(this->mpThrs);
                break;
            case 2:
                detectorArr = static_cast <double*>(this->mpHs);
                break;
            case 3:
                detectorArr = static_cast <uint32_t*>(this->mpFids);
                break;
            case 4:
                detectorArr = static_cast <uint32_t*>(this->mpChild);
                break;
            default:
                break;
        }

        if (index >= 1 && index <= 4)
        {
            for (int n = 0; n < nTreeNode; n++)
            { 
                for (int m = 0; m < nTree; m++)
                {
                    int err = 1;
                    if (index < 3 && !(iss >> (*(((double*)detectorArr) + n*nTree + m))))
                        err = 0;
                    if (index >= 3 && !(iss >> (*(((uint32_t*)detectorArr) + n*nTree + m))))
                        err = 0;
                    if (!err)
                    {
                        this->mfailed = true;
                        std::cout << "Failed to read in decision tree values to detector" << std::endl;                        
                        return;
                    }
                }
            }
        }

        if (index == 5)
        {
            mdOpts.modelDs = new float[2]; mdOpts.modelDsPad = new float[2]; mdOpts.minDs = new float[2];
            mdOpts.pad = new int[2]; mdOpts.lambdas = new float[3];
            if( !(iss >> mdOpts.modelDs[0] >> mdOpts.modelDs[1] >> mdOpts.modelDsPad[0] >> mdOpts.modelDsPad[1] >>
                mdOpts.stride >> mdOpts.cascThr >> mdOpts.cascCal >> mdOpts.nPerOct >> mdOpts.nOctUp >>
                mdOpts.nApprox >> mdOpts.minDs[0] >> mdOpts.minDs[1] >> mdOpts.smooth >> mdOpts.pad[0] >>
                mdOpts.pad[1] >> mdOpts.lambdas[0] >> mdOpts.lambdas[1] >> mdOpts.lambdas[2] >>
                mdOpts.shrink >> mdOpts.normRad >> mdOpts.normConst >> mdOpts.full >> mdOpts.nOrients >>
                mdOpts.softBin >> mdOpts.overlap)) // >> mdOpts.luvSmooth) )
            {
                this->mfailed = true;
                std::cout << "Failed to read in detector parameters to detector" << std::endl;
                return;
            } 
        }
        ++index;
    }

    for(int i = 0; i < nTreeNode*nTree; i++)
    {
        this->mpFixThrs[i] = (uint32_t)(this->mpThrs[i]*pow(2, 14)); 
    }
   
    // doing basic model parameter check
    if ( mdOpts.nPerOct != 4 || mdOpts.nOctUp != 0 || mdOpts.nApprox != 3 )
    {
        this->mfailed = true;
        std::cout << "Model octave parameter settings are incorrect \n" << std::endl;
        std::cout << "Only support Octave setting: 4 scales per octave \n" << std::endl;
        return;
    }
    if ( mdOpts.shrink != 2 || mdOpts.softBin != 1 )
    {
        std::cout << mdOpts.shrink  << mdOpts.softBin  << std::endl;
        this->mfailed = true;
        std::cout << "Model HOG parameter settings are incorrect \n" << std::endl;
        std::cout << "Only support HOG setting: downsize raito = 2 and softBin = 1 \n" << std::endl;
        return;
    }

#ifdef DEBUG_OUTPUT
    
    std::cout << " AGGCF detector configuration: " << std::endl;
    std::cout << " number of decision tree: " << nTree << " treenode number: " << nTreeNode << std::endl;
    std::cout << " = search windows parameters = " << std::endl;
    std::cout << " modelDs: " << mdOpts.modelDs[0] << "/" << mdOpts.modelDs[1] << " modelDsPad: " << mdOpts.modelDsPad[0] << "/" << mdOpts.modelDsPad[1] << std::endl;
    std::cout << " stride: " << mdOpts.stride << " cascThr/cascCal: " << mdOpts.cascThr << "/" << mdOpts.cascCal << std::endl;
    std::cout << " = pyramid parameters = " << std::endl;
    std::cout << " nPerOct: " << mdOpts.nPerOct << " nOctUp: " << mdOpts.nOctUp << " nApprox: " << mdOpts.nApprox << std::endl; 
    std::cout << " min scale size: " << mdOpts.minDs[0] << "/" << mdOpts.minDs[1] << "pad" << mdOpts.pad[0] << "/" << mdOpts.pad[1] << std::endl;
    std::cout << " = approximate scale lambdas = " << std::endl;
    std::cout << " lambda 0/1/2 "  << mdOpts.lambdas[0] << " " << mdOpts.lambdas[1] << " " << mdOpts.lambdas[2] << std::endl;
    std::cout << " feature calculation parameters " << std::endl;
    std::cout << " shrink(bin) / " << mdOpts.shrink << " softbin " << mdOpts.softBin << std::endl;
    std::cout << " full(orientation range) / " << mdOpts.full << " number of orientations " << mdOpts.nOrients << std::endl;
    std::cout << " magnitude smooth / " << mdOpts.smooth << " / magnitude normRad " << mdOpts.normRad << " / magnitude normConst " << mdOpts.normConst << std::endl;
    std::cout << " luv smooth " << mdOpts.luvSmooth << " nms overlapping parameter " << mdOpts.overlap <<  std::endl;

#endif


    detectorFile.close();

}


// based on scales/octaves configuration, calculate scaling ratio for each scales
void AggCFDetector::CalcScaleParameters( int aInwidth, int aInheight ) 
{

    int nPerOct = mdOpts.nPerOct, nOctUp = mdOpts.nOctUp, shrink = mdOpts.shrink, h = aInheight, minH = mdOpts.minDs[0], w = aInwidth, minW = mdOpts.minDs[1];

    std::vector<float> scales;
    
    if (h == 0 || w == 0)
        return;

    int nScales = floor(nPerOct*(nOctUp + log2(std::min((double)h / minH, (double)w / minW))) + 1);
    if( nScales > 12)
    {
        nScales = 12;
    }

    int d0, d1;
    if (h < w) { d0 = h; d1 = w; }
    else { d0 = w; d1 = h; }

    for (int i = 0; i < nScales; ++i)
    {
        float rtSS = 0;
        float ss, es0, es1, mEs;
        float s = pow(2.0f, (float)-i / nPerOct + nOctUp);
        float s0 = (round(d0*s / shrink)*shrink - .25f*shrink) / d0;
        float s1 = (round(d0*s / shrink)*shrink + .25f*shrink) / d0;
        s = FLT_MAX;

        for (int j = 0; j < 102; ++j)
        {
            ss = 0.01f*j*(s1 - s0) + s0;
            es0 = d0*ss;
            es0 = fabs(es0 - round(es0 / shrink)*shrink);
	    es1 = d1*ss;
            es1 = fabs(es1 - round(es1 / shrink)*shrink);
            mEs = std::max(es0, es1);

            if (mEs < s)
            {
                s = mEs;
                rtSS = ss;
            }
        }
        scales.push_back(rtSS);
    }

    for (std::vector<float>::iterator it = scales.begin(); it != scales.end() - 1;)
    {
        if (*it == *(it + 1))
            it = scales.erase(it);
        else
            ++it;
    }

    uint32_t realScale = 0;
    uint32_t thrScale = nPerOct + 1;
    
    for (uint32_t i = 0; i < scales.size(); ++i)
    {

        scalePar scaleParitem;

        if (0 == (i % nPerOct)) // real scales
        {      
            // make sure target scale width/height can be divided by shrink
            int resH = round(h*scales[i] / shrink)*shrink;
            int resW = round(w*scales[i] / shrink)*shrink;
            scaleParitem.isRealscale = 1;
            scaleParitem.realScaleIndex = i;
            scaleParitem.width    = resW; 
            scaleParitem.height   = resH;
            scaleParitem.spanSize = resW/shrink*sizeof(uint16_t);

        }  
        else // approximate scales
        {

            int resH = round(h*scales[i] / shrink)*shrink;
            int resW = round(w*scales[i] / shrink)*shrink;
            scaleParitem.isRealscale = 0;

            if ((i == thrScale) && (realScale + nPerOct) < scales.size()) 
            {
                realScale += nPerOct;
                thrScale  += nPerOct;   
            }  
            scaleParitem.realScaleIndex = realScale;
            scaleParitem.width    = resW; 
            scaleParitem.height   = resH;
            scaleParitem.spanSize = resW/shrink*sizeof(uint16_t); 

        }  

        scaleParitem.scaleRatio = scales[i];
        scalePars.push_back(scaleParitem);
    }

#ifdef DEBUG_OUTPUT

    std::cout << " AGGCF pyramid structure  " << std::endl;  
    
    for (uint32_t i = 0; i < scales.size(); ++i)
    {
        if( scalePars[i].isRealscale == 1 )
        {
           std::cout << " real scale: index: " << i << " ratio: " << scalePars[i].scaleRatio << " width: " << scalePars[i].width << " height: " << scalePars[i].height << std::endl;
        }
        else
        {
           std::cout << " approximate scale: index: " << i << " ratio: " << scalePars[i].scaleRatio << " width: " << scalePars[i].width << " height: " << scalePars[i].height << std::endl;
        }
    }

#endif

    // init feature ID index array (row major)
    initFidsArray();

    // convert feature id array from column major to row major
    this->mpFids = convertFid(this->mpFids, this->mnTreeNodes*this->mnTrees, mdOpts.modelDsPad[1]/mdOpts.shrink, mdOpts.modelDsPad[0]/shrink );
   
    //printf("feature ID array inited \n  ");
}


}










