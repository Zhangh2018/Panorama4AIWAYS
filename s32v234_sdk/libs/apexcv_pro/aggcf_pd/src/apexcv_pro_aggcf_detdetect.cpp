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

namespace apexcv{

inline void getChild(uint16_t *apChns1, uint32_t *apCids,  uint32_t *apFids,
                     uint32_t *apThrs,  uint32_t aOffset, uint32_t &aK0, uint32_t &aK)
{
   uint16_t ftr = apChns1[apCids[apFids[aK]]];
   aK = (ftr < apThrs[aK]) ? 1 : 2;
   aK0 = aK += aK0 * 2; aK += aOffset;
}

void AggCFDetector::initFidsArray()
{
    const int nChns     = 4 + mdOpts.nOrients;

    //  search window size
    const float modelH  = mdOpts.modelDsPad[0];
    const float modelW  = mdOpts.modelDsPad[1];
    const int shrink    = mdOpts.shrink;

    int numFeatures = modelH / shrink*modelW / shrink*nChns;

    mNumFeatures = numFeatures;

    mpChannelIds = new uint32_t *[scalePars.size()];

    for(uint32_t s = 0; s < scalePars.size(); s++)
    {
        mpChannelIds[s] = new uint32_t[numFeatures];

        const int height = scalePars[s].height/mdOpts.shrink;
        const int width  = scalePars[s].width/mdOpts.shrink;

        int m = 0;
        for(int i = 0; i < nChns; i++)
        {
	        for( int k = 0; k < modelH/shrink; k++)
            {
	            for( int j = 0; j < modelW/shrink; j++)
                {
                    // calculate feature index array to access calculated features
                    mpChannelIds[s][m++] = i*width*height + j + k*width;   // row major
                }
            }
        }
    }
}


void AggCFDetector::updateScaleFidsArray( uint32_t aScaleIndex, uint32_t aScaleSpan )
{
    const int nChns     = 4 + mdOpts.nOrients;
  
    const float modelH  = mdOpts.modelDsPad[0];
    const float modelW  = mdOpts.modelDsPad[1];
    const int shrink    = mdOpts.shrink;

    const int height = scalePars[aScaleIndex].height/mdOpts.shrink;
    const int width  = aScaleSpan/sizeof(uint16_t);
    scalePars[aScaleIndex].spanSize = aScaleSpan;

    int m = 0;
    for(int i = 0; i < nChns; i++)
    {
        for( int k = 0; k < modelH/shrink; k++) 
        {
            for( int j = 0; j < modelW/shrink; j++)
            {
                mpChannelIds[aScaleIndex][m++] = i*width*height + j + k*width; // row major
            }
        }
    }

}


// print out detection parameters to verify detection configuration

void AggCFDetector::ShowDetectorParameters()
{
    const int nChns     = 4 + mdOpts.nOrients;
    const int stride    = mdOpts.stride;
    const int shrink    = mdOpts.shrink;
    
    //  search window size$
    const float modelH  = mdOpts.modelDsPad[0];
    const float modelW  = mdOpts.modelDsPad[1];

    int numFeatures = modelH / shrink*modelW / shrink*nChns;

    for(uint32_t s = 0; s < scalePars.size(); s++)
    {
        const int height = scalePars[s].height/mdOpts.shrink;
        const int width  = scalePars[s].width/mdOpts.shrink;

        const int height_steps = (int)ceil(float(height*shrink - modelH + 1) / stride);
        const int width_steps  = (int)ceil(float(width*shrink - modelW + 1) / stride);
        printf("scale size width:  %4d  height:      %4d", width, height);
        printf("search step width: %3d  step height: %3d", width_steps,  height_steps);
        printf("numFeatures:       %4d  mnTrees:      %4d mnTreeNodes %3d \n", numFeatures, mnTrees,  mnTreeNodes);
    }
}

// NMS on detected bounding box to clean up final result

void AggCFDetector::ApplyPedDetectionNMS(std::vector<bbs> &aBboxes, int aGreedy )
{
    double overlap = mdOpts.overlap;
    std::sort(aBboxes.begin(), aBboxes.end(), [](const bbs a, const bbs b) {return a.score > b.score; });
    std::vector<int> kp(aBboxes.size(), 1);
    std::vector<double> as, xe, ye;

    for (uint32_t i = 0; i < aBboxes.size(); ++i)
    {
        as.push_back(aBboxes[i].w * aBboxes[i].h);
        xe.push_back(aBboxes[i].x + aBboxes[i].w);
        ye.push_back(aBboxes[i].y + aBboxes[i].h);
    }

    for (uint32_t i = 0; i < aBboxes.size(); ++i)
    {
        if (aGreedy && !kp[i])
            continue;

        for (uint32_t j = i + 1; j < aBboxes.size(); ++j)
        {
            if (kp[j] == 0)
                continue;

            double iw = std::min(xe[i], xe[j]) - std::max(aBboxes[i].x, aBboxes[j].x);
            if (iw <= 0)
                continue;

            double ih = std::min(ye[i], ye[j]) - std::max(aBboxes[i].y, aBboxes[j].y);
            if (ih <= 0)
                continue;

            double o = iw*ih;

            double u = std::min(as[i], as[j]);

            o /= u;

            if (o > overlap)
                kp[j] = 0;
        }
    }

    int index = 0;
    for (std::vector<bbs>::iterator it = aBboxes.begin(); it != aBboxes.end();)
    {
        if (!(kp[index]))
        {
            it = aBboxes.erase(it);
        }
        else
        {   
            ++it;
        }  
        ++index;
    }

}

// apply detection on calculated channel features

std::vector<bbs> AggCFDetector::ApplyPedDetectionDET(vsdk::SUMat *apChannelFeatures)
{

    // detection parameters 
//    const int nChns     = 4 + mdOpts.nOrients;
    const int stride    = mdOpts.stride;
    const int shrink    = mdOpts.shrink;
    const float cascThr = mdOpts.cascThr;
    //  search window size
    const float modelH  = mdOpts.modelDsPad[0];
    const float modelW  = mdOpts.modelDsPad[1];

    std::vector<bbs> allBBS; 

    // number of features
    // in a 64*32 bin = 2 setting,
    // 64*32/4*10 = 5120, each search window contains a array length of 5120 as feature
//    int numFeatures = modelH / shrink*modelW / shrink*nChns;

    // feature index array
    uint32_t *cids;

    for(uint32_t s = 0; s < scalePars.size(); s++)
    {
        int32_t scaleSpan = apChannelFeatures[s].step[0];

        if(scaleSpan != scalePars[s].spanSize)
        {
            updateScaleFidsArray( s, scaleSpan );
        }

        vsdk::SMat mattemp = apChannelFeatures[s].getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        uint16_t *inVaddr  = (uint16_t *)mattemp.data;

        // convert fixed point channels feature to float point column major 
        const int height = scalePars[s].height/mdOpts.shrink;
        const int width  = scalePars[s].width/mdOpts.shrink;
        uint16_t *chns = inVaddr;

        cids = mpChannelIds[s];

        // width/height in search step
        const int height_steps = (int)ceil(float(height*shrink - modelH + 1) / stride);
        const int width_steps  = (int)ceil(float(width*shrink - modelW + 1) / stride);

        std::vector<int> rs, cs; std::vector<float> hs1;
        for (int c = 0; c < width_steps; c++)
        {
            for (int r = 0; r < height_steps; r++)
            {
                double h = 0;
                // locate the search starting position
                uint16_t *chns1 = chns + (r*stride / shrink)*scaleSpan/sizeof(uint16_t)+ (c*stride / shrink);

                if (mTreeDepth == 1)
                {
                    for (int t = 0; t < mnTrees; t++)
                    {
                        uint32_t offset = t*mnTreeNodes, k = offset, k0 = 0;

                        // mpFids[k]                          feature id
                        // cids[mpFids[k]]                    index of feature in calculated feature array
                        // chns1[cids[mpFids[k]]]             featue value   
                        // chns1[cids[mpFids[k]]] < thrs[k]   decide to go left or right child
                       
                        getChild(chns1, cids, mpFids, mpFixThrs, offset, k0, k); 
                        // accunuate values from each decision stump
                        h += mpHs[k];

                        // early terminate if accumulation is less than cascade threshold 
                        if (h <= cascThr)
                            break;
                    }
                }
                else if(mTreeDepth == 2)
                {
                    for(int t = 0; t < mnTrees; t++)
                    {
                        uint32_t offset = t*mnTreeNodes, k = offset, k0 = 0;
                        getChild(chns1, cids, mpFids, mpFixThrs, offset, k0, k);
                        getChild(chns1, cids, mpFids, mpFixThrs, offset, k0, k);
                        h += mpHs[k];
                        if (h <= cascThr)
                            break;
                    }

                }
                else if(mTreeDepth > 2)
                {
                    for (int t = 0; t < mnTrees; t++)
                    {
                        uint32_t offset = t*mnTreeNodes, k = offset, k0 = 0;
                        for (int i = 0; i < mTreeDepth; i++)
                        {
                            getChild(chns1, cids, mpFids, mpFixThrs, offset, k0, k);
                        } 
                        h += mpHs[k];
                        if (h <= cascThr)
                            break;
                    }
                }
                else
                {
                    for (int t = 0; t < mnTrees; t++)
                    {
                        uint32_t offset = t*mnTreeNodes, k = offset, k0 = k;
                    
                        // loop until no mpChild node exists
                        while (mpChild[k])
                        {
                            uint16_t ftr = chns1[cids[mpFids[k]]];
                            k = (ftr < mpFixThrs[k]) ? 1 : 0;
                            k0 = k = mpChild[k0] - k + offset;
                        }
                        h += mpHs[k];
                        if (h <= cascThr)
                            break;
                    }
                }
                // find one candidate
                if (h>cascThr)
                {
                    // location
                    cs.push_back(c);
                    rs.push_back(r);
                    // confidence
                    hs1.push_back(h);
                }
            }
        }
        //delete[] chns;

        int nBBS = cs.size();
        float scale = scalePars[s].scaleRatio;

        // convert to resized bbs
        for (int i = 0; i < nBBS; i++)
        {
             
            //printf(" scale %f, stride %d, %d, %d, %f, %f, %f \n", scale, stride,  cs[i] , rs[i] , mdOpts.modelDsPad[1] , mdOpts.modelDsPad[0] ,  hs1[i]  ); 
            bbs currBBS;
            currBBS.x = (int)((cs[i] * stride) / scale);
            currBBS.y = (int)((rs[i] * stride) / scale);
            currBBS.w = (int)(mdOpts.modelDsPad[1] / scale/4*3);
            currBBS.x += currBBS.w/8;
            currBBS.h = (int)(mdOpts.modelDsPad[0] / scale);
            currBBS.score = hs1[i];
            allBBS.push_back(currBBS);
        }
    }
    
    return allBBS;
}

}



