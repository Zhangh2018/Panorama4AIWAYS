/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2015-2016 Freescale Semiconductor
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

#ifndef LAYERS_HPP
#define LAYERS_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <acf_process_apu.h>

#include "blob.hpp"

enum POOLMETHOD_ENUM {POOLMETHOD_UNAVAIL=-1, POOLMETHOD_AVG=0, POOLMETHOD_MAX=1};

enum LAYER_TYPE_ENUM {LAYER_TYPE_NONE         =-1, LAYER_TYPE_INPUT          = 0, LAYER_TYPE_FIRE2_CONV1MPS1 = 1,
                      LAYER_TYPE_FIREX_E1E3S1 = 2, LAYER_TYPE_FIRE5_E1E3MPS1 = 3,  LAYER_TYPE_FIRE9_1HALVE   = 4,
                      LAYER_TYPE_FIRE9_2HALVE = 5, LAYER_TYPE_FIRE10_1HALVE  = 6,  LAYER_TYPE_FIRE10_2HALVE  = 7};

typedef struct FilterDim 
{
  int x; // filter x dim
  int y; // filter x dim
  int d; // filter depth
  int channels_per_cu; // # channels per CU
  int channels_all; // # channels per CU
} FilterDim;

template <typename Dtype>
class Layer 
{
public:
   Layer();
   ~Layer();
   int Forward();

   LAYER_TYPE_ENUM type_;
   bool bypass_;
   std::string layer_name_;
   std::vector<Blob<Dtype>*> top_vecs_;
   std::vector<Blob<Dtype>*> bottom_vecs_;
   std::vector<Blob<Dtype>*> weight_vecs_;
   std::vector<FilterDim> filter_dim_;

   int top_dep_;
   int bottom_dep_;

   vsdk::SUMat params_;

   ACF_Process_APU *apu_process_;
};


template <typename Dtype>
Layer<Dtype>::Layer() : type_(LAYER_TYPE_NONE), bypass_(false)
{
   params_ = vsdk::SUMat(1, 16, VSDK_CV_8SC1);
}

template <typename Dtype>
Layer<Dtype>::~Layer()
{
}

template <typename Dtype>
int Layer<Dtype>::Forward()
{
   int reval = 0;

   reval |= apu_process_->Start();
   reval |= apu_process_->Wait();

   return reval;
}

#endif /* LAYERS_HPP */
