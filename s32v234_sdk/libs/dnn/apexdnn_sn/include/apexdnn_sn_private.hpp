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

/*!*********************************************************************************
 *  @file apexdnn_private.hpp
 *  @brief ApexCV Neural Networks library private header.
 **********************************************************************************/

#ifndef APEXDNNPRIVATE_HPP
#define APEXDNNPRIVATE_HPP

#include <iostream>

#include "CONV3X3MPS1_BATCH.hpp"
#include "CONV3X3MPS1_MODULE.hpp"
#include "E1E3MPS1_BATCH.hpp"
#include "E1E3MPS1_MODULE.hpp"
#include "E1E3MP_MODULE.hpp"
#include "E1E3S1_MODULE.hpp"
#include "E1E3_MODULE.hpp"
#include "E1AP_MODULE.hpp"
#include "ELTMULCRED_MODULE.hpp"

#include "oal.h"

#include "apexdnn_sn_public.hpp"

#define APEXDNN_TENSOR_DIM_MAX                  8
#define APEXDNN_NET_LAYERS_MAX                  50
#define APEXDNN_DEFAULT_MEMORY_LEFT_PADDING     16
#define APEXDNN_MAX_SUPPORTED_H_DEPENDENCY      1
#define APEXDNN_MAX_SUPPORTED_V_DEPENDENCY      1

#define APEXDNN_WEIGHT_BW                       64
#define APEXDNN_PARAMS_NUMS                     32
#define APEXDNN_MAX_TENSOR_SPLIT                2
#define APEXDNN_MAX_TENSOR_CONCAT               2

struct apexdnnWorkSpace
{
   ACF_APU_CFG mApuConfig;
   int         mApexID;
   void        *mWorkSpace;
   int         mSize;
};

struct apexdnnQuantParams
{
   int mFlParams; 
   int mFlLayerIn;
   int mFlLayerOut;
};

struct apexdnnTensorDescriptor
{
   apexdnnTensorDataType_t mDataType;
   apexdnnTensorFormat_t   mFormat;
   apexdnnTensorMemory_t   mMemory;
   OAL_MEMORY_FLAG         mOalMemFlag;

   int  mNumofDims;
   int  mDims[APEXDNN_TENSOR_DIM_MAX];
   int  mStrides[APEXDNN_TENSOR_DIM_MAX];

   void *mDataOal;
   void *mDataPtr;
   void *mOriPtr;
   int  mSize;

   vsdk::SUMat mInputUMat[APEXDNN_MAX_TENSOR_SPLIT];
   vsdk::SUMat mOutputUMat[APEXDNN_MAX_TENSOR_CONCAT];
}; 

struct apexdnnConvolutionDescriptor
{
   apexdnnTensorDescriptor *mWeightTensorDesc;
   apexdnnTensorDescriptor *mBiasTensorDesc;
   int mOutputChannel;
   int mInputChannel;
   int mWindowH;
   int mWindowW;
   int mPadH;
   int mPadW;
   int mStrideH;
   int mStrideW;

   apexdnnQuantParams mQuantParams;
};

struct apexdnnPoolingDescriptor
{
   apexdnnPoolingMode_t mMode;
   int                  mWindowH;
   int                  mWindowW;
   int                  mPadH;
   int                  mPadW;
   int                  mStrideH;
   int                  mStrideW;
};

struct apexdnnBaseLayer
{
   apexdnnLayerType_t mType;
   int mInTileHeight;
   int mOutTileHeight;

   vsdk::SUMat mParams;
   vsdk::SUMat mWeights;
   vsdk::SUMat mBias;

   ACF_Process_APU *mApexProcess;
   ACF_Process_APU *mApexProcessExt; // limited by CMEM, we need to split graph for this module
};

struct apexdnnConv3x3MPS1Module
{
   apexdnnBaseLayer             mBase;
   apexdnnConvolutionDescriptor *mConv3x3Desc;
   apexdnnPoolingDescriptor     *mPoolingDesc;
   apexdnnConvolutionDescriptor *mConvS1Desc; 
};

struct apexdnnE1E3MPS1Module
{
   apexdnnBaseLayer             mBase;
   apexdnnConvolutionDescriptor *mConvE1Desc;
   apexdnnConvolutionDescriptor *mConvE3Desc;
   apexdnnPoolingDescriptor     *mPoolingDesc;
   apexdnnConvolutionDescriptor *mConvS1Desc;
};

struct apexdnnE1E3MPModule
{
   apexdnnBaseLayer             mBase;
   apexdnnConvolutionDescriptor *mConvE1Desc;
   apexdnnConvolutionDescriptor *mConvE3Desc;
   apexdnnPoolingDescriptor     *mPoolingDesc;
   apexdnnConvolutionDescriptor *mConvS1Desc;
};

struct apexdnnE1E3S1Module
{
   apexdnnBaseLayer             mBase;
   apexdnnConvolutionDescriptor *mConvE1Desc;
   apexdnnConvolutionDescriptor *mConvE3Desc;
   apexdnnConvolutionDescriptor *mConvS1Desc;
};

struct apexdnnE1E3Module
{
   apexdnnBaseLayer             mBase;
   apexdnnConvolutionDescriptor *mConvE1Desc;
   apexdnnConvolutionDescriptor *mConvE3Desc;
};

struct apexdnnE1APModule
{
   apexdnnBaseLayer             mBase;
   apexdnnConvolutionDescriptor *mConvE1Desc;
   apexdnnPoolingDescriptor     *mPoolingDesc;

   apexdnnTensorDescriptor      *mAccmTensorDesc;

};

struct apexdnnEltMulCRedModule
{
   apexdnnBaseLayer mBase;
   apexdnnTensorDescriptor *mWeightTensorDesc;
   apexdnnTensorDescriptor *mBiasTensorDesc;
   apexdnnQuantParams mQuantParams;
};

struct apexdnnNet
{
   void* mLayers[APEXDNN_NET_LAYERS_MAX];
   apexdnnTensorDescriptor *mOutputTensorDesc[APEXDNN_NET_LAYERS_MAX];
   int   mNumofLayers;
   bool  mBatchProc;
   int32_t mApexId;

   bool mHasVerifiedCpu;
   bool mHasVerifiedApex;
};

apexdnnStatus_t apexdnnManualInit4dTensorDescriptor(
                         apexdnnTensorDescriptor* TensorDesc,
                         apexdnnTensorDataType_t  DataType,
                         apexdnnTensorFormat_t    Format,
                         apexdnnTensorMemory_t    Memory,
                         void* DataPtr,
                         int   dim0,
                         int   dim1,
                         int   dim2,
                         int   dim3
                     );

apexdnnStatus_t apexdnnFill4dTensorDescriptor(
                         apexdnnTensorDescriptor* TensorDesc,
                         void* Value,
                         int*  FilledSize
                     );

apexdnnStatus_t apexdnnTransform4dTensorDescriptor(
                         apexdnnTensorDescriptor *XTensorDesc,
                         apexdnnTensorDescriptor **YTensorDesc,
                         apexdnnTensorFormat_t   Format
                     );

apexdnnStatus_t apexdnnCreateConvolution2dDescriptor(   
                         apexdnnConvolutionDescriptor** ConvDesc,
                         apexdnnTensorDataType_t        DataType,
                         int wDim0,
                         int wDim1,
                         int wDim2,
                         int wDim3,
                         int PadH,
                         int PadW,
                         int StrideH,
                         int StrideW
                     );

apexdnnStatus_t apexdnnFillConvolution2dDescriptor(
                         apexdnnConvolutionDescriptor* ConvDesc,
                         void* Weigh,
                         int*  FilledSize
                     );

void apexdnnConvolutionDescriptorShow(
                         apexdnnConvolutionDescriptor* ConvDesc
                     );

apexdnnStatus_t apexdnnDestroyConvolution2dDescriptor(
                         apexdnnConvolutionDescriptor* ConvDesc
                     );

apexdnnStatus_t apexdnnCreatePooling2dDescriptor(
                         apexdnnPoolingDescriptor** PoolingDesc,
                         apexdnnPoolingMode_t       Mode,
                         int WindowH, 
                         int WindowW, 
                         int PadH,    
                         int PadW,
                         int StrideH, 
                         int StrideW
                     );

void apexdnnPoolingDescriptorShow(
                         apexdnnPoolingDescriptor* PoolingDesc
                     );

apexdnnStatus_t apexdnnDestroyPooling2dDescriptor(
                        apexdnnPoolingDescriptor* PoolingDesc
                     );

apexdnnStatus_t apexdnnFillConv3x3MPS1Module(
                        apexdnnConv3x3MPS1Module* Module,
                        void* Weight,
                        int*  FilledSize
                      );

apexdnnStatus_t apexdnnFillE1E3MPS1Module(
                        apexdnnE1E3MPS1Module* Module,
                        void* Weight,
                        int*  FilledSize
                      );

apexdnnStatus_t apexdnnFillE1E3MPModule(
                              apexdnnE1E3MPModule* Module,
                              void* Weight,
                              int*  FilledSize
                           );

apexdnnStatus_t apexdnnFillE1E3S1Module(
                              apexdnnE1E3S1Module* Module,
                              void* Weight,
                              int*  FilledSize
                           );

apexdnnStatus_t apexdnnFillE1E3Module(
                              apexdnnE1E3Module* Module,
                              void* Weight,
                              int*  FilledSize
                           );

apexdnnStatus_t apexdnnFillE1APModule(
                              apexdnnE1APModule* Module,
                              void* Weight,
                              int*  FilledSize
                           );

apexdnnStatus_t apexdnnFillEltMulCRedModule(
                              apexdnnEltMulCRedModule* Module,
                              void* Weight,
                              int*  FilledSize
                           );

apexdnnStatus_t apexdnnConv3x3MPS1ModuleDryrunForwardCpu(
                              apexdnnConv3x3MPS1Module* Module,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnTensorDescriptor** OutputTensorDesc
                           );

apexdnnStatus_t apexdnnE1E3MPS1ModuleDryrunForwardCpu(
                              apexdnnE1E3MPS1Module*    Module,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnTensorDescriptor** OutputTensorDesc
                           );

apexdnnStatus_t apexdnnE1E3MPModuleDryrunForwardCpu(
                              apexdnnE1E3MPModule*      Module,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnTensorDescriptor** OutputTensorDesc
                           );

apexdnnStatus_t apexdnnE1E3S1ModuleDryrunForwardCpu(
                              apexdnnE1E3S1Module*      Module,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnTensorDescriptor** OutputTensorDesc
                           );

apexdnnStatus_t apexdnnE1E3ModuleDryrunForwardCpu(
                              apexdnnE1E3Module*       Module,
                              apexdnnTensorDescriptor* InputTensorDesc,
                              apexdnnTensorDescriptor** OutputTensorDesc
                           );

apexdnnStatus_t apexdnnE1APModuleDryrunForwardCpu(
                              apexdnnE1APModule*        Module,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnTensorDescriptor** OutputTensorDesc
                           );

apexdnnStatus_t apexdnnEltMulCRedModuleDryrunForwardCpu(
                              apexdnnEltMulCRedModule*  Module,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnTensorDescriptor** OutputTensorDesc
                           );

apexdnnStatus_t apexdnnConv3x3MPS1ModuleForwardCpu(
                              apexdnnWorkSpace*  Workspace,
                              apexdnnConv3x3MPS1Module* Module,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnTensorDescriptor*  OutputTensorDesc
                           );

apexdnnStatus_t apexdnnE1E3MPS1ModuleForwardCpu(
                              apexdnnWorkSpace*  Workspace,
                              apexdnnE1E3MPS1Module*    Module,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnTensorDescriptor*  OutputTensorDesc
                           );

apexdnnStatus_t apexdnnE1E3S1ModuleForwardCpu(
                              apexdnnWorkSpace*  Workspace,
                              apexdnnE1E3S1Module*      Module,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnTensorDescriptor*  OutputTensorDesc
                           );

apexdnnStatus_t apexdnnE1E3MPModuleForwardCpu(
                              apexdnnWorkSpace*  Workspace,
                              apexdnnE1E3MPModule*      Module,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnTensorDescriptor*  OutputTensorDesc
                           );

apexdnnStatus_t apexdnnE1E3ModuleForwardCpu(
                              apexdnnWorkSpace*  Workspace,
                              apexdnnE1E3Module*       Module,
                              apexdnnTensorDescriptor* InputTensorDesc,
                              apexdnnTensorDescriptor* OutputTensorDesc
                           );

apexdnnStatus_t apexdnnE1APModuleForwardCpu(
                              apexdnnWorkSpace*  Workspace,
                              apexdnnE1APModule*       Module,
                              apexdnnTensorDescriptor* InputTensorDesc,
                              apexdnnTensorDescriptor* OutputTensorDesc
                           );

apexdnnStatus_t apexdnnConv3x3MPS1ModuleDryrunForwardApex(
                              apexdnnWorkSpace*  Workspace,
                              apexdnnTensorDescriptor*   InputTensorDesc,
                              apexdnnConv3x3MPS1Module*  Module,
                              apexdnnTensorDescriptor**  OutputTensorDesc,
                              apexdnnBaseLayer*          NextLayer,
                              bool BatchProc
                           );

apexdnnStatus_t apexdnnE1E3MPS1ModuleDryrunForwardApex(
                              apexdnnWorkSpace* Workspace,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnE1E3MPS1Module*    Module,
                              apexdnnTensorDescriptor** OutputTensorDesc,
                              apexdnnBaseLayer*         NextLayer,
                              bool BatchProc
                           );

apexdnnStatus_t apexdnnE1E3S1ModuleDryrunForwardApex(
                              apexdnnWorkSpace* Workspace,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnE1E3S1Module*      Module,
                              apexdnnTensorDescriptor** OutputTensorDesc,
                              apexdnnBaseLayer*         NextLayer,
                              bool BatchProc
                           );

apexdnnStatus_t apexdnnE1E3ModuleDryrunForwardApex(
                              apexdnnWorkSpace* Workspace,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnE1E3Module*        Module,
                              apexdnnTensorDescriptor** OutputTensorDesc,
                              apexdnnBaseLayer*         NextLayer,
                              bool BatchProc
                           );

apexdnnStatus_t apexdnnE1E3MPModuleDryrunForwardApex(
                              apexdnnWorkSpace* Workspace,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnE1E3MPModule*      Module,
                              apexdnnTensorDescriptor** OutputTensorDesc,
                              apexdnnBaseLayer*         NextLayer,
                              bool BatchProc
                           );

apexdnnStatus_t apexdnnEltMulCRedModuleDryrunForwardApex(
                              apexdnnWorkSpace* Workspace,
                              apexdnnTensorDescriptor*  InputTensorDesc,
                              apexdnnEltMulCRedModule*  Module,
                              apexdnnTensorDescriptor** OutputTensorDesc,
                              apexdnnBaseLayer*         NextLayer,
                              bool BatchProc
                           );

apexdnnStatus_t apexdnnConv3x3MPS1ModuleEstMemUsageApex(
                              apexdnnConv3x3MPS1Module*  Module
                           );

apexdnnStatus_t apexdnnE1E3MPS1ModuleEstMemUsageApex(
                              apexdnnE1E3MPS1Module*  Module
                           );

apexdnnStatus_t apexdnnE1E3S1ModuleEstMemUsageApex(
                              apexdnnE1E3S1Module*  Module
                           );

apexdnnStatus_t apexdnnE1E3ModuleEstMemUsageApex(
                              apexdnnE1E3Module*  Module
                           );

apexdnnStatus_t apexdnnE1APModuleEstMemUsageApex(
                              apexdnnE1APModule*  Module
                           );

apexdnnStatus_t apexdnnEltMulCRedModuleEstMemUsageApex(
                              apexdnnEltMulCRedModule*  Module
                           );

void apexdnnConv3x3MPS1ModuleShow(
                              apexdnnConv3x3MPS1Module* Module
                           );

void apexdnnE1E3MPS1ModuleShow(
                              apexdnnE1E3MPS1Module *Module
                           );

void apexdnnE1E3MPModuleShow(
                              apexdnnE1E3MPModule *Module
                           );

void apexdnnE1E3S1ModuleShow(
                              apexdnnE1E3S1Module* Module 
                           );

void apexdnnE1E3ModuleShow(
                              apexdnnE1E3Module* Module
                           );

void apexdnnE1APModuleShow(
                              apexdnnE1APModule* Module
                           );

#endif /* APEXDNNPRIVATE_HPP */
