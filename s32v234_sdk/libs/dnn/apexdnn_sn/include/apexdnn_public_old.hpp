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
 *  @file apexdnn_public.hpp
 *  @brief ApexCV Neural Networks library.
 **********************************************************************************/

/**
 * \defgroup UserAPI
 * This is the group of enum, structure and functions needs to be exposed to APEX-DNN library user
 */


#ifndef APEXDNN_HPP
#define APEXDNN_HPP

#include <iostream>

#include "ADVANCED_E1E3S1.hpp"
#include "oal.h"

#define APEXDNN_TENSOR_DIM_MAX                  8
#define APEXDNN_NET_LAYERS_MAX                  50
#define APEXDNN_DEFAULT_OAL_MEMORY_LEFT_PADDING 16
#define APEXDNN_MAX_SUPPORTED_H_DEPENDENCY      1


/*!*********************************************************************************
 * \ingroup UserAPI
 * \brief APEX-DNN supported layer and module types
 *
 * apexdnnLayerType_t is a enumerated type used to declares all APEX-DNN library supported layers and modules. 
 * We use term "LAYER" if one only one operation, such as convolution, max pooling or ReLu, is executed; we use term "MODULE" 
 * if multiple operations are executed. 
 ***********************************************************************************/
typedef enum
{
   APEXDNN_LAYER_UNKNOWN           = -1 /*!< Undefined type */,
   APEXDNN_MODULE_TYPE_CONV3X3MPS1 =  0 /*!< Module includes 3x3 convolution filter, followed by max pooling filter then squeeze 1x1 convolution filter */,
   APEXDNN_MODULE_TYPE_E1E3MPS1    =  1 /*!< Module includes expand 1x1 convolution filter and expand 3x3 convolution filter followed by max pooling filter then squeeze 1x1 convolution filter */,
   APEXDNN_MODULE_TYPE_E1E3S1      =  2 /*!< Module includes expand 1x1 convolution filter and expand 3x3 convolution filter followed by squeeze 1x1 convolution filter */,
   APEXDNN_MODULE_TYPE_E1E3        =  3 /*!< Module includes expand 1x1 convolution filter and expand 3x3 convolution filter */,
   APEXDNN_MODULE_TYPE_CONV1X1SP   =  4 /*!< Module includes 1x1 convolution filter followed by sum pooling filter */,
}  apexdnnLayerType_t;

/*!*********************************************************************************
 * \ingroup UserAPI
 * \brief APEX-DNN status
 *
 * apexdnnStatus_t is a enumerated type used to declare function returned status.
 ***********************************************************************************/
typedef enum
{
   APEXDNN_STATUS_SUCCESS        = 0 /*!< The operation completed successfully */,
   APEXDNN_STATUS_BAD_PARAM      = 1 /*!< An incorrect value or parameter was passed to the function */,
   APEXDNN_STATUS_ALLOC_FAILED   = 2 /*!< Memory allocation failed */,
   APEXDNN_STATUS_INTERNAL_ERROR = 3 /*!< An internal apexdnn executation failed */,
} apexdnnStatus_t;


/*!*********************************************************************************
 * \ingroup UserAPI
 * \brief APEX-DNN supported tensor data type
 *
 * apexdnnTensorDataType_t is a enumerated type used to declare supported data types.
 ***********************************************************************************/
typedef enum
{
   APEXDNN_DATA_8BIT  = 0    /*!< Fixed point 8-bit, the only one supported for now */,
   APEXDNN_DATA_16BIT = 1    /*!< Fixed point 16-bit, not supported yet */,
}  apexdnnTensorDataType_t;


/*!*********************************************************************************
 * \ingroup UserAPI
 * \brief APEX-DNN supported tensor layout
 *
 * apexdnnTensorFormat_t is a enumerated type used to declare all supported data layout.
 ***********************************************************************************/
typedef enum
{
   APEXDNN_TENSOR_UNKNOWN = -1 /*!< Unknown */, 
   APEXDNN_TENSOR_NCHW    = 0  /*!< This tensor format specifies that the data layout as batches, channels, rows and columns. Column is the inner most dimension and batches is the outer most dimension */,
   APEXDNN_TENSOR_NHCW    = 1  /*!< This tensor format specifies that the data layout as batches, rows, channels and columns. Column is the inner most dimension and batches is the outer most dimension */ ,
}  apexdnnTensorFormat_t;

/*!*********************************************************************************
 * \ingroup UserAPI
 * \brief APEX-DNN supported memory the tensor will be allocated on
 *
 * apexdnnTensorMemory_t is a enumberated type used to declare on which memory partation the tensor will be allocated.
 ***********************************************************************************/
typedef enum
{
   APEXDNN_TENSOR_MEM_HEAP = 0 /*!< regular heap memory */,
   APEXDNN_TENSOR_MEM_OAL  = 1 /*!< OAL memory */,
}  apexdnnTensorMemory_t;


/*!*********************************************************************************
 * \ingroup UserAPI
 * \brief APEX-DNN supported show level
 *
 * apexdnnShow_Lvl is a enumberated type used to declare different show level when user call Show routine on network, tensor or each layer/module for debugging purpose.
 ***********************************************************************************/
typedef enum
{
   APEXDNN_SHOW_SIMPLE  = 0 /*!< Basic level to display most critical information only */,
   APEXDNN_SHOW_VERBOSE = 1 /*!< Verbose level */,
   APEXDNN_SHOW_SUPER   = 2 /*!< Unused yet */,
} apexdnnShow_Lvl;

/*!*********************************************************************************
 * \ingroup UserAPI
 * \brief APEX DNN Workspace.
 *
 * APEX-DNN workspace maintain the temporary memory used by ApexDNN library
 * and APEX and APU ID assigned for APEX-DNN routines. All the associated APEX-DNN 
 * function calls will be executed by the appointed APEX / APU HW.
 ***********************************************************************************/
struct apexdnnWorkSpaceHandle
{
   ACF_APU_CFG mApuConfig; /*!< Apu configuration */
   int         mApexID;    /*!< APEX ID */
   void        *mWorkSpace; /*!< Pointer point to the temporary memory space */
   int         mSize;       /*!< Size of the temporary memory */
};

/*!*********************************************************************************
 * \ingroup UserAPI
 * \brief APEX DNN Tensor Descriptor.
 ***********************************************************************************/
struct apexdnnTensorDescriptor
{

   apexdnnTensorDataType_t mDataType; /*!< Data type, only 8-bit is supported for now */
   apexdnnTensorFormat_t   mFormat;   /*!< Tensor layout format */
   apexdnnTensorMemory_t   mMemory;   /*!< On which memory the tensor will be allocated */
   OAL_MEMORY_FLAG         mOalMemFlag; /*!< OAL memory flag to indicate whether the memory is contiguous and cache aligned */

   int  mNumofDims;                     /*!< Number of dimensions. Only 4-D tensor is supported for now */
   int  mDims[APEXDNN_TENSOR_DIM_MAX];  /*!< Integers to define the size of each dimension. In the decreased order of their respective strides */ 
   int  mStrides[APEXDNN_TENSOR_DIM_MAX]; /*!< Integers to define the stride of each dimension. Only positive stride is supported */

   void *mDataPtr;                        /*!< Tensor memory's pointer */
   int  mSize;                            /*!< Size of allocated memory, which at least mDims[0] * mStrides[0] */

}; 

/*!****************************************************************************
 * \ingroup UserAPI
 *  \brief Create APEX-DNN library's workspace 
 *
 *  This is an interface for creating the APEX-DNN library's workspace by allocating the memory needed to hold its data structure.
 ******************************************************************************/
apexdnnStatus_t apexdnnCreateWorkSpace(
                         apexdnnWorkSpaceHandle** Workspace /*!< Workspace handle */,
                         ACF_APU_CFG mApuConfig              /*!< Apu configuration */,
                         int         mApexID                 /*!< APEX ID */
                     );

/*!****************************************************************************
 * \ingroup UserAPI
 *  \brief Destroy APEX-DNN library's workspace and free up the workspace reserved memory
 *
 *  This is an interface for destroying the APEX-DNN library's workspace and free up the reserved memory.
 ******************************************************************************/
apexdnnStatus_t apexdnnDestroyWorkSpace(
                         apexdnnWorkSpaceHandle* Workspace  /*!< Workspace handle */
                     );

/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Create APEX-DNN library's 4-D tensor 
 *
 * This is an interface for creating the 4-D tensor descriptor by allocating the memory needed to hold its data structure. The tensor 
 * can have padding on each dimension when the stride is greather than product of the dimension and the stride of the
 * next dimension (stride[n] > stride[n+1] * dim[n+1]).
 ******************************************************************************/
apexdnnStatus_t apexdnnCreate4dTensorDescriptor(   
                         apexdnnTensorDescriptor** TensorDesc /*!< Pointer point to the tensor's handle */,
                         apexdnnTensorDataType_t   DataType   /*!< Tensor data type, only 8-bit is supported for now */,
                         apexdnnTensorFormat_t     Format     /*!< Tensor layout format */,
                         apexdnnTensorMemory_t     Memory     /*!< Tensor memory */,
                         int dim0                             /*!< Size of outer most dimension */,
                         int dim1                             /*!< Size of second dimension */,
                         int dim2                             /*!< Size of third dimension */,
                         int dim3                             /*!< Size of inner most dimension */,
                         int stride0                          /*!< Outer most dimension's stride */,
                         int stride1                          /*!< Second dimension's stride */,
                         int stride2                          /*!< Third dimension's stride */,
                         int stride3                          /*!< Inner most dimension's stride */
                     );

/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Create APEX-DNN library's 4-D compact tensor 
 *
 * This is an interface for creating the 4-D tensor descriptor by allocating the memory needed to hold its data structure. 
 * This will create a compact tensor, ie. the stride of one dimension equal to the product of the dimension and the stride of the 
 * next dimension (stride[n] = stride[n+1] * dim[n+1]).
 ******************************************************************************/
apexdnnStatus_t apexdnnCreate4dTensorDescriptor(   
                         apexdnnTensorDescriptor** TensorDesc /*!< Pointer point to the tensor's handle */,
                         apexdnnTensorDataType_t   DataType   /*!< Tensor data type, only 8-bit is supported for now */,
                         apexdnnTensorFormat_t     Format     /*!< Tensor layout format */,
                         apexdnnTensorMemory_t     Memory     /*!< Tensor memory */,
                         int dim0                             /*!< Size of outer most dimension */,
                         int dim1                             /*!< Size of second dimension */,
                         int dim2                             /*!< Size of third dimension */,
                         int dim3                             /*!< Size of inner most dimension */
                     );

/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Fill in tensor's memory 
 *
 * This is an interface to fillin tenor's memory
 ******************************************************************************/
apexdnnStatus_t apexdnnFill4dTensorDescriptor(
                         apexdnnTensorDescriptor* TensorDesc /*!< Tensor descriptor needs to be filled in */,
                         void* Value                         /*!< Memory buffer which hold the values needed by the tensor */,
                         int*  FilledSize                    /*!< Size of value buffer */
                     );

/*!****************************************************************************
 * \ingroup UserAPI
 * \brief display tensor descriptor
 *
 * This is an interface to display tensor descriptor for debugging purpose 
 ******************************************************************************/
void apexdnnTensorDescriptorShow(
                         apexdnnTensorDescriptor* Tensor /*!< Tensor descriptor needs to be displayed */
                     );


/*!****************************************************************************
 * \ingroup UserAPI
 * \brief destroy tensor descriptor
 *
 * This is an interface to destroy tensor descriptor. Tensor's memory will be freed. 
 ******************************************************************************/
apexdnnStatus_t apexdnnDestroyTensorDescriptor(
                         apexdnnTensorDescriptor** TensorDesc
                     );

apexdnnStatus_t apexdnnFillConvolution2dDescriptor(
                         apexdnnConvolutionDescriptor* ConvDesc,
                         void* Weigh,
                         int*  FilledSize
                     );

/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Create Conv3x3MPS1 convolution network module 
 *
 * This is an interface to create a convolution network module which includes 3x3 convolution filter, followed by
 * max pooling filter then squeeze 1x1 convolution filter.
 ******************************************************************************/
apexdnnStatus_t apexdnnCreateConv3x3MPS1Module(
                          apexdnnConv3x3MPS1Module** Module    /*!< Pointer which points to the module's handle */,
                          apexdnnTensorDataType_t    DataType  /*!< Data type, only support 8-bit for now */,
                          int wConv3x3Dim0                     /*!< 3x3 convolution filter's number of output channels */,   
                          int wConv3x3Dim1                     /*!< 3x3 convolution filter's number of input channels */,
                          int Conv3x3PadH                      /*!< 3x3 convolution filter's vertical padding */,    
                          int Conv3x3PadW                      /*!< 3x3 convolution filter's horizontal padding */,
                          int Conv3x3StrideH                   /*!< 3x3 convolution filter's vertical stride */, 
                          int Conv3x3StrideW                   /*!< 3x3 convolution filter's horizontal stride */,
                          int MPWindowH                        /*!< Maxpooling filter's vertical window size */,      
                          int MPWindowW                        /*!< Maxpooling filter's horizontal window size */,
                          int MPPadH                           /*!< Maxpooling filter's vertical padding */,         
                          int MPPadW                           /*!< Maxpooling filter's horizontal padding */,
                          int MPStrideH                        /*!< Maxpooling filter's vertical stride */,      
                          int MPStrideW                        /*!< Maxpooling filter's horizontal stride */,
                          int mS1Dim0                          /*!< 1x1 convolution filter's number of output channels */,
                          int ConvS1StrideH                    /*!< 1x1 convolution filter's vertical stride */,  
                          int COnvS1StrideW                    /*!< 1x1 convolution filter's horizontal stride */
                      );

/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Fill in Conv3x3MPS1 module's weight, bias and quantization parameters
 *
 * This is an interface to fill in the module's weight by using the pre-loaded buffer. 
 * All the weight, bias and quantization paramters are in 8-bit.
 * The buffer is organized as following: \n
 * <pre>
 * 3x3 filter's weight                      : 3 x 3 x wConv3x3Dim0 x wConv3x3Dim1 BYTES \n
 * 3x3 filter's bias                        : wConv3x3Dim0 BYTES \n
 * 3x3 filters quantization paramter        : 3 BYTES as [FL_PARAMS, FL_LAYER_IN, FL_LAYER_OUT]\n
 * squeeze 1x1 filter's weight              : wConv3x3Dim0 * wS1Dim0 BYTES \n
 * squeeze 1x1 filter's bias                : mS1Dim0 BYTES \n
 * squeeze 1x1 filters quantization paramter: 3 BYTES as [FL_PARAMS, FL_LAYER_IN, FL_LAYER_OUT]
 * </pre>
 ******************************************************************************/
apexdnnStatus_t apexdnnFillConv3x3MPS1Module(
                               apexdnnConv3x3MPS1Module* module /*!< Module's handle */,
                               void* Weight                     /*!< Buffer which holding the weight */
                           );


/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Create E1E3MPS1 convolution network module 
 *
 * This is an interface to create a convolution network module which includes expand 1x1 convolution filter and 
 * expand 3x3 convolution filter followed by max pooling filter then squeeze 1x1 convolution filter.
 ******************************************************************************/
apexdnnStatus_t apexdnnCreateE1E3MPS1Module(
                               apexdnnE1E3MPS1Module**  Module   /*!< Pointer which points to module's handle */,
                               apexdnnTensorDataType_t DataType  /*!< Data type, only support 8-bit for now */,
                               int wE1Dim0                       /*!< 1x1 convolution filter's number of output channels */,
                               int wE1Dim1                       /*!< 1x1 convolution filter's number of input channels */,
                               int E1StrideH                     /*!< 1x1 convolution filter's vertical stride */, 
                               int E1StrideW                     /*!< 1x1 convolution filter's horizontal stride */, 
                               int wE3Dim0                       /*!< 3x3 convolution filter's number of output channels */,   
                               int wE3Dim1                       /*!< 3x3 convolution filter's number of input channels */, 
                               int E3PadH                        /*!< 3x3 convolution filter's vertical padding */,    
                               int E3PadW                        /*!< 3x3 convolution filter's horizontal padding */,
                               int E3StrideH                     /*!< 3x3 convolution filter's vertical stride */, 
                               int E3StrideW                     /*!< 3x3 convolution filter's horizontal stride */,
                               int MPWindowH                     /*!< Maxpooling filter's vertical window size */,      
                               int MPWindowW                     /*!< 3x3 convolution filter's horizontal stride */,
                               int MPPadH                        /*!< Maxpooling filter's vertical padding */,
                               int MPPadW                        /*!< Maxpooling filter's horizontal padding */,
                               int MPStrideH                     /*!< Maxpooling filter's vertical stride */,      
                               int MPStrideW                     /*!< Maxpooling filter's horizontal stride */,
                               int wS1Dim0                       /*!< 1x1 convolution filter's number of output channels */,   
                               int S1StrideH                     /*!< 1x1 convolution filter's vertical stride */, 
                               int S1StrideW                     /*!< 1x1 convolution filter's horizontal stride */
                           );


/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Fill in E1E3MPS1 module's weight, bias and quantization parameters
 *
 * This is an interface to fill in the modules weight by using the pre-loaded buffer. All the weight, bias and quantization paramters are in 8-bit.
 * The buffer is organized as following: \n
 * <pre>
 * expand 1x1 filter's weight               : wE1Dim0 x wE1Dim1 BYTES \n
 * expand 1x1 filter's bias                 : wE1Dim0 BYTES \n
 * expand 1x1 filters quantization paramter : 3 BYTES as [FL_PARAMS, FL_LAYER_IN, FL_LAYER_OUT]\n
 * expand 3x3 filter's weight               : 3 x 3 x wE3Dim0 x wE3Dim1 BYTES\n
 * expand 3x3 filter's bias                 : wE3Dim0 BYTES \n
 * expand 3x3 filters quantization paramter : 3 BYTES as [FL_PARAMS, FL_LAYER_IN, FL_LAYER_OUT]\n
 * squeeze 1x1 filter's weight              : (wE1Dim0 + wE3Dim0) * wS1Dim0 BYTES \n
 * squeeze 1x1 filter's bias                : wS1Dim0 BYTES\n
 * squeeze 1x1 filters quantization paramter: 3 BYTES [FL_PARAMS, FL_LAYER_IN, FL_LAYER_OUT]
 * </pre>
 ******************************************************************************/
apexdnnStatus_t apexdnnFillE1E3MPS1Module(
                               apexdnnE1E3MPS1Module* module /*!< Module's handle */,
                               void* Weight                  /*!< Buffer which holding the weight */
                           );
/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Display E1E3MPS1 module
 *
 * This is a debugging helper function to show the module's each filter's configuration.
 ******************************************************************************/
void apexdnnE1E3MPS1ModuleShow(
                               apexdnnE1E3MPS1Module *Module /*!< Module's handle */
                           );

/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Create E1E3S1 convolution network module 
 *
 * This is an interface to create a convolution network module which includes expand 1x1 convolution filter and 
 * expand 3x3 convolution filter followed by squeeze 1x1 convolution filter.
 ******************************************************************************/
apexdnnStatus_t apexdnnCreateE1E3S1Module(   
                                apexdnnE1E3S1Module**   Module    /*!< Pointer which points to the module's handle */, 
                                apexdnnTensorDataType_t DataType  /*!< Data type, only support 8-bit for now */,
                                int wE1Dim0                       /*!< 1x1 convolution filter's number of output channels */,   
                                int wE1Dim1                       /*!< 1x1 convolution filter's number of input channels */,
                                int E1StrideH                     /*!< 1x1 convolution filter's vertical stride */, 
                                int E1StrideW                     /*!< 1x1 convolution filter's horizontal stride */,
                                int wE3Dim0                       /*!< 3x3 convolution filter's number of output channels */,   
                                int wE3Dim1                       /*!< 3x3 convolution filter's number of input channels */,
                                int E3PadH                        /*!< 3x3 convolution filter's vertical padding */,    
                                int E3PadW                        /*!< 3x3 convolution filter's horizontal padding */,
                                int E3StrideH                     /*!< 3x3 convolution filter's vertical stride */, 
                                int E3StrideW                     /*!< 3x3 convolution filter's horizontal stride */,
                                int wS1Dim0                       /*!< 1x1 convolution filter's number of output channels */,   
                                int S1StrideH                     /*!< 1x1 convolution filter's vertical stride */, 
                                int S1StrideW                     /*!< 1x1 convolution filter's horizontal stride */
                           );

/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Fill in E1E3S1 module's weight, bias and quantization parameters
 *
 * This is an interface to fill in the modules weight by using the pre-loaded buffer. All the weight, bias and quantization paramters are in 8-bit.
 * The buffer is organized as following: \n
 * <pre>
 * expand 1x1 filter's weight               : wE1Dim0 x wE1Dim1 BYTES\n
 * expand 1x1 filter's bias                 : wE1Dim0 BYTES\n
 * expand 1x1 filters quantization paramter : 3 BYTES as [FL_PARAMS, FL_LAYER_IN, FL_LAYER_OUT]\n
 * expand 3x3 filter's weight               : 3 x 3 x wE3Dim0 x wE3Dim1 BYTES\n
 * expand 3x3 filter's bias                 : wE3Dim0 BYTES\n
 * expand 3x3 filters quantization paramter : 3 BYTES as [FL_PARAMS, FL_LAYER_IN, FL_LAYER_OUT]\n
 * squeeze 1x1 filter's weight              : (wE1Dim0 + wE3Dim0) * wS1Dim0 BYTES\n
 * squeeze 1x1 filter's bias                : wS1Dim0 BYTES\n
 * squeeze 1x1 filters quantization paramter: 3 BYTES as [FL_PARAMS, FL_LAYER_IN, FL_LAYER_OUT]
 * </pre>
 ******************************************************************************/
apexdnnStatus_t apexdnnFillE1E3S1Module(
                                apexdnnE1E3S1Module* Module /*!< Module's handle */,
                                void* Weight                /*!< Buffer which holding the weight */
                           );
/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Display E1E3S1 module
 *
 * This is a debugging helper function to show the module's each filter's configuration.
 ******************************************************************************/
void apexdnnE1E3S1ModuleShow(
                                apexdnnE1E3S1Module* Module  /*!< Module's handle */
                           );
 
/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Create E1E3 convolution network module 
 *
 * This is an interface to create a convolution network module which includes expand 1x1 convolution filter and 
 * expand 3x3 convolution filter.
 ******************************************************************************/
apexdnnStatus_t apexdnnCreateE1E3Module(
                                apexdnnE1E3Module**     Module    /*!< Pointer which points to the module's handle */,
                                apexdnnTensorDataType_t DataType  /*!< Data type, only support 8-bit for now */,
                                int wE1Dim0                       /*!< 1x1 convolution filter's number of output channels */,   
                                int wE1Dim1                       /*!< 1x1 convolution filter's number of input channels */,
                                int E1StrideH                     /*!< 1x1 convolution filter's vertical stride */, 
                                int E1StrideW                     /*!< 1x1 convolution filter's horizontal stride */,
                                int wE3Dim0                       /*!< 3x3 convolution filter's number of output channels */,   
                                int wE3Dim1                       /*!< 3x3 convolution filter's number of input channels */,
                                int E3PadH                        /*!< 3x3 convolution filter's vertical padding */,    
                                int E3PadW                        /*!< 3x3 convolution filter's horizontal padding */,
                                int E3StrideH                     /*!< 3x3 convolution filter's vertical stride */, 
                                int E3StrideW                     /*!< 3x3 convolution filter's horizontal stride */
                           );


/*!****************************************************************************
 * \ingroup UserAPI
 *  \brief Create convolution neural network
 *
 *  This is an interface for creating an empty convolution neural network by allocating the memory needed to hold its data structure.
 ******************************************************************************/
apexdnnStatus_t apexdnnCreateEmptyNet(
                                apexdnnNet** Net  /*!< Pointer which points to the network's handle */
                            );

/*!****************************************************************************
 * \ingroup UserAPI
 *  \brief Destroy convolution neural network
 *
 *  This is an interface for destroying an convolution neural network. All the layers and related tensors will be destroied and 
 *  allocated memory will be freed.
 ******************************************************************************/
apexdnnStatus_t apexdnnDestroyNet(
                                apexdnnNet* Net  /*!< Network's handle */
                            );


/*!****************************************************************************
 * \ingroup UserAPI
 *  \brief Add layer or module into network 
 *
 *  This is an interface to add the layer into network.
 ******************************************************************************/
apexdnnStatus_t apexdnnNetAppendLayer(
                                apexdnnNet* Net /*!< network's handle */,
                                void* Layer     /*!< Created layer needs to add into network */
                            );

/*!****************************************************************************
 * \ingroup UserAPI
 *  \brief Verify network
 *
 *  This is an interface to verify the network. It won't really perform forward calculation instead it will go through the network, 
 *  check the consistency and allocate intermediate tensor for each layer or module's output based on input tensor's resolution. 
 *  This only needs to execute once if no input resolution change. 
 *  The output tensor if provided will be connect to last layer. Otherwise it will be internally allocated.  
 ******************************************************************************/
apexdnnStatus_t apexdnnNetVerifyGraphApex(
                                apexdnnWorkSpaceHandle* Workspace             /*!< APEX work space handle */,
                                apexdnnNet* Net                               /*!< Network handle */,
                                apexdnnTensorDescriptor*  NetInputTensorDesc  /*!< Input tesnor */,
                                apexdnnTensorDescriptor** NetOutputTensorDesc /*!< Pointer which points to the output tesnor's handle */
                            );

/*!****************************************************************************
 * \ingroup UserAPI
 *  \brief Forward network
 *
 *  This is an interface to forward the network. 
 ******************************************************************************/

apexdnnStatus_t apexdnnNetForwardApex(
                                apexdnnWorkSpaceHandle* Workspace             /*!< APEX work space handle */,
                                apexdnnNet* Net                               /*!< Network handle */,
                                apexdnnTensorDescriptor*  NetInputTensorDesc  /*!< Input tesnor */,
                            );

/*!****************************************************************************
 * \ingroup UserAPI
 * \brief Display Netowork 
 *
 * This is a debugging helper function to show the network's configuration.
 ******************************************************************************/
void apexdnnNetShow(apexdnnNet* Net, apexdnnShow_Lvl Lvl = APEXDNN_SHOW_SIMPLE);


#endif /* APEXDNN_HPP */

