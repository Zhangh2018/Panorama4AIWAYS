/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2018 NXP Semiconductor;
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

#ifndef _DATA_DESC_HPP
#define _DATA_DESC_HPP

#include <stdint.h>
#include <common.hpp>
#include <icp_datatype.h>  

//////////////////////////////////////////////////////////////////////////
/// DataDesc describes a contiguous region of memory.  It is a description only,
/// and is meant to be paired with a buffer that matches the description.
//////////////////////////////////////////////////////////////////////////
class DataDesc
{
public:
   
   //---------
   //methods
   //---------

   /////////////////////////////////////////////////////////////////////////
   /// Initialize N-dimensional data descriptor.  This function assumes element size
   /// is 1 and padding is 0 in all dimensions.
   ///
   /// \param[in] lNumDim
   /// Number of dimensions.
   ///
   /// \param[in] lDataType
   /// Data type (e.g. 8-bit signed, 16-bit unsigned, etc.).
   ///
   /// \param[in] lSize
   /// Array containing the data region size in each dimension.
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t InitND(int32_t lNumDim, icp::DATATYPE lDataType, int32_t lSize[]);

   /////////////////////////////////////////////////////////////////////////
   /// Initialize N-dimensional data descriptor.  This function allows specification
   /// of element and padding size information for more advanced use cases.
   ///
   /// \param[in] lNumDim
   /// Number of dimensions.
   ///
   /// \param[in] lDataType
   /// Data type (e.g. DATATYPE_08U, DATATYPE_16S, etc.).
   ///
   /// \param[in] lSize
   /// Array containing data region size in each dimension.
   ///
   /// \param[in] lElemSize
   /// Array containing element size in each dimension.  An 'element' can be seen as an indivisible
   /// unit within the data region.  In most cases, this unit will be something simple like a 16-bit value
   /// (e.g. lDataType = DATATYPE_16S and lElemSize will be 1 for all dimensions).  However, if you need to express
   /// something that doesn't map nicely to standard 8/16/32 bit types, the 'element' concept can be used.  For example,
   /// say you wanted to express 24-bit RGB data: you could express it as lDataType=DATATYPE_08U with lElemSize={3,1}.
   ///
   /// \param[in] lPad
   /// Array containing low and high padding information per dimension.  In this case, the 'core' region (described by lSize)
   /// has some padding around it as specified by "lPad".  This padding could be for alignment purposes, or could describe
   /// space that has been left in the buffer to perform edge padding.  For example, in a simple 256x128 2D image case,
   /// say a memory region of size (2+256+4)x(6+128+8) was allocated with the intention of leaving 2 and 4 columns of padding on the
   /// left and right, and 6 and 8 rows of padding on the top and bottom.  In this case, lPad={2,4,6,8}.
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t InitND(int32_t lNumDim, icp::DATATYPE lDataType, int32_t lSize[], int32_t lElemSize[], int32_t lPad[]);
   
   /////////////////////////////////////////////////////////////////////////
   /// Initialize 2D data descriptor.  This is a 2D version of InitND that is
   /// provided for convenience.  See the description of InitND for more information.
   /////////////////////////////////////////////////////////////////////////
   void Init2D(icp::DATATYPE lDataType,
               int32_t lSize0, int32_t lSize1);

   /////////////////////////////////////////////////////////////////////////
   /// Initialize 2D data descriptor.  This is a 2D version of InitND that is
   /// provided for convenience.  See the description of InitND for more information.
   /////////////////////////////////////////////////////////////////////////
   void Init2D(icp::DATATYPE lDataType,
               int32_t lSize0, int32_t lSize1,
               int32_t lElemSize0, int32_t lElemSize1,
               int32_t lPad0Low, int32_t lPad0High, int32_t lPad1Low, int32_t lPad1High);
   
   //---------
   //members
   //---------

   int32_t mNumDim;                        //number of dimensions
   icp::DATATYPE mDataType;                //8,16,32 signed/unsigned
   int32_t mElementSize[DESC_MAX_NUM_DIM]; //logical description of basic element (n-dimensional equivalent of ACF's e0)
   int32_t mSize[DESC_MAX_NUM_DIM];        //data size (in elements)
   int32_t mPad[DESC_MAX_NUM_DIM*2];       //padding beyond mSize in each dimension (i.e. low address & high address pair per dimension) (in elements)
   
   //values that can be derived from above info
   int32_t mElementStrideInElement[DESC_MAX_NUM_DIM];   //element stride in units of elements (i.e. number of elements to skip to reach the next element in a dimension)
   int32_t mElementStrideInDataType[DESC_MAX_NUM_DIM];  //element stride in units of data type (i.e. number of units of data type to skip to reach the next element in a dimension)
   int32_t mStrideInDataType[DESC_MAX_NUM_DIM];         //data type stride in units of data type (i.e. number of units of data type to skip to reach the next data type in a dimension)
   int32_t mTotalSizeInBytes;
   int32_t mTotalSizeInDataType;
   int32_t mCoreOffsetInDataType;
   
private:
   
   void DerivedInit();
};

//some debugging/helper functions (not officially exposed yet - use at own risk!)
int32_t DataDescEqualityCheck(DataDesc* lpDataDescA, void* lpCoreDataRegionA, DataDesc* lpDataDescB, void* lpCoreDataRegionB);
void DataDescElemReplicate2D(DataDesc* lpDataDesc, void* lpCoreDataRegion);

#endif //_DATA_DESC_HPP
