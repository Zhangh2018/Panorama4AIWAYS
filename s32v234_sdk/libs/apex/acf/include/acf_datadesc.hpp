/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
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

#ifndef ACFDATADESC_HPP
#define ACFDATADESC_HPP

#include <icp_datatype.h>
#include <stdint.h>
#include <acf_common.h>

#define ACF_DATADESC_MIN_BYTE_ALIGNMENT 16 //choose from 4/8/16/32/64/128

/////////////////////////////////////////////////////////////////////////
/// \addtogroup acf_datadesc ACF_DataDesc
/// @{

//////////////////////////////////////////////////////////////////////////
/// ACF_DataDesc is a container class designed to encapsulate
/// a contiguous region of data.  It does not allocate or deallocate
/// memory; it simply standardizes the representation of a contiguous
/// memory region so it can be used by framework level services.
//////////////////////////////////////////////////////////////////////////
class ACF_DataDesc
{
public:

   ACF_DataDesc();
   ACF_DataDesc(const ACF_DataDesc &original);

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Constructor that initializes a contiguous 2D data region.  Note that
   /// the data region must be physically contiguous in memory (not just
   /// contiguous from the OS point of view).
   ///
   /// Note that an 'element' (as referred to below) simply represents the
   /// smallest meaningful data granularity.
   ///
   /// For example, if an element is a single 32-bit signed value:
   /// \code{.cpp}
   /// lElementDataType = icp::DATATYPE_32S
   /// lElementDimX     = 1
   /// lElementDimY     = 1
   /// \endcode
   ///
   /// If an element is an array of 4 8-bit unsigned values:
   /// \code{.cpp}
   /// lElementDataType = icp::DATATYPE_08U
   /// lElementDimX     = 4
   /// lElementDimY     = 1
   /// \endcode
   ///
   /// \param[in] lpData
   /// Pointer to contiguous data region.
   /// \param[in] lpDataPhys
   /// Physical pointer to contiguous data region (for HW use).
   /// \param[in] lWidth
   /// Width (in elements) of the contiguous data region.
   /// \param[in] lHeight
   /// Height (in elements) of the contiguous data region.
   /// \param[in] lSpan
   /// Span is defined as the number of bytes required to jump from one
   /// line of bytes in memory to the 'next' line of bytes in memory.  Note
   /// that span must be divisible by N where
   /// N = RetIcpDataTypeSizeInBytes (lElementDataType).
   /// \param[in] lElementDataType
   /// The data type associated with an 'element' (i.e. the smallest unit of data)
   /// \param[in] lElementDimX
   /// The 'x' dimension (i.e. width) of an element in units of "lElementDataType"
   /// \param[in] lElementDimY
   /// The 'y' dimension (i.e. height) of an element in units of "lElementDataType"
   /// \param[in] lRoiInfoL,lRoiInfoR,lRoiInfoT,lRoiInfoB allow the user to explicitly
   /// express whether the described data region should be treated as an ROI.  In the typical
   /// ROI use case, these values can be conceptually interpreted as the 'margins' between the edges
   /// of the described data region and the boundaries of the parent data region.
   /// For example, if lRoiInfoL=4, the user is telling ACF to be aware that there are 4 columns
   /// of data beyond the left edge of the described data region, and that ACF should use this
   /// data for padding purposes if spatial dependencies are present.
   /// On the other hand, if lRoiInfoL=0, ACF will assume there are no additional columns of
   /// data beyond the left edge of the described data region, and any necessary left edge padding
   /// will be generated (i.e. it will NOT come directly from the source data region).
   /// In general, if the intent is for an edge to be padded with source data (instead of generated
   /// padding) then the relevant lRoiInfo parameter should be set to non-zero accordingly.
   /////////////////////////////////////////////////////////////////////////
   ACF_DataDesc(void* const lpData,
                void* const lpDataPhys,
                int32_t lWidth,
                int32_t lHeight,
                int32_t lSpan,
                icp::DATATYPE lElementDataType,
                int32_t lElementDimX,
                int32_t lElementDimY,
                int32_t lRoiInfoL = 0,
                int32_t lRoiInfoR = 0,
                int32_t lRoiInfoT = 0,
                int32_t lRoiInfoB = 0);

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Constructor that initializes a contiguous 1D data region.  Note that
   /// the data region must be physically contiguous in memory (not just
   /// contiguous from the OS point of view).
   ///
   /// \param[in] lpData
   /// Pointer to contiguous data region.
   /// \param[in] lpDataPhys
   /// Physical pointer to contiguous data region (for HW use).
   /// \param[in] lSize
   /// Size (in elements) of the contiguous data region.
   /// \param[in] lElementDataType
   /// The data type associated with an 'element' (i.e. the smallest unit of data)
   /// \param[in] lElementDimX
   /// The 'x' dimension (i.e. width) of an element in units of "lElementDataType".
   /// It is assumed that the 'y' dimension of the element is 1.
   /////////////////////////////////////////////////////////////////////////
   ACF_DataDesc(void* const lpData,
                void* const lpDataPhys,
                int32_t lSize,
                icp::DATATYPE lElementDataType,
                int32_t lElementDimX);

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Constructor that initializes a contiguous 4D data region.  Note that
   /// the data region must be physically contiguous in memory (not just
   /// contiguous from the OS point of view).
   ///
   /// Note that an 'element' (as referred to below) simply represents the
   /// smallest meaningful data granularity.
   ///
   /// For example, if an element is a single 32-bit signed value:
   /// \code{.cpp}
   /// lElementDataType = icp::DATATYPE_32S
   /// lElementDimX     = 1
   /// lElementDimY     = 1
   /// \endcode
   ///
   /// If an element is an array of 4 8-bit unsigned values:
   /// \code{.cpp}
   /// lElementDataType = icp::DATATYPE_08U
   /// lElementDimX     = 4
   /// lElementDimY     = 1
   /// \endcode
   ///
   /// \param[in] lpData
   /// Pointer to contiguous data region.
   /// \param[in] lpDataPhys
   /// Physical pointer to contiguous data region (for HW use).
   /// \param[in] lChannel
   /// Channel (in elements) of the contiguous data region.
   /// \param[in] lWidth
   /// Width (in elements) of the contiguous data region.
   /// \param[in] lHeight
   /// Height (in elements) of the contiguous data region.
   /// \param[in] lChannel
   /// Batch (in elements) of the contiguous data region.
   /// \param[in] lSpanChanel
   /// Span is defined as the number of bytes required to jump from one
   /// \param[in] lSpanWidth
   /// Span is defined as the number of bytes required to jump from one
   /// \param[in] lSpanHeight
   /// Span is defined as the number of bytes required to jump from one
   /// \param[in] lSpanBatch
   /// Span is defined as the number of bytes required to jump from one
   /// line of bytes in memory to the 'next' line of bytes in memory.  Note
   /// that span must be divisible by N where
   /// N = RetIcpDataTypeSizeInBytes (lElementDataType).
   /// \param[in] lElementDataType
   /// The data type associated with an 'element' (i.e. the smallest unit of data)
   /// \param[in] lElementDimX
   /// The 'x' dimension (i.e. width) of an element in units of "lElementDataType"
   /// \param[in] lElementDimY
   /// The 'y' dimension (i.e. height) of an element in units of "lElementDataType"
   /// \param[in] lRoiInfoL,lRoiInfoR,lRoiInfoT,lRoiInfoB allow the user to explicitly
   /// express whether the described data region should be treated as an ROI.  In the typical
   /// ROI use case, these values can be conceptually interpreted as the 'margins' between the edges
   /// of the described data region and the boundaries of the parent data region.
   /// For example, if lRoiInfoL=4, the user is telling ACF to be aware that there are 4 columns
   /// of data beyond the left edge of the described data region, and that ACF should use this
   /// data for padding purposes if spatial dependencies are present.
   /// On the other hand, if lRoiInfoL=0, ACF will assume there are no additional columns of
   /// data beyond the left edge of the described data region, and any necessary left edge padding
   /// will be generated (i.e. it will NOT come directly from the source data region).
   /// In general, if the intent is for an edge to be padded with source data (instead of generated
   /// padding) then the relevant lRoiInfo parameter should be set to non-zero accordingly.
   /////////////////////////////////////////////////////////////////////////

   ACF_DataDesc(void* const lpData,
                void* const lpDataPhys,
                int32_t lChannel,
                int32_t lWidth,
                int32_t lHeight,
                int32_t lBatch,
                int32_t lChannelSpan,
                int32_t lWidthSpan,
                int32_t lHeightSpan,
                int32_t lBatchSpan,
                icp::DATATYPE lElementDataType,
                int32_t lElementDimX,
                int32_t lElementDimY,
                int32_t lRoiInfoL = 0,
                int32_t lRoiInfoR = 0,
                int32_t lRoiInfoT = 0,
                int32_t lRoiInfoB = 0);



   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Initializes a contiguous 2D data region.  See the description of the
   /// 2D constructor for parameter details (pay special attention to the 
   /// the meaning of lRoiInfoL, lRoiInfoR, lRoiInfoT, and lRoiInfoB).
   /////////////////////////////////////////////////////////////////////////
   void Init2D (void* const lpData,
                void* const lpDataPhys,
                int32_t lWidth,
                int32_t lHeight,
                int32_t lSpan,
                icp::DATATYPE lElementDataType,
                int32_t lElementDimX,
                int32_t lElementDimY,
                int32_t lRoiInfoL = 0,
                int32_t lRoiInfoR = 0,
                int32_t lRoiInfoT = 0,
                int32_t lRoiInfoB = 0);

   /////////////////////////////////////////////////////////////////////////
   /// Initializes a contiguous 2D data region as an ROI within the 'parent'
   /// region "lContigDataDesc"
   ///
   /// \param[in] lDataDesc
   /// Description of contiguous data region that 'contains' the ROI
   ///
   /// \param[in] lROI_XOffset
   /// The X offset of the top left corner of the ROI (relative to the top left
   /// corner of the region described by "lContigDataDesc").
   ///
   /// \param[in] lROI_YOffset
   /// The Y offset of the top left corner of the ROI (relative to the top left
   /// corner of the region described by "lContigDataDesc").
   ///
   /// \param[in] lROI_Width
   /// The width of the ROI.
   ///
   /// \param[in] lROI_Height
   /// The height of the ROI.
   ///
   /// \return
   /// 0 if successful, non-zero if an an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t Init2D_ROI(const ACF_DataDesc& lDataDesc,
                      int32_t lROI_XOffset,
                      int32_t lROI_YOffset,
                      int32_t lROI_Width,
                      int32_t lROI_Height);

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Initializes a contiguous 1D data region.  See the description of the
   /// 1D constructor for parameter details.
   /////////////////////////////////////////////////////////////////////////
   void Init1D (void* const lpData,
                void* const lpDataPhys,
                int32_t lSize,
                icp::DATATYPE lElementDataType,
                int32_t lElementDimX);

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Initializes a contiguous 4D data region.  See the description of the
   /// 4D constructor for parameter details (pay special attention to the 
   /// the meaning of lRoiInfoL, lRoiInfoR, lRoiInfoT, and lRoiInfoB).
   /////////////////////////////////////////////////////////////////////////

void Init4D(void* const lpData,
            void* const lpDataPhys,
            int32_t lChannel,
            int32_t lWidth,
            int32_t lHeight,
            int32_t lBatch,
            int32_t lChannelSpan,
            int32_t lWidthSpan,
            int32_t lHeightSpan,
            int32_t lBatchSpan,
            icp::DATATYPE lElementDataType,
            int32_t lElementDimX,
            int32_t lElementDimY,
            int32_t lRoiInfoL = 0,
            int32_t lRoiInfoR = 0,
            int32_t lRoiInfoT = 0,
            int32_t lRoiInfoB = 0);


   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Update the data pointers associated with the contiguous data region.
   ///
   /// \param[in] lpData
   /// Pointer to contiguous data region.
   /// \param[in] lpDataPhys
   /// Physical pointer to contiguous data region (for HW use).
   /////////////////////////////////////////////////////////////////////////
   void UpdatePtrs(void* const lpData,
                   void* const lpDataPhys);

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns a void data pointer to the contiguous data region.
   ///
   /// \return
   /// A void data pointer to the contiguous data region.
   /////////////////////////////////////////////////////////////////////////
   void* RetDataPtr() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns a void 'physical' data pointer to the contiguous data region.
   ///
   /// \return
   /// A void 'physical' data pointer to the contiguous data region.
   /////////////////////////////////////////////////////////////////////////
   void* RetDataPtrPhys() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the data type associated with the element.
   ///
   /// \return
   /// icp::DATATYPE associated with the element.
   /////////////////////////////////////////////////////////////////////////
   icp::DATATYPE RetElementDataType() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the X dimension (i.e. width) of the element.
   ///
   /// \return
   /// The X dimension (i.e. width) of the element.
   /////////////////////////////////////////////////////////////////////////
   int32_t RetElementDimX() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the Y dimension (i.e. height) of the element.
   ///
   /// \return
   /// The Y dimension (i.e. height) of the element.
   /////////////////////////////////////////////////////////////////////////
   int32_t RetElementDimY() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the width of the data region (in elements).
   ///
   /// \return
   /// The width of the data region (in elements).
   /////////////////////////////////////////////////////////////////////////
   int32_t RetChannel() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the width of the data region (in elements).
   ///
   /// \return
   /// The width of the data region (in elements).
   /////////////////////////////////////////////////////////////////////////
   int32_t RetWidth() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the height of the data region (in elements).
   ///
   /// \return
   /// The height of the data region (in elements).
   /////////////////////////////////////////////////////////////////////////
   int32_t RetHeight() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the batch of the data region (in elements).
   ///
   /// \return
   /// The batch of the data region (in elements).
   /////////////////////////////////////////////////////////////////////////
   int32_t RetBatch() const;


   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the span of the data region (in bytes).  Span is defined as the
   /// number of bytes between vertically adjacent values in a 2D data region.
   ///
   /// \return
   /// The span of the data region (in units of bytes).
   /////////////////////////////////////////////////////////////////////////
   int32_t RetSpan() const;


   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the channel span of the data region (in bytes).  Span is defined as the
   /// number of bytes between vertically adjacent values in a 2D data region.
   ///
   /// \return
   /// The channel span of the data region (in units of bytes).
   /////////////////////////////////////////////////////////////////////////
   int32_t RetChannelSpan() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the width span of the data region (in bytes).  Span is defined as the
   /// number of bytes between vertically adjacent values in a 2D data region.
   ///
   /// \return
   /// The width span of the data region (in units of bytes).
   /////////////////////////////////////////////////////////////////////////
   int32_t RetWidthSpan() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the height span of the data region (in bytes).  Span is defined as the
   /// number of bytes between vertically adjacent values in a 2D data region.
   ///
   /// \return
   /// The height span of the data region (in units of bytes).
   /////////////////////////////////////////////////////////////////////////
   int32_t RetHeightSpan() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the batch span of the data region (in bytes).  Span is defined as the
   /// number of bytes between vertically adjacent values in a 2D data region.
   ///
   /// \return
   /// The batch span of the data region (in units of bytes).
   /////////////////////////////////////////////////////////////////////////
   int32_t RetBatchSpan() const;


   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the stride of the data region in units of 'element data type'.
   /// Stride is defined as the number of 'units' between vertically adjacent
   /// values in a 2D data region.  In this case a unit = sizeof(element data type)
   /// (i.e. 8-bit = 1 byte, 16-bit = 2 bytes, 32-bit = 4 bytes).
   ///
   /// **NOTE: If the width of your data region is not equally divisible by
   ///         the aforementioned unit, the returned value of 'stride' is
   ///         undefined.
   ///
   /// \return
   /// The stride of the data region in units of 'element data type'.
   /////////////////////////////////////////////////////////////////////////
   int32_t RetStride_ElementDataType() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   /// Returns the stride of the data region in units of 'elements' considering
   /// only the X dimension of the element.
   /// Stride is defined as the number of 'units' between vertically adjacent
   /// values in a 2D data region.  In this case a unit =
   /// sizeof(element data type)*(element dimension X)
   ///
   /// **NOTE: If the width of your data region is not equally divisible by
   ///         the aforementioned unit, the returned value of 'stride' is
   ///         undefined.
   ///
   /// \return
   /// The stride of the data region in units of 'elements' considering
   /// only the X dimension of the element.
   /////////////////////////////////////////////////////////////////////////
   int32_t RetStride_ElementX() const;

   /////////////////////////////////////////////////////////////////////////
   /// \par Description
   ///
   /// Equality operator.  Returns "true" if the data contained in "this"
   /// is equal to the data contained in "lCdd", otherwise returns
   /// "false".
   ///
   /// \param lCdd [input]
   /// Reference to a constant ACF_DataDesc.
   ///
   /// \return
   /// "true" if data is the same, otherwise "false"
   /////////////////////////////////////////////////////////////////////////
   bool operator==(ACF_DataDesc const& lCdd);

   void FillRandom(int32_t lSeedVal = 0);
   void FillZero();

   int32_t CopyTo(ACF_DataDesc& lDst);

   void E0Replicate(int32_t lPadSizeInE0L,
                    int32_t lPadSizeInE0R,
                    int32_t lPadSizeInE0T,
                    int32_t lPadSizeInE0B);

   /// User-defined distance between the left edge of the described 2D data region
   /// and the left edge of the parent region (i.e. the left margin between
   /// parent edge and ROI edge)
   int32_t RetRoiInfoL() const;
   
   /// User-defined distance between the right edge of the described 2D data region
   /// and the right edge of the parent region (i.e. the right margin between
   /// parent edge and ROI edge)
   int32_t RetRoiInfoR() const;

   /// User-defined distance between the top edge of the described 2D data region
   /// and the top edge of the parent region (i.e. the top margin between
   /// parent edge and ROI edge)
   int32_t RetRoiInfoT() const;

   /// User-defined distance between the bottom edge of the described 2D data region
   /// and the bottom edge of the parent region (i.e. the bottom margin between
   /// parent edge and ROI edge)
   int32_t RetRoiInfoB() const;

   int32_t DumpToFile(const char* lFilenameRoot);
   void DumpToStdout();

private:

   void* mpData;     //for OS access
   void* mpDataPhys; //for HW access
   icp::DATATYPE mElementDataType;
   int32_t mElementDimX;
   int32_t mElementDimY;
   nd_desc mNdSize; //N-dimension size descriptor
   nd_desc mNdSpan; //N-dimension span descriptor
   int32_t mRoiInfo[4];  //is there useful data beyond an 'edge'? (i.e. is this an ROI?)
                         //[numElementsLeft, numElementsRight, numElementsTop,  numElementsBot]

   int32_t VerifyPhysPtrAlignment();
};

/// @}

#endif /* ACFDATADESC_HPP */
