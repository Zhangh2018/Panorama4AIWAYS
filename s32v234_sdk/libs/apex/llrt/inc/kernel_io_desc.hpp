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

#ifndef _KERNEL_IO_DESC_HPP
#define _KERNEL_IO_DESC_HPP

#include <stdint.h>

//forward declarations
class CircTileSmartBuf;
class DataDesc;

//////////////////////////////////////////////////////////////////////////
/// KernelIoDesc provides IO-centric functionality that facilitates interaction
/// with circular buffers.  It collects necessary information, and provides
/// methods to retrieve tile pointers, even if tile wrapping is present.
//////////////////////////////////////////////////////////////////////////
class KernelIoDesc
{
public:

   //---------
   //methods
   //---------

   /////////////////////////////////////////////////////////////////////////
   /// Create a KernelIoDesc for a producer.
   ///
   /// \param[in] lNumTilePtrs
   /// The (maximum) number of tile pointers that will be needed.
   ///
   /// \param[in] lpCircTileSmartBuf
   /// Pointer to the associated circular buffer object (i.e. the buffer
   /// that will be produced into).
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t CreateProd(int32_t lNumTilePtrs, CircTileSmartBuf* lpCircTileSmartBuf);

   /////////////////////////////////////////////////////////////////////////
   /// Create a KernelIoDesc for a consumer.
   ///
   /// \param[in] lNumTilePtrs
   /// The (maximum) number of tile pointers that will be needed.
   ///
   /// \param[in] lpCircTileSmartBuf
   /// Pointer to the associated circular buffer object (i.e. the buffer
   /// that will be consumed from).
   ///
   /// \param[in] lConsId
   /// Consumer ID.
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t CreateCons(int32_t lNumTilePtrs, CircTileSmartBuf* lpCircTileSmartBuf, int32_t lConsId);

   /////////////////////////////////////////////////////////////////////////
   /// Deallocate memory associated with KernelIoDesc and clean up.
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t Destroy();
   
   /////////////////////////////////////////////////////////////////////////
   /// Return the current/updated tile pointer array.  The returned value 
   /// must be cast to the appropriate type before use.  For example:
   ///
   /// \code{ .cpp }
   /// vec16s** MyPtrArray = (vec16s**)MyKernelIoDesc.RetTilePtrArray();
   /// \endcode
   ///
   /// \return
   /// Pointer to an array of tile pointers.  The number of tile pointers in
   /// the array is the number of tiles selected during KernelIoDesc
   /// creation  (i.e. "lNumTilePtrs" in CreateCons/CreateProd).
   /////////////////////////////////////////////////////////////////////////
   void** RetTilePtrArray();

   /////////////////////////////////////////////////////////////////////////
   /// Return the number of tile pointers that KernelIoDesc was initialized with
   /// (i.e. "lNumTilePtrs" in CreateCons/CreateProd).
   ///
   /// \return
   /// The number of tile pointers KernelIoDesc was initialized with.
   /////////////////////////////////////////////////////////////////////////
   int32_t RetNumTilePtrs();

   /////////////////////////////////////////////////////////////////////////
   /// For the dimension "lDim", return size for the tile/vchunk managed by
   /// the circular buffer (in units of element).
   ///
   /// \param[in] lDim
   /// Dimension of interest.
   ///
   /// \return
   /// Tile/vchunk size in dimension "lDim" (in units of element).
   /////////////////////////////////////////////////////////////////////////
   int32_t RetSize(int32_t lDim);

   /////////////////////////////////////////////////////////////////////////
   /// For the dimension "lDim", return stride for the tile/vchunk managed by
   /// the circular buffer (in units of data type).
   ///
   /// \param[in] lDim
   /// Dimension of interest.
   ///
   /// \return
   /// Tile/vchunk stride in dimension "lDim" (in units of data type).
   /////////////////////////////////////////////////////////////////////////
   int32_t RetStride(int32_t lDim);

   /////////////////////////////////////////////////////////////////////////
   /// For the dimension "lDim", return element size for the tile/vchunk managed by
   /// the circular buffer (in units of data type).
   ///
   /// \param[in] lDim
   /// Dimension of interest.
   ///
   /// \return
   /// Tile/vchunk element size in dimension "lDim" (in units of data type).
   /////////////////////////////////////////////////////////////////////////
   int32_t RetElementSize(int32_t lDim);

   /////////////////////////////////////////////////////////////////////////
   /// Return the number of dimensions for the tile/vchunk managed by the
   /// circular buffer.
   ///
   /// \return
   /// Number of dimensions associated with Tile/vchunk.
   /////////////////////////////////////////////////////////////////////////
   int32_t RetNumDim();
   
   /////////////////////////////////////////////////////////////////////////
   /// Update pointer array based on current state of circular buffer
   /// (8-bit scalar version for consumer).
   /////////////////////////////////////////////////////////////////////////
   void UpdateIoDescCons_SCL08();

   /////////////////////////////////////////////////////////////////////////
   /// Update pointer array based on current state of circular buffer
   /// (16-bit scalar version for consumer).
   /////////////////////////////////////////////////////////////////////////
   void UpdateIoDescCons_SCL16();

   /////////////////////////////////////////////////////////////////////////
   /// Update pointer array based on current state of circular buffer
   /// (32-bit scalar version for consumer).
   /////////////////////////////////////////////////////////////////////////
   void UpdateIoDescCons_SCL32();

   /////////////////////////////////////////////////////////////////////////
   /// Update pointer array based on current state of circular buffer
   /// and tile wrap index "lWrapIdx" (8-bit vector version for consumer).
   ///
   /// \param[in] lWrapIdx
   /// Tile wrap index (relevant for cases where tile width exceeds
   /// vector array width).
   /////////////////////////////////////////////////////////////////////////
   void UpdateIoDescCons_VEC08(int32_t lWrapIdx);

   /////////////////////////////////////////////////////////////////////////
   /// Update pointer array based on current state of circular buffer
   /// and tile wrap index "lWrapIdx" (16-bit vector version for consumer).
   ///
   /// \param[in] lWrapIdx
   /// Tile wrap index (relevant for cases where tile width exceeds
   /// vector array width).
   /////////////////////////////////////////////////////////////////////////
   void UpdateIoDescCons_VEC16(int32_t lWrapIdx);

   /////////////////////////////////////////////////////////////////////////
   /// Update pointer array based on current state of circular buffer
   /// and tile wrap index "lWrapIdx" (32-bit vector version for consumer).
   ///
   /// \param[in] lWrapIdx
   /// Tile wrap index (relevant for cases where tile width exceeds
   /// vector array width).
   /////////////////////////////////////////////////////////////////////////
   void UpdateIoDescCons_VEC32(int32_t lWrapIdx);
    
   /////////////////////////////////////////////////////////////////////////
   /// Update pointer array based on current state of circular buffer
   /// (8-bit scalar version for producer).
   /////////////////////////////////////////////////////////////////////////
   void UpdateIoDescProd_SCL08();

   /////////////////////////////////////////////////////////////////////////
   /// Update pointer array based on current state of circular buffer
   /// (16-bit scalar version for producer).
   /////////////////////////////////////////////////////////////////////////
   void UpdateIoDescProd_SCL16();

   /////////////////////////////////////////////////////////////////////////
   /// Update pointer array based on current state of circular buffer
   /// (32-bit scalar version for producer).
   /////////////////////////////////////////////////////////////////////////
   void UpdateIoDescProd_SCL32();

   /////////////////////////////////////////////////////////////////////////
   /// Update pointer array based on current state of circular buffer
   /// and tile wrap index "lWrapIdx" (8-bit vector version for producer).
   ///
   /// \param[in] lWrapIdx
   /// Tile wrap index (relevant for cases where tile width exceeds
   /// vector array width).
   /////////////////////////////////////////////////////////////////////////
   void UpdateIoDescProd_VEC08(int32_t lWrapIdx);

   /////////////////////////////////////////////////////////////////////////
   /// Update pointer array based on current state of circular buffer
   /// and tile wrap index "lWrapIdx" (16-bit vector version for producer).
   ///
   /// \param[in] lWrapIdx
   /// Tile wrap index (relevant for cases where tile width exceeds
   /// vector array width).
   /////////////////////////////////////////////////////////////////////////
   void UpdateIoDescProd_VEC16(int32_t lWrapIdx);

   /////////////////////////////////////////////////////////////////////////
   /// Update pointer array based on current state of circular buffer
   /// and tile wrap index "lWrapIdx" (32-bit vector version for producer).
   ///
   /// \param[in] lWrapIdx
   /// Tile wrap index (relevant for cases where tile width exceeds
   /// vector array width).
   /////////////////////////////////////////////////////////////////////////
   void UpdateIoDescProd_VEC32(int32_t lWrapIdx);

protected:

   //---------
   //members
   //---------

   void** mpTilePtrArray;
   int32_t mNumTilePtrs;
   CircTileSmartBuf* mpCircTileSmartBuf;
   DataDesc* mpDataDesc;
   int32_t mConsId;
};

#endif //_KERNEL_IO_DESC_HPP
