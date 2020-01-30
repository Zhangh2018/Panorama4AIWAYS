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

#ifndef _CIRC_TILE_SMART_BUF_HPP
#define _CIRC_TILE_SMART_BUF_HPP

#include <stdint.h>
#include <circ_tile_buf.hpp>

//forward declarations
class DataDesc;
class VectorMapDesc;

typedef enum _EDGE_PAD_TYPE
{
   EDGE_PAD_TYPE__REPLICATE = 0,
   EDGE_PAD_TYPE__ZERO,
   EDGE_PAD_TYPE__UNDEFINED
} EDGE_PAD_TYPE;

#define MAX_NUM_CONS 5 //maximum number of allowable consumers on a single producer

//////////////////////////////////////////////////////////////////////////
/// CircTileSmartBuf is a circular tile buffer that provides producer and consumer
/// services to help facilitate sequencing.  It acts as a data flow and flow control
/// link between logical tasks (e.g. data transfer tasks, processing tasks, etc.).
/// 
/// The primary (iterative) usage pattern is as follows:
/// 1) query circular buffers to know if required tiles are available
/// 2) perform processing if requested tiles are available
/// 3) update circular buffer states based on what was consumed/produced
///
/// See the following code example:
///
/// \code{ .cpp }
/// //Check circular buffers to see if tiles are available for consumers (i.e. inputs) and empty slots are available for producers (i.e. outputs).
/// //In this example, the task is a kernel that takes two inputs and produces one output (e.g. an add kernel).
/// if (mpCircBufInput0->QueryTilesAvailForCons(1, mIn0ConsIdx) &&
///     mpCircBufInput1->QueryTilesAvailForCons(1, mIn1ConsIdx) &&
///     mpCircBufOutput0->QueryTilesAvailForProd(1))
/// {
///    //<Use GetTilePtrArrayForProd and GetTilePtrArrayForCons (or use KernelIoDesc helper class)
///    //to retrieve tile pointers and proceed to process the tile(s)>
/// 
///    //Once processing is complete, update the states of each circular buffer based on what was consumed and produced.
///    //In this example, mpCircBufInput0 is being used to determine the 'done' status so it can be passed along
///    //(i.e. done must be cascaded so if another task is consuming from mpCircBufOutput0, it too will know when it is done, etc.).
///    mpCircBufInput0->TilesCons(1, mIn0ConsIdx);
///    mpCircBufInput1->TilesCons(1, mIn1ConsIdx);
///    mpCircBufOutput0->TilesProd(1, mpCircBufInput0->QueryDone(mIn0ConsIdx));
/// }
/// \endcode
///
/// As a general guideline, the circular tile buffer should be set up to manage tiles
/// of the smallest meaningful granularity.  For example, in the typical 2D processing
/// case, tiles should be lines of elements.  This is an important detail; it means that
/// when you retrieve tile a pointer array, you are retrieving a line pointer array, and
/// you don't have to worry about navigating the tile height dimension since it is 1.
//////////////////////////////////////////////////////////////////////////
class CircTileSmartBuf
{
public:
   
   //---------
   //methods
   //---------

   /////////////////////////////////////////////////////////////////////////
   /// Create a circular tile buffer in DMEM (scalar).  The buffer will allocate space for
   /// "lNumTiles" tiles, and each tile is described by "lpTileDesc".
   ///
   /// \param[in] lNumTiles
   /// Number of tiles the circular buffer will manage.
   ///
   /// \param[in] lpTileDesc
   /// Data descriptor that describes a tile.
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t CreateDmem(int32_t lNumTiles, DataDesc* lpTileDesc);

   /////////////////////////////////////////////////////////////////////////
   /// Create a circular tile buffer in SMEM (scalar).  The buffer will allocate space for
   /// "lNumTiles" tiles, and each tile is described by "lpTileDesc".
   ///
   /// \param[in] lNumTiles
   /// Number of tiles the circular buffer will manage.
   ///
   /// \param[in] lpTileDesc
   /// Data descriptor that describes a tile.
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t CreateSmem(int32_t lNumTiles, DataDesc* lpTileDesc);
   
   //this one is for typical vectorized buffer in CMEM (vectorization could be on CU level or GOC level)

   /////////////////////////////////////////////////////////////////////////
   /// Create a circular tile buffer in CMEM (vector).  The buffer will allocate space for
   /// "lNumTiles" tiles, and each vchunk (i.e. vector element) comprising a tile is described
   /// by "lpVchunkDesc".
   ///
   /// \param[in] lNumTiles
   /// Number of tiles the circular buffer will manage.
   ///
   /// \param[in] lpVchunkDesc
   /// Data descriptor that describes the vchunk (i.e. vector element) associated with the vectorized
   /// tile.
   ///
   /// \param[in] lNumVchunksInTile
   /// Based on how the tile is vectorized, this is the number of vchunks (i.e. vector elements) comprising
   /// a vectorized tile.  For example, if a tile is a 256x1 line, and it is vectorized such that
   /// a vchunk is 4x1, lNumVchunksInTile = (256/4) = 64.
   ///
   /// \param[in] lpVecMapDesc
   /// Pointer to a vector map descriptor.
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t CreateCmem(int32_t lNumTiles, DataDesc* lpVchunkDesc, int32_t lNumVchunksInTile, VectorMapDesc* lpVecMapDesc);
   
   /////////////////////////////////////////////////////////////////////////
   /// Deallocate memory associated with CircTileSmartBuf and clean up.
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t Destroy();

   /////////////////////////////////////////////////////////////////////////
   /// Request a consumer ID from the circular buffer.  Any task/user wishing
   /// to consume tiles from a circular buffer must first retrieve a unique ID.
   /// It is possible for multiple consumers to be active on a single circular buffer,
   /// so each consumer must use a unique ID when interacting with consumer-centric
   /// functions (many of the functions below have an lConsId parameter).
   ///
   /// \param[out] lpConsId
   /// Pointer to the int32_t where the requested consumer ID will be returned.
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t ReqConsId(int32_t* lpConsId);

   /////////////////////////////////////////////////////////////////////////
   /// Indicate that "lNumTilesProd" tiles were produced into the circular
   /// buffer (producing tiles allows consumers waiting on tiles to move forward).
   ///
   /// \param[in] lNumTilesProd
   /// Number of tiles produced.
   ///
   /// \param[in] lLastFlag
   /// This flag should be set to 1 when the producer knows it is finished,
   /// otherwise it should be set to 0.
   /////////////////////////////////////////////////////////////////////////
   void TilesProd(int32_t lNumTilesProd, int32_t lLastFlag);

   /////////////////////////////////////////////////////////////////////////
   /// Indicate that "lNumTilesCons" tiles were consumed by the consumer "lConsId"
   /// (consuming tiles allows producers waiting on tile slots to move forward).
   ///
   /// \param[in] lNumTilesCons
   /// Number of tiles consumed.
   ///
   /// \param[in] lConsId
   /// Consumer ID.
   /////////////////////////////////////////////////////////////////////////
   void TilesCons(int32_t lNumTilesCons, int32_t lConsId);

   /////////////////////////////////////////////////////////////////////////
   /// Query to see if "lNumTiles" tile slots are available for the producer to
   /// write into.
   ///
   /// \param[in] lNumTiles
   /// Number of tiles.
   ///
   /// \return
   /// 1 if "lNumTiles" tile slots are available, 0 otherwise
   /////////////////////////////////////////////////////////////////////////
   int32_t QueryTilesAvailForProd(int32_t lNumTiles);

   /////////////////////////////////////////////////////////////////////////
   /// Query to see if "lNumTiles" tiles are available for consumer "lConsId"
   /// to read from.
   ///
   /// \param[in] lNumTiles
   /// Number of tiles.
   ///
   /// \param[in] lConsId
   /// Consumer ID.
   ///
   /// \return
   /// 1 if "lNumTiles" tiles are available, 0 otherwise
   /////////////////////////////////////////////////////////////////////////
   int32_t QueryTilesAvailForCons(int32_t lNumTiles, int32_t lConsId);
   
   /////////////////////////////////////////////////////////////////////////
   /// Return a pointer to the next tile slot available to the producer. It is
   /// assumed that the user has previously called QueryTilesAvailForProd and
   /// knows that a tile slot is available.
   ///
   /// \return
   /// Void pointer to the next tile slot available to the producer.
   /////////////////////////////////////////////////////////////////////////
   void* RetTilePtrForProd();

   /////////////////////////////////////////////////////////////////////////
   /// Return a pointer to the next tile available to consumer "lConsId".  It is
   /// assumed that the user has previously called QueryTilesAvailForCons and
   /// knows that a tile is available.
   ///
   /// \param[in] lConsId
   /// Consumer ID.
   ///
   /// \return
   /// Void pointer to the next tile available to consumer "lConsId".
   /////////////////////////////////////////////////////////////////////////
   void* RetTilePtrForCons(int32_t lConsId);

   /////////////////////////////////////////////////////////////////////////
   /// Retrieve an array of pointers to the next "lNumTiles" tile slots
   /// available to the producer.  It is assumed that the user has previously
   /// called QueryTilesAvailForProd and knows that "lNumTiles" tile slots are
   /// available.
   ///
   /// \param[in] lNumTiles
   /// Number of tiles.
   ///
   /// \param[out] lTilePtrArray
   /// Pointer to a user-allocated void pointer array of size >= "lNumTiles".
   /////////////////////////////////////////////////////////////////////////
   void GetTilePtrArrayForProd(int32_t lNumTiles, void* lTilePtrArray[]);

   /////////////////////////////////////////////////////////////////////////
   /// Retrieve an array of pointers to the next "lNumTiles" tiles
   /// available to consumer "lConsId".  It is assumed that the user has previously
   /// called QueryTilesAvailForCons and knows that "lNumTiles" tiles are
   /// available.
   ///
   /// \param[in] lNumTiles
   /// Number of tiles.
   ///
   /// \param[out] lTilePtrArray
   /// Pointer to a user-allocated void pointer array of size >= "lNumTiles".
   ///
   /// \param[in] lConsId
   /// Consumer ID.
   /////////////////////////////////////////////////////////////////////////
   void GetTilePtrArrayForCons(int32_t lNumTiles, void* lTilePtrArray[], int32_t lConsId);
 
   /////////////////////////////////////////////////////////////////////////
   /// Query to see if consumption is complete from the perspective of consumer
   /// "lConsId".  For this to be true, the producer must be done AND there must
   /// be no more core (i.e. non-padding) tiles left to consume (can be used for 'done'
   /// cascading).
   ///
   /// \param[in] lConsId
   /// Consumer ID.
   ///
   /// \return
   /// 1 if done, 0 otherwise
   /////////////////////////////////////////////////////////////////////////
   int32_t QueryDone(int32_t lConsId);

   /////////////////////////////////////////////////////////////////////////
   /// Query to see if consumption will be complete from the perspective of consumer
   /// "lConsId" if "lNumTiles" are consumed.  This is similar to QueryDone,
   /// except it asks the question 'will I be done if I consume "lNumTiles"
   /// more tiles?'.
   ///
   /// \param[in] lNumTiles
   /// Number of tiles that will be (but have not yet been) consumed.
   ///
   /// \param[in] lConsId
   /// Consumer ID.
   ///
   /// \return
   /// 1 if done, 0 otherwise
   /////////////////////////////////////////////////////////////////////////
   int32_t QueryLastCons(int32_t lNumTiles, int32_t lConsId);

   /////////////////////////////////////////////////////////////////////////
   /// Query to see if consumer "lConsId" has consumed anything yet.
   ///
   /// \param[in] lConsId
   /// Consumer ID.
   ///
   /// \return
   /// 1 if consumer "lConsId" has not consumed anything, 0 if anything has
   /// been consumed.
   /////////////////////////////////////////////////////////////////////////
   int32_t QueryFirstCons(int32_t lConsId);
   
   /////////////////////////////////////////////////////////////////////////
   /// Enable automatic 2D padding on the circular buffer for the consumer "lConsId".
   /// Padding currently only works for 2D data where the tile spans the
   /// width of the 2D region, and for usability reasons, padding should only
   /// be enabled for cases where the tile is a line (i.e. tile height is 1).
   /// When padding is enabled on vector data, padding between CUs will be
   /// taken care of, and edge padding of type "lEdgePadType" will be performed on
   /// applicable tile edges.
   /// 
   /// With support for multiple active consumers on a buffer, each consumer may have its
   /// own padding size requirements (e.g. consumer 0 may want enough padding for a 3x3 filter
   /// and consumer 1 may want enough padding for a 5x5 filter).  This is supported, with the
   /// restriction that both consumers specify the same edge padding type.
   ///
   /// Scalar padding support is not implemented yet.
   ///
   /// \param[in] lTop
   /// Top padding size (in units of 'element').
   ///
   /// \param[in] lBottom
   /// Bottom padding size (in units of 'element').
   ///
   /// \param[in] lLeft
   /// Left padding size (in units of 'element').
   ///
   /// \param[in] lRight
   /// Right padding size (in units of 'element').
   ///
   /// \param[in] lEdgePadType
   /// Edge padding type (edge replication and zero edge padding are supported).  If there are
   /// multiple consumers, edge padding type  must be the same for all of them.
   ///
   /// \param[in] lConsId
   /// ID of the consumer for whom the padding is relevant.
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t EnableTileEdgePadding2D(int32_t lTop, int32_t lBottom, int32_t lLeft, int32_t lRight, EDGE_PAD_TYPE lEdgePadType, int32_t lConsId);

   /////////////////////////////////////////////////////////////////////////
   /// Return the tile wrap count.  This will be non-zero if the number of vchunks
   /// associated with a vectorized tile exceeds the physical vector array width.
   ///
   /// \return
   /// Tile wrap count.
   /////////////////////////////////////////////////////////////////////////
   int32_t RetTileWrapCnt();

   /////////////////////////////////////////////////////////////////////////
   /// Set tile info for the tile being produced.  This function is meant to be
   /// called by a producer that wants to propagate tile information to consumers,
   /// and it must be called prior to calling the TilesProd function.
   ///
   /// \param[in] lpTileInfo
   /// Pointer to an initialized TileInfo object.
   ///
   /// \param[in] lIdx
   /// Tile index (e.g. if a producer is producing 4 tiles, this can be called
   /// with "lIdx" values ranging from 0 to 3).
   /////////////////////////////////////////////////////////////////////////
   void SetTileInfo(TileInfo* lpTileInfo, int32_t lIdx);

   /////////////////////////////////////////////////////////////////////////
   /// For consumer "lConsId", get tile info for tile "lIdx". This function
   /// must be called prior to calling the TilesCons function.
   ///
   /// \param[in] lIdx
   /// Tile index (e.g. if a consumer asked for 4 tiles, this can be called
   /// with "lIdx" values ranging from 0 to 3).
   ///
   /// \param[in] lConsId
   /// Consumer ID.
   ///
   /// \return
   /// Pointer to a TileInfo object.
   /////////////////////////////////////////////////////////////////////////
   TileInfo* GetTileInfo(int32_t lIdx, int32_t lConsId);

   /////////////////////////////////////////////////////////////////////////
   /// Reset internal state information assuming no change from previous
   /// configuration.
   /////////////////////////////////////////////////////////////////////////
   void Reset();
   
   //---------
   //members
   //---------

   CircTileBuf mCircTileBuf;
   

private:

   //members related to producer/consumer functionality
   int32_t mProdCounter;                        //keep track of number of lines produced
   int32_t mConsCounter[MAX_NUM_CONS];          //keep track of number of lines consumed
   int32_t mNumTilesAvailForProd;               //number of tiles available to the producer
   int32_t mNumTilesAvailForCons[MAX_NUM_CONS]; //number of tiles available to the consumer
   int32_t mProdDoneFlag;                       //initialized to 0 and set to 1 if there will be no further production
   int32_t mFirstFlag[MAX_NUM_CONS];            //initialized to 1 and set to 0 the first time anything is consumed
   int32_t mNumCons;                            //number of active consumers on this buffer
   int32_t mConsOrder[MAX_NUM_CONS];            //array containing consumer ids sorted by their speed (slowest consumer is first, fastest is last)
   int32_t mConsGateFlag[MAX_NUM_CONS];         //for cases where there is top padding, must gate consumption until the padding has actually been filled in!!!

   //members related to automatic tile edge padding functionality
   //top, bottom, left, right of tile in unit of mCircTileBuf.mpTileDesc->ELEMENT
   int32_t mTileEdgePadT;
   int32_t mTileEdgePadB;
   int32_t mTileEdgePadL;
   int32_t mTileEdgePadR;
   int32_t mPadDimTB; //dimension on which to perform T and B padding (all others dimensions assumed max)
   int32_t mPadDimLR; //dimension on which to perform L and R padding (all others dimensions assumed max)
   int32_t mPadEnabledFlag;
   EDGE_PAD_TYPE mTileEdgePadType;
   int32_t mPadTopCompleteFlag;
   int32_t mPadBotCompleteFlag;

   int32_t mNumTilesContEdgePadT; //worst case if multiple consumers
   int32_t mNumTilesContEdgePadB; //worst case if multiple consumers
   int32_t mNumTilesContEdgePadTConsSpec[MAX_NUM_CONS];
   int32_t mNumTilesContEdgePadBConsSpec[MAX_NUM_CONS];

   void Init();
   void DerivedInit();

   //interfaces not exposed yet
   int32_t QueryNumTilesAvailForProd();
   int32_t QueryNumTilesAvailForProdContiguous(); //could be useful for DMA purposes
   
   //functions to help with top/bottom tile edge padding
   void TopPaddingComplete();
   void GetTilePtrArrayBehindProd(int32_t lNumTilesBehindProd, void* lTilePtrArray[]);
   void GetTilePtrArrayBehindCons(int32_t lNumTilesBehindCons, void* lTilePtrArray[], int32_t lConsId);
   void GetTilePtrArrayAtCoreStart(int32_t lNumTiles, void* lTilePtrArray[]);
   void GetTilePtrArrayBehindCoreStart(int32_t lNumTiles, void* lTilePtrArray[]);
   int32_t QueryTilesAvailForTopPaddingSrc(int32_t lNumTiles);
   void PerformTilePaddingTop();
   void PerformTilePaddingLR(int32_t lNumTiles);
   void PerformTilePaddingBot();

   //padding functions
   void PadTop_Replicate();
   void PadTop_Zero();
   void PadLR_Replicate(int32_t lNumTiles);
   void PadLR_Zero(int32_t lNumTiles);
   void PadLR_InterCu(int32_t lNumTiles); //perform padding between neighboring CUs
   void PadBot_Replicate();
   void PadBot_Zero();
};

#endif //_CIRC_TILE_SMART_BUF_HPP
