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

#ifndef _VECTOR_MAP_DESC_HPP
#define _VECTOR_MAP_DESC_HPP

//VEC_MODE is meant to express the conceptual vector unit (e.g 1CU, 4CUs/GOC4, 8CUs/GOC8). this is best interpreted
//as the vector unit from a processing perspective.  GOC16 (for e.g.) is not here because there is no mode where a
//group of 16 CUs have access to 16*8K of CMEM.
typedef enum _VEC_MODE
{
   VEC_MODE_NONE = 0,
   VEC_MODE_CU,
   VEC_MODE_GOC4,
   VEC_MODE_GOC8
} VEC_MODE;

int32_t RetNumCusInGoc(VEC_MODE lVecMode);
int32_t IsGocMode(VEC_MODE lVecMode);

//////////////////////////////////////////////////////////////////////////
/// VectorMapDesc encapsulates information that influences how a vectorized tile
/// is mapped to physical vector resources.  In the APEX case, this includes information
/// about the vector array size (e.g. 32 or 64 CUs), and whether or not vector
/// elements should be left empty at the beginning/end of a logical tile.
//////////////////////////////////////////////////////////////////////////
class VectorMapDesc
{
public:
   
   //---------
   //methods
   //---------

   //////////////////////////////////////////////////////////////////////////
   /// Initialize vector map descriptor.
   ///
   /// \param[in] lVecMode
   /// Vector mode (currently only VEC_MODE_CU is supported).
   ///
   /// \param[in] lPhysVecSize
   /// Physical size of vector array (e.g. 32 for 32CU mode, 64 for 64CU mode).
   ///
   /// \param[in] lVecPad0
   /// How many vector elements to leave free at the start of a tile (e.g. if
   /// lVecMode==VEC_MODE_CU, and lVecPad0==1, vectorized data will start filling the
   /// vector array at CU 1 instead of CU0).
   ///
   /// \param[in] lVecPad1
   /// How many vector elements to leave free at the end of a tile (e.g. if
   /// lVecMode==VEC_MODE_CU, and lVecPad1==1, it is assumed that 1 additional CU past the
   /// end of the tile is reserved).
   //////////////////////////////////////////////////////////////////////////
   void Init(VEC_MODE lVecMode, int32_t lPhysVecSize, int32_t lVecPad0, int32_t lVecPad1);
   
   //---------
   //members
   //---------

   VEC_MODE mVecMode;
   int32_t  mVecPad[2];   //number of CU/GOC to leave 'empty' at beginning and end of the vector (typically for padding use case)
   int32_t  mPhysVecSize; //max vector size allowed by the physical array limitations (e.g. if physical array width is 64 CUs, this would be 64 if gocMode=0 and 4 if gocMode=16)
};

#endif //_VECTOR_MAP_DESC_HPP
