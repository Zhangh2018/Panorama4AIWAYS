/*******************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright 2017 NXP
 * All Rights Reserved
 *
 *******************************************************************************
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
 ******************************************************************************/

/*M/////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this
//  license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2014, Itseez Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright
//     notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote
//     products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is"
// and any express or implied warranties, including, but not limited to, the
// implied warranties of merchantability and fitness for a particular purpose
// are disclaimed. In no event shall the Intel Corporation or contributors be
// liable for any direct, indirect, incidental, special, exemplary, or
// consequential damages (including, but not limited to, procurement of
// substitute goods or services; loss of use, data, or profits; or business
// interruption) however caused and on any theory of liability, whether in
// contract, strict liability, or tort (including negligence or otherwise)
// arising in any way out of the use of this software, even if advised of the
// possibility of such damage.
//
//M*/

#include "sumat_mat.hpp"

/*******************************************************************************
 * Helper for Iterator ++ operator
 ******************************************************************************/
inline void initPointersIter(const vsdk::NAryMatIterator& aIter)
{
  if((aIter.iterdepth == 1) && (aIter.ptrs != static_cast< uint8_t** >(NULL)))
  {
    /* Iterate through pointers, and fill non null values */
    for(int32_t lIt = 0; lIt < aIter.narrays; lIt++)
    {
      if(aIter.ptrs[lIt] == static_cast< uint8_t* >(NULL))
      {
        continue;
      }
      aIter.ptrs[lIt] = aIter.arrays[lIt]->data + (aIter.arrays[lIt]->step[0] * aIter.idx);
    }
  }
}

/*******************************************************************************
 * Helper for Iterator ++ operator
 ******************************************************************************/
inline void initPlanesIter(const vsdk::NAryMatIterator& aIter)
{
  if((aIter.iterdepth == 1) && (aIter.planes != static_cast< vsdk::SMat* >(NULL)))
  {
    /* Iterate through planes and fill in non null values */
    for(int32_t lIt = 0; lIt < aIter.narrays; lIt++)
    {
      if(aIter.planes[lIt].data == static_cast< uint8_t* >(NULL))
      {
        continue;
      }
      aIter.planes[lIt].data = aIter.arrays[lIt]->data + (aIter.arrays[lIt]->step[0] * aIter.idx);
    }
  }
}

/*******************************************************************************
 * Helper for Iterator ++ operator - Iterate
 ******************************************************************************/
inline void iterateIterAux(const vsdk::NAryMatIterator& aIter, const vsdk::SMat& aA, uint8_t*& apData, int32_t& aIdx)
{
  for(int32_t lIt1 = (aIter.iterdepth - 1); (lIt1 >= 0) && (aIdx > 0); lIt1--)
  {
    const int32_t lSzi = static_cast< int32_t >(aA.size[lIt1]);
    const int32_t lT   = static_cast< int32_t >(aIdx / lSzi);
    apData += static_cast< uint64_t >(aIdx - (lT * lSzi)) * aA.step[lIt1];
    aIdx = lT;
  }
}

/*******************************************************************************
 * Helper for Iterator ++ operator - Iterate
 ******************************************************************************/
inline void setData(const vsdk::NAryMatIterator& aIter, uint8_t* const& apData, const int32_t& aIt)
{
  if(aIter.ptrs != static_cast< uint8_t** >(NULL))
  {
    aIter.ptrs[aIt] = apData;
  }
  if(aIter.planes != static_cast< vsdk::SMat* >(NULL))
  {
    aIter.planes[aIt].data = apData;
  }
}

/*******************************************************************************
 * Helper for Iterator ++ operator - Iterate
 ******************************************************************************/
inline void iterateIter(const vsdk::NAryMatIterator& aIter)
{
  /* Call only in case of depth NOT equal to 1 */
  for(int32_t lIt = 0; lIt < aIter.narrays; lIt++)
  {
    /* create a SMat for each array */
    const vsdk::SMat& lA = *aIter.arrays[lIt];
    if(lA.data == static_cast< uint8_t* >(NULL))
    {
      continue;
    }

    int32_t lIdx = static_cast< int32_t >(aIter.idx);

    uint8_t* lpData = lA.data;

    iterateIterAux(aIter, lA, lpData, lIdx);
    setData(aIter, lpData, lIt);
  }
}

/*******************************************************************************
 * The default constructor
 * Initializes all member function to its default values, i.e. NULL or zeroes
 ******************************************************************************/
vsdk::NAryMatIterator::NAryMatIterator()
    : arrays(static_cast< const vsdk::SMat** >(NULL)), planes(static_cast< vsdk::SMat* >(NULL)),
      ptrs(static_cast< uint8_t** >(NULL)), narrays(0), nplanes(0U), size(0U), iterdepth(0), idx(0U)
{
  /* Initializes all member function to its default values, i.e. NULL or zeroes */
  /* No specific data are passed through the constructor. */
}

/*******************************************************************************
 * The full constructor taking arbitrary number of n-dim matrices
 * Internally, it uses the init function declared bellow in this code with
 * specific parameters.
 ******************************************************************************/
vsdk::NAryMatIterator::NAryMatIterator(const vsdk::SMat** const apArrays,
                                       vsdk::SMat* const        apPlanes,
                                       const int32_t            aNArrays)
    : arrays(static_cast< const vsdk::SMat** >(NULL)), planes(static_cast< vsdk::SMat* >(NULL)),
      ptrs(static_cast< uint8_t** >(NULL)), narrays(0), nplanes(0U), size(0U), iterdepth(0), idx(0U)
{
  /* Initializes all member function to its default values, i.e. NULL or zeroes */
  /* Plus there's an init function, which is called subsequently and sets correct data to the iterator */
  init(apArrays, apPlanes, static_cast< uint8_t** >(NULL), aNArrays);
}

/*******************************************************************************
 * The full constructor taking arbitrary number of n-dim matrices
 * Internally, it uses the init function declared bellow in this code with
 * specific parameters.
 ******************************************************************************/
vsdk::NAryMatIterator::NAryMatIterator(const vsdk::SMat** const apArrays,
                                       uint8_t** const          apBuffers,
                                       const int32_t            aNArrays)
    : arrays(static_cast< const vsdk::SMat** >(NULL)), planes(static_cast< vsdk::SMat* >(NULL)),
      ptrs(static_cast< uint8_t** >(NULL)), narrays(0), nplanes(0U), size(0U), iterdepth(0), idx(0U)
{
  /* Initializes all member function to its default values, i.e. NULL or zeroes */
  /* Plus there's an init function, which is called subsequently and sets correct data to the iterator */
  init(apArrays, static_cast< vsdk::SMat* >(NULL), apBuffers, aNArrays);
}

/*******************************************************************************
 * Helper for initialization
 ******************************************************************************/
inline bool initAllocateMatIter(const vsdk::NAryMatIterator& aIter, const vsdk::SMat& aA, const int32_t& aIt)
{
  bool lRet;
  /* If data are null, make an empty matrix */
  if(aA.data == static_cast< uint8_t* >(NULL))
  {
    aIter.planes[aIt] = vsdk::SMat();
    lRet              = true;
  }
  else
  {
    /* Use buffer otherwise */
    aIter.planes[aIt] = vsdk::SMat(1, static_cast< int32_t >(aIter.size), aA.type(), aA.data);
    lRet              = false;
  }
  return lRet;
}

/*******************************************************************************
 * Helper for initialization
 ******************************************************************************/
inline void initSetAllDataIter(const vsdk::NAryMatIterator& aIter)
{
  /* If planes is not null, we can set all data for all arrays */
  if(aIter.planes != static_cast< vsdk::SMat* >(NULL))
  {
    /* For all arrays in the list */
    for(int32_t lIt = 0; lIt < aIter.narrays; lIt++)
    {
      /* Create an aux Matrix, and save it to the planes */
      const vsdk::SMat& lA = *aIter.arrays[lIt];

      if(initAllocateMatIter(aIter, lA, lIt))
      {
        continue;
      }
    }
  }
}

/*******************************************************************************
 * Helper for initialization
 ******************************************************************************/
inline void initComputePlanesIter(vsdk::NAryMatIterator& aIter, int32_t& aIt, const int32_t& aIt0, const int32_t& aD1)
{
  aIter.iterdepth = aIt;
  if(aIter.iterdepth == aD1)
  {
    aIter.iterdepth = 0;
  }

  aIter.nplanes = 1U;
  /* Compute number of planes */
  for(aIt = (aIter.iterdepth - 1); aIt >= 0; aIt--)
  {
    aIter.nplanes *= static_cast< size_t >(aIter.arrays[aIt0]->size[aIt]);
  }
}
/*******************************************************************************
 * Helper for initialization
 ******************************************************************************/
inline void initComputeNPlanesIter(vsdk::NAryMatIterator& aIter,
                                   const int32_t&         aIt0,
                                   const int32_t&         aDims,
                                   const int32_t&         aD1)
{
  /* If we have computed the number of iterations  */
  if(aIt0 >= 0)
  {
    aIter.size = static_cast< uint32_t >(aIter.arrays[aIt0]->size[aDims - 1]);

    int32_t lIt;

    /* Iterate through dims again */
    for(lIt = (aDims - 1); lIt > aIter.iterdepth; lIt--)
    {
      /* Set the total size */
      const int64_t lTotal1 = static_cast< int64_t >(aIter.size) * aIter.arrays[aIt0]->size[(lIt - 1)];
      if(lTotal1 != static_cast< int32_t >(lTotal1))
      {
        break;
      }
      aIter.size = static_cast< uint32_t >(lTotal1);
    }

    initComputePlanesIter(aIter, lIt, aIt0, aD1);
  }
  /* aIt0 is negative */
  else
  {
    aIter.iterdepth = 0;
  }
}

/*******************************************************************************
 * Helper for initialization
 ******************************************************************************/
inline void stepInlineValuesIter(vsdk::NAryMatIterator& aIter,
                                 const vsdk::SMat&      aA,
                                 const int32_t&         aDims,
                                 const int32_t&         aD1)
{
  /* In case the matrix is not continuous, work with step values */
  if(!aA.isContinuous())
  {
    int32_t lIt;
    for(lIt = (aDims - 1); lIt > aD1; lIt--)
    {
      /* break when outside the boundaries */
      if((aA.step[lIt] * static_cast< uint64_t >(aA.size[lIt])) < aA.step[lIt - 1])
      {
        break;
      }
    }
    /* Choose maximum value */
    aIter.iterdepth = std::max(aIter.iterdepth, lIt);
  }
}

/*******************************************************************************
 * 
 ******************************************************************************/
inline bool setDataIter(const vsdk::NAryMatIterator& aIter, const vsdk::SMat& aA, const int32_t& aIt)
{
  bool lRet = false;
  /* If buffer is not null, we can set specific data to it */
  if(aIter.ptrs != static_cast< uint8_t** >(NULL))
  {
    aIter.ptrs[aIt] = aA.data;
  }

  /* When NULL is encountered, we continue with next array*/
  if(aA.data == static_cast< uint8_t* >(NULL))
  {
    lRet = true;
  }

  return lRet;
}

/*******************************************************************************
 * Goes through dimensions and stops at the first with non 1 size
 ******************************************************************************/
inline void iterateThroughDims(const vsdk::SMat& aMat, int32_t& aD1, const int32_t& aDims)
{
  /* Go through the dimensions and stop on size > 1 */
  for(aD1 = 0; aD1 < aDims; aD1++)
  {
    if(aMat.size[aD1] > 1)
    {
      break;
    }
  }
}
/*******************************************************************************
 * Helper for initialization
 ******************************************************************************/
inline void initializeIter(vsdk::NAryMatIterator& aIter, int32_t& aIt0, int32_t& aDims, int32_t& aD1)
{
  const vsdk::SMat** const lIterArr = aIter.arrays;
  for(int32_t lIt = 0; lIt < aIter.narrays; ++lIt)
  {
    const vsdk::SMat& lA = *lIterArr[lIt];

    /* If buffer is not null, we can set specific data to it */
    if(setDataIter(aIter, lA, lIt) == true)
    {
      continue;
    }

    if(aIt0 < 0)
    {
      aIt0  = lIt;
      aDims = lA.dims;

      iterateThroughDims(lA, aD1, aDims);
    }
    else
    {
    }

    /* In case the matrix is not continuous, work with step values */
    stepInlineValuesIter(aIter, lA, aDims, aD1);
  }
}
/*******************************************************************************
 * The separate iterator initialization method
 * @param apArrays  Arrays of matrices as an input for iterator
 * @param apPlanes  Planes of matrices
 * @param apBuffers Array of buffers
 * @param aNArrays  Number of arrays
 ******************************************************************************/
void vsdk::NAryMatIterator::init(const vsdk::SMat** const apArrays,
                                 vsdk::SMat* const        apPlanes,
                                 uint8_t** const          apBuffers,
                                 const int32_t            aNArrays)
{
  /* Init of all members */
  int32_t lD1   = 0;
  int32_t lIt0  = -1;
  int32_t lDims = -1;

  /* Set the pointers to matrices, the planes and buffers */
  arrays  = apArrays;
  ptrs    = apBuffers;
  planes  = apPlanes;
  narrays = aNArrays;
  nplanes = 0U;
  size    = 0U;

  /* If number of arrays id lower than 0, we need to compute it */
  if(narrays < 0)
  {
    int32_t lIt;
    /* Break when first NULL is met */
    for(lIt = 0; apArrays[lIt] != static_cast< vsdk::SMat* >(NULL);)
    {
      lIt++;
    }

    /* Count number of arrays */
    narrays = lIt;
  }

  iterdepth = 0;

  /* Iterate throught arrays and set data to pointers */
  initializeIter(*this, lIt0, lDims, lD1);

  /* If we have computed the number of iterations  */
  initComputeNPlanesIter(*this, lIt0, lDims, lD1);

  idx = 0U;

  initSetAllDataIter(*this);
}

/*******************************************************************************
 * Proceeds to the next plane of every iterated matrix
 ******************************************************************************/
vsdk::NAryMatIterator& vsdk::NAryMatIterator::operator++()
{
  vsdk::NAryMatIterator* lRetVal;

  /* If the index is greates than nplanes, just return */
  if(idx >= (nplanes - 1U))
  {
    lRetVal = this;
  }
  /* Otherwise ... */
  else
  {
    ++idx;

    /* Case of depth equal to 1*/
    initPointersIter(*this);
    initPlanesIter(*this);

    /* Case of depth NOT equal to 1 */
    if(iterdepth != 1)
    {
      iterateIter(*this);
    }

    lRetVal = this;
  }

  return *lRetVal;
}

/*******************************************************************************
 * Proceeds to the next plane of every iterated matrix (postfix increment
 * operator)
 ******************************************************************************/
vsdk::NAryMatIterator vsdk::NAryMatIterator::operator++(int32_t)
{
  const NAryMatIterator lIt = *this;
  ++*this;
  return lIt;
}
