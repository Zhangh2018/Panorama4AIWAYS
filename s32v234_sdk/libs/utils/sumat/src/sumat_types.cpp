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

#include "sumat_types.hpp"
/**
 * Default constructor
 */
vsdk::Range::Range() : start(0), end(0)
{
}

/**
 * Constructor with specified values
 */
vsdk::Range::Range(const int32_t aStart, const int32_t aEnd) : start(aStart), end(aEnd)
{
}

/**
 * Function returns size of the range
 */
int32_t vsdk::Range::size() const
{
  return end - start;
}

/**
 * Function returns true is range is empty
 */
bool vsdk::Range::empty() const
{
  return (start == end);
}

/**
 * Function returns min to max range
 */
vsdk::Range vsdk::Range::all()
{
  return vsdk::Range(INT32_MIN, INT32_MAX);
}

/**
 * Equal operator
 */
bool operator==(const vsdk::Range& aRange1, const vsdk::Range& aRange2)
{
  return ((aRange1.start == aRange2.start) && (aRange1.end == aRange2.end));
}

/**
 * Non equal operator
 */
bool operator!=(const vsdk::Range& aRange1, const vsdk::Range& aRange2)
{
  return !(aRange1 == aRange2);
}

/**
 * Default constructor
 */
vsdk::MatSize::MatSize(int32_t* const apSizes) : p(apSizes)
{
}

/**
 * Get array element operator - returns the size on specified index (const)
 */
const int32_t& vsdk::MatSize::operator[](const int32_t aIndex) const
{
  return p[aIndex];
}

/**
 * Get array element operator - returns the size on specified index
 */
int32_t& vsdk::MatSize::operator[](const int32_t aIndex)
{
  return p[aIndex];
}

/**
 * Pointer retype operator
 */
vsdk::MatSize::operator const int32_t*() const
{
  return p;
}

/**
 * Comparison operator
 * Returns True if the MatSize objects are equal
 */
bool vsdk::MatSize::operator==(const vsdk::MatSize& aSize) const
{
  const int32_t lD      = p[-1];
  const int32_t lDSz    = aSize.p[-1];
  bool          lRetVal = true;
  /* Number of dimensions does not match */
  if(lD != lDSz)
  {
    lRetVal = false;
  }
  else
  {
    /* case 2 dimensions */
    if(lD == 2)
    {
      lRetVal = ((p[0] == aSize.p[0]) && (p[1] == aSize.p[1]));
    }
    else
    {
      /* case arbitrary dimensions */
      for(int32_t lIt = 0; lIt < lD; lIt++)
      {
        if(p[lIt] != aSize.p[lIt])
        {
          lRetVal = false;
          break;
        }
      }
    }
  }
  return lRetVal;
}

/**
 * Comparison operator
 */
bool vsdk::MatSize::operator!=(const vsdk::MatSize& aSize) const
{
  return (!(*this == aSize));
}

/**
 * Default constructor
 */
vsdk::MatStep::MatStep()
{
  p    = buf;
  p[0] = 1U;
  p[1] = 1U;
  p[2] = 1U;
}

/**
 * Constructor with specified value
 */
vsdk::MatStep::MatStep(const uint64_t aStep)
{
  p    = buf;
  p[0] = aStep;
  p[1] = 0U;
  p[2] = 1U;
}

/**
 * Get array element operator - returns the step on specified index (const)
 */
const uint64_t& vsdk::MatStep::operator[](const int32_t aIndex) const
{
  return p[aIndex];
}

/**
 * Get array element operator - returns the step on specified index
 */
uint64_t& vsdk::MatStep::operator[](const int32_t aIndex)
{
  return p[aIndex];
}

/**
 * Get array element operator - returns the step on specified index
 */
vsdk::MatStep::operator uint64_t() const
{
  return buf[0];
}

/**
 * Assignment operator
 */
vsdk::MatStep& vsdk::MatStep::operator=(const uint64_t aStep)
{
  buf[0] = aStep;
  return *this;
}
