/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP
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
*  \file apexcv_arithmetic_ref.hpp
*  \brief Reference code for arithmetic kernels
***********************************************************************************/

#ifndef ARITHMETICREF_H
#define ARITHMETICREF_H

#include <sumat.hpp>
#include <apexcv_enum_types.hpp>

template <typename S, typename D>
void abs_ref(const S* src, int sstr,
  D* dst, int dstr,
  int blockWidth, int blockHeight);

template <typename D>
void absdiff_ref(const D* src0, int sstr0,
  const D* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight);

void accumulate_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst);

void accumulate_squared_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, uint8_t aScale, vsdk::SMat& aDst);

void accumulate_weighted_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, float aAlpha ,vsdk::SMat& aDst);

void add_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);

void sub_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);

void phase_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst);

template <typename D>
void and_ref(const D* src0, int sstr0,
  const D* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight);

template <typename S, typename D>
void clz_ref(S* src, int sstr,
  D* dst, int dstr,
  int blockWidth, int blockHeight);

template <typename S, typename D>
void magnitude_ref(const S* src0, int sstr0,
  const S* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight);

template <typename S, typename D>
void max_ref(const S* src0, int sstr0,
  const S* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight);

template <typename S, typename D>
void min_ref(const S* src0, int sstr0,
  const S* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight);

int mul_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aScale, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);

template <typename D>
void not_ref(const D* src0, int sstr0,
  D* dst, int dstr,
  int blockWidth, int blockHeight);

template <typename D>
void or_ref(const D* src0, int sstr0,
  const D* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight);

template <typename D>
void tablelookup_ref(const D* src0, int sstr0,
  D* dst, int dstr,
  int blockWidth, int blockHeight,
  const D* lut);

template <typename D>
void threshold_ref(const D* src0, int sstr0,
  D threshold,
  unsigned char* dst, int dstr,
  int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);

template <typename D>
void threshold2_ref(const D* src0, int sstr0,
  D threshold1, D threshold2,
  unsigned char* dst, int dstr,
  int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);


template <typename D>
void xor_ref(const D* src0, int sstr0,
  const D* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight);

#endif /* ARITHMETICREF_H */
