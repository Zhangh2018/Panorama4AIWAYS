/*
 * Copyright (c) 2016 Freescale Semiconductor
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
* \file     ov9716_types.h
* \brief    types declarations for Ov9716 camera driver
* \author   Tomas Babinec
* \version  0.1
* \date     14.10.2016
* \note
****************************************************************************/
#ifndef OV9716TYPES_H
#define OV9716TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(__KERNEL__) && !defined(__STANDALONE__)
#include <stdint.h>

#endif // if !defined(__KERNEL__) && !defined(__STANDALONE__)

//*****************************************************************************
// CONST
//*****************************************************************************
#define OV9716_EXP_12COM 0  ///< 12bit combined mode
#define OV9716_EXP_LONG  1  ///< long exposure only
#define OV9716_EXP_SHRT  2  ///< short exposure only
#define OV9716_EXP_VSHRT 4  ///< veryshort exposure only
#define OV9716_EXP_3x12  8  ///< 3x12 exposure mode
#define OV9716_EXP_CNLG  16 ///< apply channel gain setup
#define OV9716_EXP_AUTO  32 ///< automatic exposure mode

#define OV9716_EMBD_OFF  0  ///< turn OFF the embedded data
#define OV9716_EMBD_REGS 1  ///< turn ON register part of the embedded data
#define OV9716_EMBD_STAT 2  ///< turn ON statistics part of the embedded data

#define OV9716_EXP_FL_RG    1       ///< real gain flag
#define OV9716_EXP_FL_RATIO 2       ///< ratios flag
#define OV9716_EXP_FL_EV    4       ///< EV bytes flag
#define OV9716_EXP_FL_CNLG  8       ///< channel gain

#define OV9716_RESET_POLARITY 0  // active low
#define OV9716_RESET_DELAY    10
#define OV9716_PWDN_POLARITY  0  // active low
#define OV9716_PWDN_DELAY     10

//*****************************************************************************
// types
//*****************************************************************************

typedef struct OV9716_Geometry
{
  enum CSI_IDX   mCsiIdx; ///< CSI interface IDX
  CamI2cClient_t mClient; ///< camera client handle

  // image size
  uint32_t mDvpVsize;
  uint32_t mDvpHsize;

  // image flip
  uint8_t mVerFlip;
  uint8_t mHorFlip;

  // FPS
  uint8_t mFps;     ///< frames per second

  // EMBD data
  uint8_t mEmbd;    ///< embedded data controll

  // single/dual exposure
  uint8_t mExpMode; ///< OV9716_EXP_<mode>
} OV9716_Geometry_t;

//*****************************************************************************
typedef enum OV9716_ExposureType
{
  OV9716_ExpType_VS = 0,
  OV9716_ExpType_LCG= 1,
  OV9716_ExpType_HCG= 2
} OV9716_ExposureType_t;

//*****************************************************************************

typedef struct OV9716_Exposure
{
  enum CSI_IDX   mCsiIdx; ///< CSI interface IDX
  CamI2cClient_t mClient; ///< camera client handle

  uint16_t mRealGainLong;
  uint16_t mRealGainShort;
  uint16_t mRealGainVeryShort;

  uint16_t mLongShortRatio;
  uint16_t mLongVeryShortRatio;
  uint16_t mShortVeryShortRatio;

  uint8_t  mLongestEvBytes[4];
  uint8_t  mShortestEvBytes[4];

    // Exposure parameters
  uint16_t mDCGExposureTime;
  uint8_t  mDCGDeflickeringEnable;
  float	   mDCGFlickerPeriod;
  uint16_t mDCGExpCoef;			// For deflicering case
  uint16_t mVSExposureTime; 	//8.5b
  // uint16_t mVSFracExposureTime;
  uint8_t mVSConversionGainEnable;	//0 or 1 : enable if the correction gain can be used in AEC
  uint8_t mVSConversionGainOn;	//0 or 1
  float  mVSConversionGain;		// 1 or the conversion gain factor
  uint8_t  mAnalogGain[3];	// x1, x2 or x4 (1b, 10b, 11b)
  uint16_t mDigitalGain[3]; // 6.8b values

  // float mExposureCG[3];		// Total exposure with CG included for HCG							// including conversion gain
  float mExposure[3];		// Total exposure without CG included for HCG
  float mRatioHLExpCG;		// Target ratio between HCG and LCG exposure
							// including conversion gain
  float mRatioHLExp;		// Target ratio between HCG and LCG exposure
							// without conversion gain
  float mRatioMargin;		// Margin on which real ratio nust be [(mRatioMargin*mRatioHLExp);((1/mRatioMargin)*mRatioHLExp)]
  uint8_t mExposurePriority;	// For which exposure (HCG or LCG) in priority does
							// the AEC tries to maximise the exposure time

  float mAECFactor;


  // Min/Max Exposure parameters
  uint16_t mMinDCGExposureTime;
  uint16_t mMinVSExposureTime;
  uint8_t  mMinAnalogGain[3];
  uint16_t mMinDigitalGain[3];

  uint16_t mMaxDCGExposureTime;
  uint16_t mMaxVSExposureTime;
  uint8_t  mMaxAnalogGain[3];
  uint16_t mMaxDigitalGain[3];

  float mMinExposure[3];
  float mMaxExposure[3];

  // channel gain

  uint16_t mpLongChnlGain[4];
  uint16_t mpShortChnlGain[4];
  uint16_t mpVeryShortChnlGain[4];

  uint8_t  mConfigFlags;     ///< what should be set; all if 0
} OV9716_Exposure_t;

//*****************************************************************************

typedef struct OV9716_ChGain
{
  enum CSI_IDX   mCsiIdx;   ///< CSI interface IDX
  CamI2cClient_t mClient;   ///< camera client handle

  uint16_t mLevel[4];       ///< one per channel
  uint16_t mMax;            ///< maximum Level
  uint16_t mMin;            ///< minium Level

}  OV9716_ChGain_t;

//*****************************************************************************

#ifdef __cplusplus
} //extern "C" {
#endif

#endif /* OV9716TYPES_H */
