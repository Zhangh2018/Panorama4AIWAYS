/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
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
/**
* \file APU_vbool.h
* \brief APU vector boolean type
* \author Igor Aleksandrowicz
* \version
* \date 26-August-2013
****************************************************************************/

#ifndef APUVBOOL_HPP
#define APUVBOOL_HPP

#include "apu_config.hpp"
#include "apu_cycles.hpp"
#include <cassert>

namespace APEX2
{

/****************************************************************************
* globals
****************************************************************************/

//global vector condition state used for disabling CUs
extern bool gEnabledCUs[VSIZE];
extern int _tileWidthInChunks;

/****************************************************************************
* types
****************************************************************************/

typedef bool vbool_elem_T;

class vbool
{
public:
  //vector boolean elements are public for easier interaction with the rest of the library and easier debugging
  //however this member does NOT exist in APU-C and shouldn't be used for the final code
  vbool_elem_T mElements[VSIZE];

  vbool();
  //broadcast
  vbool(const vbool_elem_T& arValue);

  vbool& operator=(const vbool& arRhs);

  //logical operations
  vbool operator== (const vbool& arOther) const;
  vbool operator!= (const vbool& arOther) const;

  vbool operator&& (const vbool& arOther) const;
  vbool operator|| (const vbool& arOther) const;
  vbool operator!() const;
  vbool operator~();

  

private:

  ///vector boolean move shifts and rotates declarations
  ///these functions don't modify *this
  vbool moveShiftLeft(const vbool& arFillVector) const;
  vbool moveShiftRight(const vbool& arFillVector) const;
  vbool moveRotateLeft(const vbool& arFillVector) const;
  vbool moveRotateRight(const vbool& arFillVector) const;

  ///vector boolean accumulation funtions declarations
  ///are all elements true / are any elements true?
  ///use caution when combining vall, vany with vif/velse, in this library they take only active CUs into account, in real ACF vany works like that but vall seems to be incorrect, vall seems to always returns false when some CUs are inactive
  vbool_elem_T all() const;
  vbool_elem_T any() const;

  //APU-C vector boolean move shifts and rotations operations are declared as friends

  /** vectorial move-shift-left
  *Move all values one CU to the left. Fill in the right-most CU with the right-most value in arVectorFill
  *
  *@parameter arVectorShift	- Vector to be shifted
  *@parameter arVectorFill	- Vector from which the right most value is filled
  */
  friend vbool vmsl(const vbool& arVectorShift, const vbool& arVectorFill);

  /** vectorial move-shift-right
  *Move all values one CU to the right. Fill in the left-most CU with the left-most value in arVectorFill
  *
  *@parameter arVectorShift	- Vector to be shifted
  *@parameter arVectorFill	- Vector from which the left most value is filled
  */
  friend vbool vmsr(const vbool& arVectorShift, const vbool& arVectorFill);

  /** vectorial move-rotate-left
  *Move all values one CU to the left. Fill in the right-most CU with the left-most value of the arVectorFill
  *
  *@parameter arVectorShift	- Vector to be shifted
  *@parameter arVectorFill	- Vector from which the left most value is filled
  */
  friend vbool vmrl(const vbool& arVectorRotate, const vbool& arVectorFill);
  
  /** vectorial move-rotate-right
  *Move all values one CU to the right. Fill in the left-most CU with the right-most value in arVectorFill
  *
  *@parameter arVectorShift	- Vector to be shifted
  *@parameter arVectorFill	- Vector from which the right most value is filled
  */
  friend vbool vmrr(const vbool& arVectorRotate, const vbool& arVectorFill);


  /** vectorial move-shift-left
  *Move all values one CU to the left. Fill in the right-most CU with the right-most value is set to zero
  *
  *@parameter arVectorShift	- Vector to be shifted
  */
  friend vbool vmsl(const vbool& arVectorShift);

  /** vectorial move-shift-right
  *Move all values one CU to the right. Fill in the left-most CU with the left-most value is filled with zero
  *
  *@parameter arVectorShift	- Vector to be shifted
  */
  friend vbool vmsr(const vbool& arVectorShift);

  /** vectorial move-rotate-left
  *Move all values one CU to the left. Fill in the right-most CU with the left-most value of the same vector
  *
  *@parameter arVectorShift	- Vector to be shifted
  */
  friend vbool vmrl(const vbool& arVectorRotate);

  /** vectorial move-rotate-right
  *Move all values one CU to the right. Fill in the left-most CU with the right-most value of the same vector
  *
  *@parameter arVectorShift	- Vector to be shifted
  */
  friend vbool vmrr(const vbool& arVectorRotate);

  /**
  *Depending on vc, select either values from va or vb
  *
  *@parameter va Source vector
  *@parameter vb Source Vector
  *@parameter vc boolean selection vector
  *@return if (vc ==true) va, else vb
  */
  friend vbool vselect(const vbool& va, const vbool& vb, const vbool& vc);

  //APU-C vector boolean accumulation functions declared as friends
  /**
  *@return true, if all cu's contain a true value
  */
  friend bool vall(const vbool& arCheck);

  /**
  *@return true, if at least one of the cu's contain a true value
  */
  friend bool vany(const vbool& arCheck);
};


/****************************************************************************
* non-member functions
****************************************************************************/
vbool vselect(const vbool& va, const vbool& vb, const vbool& vc);

//APU-C operations declarations

/**
@return arVector[CU[aIndex]], i.e. the value of arVector in the aIndex-th CU
*/
vbool_elem_T vget(const vbool& arVector, int aIndex);

/** Set at aIndex-th the arValue into arVector
@param  arVector	- destination vector
@param	arValue		- value to be set into destination
@param	aIndex		- CU index where to set the value 
@return aVec[CU[aIndex]] = arValue; and aVec[all other CUs] = arVector[all other CUs]
*/
vbool vput(const vbool& arVector, const vbool_elem_T& arValue, int aIndex);

vbool vmsl(const vbool& arVectorShift, const vbool& arVectorFill);
vbool vmsr(const vbool& arVectorShift, const vbool& arVectorFill);
vbool vmrl(const vbool& arVectorRotate, const vbool& arVectorFill);
vbool vmrr(const vbool& arVectorRotate, const vbool& arVectorFill);
vbool vmsl(const vbool& arVectorShift);
vbool vmsr(const vbool& arVectorShift);
vbool vmrl(const vbool& arVectorRotate);
vbool vmrr(const vbool& arVectorRotate);

bool vall(const vbool& arCheck);
bool vany(const vbool& arCheck);

//this vector boolean type std::string declaration is needed here
//see apu_cycles.hpp
template <> std::string GetVectorTypeString<vbool>();

}

#endif /* APUVBOOL_HPP */
