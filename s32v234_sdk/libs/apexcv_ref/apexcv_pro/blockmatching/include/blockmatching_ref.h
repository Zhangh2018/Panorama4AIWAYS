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

/*!*********************************************************************************
*  \file
*  \brief Reference kernels for block matching algorithm
***********************************************************************************/

#ifndef BLOCKMATCHINGREF_H
#define BLOCKMATCHINGREF_H

#include <stdint.h>

int BlockmatchingRef(uint32_t* pOutput0,
                      uint8_t* pOutput0Status,
                      uint8_t* pInputTemp, int ist, int iwt, int iht,
                      uint8_t* pInputWin, int isw, int iww, int ihw,
                      uint32_t* pPointsArray,
                      int sadThreshold,
                      int num_points,
                      int tsx, int tsy,
                      int wsx, int wsy,
                      int ncu_x, int ncu_y);



#endif /* BLOCKMATCHINGREF_H */ 
