/*****************************************************************************
* 
* NXP Confidential Proprietary
*
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

#ifndef BRIEFAPEXREF_H
#define BRIEFAPEXREF_H

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <stdint.h>
#include "icp_data.h"
#include <assert.h>
#include <vector>
#include <limits.h>
#include <sumat.hpp>
#include <oal.h>

void brief_ref(vsdk::SUMat &aInputImg,
               std::vector<signed char> &aBitPattern,
               std::vector<unsigned int> &aKeypoints,
               uint8_t *apDescrOut,
               int32_t aFilterType,
               int32_t aBorderSize,
               int32_t aDescrSizeB);

#endif /* BRIEFAPEXREF_H */
