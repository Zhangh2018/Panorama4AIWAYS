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
*  \brief Gradient magnitude and angle bin C reference code.
*
*  Contains the C reference code for computing the gradient
*  magnitude and angle bin images used to construct the HOG descriptor.
***********************************************************************************/

#ifndef HOGGRADIENTREF_H
#define HOGGRADIENTREF_H

#include "hog_types_ref.h"

#ifdef __cplusplus

/*!*********************************************************************************
*  Allocate memory for the Gradient Cache based on an image size.
***********************************************************************************/
void HOG_allocateGradientCache(HOG_GradientCache& cache /*! The HOG gradient cache.*/,
                               const Size& gradSize     /*! Size of the gradient image required.*/);

/*!*********************************************************************************
*  Allocate memory for the Gradient Cache.
***********************************************************************************/
void HOG_allocateGradientCache(HOG_GradientCache& cache,
                               const Rect& imageROI);

void HOG_deleteGradientCache(HOG_GradientCache& cache);

#endif   //#ifdef __cplusplus

void HOG_computeGradient(HOG_GradientCache& cache,
                         const Mat8u& img,
                         const Rect& imgROI);

#endif   /* HOGGRADIENTREF_H */
