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
*  \file apexcv_affine_transform_ref.hpp
*  \brief Reference code for affine transform
***********************************************************************************/

#ifndef AFFINEREF_H
#define AFFINEREF_H

/*!*********************************************************************************
*  \brief Reference implementation for the \ref secAffineTransform "affine transformation".
*  Compare results from APU execution and reference implementation for the affine transformation.
*  
*  First, buffers are allocated and wrapped in contiguous data descriptors.
*  \snippet apexcv_affine_transform_ref.cpp allocate reference memory
*  Then the reference code is executed
*  \snippet apexcv_affine_transform_ref.cpp ref
*  and the result is compared with the APU result.
*  \snippet apexcv_affine_transform_ref.cpp compare
*  Before returning, reference memories are released.
*  \snippet apexcv_affine_transform_ref.cpp free reference memory
*
*  \return 0 if the results match, 1 otherwise.
***********************************************************************************/
int
ref_affine_transform(uint8_t *dst,      /*!< Pointer to the source image.*/
               const uint8_t *src,      /*!< Pointer to the apu affine transform result.*/
                     int bw,            /*!< Source image width.*/
                     int bh,            /*!< Source image height.*/
               const int *matrix,       /*!< Source matrix.*/
               const uint16_t *offset,   /*!< Global Offset.*/
               int num_cu
);


#endif /* AFFINEREF_H */
