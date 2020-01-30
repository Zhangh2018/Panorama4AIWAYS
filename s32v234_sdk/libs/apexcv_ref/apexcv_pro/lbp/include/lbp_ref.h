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

#ifndef LBPREF_H
#define LBPREF_H

/*!****************************************************************************
* \brief Extract descriptor
* \param[in] apSrc Pointer to source image
* \param[in] aSstr Source image stride
* \param[out] apDst Pointer to the destination image
* \param[in] aChunkWidth Process chunk width
* \param[in] aChunkHeight Process chunk height
******************************************************************************/
void ExtractDescriptor(unsigned char *apSrc,
                       unsigned short aSstr,
                       unsigned char *apDst,
                       unsigned short aChunkWidth,
                       unsigned short aChunkHeight);

/*!****************************************************************************
* \brief Compare two descriptors
* \param[in] apSrc0 Pointer to the first histogram
* \param[in] apSrc1 Pointer to the second histogram
* \param[out] apDst Pointer to the destination buffer
* \param[in] aChunkSize Process chunk size
******************************************************************************/
void CompareDescriptor(unsigned char *apSrc0,
                       unsigned char *apSrc1,
                       int *apDdst,
                       unsigned short aChunkSize);

/*!****************************************************************************
* \brief Accumulate distance
* \param[in] apSrc Pointer to source buffer
* \param[out] apDst Pointer to destination buffer
******************************************************************************/
void AccumulateDistance(int *apSrc,
                        int *apDst);

#endif /* LBPREF_H */ 
