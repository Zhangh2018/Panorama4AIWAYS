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

#ifndef REFERENCERESIZE_H
#define REFERENCERESIZE_H

/*!*********************************************************************************
*  @file
*  @brief Delcaration of reference image resize implmentation.
***********************************************************************************/


/*!*********************************************************************************
*  \brief Vertical Resize
*
*  This method performs vertical resize.  Currently, only 8-bit
*  unsigned images are supported.
*
*   \param src  - [Input]   Pointer to input image.
*   \param sw   - [Input]   Width of input image in pixels.
*   \param sh   - [Input]   Height of input image in pixels.
*   \param ss   - [Input]   Span of input image in pixels.
*   \param dst  - [Input/Output]  Pointer to output image.  Memory buffer is allocated by caller.
*   \param dw   - [Input]  Width of the output image in pixels.
*   \param dh   - [Input]  Height of output image in pixels.
*   \param ds   - [Input]  Span of output image in bytes.
*
***********************************************************************************/

int
vertical_Scale_08U(uint8_t *src, int sw, int sh, int ss,
                   uint8_t *dst, int dw, int dh, int ds);
                   
/*!*********************************************************************************
*  \brief Horizontal Resize
*
*  This method performs horizontal resize.  Currently, only 8-bit
*  unsigned images are supported.
*
*   \param src  - [Input]   Pointer to input image.
*   \param sw   - [Input]   Width of input image in pixels.
*   \param sh   - [Input]   Height of input image in pixels.
*   \param ss   - [Input]   Span of input image in pixels.
*   \param dst  - [Input/Output]  Pointer to output image.  Memory buffer is allocated by caller.
*   \param dw   - [Input]  Width of the output image in pixels.
*   \param dh   - [Input]  Height of output image in pixels.
*   \param ds   - [Input]  Span of output image in bytes.
*
***********************************************************************************/

                   
int
horizontal_Scale_08U(uint8_t *src, int sw, int sh, int ss,
                     uint8_t *dst, int dw, int dh, int ds);

#endif /* REFERENCERESIZE_H */
