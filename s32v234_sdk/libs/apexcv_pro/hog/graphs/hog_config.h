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
*  \brief APU Kernel and ACF configuration definitions for HOG.
***********************************************************************************/
#ifndef HOGCONFIG_H
#define HOGCONFIG_H

struct Size
{
  unsigned int width;
  unsigned int height;
};

struct HogFixedParams
{
  Size cell;
  Size blockCells;
  Size block;
  Size blockStride;
  Size winStride;
  Size winStridePerBlock;
  unsigned int nbins;
};

// constant parameters
static const HogFixedParams gc_hogFixedParams = {{   8,   8},   /* cell size in pixels */
                                                 {   1,   1},   /* cells per block */
                                                 {   8,   8},   /* block size in pixels */
                                                 {   8,   8},   /* blockStride in pixels */
                                                 {   4,   4},   /* window stride in pixels */
                                                 {   2,   2},   /* window strides per block */
                                                 8};            /* number of orientation bins */

// APEX requires the size of the detection window already in compilation time in order to compute the size of chunks.
// That's why we define a maximum windows size. The required window size will be given in run time, but should not exceed the max.
static const Size gc_maxWinBlocks =              {  16,  16};   /* max blocks per detection window */

#endif   /* HOGCONFIG_H */

