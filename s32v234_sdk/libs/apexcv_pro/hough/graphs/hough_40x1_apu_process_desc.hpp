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
*  @file
*  @brief ACF process description for Hough Transform.
***********************************************************************************/
#ifndef HOUGH40X1PROCESSDESC_HPP
#define HOUGH40X1PROCESSDESC_HPP

#include "acf_process_desc_apu.hpp"
#include "hough_40x1_graph.hpp"
#include "hough_config_acf.h"

/*!*********************************************************************************
*  \brief ACF process description for Hough Transform
*  \see UG-10267-03 ACF User Guide, Section 3.4
***********************************************************************************/
class hough_40x1_process_desc : public ACF_Process_Desc_APU
{
public:
/*!*********************************************************************************
*  \brief Create the ACF process description.
*
*  In this function we
*     - initialize the process and give it a unique identifier.
*     - set the input chunk size
***********************************************************************************/
void Create()
{
  Initialize(mHoughGraph, "HOUGH_40X1");
  SetInputChunkSize("IMAGE", 40, 1);

  //*********RTL_SIM *****************
#define ALIGNE_ADDRESS(A) ((((A)+15) >> 4) << 4)

  const int32_t imageRows = 32;
  const int32_t imageCols = 40 * HOUGH_CU_COUNT;
  const int32_t imageSpan = imageCols;
  const int32_t imageSize = imageSpan * imageRows;

  const int32_t tableRows = 1;
  const int32_t tableCols = HOUGH_CU_COUNT;
  const int32_t tableSpan = 2 * tableCols;
  const int32_t tableSize = tableSpan * tableRows;

  const int32_t linesRows = 1;
  const int32_t linesCols = HOUGH_MAX_LINES_PER_CU * HOUGH_CU_COUNT;
  const int32_t linesSpan = 4 * linesCols;
  const int32_t linesSize = linesSpan * linesRows;

  const int32_t pixelThreshCols = 1; // 1 x int16s
  const int32_t pixelOffsetCols = 4; // 4 x int16s
  const int32_t cuCountCols     = 1; // 1 x int16s
  const int32_t rhoCountCols    = 1; // 1 x int16s
  const int32_t cuIndexCols     = 2; // 2 x int16s
  const int32_t lineThreshCols  = 6; // 6 x int16s
  const int32_t lineCountCols   = 1; // 1 x int32s

  const int32_t pixelThreshSize = 2 * pixelThreshCols; // 1 x int16s
  const int32_t pixelOffsetSize = 2 * pixelOffsetCols; // 4 x int16s
  const int32_t cuCountSize     = 2 * cuCountCols; // 1 x int16s
  const int32_t rhoCountSize    = 2 * rhoCountCols; // 1 x int16s
  const int32_t cuIndexSize     = 2 * cuIndexCols; // 2 x int16s
  const int32_t lineThreshSize  = 2 * lineThreshCols; // 6 x int16s
  const int32_t lineCountSize   = 4 * lineCountCols; // 1 x int32s

  const int32_t lDmaChIn     = 0;
  const int32_t lDmaChOut    = 1;
  const int32_t lArrayWidth  = HOUGH_CU_COUNT;
  const uint32_t lExtMemAddr = 0x30000000;
  const uint32_t lSmemAddr   = 0x10000000;

  // Input Memory Addresses
  const uint32_t imageAddress       = ALIGNE_ADDRESS(lExtMemAddr);
  const uint32_t pixelThreshAddress = imageAddress + ALIGNE_ADDRESS(imageSize);
  const uint32_t pixelOffsetAddress = pixelThreshAddress + ALIGNE_ADDRESS(pixelThreshSize);
  const uint32_t cuCountAddress     = pixelOffsetAddress + ALIGNE_ADDRESS(pixelOffsetSize);
  const uint32_t cosTableAddress    = cuCountAddress + ALIGNE_ADDRESS(cuCountSize);
  const uint32_t sinTableAddress    = cosTableAddress + ALIGNE_ADDRESS(tableSize);
  const uint32_t rhoCountAddress    = sinTableAddress + ALIGNE_ADDRESS(tableSize);
  const uint32_t cuIndexAddress     = rhoCountAddress + ALIGNE_ADDRESS(rhoCountSize);
  const uint32_t lineThreshAddress  = cuIndexAddress + ALIGNE_ADDRESS(cuIndexSize);

  // Output Memory Addresses
  const uint32_t lineCountAddress = lineThreshAddress + ALIGNE_ADDRESS(lineThreshSize);
  const uint32_t linesAddress     = lineCountAddress + ALIGNE_ADDRESS(lineCountSize);

  RtlSim_Init(lArrayWidth, lDmaChIn, lDmaChOut, lSmemAddr);

  // Connect Input
  RtlSim_ConnectIO("IMAGE", imageCols, imageRows, imageSpan, ICP_DATATYPE_08U, 1, 1, imageAddress);
  RtlSim_ConnectIO("PIXEL_THRESHOLD", pixelThreshCols, 1, pixelThreshSize, ICP_DATATYPE_16S, 1, 1, pixelThreshAddress);
  RtlSim_ConnectIO("PIXEL_OFFSET_PARAMS", pixelOffsetCols, 1, pixelOffsetSize, ICP_DATATYPE_16S, 1, 1, pixelOffsetAddress);
  RtlSim_ConnectIO("CU_COUNT", cuCountCols, 1, cuCountSize, ICP_DATATYPE_16S, 1, 1, cuCountAddress);
  RtlSim_ConnectIO("COS_TABLE", tableCols, tableRows, tableSpan, ICP_DATATYPE_16S, 1, 1, cosTableAddress);
  RtlSim_ConnectIO("SIN_TABLE", tableCols, tableRows, tableSpan, ICP_DATATYPE_16S, 1, 1, sinTableAddress);
  RtlSim_ConnectIO("RHO_COUNT", rhoCountCols, 1, rhoCountSize, ICP_DATATYPE_16S, 1, 1, rhoCountAddress);
  RtlSim_ConnectIO("CU_INDEX_PARAMS", cuIndexCols, 1, cuIndexSize, ICP_DATATYPE_16S, 1, 1, rhoCountAddress);
  RtlSim_ConnectIO("LINE_THRESHOLD_PARAMS", lineThreshCols, 1, lineThreshSize, ICP_DATATYPE_16S, 1, 1, lineThreshAddress);

  // Connect Output
  RtlSim_ConnectIO("LINE_COUNT", lineCountCols, 1, lineCountSize, ICP_DATATYPE_32S, 1, 1, lineCountAddress);
  RtlSim_ConnectIO("LINES", linesCols, linesRows, linesSpan, ICP_DATATYPE_32U, 1, 1, linesAddress);
}

Hough_Graph mHoughGraph;
};

#endif /* HOUGH40X1PROCESSDESC_HPP */

