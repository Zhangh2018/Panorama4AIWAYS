/*****************************************************************************
 *
 * Copyright (c) 2017-2018 NXP
 * All Rights Reserved
 *
 *****************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#ifdef  __cplusplus
extern "C" {
#endif

#include "seq_graph.h"
#include "seq_graph_meta.h"

#include "ov10640_12bcomb_surround_c.h"

#include "iofiles.h"
#ifndef S32DS_BUILD
#include "kmem.c"
#endif
#include "kmem_sym.h"
// *** forwards needed for cross-linking ***

static SEQ_FdmaCfg_t FastDMA;
static SEQ_MipiCfg_t OV_QUAD;
static SEQ_IpuCfg_t Debayer0;
static SEQ_IpuCfg_t RGB2Y0;
static SEQ_IpuCfg_t Debayer1;
static SEQ_IpuCfg_t RGB2Y1;
static SEQ_IpuCfg_t LUT4K0;
static SEQ_IpuCfg_t COMPRESS0;
static SEQ_IpuCfg_t LUT4K1;
static SEQ_IpuCfg_t COMPRESS1;
static SEQ_Load_t sCodedebayer_even_Debayer0;
static SEQ_Load_t sCodergb2y_RGB2Y0;
static SEQ_Load_t sCodedebayer_odd_Debayer1;
static SEQ_Load_t sCodergb2y_RGB2Y1;
static SEQ_Load_t sCodelut4k_LUT4K0;
static SEQ_Load_t sCodergb_compress_COMPRESS0;
static SEQ_Load_t sCodelut4k_LUT4K1;
static SEQ_Load_t sCodergb_compress_COMPRESS1;

/*** Buffers ****************************************************************/

SEQ_Buf_t OV_QUADBuffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  16,				// *** number of lines fitting into output buffer (wrap) ***
  IN_PIXELS * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(IN_STRIDE16),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 2556 /*OV_QUAD*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 76 /* Debayer0.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      IN_PIXELS * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      2,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 76 /* Debayer0.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      2,				// *** number of lines required for processing ***
      IN_PIXELS * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      2,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 76 /* Debayer0.mHead */,	// *** offset to consumers (node) configuration header ***
      2,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      3,				// *** number of lines required for processing ***
      IN_PIXELS * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      2,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 708 /* Debayer1.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      2,				// *** number of lines required for processing ***
      IN_PIXELS * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      2,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 708 /* Debayer1.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      3,				// *** number of lines required for processing ***
      IN_PIXELS * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      2,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 708 /* Debayer1.mHead */,	// *** offset to consumers (node) configuration header ***
      2,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      4,				// *** number of lines required for processing ***
      IN_PIXELS * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      2,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // OV_QUADBuffer0

SEQ_Buf_t COMPRESS0Buffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  12,				// *** number of lines fitting into output buffer (wrap) ***
  OUT_PIXELS_RGB * 1,				// *** number of bytes per line to read ***
  BUF_ALIGN(OUT_STRIDE8_RGB),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1632 /*COMPRESS0*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 2592 /* FastDMA.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      OUT_PIXELS_RGB * 1,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // COMPRESS0Buffer0

SEQ_Buf_t Debayer0Buffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  6,				// *** number of lines fitting into output buffer (wrap) ***
  DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(DEBAYER_STRIDE16),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 76 /*Debayer0*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 400 /* RGB2Y0.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 1632 /* COMPRESS0.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // Debayer0Buffer0

SEQ_Buf_t Debayer0Buffer2 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  6,				// *** number of lines fitting into output buffer (wrap) ***
  DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(DEBAYER_STRIDE16),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 76 /*Debayer0*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 400 /* RGB2Y0.mHead */,	// *** offset to consumers (node) configuration header ***
      2,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 1632 /* COMPRESS0.mHead */,	// *** offset to consumers (node) configuration header ***
      2,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // Debayer0Buffer2

SEQ_Buf_t Debayer0Buffer1 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  6,				// *** number of lines fitting into output buffer (wrap) ***
  DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(DEBAYER_STRIDE16),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 76 /*Debayer0*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 400 /* RGB2Y0.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 1632 /* COMPRESS0.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // Debayer0Buffer1

SEQ_Buf_t Debayer1Buffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  6,				// *** number of lines fitting into output buffer (wrap) ***
  DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(DEBAYER_STRIDE16),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 708 /*Debayer1*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1032 /* RGB2Y1.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 2240 /* COMPRESS1.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // Debayer1Buffer0

SEQ_Buf_t Debayer1Buffer2 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  6,				// *** number of lines fitting into output buffer (wrap) ***
  DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(DEBAYER_STRIDE16),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 708 /*Debayer1*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1032 /* RGB2Y1.mHead */,	// *** offset to consumers (node) configuration header ***
      2,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 2240 /* COMPRESS1.mHead */,	// *** offset to consumers (node) configuration header ***
      2,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // Debayer1Buffer2

SEQ_Buf_t Debayer1Buffer1 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  6,				// *** number of lines fitting into output buffer (wrap) ***
  DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(DEBAYER_STRIDE16),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 708 /*Debayer1*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1032 /* RGB2Y1.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 2240 /* COMPRESS1.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      DEBAYER_OUTPIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // Debayer1Buffer1

SEQ_Buf_t COMPRESS1Buffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  12,				// *** number of lines fitting into output buffer (wrap) ***
  OUT_PIXELS_RGB * 1,				// *** number of bytes per line to read ***
  BUF_ALIGN(OUT_STRIDE8_RGB),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 2240 /*COMPRESS1*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 2592 /* FastDMA.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      OUT_PIXELS_RGB * 1,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // COMPRESS1Buffer0

SEQ_Buf_t RGB2Y0Buffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  4,				// *** number of lines fitting into output buffer (wrap) ***
  RGB2YUV_PIX * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(OUT_STRIDE8_YUV),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 400 /*RGB2Y0*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1340 /* LUT4K0.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      RGB2YUV_PIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // RGB2Y0Buffer0

SEQ_Buf_t LUT4K0Buffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  4,				// *** number of lines fitting into output buffer (wrap) ***
  RGB2YUV_PIX * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(RGB2YUV_STRIDE16),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1340 /*LUT4K0*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1632 /* COMPRESS0.mHead */,	// *** offset to consumers (node) configuration header ***
      3,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      RGB2YUV_PIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // LUT4K0Buffer0

SEQ_Buf_t RGB2Y1Buffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  4,				// *** number of lines fitting into output buffer (wrap) ***
  RGB2YUV_PIX * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(RGB2YUV_STRIDE16),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1032 /*RGB2Y1*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1948 /* LUT4K1.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      RGB2YUV_PIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // RGB2Y1Buffer0

SEQ_Buf_t LUT4K1Buffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  4,				// *** number of lines fitting into output buffer (wrap) ***
  RGB2YUV_PIX * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(RGB2YUV_STRIDE16),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1948 /*LUT4K1*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 2240 /* COMPRESS1.mHead */,	// *** offset to consumers (node) configuration header ***
      3,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      RGB2YUV_PIX * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // LUT4K1Buffer0

SEQ_Buf_t ISP_OUTPUT0Buffer = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 1,		// *** pointer to output buffer ***
  OUT_LINES2_0,				// *** number of lines fitting into output buffer (wrap) ***
  OUT_PIXELS_RGB * 1,				// *** number of bytes per line to read ***
  OUT_STRIDE8_RGB,				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 2592 /*FastDMA */,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      OUT_PIXELS_RGB * 1,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // ISP_OUTPUT0Buffer

SEQ_Buf_t ISP_OUTPUT1Buffer = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 1,		// *** pointer to output buffer ***
  OUT_LINES2_1,				// *** number of lines fitting into output buffer (wrap) ***
  OUT_PIXELS_RGB * 1,				// *** number of bytes per line to read ***
  OUT_STRIDE8_RGB,				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 2592 /*FastDMA */,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      OUT_PIXELS_RGB * 1,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #5 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #6 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #7 ***
    {
      (SEQ_Head_Ptr_t) 0,		// *** pointer to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      0,				// *** number of lines required for processing ***
      0,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      0,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },

  }
}; // ISP_OUTPUT1Buffer


static SEQ_CfgPara_t Debayer0Para [] = {
{ D_IPUS_SYN_HOST_XCFG_OFF, (DEBAYER_OUTPIX << 16) | 0x1 },
    { D_IPUS_SYN_HOST_INACFG_OFF, 0x00000700 },
    { D_IPUS_SYN_HOST_OUTCFG_OFF, 0x00000700 },
    { D_IPUS_SYN_S_CH0_CFG_INA_OFF, 0x00150000 },
    { D_IPUS_SYN_S_CH1_CFG_INA_OFF, 0x00150000 },
    { D_IPUS_SYN_S_CH2_CFG_INA_OFF, 0x00150000 },
    { D_IPUS_SYN_S_CH0_CFG_OUT_OFF, 0x00000000 },
    { D_IPUS_SYN_S_CH1_CFG_OUT_OFF, 0x00000000 },
    { D_IPUS_SYN_S_CH2_CFG_OUT_OFF, 0x00000000 },
};

static SEQ_CfgPara_t RGB2Y0Para [] = {
{ D_IPUV_SYN_HOST_XCFG_OFF, (RGB2YUV_PIX << 16) | 0x0 },
    { D_IPUV_SYN_HOST_INACFG_OFF, 0x00000700 },
    { D_IPUV_SYN_HOST_OUTCFG_OFF, 0x00000100 },
    { D_IPUV_SYN_S_CH0_CFG_INA_OFF, 0x00158000 },
    { D_IPUV_SYN_S_CH1_CFG_INA_OFF, 0x00158000 },
    { D_IPUV_SYN_S_CH2_CFG_INA_OFF, 0x00158000 },
    { D_IPUV_SYN_S_CH0_CFG_OUT_OFF, 0x00000000 },
};

static SEQ_CfgPara_t Debayer1Para [] = {
{ D_IPUS_SYN_HOST_XCFG_OFF, (DEBAYER_OUTPIX << 16) | 0x1 },
    { D_IPUS_SYN_HOST_INACFG_OFF, 0x00000700 },
    { D_IPUS_SYN_HOST_OUTCFG_OFF, 0x00000700 },
    { D_IPUS_SYN_S_CH0_CFG_INA_OFF, 0x00150000 },
    { D_IPUS_SYN_S_CH1_CFG_INA_OFF, 0x00150000 },
    { D_IPUS_SYN_S_CH2_CFG_INA_OFF, 0x00150000 },
    { D_IPUS_SYN_S_CH0_CFG_OUT_OFF, 0x00000000 },
    { D_IPUS_SYN_S_CH1_CFG_OUT_OFF, 0x00000000 },
    { D_IPUS_SYN_S_CH2_CFG_OUT_OFF, 0x00000000 },
};

static SEQ_CfgPara_t RGB2Y1Para [] = {
{ D_IPUV_SYN_HOST_XCFG_OFF, (RGB2YUV_PIX << 16) | 0x0 },
    { D_IPUV_SYN_HOST_INACFG_OFF, 0x00000700 },
    { D_IPUV_SYN_HOST_OUTCFG_OFF, 0x00000100 },
    { D_IPUV_SYN_S_CH0_CFG_INA_OFF, 0x00158000 },
    { D_IPUV_SYN_S_CH1_CFG_INA_OFF, 0x00158000 },
    { D_IPUV_SYN_S_CH2_CFG_INA_OFF, 0x00158000 },
    { D_IPUV_SYN_S_CH0_CFG_OUT_OFF, 0x00000000 },
};

static SEQ_CfgPara_t LUT4K0Para [] = {
{ D_IPUS_SYN_HOST_XCFG_OFF, (RGB2YUV_PIX << 16) | 0x1 },
    { D_IPUS_SYN_HOST_INACFG_OFF, 0x00000100 },
    { D_IPUS_SYN_HOST_OUTCFG_OFF, 0x00000100 },
    { D_IPUS_SYN_S_CH0_CFG_INA_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH0_CFG_OUT_OFF, 0x00000000 },
};

static SEQ_CfgPara_t COMPRESS0Para [] = {
{ D_IPUV_SYN_HOST_XCFG_OFF, (OUT_PIXELS << 16) | 0x0 },
    { D_IPUV_SYN_HOST_INACFG_OFF, 0x00000F00 },
    { D_IPUV_SYN_HOST_OUTCFG_OFF, 0x00000103 },
    { D_IPUV_SYN_S_CH0_CFG_INA_OFF, 0x00051000 },
    { D_IPUV_SYN_S_CH1_CFG_INA_OFF, 0x00058000 },
    { D_IPUV_SYN_S_CH2_CFG_INA_OFF, 0x0005e000 },
    { D_IPUV_SYN_S_CH3_CFG_INA_OFF, 0x00050100 },
    { D_IPUV_SYN_S_CH0_CFG_OUT_OFF, 0x10000000 },
};

static SEQ_CfgPara_t LUT4K1Para [] = {
{ D_IPUS_SYN_HOST_XCFG_OFF, (RGB2YUV_PIX << 16) | 0x1 },
    { D_IPUS_SYN_HOST_INACFG_OFF, 0x00000100 },
    { D_IPUS_SYN_HOST_OUTCFG_OFF, 0x00000100 },
    { D_IPUS_SYN_S_CH0_CFG_INA_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH0_CFG_OUT_OFF, 0x00000000 },
};

static SEQ_CfgPara_t COMPRESS1Para [] = {
{ D_IPUV_SYN_HOST_XCFG_OFF, (OUT_PIXELS << 16) | 0x0 },
    { D_IPUV_SYN_HOST_INACFG_OFF, 0x00000F00 },
    { D_IPUV_SYN_HOST_OUTCFG_OFF, 0x00000103 },
    { D_IPUV_SYN_S_CH0_CFG_INA_OFF, 0x00051000 },
    { D_IPUV_SYN_S_CH1_CFG_INA_OFF, 0x00058000 },
    { D_IPUV_SYN_S_CH2_CFG_INA_OFF, 0x0005e000 },
    { D_IPUV_SYN_S_CH3_CFG_INA_OFF, 0x00050100 },
    { D_IPUV_SYN_S_CH0_CFG_OUT_OFF, 0x10000000 },
};

static SEQ_MipiCfg_t OV_QUAD = {
  {
    SEQ_NODE_OTHER,			// *** type of engine ***
    ISP_IO_DEV_BASE+ISP_OFFS_FILEIO+2*ISP_IO_DEV_SPACE*0,	// *** address of hardware device ***
    SEQ_OTHRIX_MIPICSI0,			// *** index of requested engine ***
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },


  // *** output buffers ***
  {
      (SEQ_Buf_Ptr_t) 2800 /*OV_QUADBuffer0*/,			// *** pointer to output buffer ***
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
  },
  CAM_LINES,			// *** number of input image lines ***
  0			// *** pad alignment ***
}; // OV_QUAD

static SEQ_IpuCfg_t Debayer0 = {
  {
    SEQ_NODE_SCALAR,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 1*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    1,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },
  (0x00000000),			// *** code entry address ***

  0,					// *** current processing steps ***
  DEBAYER_LINES2_0,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3232 /* Debayer0Buffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3664 /* Debayer0Buffer1 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE1_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3448 /* Debayer0Buffer2 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE2_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
  },	// output buffer structs

  {
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2800 /* OV_QUADBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        3 * 2,					// *** number of bytes to first pixel in line ***
        DEBAYER_INPIX,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2800 /* OV_QUADBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
        1,					// *** number of first line to be used ***
        3 * 2,					// *** number of bytes to first pixel in line ***
        DEBAYER_INPIX,					// *** number of pixels to transfer ***
      },
      1,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2800 /* OV_QUADBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE2_LEN_INA_OFF,				// *** offset to SDMA's register ***
        2,					// *** number of first line to be used ***
        3 * 2,					// *** number of bytes to first pixel in line ***
        DEBAYER_INPIX,					// *** number of pixels to transfer ***
      },
      2,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
  },	// input buffer structs
  (SEQ_CfgPara_CPtr_t) 300 /*Debayer0Para */,	// *** pointer to configuration parameters ***
  9,					// *** number of configuration parameters following ***
  0,				// *** pad alignment ***
};

static SEQ_IpuCfg_t RGB2Y0 = {
  {
    SEQ_NODE_VECTOR,			// *** type of engine ***
    D_IPUV_SYN_BASEADDRESS,		// *** address of hardware device ***
    0,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },
  (0x00000000),			// *** code entry address ***

  0,					// *** current processing steps ***
  OUT_LINES2_0,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 4744 /* RGB2Y0Buffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUV_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        RGB2YUV_PIX,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
  },	// output buffer structs

  {
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3664 /* Debayer0Buffer1 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3232 /* Debayer0Buffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3448 /* Debayer0Buffer2 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE2_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
  },	// input buffer structs
  (SEQ_CfgPara_CPtr_t) 624 /*RGB2Y0Para */,	// *** pointer to configuration parameters ***
  7,					// *** number of configuration parameters following ***
  0,				// *** pad alignment ***
};

static SEQ_IpuCfg_t Debayer1 = {
  {
    SEQ_NODE_SCALAR,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 4*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    4,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },
  (0x00000000),			// *** code entry address ***

  0,					// *** current processing steps ***
  DEBAYER_LINES2_1,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3880 /* Debayer1Buffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 4312 /* Debayer1Buffer1 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE1_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 4096 /* Debayer1Buffer2 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE2_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
  },	// output buffer structs

  {
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2800 /* OV_QUADBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        1,					// *** number of first line to be used ***
        3 * 2,					// *** number of bytes to first pixel in line ***
        DEBAYER_INPIX,					// *** number of pixels to transfer ***
      },
      3,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2800 /* OV_QUADBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
        2,					// *** number of first line to be used ***
        3 * 2,					// *** number of bytes to first pixel in line ***
        DEBAYER_INPIX,					// *** number of pixels to transfer ***
      },
      4,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2800 /* OV_QUADBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE2_LEN_INA_OFF,				// *** offset to SDMA's register ***
        3,					// *** number of first line to be used ***
        3 * 2,					// *** number of bytes to first pixel in line ***
        DEBAYER_INPIX,					// *** number of pixels to transfer ***
      },
      5,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
  },	// input buffer structs
  (SEQ_CfgPara_CPtr_t) 932 /*Debayer1Para */,	// *** pointer to configuration parameters ***
  9,					// *** number of configuration parameters following ***
  0,				// *** pad alignment ***
};

static SEQ_IpuCfg_t RGB2Y1 = {
  {
    SEQ_NODE_VECTOR,			// *** type of engine ***
    D_IPUV_SYN_BASEADDRESS + 2*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    2,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },
  (0x00000000),			// *** code entry address ***

  0,					// *** current processing steps ***
  OUT_LINES2_1,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 5176 /* RGB2Y1Buffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUV_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        RGB2YUV_PIX,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
  },	// output buffer structs

  {
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 4312 /* Debayer1Buffer1 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3880 /* Debayer1Buffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 4096 /* Debayer1Buffer2 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE2_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
  },	// input buffer structs
  (SEQ_CfgPara_CPtr_t) 1256 /*RGB2Y1Para */,	// *** pointer to configuration parameters ***
  7,					// *** number of configuration parameters following ***
  0,				// *** pad alignment ***
};

static SEQ_IpuCfg_t LUT4K0 = {
  {
    SEQ_NODE_SCALAR,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 2*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    2,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },
  (0x00000000),			// *** code entry address ***

  0,					// *** current processing steps ***
  OUT_LINES2_0,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 4960 /* LUT4K0Buffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        RGB2YUV_PIX,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
  },	// output buffer structs

  {
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 4744 /* RGB2Y0Buffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        RGB2YUV_PIX,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
  },	// input buffer structs
  (SEQ_CfgPara_CPtr_t) 1564 /*LUT4K0Para */,	// *** pointer to configuration parameters ***
  5,					// *** number of configuration parameters following ***
  0,				// *** pad alignment ***
};

static SEQ_IpuCfg_t COMPRESS0 = {
  {
    SEQ_NODE_VECTOR,			// *** type of engine ***
    D_IPUV_SYN_BASEADDRESS + 1*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    1,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },
  (0x00000000),			// *** code entry address ***

  0,					// *** current processing steps ***
  OUT_LINES2_0,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3016 /* COMPRESS0Buffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUV_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        OUT_PIXELS_RGB,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
  },	// output buffer structs

  {
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3664 /* Debayer0Buffer1 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
      1,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3232 /* Debayer0Buffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
      1,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3448 /* Debayer0Buffer2 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE2_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
      1,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 4960 /* LUT4K0Buffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE3_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        RGB2YUV_PIX,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
  },	// input buffer structs
  (SEQ_CfgPara_CPtr_t) 1856 /*COMPRESS0Para */,	// *** pointer to configuration parameters ***
  8,					// *** number of configuration parameters following ***
  0,				// *** pad alignment ***
};

static SEQ_IpuCfg_t LUT4K1 = {
  {
    SEQ_NODE_SCALAR,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 3*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    3,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },
  (0x00000000),			// *** code entry address ***

  0,					// *** current processing steps ***
  OUT_LINES2_1,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 5392 /* LUT4K1Buffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        RGB2YUV_PIX,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
  },	// output buffer structs

  {
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 5176 /* RGB2Y1Buffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        RGB2YUV_PIX,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
  },	// input buffer structs
  (SEQ_CfgPara_CPtr_t) 2172 /*LUT4K1Para */,	// *** pointer to configuration parameters ***
  5,					// *** number of configuration parameters following ***
  0,				// *** pad alignment ***
};

static SEQ_IpuCfg_t COMPRESS1 = {
  {
    SEQ_NODE_VECTOR,			// *** type of engine ***
    D_IPUV_SYN_BASEADDRESS + 3*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    3,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },
  (0x00000000),			// *** code entry address ***

  0,					// *** current processing steps ***
  OUT_LINES2_1,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 4528 /* COMPRESS1Buffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUV_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        OUT_PIXELS_RGB,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
    },
  },	// output buffer structs

  {
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 4312 /* Debayer1Buffer1 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
      1,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3880 /* Debayer1Buffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
      1,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 4096 /* Debayer1Buffer2 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE2_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        DEBAYER_OUTPIX,					// *** number of pixels to transfer ***
      },
      1,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 5392 /* LUT4K1Buffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUV_SYN_S_LINE3_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        RGB2YUV_PIX,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 0,				// *** pointer to input buffer ***
      {
        0,					// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        0,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
  },	// input buffer structs
  (SEQ_CfgPara_CPtr_t) 2464 /*COMPRESS1Para */,	// *** pointer to configuration parameters ***
  8,					// *** number of configuration parameters following ***
  0,				// *** pad alignment ***
};

static SEQ_FdmaCfg_t FastDMA = {
  {
    SEQ_NODE_OTHER,			// *** type of engine ***
    (Addr_t)(uintptr_t)&FDMA,	// *** address of hardware device ***
    SEQ_OTHRIX_FDMA,			// *** index of requested engine ***
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  {
    // *** output buffer #0 ***
    {
      (SEQ_Buf_Ptr_t) 5608 /* ISP_OUTPUT0Buffer */,	// *** offset to destination buffer ***
    },

    // *** output buffer #1 ***
    {
      (SEQ_Buf_Ptr_t) 5824 /* ISP_OUTPUT1Buffer */,	// *** offset to destination buffer ***
    },

    // *** output buffer #2 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #3 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #4 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #5 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #6 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #7 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #8 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #9 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #10 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #11 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #12 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #13 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #14 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

    // *** output buffer #15 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
    },

  },

  // ### inputs ###############################################################

  {
    {
      // *** input buffer #0 ***
      (SEQ_Buf_Ptr_t) 3016 /* COMPRESS0Buffer0 */,	// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #1 ***
      (SEQ_Buf_Ptr_t) 4528 /* COMPRESS1Buffer0 */,	// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #2 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #3 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #4 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #5 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #6 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #7 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #8 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #9 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #10 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #11 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #12 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #13 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #14 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #15 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

  },
}; // FastDMA

static SEQ_Load_t sCodedebayer_even_Debayer0 = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 1*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    1,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  debayer_even_start,		// *** Kernel address in KMEM *** 
  debayer_even_end - debayer_even_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 76 /* Debayer0.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodergb2y_RGB2Y0 = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUV_SYN_BASEADDRESS,		// *** address of hardware device ***
    0,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  rgb2y_start,		// *** Kernel address in KMEM *** 
  rgb2y_end - rgb2y_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 400 /* RGB2Y0.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodedebayer_odd_Debayer1 = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 4*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    4,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  debayer_odd_start,		// *** Kernel address in KMEM *** 
  debayer_odd_end - debayer_odd_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 708 /* Debayer1.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodergb2y_RGB2Y1 = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUV_SYN_BASEADDRESS + 2*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    2,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  rgb2y_start,		// *** Kernel address in KMEM *** 
  rgb2y_end - rgb2y_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 1032 /* RGB2Y1.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodelut4k_LUT4K0 = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 2*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    2,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  lut4k_start,		// *** Kernel address in KMEM *** 
  lut4k_end - lut4k_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 1340 /* LUT4K0.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodergb_compress_COMPRESS0 = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUV_SYN_BASEADDRESS + 1*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    1,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  rgb_compress_start,		// *** Kernel address in KMEM *** 
  rgb_compress_end - rgb_compress_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 1632 /* COMPRESS0.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodelut4k_LUT4K1 = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 3*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    3,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  lut4k_start,		// *** Kernel address in KMEM *** 
  lut4k_end - lut4k_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 1948 /* LUT4K1.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodergb_compress_COMPRESS1 = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUV_SYN_BASEADDRESS + 3*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    3,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  rgb_compress_start,		// *** Kernel address in KMEM *** 
  rgb_compress_end - rgb_compress_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 2240 /* COMPRESS1.mHead */,			// *** pointer to IPU node to start ***
};


/*****************************************************************************
* exported data
*****************************************************************************/

// *** graph ***

SEQ_Head_Ptr_t gpGraph[] = {
  (SEQ_Head_Ptr_t) 2592, // FastDMA
  (SEQ_Head_Ptr_t) 2556, // OV_QUAD
  (SEQ_Head_Ptr_t) 76, // Debayer0
  (SEQ_Head_Ptr_t) 400, // RGB2Y0
  (SEQ_Head_Ptr_t) 708, // Debayer1
  (SEQ_Head_Ptr_t) 1032, // RGB2Y1
  (SEQ_Head_Ptr_t) 1340, // LUT4K0
  (SEQ_Head_Ptr_t) 1632, // COMPRESS0
  (SEQ_Head_Ptr_t) 1948, // LUT4K1
  (SEQ_Head_Ptr_t) 2240, // COMPRESS1
  (SEQ_Head_Ptr_t) 372, // sCodedebayer_even_Debayer0
  (SEQ_Head_Ptr_t) 680, // sCodergb2y_RGB2Y0
  (SEQ_Head_Ptr_t) 1004, // sCodedebayer_odd_Debayer1
  (SEQ_Head_Ptr_t) 1312, // sCodergb2y_RGB2Y1
  (SEQ_Head_Ptr_t) 1604, // sCodelut4k_LUT4K0
  (SEQ_Head_Ptr_t) 1920, // sCodergb_compress_COMPRESS0
  (SEQ_Head_Ptr_t) 2212, // sCodelut4k_LUT4K1
  (SEQ_Head_Ptr_t) 2528, // sCodergb_compress_COMPRESS1
  (SEQ_Head_Ptr_t) 0		// ** END ***
};

// *** Graph parsing metadata ***

static Object_t sObjectList[42] = {
  { &gpGraph,	76,	0 },
  { &Debayer0,	224,	76 },
  { &Debayer0Para,	72,	300 },
  { &sCodedebayer_even_Debayer0,	28,	372 },
  { &RGB2Y0,	224,	400 },
  { &RGB2Y0Para,	56,	624 },
  { &sCodergb2y_RGB2Y0,	28,	680 },
  { &Debayer1,	224,	708 },
  { &Debayer1Para,	72,	932 },
  { &sCodedebayer_odd_Debayer1,	28,	1004 },
  { &RGB2Y1,	224,	1032 },
  { &RGB2Y1Para,	56,	1256 },
  { &sCodergb2y_RGB2Y1,	28,	1312 },
  { &LUT4K0,	224,	1340 },
  { &LUT4K0Para,	40,	1564 },
  { &sCodelut4k_LUT4K0,	28,	1604 },
  { &COMPRESS0,	224,	1632 },
  { &COMPRESS0Para,	64,	1856 },
  { &sCodergb_compress_COMPRESS0,	28,	1920 },
  { &LUT4K1,	224,	1948 },
  { &LUT4K1Para,	40,	2172 },
  { &sCodelut4k_LUT4K1,	28,	2212 },
  { &COMPRESS1,	224,	2240 },
  { &COMPRESS1Para,	64,	2464 },
  { &sCodergb_compress_COMPRESS1,	28,	2528 },
  { &OV_QUAD,	36,	2556 },
  { &FastDMA,	208,	2592 },
  { &OV_QUADBuffer0,	216,	2800 },
  { &COMPRESS0Buffer0,	216,	3016 },
  { &Debayer0Buffer0,	216,	3232 },
  { &Debayer0Buffer2,	216,	3448 },
  { &Debayer0Buffer1,	216,	3664 },
  { &Debayer1Buffer0,	216,	3880 },
  { &Debayer1Buffer2,	216,	4096 },
  { &Debayer1Buffer1,	216,	4312 },
  { &COMPRESS1Buffer0,	216,	4528 },
  { &RGB2Y0Buffer0,	216,	4744 },
  { &LUT4K0Buffer0,	216,	4960 },
  { &RGB2Y1Buffer0,	216,	5176 },
  { &LUT4K1Buffer0,	216,	5392 },
  { &ISP_OUTPUT0Buffer,	216,	5608 },
  { &ISP_OUTPUT1Buffer,	216,	5824 },
};

static uint32_t sPointerList[] = {
  0, /* gpGraph: pointer to FastDMA */
  4, /* gpGraph: pointer to OV_QUAD */
  8, /* gpGraph: pointer to Debayer0 */
  12, /* gpGraph: pointer to RGB2Y0 */
  16, /* gpGraph: pointer to Debayer1 */
  20, /* gpGraph: pointer to RGB2Y1 */
  24, /* gpGraph: pointer to LUT4K0 */
  28, /* gpGraph: pointer to COMPRESS0 */
  32, /* gpGraph: pointer to LUT4K1 */
  36, /* gpGraph: pointer to COMPRESS1 */
  40, /* gpGraph: pointer to sCodedebayer_even_Debayer0 */
  44, /* gpGraph: pointer to sCodergb2y_RGB2Y0 */
  48, /* gpGraph: pointer to sCodedebayer_odd_Debayer1 */
  52, /* gpGraph: pointer to sCodergb2y_RGB2Y1 */
  56, /* gpGraph: pointer to sCodelut4k_LUT4K0 */
  60, /* gpGraph: pointer to sCodergb_compress_COMPRESS0 */
  64, /* gpGraph: pointer to sCodelut4k_LUT4K1 */
  68, /* gpGraph: pointer to sCodergb_compress_COMPRESS1 */
  100, /* Debayer0: pointer to OUTput buffer */
  112, /* Debayer0: pointer to OUTput buffer */
  124, /* Debayer0: pointer to OUTput buffer */
  148, /* Debayer0: pointer to INput buffer */
  164, /* Debayer0: pointer to INput buffer */
  180, /* Debayer0: pointer to INput buffer */
  292, /* Debayer0: pointer to config parameters */
  396, /* sCodedebayer_even_Debayer0: pointer to Debayer0.mHead */
  424, /* RGB2Y0: pointer to OUTput buffer */
  472, /* RGB2Y0: pointer to INput buffer */
  488, /* RGB2Y0: pointer to INput buffer */
  504, /* RGB2Y0: pointer to INput buffer */
  616, /* RGB2Y0: pointer to config parameters */
  704, /* sCodergb2y_RGB2Y0: pointer to RGB2Y0.mHead */
  732, /* Debayer1: pointer to OUTput buffer */
  744, /* Debayer1: pointer to OUTput buffer */
  756, /* Debayer1: pointer to OUTput buffer */
  780, /* Debayer1: pointer to INput buffer */
  796, /* Debayer1: pointer to INput buffer */
  812, /* Debayer1: pointer to INput buffer */
  924, /* Debayer1: pointer to config parameters */
  1028, /* sCodedebayer_odd_Debayer1: pointer to Debayer1.mHead */
  1056, /* RGB2Y1: pointer to OUTput buffer */
  1104, /* RGB2Y1: pointer to INput buffer */
  1120, /* RGB2Y1: pointer to INput buffer */
  1136, /* RGB2Y1: pointer to INput buffer */
  1248, /* RGB2Y1: pointer to config parameters */
  1336, /* sCodergb2y_RGB2Y1: pointer to RGB2Y1.mHead */
  1364, /* LUT4K0: pointer to OUTput buffer */
  1412, /* LUT4K0: pointer to INput buffer */
  1556, /* LUT4K0: pointer to config parameters */
  1628, /* sCodelut4k_LUT4K0: pointer to LUT4K0.mHead */
  1656, /* COMPRESS0: pointer to OUTput buffer */
  1704, /* COMPRESS0: pointer to INput buffer */
  1720, /* COMPRESS0: pointer to INput buffer */
  1736, /* COMPRESS0: pointer to INput buffer */
  1752, /* COMPRESS0: pointer to INput buffer */
  1848, /* COMPRESS0: pointer to config parameters */
  1944, /* sCodergb_compress_COMPRESS0: pointer to COMPRESS0.mHead */
  1972, /* LUT4K1: pointer to OUTput buffer */
  2020, /* LUT4K1: pointer to INput buffer */
  2164, /* LUT4K1: pointer to config parameters */
  2236, /* sCodelut4k_LUT4K1: pointer to LUT4K1.mHead */
  2264, /* COMPRESS1: pointer to OUTput buffer */
  2312, /* COMPRESS1: pointer to INput buffer */
  2328, /* COMPRESS1: pointer to INput buffer */
  2344, /* COMPRESS1: pointer to INput buffer */
  2360, /* COMPRESS1: pointer to INput buffer */
  2456, /* COMPRESS1: pointer to config parameters */
  2552, /* sCodergb_compress_COMPRESS1: pointer to COMPRESS1.mHead */
  2572, /* OV_QUAD: pointer to OUTput buffer */
  2812, /* OV_QUADBuffer0: pointer to producer */
  2824, /* OV_QUADBuffer0: pointer to consumer */
  2848, /* OV_QUADBuffer0: pointer to consumer */
  2872, /* OV_QUADBuffer0: pointer to consumer */
  2896, /* OV_QUADBuffer0: pointer to consumer */
  2920, /* OV_QUADBuffer0: pointer to consumer */
  2944, /* OV_QUADBuffer0: pointer to consumer */
  3028, /* COMPRESS0Buffer0: pointer to producer */
  3040, /* COMPRESS0Buffer0: pointer to consumer */
  3244, /* Debayer0Buffer0: pointer to producer */
  3256, /* Debayer0Buffer0: pointer to consumer */
  3280, /* Debayer0Buffer0: pointer to consumer */
  3460, /* Debayer0Buffer2: pointer to producer */
  3472, /* Debayer0Buffer2: pointer to consumer */
  3496, /* Debayer0Buffer2: pointer to consumer */
  3676, /* Debayer0Buffer1: pointer to producer */
  3688, /* Debayer0Buffer1: pointer to consumer */
  3712, /* Debayer0Buffer1: pointer to consumer */
  3892, /* Debayer1Buffer0: pointer to producer */
  3904, /* Debayer1Buffer0: pointer to consumer */
  3928, /* Debayer1Buffer0: pointer to consumer */
  4108, /* Debayer1Buffer2: pointer to producer */
  4120, /* Debayer1Buffer2: pointer to consumer */
  4144, /* Debayer1Buffer2: pointer to consumer */
  4324, /* Debayer1Buffer1: pointer to producer */
  4336, /* Debayer1Buffer1: pointer to consumer */
  4360, /* Debayer1Buffer1: pointer to consumer */
  4540, /* COMPRESS1Buffer0: pointer to producer */
  4552, /* COMPRESS1Buffer0: pointer to consumer */
  4756, /* RGB2Y0Buffer0: pointer to producer */
  4768, /* RGB2Y0Buffer0: pointer to consumer */
  4972, /* LUT4K0Buffer0: pointer to producer */
  4984, /* LUT4K0Buffer0: pointer to consumer */
  5188, /* RGB2Y1Buffer0: pointer to producer */
  5200, /* RGB2Y1Buffer0: pointer to consumer */
  5404, /* LUT4K1Buffer0: pointer to producer */
  5416, /* LUT4K1Buffer0: pointer to consumer */
  5620, /* ISP_OUTPUT0Buffer: pointer to producer */
  5836, /* ISP_OUTPUT1Buffer: pointer to producer */
};

static uint32_t sBufferList[] = {
  2800, /* OV_QUADBuffer0 */
  3016, /* COMPRESS0Buffer0 */
  3232, /* Debayer0Buffer0 */
  3448, /* Debayer0Buffer2 */
  3664, /* Debayer0Buffer1 */
  3880, /* Debayer1Buffer0 */
  4096, /* Debayer1Buffer2 */
  4312, /* Debayer1Buffer1 */
  4528, /* COMPRESS1Buffer0 */
  4744, /* RGB2Y0Buffer0 */
  4960, /* LUT4K0Buffer0 */
  5176, /* RGB2Y1Buffer0 */
  5392, /* LUT4K1Buffer0 */
  5608, /* ISP_OUTPUT0Buffer */
  5824, /* ISP_OUTPUT1Buffer */
};

static const char* scppFdmaChannelNames[] = {
  "ISP_OUTPUT0",
  "ISP_OUTPUT1",
};

static const char* scppBufferNames[] = {
  "OV_QUADBuffer0",
  "COMPRESS0Buffer0",
  "Debayer0Buffer0",
  "Debayer0Buffer2",
  "Debayer0Buffer1",
  "Debayer1Buffer0",
  "Debayer1Buffer2",
  "Debayer1Buffer1",
  "COMPRESS1Buffer0",
  "RGB2Y0Buffer0",
  "LUT4K0Buffer0",
  "RGB2Y1Buffer0",
  "LUT4K1Buffer0",
  "ISP_OUTPUT0Buffer",
  "ISP_OUTPUT1Buffer",
};

GraphMetadata_t gGraphMetadata = {
  8,		// *** max number of buffer consumers
  6040,		// *** graph size [bytes]
  (const Object_t*) &sObjectList,	// *** object list pointer
  42,		// *** object count
  (const uint32_t*) &sPointerList,	// *** pointer list pointer
  108,		// *** pointer count
  (const uint32_t*) &sBufferList,	// *** buffer list pointer
  15,		// *** buffer count
  {
    0
  },
  { // MIPI-CSI metadata
    { // node OV_QUAD
      CSI_MAX_OV10640, 4,
      {
        0, 0, 0, 0,
      },
      {
        5152 * 2, 0, 0, 0,
      }
    },
    { // node not used
      CSI_NONE, 0, {0}, {0}
    },
  },
  { // VIU Lite metadata
    { // node not used
      VIU_NONE, 0, 0
    },
    { // node not used
      VIU_NONE, 0, 0
    },
  },
  scppFdmaChannelNames,
  scppBufferNames
};

#ifdef  __cplusplus
}
#endif

/*EOF*/
