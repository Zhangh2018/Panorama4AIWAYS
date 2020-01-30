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

#include "imx224_exp_ctrl_c.h"

#include "iofiles.h"
#ifndef S32DS_BUILD
#include "kmem.c"
#endif
#include "kmem_sym.h"
// *** forwards needed for cross-linking ***

static SEQ_FdmaCfg_t FastDMA;
static SEQ_MipiCfg_t CAM0;
static SEQ_IpuCfg_t ChGain_Exposure;
static SEQ_IpuCfg_t Debayer_G;
static SEQ_IpuCfg_t Debayer_BR;
static SEQ_IpuCfg_t rgb888_planar2interleaved;
static SEQ_IpuCfg_t G_SubSamp2;
static SEQ_Load_t sCodeexposure_tonehdr_ChGain_Exposure;
static SEQ_Load_t sCodedebayer_g_Debayer_G;
static SEQ_Load_t sCodedebayer_br_advanced_Debayer_BR;
static SEQ_Load_t sCodeplanar444_to_interleaved_rgb888_planar2interleaved;
static SEQ_Load_t sCodescale2by1_11_G_SubSamp2;

/*** Buffers ****************************************************************/

SEQ_Buf_t ChGain_ExposureBuffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  32,				// *** number of lines fitting into output buffer (wrap) ***
  1288 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2576),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 52 /*ChGain_Exposure*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 352 /* Debayer_G.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      1288 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 352 /* Debayer_G.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      2,				// *** number of lines required for processing ***
      1288 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 352 /* Debayer_G.mHead */,	// *** offset to consumers (node) configuration header ***
      2,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      3,				// *** number of lines required for processing ***
      1288 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
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
}; // ChGain_ExposureBuffer0

SEQ_Buf_t Debayer_GBuffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  64,				// *** number of lines fitting into output buffer (wrap) ***
  1286 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2576),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 352 /*Debayer_G*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 668 /* Debayer_BR.mHead */,	// *** offset to consumers (node) configuration header ***
      3,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      1286 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 668 /* Debayer_BR.mHead */,	// *** offset to consumers (node) configuration header ***
      4,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      2,				// *** number of lines required for processing ***
      1286 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 668 /* Debayer_BR.mHead */,	// *** offset to consumers (node) configuration header ***
      5,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      3,				// *** number of lines required for processing ***
      1286 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
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
}; // Debayer_GBuffer0

SEQ_Buf_t Debayer_GBuffer1 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  64,				// *** number of lines fitting into output buffer (wrap) ***
  643 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(1288),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 352 /*Debayer_G*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 668 /* Debayer_BR.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      643 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 668 /* Debayer_BR.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      2,				// *** number of lines required for processing ***
      643 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 668 /* Debayer_BR.mHead */,	// *** offset to consumers (node) configuration header ***
      2,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      3,				// *** number of lines required for processing ***
      643 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
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
}; // Debayer_GBuffer1

SEQ_Buf_t Debayer_BRBuffer2 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  32,				// *** number of lines fitting into output buffer (wrap) ***
  1284 * 1,				// *** number of bytes per line to read ***
  BUF_ALIGN(1288),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 668 /*Debayer_BR*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1024 /* rgb888_planar2interleaved.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      1284 * 1,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 1332 /* G_SubSamp2.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      2,				// *** number of lines required for processing ***
      1284 * 1,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      2,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 1332 /* G_SubSamp2.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      2,				// *** number of lines required for processing ***
      1284 * 1,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      2,				// *** increment of current relative index ***
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
}; // Debayer_BRBuffer2

SEQ_Buf_t Debayer_BRBuffer1 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  32,				// *** number of lines fitting into output buffer (wrap) ***
  1284 * 1,				// *** number of bytes per line to read ***
  BUF_ALIGN(1288),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 668 /*Debayer_BR*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1024 /* rgb888_planar2interleaved.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      1284 * 1,				// *** number of bytes per line to read ***
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
}; // Debayer_BRBuffer1

SEQ_Buf_t Debayer_BRBuffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  32,				// *** number of lines fitting into output buffer (wrap) ***
  1284 * 1,				// *** number of bytes per line to read ***
  BUF_ALIGN(1288),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 668 /*Debayer_BR*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1024 /* rgb888_planar2interleaved.mHead */,	// *** offset to consumers (node) configuration header ***
      2,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      1284 * 1,				// *** number of bytes per line to read ***
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
}; // Debayer_BRBuffer0

SEQ_Buf_t CAM0Buffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  256,				// *** number of lines fitting into output buffer (wrap) ***
  1288 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2576),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1632 /*CAM0*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 52 /* ChGain_Exposure.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      1288 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 52 /* ChGain_Exposure.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      1288 * 2,				// *** number of bytes per line to read ***
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
}; // CAM0Buffer0

SEQ_Buf_t rgb888_planar2interleavedBuffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  16,				// *** number of lines fitting into output buffer (wrap) ***
  3840 * 1,				// *** number of bytes per line to read ***
  BUF_ALIGN(3840),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1024 /*rgb888_planar2interleaved*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1668 /* FastDMA.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      3840 * 1,				// *** number of bytes per line to read ***
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
}; // rgb888_planar2interleavedBuffer0

SEQ_Buf_t G_SubSamp2Buffer0 = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  32,				// *** number of lines fitting into output buffer (wrap) ***
  NUM_OUT_Y_SCALED_BYTES * 1,				// *** number of bytes per line to read ***
  BUF_ALIGN(NUM_OUT_Y_SCALED_BYTES),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1332 /*G_SubSamp2*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1668 /* FastDMA.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      NUM_OUT_Y_SCALED_BYTES * 1,				// *** number of bytes per line to read ***
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
}; // G_SubSamp2Buffer0

SEQ_Buf_t RGB888Buffer = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 1,		// *** pointer to output buffer ***
  720,				// *** number of lines fitting into output buffer (wrap) ***
  3840 * 1,				// *** number of bytes per line to read ***
  3840,				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1668 /*FastDMA */,		// *** offset to producers (node) configuration header ***
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
      3840 * 1,				// *** number of bytes per line to read ***
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
}; // RGB888Buffer

SEQ_Buf_t G_SUB_2Buffer = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 1,		// *** pointer to output buffer ***
  360,				// *** number of lines fitting into output buffer (wrap) ***
  NUM_OUT_Y_SCALED_BYTES * 1,				// *** number of bytes per line to read ***
  NUM_OUT_Y_SCALED_BYTES,				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1668 /*FastDMA */,		// *** offset to producers (node) configuration header ***
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
      NUM_OUT_Y_SCALED_BYTES * 1,				// *** number of bytes per line to read ***
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
}; // G_SUB_2Buffer


static SEQ_CfgPara_t ChGain_ExposurePara [] = {
{ D_IPUS_SYN_HOST_XCFG_OFF, (1288 << 16) | 0x1 },
    { D_IPUS_SYN_HOST_INACFG_OFF, 0x00000300 },
    { D_IPUS_SYN_HOST_OUTCFG_OFF, 0x00000100 },
    { D_IPUS_SYN_S_CH0_CFG_INA_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH1_CFG_INA_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH0_CFG_OUT_OFF, 0x00000000 },
};

static SEQ_CfgPara_t Debayer_GPara [] = {
{ D_IPUS_SYN_HOST_XCFG_OFF, (1286 << 16) | 0x1},
    { D_IPUS_SYN_HOST_INACFG_OFF, 0x00000700 },
    { D_IPUS_SYN_HOST_OUTCFG_OFF, 0x00000300 },
    { D_IPUS_SYN_S_CH0_CFG_INA_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH1_CFG_INA_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH2_CFG_INA_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH0_CFG_OUT_OFF, 0x00000000 },
    { D_IPUS_SYN_S_CH1_CFG_OUT_OFF, 0x00010000 },
};

static SEQ_CfgPara_t Debayer_BRPara [] = {
{ D_IPUS_SYN_HOST_XCFG_OFF, (1284 << 16) | 0x1},
    { D_IPUS_SYN_HOST_INACFG_OFF, 0x00000700 },
    { D_IPUS_SYN_HOST_INBCFG_OFF, 0x00000700 },
    { D_IPUS_SYN_HOST_OUTCFG_OFF, 0x00000700 },
    { D_IPUS_SYN_S_CH0_CFG_INA_OFF, 0x00098000 },
    { D_IPUS_SYN_S_CH1_CFG_INA_OFF, 0x00098000 },
    { D_IPUS_SYN_S_CH2_CFG_INA_OFF, 0x00098000 },
    { D_IPUS_SYN_S_CH0_CFG_INB_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH1_CFG_INB_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH2_CFG_INB_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH0_CFG_OUT_OFF, 0x10000000 },
    { D_IPUS_SYN_S_CH1_CFG_OUT_OFF, 0x10000000 },
    { D_IPUS_SYN_S_CH2_CFG_OUT_OFF, 0x10000000 },
};

static SEQ_CfgPara_t rgb888_planar2interleavedPara [] = {
{ D_IPUS_SYN_HOST_XCFG_OFF, (1280 << 16) | 0x1 },
    { D_IPUS_SYN_HOST_INACFG_OFF, 0x00000700 },
    { D_IPUS_SYN_HOST_OUTCFG_OFF, 0x00000100 },
    { D_IPUS_SYN_S_CH0_CFG_INA_OFF, 0x10054000 },
    { D_IPUS_SYN_S_CH1_CFG_INA_OFF, 0x10054000 },
    { D_IPUS_SYN_S_CH2_CFG_INA_OFF, 0x10054000 },
    { D_IPUS_SYN_S_CH0_CFG_OUT_OFF, 0x10000000 },
};

static SEQ_CfgPara_t G_SubSamp2Para [] = {
{ D_IPUS_SYN_HOST_XCFG_OFF, (640 << 16) | 0x1 },
    { D_IPUS_SYN_HOST_INACFG_OFF, 0x00000300 },
    { D_IPUS_SYN_HOST_OUTCFG_OFF, 0x00000100 },
    { D_IPUS_SYN_S_CH0_CFG_INA_OFF, 0x10054000 },
    { D_IPUS_SYN_S_CH1_CFG_INA_OFF, 0x10054000 },
    { D_IPUS_SYN_S_CH0_CFG_OUT_OFF, 0x10000000 },
};

static SEQ_MipiCfg_t CAM0 = {
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
      (SEQ_Buf_Ptr_t) 3172 /*CAM0Buffer0*/,			// *** pointer to output buffer ***
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
  },
  NUM_CAM_LINES,			// *** number of input image lines ***
  0			// *** pad alignment ***
}; // CAM0

static SEQ_IpuCfg_t ChGain_Exposure = {
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
  NUM_IN_LINES,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 1876 /* ChGain_ExposureBuffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1288,					// *** number of pixels to transfer ***
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
      (SEQ_Buf_Ptr_t) 3172 /* CAM0Buffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1288,					// *** number of pixels to transfer ***
      },
      1,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3172 /* CAM0Buffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1288,					// *** number of pixels to transfer ***
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
  (SEQ_CfgPara_CPtr_t) 276 /*ChGain_ExposurePara */,	// *** pointer to configuration parameters ***
  6,					// *** number of configuration parameters following ***
  0,				// *** pad alignment ***
};

static SEQ_IpuCfg_t Debayer_G = {
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
  NUM_IN_M4,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2092 /* Debayer_GBuffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1286,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2308 /* Debayer_GBuffer1 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE1_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        643,					// *** number of pixels to transfer ***
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
      (SEQ_Buf_Ptr_t) 1876 /* ChGain_ExposureBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1288,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 1876 /* ChGain_ExposureBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
        1,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1288,					// *** number of pixels to transfer ***
      },
      1,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 1876 /* ChGain_ExposureBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE2_LEN_INA_OFF,				// *** offset to SDMA's register ***
        2,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1288,					// *** number of pixels to transfer ***
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
  (SEQ_CfgPara_CPtr_t) 576 /*Debayer_GPara */,	// *** pointer to configuration parameters ***
  8,					// *** number of configuration parameters following ***
  0,				// *** pad alignment ***
};

static SEQ_IpuCfg_t Debayer_BR = {
  {
    SEQ_NODE_SCALAR,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 5*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    5,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },
  (0x00000000),			// *** code entry address ***

  0,					// *** current processing steps ***
  NUM_IN_M8,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2956 /* Debayer_BRBuffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1284,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2740 /* Debayer_BRBuffer1 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE1_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1284,					// *** number of pixels to transfer ***
      },
    },
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2524 /* Debayer_BRBuffer2 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE2_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1284,					// *** number of pixels to transfer ***
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
      (SEQ_Buf_Ptr_t) 2308 /* Debayer_GBuffer1 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        643,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2308 /* Debayer_GBuffer1 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
        1,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        643,					// *** number of pixels to transfer ***
      },
      1,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2308 /* Debayer_GBuffer1 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE2_LEN_INA_OFF,				// *** offset to SDMA's register ***
        2,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        643,					// *** number of pixels to transfer ***
      },
      2,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2092 /* Debayer_GBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_INB_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1286,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2092 /* Debayer_GBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE1_LEN_INB_OFF,				// *** offset to SDMA's register ***
        1,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1286,					// *** number of pixels to transfer ***
      },
      1,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2092 /* Debayer_GBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE2_LEN_INB_OFF,				// *** offset to SDMA's register ***
        2,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1286,					// *** number of pixels to transfer ***
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
  },	// input buffer structs
  (SEQ_CfgPara_CPtr_t) 892 /*Debayer_BRPara */,	// *** pointer to configuration parameters ***
  13,					// *** number of configuration parameters following ***
  0,				// *** pad alignment ***
};

static SEQ_IpuCfg_t rgb888_planar2interleaved = {
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
  720,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3388 /* rgb888_planar2interleavedBuffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        3840,					// *** number of pixels to transfer ***
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
      (SEQ_Buf_Ptr_t) 2740 /* Debayer_BRBuffer1 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1280,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2524 /* Debayer_BRBuffer2 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1280,					// *** number of pixels to transfer ***
      },
      0,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2956 /* Debayer_BRBuffer0 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE2_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        1280,					// *** number of pixels to transfer ***
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
  (SEQ_CfgPara_CPtr_t) 1248 /*rgb888_planar2interleavedPara */,	// *** pointer to configuration parameters ***
  7,					// *** number of configuration parameters following ***
  0,				// *** pad alignment ***
};

static SEQ_IpuCfg_t G_SubSamp2 = {
  {
    SEQ_NODE_SCALAR,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS,		// *** address of hardware device ***
    0,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },
  (0x00000000),			// *** code entry address ***

  0,					// *** current processing steps ***
  360,					// *** number of processing steps per frame ***

  {
    // *** output buffer # ***
    {
      (SEQ_Buf_Ptr_t) 3604 /* G_SubSamp2Buffer0 */,			// *** pointer to output buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        NUM_OUT_Y_SCALED_BYTES,					// *** number of pixels to transfer ***
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
      (SEQ_Buf_Ptr_t) 2524 /* Debayer_BRBuffer2 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
        0,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        NUM_OUT_PIXELS,					// *** number of pixels to transfer ***
      },
      1,				// *** our index in buffers consumer array ***
      0,				// *** pad alignment ***
    },
    // *** input buffer # ***
    {
      (SEQ_Buf_Ptr_t) 2524 /* Debayer_BRBuffer2 */,			// *** pointer to input buffer ***
      {
        D_IPUS_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
        1,					// *** number of first line to be used ***
        0,					// *** number of bytes to first pixel in line ***
        NUM_OUT_PIXELS,					// *** number of pixels to transfer ***
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
  (SEQ_CfgPara_CPtr_t) 1556 /*G_SubSamp2Para */,	// *** pointer to configuration parameters ***
  6,					// *** number of configuration parameters following ***
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
      (SEQ_Buf_Ptr_t) 3820 /* RGB888Buffer */,	// *** offset to destination buffer ***
    },

    // *** output buffer #1 ***
    {
      (SEQ_Buf_Ptr_t) 4036 /* G_SUB_2Buffer */,	// *** offset to destination buffer ***
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
      (SEQ_Buf_Ptr_t) 3388 /* rgb888_planar2interleavedBuffer0 */,	// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #1 ***
      (SEQ_Buf_Ptr_t) 3604 /* G_SubSamp2Buffer0 */,	// *** input buffer ***
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

static SEQ_Load_t sCodeexposure_tonehdr_ChGain_Exposure = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 1*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    1,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  exposure_tonehdr_start,		// *** Kernel address in KMEM *** 
  exposure_tonehdr_end - exposure_tonehdr_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 52 /* ChGain_Exposure.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodedebayer_g_Debayer_G = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 2*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    2,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  debayer_g_start,		// *** Kernel address in KMEM *** 
  debayer_g_end - debayer_g_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 352 /* Debayer_G.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodedebayer_br_advanced_Debayer_BR = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 5*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    5,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  debayer_br_advanced_start,		// *** Kernel address in KMEM *** 
  debayer_br_advanced_end - debayer_br_advanced_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 668 /* Debayer_BR.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodeplanar444_to_interleaved_rgb888_planar2interleaved = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 3*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    3,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  planar444_to_interleaved_start,		// *** Kernel address in KMEM *** 
  planar444_to_interleaved_end - planar444_to_interleaved_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 1024 /* rgb888_planar2interleaved.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodescale2by1_11_G_SubSamp2 = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS,		// *** address of hardware device ***
    0,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  scale2by1_11_start,		// *** Kernel address in KMEM *** 
  scale2by1_11_end - scale2by1_11_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 1332 /* G_SubSamp2.mHead */,			// *** pointer to IPU node to start ***
};


/*****************************************************************************
* exported data
*****************************************************************************/

// *** graph ***

SEQ_Head_Ptr_t gpGraph[] = {
  (SEQ_Head_Ptr_t) 1668, // FastDMA
  (SEQ_Head_Ptr_t) 1632, // CAM0
  (SEQ_Head_Ptr_t) 52, // ChGain_Exposure
  (SEQ_Head_Ptr_t) 352, // Debayer_G
  (SEQ_Head_Ptr_t) 668, // Debayer_BR
  (SEQ_Head_Ptr_t) 1024, // rgb888_planar2interleaved
  (SEQ_Head_Ptr_t) 1332, // G_SubSamp2
  (SEQ_Head_Ptr_t) 324, // sCodeexposure_tonehdr_ChGain_Exposure
  (SEQ_Head_Ptr_t) 640, // sCodedebayer_g_Debayer_G
  (SEQ_Head_Ptr_t) 996, // sCodedebayer_br_advanced_Debayer_BR
  (SEQ_Head_Ptr_t) 1304, // sCodeplanar444_to_interleaved_rgb888_planar2interleaved
  (SEQ_Head_Ptr_t) 1604, // sCodescale2by1_11_G_SubSamp2
  (SEQ_Head_Ptr_t) 0		// ** END ***
};

// *** Graph parsing metadata ***

static Object_t sObjectList[29] = {
  { &gpGraph,	52,	0 },
  { &ChGain_Exposure,	224,	52 },
  { &ChGain_ExposurePara,	48,	276 },
  { &sCodeexposure_tonehdr_ChGain_Exposure,	28,	324 },
  { &Debayer_G,	224,	352 },
  { &Debayer_GPara,	64,	576 },
  { &sCodedebayer_g_Debayer_G,	28,	640 },
  { &Debayer_BR,	224,	668 },
  { &Debayer_BRPara,	104,	892 },
  { &sCodedebayer_br_advanced_Debayer_BR,	28,	996 },
  { &rgb888_planar2interleaved,	224,	1024 },
  { &rgb888_planar2interleavedPara,	56,	1248 },
  { &sCodeplanar444_to_interleaved_rgb888_planar2interleaved,	28,	1304 },
  { &G_SubSamp2,	224,	1332 },
  { &G_SubSamp2Para,	48,	1556 },
  { &sCodescale2by1_11_G_SubSamp2,	28,	1604 },
  { &CAM0,	36,	1632 },
  { &FastDMA,	208,	1668 },
  { &ChGain_ExposureBuffer0,	216,	1876 },
  { &Debayer_GBuffer0,	216,	2092 },
  { &Debayer_GBuffer1,	216,	2308 },
  { &Debayer_BRBuffer2,	216,	2524 },
  { &Debayer_BRBuffer1,	216,	2740 },
  { &Debayer_BRBuffer0,	216,	2956 },
  { &CAM0Buffer0,	216,	3172 },
  { &rgb888_planar2interleavedBuffer0,	216,	3388 },
  { &G_SubSamp2Buffer0,	216,	3604 },
  { &RGB888Buffer,	216,	3820 },
  { &G_SUB_2Buffer,	216,	4036 },
};

static uint32_t sPointerList[] = {
  0, /* gpGraph: pointer to FastDMA */
  4, /* gpGraph: pointer to CAM0 */
  8, /* gpGraph: pointer to ChGain_Exposure */
  12, /* gpGraph: pointer to Debayer_G */
  16, /* gpGraph: pointer to Debayer_BR */
  20, /* gpGraph: pointer to rgb888_planar2interleaved */
  24, /* gpGraph: pointer to G_SubSamp2 */
  28, /* gpGraph: pointer to sCodeexposure_tonehdr_ChGain_Exposure */
  32, /* gpGraph: pointer to sCodedebayer_g_Debayer_G */
  36, /* gpGraph: pointer to sCodedebayer_br_advanced_Debayer_BR */
  40, /* gpGraph: pointer to sCodeplanar444_to_interleaved_rgb888_planar2interleaved */
  44, /* gpGraph: pointer to sCodescale2by1_11_G_SubSamp2 */
  76, /* ChGain_Exposure: pointer to OUTput buffer */
  124, /* ChGain_Exposure: pointer to INput buffer */
  140, /* ChGain_Exposure: pointer to INput buffer */
  268, /* ChGain_Exposure: pointer to config parameters */
  348, /* sCodeexposure_tonehdr_ChGain_Exposure: pointer to ChGain_Exposure.mHead */
  376, /* Debayer_G: pointer to OUTput buffer */
  388, /* Debayer_G: pointer to OUTput buffer */
  424, /* Debayer_G: pointer to INput buffer */
  440, /* Debayer_G: pointer to INput buffer */
  456, /* Debayer_G: pointer to INput buffer */
  568, /* Debayer_G: pointer to config parameters */
  664, /* sCodedebayer_g_Debayer_G: pointer to Debayer_G.mHead */
  692, /* Debayer_BR: pointer to OUTput buffer */
  704, /* Debayer_BR: pointer to OUTput buffer */
  716, /* Debayer_BR: pointer to OUTput buffer */
  740, /* Debayer_BR: pointer to INput buffer */
  756, /* Debayer_BR: pointer to INput buffer */
  772, /* Debayer_BR: pointer to INput buffer */
  788, /* Debayer_BR: pointer to INput buffer */
  804, /* Debayer_BR: pointer to INput buffer */
  820, /* Debayer_BR: pointer to INput buffer */
  884, /* Debayer_BR: pointer to config parameters */
  1020, /* sCodedebayer_br_advanced_Debayer_BR: pointer to Debayer_BR.mHead */
  1048, /* rgb888_planar2interleaved: pointer to OUTput buffer */
  1096, /* rgb888_planar2interleaved: pointer to INput buffer */
  1112, /* rgb888_planar2interleaved: pointer to INput buffer */
  1128, /* rgb888_planar2interleaved: pointer to INput buffer */
  1240, /* rgb888_planar2interleaved: pointer to config parameters */
  1328, /* sCodeplanar444_to_interleaved_rgb888_planar2interleaved: pointer to rgb888_planar2interleaved.mHead */
  1356, /* G_SubSamp2: pointer to OUTput buffer */
  1404, /* G_SubSamp2: pointer to INput buffer */
  1420, /* G_SubSamp2: pointer to INput buffer */
  1548, /* G_SubSamp2: pointer to config parameters */
  1628, /* sCodescale2by1_11_G_SubSamp2: pointer to G_SubSamp2.mHead */
  1648, /* CAM0: pointer to OUTput buffer */
  1888, /* ChGain_ExposureBuffer0: pointer to producer */
  1900, /* ChGain_ExposureBuffer0: pointer to consumer */
  1924, /* ChGain_ExposureBuffer0: pointer to consumer */
  1948, /* ChGain_ExposureBuffer0: pointer to consumer */
  2104, /* Debayer_GBuffer0: pointer to producer */
  2116, /* Debayer_GBuffer0: pointer to consumer */
  2140, /* Debayer_GBuffer0: pointer to consumer */
  2164, /* Debayer_GBuffer0: pointer to consumer */
  2320, /* Debayer_GBuffer1: pointer to producer */
  2332, /* Debayer_GBuffer1: pointer to consumer */
  2356, /* Debayer_GBuffer1: pointer to consumer */
  2380, /* Debayer_GBuffer1: pointer to consumer */
  2536, /* Debayer_BRBuffer2: pointer to producer */
  2548, /* Debayer_BRBuffer2: pointer to consumer */
  2572, /* Debayer_BRBuffer2: pointer to consumer */
  2596, /* Debayer_BRBuffer2: pointer to consumer */
  2752, /* Debayer_BRBuffer1: pointer to producer */
  2764, /* Debayer_BRBuffer1: pointer to consumer */
  2968, /* Debayer_BRBuffer0: pointer to producer */
  2980, /* Debayer_BRBuffer0: pointer to consumer */
  3184, /* CAM0Buffer0: pointer to producer */
  3196, /* CAM0Buffer0: pointer to consumer */
  3220, /* CAM0Buffer0: pointer to consumer */
  3400, /* rgb888_planar2interleavedBuffer0: pointer to producer */
  3412, /* rgb888_planar2interleavedBuffer0: pointer to consumer */
  3616, /* G_SubSamp2Buffer0: pointer to producer */
  3628, /* G_SubSamp2Buffer0: pointer to consumer */
  3832, /* RGB888Buffer: pointer to producer */
  4048, /* G_SUB_2Buffer: pointer to producer */
};

static uint32_t sBufferList[] = {
  1876, /* ChGain_ExposureBuffer0 */
  2092, /* Debayer_GBuffer0 */
  2308, /* Debayer_GBuffer1 */
  2524, /* Debayer_BRBuffer2 */
  2740, /* Debayer_BRBuffer1 */
  2956, /* Debayer_BRBuffer0 */
  3172, /* CAM0Buffer0 */
  3388, /* rgb888_planar2interleavedBuffer0 */
  3604, /* G_SubSamp2Buffer0 */
  3820, /* RGB888Buffer */
  4036, /* G_SUB_2Buffer */
};

static const char* scppFdmaChannelNames[] = {
  "RGB888",
  "G_SUB_2",
};

static const char* scppBufferNames[] = {
  "ChGain_ExposureBuffer0",
  "Debayer_GBuffer0",
  "Debayer_GBuffer1",
  "Debayer_BRBuffer2",
  "Debayer_BRBuffer1",
  "Debayer_BRBuffer0",
  "CAM0Buffer0",
  "rgb888_planar2interleavedBuffer0",
  "G_SubSamp2Buffer0",
  "RGB888Buffer",
  "G_SUB_2Buffer",
};

GraphMetadata_t gGraphMetadata = {
  8,		// *** max number of buffer consumers
  4252,		// *** graph size [bytes]
  (const Object_t*) &sObjectList,	// *** object list pointer
  29,		// *** object count
  (const uint32_t*) &sPointerList,	// *** pointer list pointer
  76,		// *** pointer count
  (const uint32_t*) &sBufferList,	// *** buffer list pointer
  11,		// *** buffer count
  {
    0
  },
  { // MIPI-CSI metadata
    { // node CAM0
      CSI_SONY224, 1,
      {
        0, 0, 0, 0,
      },
      {
        1288 * 2, 0, 0, 0,
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
