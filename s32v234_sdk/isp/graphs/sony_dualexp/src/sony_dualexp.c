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

#include "sony_dualexp_c.h"

#include "iofiles_sony_dualexp.h"
#ifndef S32DS_BUILD
#include "kmem.c"
#endif
#include "kmem_sym.h"
// *** forwards needed for cross-linking ***

static SEQ_FdmaCfg_t FastDMA;
static SEQ_MipiCfg_t CAM0;
static SEQ_IpuCfg_t ConvertOut;
static SEQ_IpuCfg_t ChGain_Exposure;
static SEQ_IpuCfg_t Debayer_G;
static SEQ_IpuCfg_t Debayer_BR;
static SEQ_IpuCfg_t RGB_2_YUV;
static SEQ_IpuCfg_t Vignetting;
static SEQ_IpuCfg_t DenoiseFilter;
static SEQ_Load_t sCodelut4k_yuv444touyvy_ConvertOut;
static SEQ_Load_t sCodeexposure_tonehdr_ChGain_Exposure;
static SEQ_Load_t sCodedebayer_g_Debayer_G;
static SEQ_Load_t sCodedebayer_br_advanced_Debayer_BR;
static SEQ_Load_t sCodergb2yuv_sint_RGB_2_YUV;
static SEQ_Load_t sCodevignetting_Vignetting;
static SEQ_Load_t sCodedenoise_DenoiseFilter;

/*** Buffers ****************************************************************/

SEQ_Buf_t ConvertOutBuffer0_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  8,				// *** number of lines fitting into output buffer (wrap) ***
  2560 * 1,				// *** number of bytes per line to read ***
  BUF_ALIGN(2560),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 68 /*ConvertOut*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 2332 /* FastDMA.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      2560 * 1,				// *** number of bytes per line to read ***
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
}; // ConvertOutBuffer0_sony_dualexp

SEQ_Buf_t VignettingBuffer0_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  16,				// *** number of lines fitting into output buffer (wrap) ***
  1288 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2576),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1672 /*Vignetting*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 676 /* Debayer_G.mHead */,	// *** offset to consumers (node) configuration header ***
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
      (SEQ_Head_Ptr_t) 676 /* Debayer_G.mHead */,	// *** offset to consumers (node) configuration header ***
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
      (SEQ_Head_Ptr_t) 676 /* Debayer_G.mHead */,	// *** offset to consumers (node) configuration header ***
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
}; // VignettingBuffer0_sony_dualexp

SEQ_Buf_t Debayer_GBuffer0_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  16,				// *** number of lines fitting into output buffer (wrap) ***
  1286 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2576),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 676 /*Debayer_G*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 992 /* Debayer_BR.mHead */,	// *** offset to consumers (node) configuration header ***
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
      (SEQ_Head_Ptr_t) 992 /* Debayer_BR.mHead */,	// *** offset to consumers (node) configuration header ***
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
      (SEQ_Head_Ptr_t) 992 /* Debayer_BR.mHead */,	// *** offset to consumers (node) configuration header ***
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
}; // Debayer_GBuffer0_sony_dualexp

SEQ_Buf_t Debayer_GBuffer1_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  21,				// *** number of lines fitting into output buffer (wrap) ***
  643 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(1288),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 676 /*Debayer_G*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 992 /* Debayer_BR.mHead */,	// *** offset to consumers (node) configuration header ***
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
      (SEQ_Head_Ptr_t) 992 /* Debayer_BR.mHead */,	// *** offset to consumers (node) configuration header ***
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
      (SEQ_Head_Ptr_t) 992 /* Debayer_BR.mHead */,	// *** offset to consumers (node) configuration header ***
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
}; // Debayer_GBuffer1_sony_dualexp

SEQ_Buf_t Debayer_BRBuffer2_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  16,				// *** number of lines fitting into output buffer (wrap) ***
  1284 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2568),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 992 /*Debayer_BR*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1348 /* RGB_2_YUV.mHead */,	// *** offset to consumers (node) configuration header ***
      2,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      1284 * 2,				// *** number of bytes per line to read ***
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
}; // Debayer_BRBuffer2_sony_dualexp

SEQ_Buf_t Debayer_BRBuffer1_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  16,				// *** number of lines fitting into output buffer (wrap) ***
  1284 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2568),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 992 /*Debayer_BR*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1348 /* RGB_2_YUV.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      1284 * 2,				// *** number of bytes per line to read ***
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
}; // Debayer_BRBuffer1_sony_dualexp

SEQ_Buf_t Debayer_BRBuffer0_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  16,				// *** number of lines fitting into output buffer (wrap) ***
  1284 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2568),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 992 /*Debayer_BR*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1348 /* RGB_2_YUV.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      1284 * 2,				// *** number of bytes per line to read ***
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
}; // Debayer_BRBuffer0_sony_dualexp

SEQ_Buf_t RGB_2_YUVBuffer0_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  16,				// *** number of lines fitting into output buffer (wrap) ***
  1284 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2568),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1348 /*RGB_2_YUV*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1964 /* DenoiseFilter.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      1284 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 1964 /* DenoiseFilter.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      2,				// *** number of lines required for processing ***
      1284 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #2 ***
    {
      (SEQ_Head_Ptr_t) 1964 /* DenoiseFilter.mHead */,	// *** offset to consumers (node) configuration header ***
      2,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      3,				// *** number of lines required for processing ***
      1284 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #3 ***
    {
      (SEQ_Head_Ptr_t) 1964 /* DenoiseFilter.mHead */,	// *** offset to consumers (node) configuration header ***
      3,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      4,				// *** number of lines required for processing ***
      1284 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #4 ***
    {
      (SEQ_Head_Ptr_t) 1964 /* DenoiseFilter.mHead */,	// *** offset to consumers (node) configuration header ***
      4,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      5,				// *** number of lines required for processing ***
      1284 * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      1,				// *** increment of current relative index ***
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
}; // RGB_2_YUVBuffer0_sony_dualexp

SEQ_Buf_t RGB_2_YUVBuffer1_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  21,				// *** number of lines fitting into output buffer (wrap) ***
  1284 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2568),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1348 /*RGB_2_YUV*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 68 /* ConvertOut.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      3,				// *** number of lines required for processing ***
      1284 * 2,				// *** number of bytes per line to read ***
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
}; // RGB_2_YUVBuffer1_sony_dualexp

SEQ_Buf_t RGB_2_YUVBuffer2_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  21,				// *** number of lines fitting into output buffer (wrap) ***
  1284 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2568),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1348 /*RGB_2_YUV*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 68 /* ConvertOut.mHead */,	// *** offset to consumers (node) configuration header ***
      2,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      3,				// *** number of lines required for processing ***
      1284 * 2,				// *** number of bytes per line to read ***
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
}; // RGB_2_YUVBuffer2_sony_dualexp

SEQ_Buf_t CAM0Buffer0_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  200,				// *** number of lines fitting into output buffer (wrap) ***
  NUM_CAM_PIXELS * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2576),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 2296 /*CAM0*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 376 /* ChGain_Exposure.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      6,				// *** number of lines required for processing ***
      NUM_CAM_PIXELS * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      2,				// *** increment of current relative index ***
      0,				// *** total lines read from buffer ***
      0,				// *** buffer is ready to be read by this consumer ***
      0,				// *** maximum difference between read and write ***
      0,				// *** pad aligment ***
    },
    // *** consumer/read #1 ***
    {
      (SEQ_Head_Ptr_t) 376 /* ChGain_Exposure.mHead */,	// *** offset to consumers (node) configuration header ***
      1,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      51,				// *** number of lines required for processing ***
      NUM_CAM_PIXELS * 2,				// *** number of bytes per line to read ***
      0,				// *** current relative index into source buffer (read) ***
      2,				// *** increment of current relative index ***
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
}; // CAM0Buffer0_sony_dualexp

SEQ_Buf_t ChGain_ExposureBuffer0_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  16,				// *** number of lines fitting into output buffer (wrap) ***
  1288 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2576),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 376 /*ChGain_Exposure*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 1672 /* Vignetting.mHead */,	// *** offset to consumers (node) configuration header ***
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
}; // ChGain_ExposureBuffer0_sony_dualexp

SEQ_Buf_t DenoiseFilterBuffer0_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 0,		// *** pointer to output buffer ***
  16,				// *** number of lines fitting into output buffer (wrap) ***
  1280 * 2,				// *** number of bytes per line to read ***
  BUF_ALIGN(2560),				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 1964 /*DenoiseFilter*/,		// *** offset to producers (node) configuration header ***
  0,				// *** current relative index into output buffer (write) ***
  1,				// *** increment of current relative index ***
  0,				// *** total lines written to buffer ***
  0,				// *** pad alignment ***

  {
    // *** consumer/read #0 ***
    {
      (SEQ_Head_Ptr_t) 68 /* ConvertOut.mHead */,	// *** offset to consumers (node) configuration header ***
      0,				// *** the devices channel used ***
      0,				// *** number of lines to skip at frame begin ***
      1,				// *** number of lines required for processing ***
      1280 * 2,				// *** number of bytes per line to read ***
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
}; // DenoiseFilterBuffer0_sony_dualexp

SEQ_Buf_t ISP_UYVY_OUTPUTBuffer_sony_dualexp = {

  // *** buffer ***
  (SEQ_Mem_Ptr_t) 1,		// *** pointer to output buffer ***
  720,				// *** number of lines fitting into output buffer (wrap) ***
  2560 * 1,				// *** number of bytes per line to read ***
  2560,				// *** number of bytes to next line begin ***
#if defined(_SEQ_HAS_FILLLEVEL_)
  0,				// *** index of buffer input/producer incrementing the buffer fill level ***
#endif // if defined(_SEQ_HAS_FILLLEVEL_)
  // *** producer/write ***
  0,				// *** buffers status ***
  (SEQ_Head_Ptr_t) 2332 /*FastDMA */,		// *** offset to producers (node) configuration header ***
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
      2560 * 1,				// *** number of bytes per line to read ***
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
}; // ISP_UYVY_OUTPUTBuffer_sony_dualexp


static SEQ_CfgPara_t ConvertOutPara [] = {
{ D_IPUS_SYN_HOST_XCFG_OFF, (2560 << 16) | 0x1 },
    { D_IPUS_SYN_HOST_INACFG_OFF, 0x00000700 },
    { D_IPUS_SYN_HOST_OUTCFG_OFF, 0x00000100 },
    { D_IPUS_SYN_S_CH0_CFG_INA_OFF, 0x00054000 },
    { D_IPUS_SYN_S_CH1_CFG_INA_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH2_CFG_INA_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH0_CFG_OUT_OFF, 0x10000000 },
};

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
    { D_IPUS_SYN_S_CH0_CFG_OUT_OFF, 0x00000000 },
    { D_IPUS_SYN_S_CH1_CFG_OUT_OFF, 0x00000000 },
    { D_IPUS_SYN_S_CH2_CFG_OUT_OFF, 0x00000000 },
};

static SEQ_CfgPara_t RGB_2_YUVPara [] = {
{ D_IPUV_SYN_HOST_XCFG_OFF, (1284<< 16) | 0x0 },
    { D_IPUV_SYN_HOST_INACFG_OFF, 0x00000700 },
    { D_IPUV_SYN_HOST_OUTCFG_OFF, 0x00000700 },
    { D_IPUV_SYN_S_CH0_CFG_INA_OFF, 0x00054000 },
    { D_IPUV_SYN_S_CH1_CFG_INA_OFF, 0x00058000 },
    { D_IPUV_SYN_S_CH2_CFG_INA_OFF, 0x00054000 },
    { D_IPUV_SYN_S_CH0_CFG_OUT_OFF, 0x00000000 },
    { D_IPUV_SYN_S_CH1_CFG_OUT_OFF, 0x00000000 },
    { D_IPUV_SYN_S_CH2_CFG_OUT_OFF, 0x00000000 },
};

static SEQ_CfgPara_t VignettingPara [] = {
{ D_IPUS_SYN_HOST_XCFG_OFF, (1288 << 16) | 0x1 },
    { D_IPUS_SYN_HOST_INACFG_OFF, 0x00000100 },
    { D_IPUS_SYN_HOST_OUTCFG_OFF, 0x00000100 },
    { D_IPUS_SYN_S_CH0_CFG_INA_OFF, 0x00058000 },
    { D_IPUS_SYN_S_CH0_CFG_OUT_OFF, 0x00000000 },
};

static SEQ_CfgPara_t DenoiseFilterPara [] = {
{ D_IPUV_SYN_HOST_XCFG_OFF, (1280<< 16) | 0x0 },
    { D_IPUV_SYN_HOST_INACFG_OFF, 0x00001F00 },
    { D_IPUV_SYN_HOST_OUTCFG_OFF, 0x00000100 },
    { D_IPUV_SYN_S_CH0_CFG_INA_OFF, 0x00058000 },
    { D_IPUV_SYN_S_CH1_CFG_INA_OFF, 0x00058000 },
    { D_IPUV_SYN_S_CH2_CFG_INA_OFF, 0x00058000 },
    { D_IPUV_SYN_S_CH3_CFG_INA_OFF, 0x00058000 },
    { D_IPUV_SYN_S_CH4_CFG_INA_OFF, 0x00058000 },
    { D_IPUV_SYN_S_CH0_CFG_OUT_OFF, 0x00000000 },
    { D_IPUV_SYN_S_CH1_CFG_OUT_OFF, 0x00000000 },
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
          (SEQ_Buf_Ptr_t) 4700 /*CAM0Buffer0_sony_dualexp*/,			// *** pointer to output buffer ***
          (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
          (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
          (SEQ_Buf_Ptr_t) 0,			// *** pointer to output buffer ***
      },
      NUM_CAM_LINES,			// *** number of input image lines ***
      0			// *** pad alignment ***
    }; // CAM0

static SEQ_IpuCfg_t ConvertOut = {
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
          (SEQ_Buf_Ptr_t) 2540 /* ConvertOutBuffer0_sony_dualexp */,			// *** pointer to output buffer ***
          {
            D_IPUS_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
            0,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            2560,					// *** number of pixels to transfer ***
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
          (SEQ_Buf_Ptr_t) 5132 /* DenoiseFilterBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
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
          (SEQ_Buf_Ptr_t) 4268 /* RGB_2_YUVBuffer1_sony_dualexp */,			// *** pointer to input buffer ***
          {
            D_IPUS_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
            2,					// *** number of first line to be used ***
            2 * 2,					// *** number of bytes to first pixel in line ***
            1280,					// *** number of pixels to transfer ***
          },
          0,				// *** our index in buffers consumer array ***
          0,				// *** pad alignment ***
        },
        // *** input buffer # ***
        {
          (SEQ_Buf_Ptr_t) 4484 /* RGB_2_YUVBuffer2_sony_dualexp */,			// *** pointer to input buffer ***
          {
            D_IPUS_SYN_S_LINE2_LEN_INA_OFF,				// *** offset to SDMA's register ***
            2,					// *** number of first line to be used ***
            2 * 2,					// *** number of bytes to first pixel in line ***
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
      (SEQ_CfgPara_CPtr_t) 292 /*ConvertOutPara */,	// *** pointer to configuration parameters ***
      7,					// *** number of configuration parameters following ***
      0,				// *** pad alignment ***
    };

static SEQ_IpuCfg_t ChGain_Exposure = {
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
      728,					// *** number of processing steps per frame ***
    
      {
        // *** output buffer # ***
        {
          (SEQ_Buf_Ptr_t) 4916 /* ChGain_ExposureBuffer0_sony_dualexp */,			// *** pointer to output buffer ***
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
          (SEQ_Buf_Ptr_t) 4700 /* CAM0Buffer0_sony_dualexp */,			// *** pointer to input buffer ***
          {
            D_IPUS_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
            5,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1288,					// *** number of pixels to transfer ***
          },
          0,				// *** our index in buffers consumer array ***
          0,				// *** pad alignment ***
        },
        // *** input buffer # ***
        {
          (SEQ_Buf_Ptr_t) 4700 /* CAM0Buffer0_sony_dualexp */,			// *** pointer to input buffer ***
          {
            D_IPUS_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
            50,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1288,					// *** number of pixels to transfer ***
          },
          1,				// *** our index in buffers consumer array ***
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
      (SEQ_CfgPara_CPtr_t) 600 /*ChGain_ExposurePara */,	// *** pointer to configuration parameters ***
      6,					// *** number of configuration parameters following ***
      0,				// *** pad alignment ***
    };

static SEQ_IpuCfg_t Debayer_G = {
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
      726,					// *** number of processing steps per frame ***
    
      {
        // *** output buffer # ***
        {
          (SEQ_Buf_Ptr_t) 2972 /* Debayer_GBuffer0_sony_dualexp */,			// *** pointer to output buffer ***
          {
            D_IPUS_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
            0,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1286,					// *** number of pixels to transfer ***
          },
        },
        // *** output buffer # ***
        {
          (SEQ_Buf_Ptr_t) 3188 /* Debayer_GBuffer1_sony_dualexp */,			// *** pointer to output buffer ***
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
          (SEQ_Buf_Ptr_t) 2756 /* VignettingBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
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
          (SEQ_Buf_Ptr_t) 2756 /* VignettingBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
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
          (SEQ_Buf_Ptr_t) 2756 /* VignettingBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
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
      (SEQ_CfgPara_CPtr_t) 900 /*Debayer_GPara */,	// *** pointer to configuration parameters ***
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
      724,					// *** number of processing steps per frame ***
    
      {
        // *** output buffer # ***
        {
          (SEQ_Buf_Ptr_t) 3836 /* Debayer_BRBuffer0_sony_dualexp */,			// *** pointer to output buffer ***
          {
            D_IPUS_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
            0,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1284,					// *** number of pixels to transfer ***
          },
        },
        // *** output buffer # ***
        {
          (SEQ_Buf_Ptr_t) 3620 /* Debayer_BRBuffer1_sony_dualexp */,			// *** pointer to output buffer ***
          {
            D_IPUS_SYN_S_LINE1_LEN_OUT_OFF,	// *** offset to SDMA's register ***
            0,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1284,					// *** number of pixels to transfer ***
          },
        },
        // *** output buffer # ***
        {
          (SEQ_Buf_Ptr_t) 3404 /* Debayer_BRBuffer2_sony_dualexp */,			// *** pointer to output buffer ***
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
          (SEQ_Buf_Ptr_t) 3188 /* Debayer_GBuffer1_sony_dualexp */,			// *** pointer to input buffer ***
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
          (SEQ_Buf_Ptr_t) 3188 /* Debayer_GBuffer1_sony_dualexp */,			// *** pointer to input buffer ***
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
          (SEQ_Buf_Ptr_t) 3188 /* Debayer_GBuffer1_sony_dualexp */,			// *** pointer to input buffer ***
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
          (SEQ_Buf_Ptr_t) 2972 /* Debayer_GBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
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
          (SEQ_Buf_Ptr_t) 2972 /* Debayer_GBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
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
          (SEQ_Buf_Ptr_t) 2972 /* Debayer_GBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
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
      (SEQ_CfgPara_CPtr_t) 1216 /*Debayer_BRPara */,	// *** pointer to configuration parameters ***
      13,					// *** number of configuration parameters following ***
      0,				// *** pad alignment ***
    };

static SEQ_IpuCfg_t RGB_2_YUV = {
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
      724,					// *** number of processing steps per frame ***
    
      {
        // *** output buffer # ***
        {
          (SEQ_Buf_Ptr_t) 4052 /* RGB_2_YUVBuffer0_sony_dualexp */,			// *** pointer to output buffer ***
          {
            D_IPUV_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
            0,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1284,					// *** number of pixels to transfer ***
          },
        },
        // *** output buffer # ***
        {
          (SEQ_Buf_Ptr_t) 4268 /* RGB_2_YUVBuffer1_sony_dualexp */,			// *** pointer to output buffer ***
          {
            D_IPUV_SYN_S_LINE1_LEN_OUT_OFF,	// *** offset to SDMA's register ***
            0,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1284,					// *** number of pixels to transfer ***
          },
        },
        // *** output buffer # ***
        {
          (SEQ_Buf_Ptr_t) 4484 /* RGB_2_YUVBuffer2_sony_dualexp */,			// *** pointer to output buffer ***
          {
            D_IPUV_SYN_S_LINE2_LEN_OUT_OFF,	// *** offset to SDMA's register ***
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
          (SEQ_Buf_Ptr_t) 3836 /* Debayer_BRBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
          {
            D_IPUV_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
            0,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1284,					// *** number of pixels to transfer ***
          },
          0,				// *** our index in buffers consumer array ***
          0,				// *** pad alignment ***
        },
        // *** input buffer # ***
        {
          (SEQ_Buf_Ptr_t) 3620 /* Debayer_BRBuffer1_sony_dualexp */,			// *** pointer to input buffer ***
          {
            D_IPUV_SYN_S_LINE2_LEN_INA_OFF,				// *** offset to SDMA's register ***
            0,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1284,					// *** number of pixels to transfer ***
          },
          0,				// *** our index in buffers consumer array ***
          0,				// *** pad alignment ***
        },
        // *** input buffer # ***
        {
          (SEQ_Buf_Ptr_t) 3404 /* Debayer_BRBuffer2_sony_dualexp */,			// *** pointer to input buffer ***
          {
            D_IPUV_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
            0,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1284,					// *** number of pixels to transfer ***
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
      (SEQ_CfgPara_CPtr_t) 1572 /*RGB_2_YUVPara */,	// *** pointer to configuration parameters ***
      9,					// *** number of configuration parameters following ***
      0,				// *** pad alignment ***
    };

static SEQ_IpuCfg_t Vignetting = {
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
      728,					// *** number of processing steps per frame ***
    
      {
        // *** output buffer # ***
        {
          (SEQ_Buf_Ptr_t) 2756 /* VignettingBuffer0_sony_dualexp */,			// *** pointer to output buffer ***
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
          (SEQ_Buf_Ptr_t) 4916 /* ChGain_ExposureBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
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
      (SEQ_CfgPara_CPtr_t) 1896 /*VignettingPara */,	// *** pointer to configuration parameters ***
      5,					// *** number of configuration parameters following ***
      0,				// *** pad alignment ***
    };

static SEQ_IpuCfg_t DenoiseFilter = {
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
      720,					// *** number of processing steps per frame ***
    
      {
        // *** output buffer # ***
        {
          (SEQ_Buf_Ptr_t) 5132 /* DenoiseFilterBuffer0_sony_dualexp */,			// *** pointer to output buffer ***
          {
            D_IPUV_SYN_S_LINE0_LEN_OUT_OFF,	// *** offset to SDMA's register ***
            0,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1280,					// *** number of pixels to transfer ***
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
          (SEQ_Buf_Ptr_t) 4052 /* RGB_2_YUVBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
          {
            D_IPUV_SYN_S_LINE0_LEN_INA_OFF,				// *** offset to SDMA's register ***
            0,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1284,					// *** number of pixels to transfer ***
          },
          0,				// *** our index in buffers consumer array ***
          0,				// *** pad alignment ***
        },
        // *** input buffer # ***
        {
          (SEQ_Buf_Ptr_t) 4052 /* RGB_2_YUVBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
          {
            D_IPUV_SYN_S_LINE1_LEN_INA_OFF,				// *** offset to SDMA's register ***
            1,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1284,					// *** number of pixels to transfer ***
          },
          1,				// *** our index in buffers consumer array ***
          0,				// *** pad alignment ***
        },
        // *** input buffer # ***
        {
          (SEQ_Buf_Ptr_t) 4052 /* RGB_2_YUVBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
          {
            D_IPUV_SYN_S_LINE2_LEN_INA_OFF,				// *** offset to SDMA's register ***
            2,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1284,					// *** number of pixels to transfer ***
          },
          2,				// *** our index in buffers consumer array ***
          0,				// *** pad alignment ***
        },
        // *** input buffer # ***
        {
          (SEQ_Buf_Ptr_t) 4052 /* RGB_2_YUVBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
          {
            D_IPUV_SYN_S_LINE3_LEN_INA_OFF,				// *** offset to SDMA's register ***
            3,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1284,					// *** number of pixels to transfer ***
          },
          3,				// *** our index in buffers consumer array ***
          0,				// *** pad alignment ***
        },
        // *** input buffer # ***
        {
          (SEQ_Buf_Ptr_t) 4052 /* RGB_2_YUVBuffer0_sony_dualexp */,			// *** pointer to input buffer ***
          {
            D_IPUV_SYN_S_LINE4_LEN_INA_OFF,				// *** offset to SDMA's register ***
            4,					// *** number of first line to be used ***
            0,					// *** number of bytes to first pixel in line ***
            1284,					// *** number of pixels to transfer ***
          },
          4,				// *** our index in buffers consumer array ***
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
      (SEQ_CfgPara_CPtr_t) 2188 /*DenoiseFilterPara */,	// *** pointer to configuration parameters ***
      10,					// *** number of configuration parameters following ***
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
      (SEQ_Buf_Ptr_t) 5348 /* ISP_UYVY_OUTPUTBuffer_sony_dualexp */,	// *** offset to destination buffer ***
    },

    // *** output buffer #1 ***
    {
      (SEQ_Buf_Ptr_t) 0,			// *** pointer to destination buffer ***
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
      (SEQ_Buf_Ptr_t) 2540 /* ConvertOutBuffer0_sony_dualexp */,	// *** input buffer ***
      0, 					// *** index into buffers consumer array ***
      0, 				// *** line offset ***
    },

    {
      // *** input buffer #1 ***
      (SEQ_Buf_Ptr_t) 0,			// *** input buffer ***
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

static SEQ_Load_t sCodelut4k_yuv444touyvy_ConvertOut = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 3*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    3,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  lut4k_yuv444touyvy_start,		// *** Kernel address in KMEM *** 
  lut4k_yuv444touyvy_end - lut4k_yuv444touyvy_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 68 /* ConvertOut.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodeexposure_tonehdr_ChGain_Exposure = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS,		// *** address of hardware device ***
    0,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  exposure_tonehdr_start,		// *** Kernel address in KMEM *** 
  exposure_tonehdr_end - exposure_tonehdr_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 376 /* ChGain_Exposure.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodedebayer_g_Debayer_G = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 1*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    1,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  debayer_g_start,		// *** Kernel address in KMEM *** 
  debayer_g_end - debayer_g_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 676 /* Debayer_G.mHead */,			// *** pointer to IPU node to start ***
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
  (SEQ_Head_Ptr_t) 992 /* Debayer_BR.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodergb2yuv_sint_RGB_2_YUV = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUV_SYN_BASEADDRESS + 1*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    1,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  rgb2yuv_sint_start,		// *** Kernel address in KMEM *** 
  rgb2yuv_sint_end - rgb2yuv_sint_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 1348 /* RGB_2_YUV.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodevignetting_Vignetting = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUS_SYN_BASEADDRESS + 2*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    2,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  vignetting_start,		// *** Kernel address in KMEM *** 
  vignetting_end - vignetting_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 1672 /* Vignetting.mHead */,			// *** pointer to IPU node to start ***
};

static SEQ_Load_t sCodedenoise_DenoiseFilter = {
  {
    SEQ_NODE_FWLOAD,			// *** type of engine ***
    D_IPUV_SYN_BASEADDRESS + 2*ISP_IO_DEV_SPACE,		// *** address of hardware device ***
    2,			// *** index of requested engine *** (TODO resolve for -1)
    0,					// *** engine is running ***
    0,					// *** engine is done for this frame ***
    0,					// *** engine got some update ***
  },

  denoise_start,		// *** Kernel address in KMEM *** 
  denoise_end - denoise_start,	// *** Kernel length in bytes ***
  (0x00000000) / 4,	// *** Kernel address in IPU instruction mem ***
  0,				// *** pad alignment ***
  (SEQ_Head_Ptr_t) 1964 /* DenoiseFilter.mHead */,			// *** pointer to IPU node to start ***
};


/*****************************************************************************
* exported data
*****************************************************************************/

// *** graph ***

SEQ_Head_Ptr_t gpGraph_sony_dualexp[] = {
  (SEQ_Head_Ptr_t) 2332, // FastDMA
  (SEQ_Head_Ptr_t) 2296, // CAM0
  (SEQ_Head_Ptr_t) 68, // ConvertOut
  (SEQ_Head_Ptr_t) 376, // ChGain_Exposure
  (SEQ_Head_Ptr_t) 676, // Debayer_G
  (SEQ_Head_Ptr_t) 992, // Debayer_BR
  (SEQ_Head_Ptr_t) 1348, // RGB_2_YUV
  (SEQ_Head_Ptr_t) 1672, // Vignetting
  (SEQ_Head_Ptr_t) 1964, // DenoiseFilter
  (SEQ_Head_Ptr_t) 348, // sCodelut4k_yuv444touyvy_ConvertOut
  (SEQ_Head_Ptr_t) 648, // sCodeexposure_tonehdr_ChGain_Exposure
  (SEQ_Head_Ptr_t) 964, // sCodedebayer_g_Debayer_G
  (SEQ_Head_Ptr_t) 1320, // sCodedebayer_br_advanced_Debayer_BR
  (SEQ_Head_Ptr_t) 1644, // sCodergb2yuv_sint_RGB_2_YUV
  (SEQ_Head_Ptr_t) 1936, // sCodevignetting_Vignetting
  (SEQ_Head_Ptr_t) 2268, // sCodedenoise_DenoiseFilter
  (SEQ_Head_Ptr_t) 0		// ** END ***
};

// *** Graph parsing metadata ***

static Object_t sObjectList[38] = {
  { &gpGraph_sony_dualexp,	68,	0 },
  { &ConvertOut,	224,	68 },
  { &ConvertOutPara,	56,	292 },
  { &sCodelut4k_yuv444touyvy_ConvertOut,	28,	348 },
  { &ChGain_Exposure,	224,	376 },
  { &ChGain_ExposurePara,	48,	600 },
  { &sCodeexposure_tonehdr_ChGain_Exposure,	28,	648 },
  { &Debayer_G,	224,	676 },
  { &Debayer_GPara,	64,	900 },
  { &sCodedebayer_g_Debayer_G,	28,	964 },
  { &Debayer_BR,	224,	992 },
  { &Debayer_BRPara,	104,	1216 },
  { &sCodedebayer_br_advanced_Debayer_BR,	28,	1320 },
  { &RGB_2_YUV,	224,	1348 },
  { &RGB_2_YUVPara,	72,	1572 },
  { &sCodergb2yuv_sint_RGB_2_YUV,	28,	1644 },
  { &Vignetting,	224,	1672 },
  { &VignettingPara,	40,	1896 },
  { &sCodevignetting_Vignetting,	28,	1936 },
  { &DenoiseFilter,	224,	1964 },
  { &DenoiseFilterPara,	80,	2188 },
  { &sCodedenoise_DenoiseFilter,	28,	2268 },
  { &CAM0,	36,	2296 },
  { &FastDMA,	208,	2332 },
  { &ConvertOutBuffer0_sony_dualexp,	216,	2540 },
  { &VignettingBuffer0_sony_dualexp,	216,	2756 },
  { &Debayer_GBuffer0_sony_dualexp,	216,	2972 },
  { &Debayer_GBuffer1_sony_dualexp,	216,	3188 },
  { &Debayer_BRBuffer2_sony_dualexp,	216,	3404 },
  { &Debayer_BRBuffer1_sony_dualexp,	216,	3620 },
  { &Debayer_BRBuffer0_sony_dualexp,	216,	3836 },
  { &RGB_2_YUVBuffer0_sony_dualexp,	216,	4052 },
  { &RGB_2_YUVBuffer1_sony_dualexp,	216,	4268 },
  { &RGB_2_YUVBuffer2_sony_dualexp,	216,	4484 },
  { &CAM0Buffer0_sony_dualexp,	216,	4700 },
  { &ChGain_ExposureBuffer0_sony_dualexp,	216,	4916 },
  { &DenoiseFilterBuffer0_sony_dualexp,	216,	5132 },
  { &ISP_UYVY_OUTPUTBuffer_sony_dualexp,	216,	5348 },
};

static uint32_t sPointerList[] = {
  0, /* gpGraph: pointer to FastDMA */
  4, /* gpGraph: pointer to CAM0 */
  8, /* gpGraph: pointer to ConvertOut */
  12, /* gpGraph: pointer to ChGain_Exposure */
  16, /* gpGraph: pointer to Debayer_G */
  20, /* gpGraph: pointer to Debayer_BR */
  24, /* gpGraph: pointer to RGB_2_YUV */
  28, /* gpGraph: pointer to Vignetting */
  32, /* gpGraph: pointer to DenoiseFilter */
  36, /* gpGraph: pointer to sCodelut4k_yuv444touyvy_ConvertOut */
  40, /* gpGraph: pointer to sCodeexposure_tonehdr_ChGain_Exposure */
  44, /* gpGraph: pointer to sCodedebayer_g_Debayer_G */
  48, /* gpGraph: pointer to sCodedebayer_br_advanced_Debayer_BR */
  52, /* gpGraph: pointer to sCodergb2yuv_sint_RGB_2_YUV */
  56, /* gpGraph: pointer to sCodevignetting_Vignetting */
  60, /* gpGraph: pointer to sCodedenoise_DenoiseFilter */
  92, /* ConvertOut: pointer to OUTput buffer */
  140, /* ConvertOut: pointer to INput buffer */
  156, /* ConvertOut: pointer to INput buffer */
  172, /* ConvertOut: pointer to INput buffer */
  284, /* ConvertOut: pointer to config parameters */
  372, /* sCodelut4k_yuv444touyvy_ConvertOut: pointer to ConvertOut.mHead */
  400, /* ChGain_Exposure: pointer to OUTput buffer */
  448, /* ChGain_Exposure: pointer to INput buffer */
  464, /* ChGain_Exposure: pointer to INput buffer */
  592, /* ChGain_Exposure: pointer to config parameters */
  672, /* sCodeexposure_tonehdr_ChGain_Exposure: pointer to ChGain_Exposure.mHead */
  700, /* Debayer_G: pointer to OUTput buffer */
  712, /* Debayer_G: pointer to OUTput buffer */
  748, /* Debayer_G: pointer to INput buffer */
  764, /* Debayer_G: pointer to INput buffer */
  780, /* Debayer_G: pointer to INput buffer */
  892, /* Debayer_G: pointer to config parameters */
  988, /* sCodedebayer_g_Debayer_G: pointer to Debayer_G.mHead */
  1016, /* Debayer_BR: pointer to OUTput buffer */
  1028, /* Debayer_BR: pointer to OUTput buffer */
  1040, /* Debayer_BR: pointer to OUTput buffer */
  1064, /* Debayer_BR: pointer to INput buffer */
  1080, /* Debayer_BR: pointer to INput buffer */
  1096, /* Debayer_BR: pointer to INput buffer */
  1112, /* Debayer_BR: pointer to INput buffer */
  1128, /* Debayer_BR: pointer to INput buffer */
  1144, /* Debayer_BR: pointer to INput buffer */
  1208, /* Debayer_BR: pointer to config parameters */
  1344, /* sCodedebayer_br_advanced_Debayer_BR: pointer to Debayer_BR.mHead */
  1372, /* RGB_2_YUV: pointer to OUTput buffer */
  1384, /* RGB_2_YUV: pointer to OUTput buffer */
  1396, /* RGB_2_YUV: pointer to OUTput buffer */
  1420, /* RGB_2_YUV: pointer to INput buffer */
  1436, /* RGB_2_YUV: pointer to INput buffer */
  1452, /* RGB_2_YUV: pointer to INput buffer */
  1564, /* RGB_2_YUV: pointer to config parameters */
  1668, /* sCodergb2yuv_sint_RGB_2_YUV: pointer to RGB_2_YUV.mHead */
  1696, /* Vignetting: pointer to OUTput buffer */
  1744, /* Vignetting: pointer to INput buffer */
  1888, /* Vignetting: pointer to config parameters */
  1960, /* sCodevignetting_Vignetting: pointer to Vignetting.mHead */
  1988, /* DenoiseFilter: pointer to OUTput buffer */
  2036, /* DenoiseFilter: pointer to INput buffer */
  2052, /* DenoiseFilter: pointer to INput buffer */
  2068, /* DenoiseFilter: pointer to INput buffer */
  2084, /* DenoiseFilter: pointer to INput buffer */
  2100, /* DenoiseFilter: pointer to INput buffer */
  2180, /* DenoiseFilter: pointer to config parameters */
  2292, /* sCodedenoise_DenoiseFilter: pointer to DenoiseFilter.mHead */
  2312, /* CAM0: pointer to OUTput buffer */
  2552, /* ConvertOutBuffer0_sony_dualexp: pointer to producer */
  2564, /* ConvertOutBuffer0_sony_dualexp: pointer to consumer */
  2768, /* VignettingBuffer0_sony_dualexp: pointer to producer */
  2780, /* VignettingBuffer0_sony_dualexp: pointer to consumer */
  2804, /* VignettingBuffer0_sony_dualexp: pointer to consumer */
  2828, /* VignettingBuffer0_sony_dualexp: pointer to consumer */
  2984, /* Debayer_GBuffer0_sony_dualexp: pointer to producer */
  2996, /* Debayer_GBuffer0_sony_dualexp: pointer to consumer */
  3020, /* Debayer_GBuffer0_sony_dualexp: pointer to consumer */
  3044, /* Debayer_GBuffer0_sony_dualexp: pointer to consumer */
  3200, /* Debayer_GBuffer1_sony_dualexp: pointer to producer */
  3212, /* Debayer_GBuffer1_sony_dualexp: pointer to consumer */
  3236, /* Debayer_GBuffer1_sony_dualexp: pointer to consumer */
  3260, /* Debayer_GBuffer1_sony_dualexp: pointer to consumer */
  3416, /* Debayer_BRBuffer2_sony_dualexp: pointer to producer */
  3428, /* Debayer_BRBuffer2_sony_dualexp: pointer to consumer */
  3632, /* Debayer_BRBuffer1_sony_dualexp: pointer to producer */
  3644, /* Debayer_BRBuffer1_sony_dualexp: pointer to consumer */
  3848, /* Debayer_BRBuffer0_sony_dualexp: pointer to producer */
  3860, /* Debayer_BRBuffer0_sony_dualexp: pointer to consumer */
  4064, /* RGB_2_YUVBuffer0_sony_dualexp: pointer to producer */
  4076, /* RGB_2_YUVBuffer0_sony_dualexp: pointer to consumer */
  4100, /* RGB_2_YUVBuffer0_sony_dualexp: pointer to consumer */
  4124, /* RGB_2_YUVBuffer0_sony_dualexp: pointer to consumer */
  4148, /* RGB_2_YUVBuffer0_sony_dualexp: pointer to consumer */
  4172, /* RGB_2_YUVBuffer0_sony_dualexp: pointer to consumer */
  4280, /* RGB_2_YUVBuffer1_sony_dualexp: pointer to producer */
  4292, /* RGB_2_YUVBuffer1_sony_dualexp: pointer to consumer */
  4496, /* RGB_2_YUVBuffer2_sony_dualexp: pointer to producer */
  4508, /* RGB_2_YUVBuffer2_sony_dualexp: pointer to consumer */
  4712, /* CAM0Buffer0_sony_dualexp: pointer to producer */
  4724, /* CAM0Buffer0_sony_dualexp: pointer to consumer */
  4748, /* CAM0Buffer0_sony_dualexp: pointer to consumer */
  4928, /* ChGain_ExposureBuffer0_sony_dualexp: pointer to producer */
  4940, /* ChGain_ExposureBuffer0_sony_dualexp: pointer to consumer */
  5144, /* DenoiseFilterBuffer0_sony_dualexp: pointer to producer */
  5156, /* DenoiseFilterBuffer0_sony_dualexp: pointer to consumer */
  5360, /* ISP_UYVY_OUTPUTBuffer_sony_dualexp: pointer to producer */
};

static uint32_t sBufferList[] = {
  2540, /* ConvertOutBuffer0_sony_dualexp */
  2756, /* VignettingBuffer0_sony_dualexp */
  2972, /* Debayer_GBuffer0_sony_dualexp */
  3188, /* Debayer_GBuffer1_sony_dualexp */
  3404, /* Debayer_BRBuffer2_sony_dualexp */
  3620, /* Debayer_BRBuffer1_sony_dualexp */
  3836, /* Debayer_BRBuffer0_sony_dualexp */
  4052, /* RGB_2_YUVBuffer0_sony_dualexp */
  4268, /* RGB_2_YUVBuffer1_sony_dualexp */
  4484, /* RGB_2_YUVBuffer2_sony_dualexp */
  4700, /* CAM0Buffer0_sony_dualexp */
  4916, /* ChGain_ExposureBuffer0_sony_dualexp */
  5132, /* DenoiseFilterBuffer0_sony_dualexp */
  5348, /* ISP_UYVY_OUTPUTBuffer_sony_dualexp */
};

static const char* scppFdmaChannelNames[] = {
  "ISP_UYVY_OUTPUT",
};

static const char* scppBufferNames[] = {
  "ConvertOutBuffer0_sony_dualexp",
  "VignettingBuffer0_sony_dualexp",
  "Debayer_GBuffer0_sony_dualexp",
  "Debayer_GBuffer1_sony_dualexp",
  "Debayer_BRBuffer2_sony_dualexp",
  "Debayer_BRBuffer1_sony_dualexp",
  "Debayer_BRBuffer0_sony_dualexp",
  "RGB_2_YUVBuffer0_sony_dualexp",
  "RGB_2_YUVBuffer1_sony_dualexp",
  "RGB_2_YUVBuffer2_sony_dualexp",
  "CAM0Buffer0_sony_dualexp",
  "ChGain_ExposureBuffer0_sony_dualexp",
  "DenoiseFilterBuffer0_sony_dualexp",
  "ISP_UYVY_OUTPUTBuffer_sony_dualexp",
};

GraphMetadata_t gGraphMetadata_sony_dualexp = {
  8,		// *** max number of buffer consumers
  5564,		// *** graph size [bytes]
  (const Object_t*) &sObjectList,	// *** object list pointer
  38,		// *** object count
  (const uint32_t*) &sPointerList,	// *** pointer list pointer
  104,		// *** pointer count
  (const uint32_t*) &sBufferList,	// *** buffer list pointer
  14,		// *** buffer count
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
