/*****************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2016 Freescale Semiconductor;
 * Copyright 2017 NXP
 *
 * All Rights Reserved
 *
 *****************************************************************************
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
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "frame_output_display.h"
#include "video/fsl_dcu_ioctl.h"
#include <frame_output_v234fb.h>
#include "oal.h"
#include "vdb_log.h"
#include <errno.h>
#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "linux/videodev2.h"
#include "linux/fb.h"

#define DCU_FRM_CNT    2

namespace io
{
struct LinuxDisplayType {
	struct fb_var_screeninfo mScreenInfo; ///< screen info from ioctl
	uint8_t *             mpFrameBuffer;  ///< whole framebuffer mapping
	uint8_t *             mppScreens[DCU_FRM_CNT]; ///< pointer to buffers 0,1
	uint32_t              mScreenIdx;     ///< index of the displayed buffer

	IO_DATA_DEPTH         mDepth;         ///< depth of color channel [bits]
	uint32_t              mBytesPerPix;   ///< number of bytes per pixel
	uint32_t              mFormat;        ///< DCU pixel format

	int32_t               mFbFd;          ///< file descriptor for /dev/fb0
};

#define HSCR	((LinuxDisplayType*)mhScreenHandle)


//***************************************************************************

void* FrameOutputDisplay::AllocScreenHandle() {
	LinuxDisplayType* presult= new LinuxDisplayType();
	memset(presult, sizeof(LinuxDisplayType), 0);
	return presult;
}

//***************************************************************************

void FrameOutputDisplay::FreeScreenHandle(void* handle) {
	delete (LinuxDisplayType*)handle;
}

//***************************************************************************

FrameOutputDisplay::FrameOutputDisplay(uint32_t aWidth,
		uint32_t aHeight,
		IO_DATA_DEPTH aDepth,
		IO_DATA_CHANNELS aChannels,
		uint32_t aFormat)
{
	BaseInit();

	HSCR->mDepth = aDepth;
	HSCR->mFormat=aFormat;

	FrameOutputDisplay::Init(aWidth,
			aHeight,
			aDepth,
			aChannels,
			aFormat);
}

//***************************************************************************

FrameOutputDisplay::FrameOutputDisplay()
{
	BaseInit();

	FrameOutputDisplay::Init(0,
			0,
			IO_DATA_DEPTH_NOT_INITIALIZED,
			IO_DATA_CH_NOT_INITIALIZED,
			0);
}

//***************************************************************************

void FrameOutputDisplay::BaseInit()
{
	mSize     = 0;
	mWidth    = 0;
	mHeight   = 0;
	mChannels = IO_DATA_CH3;
	mStatus   = IO_OK;
	mhScreenHandle = AllocScreenHandle();

	HSCR->mpFrameBuffer = NULL;
	HSCR->mScreenIdx=0;
	HSCR->mDepth = IO_DATA_DEPTH_NOT_INITIALIZED;
	HSCR->mBytesPerPix=0;
	HSCR->mFormat=0;
	HSCR->mFbFd=-1;

}

//***************************************************************************

void FrameOutputDisplay::Init( uint32_t aWidth,
		uint32_t aHeight,
		IO_DATA_DEPTH aDepth,
		IO_DATA_CHANNELS aChannels,
		uint32_t aFormat)
{

	mWidth    = aWidth;
	mHeight   = aHeight;
	mChannels = aChannels;
	mStatus   = IO_OK;

	HSCR->mDepth    = aDepth;
	HSCR->mFormat   = aFormat;

	if(HSCR->mpFrameBuffer)
	{
		munmap(HSCR->mpFrameBuffer, mSize);
		close(HSCR->mFbFd);
		HSCR->mpFrameBuffer = NULL;
	} // reset before reopening

	HSCR->mFbFd = open("/dev/fb0", O_RDWR);

	if (HSCR->mFbFd < 0)
	{
		mStatus = IO_ERROR;
	} // if file open failed
	else
	{
		mStatus = IO_OK;
	} // else from if file open failed

	struct fb_fix_screeninfo mFixedInfo;  ///< fixed frame buffer info from ioctl

	///< Linux display:
	///< Get current Linux FB info and geometry setup.
	if (!ioctl( HSCR->mFbFd,
			FBIOGET_VSCREENINFO,
			&HSCR->mScreenInfo) &&
			!ioctl( HSCR->mFbFd,
					FBIOGET_FSCREENINFO,
					&mFixedInfo))
	{
		///< Linux display:
		///< Modify the pixel format if needed.

		// *** change number of channels ***
		if(mChannels == 2)
		{
			HSCR->mScreenInfo.bits_per_pixel = 16;
			HSCR->mScreenInfo.transp.offset = 0;
			HSCR->mScreenInfo.transp.length = 0;
			HSCR->mScreenInfo.transp.msb_right = 0;
		} // if 2 channels
		else
			if(mChannels == 3)
			{
				if(aFormat == DCU_BPP_YCbCr422)
				{
					HSCR->mScreenInfo.bits_per_pixel = 16;
					// RGBA componets setup has to be 0
					memset(&HSCR->mScreenInfo.red, 0, sizeof(HSCR->mScreenInfo.red));
					memset(&HSCR->mScreenInfo.green, 0, sizeof(HSCR->mScreenInfo.green));
					memset(&HSCR->mScreenInfo.blue, 0, sizeof(HSCR->mScreenInfo.blue));
					memset(&HSCR->mScreenInfo.transp, 0, sizeof(HSCR->mScreenInfo.transp));
					HSCR->mScreenInfo.grayscale = V4L2_PIX_FMT_UYVY;
				} // if DCU_BPP_YCbCr422
				else
				{
					// provide proper RGBA components setup
					HSCR->mScreenInfo.bits_per_pixel    = 24;
					HSCR->mScreenInfo.red.offset        = 16;
					HSCR->mScreenInfo.red.length        = 8;
					HSCR->mScreenInfo.red.msb_right     = 0;
					HSCR->mScreenInfo.green.offset      = 8;
					HSCR->mScreenInfo.green.length      = 8;
					HSCR->mScreenInfo.green.msb_right   = 0;
					HSCR->mScreenInfo.blue.offset       = 0;
					HSCR->mScreenInfo.blue.length       = 0;
					HSCR->mScreenInfo.blue.msb_right    = 0;
					HSCR->mScreenInfo.transp.offset     = 0;
					HSCR->mScreenInfo.transp.length     = 0;
					HSCR->mScreenInfo.transp.msb_right  = 0;
					HSCR->mScreenInfo.grayscale         = 0;
				} // else from if DCU_BPP_YCbCr422
			} // if 3 color channels
			else if(mChannels == 4)
			{
				// provide proper RGBA components setup
				HSCR->mScreenInfo.bits_per_pixel = 32;
				HSCR->mScreenInfo.red.offset        = 16;
				HSCR->mScreenInfo.red.length        = 8;
				HSCR->mScreenInfo.red.msb_right     = 0;
				HSCR->mScreenInfo.green.offset      = 8;
				HSCR->mScreenInfo.green.length      = 8;
				HSCR->mScreenInfo.green.msb_right   = 0;
				HSCR->mScreenInfo.blue.offset       = 0;
				HSCR->mScreenInfo.blue.length       = 0;
				HSCR->mScreenInfo.blue.msb_right    = 0;
				HSCR->mScreenInfo.transp.offset     = 24;
				HSCR->mScreenInfo.transp.length     = 8;
				HSCR->mScreenInfo.transp.msb_right  = 0;
				HSCR->mScreenInfo.grayscale         = 0;
			} // if 4 color channels
			else
			{
				VDB_LOG_WARNING("Unknown config required. Keeping default settings:\n");
				VDB_LOG_WARNING("v_heigth == %u, v_width == %u\n",
						HSCR->mScreenInfo.yres_virtual, HSCR->mScreenInfo.xres_virtual);
				VDB_LOG_WARNING("height == %u, width == %u\n",
						HSCR->mScreenInfo.yres, HSCR->mScreenInfo.xres);
				VDB_LOG_WARNING("bpp == %u\n",
						HSCR->mScreenInfo.bits_per_pixel);
			} // else from if 4 color channels

		///< Linux display:
		///< Modify geometry if needed.

		HSCR->mScreenInfo.yres_virtual  = mHeight * DCU_FRM_CNT;
		HSCR->mScreenInfo.xres_virtual  = mWidth;
		HSCR->mScreenInfo.yres          = mHeight;
		HSCR->mScreenInfo.xres          = mWidth;

		// *** reset PAN if any ***
		HSCR->mScreenInfo.yoffset       = 0;
		HSCR->mScreenInfo.xoffset       = 0;

		///< Linux display:
		///< Apply the updates to LinuxFB
		// set screeninfo
		ioctl(HSCR->mFbFd,
				FBIOPUT_VSCREENINFO,
				&HSCR->mScreenInfo);
	}

	mSize = mWidth * mHeight * HSCR->mScreenInfo.bits_per_pixel/8;

	// map the whole frambuffer
	HSCR->mpFrameBuffer = (uint8_t *)mmap(0,
			mSize * DCU_FRM_CNT, // double buffered
			PROT_READ | PROT_WRITE | PROT_EXEC,
			MAP_SHARED,
			HSCR->mFbFd,
			0x0);

	if(HSCR->mpFrameBuffer == (void*)-1)
	{
		VDB_LOG_ERROR("Framebuffer mapping failed.\n");
		VDB_LOG_ERROR("%s\n", strerror(errno));
	}
	else
	{
		VDB_LOG_ERROR("Framebuffer mapped at %p.\n", HSCR->mpFrameBuffer);
	}
	// setup screen pointers
	for(uint32_t i = 0; i < DCU_FRM_CNT; i++)
	{
		HSCR->mppScreens[i] = HSCR->mpFrameBuffer + i * mSize;
		//VDB_LOG_ERROR("%p\n", mppScreens[i]);
	} // for all screens
} // FrameOutputV234Fb::FrameOutputV234Fb()

//***************************************************************************

void FrameOutputDisplay::Disable()
{
	if(HSCR->mpFrameBuffer)
	{
		munmap(HSCR->mpFrameBuffer, mSize);
	}

	if(HSCR->mFbFd != -1)
	{
		close(HSCR->mFbFd);
	} // if fb file opened
} // FrameOutputV234Fb::Disable()

//***************************************************************************

FrameOutputDisplay::~FrameOutputDisplay()
{
	Disable();
	FreeScreenHandle(mhScreenHandle);
} // FrameOutputV234Fb::~FrameOutputV234Fb()

//***************************************************************************

void FrameOutputV234Fb::PutFrame(vsdk::SUMat frame)
{
  ///< Linux display:
  ///< Keep track of the back/front buffer index.
  uint32_t lScreenIdx = (HSCR->mScreenIdx + 1) % DCU_FRM_CNT;

  ///< Linux display:
  ///< Copy user data to backbuffer.

  vsdk::SMat matrix = frame.getMat(vsdk::ACCESS_READ | OAL_USAGE_CACHED);
  if (!matrix.empty())
    memcpy(HSCR->mppScreens[lScreenIdx], matrix.data, mSize);
  else
    printf("DCU: Cannot map a buffer.\n");

  ///< Linux display:
  ///< Adjust the panning in Linux FB.
  HSCR->mScreenInfo.yoffset = lScreenIdx * mHeight;

  ///< Linux display:
  ///< Display the backbuffer.
  // set screeninfo
  ioctl(HSCR->mFbFd,
        FBIOPAN_DISPLAY,
        &HSCR->mScreenInfo);
  HSCR->mScreenIdx = lScreenIdx;
} // FrameOutputV234Fb::PutFrame(

void FrameOutputDisplay::PutFrame(void *frame)
{
	PutFrame(frame, true);
} // FrameOutputV234Fb::PutFrame(void *frame)

//***************************************************************************

void FrameOutputDisplay::PutFrame(void *frame, bool aInvalidateCache)
{
	///< Linux display:
	///< Keep track of the back/front buffer index.
	uint32_t lScreenIdx = (HSCR->mScreenIdx + 1) % DCU_FRM_CNT;

	///< Linux display:
	///< Copy user data to backbuffer.

	if(aInvalidateCache)
	{
		OAL_MemoryInvalidate(frame, mSize);
	} // if cache to be invalidated

	memcpy(HSCR->mppScreens[lScreenIdx], frame, mSize);

	///< Linux display:
	///< Adjust the panning in Linux FB.
	HSCR->mScreenInfo.yoffset = lScreenIdx * mHeight;

	///< Linux display:
	///< Display the backbuffer.
	// set screeninfo
	ioctl(HSCR->mFbFd,
			FBIOPAN_DISPLAY,
			&HSCR->mScreenInfo);
	HSCR->mScreenIdx = lScreenIdx;
} // FrameOutputV234Fb::PutFrame(

//***************************************************************************

void FrameOutputDisplay::PutFrameAlpha(void *frame, bool aInvalidateCache)
{
	uint8_t *lpLineSrc = (uint8_t*)frame;
	uint8_t *lpLineDst = (uint8_t*)HSCR->mpFrameBuffer;

	if(aInvalidateCache)
	{
		OAL_MemoryInvalidate(frame, mSize);
	} // if cache to be invalidated

	ioctl(HSCR->mFbFd, FBIO_WAITFORVSYNC, 0);

	for(uint32_t y = 0; y < mHeight; y++)
	{
		uint8_t *lpSrc = lpLineSrc;
		uint8_t *lpDst = lpLineDst;

		for(uint32_t x = 0; x <mWidth; x++)
		{
			lpDst[0] = lpSrc[0];
			lpDst[1] = lpSrc[1];
			lpDst[2] = lpSrc[2];
			lpDst[3] = 255;
			lpDst += 4;
			lpSrc += 3;
		}
		lpLineSrc += mWidth * 4;
		lpLineDst += mWidth * 4;
	}
} // FrameOutputV234Fb::PutFrameAlpha(

} // namespace io

//***************************************************************************
