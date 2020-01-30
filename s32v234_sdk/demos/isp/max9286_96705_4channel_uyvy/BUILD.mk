##############################################################################
#
# Freescale Confidential Proprietary
#
# Copyright (c) 2015 Freescale Semiconductor;
# All Rights Reserved
#
##############################################################################
#
# THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
##############################################################################
#
#  CONTENT
#    Basic DCU output functionality test.
#
#  AUTHOR
#    Armin Strobl
#
#  DATE
#    2015-06-24
#
#  LANGUAGE
#    make
#
##############################################################################
#SDK_ROOT := $(call path_relative_to,$(CURDIR),$(CURR_SDK_ROOT))
SDK_ROOT := ../../../..

ARM_APP = max9286_96705_4channel_uyvy
ISP_GRAPH1 = max9286_96705_4
ISP_GRAPH = max9286_96705_4_uyvy
SEQ_MODE = dynamic
SEQ_FW = $(SEQ_MODE)_$(ISP_GRAPH)
##############################################################################
# isp_app
##############################################################################

VPATH = ../src/lib

ARM_APP_SRCS += main.cpp \
		mymemcpy.S \
        uart_to_mcu.c\
        ImageStitching.c\
		KeyTask.c\
		TerminalTask.c\
		demotest.c\
		network.c\
        transparent_vehicle_bottom.cpp  \
		CameraDevice.cpp \
	        DrawScene.cpp \
	        FinalScene.cpp \
		FreeScaleEgl.cpp \
                FreeScaleWin.cpp \
		ImageBilt.cpp \
		keyboardInput.cpp \
		ShaderUtil.cpp \
		TextureIL.cpp         \
                MatrixState.cpp \
		MergedPanorama3D.cpp \
		Model3D.cpp \
		MyVector3D.cpp \
		Panorama3DScene.cpp     \
                PanoramaParam.cpp \
		UnMergedPanorama3D.cpp \
		ModelRender.cpp \
		MyMatrix.cpp TrackBall.cpp     \
               camera_model.cpp \
		VertexArrayIO.cpp \
		PointGrid.cpp \
		PointMap.cpp  \
		CModifyableParam.cpp \
		Fbo.cpp \
		LUT.cpp \
		trail.cpp \
		cl_sample_timer.c  \
        nanovg.c        \
        DistanceLine.cpp    \
        RadarView.cpp       \
        BoxExpandView.cpp   \
        TransBot.cpp        \
        CVOGLBridge.cpp     \
        BirdEyeView.cpp \
        PlaneExpandView.cpp



#ARM_DEFS += -DLINUX -DEGL_API_FB
ARM_DEFS += -DLINUX -DEGL_API_FB -DGPU_TYPE_VIV -DGL_GLEXT_PROTOTYPES -DPLATFORM_S32V

APU_CCOPTS += -W -std=c++11 -fPIC -O0 -fno-strict-aliasing -fno-optimize-sibling-calls -Wall -g

ARM_INCS +=                                                  \
    -I$(SDK_ROOT)/platform/s32_v234                          \
    -I$(OPENCV_ROOT)/include                                 \
    -I$(SDK_ROOT)/include                                    \
    -I$(SDK_ROOT)/libs/io/sdi/include                       \
    -I$(SDK_ROOT)/libs/io/gdi/include                       \
    -I$(SDK_ROOT)/libs/utils/sumat/include                  \
    -I$(SDK_ROOT)/libs/utils/oal/user/include               \
    -I$(SDK_ROOT)/libs/utils/oal/kernel/include             \
    -I$(SDK_ROOT)/libs/utils/common/include                  \
    -I$(SDK_ROOT)/libs/apex/common/include                   \
    -I$(SDK_ROOT)/isp/inc                                    \
    -I$(SDK_ROOT)/libs/io/frame_io/include                   \
    -I$(SDK_ROOT)/isp/graphs/$(ISP_GRAPH1)/inc               \
    -I$(SDK_ROOT)/libs/io/dcu/include                        \
    -I${SDK_ROOT}/3rdparty/opengl/include                   \
	-I$(SDK_ROOT)/isp/graphs/$(ISP_GRAPH)/$(SEQ_MODE)/$(ODIR)\
    -I$(SDK_ROOT)/isp/graphs/$(ISP_GRAPH1)/$(ODIR)           \
    -I$(SDK_ROOT)/demos/isp/max9286_96705_4channel_uyvy/include  \
    -I${SDK_ROOT}/libs/opengl/include                           \
    -I$(SDK_ROOT)/demos/isp/max9286_96705_4channel_uyvy/inc       \
    -I$(SDK_ROOT)/demos/isp/max9286_96705_4channel_uyvy/inc/ogl       \
    -I$(SDK_ROOT)/demos/isp/max9286_96705_4channel_uyvy/inc/imgproc       \
    -I$(SDK_ROOT)/demos/isp/max9286_96705_4channel_uyvy/inc/utils       \
    -I$(SDK_ROOT)/demos/isp/max9286_96705_4channel_uyvy/inc/toml11 \
    #-I$(SDK_ROOT)/libs/isp/sequencer/kernel/include          \
    #-I${SDK_ROOT}/libs/opengl/include                        \
    #-I${OPENCL_ROOT}/include                            \

ARM_APP_LIBS +=                                              \
    $(SDK_ROOT)/libs/io/frame_io/$(ODIR)/libframe_io.a       \
    $(SDK_ROOT)/libs/isp/cam_generic/user/$(ODIR)/libcamdrv.a  \
    $(SDK_ROOT)/libs/isp/csi/user/$(ODIR)/libcsidrv.a          \
    $(SDK_ROOT)/libs/utils/log/$(ODIR)/liblog.a                \
    $(SDK_ROOT)/libs/utils/common/$(ODIR)/libcommon.a       \
    $(SDK_ROOT)/libs/utils/sumat/$(ODIR)/libsumat.a         \
    $(SDK_ROOT)/libs/isp/sequencer/user/$(ODIR)/libseqdrv.a    \
    $(SDK_ROOT)/libs/isp/fdma/user/$(ODIR)/libfdmadrv.a        \
    $(SDK_ROOT)/libs/utils/oal/user/$(ODIR)/liboal.a           \
    $(SDK_ROOT)/libs/isp/sram/user/$(ODIR)/libsramdrv.a        \
    $(SDK_ROOT)/isp/firmware/$(ODIR)/sequencer.a               \
    $(SDK_ROOT)/isp/graphs/$(ISP_GRAPH)/$(SEQ_MODE)/$(ODIR)/lib$(SEQ_FW).a  	  \
    ${SDK_ROOT}/libs/opengl/lib/libassimp.a                               \
    ${SDK_ROOT}/libs/opengl/lib/libIL.a                                   \
    ${SDK_ROOT}/libs/opengl/share/OpenCV/3rdparty/lib/libzlib.a           \
    $(SDK_ROOT)/libs/io/sdi/$(ODIR)/libsdi.a                    \
    $(SDK_ROOT)/libs/io/gdi/$(ODIR)/libgdi.a                    \

ARM_LDOPTS +=                                                             \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs                     \
    -lstdc++ -lm -lGLESv2 -lEGL -ldl -Wl,--library-path=$(SDK_ROOT)/3rdparty/opengl/lib,-rpath-link=$(SDK_ROOT)/3rdparty/opengl/lib \
    -lEGL								\
    -lGLESv2                                                            \
    -lVDK             							\
    -lVivanteOpenCL   							\
    -lOpenCL          							\
    -lGAL                                                               \


    ifneq (,$(findstring gnu-sa,$(ODIR)))
  ARM_APP_LIBS +=                                                         \
    $(SDK_ROOT)/libs/io/i2c/$(ODIR)/libi2c.a                              \
    $(SDK_ROOT)/libs/io/dcu/$(ODIR)/libdcu.a                              \

endif

ifneq (,$(findstring gnu-linux,$(ODIR)))
  ARM_APP_LIBS +=                                                         \
    $(SDK_ROOT)/libs/isp/jpegdec/user/$(ODIR)/libjpegdecdrv.a             \
    $(SDK_ROOT)/libs/isp/h264enc/user/$(ODIR)/libh264encdrv.a             \
    $(SDK_ROOT)/libs/isp/h264dec/user/$(ODIR)/libh264decdrv.a             \
    $(SDK_ROOT)/libs/isp/viu/user/$(ODIR)/libviudrv.a                     \

endif

##############################################################################
# STANDALONE SPECIFIC INCLUDES
##############################################################################
ifneq (,$(findstring -sa,$(ODIR)))

ARM_APP_LIBS +=                                                              \
    $(SDK_ROOT)/libs/startup/v234ce_standalone/$(ODIR)/libv234ce.a           \
    $(SDK_ROOT)/libs/io/i2c/$(ODIR)/libi2c.a                                 \
    $(SDK_ROOT)/libs/io/semihost/$(ODIR)/libSemihost.a                       \
    $(SDK_ROOT)/libs/io/uartlinflex_io/$(ODIR)/liblinflex.a                 \

endif
