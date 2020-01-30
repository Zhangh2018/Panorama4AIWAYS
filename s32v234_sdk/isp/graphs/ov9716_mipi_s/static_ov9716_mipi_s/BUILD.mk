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
#    ov9716 ISP graph. Includes build of relevant kernels
#
#  AUTHOR
#    Armin Strobl
#
#  DATE
#    2015-06-26
#
#  LANGUAGE
#    make
#
##############################################################################
SDK_ROOT := $(call path_relative_to,$(CURDIR),$(CURR_SDK_ROOT))

SEQ_MODE = static

include ../../graph.mk

# old code
# SDK_ROOT :=	../../../../../s32v234_sdk
# SEQ_FW_HD = $(SDK_ROOT)/isp/firmware/inc/static_seq.h

# ##############################################################################
# # ISP Graph
# ##############################################################################
# ARM_LIB =	ov9716_mipi_s.a

# ARM_LIB_SRCS =	\
# 		ov9716_mipi_s.c

# ARM_INCS =	\
# 		-I../inc					\
# 		-I$(SDK_ROOT)/isp/inc				\
# 		-I$(SDK_ROOT)/platform/s32_v234			\

# GDT_OPTS =	-m

# STATIC_SEQ = @cp ../static_seq/static_seq_ov9716.h $(SEQ_FW_HD)
# #STATIC_SEQ = @cp ../old/inc/static_seq_ov9716_old.h $(SEQ_FW_HD)
# #STATIC_SEQ = @echo "/* dynamic sequencing */" > $(SEQ_FW_HD)

# ##############################################################################
# # IPUx kernels
# ##############################################################################
# KERNEL_DIR = $(SDK_ROOT)/isp/kernels/ov9716_mipi_s/src
# IPUS_APP = ipus.elf
# IPUS_APP_SRCS = \
#   $(KERNEL_DIR)/lut4k_tee.ipus			\
#   $(KERNEL_DIR)/histogram.ipus			\
#   $(KERNEL_DIR)/hdr_16x.ipus			\
#   $(KERNEL_DIR)/debayer_even.ipus		\
#   $(KERNEL_DIR)/debayer_odd.ipus		\
#   $(KERNEL_DIR)/lut4k.ipus			\
#   $(KERNEL_DIR)/lut4k_hist.ipus			\
#   $(KERNEL_DIR)/denoise_median.ipus     	\
#   $(KERNEL_DIR)/probe2bgr_2x.ipus		\

# #  $(KERNEL_DIR)/hdr.ipus			\
#   $(KERNEL_DIR)/exposure.ipus			\
#   $(KERNEL_DIR)/rgb2y_lut.ipus			\

# IPUV_APP = ipuv.elf
# IPUV_APP_SRCS = \
#   $(KERNEL_DIR)/hdr_blend.ipuv			\
#   $(KERNEL_DIR)/rgb2y.ipuv			\
#   $(KERNEL_DIR)/rgb_compress_y_uv.ipuv		\
#   $(KERNEL_DIR)/rgb_compress_y_uv_ccm.ipuv	\
#   $(KERNEL_DIR)/denoise4x4_uv.ipuv		\

# #  $(KERNEL_DIR)/rgb_compress.ipuv		\
#   $(KERNEL_DIR)/rgb_compress_y_uv.ipuv		\
#   $(KERNEL_DIR)/rgb_compress_rgby.ipuv		\
#   $(KERNEL_DIR)/rgb_compress_vuy.ipuv		\

# KMEM = kmem
