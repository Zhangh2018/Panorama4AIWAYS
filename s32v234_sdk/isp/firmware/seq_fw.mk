##############################################################################
#
# Freescale Confidential Proprietary
#
# Copyright (c) 2015 Freescale Semiconductor;
# Copyright 2017 NXP;
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
#    Sequencer Firmware common build package
#
#  AUTHOR
#    Armin Strobl
#    Mihail Marian Nistor
#
#  DATE
#    2018-01-09
#
#  LANGUAGE
#    make
#
##############################################################################

##############################################################################
# check required parameters
##############################################################################

ifeq ($(origin SDK_ROOT),undefined)
  $(error error: SDK_ROOT undefined)
endif

#TOOLCH :=ghs
TOOLCH :=gcc

##############################################################################
# Sequencer Firmware
##############################################################################

SEQ_DEFS =
SEQ_INCS = \
	-I$(SDK_ROOT)/include                  \
	-I$(SDK_ROOT)/isp/firmware/inc         \
	-I$(SDK_ROOT)/isp/inc                  \
	-I$(SDK_ROOT)/platform/s32_v234

ifeq ($(TOOLCH),ghs)	### GHS ###########################

### release ###
#OPT_CCOPTS =-Ospeed
# -OI -Olink
# -Owholeprogram
#  DBG_CCOPTS =--no_debug

### debug ###
OPT_CCOPTS =-Onone
DBG_CCOPTS =-G -dwarf2

else	### GCC ###########################################

### release ###
OPT_CCOPTS =-O3 -fno-builtin

### debug ###
#OPT_CCOPTS =-Og -fno-builtin -D_ISP_DEBUG
DBG_CCOPTS =-g

endif

SEQ_CCOPTS =$(OPT_CCOPTS) $(DBG_CCOPTS) -DOSDIR=sa
