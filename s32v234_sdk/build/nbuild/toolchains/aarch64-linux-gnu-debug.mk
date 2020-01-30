##############################################################################
#
#  Freescale Semiconductor, Inc.
#  (c) Copyright 2011, Freescale Semiconductor, Inc.
#  Copyright 2017 NXP;
#  ALL RIGHTS RESERVED.
#
##############################################################################
#
#  CONTENT
#
#
#  AUTHOR
#    Stefan Schwarz
#
#  DATE
#    2013-08-05
#
#  LANGUAGE
#    make
#
##############################################################################

include $(SDK_ROOT)/build/nbuild/toolchains/aarch64-linux-gnu.mk

C0_DEFS +=

C0_CCOPTS += \
  -O0 \
  -g \
  -fdebug-prefix-map="$(OS_STYLE_CURR_SDK_ROOT)"="s32v234_sdk"

C0_CXOPTS += \
  -O0 \
  -g \
  -fdebug-prefix-map="$(OS_STYLE_CURR_SDK_ROOT)"="s32v234_sdk"

C0_ASOPTS += \
  -g \
  -fdebug-prefix-map="$(OS_STYLE_CURR_SDK_ROOT)"="s32v234_sdk"

C0_LDOPTS += \
  -g \
  -fdebug-prefix-map="$(OS_STYLE_CURR_SDK_ROOT)"="s32v234_sdk"

C0_PROMPT := arm64/gnu/debug

include $(SDK_ROOT)/build/nbuild/.C0.mk
