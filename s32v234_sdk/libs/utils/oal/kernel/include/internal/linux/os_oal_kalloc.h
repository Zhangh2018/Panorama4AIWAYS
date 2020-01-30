/******************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2017 NXP;
 * All Rights Reserved
 *
 *****************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITEDl TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#ifndef OS_OAL_KALLOC_H
#define OS_OAL_KALLOC_H

#include <linux/vmalloc.h>
#include <linux/slab.h>
#include "utils.h"
#include "priv_oal_kalloc.h"

static inline gfp_t to_linux_flags(unsigned flags)
{
	switch (flags) {
		case KERNEL_MEM:
			return GFP_KERNEL;
		case DMA_MEM:
			return GFP_DMA;
		default:
			return GFP_KERNEL;
	};

	return 0;
}

static inline void *OAL_Kalloc(size_t size, unsigned flags)
{
	void *ret;
	if (flags == VIRT_MEM) {
		ret = vmalloc(size);
	} else {
		ret = kmalloc(size, to_linux_flags(flags));
	}

	return ret;
}

static inline void *OAL_Kzalloc(size_t size, unsigned flags)
{
	void *ret;

	if (flags == VIRT_MEM) {
		ret = vzalloc(size);
	} else {
		ret = kzalloc(size, to_linux_flags(flags));
	}

	return ret;
}

static inline void OAL_Kfree(void *addr, unsigned flags)
{
	if (flags == VIRT_MEM) {
		vfree(addr);
	} else {
		kfree(addr);
	}
}

#endif /* OS_OAL_KALLOC_H */
