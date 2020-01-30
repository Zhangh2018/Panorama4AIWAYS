/*****************************************************************************
*
* Copyright 2017 NXP
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

#ifndef OS_UTILS_H
#define OS_UTILS_H

/* Linux kernel functinality */
#ifdef __KERNEL__
	#include <linux/types.h>
	#include <linux/fs.h>
	#include <linux/delay.h>
	#define __BEGIN_DECLS
	#define __END_DECLS

	#define IOCTL_CALLBACK(NAME) long NAME(struct file *apFile, \
                unsigned int aIoctlNum, \
                unsigned long aIoctlParam)

#else
	#include <sys/cdefs.h>
	#include <stdint.h>
	#include <signal.h>
	#include <errno.h>
	#include <stdbool.h>
	//#include "../libs/utils/oal/kernel/include/internal/posix/posix_delay.h"
	#include "../posix/posix_delay.h"

#endif

#endif /* OS_UTILS_H */
