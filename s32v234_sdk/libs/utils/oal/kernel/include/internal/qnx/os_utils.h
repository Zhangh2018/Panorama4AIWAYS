/*****************************************************************************
*
* Copyright 2017 NXP
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

#ifndef OS_UTILS_H
#define OS_UTILS_H

#include <sys/iofunc.h>
#include <sys/resmgr.h>

#include <stdint.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include "../libs/utils/oal/kernel/include/internal/posix/posix_delay.h"

/* Signal value */
#ifndef si_int
#define si_int si_value.sival_int
#endif

#define IOCTL_CALLBACK(NAME) int NAME(resmgr_context_t *ctp, \
					io_devctl_t *msg, \
					RESMGR_OCB_T *ocb)

__BEGIN_DECLS

static inline unsigned long copy_from_user(void *dest,
					const void *src, size_t l)
{
	(void) memcpy(dest, src, l);
	return 0;
}

static inline unsigned long copy_to_user(void *dest,
					const void *src, size_t l)
{
	(void) memcpy(dest, src, l);
	return 0;
}

__END_DECLS

#endif /* OS_UTILS_H */
