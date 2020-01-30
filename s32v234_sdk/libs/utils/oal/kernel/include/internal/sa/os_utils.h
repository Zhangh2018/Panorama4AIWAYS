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

#include <stdint.h>
#include <string.h>
#include <sys/cdefs.h>
#include <errno.h>
#include <sys/types.h>
#include <stdbool.h>

#define IOCTL_CALLBACK(NAME) void NAME(void)

__BEGIN_DECLS

unsigned long get_uptime_microS(void);

static inline void msleep(uint32_t msec)
{
        unsigned long lStart = get_uptime_microS();
        while((get_uptime_microS() - lStart) < msec * 1000U) {};
}

static inline unsigned long copy_from_user(void *dest, void *src, size_t l)
{
	(void) memcpy(dest, src, l);
	return 0;
}

static inline unsigned long copy_to_user(void *dest, void *src, size_t l)
{
	(void) memcpy(dest, src, l);
	return 0;
}

__END_DECLS

#endif /* OS_UTILS_H */
