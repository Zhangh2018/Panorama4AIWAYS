/******************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright 2017 NXP
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
 *****************************************************************************/

#include "oal.h"
#include "os_oal_check.h"
#include "vdb_log.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void flush_dcache_range(void *pMemory, unsigned int size);
extern void flush_and_invalidate_dcache_range(void *pMemory, unsigned int size);
extern void invalidate_dcache_range(void *pMemory, unsigned int size);

////////////////////////////////////////////////////////////////////////
/// CPU cache function - flush (todo)
////////////////////////////////////////////////////////////////////////
void OAL_MemoryFlush(void *pAddr, uint32_t size)
{
    int32_t retval = OAL_SUCCESS;

    CHECK_INIT(retval)
    if (retval == OAL_SUCCESS)
    {
        flush_dcache_range((void *)pAddr, size);
    }
}

////////////////////////////////////////////////////////////////////////
/// CPU cache function - flush and invalidate specific region
////////////////////////////////////////////////////////////////////////
void OAL_MemoryFlushAndInvalidate(void *pAddr, uint32_t size)
{
    int32_t retval = OAL_SUCCESS;

    CHECK_INIT(retval)
    if (retval == OAL_SUCCESS)
    {
        flush_and_invalidate_dcache_range((void *)pAddr, size);
    }
}

////////////////////////////////////////////////////////////////////////
/// CPU cache function - invalidate (todo)
////////////////////////////////////////////////////////////////////////
void OAL_MemoryInvalidate(void *pAddr, uint32_t size)
{
    int32_t retval = OAL_SUCCESS;

    CHECK_INIT(retval)
    if (retval == OAL_SUCCESS)
    {
        invalidate_dcache_range((void *)pAddr, size);
    }
}

////////////////////////////////////////////////////////////////////////
/// CPU cache function - flush and invalidate all
////////////////////////////////////////////////////////////////////////
void OAL_MemoryFlushAndInvalidateAll (void)
{
    int32_t retval = OAL_SUCCESS;

    CHECK_INIT(retval)
    VDB_LOG_WARNING("TODO IMPLEMENT FLUSH ALL!!!!!\n");
}

#ifdef __cplusplus
}
#endif
