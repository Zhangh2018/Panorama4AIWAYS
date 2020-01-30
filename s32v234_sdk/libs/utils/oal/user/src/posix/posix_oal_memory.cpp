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

#include "oal_memory_driver_if.h"
#include "os_oal_check.h"
#include "oal.h"

/* CPU cache function - flush */
void OAL_MemoryFlush(void *pAddr, uint32_t size)
{
    int32_t ret = OAL_SUCCESS;
    CHECK_INIT(ret)
    if (ret == OAL_SUCCESS)
    {
        if (pAddr) {
            CleanDCache(pAddr, size);
        }
    }
}

/* CPU cache function - flush and invalidate */
void OAL_MemoryFlushAndInvalidate(void *pAddr, uint32_t size)
{
    int32_t ret = OAL_SUCCESS;
	CHECK_INIT(ret)
    if (ret == OAL_SUCCESS)
    {
        if (pAddr) {
            CleanAndInvalidateDCache(pAddr, size);
        }
    }
}

/* CPU cache function - invalidate */
void OAL_MemoryInvalidate(void *pAddr, uint32_t size)
{
    int32_t ret = OAL_SUCCESS;
    CHECK_INIT(ret)
    if (ret == OAL_SUCCESS)
    {
        if (pAddr) {
            InvalidateDCache(pAddr, size);
        }
    }
}

/* CPU cache function - flush and invalidate all */
void OAL_MemoryFlushAndInvalidateAll (void)
{
    int32_t ret = OAL_SUCCESS;
    CHECK_INIT(ret)
    if (ret == OAL_SUCCESS)
    {
        CleanAndInvalidateDCacheAll ();
    }
}
