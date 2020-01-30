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


#ifndef OAL_TYPES_H_
#define OAL_TYPES_H_

#ifdef __cplusplus
extern "C"
{
#else
#include <stdbool.h>
#endif

typedef struct
{
    char     name[50];
    uint64_t start_address;
    uint32_t size;
    uint8_t  id;
    uint32_t align;
    uint8_t  init;
    uint8_t  autobalance;
} oal_mem;

static inline int is_memory_zone_empty(const oal_mem zone)
{
    bool a = ((bool)zone.start_address || (bool)zone.size || (bool)zone.id ||
              (bool)zone.align || (bool)zone.init || (bool)zone.autobalance);
    return !a;
}

#ifdef __cplusplus
}
#endif

#endif /* OAL_TYPES_H_ */
