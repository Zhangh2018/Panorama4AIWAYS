/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2014 Freescale Semiconductor
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
#ifndef __STRUCT_DEFS_H__
#define __STRUCT_DEFS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#if 0
/**
 * Structure saves info about user space mappings
 */
typedef struct UserAddress
{
  /**
   * Process id
   */
  uint32_t pid;

  /**
   * User virtual memory
   */
  uint64_t user_virtual;

  /**
   * Next in double-linked list
   */
  struct UserAddress *next;

  /**
   * Previous in double-linked list
   */
  struct UserAddress *prev;
} t_UserAddress;
#endif	/* if 0 */

/**
 * Structure saves info about kernel space contiguous memory allocation
 */
typedef struct
{
  /**
   * Process id
   */
  uint32_t pid;

  uint64_t address[5];

  /**
   * Allocation size
   */
  uint64_t size;

  /**
   * Memory chunk id
   */
  uint8_t chunk_id;

} t_MemArea;

#ifdef __cplusplus
}
#endif

#endif
