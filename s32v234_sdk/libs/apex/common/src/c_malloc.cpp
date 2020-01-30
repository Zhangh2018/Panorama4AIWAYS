/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2018 NXP Semiconductor;
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
****************************************************************************/

#include <string.h>
#include <errno.h>

#include <stdio.h>
#include <malloc.h>
#include "c_malloc.h"

// Make sure NUM_BLOCKS is multiple of 2 for 16-bit alignment
#define NUM_BLOCKS    (8192-2-256-128)
#define HDR_RESERVED 0x8000
#define HDR_SIZE     0x7FFF

// #define DEBUG_C_MALLOC

typedef struct CBlock
{
   struct CBlock* next;
   struct CBlock* prev;
   unsigned size;
   unsigned idx;
} CBlock;

static vec08u chess_storage(VMb%2) cmem_pool[NUM_BLOCKS+2];
static CBlock* cmem_track = 0;
static CBlock* head = 0;

static void init() {
#ifdef DEBUG_C_MALLOC
   printf("Initializing c_malloc\nFirst cmem block addr: 0x%x\n", (uint32_t)&cmem_pool[0]);
#endif
   cmem_track = (CBlock*)malloc(sizeof(CBlock));
   head = cmem_track;
   head->next = 0;
   head->prev = 0;
   head->size = NUM_BLOCKS;
   head->idx = 2; // Set to 0 if for sure cmem address's will not start at 0
}

vec08u* c_malloc(unsigned size) {
#ifdef DEBUG_C_MALLOC
   printf("Allocating %d bytes of CMEM\n", size);
#endif
   if (!head) {
      init();
   }
   if (size == 0) return 0;
   vec08u* ptr = (vec08u*)0;
   CBlock* block = head;
   unsigned aligned = (size+3)&(~0x3);
   while(block) {
      if (!(block->size & HDR_RESERVED)) {
         if (aligned < block->size) {
            unsigned idx = block->idx + block->size - aligned;
            
            CBlock* alloc = (CBlock*)malloc(sizeof(CBlock));
            
            alloc->next = block->next;
            alloc->prev = block;
            alloc->size = aligned | HDR_RESERVED;
            alloc->idx = idx;
            if (block->next) block->next->prev = alloc;

            block->next = alloc;
            block->size = block->size - aligned;
            
            ptr = (vec08u*)(&cmem_pool[idx]);
            break;
         } else if (aligned == block->size) {
            block->size |= HDR_RESERVED;
            ptr = (vec08u*)(&cmem_pool[block->idx]);
            break;
         }
      }
      block = block->next;
   }
#ifdef DEBUG_C_MALLOC
   printf("CMEM:\n");
   CBlock* tmp = head;
   while(tmp) {
      if (tmp->size & HDR_RESERVED) {
         printf("-- RSRV  0x%8p: %5u %u %5u\n", tmp, tmp->size&HDR_SIZE, !!(tmp->size&HDR_RESERVED), tmp->idx);
      } else {
         printf("-- FREE  0x%8p: %5u %u %5u\n", tmp, tmp->size&HDR_SIZE, !!(tmp->size&HDR_RESERVED), tmp->idx);
      }
      printf("--- next 0x%8p: %5u %u %5u\n", tmp->next, tmp->next->size&HDR_SIZE, !!(tmp->next->size&HDR_RESERVED), tmp->next->idx);
      printf("--- prev 0x%8p: %5u %u %5u\n", tmp->prev, tmp->prev->size&HDR_SIZE, !!(tmp->prev->size&HDR_RESERVED), tmp->prev->idx);
      tmp = tmp->next;
   }
#endif
   return ptr;
}

void c_free_(uint32_t address) {
   if (address == 0) return;
   if (!head) {
      init();
   }
#ifdef DEBUG_C_MALLOC
   printf("Free CMEM at %u\n", address);
#endif
   uint32_t idx = (uint32_t)address - (uint32_t)&cmem_pool[0];
   CBlock* block = head;
   while(block) {
      if (block->idx == idx) {
         if (block->size & HDR_RESERVED) {
#ifdef DEBUG_C_MALLOC
            printf("Freed %d bytes at %u of CMEM\n", block->size&HDR_SIZE, address);
#endif
            // If the previous block is free, merge them together
            // else just set this block as free
            block->size &= HDR_SIZE;
            if (block->prev && !(block->prev->size & HDR_RESERVED)) {
               CBlock* prev = block->prev;
               prev->next = block->next;
               if (block->next) block->next->prev = prev;
               prev->size += block->size;
               free(block);
               block = prev;
            }
            // Check the next block to see if it is free, if so
            // merge them
            if (block->next && !(block->next->size&HDR_RESERVED)) {
               CBlock* next = block->next;
               block->next = next->next;
               if (next->next) next->next->prev = block;
               block->size += next->size;
               free(next);
            }
         }
         break;
      }
      block = block->next;
   }
#ifdef DEBUG_C_MALLOC
   printf("CMEM:\n");
   CBlock* tmp = head;
   while(tmp) {
      if (tmp->size & HDR_RESERVED) {
         printf("-- RSRV  0x%8p: %5u %u %5u\n", tmp, tmp->size&HDR_SIZE, !!(tmp->size&HDR_RESERVED), tmp->idx);
      } else {
         printf("-- FREE  0x%8p: %5u %u %5u\n", tmp, tmp->size&HDR_SIZE, !!(tmp->size&HDR_RESERVED), tmp->idx);
      }
      printf("--- next 0x%8p: %5u %u %5u\n", tmp->next, tmp->next->size&HDR_SIZE, !!(tmp->next->size&HDR_RESERVED), tmp->next->idx);
      printf("--- prev 0x%8p: %5u %u %5u\n", tmp->prev, tmp->prev->size&HDR_SIZE, !!(tmp->prev->size&HDR_RESERVED), tmp->prev->idx);
      tmp = tmp->next;
   }
#endif
}

