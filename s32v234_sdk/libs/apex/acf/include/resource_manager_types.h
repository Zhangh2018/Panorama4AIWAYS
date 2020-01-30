/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017 NXP 
* All Rights Reserved
*
******************************************************************************
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

/* Information for doxygen */
/**
 *  \file   <<FileName>>
 *  \brief  <<One line Description>>
 *
 *  <<Replace this with Multiline description. This should describe the
 *  file in general.>>
 */

#ifndef RESOURCEMANAGERTYPES_H
#define RESOURCEMANAGERTYPES_H

/***************************************************************************/
/* Include files. */
#include "resource_manager_private.h"
#include "oal.h"
#include "global_errors.h"
#include <stdint.h>
#include <stdio.h>


#if defined __INTEGRITY
#if defined __cplusplus
  extern "C" {
#endif
#endif

/****************************************************************************/
/* Literals & definitions */


/****************************************************************************/
/* Class declaration. */
/**
 * \brief This class is responsible for ...
 *
 *
 */
LIB_RESULT ResMgr_Helper_TypeName_Find(const char* const cpcTypeNameToFind,
                                       const char* const cpcTypeNames,
                                       uint32_t const cMaxNameLength,
                                       uint32_t const cNumberOfTypes,
                                       uint32_t* const pReturnedIndex);

int ResMgr_Helper_qsort_compare_uint32_t_x3_2_1(const void* a,
                                                const void* b);


int ResMgr_Helper_qsort_compare_uint32_t_x3_2_1_0(const void* a,
                                                  const void* b);

int ResMgr_Helper_qsort_compare_uint32_t_x4_3_2_1_0(const void * a,
                                                    const void * b);


#if defined __INTEGRITY
#if defined __cplusplus
  }
#endif
#endif


#endif /* RESOURCEMANAGERTYPES_H */
