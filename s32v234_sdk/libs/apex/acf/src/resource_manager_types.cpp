/******************************************************************************
 *  (C) Copyright CogniVue Corp. 2015 All right reserved.
 *    Confidential Information
 *
 *  All parts of the CogniVue Corp. Program Source are protected by copyright law
 *  and all rights are reserved.
 *  This documentation may not, in whole or in part, be copied, photocopied,
 *  reproduced, translated, or reduced to any electronic medium or machine
 *  readable form without prior consent, in writing, from Cognivue.
 ******************************************************************************/


#include "resource_manager_types.h"
#include <string.h>


LIB_RESULT ResMgr_Helper_TypeName_Find(const char* const cpcTypeNameToFind,
                                       const char* const cpcTypeNames,
                                       uint32_t const cMaxNameLength,
                                       uint32_t const cNumberOfTypes,
                                       uint32_t* const cpReturnedIndex)
{
   /* */
   /* Could optimize if I used OrderedLookup and did a binary search */
   /* */
   LIB_RESULT libResult = LIB_FAILURE;

   if(cpcTypeNameToFind != 0 &&
      cpcTypeNameToFind[0] != 0 &&  /* Don't want empty string */
      cpcTypeNames != 0 &&
      cMaxNameLength > 0 &&
      cpReturnedIndex != 0)
   {
      uint32_t i = 0;

      const char* pcCurrentName = cpcTypeNames;
      const uint32_t cNameSpanInBytes = cMaxNameLength + 1; /* +1 For null terminating byte. */

      *cpReturnedIndex = ~(0U);

      for (i = 0; i < cNumberOfTypes; ++i)
      {
         if(strncmp(cpcTypeNameToFind, pcCurrentName, cMaxNameLength) == 0)
         {
            *cpReturnedIndex = i;
            break;
         }
         pcCurrentName += cNameSpanInBytes;
      }

      libResult = LIB_SUCCESS;
   }

   return libResult;
}



int ResMgr_Helper_qsort_compare_uint32_t_x3_2_1 (const void * a, const void * b)
{
   int intResult = 0;

   const uint32_t* const cpcA = (uint32_t*)a;
   const uint32_t* const cpcB = (uint32_t*)b;

   intResult = (cpcA[2] > cpcB[2]) - (cpcB[2] > cpcA[2]);

   if(intResult == 0)
   {
      intResult = (cpcA[1] > cpcB[1]) - (cpcB[1] > cpcA[1]);
   }

   return intResult;
}



int ResMgr_Helper_qsort_compare_uint32_t_x3_2_1_0(const void * a, const void * b)
{
   int intResult = 0;

   const uint32_t* const cpcA = (uint32_t*)a;
   const uint32_t* const cpcB = (uint32_t*)b;

   intResult = (cpcA[2] > cpcB[2]) - (cpcB[2] > cpcA[2]);

   if(intResult == 0)
   {
      intResult = (cpcA[1] > cpcB[1]) - (cpcB[1] > cpcA[1]);

      if(intResult == 0)
      {
         intResult = (cpcA[0] > cpcB[0]) - (cpcB[0] > cpcA[0]);
      }
   }

   return intResult;
}



int ResMgr_Helper_qsort_compare_uint32_t_x4_3_2_1_0(const void * a, const void * b)
{
   int intResult = 0;

   const uint32_t* const cpcA = (uint32_t*)a;
   const uint32_t* const cpcB = (uint32_t*)b;

   intResult = (cpcA[3] > cpcB[3]) - (cpcB[3] > cpcA[3]);

   if(intResult == 0)
      {
      intResult = (cpcA[2] > cpcB[2]) - (cpcB[2] > cpcA[2]);

      if(intResult == 0)
         {
         intResult = (cpcA[1] > cpcB[1]) - (cpcB[1] > cpcA[1]);

         if(intResult == 0)
{
            intResult = (cpcA[0] > cpcB[0]) - (cpcB[0] > cpcA[0]);
      }
   }
}

   return intResult;
   }

