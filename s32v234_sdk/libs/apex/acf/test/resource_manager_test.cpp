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

#include "resource_manager_protected.h"
#include "resource_manager_private.h"
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#define SHOW_ERROR(expr) if(!(expr)) { printf("%s : %s [%d]\n", __FILE__, __PRETTY_FUNCTION__, __LINE__); }

#ifndef LIB_ERROR_INVALID_USER_ID
  #define LIB_RESOURCES_NOT_CURRENTLY_AVAILABLE           0xB5000 ///< The allocation request is valid but the not all resources were available presently
  #define LIB_ERROR_ATTEMPT_DEALLOC_UNOWNED_RESOURCE      0xB5001 ///< User attempted to deallocate a resource which did no belong to him/her
  #define LIB_ERROR_USER_STILL_OWNS_RESOURCES             0xB5002 ///< User attempted to deallocate a user who still has resources that need to be deallocated
  #define LIB_ERROR_USER_WAITING_ON_RESOURCES             0xB5003 ///< User attempted to deallocate a user who is still waiting on resources to be allocated
  #define LIB_ERROR_INITIALIZATION_VALUE_TOO_SMALL        0xB5004 ///< User attempted passed a value larger than what is supported by the values passed to ResMgr_Initialize
  #define LIB_ERROR_INVALID_USER_ID                       0xB5005 ///< User attempted to reuse a user id which is no longer valid or provide a wrong value
  #define LIB_ERROR_OUTPUT_ARRAY_SIZE_TOO_LARGE           0xB5006 ///< User passed a value larger than what is supported by the values passed to ResMgr_Initialize
  #define LIB_ERROR_USER_INDEX_TOO_LARGE                  0xB5007 ///< User passed a value larger than what is supported by the values passed to ResMgr_Initialize
  #define LIB_ERROR_SESSION_ID_MISMATCH                   0xB5008 ///< User session id does not match with current session for that user index
  #define LIB_ERROR_USER_STILL_HAS_RESOURCES              0xB5009 ///< User attempted to allocate resources before freeing those he currently owns
  #define LIB_NO_USER_WAITING_FOR_RESOURCES               0xB500A ///< There are no users waiting for resources, meaning the queue is empty
  #define LIB_USER_OBTAINED_RESOURCES                     0xB500B ///< A user was allocated resources in the returning function, if he was blocked he should be unblocked
  #define LIB_USER_WAITING_FOR_RESOURCES                  0xB500C
#endif

/* Initialize */
LIB_RESULT ResourceManager_h_Test_000()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult = LIB_FAILURE;

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { { 4,  1,   2}, /* Set */
                                                              {13, 10, 124}, /* Resource */
                                                              { 3,  1,   6}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 64;  /* In this case 10 comes from the ten resource types associated with the "APEX" set type */

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize(0,
                                 0,
                                 cInitInputArray,
                                 cInitInputArraySize,
                                 cMaxSizeOfResourceAllocateInputArray,
                                 &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize(pBuffer,
                                       neededSizeInBytes,
                                       cInitInputArray,
                                       cInitInputArraySize,
                                       cMaxSizeOfResourceAllocateInputArray,
                                       &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            libResult = ResMgr_Deinitialize(); SHOW_ERROR(libResult == LIB_SUCCESS);
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



/* Initialize */
/* Set Registration */
LIB_RESULT ResourceManager_h_Test_001()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult = LIB_FAILURE;

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { { 4,  1,   2}, /* Set */
                                                              {13, 10, 124}, /* Resource */
                                                              { 3,  1,   6}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 64;  /* In this case 10 comes from the ten resource types associated with the "APEX" set type */

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize(0,
                                 0,
                                 cInitInputArray,
                                 cInitInputArraySize,
                                 cMaxSizeOfResourceAllocateInputArray,
                                 &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize(pBuffer,
                                       neededSizeInBytes,
                                       cInitInputArray,
                                       cInitInputArraySize,
                                       cMaxSizeOfResourceAllocateInputArray,
                                       &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            #define cNumberOfSetAndResTypes  11          /* 1 Set Type + 10 Resource Types */
            #define cSetAndResMaxNameLength  13          /* const uint32_t cResMaxNameLength = 13; */

            const char cSetAndResName[cNumberOfSetAndResTypes][cSetAndResMaxNameLength+1] = { "APEX",            /* Name of the set */
                                                                                              "APU",             /* Following are the names of the resource types */
                                                                                              "DMA",
                                                                                              "MCE",
                                                                                              "STREAMIN",
                                                                                              "STREAMOUT",
                                                                                              "SEQ_FIXED_RES",
                                                                                              "SEQ_ZERO_RES",
                                                                                              "SEQ_PROC_DEC",
                                                                                              "CU_32",
                                                                                              "SMEM_64K"
                                                                                            };

            uint32_t const cSetAndResInputArray[cNumberOfSetAndResTypes] = { 2,  /* "APEX",          */   /* Number of Set instances */
                                                                             2,  /* "APU",           */   /* Number of Res instances for the Set type */
                                                                             8,  /* "DMA",           */
                                                                             2,  /* "MCE",           */
                                                                             4,  /* "STREAMIN",      */
                                                                             2,  /* "STREAMOUT",     */
                                                                            20,  /* "SEQ_FIXED_RES", */
                                                                             4,  /* "SEQ_ZERO_RES",  */
                                                                            16,  /* "SEQ_PROC_DEC",  */
                                                                             2,  /* "CU_32",         */
                                                                             2   /* "SMEM _64K"      */
                                                                           };

            uint32_t setAndResOutputArray[cNumberOfSetAndResTypes];


            libResult = ResMgr_Set_Register((const char*)cSetAndResName,
                                            cSetAndResMaxNameLength,
                                            cSetAndResInputArray,
                                            setAndResOutputArray,
                                            cNumberOfSetAndResTypes); SHOW_ERROR(libResult == LIB_SUCCESS);

            libResult = ResMgr_Deinitialize(); SHOW_ERROR(libResult == LIB_SUCCESS);
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



/* Initialize */
/* Set Registration */
/* User Registration */
LIB_RESULT ResourceManager_h_Test_002()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult = LIB_FAILURE;

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { { 4,  1,   2}, /* Set */
                                                              {13, 10, 124}, /* Resource */
                                                              { 3,  1,   6}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 64;  /* In this case 10 comes from the ten resource types associated with the "APEX" set type */

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize(0,
                                 0,
                                 cInitInputArray,
                                 cInitInputArraySize,
                                 cMaxSizeOfResourceAllocateInputArray,
                                 &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize(pBuffer,
                                       neededSizeInBytes,
                                       cInitInputArray,
                                       cInitInputArraySize,
                                       cMaxSizeOfResourceAllocateInputArray,
                                       &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            #define cNumberOfSetAndResTypes  11          /* 1 Set Type + 10 Resource Types */
            #define cSetAndResMaxNameLength  13          /* const uint32_t cResMaxNameLength = 13; */

            const char cSetAndResName[cNumberOfSetAndResTypes][cSetAndResMaxNameLength+1] = { "APEX",            /* Name of the set */
                                                                                              "APU",             /* Following are the names of the resource types */
                                                                                              "DMA",
                                                                                              "MCE",
                                                                                              "STREAMIN",
                                                                                              "STREAMOUT",
                                                                                              "SEQ_FIXED_RES",
                                                                                              "SEQ_ZERO_RES",
                                                                                              "SEQ_PROC_DEC",
                                                                                              "CU_32",
                                                                                              "SMEM_64K"
                                                                                            };

            uint32_t const cSetAndResInputArray[cNumberOfSetAndResTypes] = { 2,  /* "APEX",          */   /* Number of Set instances */
                                                                             2,  /* "APU",           */   /* Number of Res instances for the Set type */
                                                                             8,  /* "DMA",           */
                                                                             2,  /* "MCE",           */
                                                                             4,  /* "STREAMIN",      */
                                                                             2,  /* "STREAMOUT",     */
                                                                            20,  /* "SEQ_FIXED_RES", */
                                                                             4,  /* "SEQ_ZERO_RES",  */
                                                                            16,  /* "SEQ_PROC_DEC",  */
                                                                             2,  /* "CU_32",         */
                                                                             2   /* "SMEM _64K"      */
                                                                           };

            uint32_t setAndResOutputArray[cNumberOfSetAndResTypes];


            libResult = ResMgr_Set_Register((const char*)cSetAndResName,
                                            cSetAndResMaxNameLength,
                                            cSetAndResInputArray,
                                            setAndResOutputArray,
                                            cNumberOfSetAndResTypes); SHOW_ERROR(libResult == LIB_SUCCESS);
            if(libResult == LIB_SUCCESS)
            {
               #define cNumberOfUserNames 1 /* const uint32_t cNumberOfUserNames = 1; */
               #define cUserMaxNameLength 3 /* const uint32_t cUserMaxNameLength = 3; */

               const char cUserName[cNumberOfUserNames][cUserMaxNameLength+1] = { "ACF" };

               uint32_t userOutputArray[cNumberOfUserNames];

               libResult = ResMgr_User_Type_Register((const char*)cUserName,
                                                     cUserMaxNameLength,
                                                     userOutputArray,
                                                     cNumberOfUserNames); SHOW_ERROR(libResult == LIB_SUCCESS);
            }

            libResult = ResMgr_Deinitialize(); SHOW_ERROR(libResult == LIB_SUCCESS);
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



/* Initialize */
/* Set Registration */
/* User Registration */
/* User Allocation   */
LIB_RESULT ResourceManager_h_Test_003()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult = LIB_FAILURE;

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { { 4,  1,   2}, /* Set */
                                                              {13, 10, 124}, /* Resource */
                                                              { 3,  1,   6}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 64;  /* In this case 10 comes from the ten resource types associated with the "APEX" set type */

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize(0,
                                 0,
                                 cInitInputArray,
                                 cInitInputArraySize,
                                 cMaxSizeOfResourceAllocateInputArray,
                                 &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize(pBuffer,
                                       neededSizeInBytes,
                                       cInitInputArray,
                                       cInitInputArraySize,
                                       cMaxSizeOfResourceAllocateInputArray,
                                       &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            #define cNumberOfSetAndResTypes  11          /* 1 Set Type + 10 Resource Types */
            #define cSetAndResMaxNameLength  13          /* const uint32_t cResMaxNameLength = 13; */

            const char cSetAndResName[cNumberOfSetAndResTypes][cSetAndResMaxNameLength+1] = { "APEX",            /* Name of the set */
                                                                                              "APU",             /* Following are the names of the resource types */
                                                                                              "DMA",
                                                                                              "MCE",
                                                                                              "STREAMIN",
                                                                                              "STREAMOUT",
                                                                                              "SEQ_FIXED_RES",
                                                                                              "SEQ_ZERO_RES",
                                                                                              "SEQ_PROC_DEC",
                                                                                              "CU_32",
                                                                                              "SMEM_64K"
                                                                                            };

            uint32_t const cSetAndResInputArray[cNumberOfSetAndResTypes] = { 2,  /* "APEX",          */   /* Number of Set instances */
                                                                             2,  /* "APU",           */   /* Number of Res instances for the Set type */
                                                                             8,  /* "DMA",           */
                                                                             2,  /* "MCE",           */
                                                                             4,  /* "STREAMIN",      */
                                                                             2,  /* "STREAMOUT",     */
                                                                            20,  /* "SEQ_FIXED_RES", */
                                                                             4,  /* "SEQ_ZERO_RES",  */
                                                                            16,  /* "SEQ_PROC_DEC",  */
                                                                             2,  /* "CU_32",         */
                                                                             2   /* "SMEM _64K"      */
                                                                           };

            uint32_t setAndResOutputArray[cNumberOfSetAndResTypes];


            libResult = ResMgr_Set_Register((const char*)cSetAndResName,
                                            cSetAndResMaxNameLength,
                                            cSetAndResInputArray,
                                            setAndResOutputArray,
                                            cNumberOfSetAndResTypes); SHOW_ERROR(libResult == LIB_SUCCESS);
            if(libResult == LIB_SUCCESS)
            {
               #define cNumberOfUserNames 1 /* const uint32_t cNumberOfUserNames = 1; */
               #define cUserMaxNameLength 3 /* const uint32_t cUserMaxNameLength = 3; */

               const char cUserName[cNumberOfUserNames][cUserMaxNameLength+1] = { "ACF" };

               uint32_t userOutputArray[cNumberOfUserNames];

               libResult = ResMgr_User_Type_Register((const char*)cUserName,
                                                     cUserMaxNameLength,
                                                     userOutputArray,
                                                     cNumberOfUserNames); SHOW_ERROR(libResult == LIB_SUCCESS);

               if(libResult == LIB_SUCCESS)
               {
                  uint32_t userId = 0;

                  libResult = ResMgr_User_Allocate(userOutputArray[0],
                                                   &userId); SHOW_ERROR(libResult == LIB_SUCCESS);
               }
            }

            libResult = ResMgr_Deinitialize(); SHOW_ERROR(libResult == LIB_SUCCESS);
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



/* Initialize */
/* Set Registration */
/* User Registration */
/* User Allocation   */
LIB_RESULT ResourceManager_h_Test_004()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult = LIB_FAILURE;

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { { 4,  1,   2}, /* Set */
                                                              {13, 10, 124}, /* Resource */
                                                              { 3,  1,   6}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 64;  /* In this case 10 comes from the ten resource types associated with the "APEX" set type */

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize(0,
                                 0,
                                 cInitInputArray,
                                 cInitInputArraySize,
                                 cMaxSizeOfResourceAllocateInputArray,
                                 &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize(pBuffer,
                                       neededSizeInBytes,
                                       cInitInputArray,
                                       cInitInputArraySize,
                                       cMaxSizeOfResourceAllocateInputArray,
                                       &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            #define cNumberOfSetAndResTypes  11          /* 1 Set Type + 10 Resource Types */
            #define cSetAndResMaxNameLength  13          /* const uint32_t cResMaxNameLength = 13; */

            const char cSetAndResName[cNumberOfSetAndResTypes][cSetAndResMaxNameLength+1] = { "APEX",            /* Name of the set */
                                                                                              "APU",             /* Following are the names of the resource types */
                                                                                              "DMA",
                                                                                              "MCE",
                                                                                              "STREAMIN",
                                                                                              "STREAMOUT",
                                                                                              "SEQ_FIXED_RES",
                                                                                              "SEQ_ZERO_RES",
                                                                                              "SEQ_PROC_DEC",
                                                                                              "CU_32",
                                                                                              "SMEM_64K"
                                                                                            };

            uint32_t const cSetAndResInputArray[cNumberOfSetAndResTypes] = { 2,  /* "APEX",          */   /* Number of Set instances */
                                                                             2,  /* "APU",           */   /* Number of Res instances for the Set type */
                                                                             8,  /* "DMA",           */
                                                                             2,  /* "MCE",           */
                                                                             4,  /* "STREAMIN",      */
                                                                             2,  /* "STREAMOUT",     */
                                                                            20,  /* "SEQ_FIXED_RES", */
                                                                             4,  /* "SEQ_ZERO_RES",  */
                                                                            16,  /* "SEQ_PROC_DEC",  */
                                                                             2,  /* "CU_32",         */
                                                                             2   /* "SMEM _64K"      */
                                                                           };

            uint32_t setAndResOutputArray[cNumberOfSetAndResTypes];


            libResult = ResMgr_Set_Register((const char*)cSetAndResName,
                                            cSetAndResMaxNameLength,
                                            cSetAndResInputArray,
                                            setAndResOutputArray,
                                            cNumberOfSetAndResTypes); SHOW_ERROR(libResult == LIB_SUCCESS);
            if(libResult == LIB_SUCCESS)
            {
               #define cNumberOfUserNames 1 /* const uint32_t cNumberOfUserNames = 1; */
               #define cUserMaxNameLength 3 /* const uint32_t cUserMaxNameLength = 3; */

               const char cUserName[cNumberOfUserNames][cUserMaxNameLength+1] = { "ACF" };

               uint32_t userOutputArray[cNumberOfUserNames];

               libResult = ResMgr_User_Type_Register((const char*)cUserName,
                                                     cUserMaxNameLength,
                                                     userOutputArray,
                                                     cNumberOfUserNames); SHOW_ERROR(libResult == LIB_SUCCESS);

               if(libResult == LIB_SUCCESS)
               {
                  uint32_t userId = 0;

                  libResult = ResMgr_User_Allocate(userOutputArray[0],
                                                   &userId); SHOW_ERROR(libResult == LIB_SUCCESS);

                  if(libResult == LIB_SUCCESS)
                  {
                     libResult = ResMgr_User_Deallocate(userId); SHOW_ERROR(libResult == LIB_SUCCESS);
                  }
               }
            }

            libResult = ResMgr_Deinitialize(); SHOW_ERROR(libResult == LIB_SUCCESS);
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



/* Initialize */
/* Set Registration */
/* User Registration */
/* User Allocation   */
/* User Deallocation */
LIB_RESULT ResourceManager_h_Test_005()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult = LIB_FAILURE;

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { { 4,  1,   2}, /* Set */
                                                              {13, 10, 124}, /* Resource */
                                                              { 3,  1,   6}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 64;  /* In this case 10 comes from the ten resource types associated with the "APEX" set type */

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize(0,
                                 0,
                                 cInitInputArray,
                                 cInitInputArraySize,
                                 cMaxSizeOfResourceAllocateInputArray,
                                 &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize(pBuffer,
                                       neededSizeInBytes,
                                       cInitInputArray,
                                       cInitInputArraySize,
                                       cMaxSizeOfResourceAllocateInputArray,
                                       &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            #define cNumberOfSetAndResTypes  11          /* 1 Set Type + 10 Resource Types */
            #define cSetAndResMaxNameLength  13          /* const uint32_t cResMaxNameLength = 13; */

            const char cSetAndResName[cNumberOfSetAndResTypes][cSetAndResMaxNameLength+1] = { "APEX",            /* Name of the set */
                                                                                              "APU",             /* Following are the names of the resource types */
                                                                                              "DMA",
                                                                                              "MCE",
                                                                                              "STREAMIN",
                                                                                              "STREAMOUT",
                                                                                              "SEQ_FIXED_RES",
                                                                                              "SEQ_ZERO_RES",
                                                                                              "SEQ_PROC_DEC",
                                                                                              "CU_32",
                                                                                              "SMEM_64K"
                                                                                            };

            uint32_t const cSetAndResInputArray[cNumberOfSetAndResTypes] = { 2,  /* "APEX",          */   /* Number of Set instances */
                                                                             2,  /* "APU",           */   /* Number of Res instances for the Set type */
                                                                             8,  /* "DMA",           */
                                                                             2,  /* "MCE",           */
                                                                             4,  /* "STREAMIN",      */
                                                                             2,  /* "STREAMOUT",     */
                                                                            20,  /* "SEQ_FIXED_RES", */
                                                                             4,  /* "SEQ_ZERO_RES",  */
                                                                            16,  /* "SEQ_PROC_DEC",  */
                                                                             2,  /* "CU_32",         */
                                                                             2   /* "SMEM _64K"      */
                                                                           };

            uint32_t setAndResOutputArray[cNumberOfSetAndResTypes];


            libResult = ResMgr_Set_Register((const char*)cSetAndResName,
                                            cSetAndResMaxNameLength,
                                            cSetAndResInputArray,
                                            setAndResOutputArray,
                                            cNumberOfSetAndResTypes); SHOW_ERROR(libResult == LIB_SUCCESS);
            if(libResult == LIB_SUCCESS)
            {
               #define cNumberOfUserNames 1 /* const uint32_t cNumberOfUserNames = 1; */
               #define cUserMaxNameLength 3 /* const uint32_t cUserMaxNameLength = 3; */

               const char cUserName[cNumberOfUserNames][cUserMaxNameLength+1] = { "ACF" };

               uint32_t userOutputArray[cNumberOfUserNames];

               libResult = ResMgr_User_Type_Register((const char*)cUserName,
                                                     cUserMaxNameLength,
                                                     userOutputArray,
                                                     cNumberOfUserNames); SHOW_ERROR(libResult == LIB_SUCCESS);

               if(libResult == LIB_SUCCESS)
               {
                  uint32_t userId = 0;

                  libResult = ResMgr_User_Allocate(userOutputArray[0],
                                                   &userId); SHOW_ERROR(libResult == LIB_SUCCESS);

                  if(libResult == LIB_SUCCESS)
                  {
                     libResult = ResMgr_User_Deallocate(userId); SHOW_ERROR(libResult == LIB_SUCCESS);
                  }
               }
            }

            libResult = ResMgr_Deinitialize(); SHOW_ERROR(libResult == LIB_SUCCESS);
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



/* Initialize */
/* Set Registration */
/* User Registration */
/* User Allocation   */
/* Resource Allocation */
LIB_RESULT ResourceManager_h_Test_006()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult = LIB_FAILURE;

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { { 4,  1,   2}, /* Set */
                                                              {13, 10, 124}, /* Resource */
                                                              { 3,  1,   6}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 64;  /* In this case 10 comes from the ten resource types associated with the "APEX" set type */

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize(0,
                                 0,
                                 cInitInputArray,
                                 cInitInputArraySize,
                                 cMaxSizeOfResourceAllocateInputArray,
                                 &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize(pBuffer,
                                       neededSizeInBytes,
                                       cInitInputArray,
                                       cInitInputArraySize,
                                       cMaxSizeOfResourceAllocateInputArray,
                                       &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            #define cNumberOfSetAndResTypes  11          /* 1 Set Type + 10 Resource Types */
            #define cSetAndResMaxNameLength  13          /* const uint32_t cResMaxNameLength = 13; */

            const char cSetAndResName[cNumberOfSetAndResTypes][cSetAndResMaxNameLength+1] = { "APEX",            /* Name of the set */
                                                                                              "APU",             /* Following are the names of the resource types */
                                                                                              "DMA",
                                                                                              "MCE",
                                                                                              "STREAMIN",
                                                                                              "STREAMOUT",
                                                                                              "SEQ_FIXED_RES",
                                                                                              "SEQ_ZERO_RES",
                                                                                              "SEQ_PROC_DEC",
                                                                                              "CU_32",
                                                                                              "SMEM_64K"
                                                                                            };

            uint32_t const cSetAndResInputArray[cNumberOfSetAndResTypes] = { 2,  /* "APEX",          */   /* Number of Set instances */
                                                                             2,  /* "APU",           */   /* Number of Res instances for the Set type */
                                                                             8,  /* "DMA",           */
                                                                             2,  /* "MCE",           */
                                                                             4,  /* "STREAMIN",      */
                                                                             2,  /* "STREAMOUT",     */
                                                                            20,  /* "SEQ_FIXED_RES", */
                                                                             4,  /* "SEQ_ZERO_RES",  */
                                                                            16,  /* "SEQ_PROC_DEC",  */
                                                                             2,  /* "CU_32",         */
                                                                             2   /* "SMEM _64K"      */
                                                                           };

            uint32_t setAndResOutputArray[cNumberOfSetAndResTypes];


            libResult = ResMgr_Set_Register((const char*)cSetAndResName,
                                            cSetAndResMaxNameLength,
                                            cSetAndResInputArray,
                                            setAndResOutputArray,
                                            cNumberOfSetAndResTypes); SHOW_ERROR(libResult == LIB_SUCCESS);
            if(libResult == LIB_SUCCESS)
            {
               #define cNumberOfUserNames 1 /* const uint32_t cNumberOfUserNames = 1; */
               #define cUserMaxNameLength 3 /* const uint32_t cUserMaxNameLength = 3; */

               const char cUserName[cNumberOfUserNames][cUserMaxNameLength+1] = { "ACF" };

               uint32_t userOutputArray[cNumberOfUserNames];

               libResult = ResMgr_User_Type_Register((const char*)cUserName,
                                                     cUserMaxNameLength,
                                                     userOutputArray,
                                                     cNumberOfUserNames); SHOW_ERROR(libResult == LIB_SUCCESS);

               if(libResult == LIB_SUCCESS)
               {
                  uint32_t userId = 0;

                  libResult = ResMgr_User_Allocate(userOutputArray[0],
                                                   &userId); SHOW_ERROR(libResult == LIB_SUCCESS);

                  if(libResult == LIB_SUCCESS)
                  {
                     const uint32_t cSetTypeId = setAndResOutputArray[0];

                     #define cResAllocInputArraySize6 2
                     #define cNumberOfResToAllocate6  2

                     uint32_t resAllocInputArray[cResAllocInputArraySize6][4] = { { cSetTypeId, 0, setAndResOutputArray[5+1], 0 },
                                                                                 { cSetTypeId, 0, setAndResOutputArray[7+1], 0 }
                                                                               };

                     uint32_t resAllocOutputArray[cNumberOfResToAllocate6][5];

                     memset(resAllocOutputArray, 0xFF, sizeof(resAllocOutputArray));

                     libResult = ResMgr_Resource_Allocate_II(userId,
                                                             resAllocInputArray,
                                                             cResAllocInputArraySize6,
                                                             resAllocOutputArray,
                                                             cNumberOfResToAllocate6); SHOW_ERROR(libResult == LIB_SUCCESS);
                  }
               }
            }

            libResult = ResMgr_Deinitialize(); SHOW_ERROR(libResult == LIB_SUCCESS);
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



/* Initialize */
/* Set Registration */
/* User Registration */
/* User Allocation   */
/* Resource Allocation */
/* User Deallocation   */
LIB_RESULT ResourceManager_h_Test_007()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult = LIB_FAILURE;

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { { 4,  1,   2}, /* Set */
                                                              {13, 10, 124}, /* Resource */
                                                              { 3,  1,   6}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 64;  /* In this case 10 comes from the ten resource types associated with the "APEX" set type */

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize(0,
                                 0,
                                 cInitInputArray,
                                 cInitInputArraySize,
                                 cMaxSizeOfResourceAllocateInputArray,
                                 &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize(pBuffer,
                                       neededSizeInBytes,
                                       cInitInputArray,
                                       cInitInputArraySize,
                                       cMaxSizeOfResourceAllocateInputArray,
                                       &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            #define cNumberOfSetAndResTypes  11          /* 1 Set Type + 10 Resource Types */
            #define cSetAndResMaxNameLength  13          /* const uint32_t cResMaxNameLength = 13; */

            const char cSetAndResName[cNumberOfSetAndResTypes][cSetAndResMaxNameLength+1] = { "APEX",            /* Name of the set */
                                                                                              "APU",             /* Following are the names of the resource types */
                                                                                              "DMA",
                                                                                              "MCE",
                                                                                              "STREAMIN",
                                                                                              "STREAMOUT",
                                                                                              "SEQ_FIXED_RES",
                                                                                              "SEQ_ZERO_RES",
                                                                                              "SEQ_PROC_DEC",
                                                                                              "CU_32",
                                                                                              "SMEM_64K"
                                                                                            };

            uint32_t const cSetAndResInputArray[cNumberOfSetAndResTypes] = { 2,  /* "APEX",          */   /* Number of Set instances */
                                                                             2,  /* "APU",           */   /* Number of Res instances for the Set type */
                                                                             8,  /* "DMA",           */
                                                                             2,  /* "MCE",           */
                                                                             4,  /* "STREAMIN",      */
                                                                             2,  /* "STREAMOUT",     */
                                                                            20,  /* "SEQ_FIXED_RES", */
                                                                             4,  /* "SEQ_ZERO_RES",  */
                                                                            16,  /* "SEQ_PROC_DEC",  */
                                                                             2,  /* "CU_32",         */
                                                                             2   /* "SMEM _64K"      */
                                                                           };

            uint32_t setAndResOutputArray[cNumberOfSetAndResTypes];


            libResult = ResMgr_Set_Register((const char*)cSetAndResName,
                                            cSetAndResMaxNameLength,
                                            cSetAndResInputArray,
                                            setAndResOutputArray,
                                            cNumberOfSetAndResTypes); SHOW_ERROR(libResult == LIB_SUCCESS);
            if(libResult == LIB_SUCCESS)
            {
               #define cNumberOfUserNames 1 /* const uint32_t cNumberOfUserNames = 1; */
               #define cUserMaxNameLength 3 /* const uint32_t cUserMaxNameLength = 3; */

               const char cUserName[cNumberOfUserNames][cUserMaxNameLength+1] = { "ACF" };

               uint32_t userOutputArray[cNumberOfUserNames];

               libResult = ResMgr_User_Type_Register((const char*)cUserName,
                                                     cUserMaxNameLength,
                                                     userOutputArray,
                                                     cNumberOfUserNames); SHOW_ERROR(libResult == LIB_SUCCESS);

               if(libResult == LIB_SUCCESS)
               {
                  uint32_t userId = 0;

                  libResult = ResMgr_User_Allocate(userOutputArray[0],
                                                   &userId); SHOW_ERROR(libResult == LIB_SUCCESS);

                  if(libResult == LIB_SUCCESS)
                  {
                     const uint32_t cSetTypeId = setAndResOutputArray[0];

                     #define cResAllocInputArraySize7 2
                     #define cNumberOfResToAllocate7  2

                     uint32_t resAllocInputArray[cResAllocInputArraySize7][4] = { { cSetTypeId, 0, setAndResOutputArray[5+1], 0 },
                                                                                  { cSetTypeId, 0, setAndResOutputArray[7+1], 0  }
                                                                               };

                     uint32_t resAllocOutputArray[cNumberOfResToAllocate7][5];

                     memset(resAllocOutputArray, 0xFF, sizeof(resAllocOutputArray));

                     libResult = ResMgr_Resource_Allocate_II(userId,
                                                             resAllocInputArray,
                                                             cResAllocInputArraySize7,
                                                             resAllocOutputArray,
                                                             cNumberOfResToAllocate7); SHOW_ERROR(libResult == LIB_SUCCESS);

                     libResult = ResMgr_User_Deallocate(userId); SHOW_ERROR(libResult == LIB_ERROR_USER_STILL_OWNS_RESOURCES);
                  }
               }
            }

            libResult = ResMgr_Deinitialize(); SHOW_ERROR(libResult == LIB_SUCCESS);
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



/* Initialize */
/* Set Registration */
/* User Registration */
/* User Allocation   */
/* Resource Allocation */
/* Resource Deallocation */
/* User Deallocation   */
LIB_RESULT ResourceManager_h_Test_008()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult = LIB_FAILURE;

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { { 4,  1,   2}, /* Set */
                                                              {13, 10, 124}, /* Resource */
                                                              { 3,  1,   6}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 64;  /* In this case 10 comes from the ten resource types associated with the "APEX" set type */

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize(0,
                                 0,
                                 cInitInputArray,
                                 cInitInputArraySize,
                                 cMaxSizeOfResourceAllocateInputArray,
                                 &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize(pBuffer,
                                       neededSizeInBytes,
                                       cInitInputArray,
                                       cInitInputArraySize,
                                       cMaxSizeOfResourceAllocateInputArray,
                                       &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            #define cNumberOfSetAndResTypes  11          /* 1 Set Type + 10 Resource Types */
            #define cSetAndResMaxNameLength  13          /* const uint32_t cResMaxNameLength = 13; */

            const char cSetAndResName[cNumberOfSetAndResTypes][cSetAndResMaxNameLength+1] = { "APEX",            /* Name of the set */
                                                                                              "APU",             /* Following are the names of the resource types */
                                                                                              "DMA",
                                                                                              "MCE",
                                                                                              "STREAMIN",
                                                                                              "STREAMOUT",
                                                                                              "SEQ_FIXED_RES",
                                                                                              "SEQ_ZERO_RES",
                                                                                              "SEQ_PROC_DEC",
                                                                                              "CU_32",
                                                                                              "SMEM_64K"
                                                                                            };

            uint32_t const cSetAndResInputArray[cNumberOfSetAndResTypes] = { 2,  /* "APEX",          */   /* Number of Set instances */
                                                                             2,  /* "APU",           */   /* Number of Res instances for the Set type */
                                                                             8,  /* "DMA",           */
                                                                             2,  /* "MCE",           */
                                                                             4,  /* "STREAMIN",      */
                                                                             2,  /* "STREAMOUT",     */
                                                                            20,  /* "SEQ_FIXED_RES", */
                                                                             4,  /* "SEQ_ZERO_RES",  */
                                                                            16,  /* "SEQ_PROC_DEC",  */
                                                                             2,  /* "CU_32",         */
                                                                             2   /* "SMEM _64K"      */
                                                                           };

            uint32_t setAndResOutputArray[cNumberOfSetAndResTypes];


            libResult = ResMgr_Set_Register((const char*)cSetAndResName,
                                            cSetAndResMaxNameLength,
                                            cSetAndResInputArray,
                                            setAndResOutputArray,
                                            cNumberOfSetAndResTypes); SHOW_ERROR(libResult == LIB_SUCCESS);
            if(libResult == LIB_SUCCESS)
            {
               #define cNumberOfUserNames 1 /* const uint32_t cNumberOfUserNames = 1; */
               #define cUserMaxNameLength 3 /* const uint32_t cUserMaxNameLength = 3; */

               const char cUserName[cNumberOfUserNames][cUserMaxNameLength+1] = { "ACF" };

               uint32_t userOutputArray[cNumberOfUserNames];

               libResult = ResMgr_User_Type_Register((const char*)cUserName,
                                                     cUserMaxNameLength,
                                                     userOutputArray,
                                                     cNumberOfUserNames); SHOW_ERROR(libResult == LIB_SUCCESS);

               if(libResult == LIB_SUCCESS)
               {
                  uint32_t userId = 0;

                  libResult = ResMgr_User_Allocate(userOutputArray[0],
                                                   &userId); SHOW_ERROR(libResult == LIB_SUCCESS);

                  if(libResult == LIB_SUCCESS)
                  {
                     const uint32_t cSetTypeId = setAndResOutputArray[0];

                     #define cResAllocInputArraySize8 2
                     #define cNumberOfResToAllocate8  2

                     uint32_t resAllocInputArray[cResAllocInputArraySize8][4] = { { cSetTypeId, 0, setAndResOutputArray[5+1], 0 },
                                                                                 { cSetTypeId, 0, setAndResOutputArray[7+1], 0  }
                                                                               };

                     uint32_t resAllocOutputArray[cNumberOfResToAllocate8][5];

                     memset(resAllocOutputArray, 0xFF, sizeof(resAllocOutputArray));

                     libResult = ResMgr_Resource_Allocate_II(userId,
                                                             resAllocInputArray,
                                                             cResAllocInputArraySize8,
                                                             resAllocOutputArray,
                                                             cNumberOfResToAllocate8); SHOW_ERROR(libResult == LIB_SUCCESS);

                     if(libResult == LIB_SUCCESS)
                     {
                        libResult = ResMgr_Resource_Deallocate(userId,
                                                               resAllocOutputArray,
                                                               cNumberOfResToAllocate8); SHOW_ERROR(libResult == LIB_SUCCESS);
                     }

                     libResult = ResMgr_User_Deallocate(userId); SHOW_ERROR(libResult == LIB_SUCCESS);
                  }
               }
            }

            libResult = ResMgr_Deinitialize(); SHOW_ERROR(libResult == LIB_SUCCESS);
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



/* Initialize */
/* Set Registration */
/* User Registration */
/* Multi-threaded */
/*    User Allocation   */
/*    Resource Allocation */
/*    Resource Deallocation */
/*    User Deallocation   */
struct ResourceManager_h__Test_009_Thread_Input
{
   uint32_t   mThreadId;
   uint32_t   mNumberOfIterations;
   struct ResourceManager* mpResMgr;
   uint32_t   mUserType;
   uint32_t*  mpSetAndResOutputArray;
   uint32_t   mNumberOfSetAndResTypes;
   LIB_RESULT mLibResult;
};

uint32_t gResourceManager_h_Test_009_Iteration[9];
uint32_t gResourceManager_h_Test_009_Trace[9];
uint32_t gResourceManager_h_Test_009_UserId[9];

void* ResourceManager_h_Test_009_Thread_Entry_Point(void* pVoid)
{
   LIB_RESULT libResult    = LIB_SUCCESS;
   LIB_RESULT libIntResult = LIB_FAILURE;

   struct ResourceManager_h__Test_009_Thread_Input* pInput = (struct ResourceManager_h__Test_009_Thread_Input*)pVoid;

   gResourceManager_h_Test_009_Trace[pInput->mThreadId] = __LINE__;

   uint32_t iterationCounter = pInput->mNumberOfIterations;

   while(iterationCounter > 0 &&
         libResult == LIB_SUCCESS)
   {
      uint32_t userId = pInput->mThreadId;

      --iterationCounter;

      gResourceManager_h_Test_009_Iteration[pInput->mThreadId] = iterationCounter;

      gResourceManager_h_Test_009_Trace[pInput->mThreadId] = __LINE__;
      libResult = ResMgr_User_Allocate_Private(pInput->mThreadId,
                                               pInput->mpResMgr,
                                               pInput->mUserType,
                                               &userId); SHOW_ERROR(libResult == LIB_SUCCESS);
      gResourceManager_h_Test_009_Trace[pInput->mThreadId] = __LINE__;
      if(libResult == LIB_SUCCESS)
      {
         gResourceManager_h_Test_009_UserId[pInput->mThreadId] = userId;

         const uint32_t cSetTypeId = pInput->mpSetAndResOutputArray[0];

         #define cResAllocInputArraySize9 2
         #define cNumberOfResToAllocate9  2

         uint32_t resAllocInputArray[cResAllocInputArraySize9][4] = { { cSetTypeId, 0, pInput->mpSetAndResOutputArray[5+1], 0 },
                                                                      { cSetTypeId, 0, pInput->mpSetAndResOutputArray[7+1], 0 }
                                                                    };

         uint32_t resAllocOutputArray[cNumberOfResToAllocate9][5];

         memset(resAllocOutputArray, 0xFF, sizeof(resAllocOutputArray));

         gResourceManager_h_Test_009_Trace[pInput->mThreadId] = __LINE__;
         libResult = ResMgr_Resource_Allocate_II_Private(pInput->mThreadId,
                                                         pInput->mpResMgr,
                                                         userId,
                                                         resAllocInputArray,
                                                         cResAllocInputArraySize9,
                                                         resAllocOutputArray,
                                                         cNumberOfResToAllocate9); SHOW_ERROR(libResult == LIB_SUCCESS);
         gResourceManager_h_Test_009_Trace[pInput->mThreadId] = __LINE__;

         if(libResult == LIB_SUCCESS)
         {
            usleep(100);

            gResourceManager_h_Test_009_Trace[pInput->mThreadId] = __LINE__;
            libResult = ResMgr_Resource_Deallocate_Private(pInput->mThreadId,
                                                           pInput->mpResMgr,
                                                           userId,
                                                           resAllocOutputArray,
                                                           cNumberOfResToAllocate9); SHOW_ERROR(libResult == LIB_SUCCESS);
            gResourceManager_h_Test_009_Trace[pInput->mThreadId] = __LINE__;
         }

         gResourceManager_h_Test_009_Trace[pInput->mThreadId] = __LINE__;
         libIntResult = ResMgr_User_Deallocate_Private(pInput->mThreadId,
                                                       pInput->mpResMgr,
                                                       userId); SHOW_ERROR(libIntResult == LIB_SUCCESS);
         gResourceManager_h_Test_009_Trace[pInput->mThreadId] = __LINE__;

         if(libResult == LIB_SUCCESS)
         {
            libResult = libIntResult;
         }
      }
   }

   gResourceManager_h_Test_009_Trace[pInput->mThreadId] = __LINE__;

   printf("%d %d %X\n",
          pInput->mThreadId,
          (pInput->mNumberOfIterations - iterationCounter),
          (uint32_t)libResult);

   pInput->mLibResult = libResult;

   gResourceManager_h_Test_009_Trace[pInput->mThreadId] = __LINE__;

   return 0;
}

ResourceManager* gspResourceManager_h_Test_009_ResMgr = 0;

LIB_RESULT ResourceManager_h_Test_009()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult    = LIB_FAILURE;
   LIB_RESULT libIntResult = LIB_FAILURE;

   struct ResourceManager resMgr;

   gspResourceManager_h_Test_009_ResMgr = &resMgr;

   memset(&resMgr, 0, sizeof(struct ResourceManager));

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { { 4,  1,   2}, /* Set */
                                                              {13, 10, 124}, /* Resource */
                                                              { 3,  1,   6}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 64;  /* In this case 10 comes from the ten resource types associated with the "APEX" set type */

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize_Private(&resMgr,
                                         0,
                                         0,
                                         cInitInputArray,
                                         cInitInputArraySize,
                                         cMaxSizeOfResourceAllocateInputArray,
                                         &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize_Private(&resMgr,
                                               pBuffer,
                                               neededSizeInBytes,
                                               cInitInputArray,
                                               cInitInputArraySize,
                                               cMaxSizeOfResourceAllocateInputArray,
                                               &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            #define cNumberOfSetAndResTypes  11          /* 1 Set Type + 10 Resource Types */
            #define cSetAndResMaxNameLength  13          /* const uint32_t cResMaxNameLength = 13; */

            const char cSetAndResName[cNumberOfSetAndResTypes][cSetAndResMaxNameLength+1] = { "APEX",            /* Name of the set */
                                                                                              "APU",             /* Following are the names of the resource types */
                                                                                              "DMA",
                                                                                              "MCE",
                                                                                              "STREAMIN",
                                                                                              "STREAMOUT",
                                                                                              "SEQ_FIXED_RES",
                                                                                              "SEQ_ZERO_RES",
                                                                                              "SEQ_PROC_DEC",
                                                                                              "CU_32",
                                                                                              "SMEM_64K"
                                                                                            };

            uint32_t const cSetAndResInputArray[cNumberOfSetAndResTypes] = { 2,  /* "APEX",          */   /* Number of Set instances */
                                                                             2,  /* "APU",           */   /* Number of Res instances for the Set type */
                                                                             8,  /* "DMA",           */
                                                                             2,  /* "MCE",           */
                                                                             4,  /* "STREAMIN",      */
                                                                             2,  /* "STREAMOUT",     */
                                                                            20,  /* "SEQ_FIXED_RES", */
                                                                             4,  /* "SEQ_ZERO_RES",  */
                                                                            16,  /* "SEQ_PROC_DEC",  */
                                                                             2,  /* "CU_32",         */
                                                                             2   /* "SMEM _64K"      */
                                                                           };

            uint32_t setAndResOutputArray[cNumberOfSetAndResTypes];


            libResult = ResMgr_Set_Register_Private(&resMgr,
                                                    (const char*)cSetAndResName,
                                                    cSetAndResMaxNameLength,
                                                    cSetAndResInputArray,
                                                    setAndResOutputArray,
                                                    cNumberOfSetAndResTypes); SHOW_ERROR(libResult == LIB_SUCCESS);
            if(libResult == LIB_SUCCESS)
            {
               #define cNumberOfUserNames 1 /* const uint32_t cNumberOfUserNames = 1; */
               #define cUserMaxNameLength 3 /* const uint32_t cUserMaxNameLength = 3; */

               const char cUserName[cNumberOfUserNames][cUserMaxNameLength+1] = { "ACF" };

               uint32_t userOutputArray[cNumberOfUserNames];

               libResult = ResMgr_User_Type_Register_Private(&resMgr,
                                                             (const char*)cUserName,
                                                             cUserMaxNameLength,
                                                             userOutputArray,
                                                             cNumberOfUserNames); SHOW_ERROR(libResult == LIB_SUCCESS);

               if(libResult == LIB_SUCCESS)
               {
                  #define cNumberOfThreads 9

                  int intResult = -1;
                  uint32_t i = 0;

                  pthread_t pThreads[cNumberOfThreads];

                  struct ResourceManager_h__Test_009_Thread_Input threadInput[cNumberOfThreads];

                  for(i = 0; i < cNumberOfThreads; ++i)
                  {
                     threadInput[i].mThreadId               = i;
                     threadInput[i].mNumberOfIterations     = 5000;
                     threadInput[i].mpResMgr                = &resMgr;
                     threadInput[i].mUserType               = userOutputArray[0];
                     threadInput[i].mpSetAndResOutputArray  = setAndResOutputArray;
                     threadInput[i].mNumberOfSetAndResTypes = cNumberOfSetAndResTypes;
                     threadInput[i].mLibResult              = LIB_FAILURE;

                     intResult = pthread_create(&(pThreads[i]),
                                                NULL,
                                                ResourceManager_h_Test_009_Thread_Entry_Point,
                                                (void*)&(threadInput[i])); SHOW_ERROR(intResult == 0);
                     if(intResult != 0)
                     {
                        break;
                     }
                  }

                  if(intResult == 0)
                  {
                     for(i = 0; i < cNumberOfThreads; ++i)
                     {
                        intResult = pthread_join(pThreads[i], NULL); SHOW_ERROR(intResult == 0);
                        if(intResult != 0)
                        {
                           break;
                        }
                     }
                  }

                  if(intResult == 0)
                  {
                     for(i = 0; i < cNumberOfThreads; ++i)
                     {
                        if(threadInput[i].mLibResult != LIB_SUCCESS)
                        {
                           libResult = threadInput[i].mLibResult;
                           break;
                        }
                     }
                  }
               }
            }

            libIntResult = ResMgr_Deinitialize_Private(&resMgr); SHOW_ERROR(libIntResult == LIB_SUCCESS);

            if(libResult == LIB_SUCCESS)
            {
               libResult = libIntResult;
            }
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



/* Initialize */
/* Set Registration */
/* User Registration */
/* User Allocation   */
/* Resource Allocation TI*/
/* Resource Deallocation */
/* User Deallocation   */
LIB_RESULT ResourceManager_h_Test_010()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult    = LIB_FAILURE;
   LIB_RESULT libIntResult = LIB_FAILURE;

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { { 4,  1,   2}, /* Set */
                                                              {13, 10, 124}, /* Resource */
                                                              { 3,  1,   6}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 124;

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize(0,
                                 0,
                                 cInitInputArray,
                                 cInitInputArraySize,
                                 cMaxSizeOfResourceAllocateInputArray,
                                 &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize(pBuffer,
                                       neededSizeInBytes,
                                       cInitInputArray,
                                       cInitInputArraySize,
                                       cMaxSizeOfResourceAllocateInputArray,
                                       &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            #define cNumberOfSetAndResTypes  11          /* 1 Set Type + 10 Resource Types */
            #define cSetAndResMaxNameLength  13          /* const uint32_t cResMaxNameLength = 13; */

            const char cSetAndResName[cNumberOfSetAndResTypes][cSetAndResMaxNameLength+1] = { "APEX",            /* Name of the set */
                                                                                              "APU",             /* Following are the names of the resource types */
                                                                                              "DMA",
                                                                                              "MCE",
                                                                                              "STREAMIN",
                                                                                              "STREAMOUT",
                                                                                              "SEQ_FIXED_RES",
                                                                                              "SEQ_ZERO_RES",
                                                                                              "SEQ_PROC_DEC",
                                                                                              "CU_32",
                                                                                              "SMEM_64K"
                                                                                            };

            uint32_t const cSetAndResInputArray[cNumberOfSetAndResTypes] = { 2,  /* "APEX",          */   /* Number of Set instances */
                                                                             2,  /* "APU",           */   /* Number of Res instances for the Set type */
                                                                             8,  /* "DMA",           */
                                                                             2,  /* "MCE",           */
                                                                             4,  /* "STREAMIN",      */
                                                                             2,  /* "STREAMOUT",     */
                                                                            20,  /* "SEQ_FIXED_RES", */
                                                                             4,  /* "SEQ_ZERO_RES",  */
                                                                            16,  /* "SEQ_PROC_DEC",  */
                                                                             2,  /* "CU_32",         */
                                                                             2   /* "SMEM _64K"      */
                                                                           };

            uint32_t setAndResOutputArray[cNumberOfSetAndResTypes];


            libResult = ResMgr_Set_Register((const char*)cSetAndResName,
                                            cSetAndResMaxNameLength,
                                            cSetAndResInputArray,
                                            setAndResOutputArray,
                                            cNumberOfSetAndResTypes); SHOW_ERROR(libResult == LIB_SUCCESS);
            if(libResult == LIB_SUCCESS)
            {
               #define cNumberOfUserNames 1 /* const uint32_t cNumberOfUserNames = 1; */
               #define cUserMaxNameLength 3 /* const uint32_t cUserMaxNameLength = 3; */

               const char cUserName[cNumberOfUserNames][cUserMaxNameLength+1] = { "ACF" };

               uint32_t userOutputArray[cNumberOfUserNames];

               libResult = ResMgr_User_Type_Register((const char*)cUserName,
                                                     cUserMaxNameLength,
                                                     userOutputArray,
                                                     cNumberOfUserNames); SHOW_ERROR(libResult == LIB_SUCCESS);

               if(libResult == LIB_SUCCESS)
               {
                  uint32_t userId = 0;

                  libResult = ResMgr_User_Allocate(userOutputArray[0],
                                                   &userId); SHOW_ERROR(libResult == LIB_SUCCESS);

                  if(libResult == LIB_SUCCESS)
                  {
                     const uint32_t cSetTypeId = setAndResOutputArray[0];

                     #define cResAllocInputArraySizeTI 6
                     #define cNumberOfResToAllocateTI  6

                     uint32_t resAllocInputArray[cResAllocInputArraySizeTI][4] = { { cSetTypeId, 0, setAndResOutputArray[5+1], 19 },
                                                                                   { cSetTypeId, 0, setAndResOutputArray[7+1], 15 },
                                                                                   { cSetTypeId, 0, setAndResOutputArray[5+1], 10 },
                                                                                   { cSetTypeId, 0, setAndResOutputArray[7+1], 8 },
                                                                                   { cSetTypeId, 0, setAndResOutputArray[5+1], 0 },
                                                                                   { cSetTypeId, 0, setAndResOutputArray[7+1], 0 }
                                                                                 };

                     uint32_t resAllocOutputArray[cNumberOfResToAllocateTI][5];

                     memset(resAllocOutputArray, 0xFF, sizeof(resAllocOutputArray));

                     libResult = ResMgr_Resource_Allocate_II(userId,
                                                             resAllocInputArray,
                                                             cResAllocInputArraySizeTI,
                                                             resAllocOutputArray,
                                                             cNumberOfResToAllocateTI); SHOW_ERROR(libResult == LIB_SUCCESS);

                     if(libResult == LIB_SUCCESS)
                     {
                        libResult = ResMgr_Resource_Deallocate(userId,
                                                               resAllocOutputArray,
                                                               cNumberOfResToAllocateTI); SHOW_ERROR(libResult == LIB_SUCCESS);
                     }

                     libIntResult = ResMgr_User_Deallocate(userId); SHOW_ERROR(libIntResult == LIB_SUCCESS);

                     if(libResult == LIB_SUCCESS)
                     {
                        libResult = libIntResult;
                     }
                  }
               }
            }

            libResult = ResMgr_Deinitialize(); SHOW_ERROR(libResult == LIB_SUCCESS);
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



void* ResourceManager_h_Test_011_Thread_Entry_Point(void* pVoid)
{
   LIB_RESULT libResult    = LIB_SUCCESS;
   LIB_RESULT libIntResult = LIB_FAILURE;

   struct ResourceManager_h__Test_009_Thread_Input* pInput = (struct ResourceManager_h__Test_009_Thread_Input*)pVoid;

   uint32_t iterationCounter = pInput->mNumberOfIterations;

   while(iterationCounter > 0 &&
         libResult == LIB_SUCCESS)
   {
      uint32_t userId = pInput->mThreadId;

      --iterationCounter;

      libResult = ResMgr_User_Allocate_Private(pInput->mThreadId,
                                               pInput->mpResMgr,
                                               pInput->mUserType,
                                               &userId); SHOW_ERROR(libResult == LIB_SUCCESS);
      if(libResult == LIB_SUCCESS)
      {
         const uint32_t cSetTypeId = pInput->mpSetAndResOutputArray[0];

         #define cResAllocInputArraySize11 5
         #define cNumberOfResToAllocate11  5

         uint32_t resAllocInputArray[cResAllocInputArraySize11][4] = { { cSetTypeId, 0, pInput->mpSetAndResOutputArray[0+1], 0 },
                                                                       { cSetTypeId, 0, pInput->mpSetAndResOutputArray[1+1], 0 },
                                                                       { cSetTypeId, 0, pInput->mpSetAndResOutputArray[1+1], 1 },
                                                                       { cSetTypeId, 0, pInput->mpSetAndResOutputArray[2+1], 0 },
                                                                       { cSetTypeId, 0, pInput->mpSetAndResOutputArray[2+1], 1 }
                                                                     };

         uint32_t resAllocOutputArray[cNumberOfResToAllocate11][5];

         memset(resAllocOutputArray, 0xFF, sizeof(resAllocOutputArray));

         libResult = ResMgr_Resource_Allocate_II_Private(pInput->mThreadId,
                                                         pInput->mpResMgr,
                                                         userId,
                                                         resAllocInputArray,
                                                         cResAllocInputArraySize11,
                                                         resAllocOutputArray,
                                                         cNumberOfResToAllocate11);SHOW_ERROR(libResult == LIB_SUCCESS);

         if(libResult == LIB_SUCCESS)
         {
            usleep(100);

            libResult = ResMgr_Resource_Deallocate_Private(pInput->mThreadId,
                                                           pInput->mpResMgr,
                                                           userId,
                                                           resAllocOutputArray,
                                                           cNumberOfResToAllocate11); SHOW_ERROR(libResult == LIB_SUCCESS);
         }

         libIntResult = ResMgr_User_Deallocate_Private(pInput->mThreadId,
                                                       pInput->mpResMgr,
                                                       userId); SHOW_ERROR(libIntResult == LIB_SUCCESS);

         if(libResult == LIB_SUCCESS)
         {
            libResult = libIntResult;
         }
      }
   }

   printf("%d %d %X\n",
          pInput->mThreadId,
          (pInput->mNumberOfIterations - iterationCounter),
          (uint32_t)libResult);

   pInput->mLibResult = libResult;

   return 0;
}


LIB_RESULT ResourceManager_h_Test_011()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult    = LIB_FAILURE;
   LIB_RESULT libIntResult = LIB_FAILURE;

   struct ResourceManager resMgr;

   memset(&resMgr, 0, sizeof(struct ResourceManager));

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { {20,  1,   1}, /* Set */
                                                              {20,  3,   6}, /* Resource */
                                                              {20,  1,   8}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 6;  /* In this case 10 comes from the ten resource types associated with the "APEX" set type */

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize_Private(&resMgr,
                                         0,
                                         0,
                                         cInitInputArray,
                                         cInitInputArraySize,
                                         cMaxSizeOfResourceAllocateInputArray,
                                         &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize_Private(&resMgr,
                                               pBuffer,
                                               neededSizeInBytes,
                                               cInitInputArray,
                                               cInitInputArraySize,
                                               cMaxSizeOfResourceAllocateInputArray,
                                               &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            #define cNumberOfSetAndResTypes11  4          /* 1 Set Type + 10 Resource Types */
            #define cSetAndResMaxNameLength11  8          /* const uint32_t cResMaxNameLength = 13; */

            const char cSetAndResName[cNumberOfSetAndResTypes11][cSetAndResMaxNameLength11+1] = { "APEX",            /* Name of the set */
                                                                                                  "APU",             /* Following are the names of the resource types */
                                                                                                  "CU_32",
                                                                                                  "SMEM_64K"
                                                                                                };

            uint32_t const cSetAndResInputArray[cNumberOfSetAndResTypes11] = { 1,  /* "APEX",          */   /* Number of Set instances */
                                                                             2,  /* "APU",           */   /* Number of Res instances for the Set type */
                                                                             2,  /* "CU_32",           */
                                                                             2   /* SMEM_64K */
                                                                           };

            uint32_t setAndResOutputArray[cNumberOfSetAndResTypes11];


            libResult = ResMgr_Set_Register_Private(&resMgr,
                                                    (const char*)cSetAndResName,
                                                    cSetAndResMaxNameLength11,
                                                    cSetAndResInputArray,
                                                    setAndResOutputArray,
                                                    cNumberOfSetAndResTypes11); SHOW_ERROR(libResult == LIB_SUCCESS);
            if(libResult == LIB_SUCCESS)
            {
               #define cNumberOfUserNames 1 /* const uint32_t cNumberOfUserNames = 1; */
               #define cUserMaxNameLength 3 /* const uint32_t cUserMaxNameLength = 3; */

               const char cUserName[cNumberOfUserNames][cUserMaxNameLength+1] = { "ACF" };

               uint32_t userOutputArray[cNumberOfUserNames];

               libResult = ResMgr_User_Type_Register_Private(&resMgr,
                                                             (const char*)cUserName,
                                                             cUserMaxNameLength,
                                                             userOutputArray,
                                                             cNumberOfUserNames); SHOW_ERROR(libResult == LIB_SUCCESS);

               if(libResult == LIB_SUCCESS)
               {
                  #define cNumberOfThreads11 9

                  int intResult = -1;
                  uint32_t i = 0;

                  pthread_t pThreads[cNumberOfThreads11];

                  struct ResourceManager_h__Test_009_Thread_Input threadInput[cNumberOfThreads11];

                  for(i = 0; i < cNumberOfThreads11; ++i)
                  {
                     threadInput[i].mThreadId               = i;
                     threadInput[i].mNumberOfIterations     = 500;
                     threadInput[i].mpResMgr                = &resMgr;
                     threadInput[i].mUserType               = userOutputArray[0];
                     threadInput[i].mpSetAndResOutputArray  = setAndResOutputArray;
                     threadInput[i].mNumberOfSetAndResTypes = cNumberOfSetAndResTypes11;
                     threadInput[i].mLibResult              = LIB_FAILURE;

                     intResult = pthread_create(&(pThreads[i]),
                                                NULL,
                                                ResourceManager_h_Test_011_Thread_Entry_Point,
                                                (void*)&(threadInput[i])); SHOW_ERROR(intResult == 0);
                     if(intResult != 0)
                     {
                        break;
                     }
                  }

                  if(intResult == 0)
                  {
                     for(i = 0; i < cNumberOfThreads11; ++i)
                     {
                        intResult = pthread_join(pThreads[i], NULL); SHOW_ERROR(intResult == 0);
                        if(intResult != 0)
                        {
                           break;
                        }
                     }
                  }

                  if(intResult == 0)
                  {
                     for(i = 0; i < cNumberOfThreads11; ++i)
                     {
                        if(threadInput[i].mLibResult != LIB_SUCCESS)
                        {
                           libResult = threadInput[i].mLibResult;
                           break;
                        }
                     }
                  }
               }
            }

            libIntResult = ResMgr_Deinitialize_Private(&resMgr); SHOW_ERROR(libIntResult == LIB_SUCCESS);

            if(libResult == LIB_SUCCESS)
            {
               libResult = libIntResult;
            }
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



/* Initialize */
/* Set Registration */
/* User Registration */
/* Multi-threaded */
/*    User Allocation   */
/*    Resource Allocation */
/*    Scanning of the Queue */
/*    Resource Deallocation */
/*    User Deallocation   */
#define gcTest_012_NumberOfResPerThread 2

struct ResourceManager_h__Test_012_Thread_Input
{
   uint32_t   mThreadId;
   struct ResourceManager* mpResMgr;
   uint32_t   mUserType;
   uint32_t*  mpSetAndResOutputArray;
   uint32_t   mNumberOfSetAndResTypes;
   uint32_t   mResAllocInputArray[gcTest_012_NumberOfResPerThread][4];
   uint32_t   mResAllocOutputArray[gcTest_012_NumberOfResPerThread][5];
   LIB_RESULT mLibResult;
};



void* ResourceManager_h_Test_012_Thread_Entry_Point(void* pVoid)
{
   LIB_RESULT libResult    = LIB_SUCCESS;
   LIB_RESULT libIntResult = LIB_FAILURE;

   struct ResourceManager_h__Test_012_Thread_Input* pInput = (struct ResourceManager_h__Test_012_Thread_Input*)pVoid;

   uint32_t userId = pInput->mThreadId;

   libResult = ResMgr_User_Allocate_Private(pInput->mThreadId,
                                            pInput->mpResMgr,
                                            pInput->mUserType,
                                            &userId); SHOW_ERROR(libResult == LIB_SUCCESS);
   if(libResult == LIB_SUCCESS)
   {
      uint32_t resAllocOutputArray[gcTest_012_NumberOfResPerThread][5];

      memset(resAllocOutputArray, 0xFF, sizeof(resAllocOutputArray));

      libResult = ResMgr_Resource_Allocate_II_Private(pInput->mThreadId,
                                                      pInput->mpResMgr,
                                                      userId,
                                                      pInput->mResAllocInputArray,
                                                      gcTest_012_NumberOfResPerThread,
                                                      resAllocOutputArray,
                                                      gcTest_012_NumberOfResPerThread);SHOW_ERROR(libResult == LIB_SUCCESS);
      if(libResult == LIB_SUCCESS)
      {
         usleep(500*1000);

         libResult = ResMgr_Resource_Deallocate_Private(pInput->mThreadId,
                                                        pInput->mpResMgr,
                                                        userId,
                                                        resAllocOutputArray,
                                                        gcTest_012_NumberOfResPerThread); SHOW_ERROR(libResult == LIB_SUCCESS);
      }


      libIntResult = ResMgr_User_Deallocate_Private(pInput->mThreadId,
                                                    pInput->mpResMgr,
                                                    userId); SHOW_ERROR(libIntResult == LIB_SUCCESS);

      if(libResult == LIB_SUCCESS)
      {
         libResult = libIntResult;
      }
   }

   printf("Thread [%d] Result[%X]\n",
          pInput->mThreadId,
          (uint32_t)libResult);

   pInput->mLibResult = libResult;

   return 0;
}


LIB_RESULT ResourceManager_h_Test_012()
{
   printf("%s\n", __PRETTY_FUNCTION__);
   LIB_RESULT libResult    = LIB_FAILURE;
   LIB_RESULT libIntResult = LIB_FAILURE;

   struct ResourceManager resMgr;

   memset(&resMgr, 0, sizeof(struct ResourceManager));

   /* 0 is Max <Object> Name Length
    * 1 is Max Number of <Object> Types
    * 2 is Max Number of <Object>
    */

   #define cInitInputArraySize 3 /* uint32_t const cInitInputArraySize = 3; */

   uint32_t const cInitInputArray[cInitInputArraySize][3] = { { 4,  1,   2}, /* Set */
                                                              {13, 10, 124}, /* Resource */
                                                              { 3,  1,   6}  /* User */
                                                            };
   uint32_t const cMaxSizeOfResourceAllocateInputArray = 64;  /* In this case 10 comes from the ten resource types associated with the "APEX" set type */

   uint32_t neededSizeInBytes = 0;

   /*
    * Get the neededSizeInBytes
    */
   libResult = ResMgr_Initialize_Private(&resMgr,
                                         0,
                                         0,
                                         cInitInputArray,
                                         cInitInputArraySize,
                                         cMaxSizeOfResourceAllocateInputArray,
                                         &neededSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);

   if(libResult == LIB_SUCCESS &&
      neededSizeInBytes > 0)
   {
      void* pBuffer = malloc(neededSizeInBytes);

      if(pBuffer != 0)
      {
         uint32_t usedSizeInBytes = 0;

         /* */
         /* Actual initialization */
         /* */
         libResult = ResMgr_Initialize_Private(&resMgr,
                                               pBuffer,
                                               neededSizeInBytes,
                                               cInitInputArray,
                                               cInitInputArraySize,
                                               cMaxSizeOfResourceAllocateInputArray,
                                               &usedSizeInBytes); SHOW_ERROR(libResult == LIB_SUCCESS);
         if(libResult == LIB_SUCCESS)
         {
            #define cTest_012_NumberOfSetAndResTypes  4           /* 1 Set Type + 3 Resource Types */
            #define cTest_012_SetAndResMaxNameLength  13          /* const uint32_t cResMaxNameLength = 13; */

            const char cSetAndResName[cTest_012_NumberOfSetAndResTypes][cTest_012_SetAndResMaxNameLength+1] = { "APEX",            /* Name of the set */
                                                                                              "APU",             /* Following are the names of the resource types */
                                                                                              "DMA",
                                                                                              "MCE"
                                                                                            };

            uint32_t const cSetAndResInputArray[cTest_012_NumberOfSetAndResTypes] = { 2,  /* "APEX",          */   /* Number of Set instances */
                                                                             2,  /* "APU",           */   /* Number of Res instances for the Set type */
                                                                             8,  /* "DMA",           */
                                                                             2,  /* "MCE",           */
                                                                           };

            uint32_t setAndResOutputArray[cTest_012_NumberOfSetAndResTypes];


            libResult = ResMgr_Set_Register_Private(&resMgr,
                                                    (const char*)cSetAndResName,
                                                    cTest_012_SetAndResMaxNameLength,
                                                    cSetAndResInputArray,
                                                    setAndResOutputArray,
                                                    cTest_012_NumberOfSetAndResTypes); SHOW_ERROR(libResult == LIB_SUCCESS);
            if(libResult == LIB_SUCCESS)
            {
               #define cNumberOfUserNames 1 /* const uint32_t cNumberOfUserNames = 1; */
               #define cUserMaxNameLength 3 /* const uint32_t cUserMaxNameLength = 3; */

               const char cUserName[cNumberOfUserNames][cUserMaxNameLength+1] = { "ACF" };

               uint32_t userOutputArray[cNumberOfUserNames];

               libResult = ResMgr_User_Type_Register_Private(&resMgr,
                                                             (const char*)cUserName,
                                                             cUserMaxNameLength,
                                                             userOutputArray,
                                                             cNumberOfUserNames); SHOW_ERROR(libResult == LIB_SUCCESS);

               if(libResult == LIB_SUCCESS)
               {
                  #define cTest_012_NumberOfThreads 4

                  int intResult = -1;
                  uint32_t i = 0;

                  pthread_t pThreads[cTest_012_NumberOfThreads];

                  uint32_t userId = 0;

                  struct ResourceManager_h__Test_012_Thread_Input threadInput[cNumberOfThreads];

                  /*
                   * Allocate resources which will block some of the threads
                   */
                  threadInput[i].mResAllocInputArray[0][0] = 0;
                  threadInput[i].mResAllocInputArray[0][1] = 0;   // APEX 0
                  threadInput[i].mResAllocInputArray[0][2] = 0;
                  threadInput[i].mResAllocInputArray[0][3] = 0;   // APU 0

                  threadInput[i].mResAllocInputArray[1][0] = 0;
                  threadInput[i].mResAllocInputArray[1][1] = 0;   // APEX 0
                  threadInput[i].mResAllocInputArray[1][2] = 1;
                  threadInput[i].mResAllocInputArray[1][3] = 0;   // DMA 0

                  libResult = ResMgr_User_Allocate_Private(0,
                                                           &resMgr,
                                                           userOutputArray[0],
                                                           &userId); SHOW_ERROR(libResult == LIB_SUCCESS);

                  uint32_t resAllocOutputArray[gcTest_012_NumberOfResPerThread][5];

                  memset(resAllocOutputArray, 0xFF, sizeof(resAllocOutputArray));

                  libResult = ResMgr_Resource_Allocate_II_Private(0,
                                                                  &resMgr,
                                                                  userId,
                                                                  threadInput[0].mResAllocInputArray,
                                                                  gcTest_012_NumberOfResPerThread,
                                                                  resAllocOutputArray,
                                                                  gcTest_012_NumberOfResPerThread);SHOW_ERROR(libResult == LIB_SUCCESS);

                  /*
                   * Create other threads which will compete for the same resources
                   */
                  for(i = 1; i < cTest_012_NumberOfThreads; ++i)
                  {
                     threadInput[i].mThreadId               = i;
                     threadInput[i].mpResMgr                = &resMgr;
                     threadInput[i].mUserType               = userOutputArray[0];
                     threadInput[i].mpSetAndResOutputArray  = setAndResOutputArray;
                     threadInput[i].mNumberOfSetAndResTypes = cNumberOfSetAndResTypes;
                     threadInput[i].mLibResult              = LIB_FAILURE;

                     if(i == 1)
                     {
                        threadInput[i].mResAllocInputArray[0][0] = 0;
                        threadInput[i].mResAllocInputArray[0][1] = 0;   // APEX 0
                        threadInput[i].mResAllocInputArray[0][2] = 0;
                        threadInput[i].mResAllocInputArray[0][3] = 1;   // APU 1

                        threadInput[i].mResAllocInputArray[1][0] = 0;
                        threadInput[i].mResAllocInputArray[1][1] = 0;   // APEX 0
                        threadInput[i].mResAllocInputArray[1][2] = 1;
                        threadInput[i].mResAllocInputArray[1][3] = 0;   // DMA 0
                     }
                     else if(i == 2)
                     {
                        threadInput[i].mResAllocInputArray[0][0] = 0;
                        threadInput[i].mResAllocInputArray[0][1] = 0;   // APEX 0
                        threadInput[i].mResAllocInputArray[0][2] = 0;
                        threadInput[i].mResAllocInputArray[0][3] = 0;   // APU 0

                        threadInput[i].mResAllocInputArray[1][0] = 0;
                        threadInput[i].mResAllocInputArray[1][1] = 0;   // APEX 0
                        threadInput[i].mResAllocInputArray[1][2] = 1;
                        threadInput[i].mResAllocInputArray[1][3] = 0;   // DMA 0
                     }
                     else if(i == 3)
                     {
                        threadInput[i].mResAllocInputArray[0][0] = 0;
                        threadInput[i].mResAllocInputArray[0][1] = 1;   // APEX 1
                        threadInput[i].mResAllocInputArray[0][2] = 0;
                        threadInput[i].mResAllocInputArray[0][3] = 0;   // APU 0

                        threadInput[i].mResAllocInputArray[1][0] = 0;
                        threadInput[i].mResAllocInputArray[1][1] = 1;   // APEX 1
                        threadInput[i].mResAllocInputArray[1][2] = 1;
                        threadInput[i].mResAllocInputArray[1][3] = 0;   // DMA 0
                     }

                     intResult = pthread_create(&(pThreads[i]),
                                                NULL,
                                                ResourceManager_h_Test_012_Thread_Entry_Point,
                                                (void*)&(threadInput[i])); SHOW_ERROR(intResult == 0);
                     if(intResult != 0)
                     {
                        break;
                     }
                  }

                  if(libResult == LIB_SUCCESS)
                  {
                     usleep(cTest_012_NumberOfThreads*500*1000);

                     libResult = ResMgr_Resource_Deallocate_Private(0,
                                                                    &resMgr,
                                                                    userId,
                                                                    resAllocOutputArray,
                                                                    gcTest_012_NumberOfResPerThread); SHOW_ERROR(libResult == LIB_SUCCESS);
                  }


                  libIntResult = ResMgr_User_Deallocate_Private(0,
                                                                &resMgr,
                                                                userId); SHOW_ERROR(libIntResult == LIB_SUCCESS);

                  if(intResult == 0)
                  {
                     for(i = 1; i < cTest_012_NumberOfThreads; ++i)
                     {
                        intResult = pthread_join(pThreads[i], NULL); SHOW_ERROR(intResult == 0);

                        if(intResult != 0)
                        {
                           break;
                        }
                     }
                  }

                  if(intResult == 0)
                  {
                     for(i = 1; i < cTest_012_NumberOfThreads; ++i)
                     {
                        if(threadInput[i].mLibResult != LIB_SUCCESS)
                        {
                           libResult = threadInput[i].mLibResult;
                           break;
                        }
                     }
                  }
               }
            }

            libIntResult = ResMgr_Deinitialize_Private(&resMgr); SHOW_ERROR(libIntResult == LIB_SUCCESS);

            if(libResult == LIB_SUCCESS)
            {
               libResult = libIntResult;
            }
         }

         free(pBuffer);
         pBuffer = 0;
      }
   }

   return libResult;
}



void ResourceManager_h_Tests()
{
   LIB_RESULT libResult = LIB_SUCCESS;

   libResult = ResourceManager_h_Test_000(); SHOW_ERROR(libResult == LIB_SUCCESS);
   
   libResult = ResourceManager_h_Test_001(); SHOW_ERROR(libResult == LIB_SUCCESS);
   libResult = ResourceManager_h_Test_002(); SHOW_ERROR(libResult == LIB_SUCCESS);
   libResult = ResourceManager_h_Test_003(); SHOW_ERROR(libResult == LIB_SUCCESS);
   libResult = ResourceManager_h_Test_004(); SHOW_ERROR(libResult == LIB_SUCCESS);
   libResult = ResourceManager_h_Test_005(); SHOW_ERROR(libResult == LIB_SUCCESS);
   libResult = ResourceManager_h_Test_006(); SHOW_ERROR(libResult == LIB_SUCCESS);
   libResult = ResourceManager_h_Test_007(); SHOW_ERROR(libResult == LIB_SUCCESS);
   libResult = ResourceManager_h_Test_008(); SHOW_ERROR(libResult == LIB_SUCCESS);

   libResult = ResourceManager_h_Test_009(); SHOW_ERROR(libResult == LIB_SUCCESS);
   libResult = ResourceManager_h_Test_010(); SHOW_ERROR(libResult == LIB_SUCCESS);
   libResult = ResourceManager_h_Test_011(); SHOW_ERROR(libResult == LIB_SUCCESS);
   libResult = ResourceManager_h_Test_012(); SHOW_ERROR(libResult == LIB_SUCCESS);
}

