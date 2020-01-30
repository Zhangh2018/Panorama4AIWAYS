/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2018 Freescale Semiconductor
* Copyright 2018 NXP 
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

#ifndef APU2KERNELDBTYPES_H
#define APU2KERNELDBTYPES_H

#if __chess__
#define KNDECL static
#else
#define KNDECL
#endif


/**
 *  \file APU2_Kernel_DB.h
 */

// forward declaration of KERNEL_INFO, KERNEL_HANDLE
//class KERNEL_INFO;

/////////////////////////////////////////////////////////////////////////
/// \addtogroup apu2_kernel_db_api
/// @{
///
/// \section intro_sec Introduction
///
/// This is the APU2 kernel database parser library.
///
///
//////////////////////////////////////////////////////////////////////////

//NOTE: these sizes are chosen  to be 'reasonable' but can be adjusted as required
#define MAX_KERNEL  512
#define MAX_PORT    32
#define MIN_PORT    2
const int PORT_IDENTIFIER_LENGTH   = 64;
const int KERNEL_IDENTIFIER_LENGTH = 64;

// Kernel DB error values
const int KERNEL_DB_ERROR_BASE=64;
const int KERNEL_DB_DUPLICATE_NAME = KERNEL_DB_ERROR_BASE+1;
const int KERNEL_DB_TOO_MANY_PORTS = KERNEL_DB_ERROR_BASE+2;
const int KERNEL_DB_OUTPUT_ERROR   = KERNEL_DB_ERROR_BASE+3;


//--------------------------------------------------------------------------
// KERNEL DATABASE
//--------------------------------------------------------------------------
#include <string.h>
#include <stdint.h>
#include <map>
#include <string>
#include <vector>


//--------------------------
// KERNEL DATABASE SUPPORT
//--------------------------

// The below is used in KERNEL_INFO static declarations.
#define __port(index, identifier, attributes, spatial_dep, e0_data_type, e0_size, ek_size) \
               PORT_STRUCT{index, identifier, attributes, spatial_dep, e0_data_type, e0_size, ek_size}

#define __index(index)            index
#define __identifier(id)          id
#define __attributes(attr)        attr
#define __spatial_dep(l, r, t, b) {l, r, t, b}
#define __e0_data_type(dtype)     dtype
#define __e0_size(x_size, y_size) {x_size, y_size}
#define __ek_size(x_size, y_size) {x_size, y_size}

#define __port_0                  PORT_STRUCT{0,"null",0,{0,0,0,0},0,{0,0},{0,0}}
#define __spatial_dep_0           {0, 0, 0, 0}

//-------------------
//port attributes
//-------------------
enum
{
   //input
   //output
   ATTR_IN  = 0 << 0,
   ATTR_OUT = 1 << 0,

   //cmem
   //dmem
   ATTR_CMEM = 0 << 1,
   ATTR_DMEM = 1 << 1,

   //nonstatic
   //static
   ATTR_NONSTATIC = 0 << 2,
   ATTR_STATIC    = 1 << 2,

   //nonfixed: chunk size is regular (i.e. same for all iterations and/or processors) but scalable
   //fixed: chunk size is regular (i.e. same for all iterations and/or processors) but NOT scalable
   //      (e.g. kernel only accepts input size of 8x8 due to implementation details))
   ATTR_NONFIXED = 0 << 3,
   ATTR_FIXED    = 1 << 3,

   //NOTE: the following attribute may eventually be more than 1 bit to express other corner
   //      type cases that are harder to classify/express!
   //reg: chunk size is regular and defined by e0 & ek (i.e. same for all iterations and/or processors)
   //fifo: chunk size is variable and e0 & ek define 'worst case' chunk size (i.e. between iterations
   //      or processors the chunk size may vary between 0 and the worst case).  also, data output
   //      becomes 1D and the order is not guaranteed (i.e. output resembles a fifo).  this should be
   //      used when each CU is producing a variable size 1D output and the user wishes to collect/reduce
   //      all sub-lists to a single 1D final list in external memory.
   ATTR_REG  = 0 << 4,
   ATTR_FIFO = 1 << 4
};

//----------------------------------------------------------------------------------
//predefined attribute macros that hide/abstract core attribute combinations beneath
//generalized concepts
//----------------------------------------------------------------------------------

//****************
//vector I/O types
//****************

//input
#define ACF_ATTR_VEC_IN               (ATTR_IN |ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED)
#define ACF_ATTR_VEC_IN_FIXED         (ATTR_IN |ATTR_CMEM|ATTR_NONSTATIC|ATTR_FIXED)
#define ACF_ATTR_VEC_IN_STATIC        (ATTR_IN |ATTR_CMEM|ATTR_STATIC   |ATTR_NONFIXED)
#define ACF_ATTR_VEC_IN_STATIC_FIXED  (ATTR_IN |ATTR_CMEM|ATTR_STATIC   |ATTR_FIXED)

//output
#define ACF_ATTR_VEC_OUT              (ATTR_OUT|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED)
#define ACF_ATTR_VEC_OUT_FIXED        (ATTR_OUT|ATTR_CMEM|ATTR_NONSTATIC|ATTR_FIXED)
#define ACF_ATTR_VEC_OUT_STATIC       (ATTR_OUT|ATTR_CMEM|ATTR_STATIC   |ATTR_NONFIXED)
#define ACF_ATTR_VEC_OUT_STATIC_FIXED (ATTR_OUT|ATTR_CMEM|ATTR_STATIC   |ATTR_FIXED)
#define ACF_ATTR_VEC_OUT_FIFO         (ATTR_OUT|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_FIFO)
#define ACF_ATTR_VEC_OUT_FIFO_FIXED   (ATTR_OUT|ATTR_CMEM|ATTR_NONSTATIC|ATTR_FIXED|ATTR_FIFO)

//****************
//scalar I/O types
//****************

//input
#define ACF_ATTR_SCL_IN               (ATTR_IN |ATTR_DMEM|ATTR_NONSTATIC|ATTR_NONFIXED)
#define ACF_ATTR_SCL_IN_FIXED         (ATTR_IN |ATTR_DMEM|ATTR_NONSTATIC|ATTR_FIXED)
#define ACF_ATTR_SCL_IN_STATIC        (ATTR_IN |ATTR_DMEM|ATTR_STATIC   |ATTR_NONFIXED)
#define ACF_ATTR_SCL_IN_STATIC_FIXED  (ATTR_IN |ATTR_DMEM|ATTR_STATIC   |ATTR_FIXED)

//output
#define ACF_ATTR_SCL_OUT              (ATTR_OUT|ATTR_DMEM|ATTR_NONSTATIC|ATTR_NONFIXED)
#define ACF_ATTR_SCL_OUT_FIXED        (ATTR_OUT|ATTR_DMEM|ATTR_NONSTATIC|ATTR_FIXED)
#define ACF_ATTR_SCL_OUT_STATIC       (ATTR_OUT|ATTR_DMEM|ATTR_STATIC   |ATTR_NONFIXED)
#define ACF_ATTR_SCL_OUT_STATIC_FIXED (ATTR_OUT|ATTR_DMEM|ATTR_STATIC   |ATTR_FIXED)

/*-------------------------------------------------------------------------------------
- Kernel read only (e.g. reduction kernel input)
  * ACF_ATTR_VEC_IN_STATIC
  * ACF_ATTR_VEC_IN_STATIC_FIXED

- Initialized by framework then kernel read-write (e.g. histogram kernel output)
  * ACF_ATTR_VEC_OUT_STATIC
  * ACF_ATTR_VEC_OUT_STATIC_FIXED

- Used for one-shot global 'initialization'-type input transfers (e.g. filter coefficients)
  * ACF_ATTR_SCL_IN_STATIC
  * ACF_ATTR_SCL_IN_STATIC_FIXED

- Used for one-shot global 'gather/reduction'-type output transfers (e.g. histogram kernel output)
  * ACF_ATTR_SCL_OUT_STATIC
  * ACF_ATTR_SCL_OUT_STATIC_FIXED

- Fifo-like 1D output per CU (e.g. each prc/CU outputs a 1D list of values and
  potentially different number of values for each CU) and these values need to
  be collected and transfered to external memory as a single packed 1D list
  * ACF_ATTR_VEC_OUT_FIFO
--------------------------------------------------------------------------------------*/

//deprecated ...
#define ACF_ATTR_IN                  ACF_ATTR_VEC_IN
#define ACF_ATTR_OUT                 ACF_ATTR_VEC_OUT
#define ACF_ATTR_CFG                 ACF_ATTR_SCL_IN_STATIC_FIXED
#define ACF_ATTR_IN_STATIC_GLB_FIXED ACF_ATTR_SCL_IN_STATIC_FIXED


#define PORT_ATTR_IN(attr)        ((((attr>>0) & 1) == 0) ? 1 : 0)
#define PORT_ATTR_OUT(attr)       ((((attr>>0) & 1) == 1) ? 1 : 0)
#define PORT_ATTR_CMEM(attr)      ((((attr>>1) & 1) == 0) ? 1 : 0)
#define PORT_ATTR_DMEM(attr)      ((((attr>>1) & 1) == 1) ? 1 : 0)
#define PORT_ATTR_NONSTATIC(attr) ((((attr>>2) & 1) == 0) ? 1 : 0)
#define PORT_ATTR_STATIC(attr)    ((((attr>>2) & 1) == 1) ? 1 : 0)
#define PORT_ATTR_NONFIXED(attr)  ((((attr>>3) & 1) == 0) ? 1 : 0)
#define PORT_ATTR_FIXED(attr)     ((((attr>>3) & 1) == 1) ? 1 : 0)
#define PORT_ATTR_REG(attr)       ((((attr>>4) & 1) == 0) ? 1 : 0)
#define PORT_ATTR_FIFO(attr)      ((((attr>>4) & 1) == 1) ? 1 : 0)

typedef enum
{
   d08s = 1,
   d08u,
   d16s,
   d16u,
   d32s,
   d32u,
} DATA_TYPE;

//--------------------------
// PORT INFO
//--------------------------

// NB: this struct is current only used to define a PORT_HANDLE (see APU2_Kernel_DB_Lib.h)
struct _PORT_STRUCT_tag{
   uint32_t    index;
   char        identifier[PORT_IDENTIFIER_LENGTH];
   uint32_t    attributes;
   uint32_t    spatial_dep[4]; //{left, right, top, bottom}
   uint32_t    e0_data_type;
   uint32_t    e0_size[2];
   uint32_t    ek_size[2];
};

typedef _PORT_STRUCT_tag PORT_STRUCT;

// Handle for read-only access to PORT_STRUCT
typedef const PORT_STRUCT* PORT_HANDLE;

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Use the KERNEL_INFO class to specify the kernel intrinsics,
/// by constructing a KERNEL_INFO object for a kernel, it automatically
/// gets registered with static kernel database.
//////////////////////////////////////////////////////////////////////////////////////////////////
class KERNEL_INFO
{
public:
   char           mKernelName[KERNEL_IDENTIFIER_LENGTH];
   uint32_t       mPortCount;
   PORT_STRUCT    mPort[MAX_PORT];

   KERNEL_INFO():mPortCount(0) {};
   /////////////////////////////////////////////////////////////////////////
   /// KERNEL_INFO constructor:
   /// KERNEL_INFO(
   /// "kernel function name", number of ports,
   /// /*1st port*/
   /// port_id,
   /// port_identifier (string)
   /// port_type,
   /// __data_type   (datatype, data_space),
   /// __spatial_dep (x_left, x_right, y_l, y_r),
   /// __element_0   (x_size, y_size),
   /// __element_k   (x_size, y_size),
   /// __access      (direct),
   /// __data_pad    (enable_bit)
   /// /*2nd port*/
   /// ...
   /// );
   ///
   /// \par Description:
   /// per port:
   /// [__port_id]:     port index in the order it appears in the parameters of the kernel function signature
   /// [__port_identifier]: short string which uniquely identifies the port
   /// [__port_type]:   port characteristic, DST(destination), SRC(source), GLB(global), CON (configuration)
   //  [__data_type]:   data type and data address space(DMEM/CMEM), only accepts following data types currently:
   /// d08u, d08s, d16u, d16s, d32u, d32s
   /// [__spatial_dep]: spatial dependencies on x and y directions, relates
   /// [__element_0]:   size of one data unit, for basic data types this will be (1, 1);
   /// [__element_k]:   minimum size (expressed in a 2D array fashion) of element 0 required for the kernel
   /// [__access]:      "direct" indicate data array while "indirect" is a data address array; this is a potentially
   ///                  useful feature if the framework has the intelligence to adjust scan order for optimization;
   /// [__data_pad]:    set "1" to allow the framework to ensure the "input data locality".
   /////////////////////////////////////////////////////////////////////////
   template<typename... Args>
   KERNEL_INFO(const char* kernel_name, int port_count, Args... ports) : mPortVector({ports...}) 
   {
      RegisterKernel(kernel_name, port_count);
   };


   KERNEL_INFO(const KERNEL_INFO &kernel)
   {
      mPortCount = kernel.mPortCount;
      memcpy(mKernelName, kernel.mKernelName, strlen(kernel.mKernelName)+1);
      memcpy(mPort, kernel.mPort, sizeof(PORT_STRUCT)*MAX_PORT);
   }
   KERNEL_INFO& operator= (const KERNEL_INFO &kernel)
   {
      mPortCount = kernel.mPortCount;
      memcpy(mKernelName, kernel.mKernelName, strlen(kernel.mKernelName)+1);
      memcpy(mPort, kernel.mPort, sizeof(PORT_STRUCT)*MAX_PORT);
      return *this;
   };
   ~KERNEL_INFO(){};

private:
   std::vector<PORT_STRUCT> mPortVector;
   int checkForErrors();

   void RegisterKernel(const char *kernel_name, int port_count);
} ;

//////////////////////////////////////////////////////////////////////////
/// First step, compile your APU kernel code with the kernel constructs
/// described below with kernel database support library.
/// To use/access a kernel database, you can either:
/// 1) Generate a APU_Kernel_Database.a, which contains the database object,
/// then later on link your code  against it(standard C++ compilation).
/// or
/// 2) Have the database written into a file, then read from it later on.
/// link your code (standard C++ compilation) against a pre-generated APU
/// Kernel database library.
/// (our old way of database access and should try not to do this way).
/////////////////////////////////////////////////////////////////////////
typedef std::map <std::string, uint32_t> MapType;

class KERNEL_DB{
   friend class KERNEL_INFO;
   MapType     mKernelMap;
   KERNEL_INFO mKernel[MAX_KERNEL];
   uint32_t    mKernelCount;
   uint32_t    mErrorValue;         // 0 if there are no errors encountered building database

public:
   KERNEL_DB():mKernelCount(0), mErrorValue(0)          {};
   int Get_KernelCounter() const             {return mKernelCount;}
   int Get_Error() const                     {return mErrorValue;}
   void Set_Error(int value)                  {mErrorValue = value;}
   int Retrieve_Kernel(const KERNEL_INFO* &kernel_hl, const char* kernel_name);
   //int Get_KernelEntry(int kernel_id) const  {return mKernelEntry[kernel_id];}
   int Write_Datatbase  (const char* load_name);
   //void Read_Database   (const char* load_name);
};

/////////////////////////////////////////////////////////////////////////
/// \par Descriptions:
/// \return The number of kernels registered with this database;
/////////////////////////////////////////////////////////////////////////
int   GET_DATABASECOUNTER  ();

/////////////////////////////////////////////////////////////////////////
/// \par Descriptions:
/// \return The error status of the kernel database.  If no errors
///   were encountered building the database, returns zero (0).
/////////////////////////////////////////////////////////////////////////
int   GET_DATABASE_ERROR  ();

/////////////////////////////////////////////////////////////////////////
/// \par Descriptions:
/// \return The error status of the kernel database.  If no errors
///   were encountered building the database, returns zero (0).
/////////////////////////////////////////////////////////////////////////
void   SET_DATABASE_ERROR  (int value);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the kernel name, query the database and if it exists, return
/// KERNEL_INFO pointer to this kernel.
///
/// \param kernel_hl [output]
/// KERNEL_INFO pointer to the requested kernel.
/// \param kernel_hl [input]
/// Char array that contains the kernel name.
/// \return
/// 0 if successful, -1 if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
int   GET_KERNEL_HANDLE    (const KERNEL_INFO* kernel_hl,  const char* kernel_name);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the APU load file name, write the content of the database into the file.
///
/// \param load_name [input]
/// Char array that contains the load name.
/// \return
/// void
/////////////////////////////////////////////////////////////////////////
void  WRITE_DATABASE2FILE  (const char* load_name);

/// @}



#endif /* APU2KERNELDBTYPES_H */
