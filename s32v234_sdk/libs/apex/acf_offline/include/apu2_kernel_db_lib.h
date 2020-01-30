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

#ifndef APU2KERNELDBLIB_H
#define APU2KERNELDBLIB_H

#include "apu2_kernel_db_types.h"

// Handle for read-only access to KERNEL_INFO
typedef const KERNEL_INFO* KERNEL_HANDLE;

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

//--------------------------------------------------------------------------
// KERNEL DATABASE
//--------------------------------------------------------------------------

//--------------------------
// PORT INFO
//--------------------------

// Handle for read-only access to PORT_STRUCT
typedef const PORT_STRUCT* PORT_HANDLE;

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the kernel handle, and the port number, query the port handle.
///
/// \param kh [input]
/// KERNEL_INFO pointer to the requested kernel.
/// \param portNumber [input]
/// The port number of the port.  Ports are numbers from 0 to N-1, where
/// N is the number of ports in the kernel.
/// \return
/// Returns a value which uniquely identifies the port if successful,
/// 0 if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
PORT_HANDLE getPortHandle(KERNEL_HANDLE kh, int portNumber);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the port identification number
/// (i.e. port index).
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \return
/// Returns the assigned port identification number if successful,
/// 0 if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
int getPortIndex(PORT_HANDLE ph);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the port identification string.
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \param identifier [output]
/// Returns the assigned port identification string, if successful,
/// or an empty string if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
void getPortIdentifier(PORT_HANDLE ph, char * const identifier);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the port attributes.
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \return
/// Returns an integer containing the port attributes if successful,
/// 0 if fails to find the kernel in the database.
/// Port type values:
/// 1 - the port is an input to the kernel
/// 2 - the port is an output from the kernel
/// 3 - the port represents a global data store
/// 4 - the port exists on a processing stack,
/// 5 - the port represents constant data,
/////////////////////////////////////////////////////////////////////////
int getPortAttributes(PORT_HANDLE ph);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the port left (west) dependency value.
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \return
/// Returns the dependency value in pixels if successful,
/// 0 if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
int getPortDep_l(PORT_HANDLE ph);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the port right (east) dependency value.
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \return
/// Returns the dependency value in pixels if successful,
/// 0 if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
int getPortDep_r(PORT_HANDLE ph);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the port top (north) left dependency value.
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \return
/// Returns the dependency value in pixels if successful,
/// 0 if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
int getPortDep_t(PORT_HANDLE ph);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the port bottom (south) dependency value.
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \return
/// Returns the dependency value in pixels if successful,
/// 0 if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
int getPortDep_b(PORT_HANDLE ph);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the e0 datatype.
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \return
/// Returns 0 if fails to find the kernel in the database.  Otherwise,
/// returns:
/// 1 - 8-bit signed
/// 2 - 8-bit unsigned
/// 3 - 16-bit signed
/// 4 - 16-bit unsigned
/// 5 - 32-bit signed
/// 6 - 32-bit unsigned
/////////////////////////////////////////////////////////////////////////
int getPortE0Datatype(PORT_HANDLE ph);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the port size of the element zero value
/// in the x direction, in pixels.
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \return
/// Returns the size value in pixels if successful,
/// 0 if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
int getPortE0_x_size(PORT_HANDLE ph);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the port size of the element zero value
/// in the y direction, in pixels.
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \return
/// Returns the size value in pixels if successful,
/// 0 if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
int getPortE0_y_size(PORT_HANDLE ph);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the TBD value
/// in the x direction, in pixels.
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \return
/// Returns the value in pixels if successful,
/// 0 if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
int getPortEk_x_size(PORT_HANDLE ph);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the TBD value
/// in the y direction, in pixels.
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \return
/// Returns the value in pixels if successful,
/// 0 if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
int getPortEk_y_size(PORT_HANDLE ph);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the port handle, query the TBD value.
///
/// \param ph [input]
/// Unique system identifier for the port being queried.
/// \return
/// Returns the value in pixels if successful,
/// 0 if fails to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
int getPortAccessType(PORT_HANDLE ph);
/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the kernel handle, query the number of ports associated with the kernel.
///
/// \param kh [input]
/// KERNEL_INFO pointer to the requested kernel.
/// \return
/// Returns 0 if fails to find the kernel in the database.  Otherwise,
/// returns:
/// 1 - direct access
/////////////////////////////////////////////////////////////////////////
int getNumPorts(KERNEL_HANDLE kh);

/////////////////////////////////////////////////////////////////////////
/// \par Description
///
/// Given the kernel name, query the database and if it exists, return
/// KERNEL_INFO pointer to this kernel.
///
/// \param kernel_name [input]
/// Char array that contains the kernel name.
/// \return
/// KERNEL_INFO pointer to the requested kernel if successful, 0 if fails
/// to find the kernel in the database.
/////////////////////////////////////////////////////////////////////////
KERNEL_HANDLE   getKernelHandle    (const char * kernel_name);

/// @}


#endif /* APU2KERNELDBLIB_H */