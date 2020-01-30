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

#ifndef ACFREPORT_H
#define ACFREPORT_H

#include <stdio.h>
#include <time.h>
#include <cstring>

#ifndef TEST_LOG_DIR
	#define TEST_LOG_DIR ("C:/__Traceability/APEXCV/")
#endif

////////////////////////////////////////////////////////////////////////
/// \defgroup CGV_TOOLS          S/W Verification Tools
////////////////////////////////////////////////////////////////////////
///@{ 

////////////////////////////////////////////////////////////////////////
/// \defgroup CGV_TOOLS_REPORT   S/W Tools - Test Cases Report
////////////////////////////////////////////////////////////////////////
///@{ 

//! Get time.
time_t   CGV_LOG_GetTime();

// disable report to stdout
#define CGV_REPORT_STDOUT (0)

#define CGV_REPORT_TC_NAME_SIZE     (128)
#define CGV_REPORT_SIM_NAME_SIZE    (128)
#define CGV_REPORT_PATH_NAME_SIZE   (128)
#define CGV_REPORT_FILE_NAME_SIZE   (128)

#ifndef gCGV_RPT_DBASE

   #define CGV_REPORT_INIT(_PATH, _TC)
   #define CGV_REPORT_START(_SIM)
   #define CGV_REPORT_END(_NB)
   #define CGV_REPORT_CLOSE()
   #define CGV_REPORT_FILE_XML_STR(_HANDLE, _STR, _EOL, _TAB)
   #define CGV_REPORT_FILE_XML_INT(_HANDLE, _STR, _EOL, _TAB)
   #define CGV_REPORT_FILE(_R)
   #define CGV_REPORT_LOG(...)
   #define CGV_REPORT_GRAPH(_R)

   //! Report Function Database (Functions)
   extern char*         gCGV_RPT_TC_NAME;
   extern char*         gCGV_RPT_SIM_NAME;
   extern char*         gCGV_RPT_PATH_NAME;
   extern char*         gCGV_RPT_FILE_NAME;
   extern FILE*         gCGV_RPT_FILE_HANDLE;
   extern time_t        gCGV_TIME;
   extern unsigned int  gCGV_ERROR;

#else

   #define CGV_REPORT_LOG(...)
   
#endif

#endif /* ACFREPORT_H */
