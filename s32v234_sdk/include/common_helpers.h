/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright 2017-2018 NXP
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
*  \file   COMMON_helpers.h
*  \brief  Helper functions from the common library.
*/
#ifndef COMMON_HELPERS_H
#define COMMON_HELPERS_H

/*** to suppress unused parameter warning ***/
#define UNUSED_ARG(_x) (void)(_x)

#ifdef __cplusplus
extern "C" {
#endif 

/**
* @brief          Extract from a path the program name
*
* @param[in]      apProgPath - a path to a program/file
*/
const char* COMMON_ExtractProgramName(const char* apProgPath);


/**
* @brief          Convert a string to upper case
* @pre            aUpperStr should have the same length/memory allocated as aStr
*
* @param[in]      apStr - any input string
* @param[out]     apUpperStr - the string converted to an upper string
*/
void COMMON_StrToUpper(const char* apStr, char* apUpperStr);


/**
* @brief         Writes out a help message if apArgv contains a "help" option
*
* @param[in]     aArgc  - number of arguments provided in apArgv
* @param[in]     apArgv - command line arguments
* @param[in]     apMsg  - the help message to be displayed in case the arguments contain a help option
* @return        the argument index where the help option was found, otherwise -1 
*/
int COMMON_HelpMessage(int aArgc, char** apArgv, const char* apMsg);

#ifdef __cplusplus
}
#endif 

#endif
