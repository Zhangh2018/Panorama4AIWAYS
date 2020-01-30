/*******************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright 2018 NXP
 * All Rights Reserved
 *
 *******************************************************************************
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
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <string>

// To be included
#include <common_helpers.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) || defined(WIN32) || defined(__WIN32)
// Windows style path delimiter, short and long option prefixes and postfixes
#define DELIM '\\'
#define OPT_LPREFIX "/"
#define OPT_SPREFIX "/"
#define OPT_SPOSTFIX "?"
#else
// Linux style path delimiter, short and long option prefixes and postfixes
#define DELIM '/'
#define OPT_LPREFIX "--"
#define OPT_SPREFIX "-"
#define OPT_SPOSTFIX "h"
#endif

#define HELP_SHORT_OPT  OPT_SPREFIX OPT_SPOSTFIX
#define HELP_LONG_OPT   OPT_LPREFIX "HELP"
#define HELP_MAX_LEN    sizeof HELP_LONG_OPT

const char* COMMON_ExtractProgramName(const char* apProgPath) {
#ifndef __STANDALONE__
  std::string prog(apProgPath);
  // find last slash or back slash of the path
  size_t pos = prog.find_last_of(DELIM);  // find last path slash
  const char *subProgName = apProgPath;

  // if there is a slash/backslash
  if (pos > 0 && pos < prog.length() - 1) {
    // extract only program name
    subProgName = &apProgPath[++pos];
  }

  return subProgName;
#else
  std::string prog("");
  return prog.c_str();
#endif
}


void COMMON_StrToUpper(const char* apStr, char* apUpperStr)
{
  size_t sLen = strlen(apStr);
  size_t i = 0;
  for (; i< sLen; ++i)
    apUpperStr[i] = toupper(apStr[i]);
  apUpperStr[i] = '\0';
}

int COMMON_HelpMessage(int aArgc, char** apArgv, const char* apMsg)
{
#ifndef __STANDALONE__
  int idxHelp = -1; // if no help option is found a -1 is returned.
 	if (aArgc >= 2)
	{
    //allocate a string for upper case transforming of the option. It has the size of max possible help option
    char upperArgv[HELP_MAX_LEN+2];
    for (int i = 1; idxHelp < 0 && i < aArgc; ++i)
    {
      // scan through all arguments
      if (strlen(apArgv[i]) <= HELP_MAX_LEN)
      {
        // if option could be a help option (i.e. is no longer than the max possible help option
        COMMON_StrToUpper(apArgv[i], upperArgv);

        // compare to short or long help option
        if (0 == strcmp(upperArgv, HELP_SHORT_OPT) || 0 == strcmp(upperArgv, HELP_LONG_OPT))
        {// if there is a help option in the argument list print the help message
          printf("%s\n", apMsg);

          // return the argument index of the help option
          idxHelp = i;
        }
      }
    }
	}
  return idxHelp;
#else
  return 0;
#endif
}

#ifdef __cplusplus
}
#endif
