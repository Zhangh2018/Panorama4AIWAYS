/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2017 NXP
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
/*!*********************************************************************************
*  \file apexcv_integral_image_ref.cpp
*  \brief Reference code for integral image kernel
***********************************************************************************/

#include <stdint.h>

#include <integral_image_ref.h>

void integral_image_ref(uint32_t* dst, const uint8_t* src, int ss, int sw, int sh)
{
  uint32_t *d = dst;
  uint8_t *s = (uint8_t *)src;
  for (int y = 0; y < sh; ++y)
  {
    d[0] = s[0];
    for (int x = 1; x < sw; ++x)
    {
      d[x] = d[x - 1] + s[x];
    }

    d += ss;
    s += ss;
  }

  d = dst;
  s = (uint8_t *)src;
  d += ss;
  s += ss;

  for (int y = 1; y < sh; ++y)
  {
    for (int x = 0; x < sw; ++x)
    {
      d[x] = d[x - ss] + d[x];
    }

    d += ss;
    s += ss;
  }

}
