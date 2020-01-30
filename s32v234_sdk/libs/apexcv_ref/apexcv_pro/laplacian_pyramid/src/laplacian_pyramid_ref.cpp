/**************************************************************************************************
 * 
 * NXP Confidential Proprietary
 *
 * Copyright 2016 NXP 
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
 **************************************************************************************************/

/*==================================================================================================
Revision History:
                              Modification     Tracking
Author (core ID)              Date D/M/Y       Number     Description of Changes
---------------------------   ----------    ------------  ------------------------------------------
Adrian Grigore (nxa11567)     13/12/2016    VSDK-554      Created
---------------------------   ----------    ------------  ------------------------------------------
==================================================================================================*/

#include <stdint.h>
#include <stdlib.h>
#define LAP_PAD 2

static void ref_lap_pad_(
         uint8_t* outp,
   const int      outs,
   const uint8_t* inp,
   const int      ins,
   const int      width,
   const int      height)
{
   for (int y=0; y<height; ++y)
   {
      //Main data
      for (int x=0; x<width; ++x)
      {
         outp[x + LAP_PAD + (y + LAP_PAD) * outs] = inp[x + y * ins];
      }
   }
   for (int y=LAP_PAD; y<height + LAP_PAD; ++y)
   {
      for (int x=0; x<LAP_PAD; ++x)
      {
         //Left padding
         outp[x                   + y * outs] = outp[LAP_PAD             + y * outs];
         //Right padding
         outp[x + width + LAP_PAD + y * outs] = outp[LAP_PAD + width - 1 + y * outs];
      }
   }
   for (int y=0; y<LAP_PAD; ++y)
   {
      for (int x=0; x<width + 2 * LAP_PAD; ++x)
      {
         //Top padding
         outp[x + y * outs]                      = outp[x + LAP_PAD * outs];
         //bottom padding
         outp[x + (y + height + LAP_PAD) * outs] = outp[x + (LAP_PAD + height - 1) * outs];
      }
   }
}

static void ref_lap_gauss5x5_(
         int16_t* outp,
   const int      outs,
   const uint8_t* inp,
   const int      ins,
   const int      width,
   const int      height)
{
   const uint8_t* src;
   uint16_t  data;
   for (int y=0; y<height; ++y)
   {
      for (int x=0; x<width; ++x)
      {
         src = inp + x + LAP_PAD + (y + LAP_PAD) * ins;
         data =   (uint16_t)src[-2*ins - 2] +  4*(uint16_t)src[-2*ins - 1] +  6*(uint16_t)src[-2*ins] +  4*(uint16_t)src[-2*ins + 1] +   (uint16_t)src[-2*ins + 2]
              + 4*(uint16_t)src[-  ins - 2] + 16*(uint16_t)src[-  ins - 1] + 24*(uint16_t)src[-  ins] + 16*(uint16_t)src[-  ins + 1] + 4*(uint16_t)src[-  ins + 2]
              + 6*(uint16_t)src[       - 2] + 24*(uint16_t)src[       - 1] + 36*(uint16_t)src[     0] + 24*(uint16_t)src[       + 1] + 6*(uint16_t)src[       + 2]
              + 4*(uint16_t)src[+  ins - 2] + 16*(uint16_t)src[+  ins - 1] + 24*(uint16_t)src[+  ins] + 16*(uint16_t)src[+  ins + 1] + 4*(uint16_t)src[+  ins + 2]
              +   (uint16_t)src[+2*ins - 2] +  4*(uint16_t)src[+2*ins - 1] +  6*(uint16_t)src[+2*ins] +  4*(uint16_t)src[+2*ins + 1] +   (uint16_t)src[+2*ins + 2];
         outp[x+y*outs] = (int16_t)(data>>8);
      }
   }
}


static void ref_lap_subtract_(
         int16_t* outp,
   const int      outs,
   const uint8_t* in1p,
   const int      in1s,
   const int16_t* in2p,
   const int      in2s,
   const int      width,
   const int      height)
{
   for (int y=0; y<height; ++y)
   {
      for (int x=0; x<width; ++x)
      {
         outp[x+y*outs] = (int16_t)in1p[x+y*in1s] - in2p[x+y*in2s];
      }
   }
}

static void ref_lap_decimate_(
         uint8_t* outp,
   const int      outs,
   const int16_t* inp,
   const int      ins,
   const int      width,
   const int      height)
{
   for (int y=0; y<height/2; ++y)
   {
      for (int x=0; x<width/2; ++x)
      {
         outp[x+y*outs] = (uint8_t)inp[2*x+1+(2*y+1)*ins];
      }
   }
}

void 
ref_laplacian(
   const uint8_t* ip, 
   const int      is,
         int16_t* ofiltp,
   const int      ofilts,
         int16_t* olapp,
   const int      olaps,
         uint8_t* onextp,
   const int      onexts,
   const int      width,
   const int      height)
{
   //Create scratch area for gaussian 5x5 filtering
   uint8_t*  scrp = (uint8_t*)malloc((height + 2 * LAP_PAD) * (width + 2 * LAP_PAD) * sizeof(uint8_t) );
   const int scrs = width + 2 * LAP_PAD;
   
   ref_lap_pad_(scrp, scrs, ip, is, width, height);
   
   ref_lap_gauss5x5_(ofiltp, ofilts, scrp, scrs, width, height);
   
   ref_lap_subtract_(olapp, olaps, ip, is, ofiltp, ofilts, width, height);
   
   ref_lap_decimate_(onextp, onexts, ofiltp, ofilts, width, height);
   free(scrp);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
