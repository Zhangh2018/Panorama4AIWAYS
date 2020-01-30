/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2017 NXP
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

 #ifdef ACF_KERNEL_IMPLEMENTATION 
#include "fast_apu.h"

void apu_nms(
      const vec08u* apcIn, vec08u* apOut,
      int aInStride, int aOutStride,
      int aTileWidth, int aTileHeight)
{
  for (int y = 0; y < aTileHeight; ++y)
  {
    for (int x = 0; x < aTileWidth; ++x)
    {
      vec08u v = apcIn[x];

      vbool fail = v < apcIn[x - 1 - aInStride];
      fail = fail || (v < apcIn[x - aInStride]);
      fail = fail || (v < apcIn[x + 1 - aInStride]);
      fail = fail || (v < apcIn[x - 1]);
      fail = fail || (v <= apcIn[x + 1]);
      fail = fail || (v <= apcIn[x - 1 + aInStride]);
      fail = fail || (v <= apcIn[x + aInStride]);
      fail = fail || (v <= apcIn[x + 1 + aInStride]);
      vif (fail)
      {
        v = 0;
      }
      vendif;
      apOut[x] = v;
    }

    apcIn += aInStride;
    apOut += aOutStride;
  }
}

void apu_fastx_offset
(
   uint32_t* offsets,
   int      sstr,
   int      circumference
)
{
   // The accelerated segment tests, uses a set of predefined points around
   // a center point that will be consider a "corner" if it passes the FAST test   
   // These offset in bytes relative to the center are calculated here
   // The circumference can be: 16, 12 and 8 points
   // For each of these points, the offset is calculated
   int rval = 0;
   static const int16_t pos[][2] =
   {
   //  9/16: 0 - 16
      {0,  3}, { 1,  3}, { 2,  2}, { 3,  1}, { 3, 0}, { 3, -1}, { 2, -2}, { 1, -3},
      {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3,  1}, {-2,  2}, {-1,  3},
   //  7/12: 16 - 12
      {0,  2}, { 1,  2}, { 2,  1}, { 2, 0}, { 2, -1}, { 1, -2},
      {0, -2}, {-1, -2}, {-2, -1}, {-2, 0}, {-2,  1}, {-1,  2},
   //  5/ 8: 28 - 36
      {0,  1}, { 1,  1}, { 1, 0}, { 1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1,  1}
   };

   const int16_t (*p)[2] = 0;
   int16_t s = 0;
   
   switch (circumference)
   {
   case 16:  p = &pos[ 0]; s = 16; break;
   case 12:  p = &pos[16]; s = 12; break;
   case  8:  p = &pos[28]; s =  8; break;
   default: rval = -1;   
   }
   
   if (0 == rval)
   {            
      for (int k=0; k<s; ++k)
         // Calculate the offset in bytes relative to the input image
         offsets[k] = p[k][0] + p[k][1]*sstr;
   }
}

/* FAST without NMS */
void apu_fast
(
   vec08u*  dst, int dstr,    //!< Image of corners (binary/corner score/reponse/strenght)
   vec08u*  src, int sstr,    //!< Source image (grayscale)
   int  bw,  int bh,          //!< block width/height
   int  threshold,            //!< Threshold
   int* offset                //!< pre-computed offset positions
)
{
   const vec08u* sptr = src;
         vec08u* dptr = dst;

   // The process is iterative, the code goes over all the pixels in a chunk
   // and applies the FAST test
   // The test is relatively simple:
   // 1. Select a pixel, p
   // 2. Read the threshold value
   // 3. Consider a circle around the center pixel of "circumference" pixels
   // 4. If p is a corner, if the algorithm finds n consecutive pixels in the circle
   //    which are all brighter/darker than Intensity of P + threshold / - threshold   

   #ifdef __llvm__
     __builtin_assume(bh > 0);
     __builtin_assume(bw > 0);
   #endif
   // Loop rows
   for (int y = 0; y<bh; ++y) chess_loop_range(1, )
   {
       // Loop cols
      for (int x=0; x<bw; ++x) chess_loop_range(1,) 
      {
         // Read the interest point, step 1
         vec16s ic  = (vec16s)sptr[x];

         // Calculate the lower and upper bound threshold
         vec16s tl  = ic - threshold; //lower threshold
         vec16s th  = ic + threshold; //upper threshold

         // Accumulators for calculating the FAST test
         vec16u fl  = 0;
         vec16u fh  = 0;
         
         // Loop all the circumference pixel (step 3)
#ifdef __llvm__
         #pragma unroll(2)
         for (int k = 0; k<16; ++k)
         {
            vec16s ik = (vec16s)sptr[x + offset[k]];
            fl = (vec16u)vsrc((vec16s)fl, (ik < tl));  // pushbit
            fh = (vec16u)vsrc((vec16s)fh, (ik > th));  // pushbit
         }
#else
         vec16s ik = (vec16s)sptr[x + offset[0]];
         for (int k=1; k<16; ++k)
         {
            // Do the brightness test (step 4)
            fl = (vec16u)vsrc((vec16s)fl, (ik < tl));
            fh = (vec16u)vsrc((vec16s)fh, (ik > th));

            // Software pipeline, read the next sample in the circumference circle
            ik = (vec16s)sptr[x + offset[k]];
         }
         fl = (vec16u)vsrc((vec16s)fl, (ik < tl));  // pushbit
         fh = (vec16u)vsrc((vec16s)fh, (ik > th));  // pushbit
#endif

         // Select the highest population count
         vec16u fs  = vselect(fl, fh, vpcnt(fl) > vpcnt(fh));

         // Computing the Fast9 response (Step 4)
         // Test if "fs" has at least 9 consecutive bits (FAST9 Test)
         // This is called "consecutive bits test" and can be found on the Web
         vec16u rs;
         {
            vec16u tl, th;
            vsr(tl, th, fs, fs, 1); rs = fs & tl;  
            vsr(tl, th, rs, rs, 2); rs = rs & tl;  
            vsr(tl, th, rs, rs, 4); rs = rs & tl;  
            vsr(tl, th, fs, fs, 8); rs = rs & tl;
         }

         // If "rs" has residual data after the consecutive bit tests
         // this means that the algorithm found at least 9 consecutive bits         
         // 1: corners
         // 0: no corners
         // The output is an image
         vec16s   score = (vec16s)vselect((vec16s)1, (vec16s)0, (rs != 0));
         dptr[x] = (vec08u)score;
      }
      sptr += sstr;
      dptr += dstr;
   }
}

/* fast without NMS and serialized cofficients */
#define X 0
#define Y 1
#define isPowerOf2(x) ((~((x) & 0x00000001)) & 0x00000001)

static int yOffset = 0;

void apu_fast_serialized
(
   unsigned short * list, 
   int * nrOfElements,
   int firstCuId, 
   int nrOfCu,
   int maxOutSample,
   vec08u * src,
   int sstr,
   int bw,  
   int bh,
   int threshold,
   int * offset)
{
    
    // The process is iterative, the code goes over all the pixels in a chunk
    // and applies the FAST test
    // The test is relatively simple:
    // 1. Select a pixel, p
    // 2. Read the threshold value
    // 3. Consider a circle around the center pixel of "circumference" pixels
    // 4. If p is a corner, if the algorithm finds n consecutive pixels in the circle
    //    which are all brighter/darker than Intensity of P + threshold / - threshold  

    const int32_t pair = 2;    
    const int32_t maxTmpPairs = 64;

    // Precalculate the CU start/end
    int32_t startCu = firstCuId;
    int32_t endCu = (firstCuId + nrOfCu);

    // Counter for the vector samples
    vec16s cnt = 0;

    // Temporary storage
    vec16s coordinates[maxTmpPairs * pair];
     
     // Duplicate of the source pointer
    const vec08u * sptr = src;

    // Load the current number of elements
    int32_t gCnt = *nrOfElements;

    vec16s cuIndex = 0;
    for(int cuId = startCu; cuId < endCu; cuId++)
    {
        cuIndex = vput(cuIndex, cuId, cuId);
    }

    vec16s cuWidthOffset = (cuIndex * bw);

    // Special exit conditions
    if(isPowerOf2(maxOutSample) == 0) {*nrOfElements = -1; return;};

    int maxOutPairs = maxOutSample >> 1;
    
    // Loop rows
    for (int y = 0; y < bh; y++) chess_loop_range(1,) 
    {
        // Loop cols
        for (int x = 0; x < bw; x++) chess_loop_range(1,) 
        {
            // Read the interest point, step 1
            vec16s ic  = (vec16s)sptr[x];

            // Calculate the lower and upper bound threshold
            vec16s tl  = ic - threshold; //lower threshold
            vec16s th  = ic + threshold; //upper threshold

            // Accumulators for calculating the FAST test
            vec16u fl  = 0;
            vec16u fh  = 0;         
            
            // Read the first pixel from the circumference
            vec16s ik = (vec16s)sptr[x + offset[0]];

            // Loop all the circumference pixel (step 3)
            for (int k=1; k<16; ++k)
            {
                fl = (vec16u)vsrc((vec16s)fl, (ik < tl));  // pushbit
                fh = (vec16u)vsrc((vec16s)fh, (ik > th));  // pushbit                
                ik = (vec16s)sptr[x + offset[k]];
            }
            fl = (vec16u)vsrc((vec16s)fl, (ik < tl));  // pushbit
            fh = (vec16u)vsrc((vec16s)fh, (ik > th));  // pushbit
                        
            // Select the highest population count
            vec16u fs  = vselect(fl, fh, vpcnt(fl) > vpcnt(fh));

            // Computing the Fast9 response (Step 4)
            // Test if "fs" has at least 9 consecutive bits (FAST9 Test)
            // This is called "consecutive bits test" and can be found on the Web
            vec16u rs;
            {
                vec16u tl, th;
                vsr(tl, th, fs, fs, 1); rs = fs & tl;  
                vsr(tl, th, rs, rs, 2); rs = rs & tl;  
                vsr(tl, th, rs, rs, 4); rs = rs & tl;  
                vsr(tl, th, fs, fs, 8); rs = rs & tl;
            }
            // If "rs" has residual data after the consecutive bit tests
            // this means that the algorithm found at least 9 consecutive bits         
            // 1: corners
            // 0: no corners            
            vif(rs != 0)
            {
                vstore((vec16s *) coordinates, (vec16s) (pair * cnt + X), cuWidthOffset + (vec16s) x);
                vstore((vec16s *) coordinates, (vec16s) (pair * cnt + Y), (vec16s)(yOffset + y));
                cnt++;
            }vendif

            // This kernel output a list of coordinates in the form: <x, y>
            // If there are valid keypoints
            if(vany((cnt == maxTmpPairs)))
            {                
                // This processing is done on the scalar core
                // Loop over all the valid CUs
                for(int cuId = startCu; cuId < endCu; cuId++)
                {
                    // Ok to put inside the queue
                    int32_t size = vget(cnt, cuId);

                    // Clear the counter
                    cnt = vput(cnt, 0, cuId);

                    for(int idx = 0; idx < size; idx++)
                    {    
                        // Overflow check      
                        if(gCnt == maxOutPairs)
                        {
                            // Return an error
                            *nrOfElements = -2;
                            return;
                        }         

                        // Packing all the data next to each other
                        list[pair * gCnt + X] = vget(coordinates[pair * idx + X], cuId);
                        list[pair * gCnt + Y] = vget(coordinates[pair * idx + Y], cuId); 

                        // Increment the keypoints counter                       
                        gCnt++;
                    }
                }                
            }
        }
        sptr += sstr;
    }

    // Jump the next line
    yOffset += bh;

    // Dump the residual coordinates that are in the coordinate buffer
    if (vany(cnt > 0))
    {
        // Scalar processing for each CU
        for(int cuId = startCu; cuId < endCu; cuId++)
        {
            // Ok to put inside the queue
            int32_t size = vget(cnt, cuId);

            // Loop all valid keypoints in the buffer
            for (int idx = 0; idx < size; idx++)
            {
                // Overflow check
                if(gCnt == maxOutPairs)
                {
                    // Return an error
                    *nrOfElements = -2;
                    return;
                }
                // Packing all the data next to each other
                list[pair * gCnt + X] = vget(coordinates[pair * idx + X], cuId);
                list[pair * gCnt + Y] = vget(coordinates[pair * idx + Y], cuId);

                // Increment the keypoints counter 
                gCnt++;
            }            
        }
    }

    // Update the global counter
    *nrOfElements = gCnt;
}

/* FAST with NMS */
void apu_fast_nms
(
   vec08u*  dst, int dstr,    //!< Image of corners (binary/corner score/reponse/strenght)
   vec08u*  src, int sstr,    //!< Source image (grayscale)
   int  bw,  int bh,          //!< block width/height
   int  threshold,            //!< Threshold
   int* offset                //!< pre-computed offset positions
)
{
   vec08u d[16]; // difference
   
    // The process is iterative, the code goes over all the pixels in a chunk
    // and applies the FAST test
    // The test is relatively simple:
    // 1. Select a pixel, p
    // 2. Read the threshold value
    // 3. Consider a circle around the center pixel of "circumference" pixels
    // 4. If p is a corner, if the algorithm finds n consecutive pixels in the circle
    //    which are all brighter/darker than Intensity of P + threshold / - threshold   
    // 5. Perform non-maximum suppresion

   const vec08u* sptr = src;
         vec08u* dptr = dst;

   #ifdef __llvm__
     __builtin_assume(bh > 0);
     __builtin_assume(bw > 0);
   #endif
   
   // Loop rows
   for (int y = 0; y<bh; ++y) chess_loop_range(1, )
   {
       // Loop cols
      for (int x=0; x<bw; ++x) chess_loop_range(1,) 
      {
         // Read the interest point, step 1
         vec16s ic  = (vec16s)sptr[x];

         // Calculate the lower and upper bound threshold
         vec16s tl  = ic - threshold; //lower threshold
         vec16s th  = ic + threshold; //upper threshold

         // Accumulators for calculating the FAST test
         vec16u fl  = 0;
         vec16u fh  = 0;
         
        // Loop all the circumference pixel (step 3)
#ifdef __llvm__
         for (int k = 0; k<16; ++k)
         {
            vec16s ik = (vec16s)sptr[x + offset[k]];

            // Do the brightness test (step 4)
            fl = (vec16u)vsrc((vec16s)fl, (ik < tl));  // pushbit
            fh = (vec16u)vsrc((vec16s)fh, (ik > th));  // pushbit

            d[k] = (vec08u)vabs_diff(ic, ik); // difference information
         }
#else
         vec16s ik = (vec16s)sptr[x + offset[0]];
         for (int k=1; k<16; ++k)
         {
            // Do the brightness test (step 4)
            fl = (vec16u)vsrc((vec16s)fl, (ik < tl));
            fh = (vec16u)vsrc((vec16s)fh, (ik > th));            

            // Keep the difference information
            d[k - 1] =  (vec08u)vabs_diff(ic, ik);

            // Software pipeline, read the next sample in the circumference circle
            ik = (vec16s)sptr[x + offset[k]];           
         }
         fl = (vec16u)vsrc((vec16s)fl, (ik < tl));  // pushbit
         fh = (vec16u)vsrc((vec16s)fh, (ik > th));  // pushbit
         d[15] =  (vec08u)vabs_diff(ic, ik);  // difference information
#endif

         // Select the highest population count
         vec16u fs  = vselect(fl, fh, vpcnt(fl) > vpcnt(fh));

         // Computing the Fast9 response (Step 4)
         // Test if "fs" has at least 9 consecutive bits (FAST9 Test)
         // This is called "consecutive bits test" and can be found on the Web
         vec16u rs;
         {
            vec16u tl, th;
            vsr(tl, th, fs, fs, 1); rs = fs & tl;  
            vsr(tl, th, rs, rs, 2); rs = rs & tl;  
            vsr(tl, th, rs, rs, 4); rs = rs & tl;  
            vsr(tl, th, fs, fs, 8); rs = rs & tl;
         }

         // Select the threshold if the interst point is a corner
         vec16s   score = (vec16s)vselect((vec16s)threshold, (vec16s) 0, (rs != 0));

         // Non maximum suppresion test
         // 1. Compute a score function for all the detected keypoints, this is done
         //    by calculating the absolute difference between the interst point
         //    and the surrounding pixels -> d[16]
         // 2. Consider two adjacent keypoints and compute their score
         // 3. Discard the one with the lowes value

         vif (rs != 0)
         {
            vec16u ft = 0;
            for (int k=0; k<16; ++k) chess_unroll_loop(*)
            {
              // Generate a bit pattern               
              ft = (vec16u)vsrc((vec16s)ft, (d[k] > ((vec08u)score + 1))); // pushbit 
            }
            fs &= ft;

            // Early exit
            {
               vec16u tl, th;
               vsr(tl, th, fs, fs, 1); rs = fs & tl;  
               vsr(tl, th, rs, rs, 2); rs = rs & tl;  
               vsr(tl, th, rs, rs, 4); rs = rs & tl;  
               vsr(tl, th, fs, fs, 8); rs = rs & tl;
            }         

            // Do the computation only if at least one CU has valid data
#if (__chess__)
            while (rs != 0)
#elif (__llvm__)
            while (vany(rs != 0))
#endif
            {
#ifdef __llvm__
                vif(rs != 0) {
#endif
               // Find the minimum value & position(s)
               vec16s tmin = 256;
               vec16u kmin =   0;
               vec16u fs_copy = fs;
               for (int k=0; k<16; ++k)
               {
                  vec16s dk = (vec16s)d[k];
                  /* move lsb -> vc */ 
                  vbool fe  = (vbool)(fs_copy);
                  fs_copy >>= 1;
                  vbool dlt = (dk <  tmin) && fe;
                  vbool dle = (dk <= tmin) && fe;
                  tmin = vselect(dk, tmin, dlt);
                  kmin = vselect( (vec16u) 0, kmin, dlt);
                  kmin = (vec16u)vsrc((vec16s)kmin, dle);
               }
               
               // update fs and rs...
               fs &= ~kmin;
               {
                  vec16u tl, th;
                  vsr(tl, th, fs, fs, 1); rs = fs & tl;  
                  vsr(tl, th, rs, rs, 2); rs = rs & tl;  
                  vsr(tl, th, rs, rs, 4); rs = rs & tl;  
                  vsr(tl, th, fs, fs, 8); rs = rs & tl;
               }
               score = tmin - 1;
#ifdef __llvm__
                } vendif
#endif
            }     // while (rs != 0)
         }        // if (rs != 0)
         dptr[x] = (vec08u)score;
      }
      sptr += sstr;
      dptr += dstr;
   }
}
#endif   // #ifdef ACF_KERNEL_IMPLEMENTATION 