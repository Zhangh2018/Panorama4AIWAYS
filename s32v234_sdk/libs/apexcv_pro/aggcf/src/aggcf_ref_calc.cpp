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


#include "aggcf_ref_calc.h"

static ufx32p0 fxrecip22(ufx32p1 d);


// pad image and return padded image point( which point to starting position of valid data)
uint16_t * image_pad(uint16_t *src, int srcwidth, int srcheight, int left, int right, int up, int bot)
{
    int total_w = srcwidth + left + right;
    int total_h = srcheight + up + bot;

    uint16_t *pad = new uint16_t[total_w*total_h]();

    uint16_t *pad_start = pad + total_w*up + left;
    for (int i = 0; i < srcheight; i++)
    {
        memcpy(pad_start+i*total_w, src+i*srcwidth, sizeof(uint16_t)*srcwidth);
    }

    for (int i = 0; i < up; i++)
    {
        memcpy(pad+i*total_w, pad+up*total_w, sizeof(uint16_t)*total_w);
    }

    for (int i = 0; i < bot; i++)
    {
        memcpy(pad + (i + up + srcheight)*total_w, pad + (up + srcheight - 1)*total_w, sizeof(uint16_t)*total_w);
    }

    for (int i = 0; i < total_h; i++)
    {
        for (int j = 0; j < left; j++)
        {
            pad[i*total_w + j] = pad[i*total_w + left];
        }
        for (int j = 0; j < right; j++)
        {
            pad[i*total_w + srcwidth + left + j] = pad[i*total_w + srcwidth + left - 1];
        }
    }

    return pad;
}


int32_t calcAtanReg(uint32_t Gy, uint32_t Gx)
{

    int32_t start = 0, end = 64, mid;
    while ((start + 1) < end)
    {
        mid = start + ((end - start) >> 1);
        if ((Gy << 6) == (Gx*mid))
        {
            start = mid;
        }
        else if ((Gy << 6) > (Gx*mid))
        {
            start = mid;
        }
        else
        {
            end = mid;
        }
    }


    if ((Gy << 6) <= (Gx*start))
    {
        return start;
    }
    return end;
}


uint32_t atanlut[65] = { 0,
16775850, 33543515, 50294832, 67021686, 83716035, 100369929, 116975535, 133525158, 150011261, 166426484, 182763663, 199015845,
215176308, 231238569, 247196399, 263043836, 278775191, 294385059, 309868320, 325220151, 340436022, 355511705, 370443267,
385227073, 399859787, 414338361, 428660037, 442822339, 456823069, 470660297, 484332355, 497837829, 511175551, 524344587, 537344231,
550173993, 562833590, 575322935, 587642129, 599791447, 611771333, 623582386, 635225352, 646701113, 658010681, 669155185, 680135862,
690954054, 701611191, 712108791, 722448446, 732631822, 742660642, 752536690, 762261796, 771837834, 781266719, 790550394, 799690833,
808690030, 817549998, 826272766, 834860371, 843314856 };


uint32_t simatan(int32_t Gx, int32_t Gy, uint32_t *lut)
{
    uint32_t PI_1_2 = 1686629684;
    uint32_t PI_1_4 = 843314842;
    uint32_t PI_0 = 3373259368;
    uint32_t ret;
    uint32_t Gx_abs = abs(Gx);
    uint32_t Gy_abs = abs(Gy);
    Gx_abs = Gx_abs >> 6;
    Gy_abs = Gy_abs >> 6;
    if (Gy_abs == 0)
    {
        if (Gx < 0)
        {
            ret = PI_0;
        }
        else
        {
            ret = 0;
        }
    }
    else if (Gx_abs == 0 || (Gy_abs > (Gx_abs << 6)))
    {
        ret = PI_1_2;
    }
    else
    {
        if (Gx_abs == Gy_abs)
        {
            ret = PI_1_4;
        }
        else
        {
            int32_t ret_idx = calcAtanReg ( ( (Gx_abs < Gy_abs) ? Gx_abs : Gy_abs),
                ( (Gx_abs > Gy_abs) ? Gx_abs : Gy_abs ) );
            ret = (uint32_t)(lut[(int16_t)ret_idx]);
            if (Gx_abs < Gy_abs)
            {
                ret = PI_1_2 - ret;
            }
        }
        if (Gx < 0)
        {
            ret = PI_0 - ret;
        }
        if (Gy < 0)
        {
            ret = PI_0 - ret;
        }
    }

    return ret;
}

void fxgrad1_c_row(uint16_t *I, int32_t *Gx, int32_t *Gy, int32_t stride, int32_t w)
{
    int x;
    uint16_t *Ip, *In;

    Ip = I - stride;
    In = I + stride;

    for (x = 0; x < w; x++)
    {
        int32_t temp_sum = ((int32_t)In[0] << 16) - ((int32_t)Ip[0] << 16);
        int32_t temp = temp_sum >> 1;
        *Gy++ = temp; // Gx is Q1.30
        In++;
        Ip++;
    }

    Ip = I - 1;
    In = I + 1;

    int32_t temp = (((int32_t)In[0] << 16) - ((int32_t)Ip[0] << 16)) >> 1;
    *Gx++ = temp;
    In++;
    Ip++;
    for (x = 1; x < w - 1; x++)
    {
        temp = (((int32_t)In[0] << 16) - ((int32_t)Ip[0] << 16)) >> 1;
        *Gx++ = temp;
        In++;
        Ip++;
    }
    temp = (((int32_t)In[0] << 16) - ((int32_t)Ip[0] << 16)) >> 1;
    *Gx++ = temp;
}


void fxgradMag_c_row(uint16_t* L, uint16_t* U, uint16_t* V, int32_t instride, int32_t outstride,
    uint16_t* M, uint16_t* O,
    int32_t h, int32_t w,
    int32_t *Gx, int32_t *Gy, uint32_t *M2, float &diff)
{
    int32_t x, y, x1, c;

    // compute gradient magnitude and orientation for each column
    for (y = 0; y < h; y++)
    {
        // compute gradients (Gx, Gy) with maximum squared magnitude (M2)
        fxgrad1_c_row(L + y*instride, Gx,         Gy,         instride, w); //L Gx and Gy are Q1.30 here
        fxgrad1_c_row(U + y*instride, Gx + w,     Gy + w,     instride, w); //U Gx and Gy are Q1.30 here
        fxgrad1_c_row(V + y*instride, Gx + 2 * w, Gy + 2 * w, instride, w); //V Gx and Gy are Q1.30 here

        for (c = 0; c < 3; c++)
        {
            for (x = 0; x < w; x++)
            {
                x1 = w * c + x;
                int32_t gx_ = Gx[x1];
                int32_t gy_ = Gy[x1];

                M2[x1] = (abs(gx_) + abs(gy_)) >> 16; // M2 is (U2.30 --> 2.14) here

                if (c == 0)
                {
                    M[y*outstride + x] = (uint16_t)M2[x];
                    continue;
                }
                else if (M2[x1] > M2[x])
                {
                    M2[x] = M2[x1];
                    Gx[x] = gx_;
                    Gy[x] = gy_;
                    M[y*outstride + x] = (uint16_t)M2[x];
                }
            }
        }

        if (O)
        {
            for (x = 0; x < w; x++)
            {
                uint32_t ret = simatan(Gx[x], Gy[x], atanlut);

                float ty = (float)Gy[x] * pow(2, -30);
                float tx = (float)Gx[x] * pow(2, -30);

                float ret_ = atan2(ty, tx);
                if (ret_ < 0)
                {
                    ret_ += 3.14159265f;
                }

                diff += abs((float)ret*pow(2, -30) - ret_);

                // final result in precison of 3.13
                O[y*outstride + x] = (uint16_t)(ret >> 17);// O is U3.13
            }
        }
    }
}

void fxgradQuantize_c_row_trilinear(uint16_t *O, uint16_t *M, uint32_t *O0, uint32_t *O1, uint32_t *M0, uint32_t *M1,
    int32_t , int32_t , int32_t w, int32_t bin, uint32_t norm, int32_t nOrients, int32_t full, int32_t interpolate)
{
    int32_t i;
    uint32_t o0, o1;
    uint32_t o, od, m;
    uint32_t Pi30 = 3373259427; // 3.141592654 * pow(2,30)
    uint32_t invPi30 = fxrecip22(Pi30) >> 1; // U0.32

    const uint32_t oMult = (uint32_t)nOrients * (full ? ((invPi30 >> 1) >> 16) : (invPi30 >> 16)); //U16.16
    const uint32_t oMax = nOrients;

    if (interpolate)
    {
        for (i = 0; i < w + bin; i++)
        {
            o = ((uint32_t)O[i])*((uint32_t)(oMult >> 1)); // 4.28

            o0 = o >> 28;//(int)o;

            od = o - (uint32_t)(o0 << 28); //U4.28

            //o0 *= nb;
            if (o0 >= oMax)
                o0 = 0;
            O0[i] = o0;

            //o1 = o0 + nb;
            o1 = o0 + 1;
            if (o1 == oMax)
                o1 = 0;
            O1[i] = o1;

            m = ((uint32_t)M[i])*((uint32_t)((norm >> 16) << 1)); //U2,30

            M1[i] = ((od >> 16)*(m >> 16)); //U6.26
            M0[i] = (m >> 4) - M1[i]; //U6.26
        }
    }
    else
    {
        for (i = 0; i < w + bin; i++)
        {
            o = ((uint32_t)O[i])*((uint32_t)(oMult >> 1)); //4.28
            o0 = (o + (1 << 27)) >> 28;
            //o0 *= nb;
            if (o0 >= oMax)
                o0 = 0;
            O0[i] = o0;
            M0[i] = ((uint32_t)M[i])*((uint32_t)((norm >> 16) >> 3));
            M1[i] = 0;
            O1[i] = 0;
        }
    }
}


void locateHistBin_evb(uint16_t *H0, uint16_t *H1, uint16_t *H2, uint16_t *H3, uint16_t *H4, uint16_t *H5,
    uint32_t index, uint32_t value, int32_t offset)
{
    if (index == 0)
    {
        H0[offset] = (uint16_t)((uint32_t)H0[offset] + value);
    }
    else if (index == 1)
    {
        H1[offset] = (uint16_t)((uint32_t)H1[offset] + value);
    }
    else if (index == 2)
    {
        H2[offset] = (uint16_t)((uint32_t)H2[offset] + value);
    }
    else if (index == 3)
    {
        H3[offset] = (uint16_t)((uint32_t)H3[offset] + value);
    }
    else if (index == 4)
    {
        H4[offset] = (uint16_t)((uint32_t)H4[offset] + value);
    }
    else
    {
        H5[offset] = (uint16_t)((uint32_t)H5[offset] + value);
    }
}


uint32_t count_leading_zero_s(uint32_t x)
{
    uint32_t n;

    if (x == 0) return(32);
    n = 1;
    if ((x >> 16) == 0) { n = n + 16; x = x << 16; }
    if ((x >> 24) == 0) { n = n + 8; x = x << 8; }
    if ((x >> 28) == 0) { n = n + 4; x = x << 4; }
    if ((x >> 30) == 0) { n = n + 2; x = x << 2; }
    n = n - (uint32_t)(x >> 31);
    return n;
}

/* Fix-point math simulation */
/**
 * Get bit [63..56] from an unsigned int 64
 */
static uint64_t getA(uint64_t x){return (x>>56)&0xFF;}
/**
 * Get bit [55..48] from an unsigned int 64
 */
static uint64_t getB(uint64_t x){return (x>>48)&0xFF;}
/**
//  * Get bit [63..56] from an unsigned int 64
//  */
 static uint64_t getM(uint64_t x){return (x>>56)&0xFF;}
// /**
//  * Get bit [55..48] from an unsigned int 64
//  */
 static uint64_t getN(uint64_t x){return (x>>48)&0xFF;}
// /**
//  * Get bit [47..40] from an unsigned int 64
//  */
 static uint64_t getO(uint64_t x){return (x>>40)&0xFF;}

/**
 * Compute multiplication of two uint64 as:\n
 *   A[63..56]*B[63..56] + A[63..56]*B[55..48] +
 *   A[55..48]*B[63..56]
 */
static uint64_t mul1(uint64_t abcd, uint64_t mnop){
    uint64_t am = getA(abcd)*getM(mnop) << 48;
    uint64_t an = getA(abcd)*getN(mnop) << 40;
    uint64_t bm = getB(abcd)*getM(mnop) << 40;
    
    return (am  +
             an +
             bm  ) & 0xFFFF000000000000;
}
/**
 * Compute multiplication of two uint64 as:\n
 * A[63..56]*B[63..56] + A[63..56]*B[55..48]
 */
static uint64_t mul2(uint64_t abcd, uint64_t mnop){
    uint64_t am = getA(abcd)*getM(mnop) << 48;
    uint64_t an = getA(abcd)*getN(mnop) << 40;
    
    return (am  +
             an)  & 0xFFFF000000000000;
}

/**
 * Compute multiplication of two uint64 as:\n
     A[63..56]*B[63..56] + A[63..56]*B[55..48] + A[63..56]*B[47..40] +
 *   A[55..48]*B[63..56] + A[55..48]*B[55..48]
 */
static uint64_t mul4(uint64_t abcd, uint64_t mnop){
    uint64_t am = getA(abcd)*getM(mnop) << 48;
    uint64_t an = getA(abcd)*getN(mnop) << 40;
    uint64_t ao = getA(abcd)*getO(mnop) << 32;
    uint64_t bm = getB(abcd)*getM(mnop) << 40;
    uint64_t bn = getB(abcd)*getN(mnop) << 32;
    
    return (am   +
             an  +
              ao +
             bm  +
              bn) & 0xFFFFFFFF00000000;
}

ufx16p0 fxrecip12(ufx16p1 d){
    uint64_t dn = (uint64_t)d << 48;
    uint64_t x1 = (dn<<1) & 0xFFFE000000000000ULL;
    uint64_t k0, k1, k2, s0, s1;
    
    if(dn > (46006ULL<<48)){
        s1 = 3;
        s0 = 2;
        k1 =   181ULL<<56;
        k2 = 30945ULL<<48;
        k0 = 63962ULL<<48;
        
        k0 -= 9ULL<<48;
    }else{
        s1 = 1;
        s0 = 4;
        k1 =    31ULL<<56;
        k2 = 58691ULL<<48;
        k0 = 65532ULL<<48;
        
        k0 -= 9ULL<<48;
    }
    
    uint64_t x2   = mul1(x1, x1);
    uint64_t k1x1 = mul2(k1, x1);
    uint64_t a    = k0 - x1 + ((k1x1 >> s0) & 0xFFFF000000000000ULL);
    uint64_t b    = k2 - ((x1 >> s1) & 0xFFFF000000000000ULL);
    uint64_t bx2  = mul1(b, x2);
    uint64_t abx2 = a+bx2;
    return   abx2 >> 48;
}

/**
 * Compute reciprocal estimate to 22 bits.
 * 
 * h  = 1-d*x
 * x' = x+x*(h)
 *
 * @param [in]  d    The value of dn must be in the range [1.0, 2.0)
 * @return           Reciprocal of input value
 */

ufx32p0 fxrecip22(ufx32p1 d){
    uint64_t dn   = (uint64_t)d<<32;
    uint64_t r0   = (uint64_t)fxrecip12(d>>16)<<48;
    uint64_t h0   = 0x8000000000000000ULL - mul4(r0, dn);
    uint64_t r0h0 = mul4(r0,h0)<<1;
    uint64_t r1 = r0 + r0h0;
    return   r1 >> 32;
}



void fxgradHist_c_row_trilinear(uint16_t* M, uint16_t* O,
    uint16_t* bin0, uint16_t *bin1, uint16_t *bin2, uint16_t *bin3, uint16_t *bin4, uint16_t *bin5,
    int32_t h, int32_t w, int instride, int32_t outstride,
    int32_t wb, int32_t hb, int32_t nb, int32_t bin, int32_t nOrientations, int32_t softBin, int32_t full,
    uint32_t *M0, uint32_t*M1, uint32_t*O0, uint32_t *O1)
{
    uint32_t binInv;

    if (bin == 1)
    {
        binInv = (1 << 31);
    }
    else
    {
        uint16_t lZ = count_leading_zero_s((uint32_t)bin);
        binInv = fxrecip22(bin << lZ) >> 2;
    }

    uint32_t sInv = binInv, sInv2;

    if (bin == 1)
        sInv2 = (1 << 31); //1 / s / s; 1.31
    else
        sInv2 = ((binInv >> 16) * (binInv >> 16)) << 1; // U1.31

    int32_t x, y;
    int32_t xb, init;
    uint32_t ONE_F = 2147483648;


    // 1/bin-1.0f: this is based on how padding is done
    // sinV is 1.31
    init = (ONE_F - (sInv >> 1));
    init = -init;
    // convert to 2.14
    init >>= 17;
    xb = init;
    bool hasUp, hasBot;

    // move M and O to top/left of padded chunk buffer
    M = M - bin*instride - bin;
    O = O - bin*instride - bin;

    for (int i = 0; i < hb; i++)
    {
        for (int j = 0; j < wb; j++)
        {
            bin0[i*outstride + j] = 0;
            bin1[i*outstride + j] = 0;
            bin2[i*outstride + j] = 0;
            bin3[i*outstride + j] = 0;
            bin4[i*outstride + j] = 0;
            bin5[i*outstride + j] = 0;
        }
    }
    for (x = 0; x < hb*bin + bin; x++)
    {
        fxgradQuantize_c_row_trilinear(O + x*instride, M + x*instride, O0, O1, M0, M1, nb, h, w, bin, sInv2, nOrientations, full, softBin >= 0);

        uint32_t ms[4], xyd, xd, yd; int32_t yb, xb0, yb0;
        uint32_t ONE2_14 = 16384; // 1 in 2.14 format

        hasUp = (xb >= 0);
        xb0 = xb >> 14;
        hasBot = (xb0 < hb-1);
        xb0 = xb0 << 14;

        xd = xb - xb0;
        xb += sInv >> 17;

        yb = init;
        for (y = 0; y < wb*bin + bin; y++)
        {
            yb0 = yb >> 14;
            yb0 = yb0 << 14;
            yd = yb - yb0;
            yb += sInv >> 17;

            xyd = ((xd*yd) << 2) >> 16;
            ms[0] = ONE2_14 - xd - yd + xyd;
            ms[1] = xd - xyd;
            ms[2] = yd - xyd;
            ms[3] = xyd;

            // floored intger point in histogtam
            //H0 = H + (yb0>>14) + (xb0>>14)*wb;
            int32_t start_idx = (yb0 >> 14) + (xb0 >> 14)*outstride;
            if (y < bin)
            {
                if (hasUp)
                {
                    //ms 2.14 M 6.26
                    uint32_t ti = ms[2] * (M0[y] >> 12);
                    // convert to 2.14
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O0[y], (ti >> 14), start_idx + 1);

                    ti = ms[2] * (M1[y] >> 12);
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O1[y], (ti >> 14), start_idx + 1);
                }
                if (hasBot)
                {
                    //ms 2.14 M 6.26
                    uint32_t ti = ms[3] * (M0[y] >> 12);
                    //2.14
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O0[y], (ti >> 14), start_idx + outstride + 1);

                    ti = ms[3] * (M1[y] >> 12);
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O1[y], (ti >> 14), start_idx + outstride + 1);
                }
            }
            else if (y >= bin && y < wb*bin)
            {
                if (hasUp)
                {
                    uint32_t ti = ms[0] * (M0[y] >> 12);
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O0[y], (ti >> 14), start_idx);

                    ti = ms[0] * (M1[y] >> 12);
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O1[y], (ti >> 14), start_idx);

                    ti = ms[2] * (M0[y] >> 12);
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O0[y], (ti >> 14), start_idx + 1);

                    ti = ms[2] * (M1[y] >> 12);
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O1[y], (ti >> 14), start_idx + 1);
                }

                if (hasBot)
                {
                    uint32_t ti = ms[1] * (M0[y] >> 12);
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O0[y], (ti >> 14), start_idx + outstride);

                    ti = ms[1] * (M1[y] >> 12);
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O1[y], (ti >> 14), start_idx + outstride);

                    ti = ms[3] * (M0[y] >> 12);
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O0[y], (ti >> 14), start_idx + outstride + 1);

                    ti = ms[3] * (M1[y] >> 12);
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O1[y], (ti >> 14), start_idx + outstride + 1);

                }
            }
            else
            {
                if (hasUp)
                {
                    uint32_t ti = ms[0] * (M0[y] >> 12);
                    // convert to 2.14
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O0[y], (ti >> 14), start_idx);

                    ti = ms[0] * (M1[y] >> 12);
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O1[y], (ti >> 14), start_idx);
                }

                if (hasBot)
                {
                    uint32_t ti = ms[1] * (M0[y] >> 12);
                    // convert to 2.14
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O0[y], (ti >> 14), start_idx + outstride);

                    ti = ms[1] * (M1[y] >> 12);
                    locateHistBin_evb(bin0, bin1, bin2, bin3, bin4, bin5, O1[y], (ti >> 14), start_idx + outstride);

                }
            }
        }
    }
}

void triangleFilt1x_c_row(uint32_t* in, uint16_t* out, int32_t w, uint32_t i_index, int32_t p, int32_t pad, int32_t outstride)
{
    uint32_t *Il, *Im, *Ir;

    uint32_t vl, vm, vr, ret;

    for (int32_t j = pad; j < w + pad; j++)
    {
        Il = Im = Ir = in + j;
        Il--;
        Ir++;

        vl = *Il;
        vm = *Im;
        vr = *Ir;

        ret = (uint32_t)(vl + p*vm + vr);
        ret = ret >> 16;
        int32_t offset = (j - pad + i_index*outstride);
        out[offset] = ((uint16_t)ret);
    }
}

void triangleFilt1_c_row(uint16_t* in, uint16_t* out, int32_t h, int32_t w, int32_t instride, int32_t outstride, uint32_t *T, int32_t p, int32_t pad)
{
    uint32_t nrm = (p + 2) * (p + 2);
    uint16_t lZ = count_leading_zero_s(nrm);
    nrm = fxrecip22(nrm << lZ) >> (32 - lZ - 1);//nrm is U32

    int32_t i, j;
    uint16_t *Iu, *Im, *Ib;
    uint16_t up, middle, bot, temp, nrm_v;
    nrm_v = (uint16_t)(nrm >> 16);
    uint32_t ret;

    int32_t wstride = instride;

    for (i = 0; i < h; i++)
    {
        Iu = Im = Ib = in + i*wstride - pad;
        Iu -= wstride;
        Ib += wstride;

        for (j = 0; j < w + 2 * pad; j++)
        {
            up = Iu[j];
            middle = Im[j];
            bot = Ib[j];

            temp = (uint16_t)(up + middle*p + bot);
            ret = ((uint32_t)temp)*((uint32_t)nrm_v);
            T[j] = ret;

        }
        triangleFilt1x_c_row(T, out, w, i, p, pad, outstride);
    }
}

uint16_t lTableData[1064] = {
    0, 214, 428, 642, 856, 1070, 1284, 1498, 1712, 1927, 2134, 2328, 2511, 2684,
    2848, 3005, 3155, 3299, 3437, 3570, 3698, 3823, 3943, 4060, 4174, 4284, 4391, 4496, 4598, 4698, 4796, 4891, 4985,
    5076, 5166, 5253, 5340, 5424, 5507, 5589, 5669, 5748, 5826, 5902, 5978, 6052, 6125, 6197, 6268, 6338, 6407, 6475,
    6542, 6609, 6674, 6739, 6803, 6866, 6929, 6991, 7052, 7112, 7172, 7231, 7290, 7348, 7405, 7462, 7518, 7573, 7628,
    7683, 7737, 7791, 7844, 7896, 7948, 8000, 8051, 8102, 8152, 8202, 8252, 8301, 8350, 8398, 8446, 8494, 8541, 8588,
    8634, 8681, 8727, 8772, 8817, 8862, 8907, 8951, 8995, 9039, 9082, 9125, 9168, 9210, 9253, 9295, 9336, 9378, 9419,
    9460, 9500, 9541, 9581, 9621, 9661, 9700, 9740, 9779, 9817, 9856, 9894, 9933, 9970, 10008, 10046, 10083, 10120,
    10157, 10194, 10231, 10267, 10303, 10339, 10375, 10411, 10446, 10481, 10517, 10551, 10586, 10621, 10655, 10690,
    10724, 10758, 10792, 10825, 10859, 10892, 10925, 10958, 10991, 11024, 11057, 11089, 11121, 11154, 11186, 11218,
    11249, 11281, 11313, 11344, 11375, 11406, 11437, 11468, 11499, 11530, 11560, 11591, 11621, 11651, 11681, 11711,
    11741, 11771, 11800, 11830, 11859, 11888, 11917, 11947, 11975, 12004, 12033, 12062, 12090, 12119, 12147, 12175,
    12203, 12231, 12259, 12287, 12315, 12342, 12370, 12397, 12425, 12452, 12479, 12506, 12533, 12560, 12587, 12614,
    12640, 12667, 12694, 12720, 12746, 12772, 12799, 12825, 12851, 12877, 12902, 12928, 12954, 12979, 13005, 13030,
    13056, 13081, 13106, 13131, 13156, 13181, 13206, 13231, 13256, 13281, 13305, 13330, 13354, 13379, 13403, 13427,
    13452, 13476, 13500, 13524, 13548, 13572, 13595, 13619, 13643, 13666, 13690, 13714, 13737, 13760, 13784, 13807,
    13830, 13853, 13876, 13899, 13922, 13945, 13968, 13991, 14013, 14036, 14059, 14081, 14104, 14126, 14148, 14171,
    14193, 14215, 14237, 14260, 14282, 14304, 14326, 14347, 14369, 14391, 14413, 14434, 14456, 14478, 14499, 14521,
    14542, 14563, 14585, 14606, 14627, 14648, 14670, 14691, 14712, 14733, 14754, 14775, 14795, 14816, 14837, 14858,
    14878, 14899, 14919, 14940, 14960, 14981, 15001, 15022, 15042, 15062, 15082, 15103, 15123, 15143, 15163, 15183,
    15203, 15223, 15243, 15263, 15282, 15302, 15322, 15341, 15361, 15381, 15400, 15420, 15439, 15459, 15478, 15497,
    15517, 15536, 15555, 15575, 15594, 15613, 15632, 15651, 15670, 15689, 15708, 15727, 15746, 15765, 15783, 15802,
    15821, 15840, 15858, 15877, 15895, 15914, 15933, 15951, 15969, 15988, 16006, 16025, 16043, 16061, 16079, 16098,
    16116, 16134, 16152, 16170, 16188, 16206, 16224, 16242, 16260, 16278, 16296, 16314, 16332, 16349, 16367, 16385,
    16402, 16420, 16438, 16455, 16473, 16490, 16508, 16525, 16543, 16560, 16577, 16595, 16612, 16629, 16647, 16664,
    16681, 16698, 16715, 16732, 16750, 16767, 16784, 16801, 16818, 16835, 16851, 16868, 16885, 16902, 16919, 16936,
    16952, 16969, 16986, 17002, 17019, 17036, 17052, 17069, 17085, 17102, 17118, 17135, 17151, 17168, 17184, 17200,
    17217, 17233, 17249, 17266, 17282, 17298, 17314, 17330, 17346, 17363, 17379, 17395, 17411, 17427, 17443, 17459,
    17475, 17491, 17506, 17522, 17538, 17554, 17570, 17586, 17601, 17617, 17633, 17648, 17664, 17680, 17695, 17711,
    17727, 17742, 17758, 17773, 17789, 17804, 17819, 17835, 17850, 17866, 17881, 17896, 17912, 17927, 17942, 17957,
    17973, 17988, 18003, 18018, 18033, 18048, 18064, 18079, 18094, 18109, 18124, 18139, 18154, 18169, 18184, 18198,
    18213, 18228, 18243, 18258, 18273, 18288, 18302, 18317, 18332, 18346, 18361, 18376, 18390, 18405, 18420, 18434,
    18449, 18463, 18478, 18493, 18507, 18522, 18536, 18550, 18565, 18579, 18594, 18608, 18622, 18637, 18651, 18665,
    18680, 18694, 18708, 18722, 18737, 18751, 18765, 18779, 18793, 18807, 18821, 18836, 18850, 18864, 18878, 18892,
    18906, 18920, 18934, 18948, 18962, 18975, 18989, 19003, 19017, 19031, 19045, 19059, 19072, 19086, 19100, 19114,
    19127, 19141, 19155, 19168, 19182, 19196, 19209, 19223, 19237, 19250, 19264, 19277, 19291, 19304, 19318, 19331,
    19345, 19358, 19372, 19385, 19399, 19412, 19425, 19439, 19452, 19465, 19479, 19492, 19505, 19519, 19532, 19545,
    19558, 19572, 19585, 19598, 19611, 19624, 19637, 19651, 19664, 19677, 19690, 19703, 19716, 19729, 19742, 19755,
    19768, 19781, 19794, 19807, 19820, 19833, 19846, 19859, 19871, 19884, 19897, 19910, 19923, 19936, 19948, 19961,
    19974, 19987, 20000, 20012, 20025, 20038, 20050, 20063, 20076, 20088, 20101, 20114, 20126, 20139, 20151, 20164,
    20177, 20189, 20202, 20214, 20227, 20239, 20252, 20264, 20277, 20289, 20301, 20314, 20326, 20339, 20351, 20363,
    20376, 20388, 20400, 20413, 20425, 20437, 20450, 20462, 20474, 20486, 20499, 20511, 20523, 20535, 20547, 20560,
    20572, 20584, 20596, 20608, 20620, 20632, 20644, 20656, 20668, 20681, 20693, 20705, 20717, 20729, 20741, 20753,
    20765, 20776, 20788, 20800, 20812, 20824, 20836, 20848, 20860, 20872, 20884, 20895, 20907, 20919, 20931, 20943,
    20954, 20966, 20978, 20990, 21001, 21013, 21025, 21037, 21048, 21060, 21072, 21083, 21095, 21107, 21118, 21130,
    21141, 21153, 21165, 21176, 21188, 21199, 21211, 21222, 21234, 21245, 21257, 21268, 21280, 21291, 21303, 21314,
    21326, 21337, 21349, 21360, 21371, 21383, 21394, 21405, 21417, 21428, 21440, 21451, 21462, 21473, 21485, 21496,
    21507, 21519, 21530, 21541, 21552, 21564, 21575, 21586, 21597, 21608, 21620, 21631, 21642, 21653, 21664, 21675,
    21686, 21697, 21709, 21720, 21731, 21742, 21753, 21764, 21775, 21786, 21797, 21808, 21819, 21830, 21841, 21852,
    21863, 21874, 21885, 21896, 21907, 21918, 21929, 21939, 21950, 21961, 21972, 21983, 21994, 22005, 22016, 22026,
    22037, 22048, 22059, 22070, 22080, 22091, 22102, 22113, 22123, 22134, 22145, 22156, 22166, 22177, 22188, 22198,
    22209, 22220, 22230, 22241, 22252, 22262, 22273, 22284, 22294, 22305, 22315, 22326, 22336, 22347, 22358, 22368,
    22379, 22389, 22400, 22410, 22421, 22431, 22442, 22452, 22463, 22473, 22484, 22494, 22505, 22515, 22525, 22536,
    22546, 22557, 22567, 22577, 22588, 22598, 22608, 22619, 22629, 22639, 22650, 22660, 22670, 22681, 22691, 22701,
    22712, 22722, 22732, 22742, 22753, 22763, 22773, 22783, 22793, 22804, 22814, 22824, 22834, 22844, 22855, 22865,
    22875, 22885, 22895, 22905, 22915, 22926, 22936, 22946, 22956, 22966, 22976, 22986, 22996, 23006, 23016, 23026,
    23036, 23046, 23056, 23066, 23076, 23086, 23096, 23106, 23116, 23126, 23136, 23146, 23156, 23166, 23176, 23186,
    23196, 23206, 23216, 23225, 23235, 23245, 23255, 23265, 23275, 23285, 23295, 23304, 23314, 23324, 23334, 23344,
    23353, 23363, 23373, 23383, 23393, 23402, 23412, 23422, 23432, 23441, 23451, 23461, 23470, 23480, 23490, 23500,
    23509, 23519, 23529, 23538, 23548, 23558, 23567, 23577, 23586, 23596, 23606, 23615, 23625, 23635, 23644, 23654,
    23663, 23673, 23682, 23692, 23702, 23711, 23721, 23730, 23740, 23749, 23759, 23768, 23778, 23787, 23797, 23806,
    23816, 23825, 23835, 23844, 23853, 23863, 23872, 23882, 23891, 23901, 23910, 23919, 23929, 23938, 23948, 23957,
    23966, 23976, 23985, 23994, 24004, 24013, 24022, 24032, 24041, 24050, 24060, 24069, 24078, 24088, 24097, 24106,
    24115, 24125, 24134, 24143, 24152, 24162, 24171, 24180, 24189, 24199, 24208, 24217, 24226, 24235, 24245, 24254,
    24263, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272,
    24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272,
    24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272, 24272
};


void apu_aggcf_rgb2luv(
    uint32_t *rgbImgIn,
    uint16_t *lImgOut,
    uint16_t *uImgOut,
    uint16_t *vImgOut,
    uint16_t strideIn,
    uint16_t strideOut,
    uint16_t Width,
    uint16_t Height
    )
{
    uint16_t minu = 5440;
    uint16_t minv = 8109;
    int32_t un = 84273;
    int32_t vn = 398996;
    
    uint8_t mr[3] = { 110, 57, 5 };
    uint8_t mg[3] = { 87, 182, 33 };
    uint8_t mb[3] = { 45, 18, 241 };

    uint32_t xZ, yZ;
    uint32_t x, y, z;
    int32_t l;
    
    uint8_t r, g, b;
    uint32_t d;
    
    for (int h = 0; h < Height; h++)
    {
        for (int w = 0; w < Width; w++)
        {
            // convert RGB to LUV
            r = uint8_t(rgbImgIn[w] >> 24);
            g = uint8_t(rgbImgIn[w] >> 16);
            b = uint8_t(rgbImgIn[w] >> 8);

            x = mr[0]*r + mg[0]*g + mb[0]*b;
            y = mr[1]*r + mg[1]*g + mb[1]*b;
            z = mr[2]*r + mg[2]*g + mb[2]*b;

            uint32_t y_temp = y >> 6;
            y_temp = (y_temp <= 1063)?y_temp:1063;
            l = lTableData[y_temp];

            lImgOut[w] = (uint16_t)(l >> 2);

            d = 1 + x + 15 * y + 3 * z;

            xZ = (x << 16) / d;
            yZ = (y << 16) / d;

            uImgOut[w] = (uint16_t)( (((int32_t)(l*(26 * xZ - un))) >> 17) + minu);
            int32_t tmp = ((int32_t)(117 * yZ - vn)) >> 1;
            vImgOut[w] = (uint16_t)(((l*tmp) >> 17) + minv);
        }
        rgbImgIn += strideIn;
        lImgOut  += strideOut;
        uImgOut  += strideOut;
        vImgOut  += strideOut;
    }
}

uint32_t count_leading_zero_16_s(uint16_t x)
{
    uint32_t n;

    if (x == 0) return(16);
    n = 1;
    if ((x >> 8) == 0) { n = n + 8; x = x << 8; }
    if ((x >> 12) == 0) { n = n + 4; x = x << 4; }
    if ((x >> 14) == 0) { n = n + 2; x = x << 2; }
    n = n - (uint32_t)(x >> 15);
    return n;
}


void aggcf_mag_norm_fix(uint16_t *mag, uint16_t *filtered_mag, int stride, int width, int height, float normconst)
{
    uint16_t normconst_i = (uint32_t)(normconst*pow(2, 14));

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            uint16_t fm = filtered_mag[i*stride + j]; //2.14
            uint16_t m = mag[i*stride + j];

            fm = fm + normconst_i;
            uint16_t lZ = count_leading_zero_16_s(fm);

            fm = fxrecip12(fm << lZ) >> (16 - lZ - 1); //U16
            m = (uint16_t)(((uint32_t)m * (uint32_t)fm) >> 2); //U2.14
            filtered_mag[i*stride + j] = m;
        }
    }
}


void aggcf_mag_norm(uint16_t *mag, uint16_t *filtered_mag, int stride, int width, int height, float normconst)
{

    for(int i = 0; i < height; i ++)
    {
        for(int j = 0; j < width; j++)
        {
            float fm = (float)filtered_mag[i*stride + j]/pow(2,14);
            float m = (float)mag[i*stride+j]/pow(2, 14);

            m = m/(fm+normconst);
            filtered_mag[i*stride + j] = (uint16_t)(m*pow(2,14));
        }
    }

}

// scale can only be 1, 2, 3 which means downscale by 1/4, 1/16, 1/64
void aggcf_pyramid_downscale_ref(uint16_t *in, uint16_t *out, int width, int height, int scale)
{
    int step = 1<<scale;
    for(int i = 0; i < height; i+=step)
    {
        for(int j = 0; j < width; j+=step)
        {
            uint32_t sum = 0;
            for(int k = i; k < i+step; k++)
            {
                int temp = 0; 
                for(int l = j; l < j+step; l++)
                {
                   temp += in[k*width+l];
                }  
                sum += temp >>scale;                
            }  
            out[i/step*width/step+j/step] = (sum>>scale);
        }
    } 
}



