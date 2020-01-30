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


#ifndef AGGCFACF_H
#define AGGCFACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define AGG_ACF_BIRESIZE_PASS_K                   agg_acf_biresize_pass
#define AGG_ACF_BIRESIZE_PASS_KN                  XSTR(AGG_ACF_BIRESIZE_PASS_K)
#define AGG_ACF_BIRESIZE_HORI_R1_K                agg_acf_biresize_hori_r1
#define AGG_ACF_BIRESIZE_HORI_R1_KN               XSTR(AGG_ACF_BIRESIZE_HORI_R1_K)
#define AGG_ACF_BIRESIZE_HORI_R2_K                agg_acf_biresize_hori_r2
#define AGG_ACF_BIRESIZE_HORI_R2_KN               XSTR(AGG_ACF_BIRESIZE_HORI_R2_K)
#define AGG_ACF_BIRESIZE_HORI_R3_K                agg_acf_biresize_hori_r3
#define AGG_ACF_BIRESIZE_HORI_R3_KN               XSTR(AGG_ACF_BIRESIZE_HORI_R3_K)
#define AGG_ACF_BIRESIZE_VERTI_R1_K               agg_acf_biresize_verti_r1
#define AGG_ACF_BIRESIZE_VERTI_R1_KN              XSTR(AGG_ACF_BIRESIZE_VERTI_R1_K)
#define AGG_ACF_BIRESIZE_VERTI_R2_K               agg_acf_biresize_verti_r2
#define AGG_ACF_BIRESIZE_VERTI_R2_KN              XSTR(AGG_ACF_BIRESIZE_VERTI_R2_K)
#define AGG_ACF_BIRESIZE_VERTI_R3_K               agg_acf_biresize_verti_r3
#define AGG_ACF_BIRESIZE_VERTI_R3_KN              XSTR(AGG_ACF_BIRESIZE_VERTI_R3_K)
#define AGG_ACF_FLT_K                             agg_acf_flt
#define AGG_ACF_FLT_KN                            XSTR(AGG_ACF_FLT_K)
#define AGG_ACF_FLT_NORM_K                        agg_acf_flt_norm
#define AGG_ACF_FLT_NORM_KN                       XSTR(AGG_ACF_FLT_NORM_K)
#define AGG_ACF_LUV_K                             agg_acf_luv
#define AGG_ACF_LUV_KN                            XSTR(AGG_ACF_LUV_K)
#define AGG_ACF_TRIFLT_K                          agg_acf_triflt
#define AGG_ACF_TRIFLT_KN                         XSTR(AGG_ACF_TRIFLT_K)
#define AGG_ACF_K                                 agg_acf
#define AGG_ACF_KN                                XSTR(AGG_ACF_K)
#define AGG_ACFHIST_NT_1X1_K                      agg_acfhist_nt_1x1
#define AGG_ACFHIST_NT_1X1_KN                     XSTR(AGG_ACFHIST_NT_1X1_K)
#define AGG_ACFHIST_NT_2X2_K                      agg_acfhist_nt_2x2
#define AGG_ACFHIST_NT_2X2_KN                     XSTR(AGG_ACFHIST_NT_2X2_K)
#define AGG_ACFHIST_NT_3X3_K                      agg_acfhist_nt_3x3
#define AGG_ACFHIST_NT_3X3_KN                     XSTR(AGG_ACFHIST_NT_3X3_K)
#define AGG_ACFHIST_NT_4X4_K                      agg_acfhist_nt_4x4
#define AGG_ACFHIST_NT_4X4_KN                     XSTR(AGG_ACFHIST_NT_4X4_K)
#define AGG_ACFHIST_TI_2X2_K                      agg_acfhist_ti_2x2
#define AGG_ACFHIST_TI_2X2_KN                     XSTR(AGG_ACFHIST_TI_2X2_K)
#define AGG_ACFHIST_TI_3X3_K                      agg_acfhist_ti_3x3
#define AGG_ACFHIST_TI_3X3_KN                     XSTR(AGG_ACFHIST_TI_3X3_K)
#define AGG_ACFHIST_TI_4X4_K                      agg_acfhist_ti_4x4
#define AGG_ACFHIST_TI_4X4_KN                     XSTR(AGG_ACFHIST_TI_4X4_K)
#define INPUT                           "INPUT"
#define OUTPUT                          "OUTPUT"
#define TEMP_ROW                        "TEMP_ROW"
#define PARAMS                          "PARAMS"
#define OUTPUTL                         "OUTPUTL"
#define OUTPUTU                         "OUTPUTU"
#define OUTPUTV                         "OUTPUTV"
#define LUV_LUT                         "LUV_LUT"
#define t_lastline                      "t_lastline"
#define u_lastline                      "u_lastline"
#define INPUTL                          "INPUTL"
#define INPUTU                          "INPUTU"
#define INPUTV                          "INPUTV"
#define OUTPUTM                         "OUTPUTM"
#define OUTPUTO                         "OUTPUTO"
#define OUTPUTGx                        "OUTPUTGx"
#define OUTPUTGy                        "OUTPUTGy"
#define OUTPUTM2                        "OUTPUTM2"
#define ATANTABLE                       "ATANTABLE"
#define INPUTM                          "INPUTM"
#define INPUTO                          "INPUTO"
#define OFFSET                          "OFFSET"
#define OUTPUTH1                        "OUTPUTH1"
#define OUTPUTH2                        "OUTPUTH2"
#define OUTPUTH3                        "OUTPUTH3"
#define OUTPUTH4                        "OUTPUTH4"
#define OUTPUTH5                        "OUTPUTH5"
#define OUTPUTH6                        "OUTPUTH6"
#define OUTPUTM0                        "OUTPUTM0"
#define OUTPUTM1                        "OUTPUTM1"
#define OUTPUTO0                        "OUTPUTO0"
#define OUTPUTO1                        "OUTPUTO1"

extKernelInfoDecl(AGG_ACF_FLT_K);
extKernelInfoDecl(AGG_ACF_LUV_K);
extKernelInfoDecl(AGG_ACF_TRIFLT_K);
extKernelInfoDecl(AGG_ACF_K);
extKernelInfoDecl(AGG_ACFHIST_NT_1X1_K);
extKernelInfoDecl(AGG_ACFHIST_NT_2X2_K);
extKernelInfoDecl(AGG_ACFHIST_NT_3X3_K);
extKernelInfoDecl(AGG_ACFHIST_NT_4X4_K);
extKernelInfoDecl(AGG_ACFHIST_TI_2X2_K);
extKernelInfoDecl(AGG_ACFHIST_TI_3X3_K);
extKernelInfoDecl(AGG_ACFHIST_TI_4X4_K);
extKernelInfoDecl(AGG_ACF_FLT_NORM_K);
extKernelInfoDecl(AGG_ACF_BIRESIZE_PASS_K);
extKernelInfoDecl(AGG_ACF_BIRESIZE_VERTI_R1_K);
extKernelInfoDecl(AGG_ACF_BIRESIZE_VERTI_R2_K);
extKernelInfoDecl(AGG_ACF_BIRESIZE_VERTI_R3_K);
extKernelInfoDecl(AGG_ACF_BIRESIZE_HORI_R1_K);
extKernelInfoDecl(AGG_ACF_BIRESIZE_HORI_R2_K);
extKernelInfoDecl(AGG_ACF_BIRESIZE_HORI_R3_K);


#endif /* ACF_KERNEL_METADATA */

#endif /* AGGCFACF_H */
