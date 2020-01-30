/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2015-2016 Freescale Semiconductor
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

#ifndef BLOB_HPP
#define BLOB_HPP

#include <icp_data.h>
#include <sumat.hpp>
#include <oal.h>
#include <stdio.h>
#include <string>

#define ERROR_CHECKER(expr) if(!(expr)) { printf("%s : %s [%d]\n", __FILE__, __PRETTY_FUNCTION__, __LINE__); }

template <typename Dtype>
class Blob {
public:
   Blob() {}
   int Alloc(int height, int width, int32_t type);
   void Dump(FILE *fp, int type); // 0: bottom; 1: top
   void Dump2(FILE *fp, int type);
   void Show(int type, int verbose = 0); // 0: bottom; 1: top
   ~Blob();

   std::string blob_name_;
   vsdk::SUMat data_handle_;

   vsdk::SUMat bottom_data_desc_;
   vsdk::SUMat top_data_desc_;
   vsdk::SUMat weight_data_desc_;

   vsdk::SUMat bottom_data_desc_splitted_[2];
   vsdk::SUMat top_data_desc_splitted_[2];

   int bottom_data_bw_;
   int bottom_data_bs_;
   int bottom_data_bd_;
   int bottom_data_bh_;
   int bottom_data_ih_;

   int top_data_bw_;
   int top_data_bs_;
   int top_data_bd_;
};

template <typename Dtype>
Blob<Dtype>::~Blob() {
}

template <typename Dtype>
int Blob<Dtype>::Alloc(int height, int width, int32_t type) {
   int reval;

   data_handle_ = vsdk::SUMat(height, width, type);
   if (data_handle_.empty())
   {
      printf("BLOB cannot allocate memory\n");
      reval = 1;
   }
   else
   {
      vsdk::SMat tempMat = data_handle_.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      memset(tempMat.data, 0, width * height * (type == VSDK_CV_8SC1 ? sizeof(int8_t) : sizeof(int16_t)));
   }

   return reval;
}

template <typename Dtype>
void Blob<Dtype>::Dump(FILE *fp, int type) 
{
   int bs, bw, bd;
   vsdk::SMat tempMat;
   if (type == 0) // bottom
   {
      bs = bottom_data_bs_;
      bw = bottom_data_bw_;
      bd = bottom_data_bd_;

      tempMat = bottom_data_desc_.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   }
   else
   {
      bs = top_data_bs_;
      bw = top_data_bw_;
      bd = top_data_bd_;

      tempMat = top_data_desc_.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   }
  

   Dtype *p_data = (Dtype*)tempMat.data;
   for (int i = 0; i < tempMat.rows; i++)
   {
      for (int j = 0; j < bd; j++)
      {
         for (int k = 0; k < (type == 0 ? bs : bw); k++)
         {
            fprintf(fp, "%d, ", p_data[i*tempMat.step[0]+j*bs+k]);
         }
      }
      fprintf(fp, "\n");
   }
}


template <typename Dtype>
void Blob<Dtype>::Dump2(FILE *fp, int type) 
{
   int bs, bw, bd;
   vsdk::SMat tempMat;
   if (type == 0) // bottom
   {
      bs = bottom_data_bs_;
      bw = bottom_data_bw_;
      bd = bottom_data_bd_;

      tempMat = bottom_data_desc_.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   }
   else
   {
      bs = top_data_bs_;
      bw = top_data_bw_;
      bd = top_data_bd_;

      tempMat = bottom_data_desc_.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   }
  

   int16_t *p_data = (int16_t*)tempMat.data;
   for (int j = 0; j < bd; j++)
   {
      fprintf(fp, "%d, ", p_data[j]);
   }
}



template <typename Dtype>
void Blob<Dtype>::Show(int type, int verbose)
{
   char type_string[25];
   int bs, bw, bd;
   vsdk::SMat tempMat;

   if (type == 0) // bottom
   {
      sprintf(type_string, "Bottom");
      
      bs = bottom_data_bs_;
      bw = bottom_data_bw_;
      bd = bottom_data_bd_;

      tempMat = bottom_data_desc_.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   }
   else
   {
      sprintf(type_string, "Top");
      
      bs = top_data_bs_;
      bw = top_data_bw_;
      bd = top_data_bd_;

      tempMat = top_data_desc_.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   }

   int offset = (int)((unsigned long)tempMat.data - (unsigned long)data_handle_.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data);
   int top  = offset / tempMat.step[0];
   int left = offset % tempMat.step[0];
   printf("/****** %s %s ******/\n", type_string, blob_name_.c_str());
   printf("data_    :    0x%x size %d [%d x %d = %d]\n", 
      (Dtype*)data_handle_.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data, data_handle_.cols * data_handle_.rows, tempMat.step[0], tempMat.rows, 
      tempMat.step[0] * tempMat.rows);
   printf("         :    0x%x\n", (Dtype*)tempMat.data);
   if (verbose)
   {
      int8_t *pData = (int8_t*)tempMat.data;
      for (int i = 0; i < 10; i++)
      {
         printf("              ");
         for (int j = 0; j < 10; j++)
         {
            printf("0x%x ", pData[i*10+j]);
         }
         printf("\n");
      }
   }
   printf("         :    = top %d left %d\n", top, left);
   printf("Span     :    %d [%d x %d + %d]\n", 
      tempMat.step[0],
      bs, 64, tempMat.step[0] - bs*64);
   printf("Width    :    %d\n", tempMat.cols);
   if (type == 0)
      printf("              = %d x %d + %d\n", bs, bd, tempMat.cols-bs*bd);
   else 
      printf("              = %d x %d + %d\n", bs, 64, tempMat.cols-bs*64);
   printf("Height   :    %d\n", tempMat.rows);

   printf("BS       :    %d\n", bs);
   printf("BW       :    %d\n", bw);
   printf("BD       :    %d\n", bd);
   printf("BS*BD    :    %d\n", bs*bd);
   if (type == 0) // bottom
   {
      printf("BH       :    %d\n", bottom_data_bh_);
      printf("IH       :    %d\n", bottom_data_ih_);
   }
}

#endif /* BLOB_HPP */
