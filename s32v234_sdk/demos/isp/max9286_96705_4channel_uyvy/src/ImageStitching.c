#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include "ImageStitching.h"






#define BASE_ADJUSTMENT (128)
#define CAMERA_NUM (4)
//int gains[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/*******************************************************************************/





unsigned int sum_q1 = 256;
unsigned int sum_q3 = 256;
unsigned int sum_q4 = 256;


////int lumaPixelAvg = 0;
int lumaPixelCnt = 0;
int lumaPixelSum = 0;

void Resize_Interpolation_SingleView(unsigned short *resize_lut,unsigned int **remap_lut, unsigned char * dout,int Nwidth,int Nheight)
{
	unsigned short i_int_1, j_int_1,i_int_2, j_int_2;
	unsigned short a1a2_1,a3a4_1,a1a2_2,a3a4_2;
	unsigned char a1_1, a2_1, a3_1, a4_1,a1_2, a2_2, a3_2, a4_2;

	unsigned char *remap_ptr_y1;
	unsigned char *remap_ptr_uv1;
	unsigned char *remap_ptr_y2;
	unsigned char *remap_ptr_uv2;

	int i,j;
	int currentLine = 1;
	int nextLine = 1;
	int row = 0;
        for (i = 0; i<Nheight; i++)
	{
		for (j = 0; j<Nwidth; j=j+2)
		{
			j_int_1 = *(resize_lut + i*Nwidth*4 + j*4) ;
			i_int_1 = *(resize_lut + i*Nwidth*4 + j*4 + 1) ;
			a1a2_1    = *(resize_lut + i*Nwidth*4 + j*4 + 2) ;
			a3a4_1    = *(resize_lut + i*Nwidth*4 + j*4 + 3) ;

			a1_1 = a1a2_1 >> 8;
			a2_1 = a1a2_1 % 256;
			a3_1 = a3a4_1 >> 8;
			a4_1 = a3a4_1 % 256;


			row = j_int_1;
			if(((i^row)&1) == 1)
			{
				currentLine = 1;
				nextLine = 1;
			}
			else
			{
				currentLine = 0;
				nextLine = 2;
			}		
			remap_ptr_y1=(unsigned char *)remap_lut[row+currentLine]+1;
			remap_ptr_uv1=(unsigned char *)remap_lut[row+currentLine];
			remap_ptr_y2=(unsigned char *)remap_lut[row+nextLine]+1;
			remap_ptr_uv2=(unsigned char *)remap_lut[row+nextLine];
			
			if(i_int_1%2==0)
			{
				i_int_1=i_int_1 & 0xfffe;
				*dout++= (a1_1*(*(remap_ptr_uv1 + i_int_1*2)) + a2_1*(*(remap_ptr_uv1 + (i_int_1)*2)) +
						   a3_1*(*(remap_ptr_uv2 + i_int_1*2)) + a4_1*(*(remap_ptr_uv2 + (i_int_1)*2)))>>8;
			}
			else
			{
				i_int_1=i_int_1 | 0x01;
				*dout++= (a1_1*(*(remap_ptr_uv1 + i_int_1*2+2)) + a2_1*(*(remap_ptr_uv1 + (i_int_1)*2+2)) +
						   a3_1*(*(remap_ptr_uv2 + i_int_1*2+2)) + a4_1*(*(remap_ptr_uv2 + (i_int_1)*2+2)))>>8;
			}

			*dout++= (a1_1*(*(remap_ptr_y1 + i_int_1*2)) + a2_1*(*(remap_ptr_y1 + (i_int_1+1)*2)) +
					  a3_1*(*(remap_ptr_y2 + i_int_1*2)) + a4_1*(*(remap_ptr_y2 + (i_int_1+1)*2)))>>8;




			j_int_2 = *(resize_lut + i*Nwidth*4 + j*4 + 4) ;
			i_int_2 = *(resize_lut + i*Nwidth*4 + j*4 + 4+ 1) ;
			a1a2_2	  = *(resize_lut + i*Nwidth*4 + j*4 + 4 + 2) ;
			a3a4_2	  = *(resize_lut + i*Nwidth*4 + j*4 + 4 + 3) ;
			

			a1_2 = a1a2_2 >> 8;
			a2_2 = a1a2_2 % 256;
			a3_2 = a3a4_2 >> 8;
			a4_2 = a3a4_2 % 256;

			row = j_int_2;
			if(((i^row)&1) == 1)
			{
				currentLine = 1;
				nextLine = 1;
			}
			else
			{
				currentLine = 0;
				nextLine = 2;
			}
			
			remap_ptr_y1=(unsigned char *)remap_lut[row+currentLine]+1;
			remap_ptr_uv1=(unsigned char *)remap_lut[row+currentLine];
			remap_ptr_y2=(unsigned char *)remap_lut[row+nextLine]+1;
			remap_ptr_uv2=(unsigned char *)remap_lut[row+nextLine];
			
			if(i_int_2%2==1)
			{
				i_int_2=i_int_2 | 0x01;
				*dout++= (a1_2*(*(remap_ptr_uv1 + i_int_2*2)) + a2_2*(*(remap_ptr_uv1 + (i_int_2)*2)) +
						   a3_2*(*(remap_ptr_uv2 + i_int_2*2)) + a4_2*(*(remap_ptr_uv2 + (i_int_2)*2)))>>8;
			}
			else
			{
				i_int_2=i_int_2 & 0xfffe;
				*dout++= (a1_2*(*(remap_ptr_uv1 + i_int_2*2+2)) + a2_2*(*(remap_ptr_uv1 + (i_int_2)*2+2)) +
						   a3_2*(*(remap_ptr_uv2 + i_int_2*2+2)) + a4_2*(*(remap_ptr_uv2 + (i_int_2)*2+2)))>>8;
			}
			
			*dout++= (a1_2*(*(remap_ptr_y1 + i_int_2*2)) + a2_2*(*(remap_ptr_y1 + (i_int_2+1)*2)) +
					  a3_2*(*(remap_ptr_y2 + i_int_2*2)) + a4_2*(*(remap_ptr_y2 + (i_int_2+1)*2)))>>8;
			
		}
	}
	
}


void rotate270(unsigned char *src,unsigned char *dst,int width,int height)
{
	int copyBytes = 4;
	int bytesPerLine = width << 1;
	int step = height << 2;
	int offset = bytesPerLine - copyBytes;
	
	unsigned char *dest = dst;
	unsigned char *source = src;
	unsigned char *psrc = NULL;
	unsigned char *pdst[2] = {NULL,NULL};	
	int i,j,k;
	unsigned char temp;

	for(i=0;i<bytesPerLine;i+=copyBytes)
	{
		pdst[1] = dest;
		pdst[0] = dest+(height<<1);
		psrc = source + offset;

		for(j=0;j<height;++j)
		{
			k = j % 2;
			*((unsigned int *)pdst[k]) = *((unsigned int *)psrc);

			if(1==k)
			{
				temp = *(pdst[1]+1);
				*(pdst[1]+1) = *(pdst[0]-1);
				*(pdst[0]-1) = temp;
			}

			pdst[k] += copyBytes;
			psrc +=bytesPerLine;
		}
		
		dest += step;
		source -= copyBytes;

	
	}
}

void Rotate180(unsigned char *src,int width,int height)
{
	int copyBytes = 4;
	int totalBytes = width*height << 1;
	int end = totalBytes >> 1;
	unsigned char *dst = src + totalBytes - copyBytes;
	unsigned int tmp0;					
	unsigned char tmp1;
	int i;
	
	for(i = 0; i < end; i += copyBytes)
	{
		tmp0 = *((unsigned int *)dst);
		*((unsigned int *)dst) = *((unsigned int *)src);
		*((unsigned int *)src) = tmp0;
	
		//\u4ea4\u6362Y\u5206\u91cf
		tmp1 = dst[1];
		dst[1] = dst[3];
		dst[3] = tmp1;
	
		tmp1 = src[1];
		src[1] = src[3];
		src[3] = tmp1;
	
		dst -= copyBytes;
		src += copyBytes;
	
		//watchdog();
	}
}


void Rotate90(unsigned char *src,unsigned char *dst,int width,int height)
{
	int copyBytes = 4;
	int bytesPerLine = width << 1;
	int step = height << 2;
	int offset = (height-1)*bytesPerLine;	
	unsigned char *dest = dst;
	unsigned char *source = src;
	unsigned char *psrc = NULL;
	unsigned char *pdst[2] = {NULL,NULL};	
	int i,j,k;
	unsigned char temp;
	for(i=0;i<bytesPerLine;i+=copyBytes)
	{
		pdst[0] = dest;
		pdst[1] = dest+(height<<1);
		psrc = source + offset;
		for(j=0;j<height;++j)
		{
			k = j % 2;
			*((unsigned int *)pdst[k]) = *((unsigned int *)psrc);

			if(1==k)
			{
				temp = *(pdst[0]-1);
				*(pdst[0]-1) = *(pdst[1]+1);
				*(pdst[1]+1) = temp;
			}
			pdst[k] += copyBytes;
			psrc -=bytesPerLine;
		}		
		dest += step;
		source += copyBytes;
	}
}

#if 1
void ReadOriginalImage(void)
{
		FILE* fp = NULL;
		printf("read  image begin\n");
		if((fp = fopen("/data/front.yuv", "rb")) == NULL)
		{
			printf("\nFail to fopen front.yuv\n");
		}
		else
		{	
			front_pd = (unsigned char* )malloc(sizeof(unsigned char)*PIC_ORISIZE);
			fread(front_pd, sizeof(unsigned char), PIC_ORISIZE, fp);			
			fflush(fp);
			fclose(fp);
			printf("front_p.yuv success !\n");	
		}
	
		if ((fp = fopen("/data/back.yuv", "rb")) == NULL)
		{
			printf("Fail to fopen back.yuv\n");
		}
		else
		{		
			back_pd = (unsigned char* )malloc(sizeof(unsigned char)*PIC_ORISIZE);
			fread(back_pd, sizeof(unsigned char), PIC_ORISIZE, fp);			
			fflush(fp);
			fclose(fp);
			printf("back_p.yuv success !\n");	
		}
			
		if ((fp = fopen("/data/left.yuv", "rb")) == NULL)
		{
			printf("Fail to fopen left.yuv\n");
		}
		else
		{		
			left_pd = (unsigned char* )malloc(sizeof(unsigned char)*PIC_ORISIZE);
			fread(left_pd, sizeof(unsigned char), PIC_ORISIZE, fp);			
			fflush(fp);
			fclose(fp);
			printf("left_p.yuv success !\n");	
		}

	if ((fp = fopen("/data/right.yuv", "rb")) == NULL)
	{
		printf("Fail to fopen right.yuv\n");
	}
	else
	{		
		right_pd = (unsigned char* )malloc(sizeof(unsigned char)*PIC_ORISIZE);
		fread(right_pd, sizeof(unsigned char), PIC_ORISIZE, fp);			
		fflush(fp);
		fclose(fp);
		printf("right_p.yuv success !\n");	
	}
	if ((fp = fopen("/data/leftbar.yuv", "rb")) == NULL)
	{
		printf("Fail to fopen leftbar.yuv\n");
	}
	else
	{	
		leftbar_pd = (unsigned char* )malloc(sizeof(unsigned char)*1080*256*2);
		fread(leftbar_pd, sizeof(unsigned char), 1080*256*2, fp);			
		fflush(fp);
		fclose(fp);
		printf("leftbar_p.yuv success !\n");	
	}
		
	if ((fp = fopen("/data/bottombar.yuv", "rb")) == NULL)
	{
		printf("Fail to fopen bottombar.yuv\n");
	}
	else
	{
		bottombar_pd = (unsigned char* )malloc(sizeof(unsigned char)*SINGLE_VIEW_WIDTH*320*2);
		fread(bottombar_pd, sizeof(unsigned char), SINGLE_VIEW_WIDTH*320*2, fp);			
		fflush(fp);
		fclose(fp);
		printf("bottombar_p.yuv success !\n");	
	}


	if ((fp = fopen("/data/carbirdview.yuv", "rb")) == NULL)
	{
		printf("Fail to fopen carbirdview.yuv\n");
	}
	else
	{		
		carbirdview_pd = (unsigned char* )malloc(sizeof(unsigned char)*253*76*2);
		fread(carbirdview_pd, sizeof(unsigned char), 253*76*2, fp);			
		fflush(fp);
		fclose(fp);
		printf("carbirdview_p.yuv success !\n");	
	}


	if ((fp = fopen("/data/carbirdviewright.yuv", "rb")) == NULL)
	{
		printf("Fail to fopen carbirdviewright.yuv\n");
	}
	else
	{	
		carbirdviewright_pd = (unsigned char* )malloc(sizeof(unsigned char)*320*160*2);
		fread(carbirdviewright_pd, sizeof(unsigned char), 320*160*2,fp);			
		fflush(fp);
		fclose(fp);
		printf("carbirdviewright_p.yuv success !\n");	
	}
		printf("read  image end\n");

}
#endif

void ReadLut_mini(void)
{
	FILE*  fp;
	printf("read mini LUT begin\n");
	if((fp= fopen("/data/front_single_view_lut_mini.bin", "rb"))== NULL){
		printf("front_single_view_lut_mini.bin was not opened\n");		
	}
	
		
	else	{// front single view lut
		Lut_Fsv_View_Minipd = (unsigned int* )malloc(sizeof(unsigned int)*SINGLE_VIEW_WIDTH*SINGLE_VIEW_HEIGHT);
		fread(Lut_Fsv_View_Minipd,sizeof(unsigned int),SINGLE_VIEW_WIDTH*SINGLE_VIEW_HEIGHT,fp);   
		fflush(fp);
		fclose(fp);
		printf("front_single_view_lut_mini.bin success !\n");		
	}
	if((fp= fopen("/data/back_single_view_lut_mini.bin", "rb"))== NULL){
		printf("back_single_view_lut_mini.bin was not opened\n");		
	}

	else	{// back single view lut
		Lut_Bsv_View_Minipd = (unsigned int* )malloc(sizeof(unsigned int)*SINGLE_VIEW_WIDTH*SINGLE_VIEW_HEIGHT);
		fread(Lut_Bsv_View_Minipd,sizeof(unsigned int),SINGLE_VIEW_WIDTH*SINGLE_VIEW_HEIGHT,fp);    
		fflush(fp);
		fclose(fp);
		printf("back_single_view_lut_mini.bin success !\n");		
	}
		
	if((fp= fopen("/data/left_single_view_lut_mini.bin", "rb"))== NULL){
		printf("left_single_view_lut_mini.bin was not opened\n");		
	}

	else	{// left single view lut.
		Lut_Lsv_View_Minipd = (unsigned int* )malloc(sizeof(unsigned int)*SINGLE_VIEW_WIDTH*SINGLE_VIEW_HEIGHT);
		fread(Lut_Lsv_View_Minipd,sizeof(unsigned int),SINGLE_VIEW_WIDTH*SINGLE_VIEW_HEIGHT,fp);    
		fflush(fp);
		fclose(fp);
		printf("left_single_view_lut_mini.bin success !\n");		
	}

	if((fp= fopen("/data/right_single_view_lut_mini.bin", "rb"))== NULL){
		printf("right_single_view_lut_mini.bin was not opened\n");		
	}

	else	{// right single view lut
		Lut_Rsv_View_Minipd = (unsigned int* )malloc(sizeof(unsigned int)*SINGLE_VIEW_WIDTH*SINGLE_VIEW_HEIGHT);
		fread(Lut_Rsv_View_Minipd,sizeof(unsigned int),SINGLE_VIEW_WIDTH*SINGLE_VIEW_HEIGHT,fp);    
		fflush(fp);
		fclose(fp);
		printf("right_single_view_lut_mini.bin success !\n");		
	}


	if((fp= fopen("/data/Lut_Front_mini.bin", "rb"))== NULL){
		printf("Lut_Front_mini.bin was not opened\n");	
		}

	else	{//mini avm front lut
		Lut_Front_Minipd = (unsigned int* )malloc(sizeof(unsigned int)*LUT_POS_FB_MINI);
		fread(Lut_Front_Minipd,sizeof(unsigned int),LUT_POS_FB_MINI,fp);    
		fflush(fp);
		fclose(fp);
		printf("Lut_Front_mini.bin  success !\n");		
	}

	if((fp= fopen("/data/Lut_Back_mini.bin", "rb"))== NULL){
		printf("Lut_Back_mini.bin was not opened\n");	
		}

	else	{//mini avm back lut
		Lut_Back_Minipd = (unsigned int* )malloc(sizeof(unsigned int)*LUT_POS_FB_MINI);
		fread(Lut_Back_Minipd,sizeof(unsigned int),LUT_POS_FB_MINI,fp);   
		fflush(fp);
		fclose(fp);
		printf("Lut_Back_mini.bin  success !\n");		
	}

	if((fp= fopen("/data/Lut_Left_mini.bin", "rb"))== NULL){
		printf("Lut_Left_mini.bin was not opened\n");	
		}

	else	{//mini avm left lut
		Lut_Left_Minipd = (unsigned int* )malloc(sizeof(unsigned int)*LUT_POS_LR_MINI);
		fread(Lut_Left_Minipd,sizeof(unsigned int),LUT_POS_LR_MINI,fp);  
		fflush(fp);
		fclose(fp);
		printf("Lut_Left_mini.bin success !\n");		
	}

	if((fp= fopen("/data/Lut_Right_mini.bin", "rb"))== NULL){
		printf("Lut_Right_mini.bin was not opened\n");	
		}

	else	{//mini avm right lut
		Lut_Right_Minipd = (unsigned int* )malloc(sizeof(unsigned int)*LUT_POS_LR_MINI);
		fread(Lut_Right_Minipd,sizeof(unsigned int),LUT_POS_LR_MINI,fp);   
		fflush(fp);
		fclose(fp);
		printf("Lut_Right_mini.bin success !\n");		
	}	


	if((fp= fopen("/data/Wt_Front_mini.bin", "rb"))== NULL){
		printf("Wt_Front_mini.bin was not opened\n");	
		}

	else	{//mini avm front wt lut
		Wt_Lut_Front_Minipd = (UInt64_t* )malloc(sizeof(UInt64_t)*LUT_WT_FB_MINI);
		fread(Wt_Lut_Front_Minipd,sizeof(UInt64_t),LUT_WT_FB_MINI,fp);   
		fflush(fp);
		fclose(fp);
		printf("Wt_Front_mini.bin   success !\n");		
	}

	if((fp= fopen("/data/Wt_Back_mini.bin", "rb"))== NULL){
		printf("Wt_Back_mini.bin was not opened\n");	
		}

	else	{//mini avm back wt lut
		Wt_Lut_Back_Minipd = (UInt64_t* )malloc(sizeof(UInt64_t)*LUT_WT_FB_MINI);
		fread(Wt_Lut_Back_Minipd,sizeof(UInt64_t),LUT_WT_FB_MINI,fp);  
		fflush(fp);
		fclose(fp);
		printf("Wt_Back_mini.bin   success !\n");		
	}

	if((fp= fopen("/data/Wt_Left_mini.bin", "rb"))== NULL){
		printf("Wt_Left_mini.bin was not opened\n");	
		}

	else	{//mini avm left wt lut
		Wt_Lut_Left_Minipd = (UInt64_t* )malloc(sizeof(UInt64_t)*LUT_WT_LR_MINI);
		fread(Wt_Lut_Left_Minipd,sizeof(UInt64_t),LUT_WT_LR_MINI,fp);    
		fflush(fp);
		fclose(fp);
		printf("Wt_Left_mini.bin   success !\n");		
	}

	if((fp= fopen("/data/Wt_Right_mini.bin", "rb"))== NULL){
		printf("Wt_Right_mini.bin was not opened\n");	
		}

	else	{//mini avm right wt lut
		Wt_Lut_Right_Minipd = (UInt64_t* )malloc(sizeof(UInt64_t)*LUT_WT_LR_MINI);
		fread(Wt_Lut_Right_Minipd,sizeof(UInt64_t),LUT_WT_LR_MINI,fp);   
		fflush(fp);
		fclose(fp);
		printf("Wt_Right_mini.bin   success !\n");		
	}
	
	printf("read mini LUT end\n");
}
void ReadLut(void)
{
	FILE*  fp;
	printf("read big LUT begin\n");
	if((fp= fopen("/data/Lut_Front.bin", "rb"))== NULL){
		printf("Lut_Front.bin was not opened\n");	
		}

	else	{//big avm front lut
		Lut_Frontpd = (unsigned int* )malloc(sizeof(unsigned int)*LUT_POS_FB);
		fread(Lut_Frontpd,sizeof(unsigned int),LUT_POS_FB,fp);   
		fflush(fp);
		fclose(fp);
		printf("Lut_Front.bin  success !\n");		
	}
	

	if((fp= fopen("/data/Lut_Back.bin", "rb"))== NULL){
		printf("Lut_Back.bin was not opened\n");
		}

	else	{//big avm back lut
		Lut_Backpd = (unsigned int* )malloc(sizeof(unsigned int)*LUT_POS_FB);
		fread(Lut_Backpd,sizeof(unsigned int),LUT_POS_FB,fp);   
		fflush(fp);
		fclose(fp);
		printf("Lut_Back.bin  success !\n");		
	}

	if((fp= fopen("/data/Lut_Left.bin", "rb"))== NULL){
		printf("Lut_Left.bin was not opened\n");	
		}

	else	{//big avm left lut
		Lut_Leftpd = (unsigned int* )malloc(sizeof(unsigned int)*LUT_POS_LR);
		fread(Lut_Leftpd,sizeof(unsigned int),LUT_POS_LR,fp);   
		fflush(fp);
		fclose(fp);
		printf("Lut_Left.bin success !\n");		
	}

	if((fp= fopen("/data/Lut_Right.bin", "rb"))== NULL){
		printf("Lut_Right.bin was not opened\n");	
		}

	else	{//big avm right lut
		Lut_Rightpd = (unsigned int* )malloc(sizeof(unsigned int)*LUT_POS_LR);
		fread(Lut_Rightpd,sizeof(unsigned int),LUT_POS_LR,fp);   
		fflush(fp);
		fclose(fp);
		printf("Lut_Right.bin   success !\n");		
	}


	if((fp= fopen("/data/Wt_Front.bin", "rb"))== NULL){
		printf("Wt_Front.bin was not opened\n");	
		}

	else	{//big avm front wt lut
		Wt_Lut_Frontpd = (UInt64_t* )malloc(sizeof(UInt64_t)*LUT_WT_FB);
		fread(Wt_Lut_Frontpd,sizeof(UInt64_t),LUT_WT_FB,fp);  
		fflush(fp);
		fclose(fp);
		printf("Wt_Front.bin   success !\n");		
	}

	if((fp= fopen("/data/Wt_Back.bin", "rb"))== NULL){
		printf("Wt_Back.bin was not opened\n");	
		}

	else	{//big avm back wt lut
		Wt_Lut_Backpd = (UInt64_t* )malloc(sizeof(UInt64_t)*LUT_WT_FB);
		fread(Wt_Lut_Backpd,sizeof(UInt64_t),LUT_WT_FB,fp);   
		fflush(fp);
		fclose(fp);
		printf("Wt_Back.bin   success !\n");		
	}

	if((fp= fopen("/data/Wt_Left.bin", "rb"))== NULL){
		printf("Wt_Left.bin was not opened\n");	
		}

	else	{//big avm left wt lut
		Wt_Lut_Leftpd = (UInt64_t* )malloc(sizeof(UInt64_t)*LUT_WT_LR);
		fread(Wt_Lut_Leftpd,sizeof(UInt64_t),LUT_WT_LR,fp);   
		fflush(fp);
		fclose(fp);
		printf("Wt_Left.bin   success !\n");		
	}

	if((fp= fopen("/data/Wt_Right.bin", "rb"))== NULL){
		printf("Wt_Right.bin was not opened\n");	
		}

	else	{//big avm right wt lut
		Wt_Lut_Rightpd = (UInt64_t* )malloc(sizeof(UInt64_t)*LUT_WT_LR);
		fread(Wt_Lut_Rightpd,sizeof(UInt64_t),LUT_WT_LR,fp);    
		fflush(fp);
		fclose(fp);
		printf("Wt_Right.bin   success !\n");		
	}
		printf("read big LUT end\n");
}

 int saveframe(char *str, void *p, int length, int is_oneframe)
{
	FILE *fd;

	if (is_oneframe) {
		fd = fopen(str, "wb");
	}
	else {
		fd = fopen(str, "a");
	}     

	if (!fd) {
		printf("Open file error\n");
		return -1;
	}
	if (fwrite(p, 1, length, fd)) {
		printf("Write file successfully\n");
		fclose(fd);
		return 0;
	}
	else {
		printf("Write file fail\n");
		fclose(fd);
		return -1;
	}
	
}
/// void Resize_Interpolation_SingleView(unsigned short *resize_lut,unsigned int **remap_lut, unsigned char * dout,int Nwidth,int Nheight)
#if 1
int UndistortionSVProccess(unsigned int * p_lut,
	unsigned char* result_image,
	unsigned char* source_image,
	int result_height,
	int result_width,
	int source_height,
	int source_width)
{
       int ret = 0;

	unsigned int  item;
	int source_pos[2]; 
	int wt_up, wt_left;
	int wt_upleft, wt_upright, wt_downleft, wt_downright;
	int src_widthstep, result_widthstep;// result_nchannels; src_nchannels,
	int yuv[3];
	unsigned char* p_src = source_image;


	result_widthstep = result_width << 1;
	src_widthstep = source_width << 1;

	for (int i = 0; i < result_height; i++)
	{
		for (int j = 0; j < result_width; j++)
		{
			item = p_lut[i * result_width + j];

			source_pos[0] = item >> 21;
			source_pos[1] = (item >> 10) & 0x7FF;

			source_pos[0] = source_pos[0] < 0 ? 0 : (source_pos[0] > source_height - 1 ? source_height - 1 : source_pos[0]);
			source_pos[1] = source_pos[1] < 0 ? 0 : (source_pos[1] > source_width - 1 ? source_width - 1 : source_pos[1]);

			wt_up = (item >> 5) & 0x1F;
			wt_left = item & 0x1F;

			wt_upleft = (31 - wt_up) * (31 - wt_left);
			wt_upright = (31 - wt_up) * wt_left;
			wt_downleft = wt_up * (31 - wt_left);
			wt_downright = 31 * 31 - wt_upleft - wt_upright - wt_downleft;

			yuv[0] = (p_src[source_pos[0] * src_widthstep + source_pos[1] * 2 + 1] * wt_upleft
				+ p_src[source_pos[0] * src_widthstep + (source_pos[1] + 1) * 2 + 1] * wt_upright
				+ p_src[(source_pos[0] + 1) * src_widthstep + source_pos[1] * 2 + 1] * wt_downleft
				+ p_src[(source_pos[0] + 1) * src_widthstep + (source_pos[1] + 1) * 2 + 1] * wt_downright) >> 10;

			yuv[0] = yuv[0] < 0 ? 0 : (yuv[0] > 255 ? 255 : yuv[0]);

			result_image[i * result_widthstep + j*2+1] = yuv[0];///

			// process u & v channel
			if(((j^source_pos[1])&1)==1)
				
				source_pos[1]+=1;
				
			
			if (j % 2 == 0)
			{
				yuv[1] = p_src[source_pos[0] * src_widthstep + source_pos[1] * 2];
				yuv[1] = yuv[1] < 0 ? 0 : (yuv[1] > 255 ? 255 : yuv[1]);
				result_image[i * result_widthstep + j * 2] = yuv[1];
			}
			else
			{
				yuv[2] = p_src[source_pos[0] * src_widthstep + source_pos[1] * 2];
				yuv[2] = yuv[2] < 0 ? 0 : (yuv[2] > 255 ? 255 : yuv[2]);
				result_image[i * result_widthstep + j * 2] = yuv[2];
			}
		}
	}

	return ret;
}
#endif






Int32_t bev_process(
	uchar* result_image,
	uchar* front_image_uyvy,
	uchar* back_image_uyvy,
	uchar* left_image_uyvy,
	uchar* right_image_uyvy,
	CvPoint1 car_up_left,
	CvPoint1 car_down_right,
	Int32_t frontback_fov_height,
	Int32_t leftright_fov_width,
	Int32_t result_width,
	Int32_t source_width,
	TabBev **bev_Table[4])
{
	Int32_t ret = 0;
	uchar* p_src, *p_src1, *p_src2;
	memset(result_image, 0, DST_WIDTH * DST_HEIGHT* 2);
	
///	Int32_t yuv[3];
	Int32_t result_widthstep, src_widthstep;
	CvRect1 region_roi;
	Int32_t camid, fusion_pic1, fusion_pic2;

	result_widthstep = result_width << 1;
	src_widthstep = source_width << 1;

	UInt32_t loop_count = 0;
//	Float64_t total_time_1 = 0, total_time_2 = 0;
#if 0
	printf("car_up_left_mini.x= %d\n", car_up_left.x);
	printf("car_up_left_mini.y= %d\n", car_up_left.y);
	printf("car_down_right_mini.x= %d\n", car_down_right.x);
	printf("car_down_right_mini.y= %d\n", car_down_right.y);
	printf("frontback_fov_height= %d\n", frontback_fov_height);
	printf("leftright_fov_width= %d\n", leftright_fov_width);
#endif

	while (loop_count < 1)
	{
#ifdef TIME_CONSUME
	time_t  start_time = clock();
#endif
		//front left
		p_src1 = front_image_uyvy;
		p_src2 = left_image_uyvy;
		fusion_pic1 = 0;
		fusion_pic2 = 2;
	///	printf("0000\n");
		region_roi = cvRect1(0, 0, car_up_left.x, car_up_left.y);
		analysis_fusion_region_lut_uyvy(result_image, p_src1, p_src2, fusion_pic1, fusion_pic2, &region_roi, result_widthstep, src_widthstep, bev_Table);
#ifdef TIME_CONSUME		
		time_t  time1_1 = clock();
		double time0_1 = (double)(clock() - start_time);
		//printf("image_result_test cost :%f\n", time0_1);
		total_time_1 += time0_1;
#endif
		loop_count++;
	}
	

	///time_t time1_1 = clock();
	//printf("1111\n");
	//front
	region_roi = cvRect1(car_up_left.x, 0, car_down_right.x - car_up_left.x + 1, frontback_fov_height);//-10
	camid = 0;
	p_src = front_image_uyvy;
	analysis_single_region_lut_uyvy(result_image, p_src, &region_roi, camid, result_widthstep, src_widthstep, bev_Table);

	//printf("2222\n");
#ifdef TIME_CONSUME		
	time_t  time1_2 = clock();
	double time0_1 = (double)(clock() - time1_1);

///	time1_1 = clock();
#endif
	//front right
	p_src1 = front_image_uyvy;
	p_src2 = right_image_uyvy;
	fusion_pic1 = 0;
	fusion_pic2 = 3;
	//printf("3333\n");
	region_roi = cvRect1(car_down_right.x + 1, 0, car_up_left.x, car_up_left.y);
	analysis_fusion_region_lut_uyvy(result_image, p_src1, p_src2, fusion_pic1, fusion_pic2, &region_roi, result_widthstep, src_widthstep, bev_Table);
#ifdef TIME_CONSUME		
	time1_2 = clock();
	time0_1 = (double)(clock() - time1_1);
	///time1_1 = clock();
#endif
	//left
	//printf("4444\n");
	region_roi = cvRect1(0, car_up_left.y, car_up_left.x, car_down_right.y - car_up_left.y + 1);
//printf("y:%d ");
	analysis_single_region_lut_uyvy(result_image, left_image_uyvy, &region_roi, 2, result_widthstep, src_widthstep, bev_Table);

#ifdef TIME_CONSUME		
	time1_2 = clock();
	time0_1 = (double)(clock() - time1_1);

	///time1_1 = clock();
#endif
//right
	//printf("5555\n");
	region_roi = cvRect1(car_down_right.x + 1, car_up_left.y, result_width - car_down_right.x - 1, car_down_right.y - car_up_left.y + 1);
	analysis_single_region_lut_uyvy(result_image, right_image_uyvy, &region_roi, 3, result_widthstep, src_widthstep, bev_Table);


#ifdef TIME_CONSUME		
	time1_2 = clock();
	time0_1 = (double)(clock() - time1_1);
	time1_1 = clock();
#endif	
//back left
	p_src1 = left_image_uyvy;
	p_src2 = back_image_uyvy;
	fusion_pic1 = 2;
	fusion_pic2 = 1;
	//printf("6666\n");
	region_roi = cvRect1(0, car_down_right.y + 1, car_up_left.x, frontback_fov_height);
	analysis_fusion_region_lut_uyvy(result_image, p_src1, p_src2, fusion_pic1, fusion_pic2, &region_roi, result_widthstep, src_widthstep, bev_Table);

#ifdef TIME_CONSUME		
	time1_2 = clock();
	time0_1 = (double)(clock() - time1_1);
#endif
	//back
	//printf("7777\n");
	region_roi = cvRect1(car_up_left.x, car_down_right.y + 1, car_down_right.x - car_up_left.x + 1, frontback_fov_height);
	analysis_single_region_lut_uyvy(result_image, back_image_uyvy, &region_roi, 1, result_widthstep, src_widthstep, bev_Table);

///	time1_1 = clock();
	//back right

	p_src1 = right_image_uyvy;
	p_src2 = back_image_uyvy;
	fusion_pic1 = 3;
	fusion_pic2 = 1;
	//printf("8888\n");
	region_roi = cvRect1(car_down_right.x + 1, car_down_right.y + 1, car_up_left.x, frontback_fov_height);
	analysis_fusion_region_lut_uyvy(result_image, p_src1, p_src2, fusion_pic1, fusion_pic2, &region_roi, result_widthstep, src_widthstep, bev_Table);
	//printf("9999\n");

#ifdef TIME_CONSUME		
	time1_2 = clock();
	time0_1 = (double)(clock() - time1_1);
#endif	

	return ret;
}




void analysis_single_region_lut_uyvy(
	uchar* result_image, uchar* p_src,
	CvRect1* region_roi,
	Int32_t camid,
	Int32_t result_widthstep,
	Int32_t src_widthstep,
	TabBev **bev_Table[4])
{
	Int32_t yuv[2], x, y, row, col,i,j;
	if (camid == 0 || camid == 2) //0:front , 2:left
	{
		for ( i = region_roi->y; i < region_roi->height + region_roi->y; i++)
		{
			for ( j = region_roi->x; j < region_roi->width + region_roi->x; j++)
			{
				x = bev_Table[camid][i][j].point_pos.x;
				y = bev_Table[camid][i][j].point_pos.y;

				yuv[0] = ((p_src[y * src_widthstep + x * 2 + 1] * bev_Table[camid][i][j].wt_upleft
					+ p_src[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[camid][i][j].wt_upright
					+ p_src[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[camid][i][j].wt_downleft
					+ p_src[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[camid][i][j].wt_downright) >> 16)
					;

				yuv[1] = ((p_src[y * src_widthstep + x * 2]))
					;

				result_image[(i)* result_widthstep + (j)* 2 + 1] += yuv[0];
				result_image[(i)* result_widthstep + (j)* 2] += yuv[1];
			}
		}
	}
	else if (camid == 3)  //right
	{
		for ( i = region_roi->y; i < region_roi->height + region_roi->y; i++)
		{
			for ( j = region_roi->x; j < region_roi->width + region_roi->x; j++)
			{
				//row = i - region_roi->y;
				col = j - region_roi->x;
				x = bev_Table[camid][i][col].point_pos.x;
				y = bev_Table[camid][i][col].point_pos.y;

				yuv[0] = ((p_src[y * src_widthstep + x * 2 + 1] * bev_Table[camid][i][col].wt_upleft
					+ p_src[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[camid][i][col].wt_upright
					+ p_src[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[camid][i][col].wt_downleft
					+ p_src[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[camid][i][col].wt_downright) >> 16)
					;

				yuv[1] = ((p_src[y * src_widthstep + x * 2]))
					;

				result_image[(i)* result_widthstep + (j)* 2 + 1] += yuv[0];
				result_image[(i)* result_widthstep + (j)* 2] += yuv[1];
			}
		}
	}
	else    //back
	{
		for ( i = region_roi->y; i < region_roi->height + region_roi->y; i++)
		{
			for ( j = region_roi->x; j < region_roi->width + region_roi->x; j++)
			{
				row = i - region_roi->y;
				//col = j - region_roi->x;
				x = bev_Table[camid][row][j].point_pos.x;
				y = bev_Table[camid][row][j].point_pos.y;

				yuv[0] = ((p_src[y * src_widthstep + x * 2 + 1] * bev_Table[camid][row][j].wt_upleft
					+ p_src[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[camid][row][j].wt_upright
					+ p_src[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[camid][row][j].wt_downleft
					+ p_src[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[camid][row][j].wt_downright) >> 16)
					;

				yuv[1] = ((p_src[y * src_widthstep + x * 2]))
					;

				result_image[(i)* result_widthstep + (j)* 2 + 1] += yuv[0];
				result_image[(i)* result_widthstep + (j)* 2] += yuv[1];
			}
		}
	}
}

void analysis_fusion_region_lut_uyvy(uchar* result_image,
	uchar* p_src1,
	uchar* p_src2,
	Int32_t fusion_pic1,
	Int32_t fusion_pic2,
	CvRect1* region_roi,
	Int32_t result_widthstep,
	Int32_t src_widthstep,
	TabBev **bev_Table[4])
{
	Int32_t yuv[2], x, y, row, col,i,j;
	if (fusion_pic1 == 0 && fusion_pic2 == 2) //front_left
	{
		for ( i = region_roi->y; i < region_roi->height + region_roi->y; i++)
		{
			for ( j = region_roi->x; j < region_roi->width + region_roi->x; j++)
			{
				x = bev_Table[fusion_pic1][i][j].point_pos.x;
				y = bev_Table[fusion_pic1][i][j].point_pos.y;
				col = j - region_roi->x;
				row = i - region_roi->y;

				if (bev_Table[fusion_pic1][i][j].wt_fusion == 1023)
				{
					yuv[0] = ((p_src1[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upleft
						+ p_src1[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upright
						+ p_src1[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downleft
						+ p_src1[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downright) >> 16)
						;

					yuv[1] = ((p_src1[y * src_widthstep + x * 2]))
						;
				}
				else if (bev_Table[fusion_pic2][row][col].wt_fusion == 1023)
				{
					x = bev_Table[fusion_pic2][row][col].point_pos.x;
					y = bev_Table[fusion_pic2][row][col].point_pos.y;
					yuv[0] = ((p_src2[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upleft
						+ p_src2[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upright
						+ p_src2[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downleft
						+ p_src2[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downright) >> 16)
						;

					yuv[1] = ((p_src2[y * src_widthstep + x * 2]))
						;
				}
				else  // fusion region
				{
					yuv[0] = ((p_src1[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upleft
						+ p_src1[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upright
						+ p_src1[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downleft
						+ p_src1[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downright) >> 16)
						* bev_Table[fusion_pic1][i][j].wt_fusion >> 10;

					yuv[1] = ((p_src1[y * src_widthstep + x * 2]))
						* bev_Table[fusion_pic1][i][j].wt_fusion >> 10;


					x = bev_Table[fusion_pic2][row][col].point_pos.x;
					y = bev_Table[fusion_pic2][row][col].point_pos.y;
					yuv[0] += ((p_src2[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upleft
						+ p_src2[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upright
						+ p_src2[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downleft
						+ p_src2[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downright) >> 16)
						* bev_Table[fusion_pic2][row][col].wt_fusion >> 10;
					yuv[1] += ((p_src2[y * src_widthstep + x * 2]))
						* bev_Table[fusion_pic2][row][col].wt_fusion >> 10;
				}

				result_image[(i)* result_widthstep + (j)* 2 + 1] += yuv[0];
				result_image[(i)* result_widthstep + (j)* 2] += yuv[1];
			}
		}
	}
	else if ((fusion_pic1 == 0 && fusion_pic2 == 3) || (fusion_pic1 == 2 && fusion_pic2 == 1)) //front_right  and  left_back
	{
		for ( i = region_roi->y; i < region_roi->height + region_roi->y; i++)
		{
			for ( j = region_roi->x; j < region_roi->width + region_roi->x; j++)
			{
				if (bev_Table[fusion_pic1][i][j].wt_fusion == 1023)
				{
					x = bev_Table[fusion_pic1][i][j].point_pos.x;
					y = bev_Table[fusion_pic1][i][j].point_pos.y;
					//col = j - region_roi->x;
					//row = i - region_roi->y;

					yuv[0] = ((p_src1[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upleft
						+ p_src1[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upright
						+ p_src1[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downleft
						+ p_src1[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downright) >> 16)
						;

					yuv[1] = ((p_src1[y * src_widthstep + x * 2]))
						;
				}
				else if (bev_Table[fusion_pic1][i][j].wt_fusion == 0)
				{
					col = j - region_roi->x;
					row = i - region_roi->y;
					x = bev_Table[fusion_pic2][row][col].point_pos.x;
					y = bev_Table[fusion_pic2][row][col].point_pos.y;

					yuv[0] = ((p_src2[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upleft
						+ p_src2[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upright
						+ p_src2[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downleft
						+ p_src2[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downright) >> 16)
						;

					yuv[1] = ((p_src2[y * src_widthstep + x * 2]))
						;
				}
				else
				{
					x = bev_Table[fusion_pic1][i][j].point_pos.x;
					y = bev_Table[fusion_pic1][i][j].point_pos.y;
					yuv[0] = ((p_src1[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upleft
						+ p_src1[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upright
						+ p_src1[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downleft
						+ p_src1[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downright) >> 16)
						* bev_Table[fusion_pic1][i][j].wt_fusion >> 10;

					yuv[1] = ((p_src1[y * src_widthstep + x * 2]))
						* bev_Table[fusion_pic1][i][j].wt_fusion >> 10;

					col = j - region_roi->x;
					row = i - region_roi->y;
					x = bev_Table[fusion_pic2][row][col].point_pos.x;
					y = bev_Table[fusion_pic2][row][col].point_pos.y;
					yuv[0] += ((p_src2[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upleft
						+ p_src2[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upright
						+ p_src2[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downleft
						+ p_src2[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downright) >> 16)
						* bev_Table[fusion_pic2][row][col].wt_fusion >> 10;
					yuv[1] += ((p_src2[y * src_widthstep + x * 2]))
						* bev_Table[fusion_pic2][row][col].wt_fusion >> 10;
				}

				result_image[(i)* result_widthstep + (j)* 2 + 1] += yuv[0];
				result_image[(i)* result_widthstep + (j)* 2] += yuv[1];
			}
		}
	}
	else   //right_back
	{
		for ( i = region_roi->y; i < region_roi->height + region_roi->y; i++)
		{
			for ( j = region_roi->x; j < region_roi->width + region_roi->x; j++)
			{
				if (bev_Table[fusion_pic1][i][col = j - region_roi->x].wt_fusion == 1023)
				{
					col = j - region_roi->x;
					//row = i - region_roi->y;
					x = bev_Table[fusion_pic1][i][col].point_pos.x;
					y = bev_Table[fusion_pic1][i][col].point_pos.y;

					yuv[0] = ((p_src1[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_upleft
						+ p_src1[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_upright
						+ p_src1[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_downleft
						+ p_src1[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_downright) >> 16)
						;

					yuv[1] = ((p_src1[y * src_widthstep + x * 2]))
						;
				}
				else if (bev_Table[fusion_pic1][i][col = j - region_roi->x].wt_fusion == 0)
				{
					//col = j - region_roi->x;
					row = i - region_roi->y;
					x = bev_Table[fusion_pic2][row][j].point_pos.x;
					y = bev_Table[fusion_pic2][row][j].point_pos.y;

					yuv[0] = ((p_src2[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_upleft
						+ p_src2[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_upright
						+ p_src2[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_downleft
						+ p_src2[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_downright) >> 16)
						;

					yuv[1] = ((p_src2[y * src_widthstep + x * 2]))
						;
				}
				else
				{
					col = j - region_roi->x;
					x = bev_Table[fusion_pic1][i][col].point_pos.x;
					y = bev_Table[fusion_pic1][i][col].point_pos.y;
					yuv[0] = ((p_src1[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_upleft
						+ p_src1[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_upright
						+ p_src1[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_downleft
						+ p_src1[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_downright) >> 16)
						* bev_Table[fusion_pic1][i][col].wt_fusion >> 10;

					yuv[1] = ((p_src1[y * src_widthstep + x * 2]))
						* bev_Table[fusion_pic1][i][col].wt_fusion >> 10;

					//col = j - region_roi->x;
					row = i - region_roi->y;
					x = bev_Table[fusion_pic2][row][j].point_pos.x;
					y = bev_Table[fusion_pic2][row][j].point_pos.y;
					yuv[0] += ((p_src2[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_upleft
						+ p_src2[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_upright
						+ p_src2[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_downleft
						+ p_src2[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_downright) >> 16)
						* bev_Table[fusion_pic2][row][j].wt_fusion >> 10;
					yuv[1] += ((p_src2[y * src_widthstep + x * 2]))
						* bev_Table[fusion_pic2][row][j].wt_fusion >> 10;
				}

				result_image[(i)* result_widthstep + (j)* 2 + 1] += yuv[0];
				result_image[(i)* result_widthstep + (j)* 2] += yuv[1];
			}
		}
	}
}

CvRect1  cvRect1( int x, int y, int width, int height )
{
	CvRect1 r;

	r.x = x;
	r.y = y;
	r.width = width;
	r.height = height;

	return r;
}



