#ifndef _IMAGESTITCHING_H_

#define _IMAGESTITCHING_H_
#include "data_type.h"


//#define LUT_WT_FB_MINI    			413*644
//#define LUT_WT_LR_MINI    			1080*284
//#define LUT_POS_FB_MINI   			413*644
//#define LUT_POS_LR_MINI   			1080*284
int LUT_WT_FB_MINI,LUT_WT_LR_MINI,LUT_POS_FB_MINI,LUT_POS_LR_MINI;
//#define LUT_WT_FB    			458*1080
//#define LUT_WT_LR    			1664*404
//#define LUT_POS_FB   			458*1080
//#define LUT_POS_LR   			1664*404
int LUT_WT_FB,LUT_WT_LR,LUT_POS_FB,LUT_POS_LR;


#define SINGLE_VIEW_WIDTH 			1020
#define SINGLE_VIEW_HEIGHT 		760

#define IMG_WIDTH         				1280	
#define IMG_HEIGHT        				720
#define DST_HEIGHT            			1664
#define DST_WIDTH             			1080
#define DST_HEIGHT_MINI            		1080
#define DST_WIDTH_MINI            		644
#define PIC_ORISIZE  				(IMG_WIDTH*IMG_HEIGHT*2) 


 typedef struct CvPoint1
{
	int x;
 	int y;
 } 
CvPoint1;
CvPoint1  car_up_left, car_down_right,car_up_left_mini,car_down_right_mini;



//***************************Global variable*****wyf added 2017.11.29****************


unsigned char*  front_pd;
unsigned char*  back_pd;
unsigned char*  left_pd;
unsigned char*  right_pd; 
unsigned char*  leftbar_pd; 
unsigned char*  bottombar_pd; 
unsigned char*  carbirdview_pd; 	
unsigned char*  carbirdviewright_pd; 

unsigned int* Lut_Frontpd;
unsigned int* Lut_Backpd;
unsigned int* Lut_Leftpd;
unsigned int* Lut_Rightpd;
UInt64_t* Wt_Lut_Frontpd;
UInt64_t* Wt_Lut_Backpd;
UInt64_t* Wt_Lut_Leftpd;
UInt64_t* Wt_Lut_Rightpd;

unsigned int* Lut_Fsv_View_Minipd;
unsigned int* Lut_Bsv_View_Minipd;
unsigned int* Lut_Lsv_View_Minipd;
unsigned int* Lut_Rsv_View_Minipd;

unsigned int* Lut_Front_Minipd;
unsigned int* Lut_Back_Minipd;
unsigned int* Lut_Left_Minipd;
unsigned int* Lut_Right_Minipd;

UInt64_t* Wt_Lut_Front_Minipd;
UInt64_t* Wt_Lut_Back_Minipd;
UInt64_t* Wt_Lut_Left_Minipd;
UInt64_t* Wt_Lut_Right_Minipd;



/************************************************************************/
unsigned char  SVM_BUFFER_mini[DST_WIDTH_MINI * DST_HEIGHT_MINI * 2]; 

/************************************************************************/
unsigned char  SVM_BUFFER[DST_WIDTH * DST_HEIGHT * 2]; 
unsigned char  SVM_BUFFER_dst[DST_WIDTH * DST_HEIGHT * 2]; 
unsigned char  SVM_BUFFER_dst_y[DST_WIDTH * DST_HEIGHT];
unsigned char  SVM_BUFFERbakpre[DST_WIDTH * DST_HEIGHT * 2]; 
unsigned char  SINGLE_BUFFER[IMG_WIDTH * IMG_HEIGHT * 2];
unsigned char  canrxbuf[8];
int key,key16;
int channel_select,cha16;
int  snap_number,console_cmd;
/****************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
int saveframe(char *str, void *p, int length, int is_oneframe);


void stitchPart(Uint32 **rawDataAddr, Uint16 ccdNum, Uint16 rowStart, Uint16 rowEnd, Uint16 colStart, Uint16 colEnd);

void stitchFusion(Uint32 **rawDataAddrFB, Uint32 **rawDataAddrLR, int fusionBlock, Uint16 rowStart, Uint16 rowEnd, Uint16 colStart, Uint16 colEnd);

void Resize_Interpolation_SingleView(unsigned short *resize_lut, unsigned int **remap_lut, unsigned char * dout, int Nwidth, int Nheight);

void rotate270(unsigned char *src,unsigned char *dst,int width,int height);
void ReadOriginalImage(void);
void ReadLut(void);
void ReadLut_mini(void);
void Show4picT(void);

void  DisOriginalSingleViewPic(char num, char* dis_buf);

void Show4videoT(void);

void Show4videoLoop(int num);

void DisSnapOriginalVideocap(int num);

void DisUndistortionSingleVideo(int num,char* dis_buf);

void DisUndistortionSingleViewPic(int num, char* dis_buf);
int UndistortionSVProccess(unsigned int* p_lut,
	unsigned char* result_image,
	unsigned char* source_image,
	int result_height,
	int result_width,
	int source_height,
	int source_width);
/*****************************************************************/
typedef struct CvRect1
{
	int x;
	int y;
	int width;
	int height;
}
CvRect1;

CvRect1 cvRect1(int init_x, int init_y, int rect_width, int rect_height);
typedef struct TacPoint_Struct
{
	Int16_t x;
	Int16_t y;
}TacPoint;

typedef struct TabBev_Struct
{
	TacPoint point_pos;
	Int32_t wt_fusion;
	Int32_t wt_upleft;
	Int32_t wt_upright;
	Int32_t wt_downleft;
	Int32_t wt_downright;
}TabBev;

TabBev **bev_Table[4];
TabBev **bev_Table_mini[4];
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
	TabBev **bev_Table[4]);
void analysis_fusion_region_lut_uyvy(uchar* result_image,
	uchar* p_src1,
	uchar* p_src2,
	Int32_t fusion_pic1,
	Int32_t fusion_pic2,
	CvRect1* region_roi,
	Int32_t result_widthstep,
	Int32_t src_widthstep,
	TabBev **bev_Table[4]);
void analysis_single_region_lut_uyvy(
	uchar* result_image, uchar* p_src,
	CvRect1* region_roi,
	Int32_t camid,
	Int32_t result_widthstep,
	Int32_t src_widthstep,
	TabBev **bev_Table[4]);


/**********************************************************************/



void *CANTXTask(void *ptr1 );
void *CANRXTask(void *ptr1);
void *KeyTask(void *ptr1);
void *TerminalTask(void *ptr1) ;
void *CLTask(void *ptr1)  ;



#ifdef __cplusplus
}
#endif

#endif
