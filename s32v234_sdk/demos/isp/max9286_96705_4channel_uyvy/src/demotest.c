
#ifdef __cplusplus
extern "C" {
#endif
#ifndef __STANDALONE__
  #include <signal.h>
#endif // #ifdef __STANDALONE__
#include <string.h>
#include <termios.h>
///#include "frame_output_v234fb.h"
///#include "oal.h"
///#include "vdb_log.h"
///#include "sdi.hpp"
///#include "vdb_log.h"
#include <stdio.h> 
#include <stdlib.h>
#include "ImageStitching.h"
#ifdef __cplusplus
}
#endif

#if 1
void  DisOriginalSingleViewPic(char num,char* dis_buf)
{
    	int  i = 0;	
	unsigned char* p_source = NULL ;
	memset((char *)dis_buf,0,1920*1080*2);
     	switch(num)
     	{
		case  0: 
				p_source  =  front_pd;/*front_p*/; 	break;
		case  1:	
				p_source  =  back_pd;	break;
		case  2:	
				p_source  =  left_pd;	break;
		case  3:	
				p_source  =  right_pd;	break;
		default:break;
	}
	for(i=0;i<720;i++)  
		memcpy(dis_buf+i*1920*2,p_source+IMG_WIDTH*2*i, IMG_WIDTH*2);			


}


void DisUndistortionSingleViewPic(int num,char* dis_buf)
{
	int  i = 0;		
	unsigned char* p_source =NULL;
	unsigned char  p_result[SINGLE_VIEW_WIDTH*SINGLE_VIEW_HEIGHT*2];
	memset((char *)dis_buf,0,1920*1080*2);
     	switch(num)
     	{
		case  0:   
				p_source  =front_pd;// front_p;		
				UndistortionSVProccess(Lut_Fsv_View_Minipd, p_result,p_source,SINGLE_VIEW_HEIGHT,SINGLE_VIEW_WIDTH,720,IMG_WIDTH );
				break;
		case  1:
				p_source  = back_pd;		
				UndistortionSVProccess(Lut_Bsv_View_Minipd, p_result,p_source,SINGLE_VIEW_HEIGHT,SINGLE_VIEW_WIDTH,720,IMG_WIDTH );
				break;
		case  2:
				p_source  = left_pd;		
				UndistortionSVProccess(Lut_Lsv_View_Minipd, p_result,p_source,SINGLE_VIEW_HEIGHT,SINGLE_VIEW_WIDTH,720,IMG_WIDTH );
				break;
		case  3:
				p_source  = right_pd;		
				UndistortionSVProccess(Lut_Rsv_View_Minipd, p_result,p_source,SINGLE_VIEW_HEIGHT,SINGLE_VIEW_WIDTH,720,IMG_WIDTH );
				break;
		default:break;
			 
	}
	for(i=0;i<SINGLE_VIEW_HEIGHT;i++)  
	memcpy((char *)dis_buf+i*1920*2+900*2,(char*)p_result+SINGLE_VIEW_WIDTH*2*i, SINGLE_VIEW_WIDTH*2);
}

#endif
