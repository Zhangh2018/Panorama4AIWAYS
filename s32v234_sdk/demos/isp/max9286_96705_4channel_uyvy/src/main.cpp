/*****************************************************************************
*
* Freescale Confidential Proprietary
*
* Copyright (c) 2016 Freescale Semiconductor;
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include "ImageStitching.h"
#include "uart_to_mcu.h"
#include "network.h"
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <errno.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#ifndef __STANDALONE__
  #include <signal.h>
  #include <termios.h>
#endif // #ifdef __STANDALONE__
#include <string.h>

#ifdef __STANDALONE__
  #include "frame_output_dcu.h"
  #define CHNL_CNT io::IO_DATA_CH3
#else // #ifdef __STANDALONE__
  #include "frame_output_v234fb.h"
  #define CHNL_CNT io::IO_DATA_CH3
#endif // else from #ifdef __STANDALONE__

#include "oal.h"
#include "vdb_log.h"
#include "sdi.hpp"

#include "max9286_96705_4_uyvy_c.h"

#include "vdb_log.h"

///////////////////////////////////new head files//////////////////////////////////////////////////////
//#include <stdio.h>
#include <fstream>

#include "CameraDevice.h"
#include "DrawScene.h"
#include "ImageBilt.h"
#include "FinalScene.h"

#include "keyboardInput.h"
#include "OGLEGL.h"
#include "PanoramaParam.h"
#include "Panorama3DScene.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "dump.h"
#include "utils/Timer.hpp"
#include "isp_cam_maxim.h"
//#include "json.hpp"
//using json = nlohmann::json;


bool fbo_on = false;

char path_pattern[256+1]={0};
int num_frames = 0;
pthread_mutex_t videocapture_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t videocapture_cond = PTHREAD_COND_INITIALIZER;
	
using namespace keyboard_device;   //ʹ�ñ�׼�����豸

//for gl end

//****************************************************************************//
using namespace cv;
extern unsigned int McuTimeData;
extern unsigned long long Mcu_tt;
unsigned long long Soc_tt;
BasicData *BasicData_Ptr;
CamData *CamData_Ptr;
ParkingPlace *ParkingPlace_Ptr;
McuSendDada *McuSendDada_Ptr;
LanelineData *LanelineData_Ptr;
ObstacleData *ObstacleData_Ptr;

PCWRITEDATA *PCWRITEDATA_Ptr;

extern volatile unsigned int CounterTick;
unsigned long long GetTime(void);
unsigned long long GetNowTimeUs(void);
extern int num;
extern short int McuSendOffset[10*3];
extern unsigned int McuSendCounter[10];
extern int reset_status_changed_num;
extern int reset_new_value_cnt;
extern int reset_new_value_cnt_flag;
extern int reset_chang_flag;
vsdk::SMat frame_map[4];

unsigned char* frameDataPtr[4];


vsdk::SMat frame_map_out;


unsigned char  mem_tmp_T0[IMG_WIDTH*IMG_HEIGHT*2];
unsigned char  mem_tmp_T1[IMG_WIDTH*IMG_HEIGHT*2];
unsigned char  mem_tmp_T2[IMG_WIDTH*IMG_HEIGHT*2];
unsigned char  mem_tmp_T3[IMG_WIDTH*IMG_HEIGHT*2];

void *VideoCaptureTask(void * ptr);
void *GLTask(void * ptr);
int Ipc_Create();
//int saveframe(char *str, void *p, int length, int is_oneframe);
void OpenglInit();


//***************************************************************************

// Possible to set input resolution (must be supported by the DCU)
#define WIDTH           1280 ///< width of DDR buffer in pixels
#define HEIGHT          720 ///< height of DDR buffer in pixels
#define DDR_BUFFER_CNT  3    ///< number of DDR buffers per ISP stream

static uint8_t          OutPutBufferUYVY[WIDTH*HEIGHT*2];
static uint8_t          RGB24Buffer[WIDTH*HEIGHT*3];


//***************************************************************************

#define FRM_TIME_MSR 300 ///< number of frames to measure the time and fps

#ifdef __STANDALONE__
//extern SEQ_Buf_t  producerless_buffer_1;
extern "C" {
  unsigned long get_uptime_microS(void);
}

#define GETTIME(time)   (*(time)=get_uptime_microS())
#else // ifdef __STANDALONE__
#define GETTIME(time) \
  { \
  struct timeval lTime; gettimeofday(&lTime,0); \
  *time=(lTime.tv_sec*1000000+lTime.tv_usec);   \
  }
#endif // else from #ifdef __STANDALONE__

/************************************************************************/
/** User defined call-back function for Sequencer event handling.
  *
  * \param  aEventType defines Sequencer event type
  * \param  apUserVal  pointer to any user defined object
  ************************************************************************/
void SeqEventCallBack(uint32_t aEventType, void* apUserVal);

#ifndef __STANDALONE__
/************************************************************************/
/** SIGINT handler.
  *
  * \param  aSigNo
  ************************************************************************/
void SigintHandler(int);

/************************************************************************/
/** SIGINT handler.
  *
  * \param  aSigNo
  *
  * \return SEQ_LIB_SUCCESS if all ok
  *         SEQ_LIB_FAILURE if failed
  ************************************************************************/
int32_t SigintSetup(void);

//***************************************************************************

static bool sStop = false; ///< to signal Ctrl+c from command line

#endif // #ifndef __STANDALONE__
/************************************************************************/
/** Allocates contiguous DDR buffers for one ISP stream.
  *
  * \param  appFbsVirtual array of virtual buffer pointers to be filled
  * \param  apFbsPhysical array of buffer physical addresses to be filled
  * \param  aMemSize      size of the array in bytes
  * \param  aBufferCnt    number of buffers to be allocated
  *
  * \return 0 if all OK
  *         < 0 otherwise
  ************************************************************************/
int32_t DdrBuffersAlloc(void** appFbsVirtual, uint32_t*  apFbsPhysical,
                        size_t aMemSize, uint32_t aBufferCnt);

/************************************************************************/
/** Frees DDR buffers for one ISP stream.
  *
  * \param  appFbsVirtual array of virtual buffer pointers to be filled
  * \param  apFbsPhysical array of buffer physical addresses to be filled
  * \param  aBufferCnt    number of buffers to be freed
  ************************************************************************/
void DdrBuffersFree(void** appFbsVirtual, uint32_t*  apFbsPhysical,
                    uint32_t aBufferCnt);


static void TermNonBlockSet()
{
  struct termios lTermState;

  // get current state of the terminal
  tcgetattr(STDIN_FILENO, &lTermState);

  // disable canonical mode
  lTermState.c_lflag &= ~ICANON;
  // set minimum number of input characters to read
  lTermState.c_cc[VMIN] = 1;

  // set the terminal attributes
  tcsetattr(STDIN_FILENO, TCSANOW, &lTermState);
} // TermNonBlockSet()



static int KeyDown()
{
  struct timeval lTv;
  fd_set lFdSet;
  lTv.tv_sec = 0;
  lTv.tv_usec = 50;
  FD_ZERO(&lFdSet);
  FD_SET(STDIN_FILENO, &lFdSet);
  select(STDIN_FILENO + 1, &lFdSet, NULL, NULL, &lTv);
  return FD_ISSET(STDIN_FILENO, &lFdSet);
} // KeyDown()


static int GetCharNonBlock()
{
  int lChar = EOF;

  usleep(1);
  if(KeyDown())
  {
    lChar = fgetc(stdin);
  } // if Key pressed

  return lChar;
} // KeyDown()

static char GetChar()
{
#ifdef __STANDALONE__
  return sci_0_testchar();
#else // #ifdef __STANDALONE__
  int lChar = GetCharNonBlock();
  return (char)((lChar < 0)? 0: lChar&0xff);
#endif // else from #ifdef __STANDALONE__
} // GetChar()
#if 1
int main(int, char **)
{
	int ret = 0;
	pthread_t gltask,cltask,keytask,tertask,guitask,uartrxtask,uarttxtask,videotask,id1;
	pthread_t  net_staus_thread,tr_thread,rv_thread;

	Ipc_Create();
	//OpenglInit();
	pthread_create(&videotask, NULL, VideoCaptureTask,NULL);

	sleep(1);
	printf("start GLTask\n");
	//ret = pthread_create(&gltask, NULL, GLTask,NULL);
	if(ret)
	{
		printf("Create GL Task error!\n");
		return 1;
	}

	//ret = pthread_create(&cltask, NULL, CLTask,NULL);
	if(ret)
	{
		printf("Create CLTask error!\n");
		return 1;
	}

	ret = pthread_create(&keytask, NULL, KeyTask,NULL);
	if(ret)
	{
		printf("Create keytask error!\n");
		return 1;
	}

	//ret = pthread_create(&tertask, NULL, TerminalTask,NULL);
	if(ret)
	{
		printf("Create TerminalTask error!\n");
		return 1;
	}

	//ret = pthread_create(&guitask, NULL, Gui_meg_thread,NULL);
	//pthread_create(&id1, NULL, Gui_draw_thread,NULL);
///	if(ret)
///	{
///		printf("Create Gui_meg_thread error!\n");
///		return 1;
///	}

	ret = pthread_create(&uartrxtask, NULL, Uart_meg_thread,NULL);
	if(ret)
	{
		printf("Create Uart_meg_thread error!\n");
		return 1;
	}
	ret = pthread_create(&uarttxtask, NULL, Uart_TX_thread,NULL);
	if(ret)
	{
		printf("Create Uart_TX_thread error!\n");
		return 1;
	}
	ret = pthread_create(&tr_thread, NULL, net_tr_thread, NULL);
	if(ret)
	{
		printf("Create net_tr_thread error!\n");
		return 1;
	}
	 pthread_join(videotask,NULL);
	//pthread_join(cltask,NULL);
	//pthread_join(gltask,NULL);
	pthread_join(keytask,NULL);
	//pthread_join(tertask,NULL);
	//pthread_join(guitask,NULL);
	pthread_join(uartrxtask,NULL);
	pthread_join(uarttxtask,NULL);
	pthread_join(tr_thread,NULL);
	return 0;
}
#endif
char temp_buf[10];

void *VideoCaptureTask(void *ptr1)
{
  LIB_RESULT lRet = LIB_SUCCESS;
  LIB_RESULT lRes = LIB_SUCCESS;

  char cam_cmd[12];
  char cur_char;
  vsdk::SMat frame_map_UYVY;

  OAL_Initialize();

  //*** Init DCU Output ***
#ifdef __STANDALONE__
  io::FrameOutputDCU
    lDcuOutput(WIDTH,
               HEIGHT,
               io::IO_DATA_DEPTH_08,
               io::IO_DATA_CH3,
               DCU_BPP_YCbCr422
               );
#else // #ifdef __STANDALONE__

  // setup Ctrl+C handler
  //if(SigintSetup() != SEQ_LIB_SUCCESS)
  //{
  //  VDB_LOG_ERROR("Failed to register Ctrl+C signal handler.");
    //return -1;
  //}

  //printf("Press Ctrl+C to terminate the demo.\n");

  // *** set terminal to nonblock input ***
  //TermNonBlockSet();

  /*
  io::FrameOutputV234Fb
    lDcuOutput(1280,
               720,
               io::IO_DATA_DEPTH_08,
               io::IO_DATA_CH3,
               DCU_BPP_YCbCr422
               );
  */
#endif // else from #ifdef __STANDALONE__
  //printf("main.cpp line135\n");
  //
  // *** Initialize SDI ***
  //
  lRes = sdi::Initialize(0);
  printf("main.cpp line140\n");
  // *** create grabber ***
  sdi_grabber *lpGrabber = new(sdi_grabber);
  lpGrabber->ProcessSet(gpGraph, &gGraphMetadata);
  printf("main.cpp line144\n");
  // *** set user defined Sequencer event handler ***
  int32_t lCallbackUserData = 12345;
  lpGrabber->SeqEventCallBackInstall(&SeqEventCallBack, &lCallbackUserData);

  // *** prepare IOs ***
  sdi_FdmaIO *lpFdma = (sdi_FdmaIO*)lpGrabber->IoGet(SEQ_OTHRIX_FDMA);

  // modify DDR frame geometry to fit display output
  SDI_ImageDescriptor lFrmDesc = SDI_ImageDescriptor(WIDTH, HEIGHT, YUV422Stream_UYVY);
  lpFdma->DdrBufferDescSet(0, lFrmDesc);

  lFrmDesc = SDI_ImageDescriptor(WIDTH, HEIGHT, YUV422Stream_UYVY);
  lpFdma->DdrBufferDescSet(1, lFrmDesc);

  lFrmDesc = SDI_ImageDescriptor(WIDTH, HEIGHT, YUV422Stream_UYVY);
  lpFdma->DdrBufferDescSet(2, lFrmDesc);

  lFrmDesc = SDI_ImageDescriptor(WIDTH, HEIGHT, YUV422Stream_UYVY);
  lpFdma->DdrBufferDescSet(3, lFrmDesc);

  //*** allocate DDR buffers ***
  lpFdma->DdrBuffersAlloc(DDR_BUFFER_CNT);
  //printf("main.cpp line154\n");

  TermNonBlockSet();


  // *** prestart grabber ***
  lpGrabber->PreStart();
  printf("main.cpp line157\n");
  // fetched frame buffer storage
  SDI_Frame lFrame[4];

  // *** start grabbing ***
  lpGrabber->Start();
  printf("main.cpp line163\n");
  unsigned long lTimeStart = 0, lTimeEnd = 0, lTimeDiff = 0;
  GETTIME(&lTimeStart);

  uint32_t actualBufferIndex = 2;

  uint32_t lFrmCnt = 0;
  printf("Complete init done.\n");
  uint8_t lLoop=0;
  uint8_t num=0;

  int lVal = 0;
  uint8_t first, tmp1, lVal1, lVal3 = 0;
  CAM_LIB_RESULT tmp = CAM_LIB_SUCCESS;
  static int nn;
  static int numCopy;
  static short int McuSendOffsetCopy[10*3];
  static unsigned int McuSendCounterCopy[10];
  int numtmp;
  frameDataPtr[0] = mem_tmp_T0;
  frameDataPtr[1] = mem_tmp_T1;
  frameDataPtr[2] = mem_tmp_T2;
  frameDataPtr[3] = mem_tmp_T3;
  ///////////////////////////////////////////////////////opengl init///////////////////
  	if(initialGL(1920, 720) != 0)          //初始化opengl es环境
    {
	     fprintf(stderr,"opengl init unsucess!\n");
	     //return -1;
	}
	printf("initialGL()::\n");
	auto glVer = glGetString(GL_VERSION);
	if (!glVer)
	{
		printf("glGetString failed: %d\n", glGetError());
	}
	else
	{
		printf("OpenGL version: %s\n", glGetString(GL_VERSION));
	}

	//if(getCameraDevice().initCamera() != 0)   //初始化视频采集设备
	{
	     printf("can not initialize the camera device！\n");
	     //return -1;
	}
	printf("getCameraDevice()::\n");

	if(initPanoramaParam() != 0){
	    printf("can not initialize the camera param");
	    //return -1;
	}

	printf("initPanoramaParam() success!\n");
	//keyboard_device::initDevice();                            //使用标准键盘设备
	printf("keyboard_device::initDevice() sucess!\n");
	CFinalScene scene;
	scene.setBirdEyeViewport(1280,
							 0,
							 640,
							 720);
	scene.setSingleViewport(0,
							0,
							1280,
							720);
	scene.set3DViewport(0,
						0,
						1280,
						720);
	scene.onStart();

	//std::cout << displaySetting << std::endl;
	//Calib_2D_Init();
  ///////////////////////////////////////////////////////////////////////////////////////
	int flag_pop = 0, flag_pop1 = 0;
	static int continue_count = 0;
	
	FILE *temp_fp = fopen("/sys/class/hwmon/hwmon0/temp1_input", "rb");
	if(temp_fp == NULL)
	printf("open temp file failed\n");
	
  while(1)
  {
  	pthread_mutex_lock(&videocapture_lock);

	fread(temp_buf, 1, 10, temp_fp);
	fseek(temp_fp, 0, SEEK_SET);
	
  	lVal++;

	if(lVal == 5)
	{
		printf("temperature=%s\n", temp_buf);
		MAXIM_DES_RegRead(CSI_IDX_0, (uint8_t)0x70, lVal1);
		printf("Maxim9286_96705 Init statu L:%d,R:%d,F:%d,B:%d\n", (lVal1>>6&0x01),
	         			(lVal1>>7&0x01), (lVal1>>4&0x01), (lVal1>>5&0x01));
		if (first != lVal1)
		{
			first = lVal1;
			lpGrabber->Restart();
		 	printf("1-> tmp1 = %d,first=0x%x\n",tmp1,first);
		}	
		tmp = MAXIM9286_96705_Channel_Check(lVal1);
		if(tmp != CAM_LIB_SUCCESS)
		{
			printf("MAXIM9286_96705_Channel_Check is error\n");
		}
		lVal = 0;
	}
	  for (int i = 0;i < 4;i++)
	  {
		  lFrame[i] = lpGrabber->FramePop(i);
		  /*IFrame[0] -->T1 ,IFrame[1] -->T2 ,IFrame[2] -->T3 ,IFrame[3] -->T4 ****wyf added 2017.11.29**/
		  if(lFrame[i].mUMat.empty())
		  {
			  printf("Failed to grab image number %u\n", i);
			  //break;
		  } // if pop failed
	  }
		if(flag_pop > 0)
		{
			flag_pop = 0;
			flag_pop1 = 1;
			continue_count++;
			printf("pop continue======================continue_count=%d============================\n", continue_count);
			continue;
		}

		if(flag_pop1 > 0)
		{
			printf("flag_pop1+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
			flag_pop1 = 0;
		}
		
	  //GETTIME(&lTimeEnd2);
	  //time_elapsed = (lTimeEnd2-lTimeStart2)/1000;
	  //GETTIME(&lTimeStart2);
	  #if 1
	  for (int i = 0;i < 4;i++)
	  {
		  frame_map[i] = lFrame[i].mUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	  }
	   //frame_map_out = lFrame[3].mUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

	  //buf_camfront = (unsigned char *)frame_map[0].data;
	  //buf_camback  = (unsigned char *)frame_map[1].data;
	  //buf_camleft  = (unsigned char *)frame_map[2].data;
	  //sbuf_camright = (unsigned char *)frame_map[3].data;
	  //memcpy(mem_tmp_T0,  (unsigned char *)frame_map[0].data, 1280*720*2 );
	  //memcpy(mem_tmp_T1,  (unsigned char *)frame_map[1].data,  1280*720*2 );
	  //memcpy(mem_tmp_T2,  (unsigned char *)frame_map[2].data,  1280*720*2 );
	  //memcpy(mem_tmp_T3,  (unsigned char *)frame_map[3].data, 1280*720*2 );
	  #endif

	  memcpy(CamData_Ptr->Mem_FrontCam, frame_map[0].data, 1280*720*2 );
	  memcpy(CamData_Ptr->Mem_BackCam, frame_map[1].data,  1280*720*2 );
	  memcpy(CamData_Ptr->Mem_LeftCam, frame_map[2].data,  1280*720*2 );
	  memcpy(CamData_Ptr->Mem_RightCam, frame_map[3].data, 1280*720*2 );

	  #if 0
	  static int uu;
	  uu++;
	  if(uu==600)
	  {
	  	printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&^^^^^^^^^^^\n");
		printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&^^^^^^^^^^^\n");
	  	saveframe("/app/front.yuv",CamData_Ptr->Mem_FrontCam , 1280*720*2, 1);
		saveframe("/app/back.yuv",CamData_Ptr->Mem_BackCam , 1280*720*2, 1);
		saveframe("/app/left.yuv",CamData_Ptr->Mem_LeftCam , 1280*720*2, 1);
		saveframe("/app/right.yuv",CamData_Ptr->Mem_RightCam , 1280*720*2, 1);
		printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&^^^^^^^^^^^\n");
		printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&^^^^^^^^^^^\n");
		uu=0;
	  }
	  #endif
	  //memset((char*)frame_map_out.data,0,1280*720*2);
	  //memcpy((unsigned char*)frame_map_out.data,  mem_tmp_T2, 1280*720*2 );

	  CounterTick = GetTime( ); //获取当前基准COUNTER值

	  unsigned char LA[4],LB[4],SA[4],SB[4];
	  float dT[8];

	  LA[0] = CamData_Ptr->Mem_FrontCam[9];
	  LB[0] = CamData_Ptr->Mem_FrontCam[8];
	  SA[0] = CamData_Ptr->Mem_FrontCam[11];
	  SB[0] = CamData_Ptr->Mem_FrontCam[10];
	  dT[0] = (LA[0]*256+LB[0])*42.708/1000;
	  dT[1] = (SA[0]*256+SB[0])*42.708/1000;

	  LA[1] = CamData_Ptr->Mem_BackCam[9];
	  LB[1] = CamData_Ptr->Mem_BackCam[8];
	  SA[1] = CamData_Ptr->Mem_BackCam[11];
	  SB[1] = CamData_Ptr->Mem_BackCam[18];
	  dT[2] = (LA[1]*256+LB[1])*42.708/1000;
	  dT[3] = (SA[1]*256+SB[1])*42.708/1000;

	  LA[2] = CamData_Ptr->Mem_LeftCam[9];
	  LB[2] = CamData_Ptr->Mem_LeftCam[8];
	  SA[2] = CamData_Ptr->Mem_LeftCam[11];
	  SB[2] = CamData_Ptr->Mem_LeftCam[10];
	  dT[4] = (LA[2]*256+LB[2])*42.708/1000;
	  dT[5] = (SA[2]*256+SB[2])*42.708/1000;

	  LA[3] = CamData_Ptr->Mem_RightCam[9];
	  LB[3] = CamData_Ptr->Mem_RightCam[8];
	  SA[3] = CamData_Ptr->Mem_RightCam[11];
	  SB[3] = CamData_Ptr->Mem_RightCam[10];
	  dT[6] = (LA[3]*256+LB[3])*42.708/1000;
	  dT[7] = (SA[3]*256+SB[3])*42.708/1000;

	  float dTmax = dT[0];
	  for(int i=0;i<8;i++)
	  {
		  if(dT[i]>dTmax)
			  dTmax = dT[i];
	  }
	  CamData_Ptr->TimeStamp = CounterTick-dTmax-10; //补偿摄像头的曝光时间
	  //printf("TimeStamp==%d\n",CamData_Ptr->TimeStamp);
	  //printf("N7F %x,N8 %x\n",CamData_Ptr->Mem_FrontCam[6],CamData_Ptr->Mem_FrontCam[7]);
	  //printf("N7B %x,N8 %x\n",CamData_Ptr->Mem_BackCam[6],CamData_Ptr->Mem_BackCam[7]);
	  //printf("N7L %x,N8 %x\n",CamData_Ptr->Mem_LeftCam[6],CamData_Ptr->Mem_LeftCam[7]);
	  //printf("N7R %x,N8 %x\n",CamData_Ptr->Mem_RightCam[6],CamData_Ptr->Mem_RightCam[7]);
	  //printf("T0:%f,%f T1:%f,%f T2:%f,%f T3:%f,%f  dTmax=%f\n", dT[0],dT[1], dT[2],dT[3], dT[4],dT[5], dT[6],dT[7],dTmax);

	  if(num >0)
	  {
		  numCopy = num-1;
	  }
	  else if(num == 0)
	  {
		  numCopy=9;
	  }
	  for(int j=0;j<10;j++)
	  {
		  McuSendOffsetCopy[j*3]   = McuSendOffset[j*3];
		  McuSendOffsetCopy[j*3+1] = McuSendOffset[j*3+1];
		  McuSendOffsetCopy[j*3+2] = McuSendOffset[j*3+2];
		  McuSendCounterCopy[j]    = McuSendCounter[j];
		  //printf("McuSendCounterCopy[%d]=%u\n",j,McuSendCounterCopy[j]);
	  }
	  /////////////////////////////////////////////////////////////////////////////////////
	  for(int j=0;j<10;j++)  //找到缓冲数组中距离补偿过的Counter值最近的序号
	  {
		  if(j<9)
		  {
			  if( CamData_Ptr->TimeStamp >= McuSendCounterCopy[j] && CamData_Ptr->TimeStamp<McuSendCounterCopy[j+1])
			  {
				  numCopy = j;
				  //printf("ok1 numCopy=%d\n",j);
				  break;
			  }
		  }
		  else
		  {
			  if( CamData_Ptr->TimeStamp >= McuSendCounterCopy[9] && CamData_Ptr->TimeStamp<McuSendCounterCopy[0])
			  {
				  numCopy = j;
				  //printf("ok2 numCopy=%d\n",j);
				  break;
			  }
		  }
	  }
	  ////////////////////////////////////////2019.6.29 added by wyf////////////////////////////////////
	  //if(reset_new_value_cnt <10)
	  if(reset_new_value_cnt_flag == 1)
	  {
		  if( reset_status_changed_num > 0)
		  {
			  if(numCopy == reset_status_changed_num -1)
			  {
				  CamData_Ptr->x = McuSendOffsetCopy[reset_status_changed_num*3];
				  CamData_Ptr->y = McuSendOffsetCopy[reset_status_changed_num*3+1];
				  CamData_Ptr->z = McuSendOffsetCopy[reset_status_changed_num*3+2];
				  //printf("$$$$$$$$$$$$$$$$$$$$$$$$$numCopy == reset_status_changed_num -1\n");
			  }
			  else
			  {
			  //////////////////////////////////////////////////////////////////////////////////////////////////////
				  if(numCopy>0)
				  {
					  numtmp = numCopy-1;
				  }
				  else if(numCopy == 0)
				  {
					  numtmp = 9;
				  }

				  float deltaX = McuSendOffsetCopy[numCopy*3]-McuSendOffsetCopy[numtmp*3];
				  float deltaY = McuSendOffsetCopy[numCopy*3+1]-McuSendOffsetCopy[numtmp*3+1];
				  float deltaZ = McuSendOffsetCopy[numCopy*3+2]-McuSendOffsetCopy[numtmp*3+2];
				  float deltaCnt = McuSendCounterCopy[numCopy]-McuSendOffsetCopy[numtmp];
				  float deltaCntNow = CamData_Ptr->TimeStamp-McuSendCounterCopy[numCopy];
				  CamData_Ptr->x = deltaX/deltaCnt*deltaCntNow + McuSendOffsetCopy[numCopy*3];
				  CamData_Ptr->y = deltaY/deltaCnt*deltaCntNow + McuSendOffsetCopy[numCopy*3+1];
				  CamData_Ptr->z = deltaZ/deltaCnt*deltaCntNow + McuSendOffsetCopy[numCopy*3+2];
			  //printf("numCpy=%d numtmp=%d CounterTick=%llu \n",numCopy,numtmp,CounterTick);
			  //////////////////////////////////////////////////////////////////////////////////////////////////////
			  }

			  if(CamData_Ptr->TimeStamp >= McuSendCounterCopy[reset_status_changed_num-1])
			  	scene.reset     = reset_chang_flag%2;
			  	printf("111reset_chang_flag=%d,scene.reset =%d\n",reset_chang_flag,scene.reset);
		  }
		  else if( reset_status_changed_num == 0)
		  {
			  if(numCopy == 9)
			  {
				  CamData_Ptr->x = McuSendOffsetCopy[reset_status_changed_num*3];
				  CamData_Ptr->y = McuSendOffsetCopy[reset_status_changed_num*3+1];
				  CamData_Ptr->z = McuSendOffsetCopy[reset_status_changed_num*3+2];
				  //printf("$$$$$$$$$$$$$$$$$$$$$$$$$reset_status_changed_num == 0\n");
			  }
			  else
			  {
			  //////////////////////////////////////////////////////////////////////////////////////////////////////
				  if(numCopy>0)
				  {
					  numtmp = numCopy-1;
				  }
				  else if(numCopy == 0)
				  {
					  numtmp = 9;
				  }

				  float deltaX = McuSendOffsetCopy[numCopy*3]-McuSendOffsetCopy[numtmp*3];
				  float deltaY = McuSendOffsetCopy[numCopy*3+1]-McuSendOffsetCopy[numtmp*3+1];
				  float deltaZ = McuSendOffsetCopy[numCopy*3+2]-McuSendOffsetCopy[numtmp*3+2];
				  float deltaCnt = McuSendCounterCopy[numCopy]-McuSendOffsetCopy[numtmp];
				  float deltaCntNow = CamData_Ptr->TimeStamp-McuSendCounterCopy[numCopy];
				  CamData_Ptr->x = deltaX/deltaCnt*deltaCntNow + McuSendOffsetCopy[numCopy*3];
				  CamData_Ptr->y = deltaY/deltaCnt*deltaCntNow + McuSendOffsetCopy[numCopy*3+1];
				  CamData_Ptr->z = deltaZ/deltaCnt*deltaCntNow + McuSendOffsetCopy[numCopy*3+2];
			  //printf("numCpy=%d numtmp=%d CounterTick=%llu \n",numCopy,numtmp,CounterTick);
			  //////////////////////////////////////////////////////////////////////////////////////////////////////
			  }
			   if(CamData_Ptr->TimeStamp >= McuSendCounterCopy[9])
			  	scene.reset     = reset_chang_flag%2;
			   printf("222reset_chang_flag=%d,scene.reset =%d\n",reset_chang_flag,scene.reset);
		  }
	  }
	  else
	  {
		  //////////////////////////////////////////////////////////////////////////////////////
		  if(numCopy>0)
		  {
			  numtmp = numCopy-1;
		  }
		  else if(numCopy ==0)
		  {
			  numtmp = 9;
		  }

		  float deltaX = McuSendOffsetCopy[numCopy*3]-McuSendOffsetCopy[numtmp*3];
		  float deltaY = McuSendOffsetCopy[numCopy*3+1]-McuSendOffsetCopy[numtmp*3+1];
		  float deltaZ = McuSendOffsetCopy[numCopy*3+2]-McuSendOffsetCopy[numtmp*3+2];
		  float deltaCnt = McuSendCounterCopy[numCopy]-McuSendOffsetCopy[numtmp];
		  float deltaCntNow = CamData_Ptr->TimeStamp-McuSendCounterCopy[numCopy];
		  CamData_Ptr->x = deltaX/deltaCnt*deltaCntNow + McuSendOffsetCopy[numCopy*3];
		  CamData_Ptr->y = deltaY/deltaCnt*deltaCntNow + McuSendOffsetCopy[numCopy*3+1];
		  CamData_Ptr->z = deltaZ/deltaCnt*deltaCntNow + McuSendOffsetCopy[numCopy*3+2];
		  //printf("numCpy=%d numtmp=%d CounterTick=%llu \n",numCopy,numtmp,CounterTick);
	  }
	  scene.encoderData.x = CamData_Ptr->x;
	  scene.encoderData.y = CamData_Ptr->y;
	  scene.encoderData.theta = CamData_Ptr->z;
	  //scene.reset         = 0;
	  //lDcuOutput.PutFrame(lFrame[0].mUMat);
	  //lDcuOutput.PutFrame(lFrame[3].mUMat);
	   for (int i = 0;i < 4;i++)  //*******;************DY****************
	   {
		  if(lpGrabber->FramePush(lFrame[i]) != LIB_SUCCESS)
		  {
		   printf("lpGrabber->FramePush(lFrame[i]) %d\n", i);
			break;
		  } // if push failed
	   }

	   pthread_cond_signal(&videocapture_cond);
	   pthread_mutex_unlock(&videocapture_lock);
	   scene.renderToSreen();
	   flushToScreen(); 
	  // usleep(300000);
	  //printf("wwwwwww50\n");
	  //printf("&&&&&&&&&ParkPlaceNum =%d car_paring_status=%d\n",ParkingPlace_Ptr->ParkPlaceNum,McuSendDada_Ptr->car_paring_status);
  }
} // main()

void *GLTask(void *ptr1)
{
	#if 0
	FILE* fp = fopen("config.txt", "r");
	if (fp == NULL)
	{
		printf("could not found config.txt\n");
		//return -1;
	}
	fscanf(fp, "path: %s\n", path_pattern);
	fscanf(fp, "num_frames: %d\n", &num_frames);
	fclose(fp);
	printf("path = %s", path_pattern);


	// load display settings
	std::ifstream displaySettingFile("param/display.json");
	json displaySetting;
	displaySettingFile >> displaySetting;
	#endif

	//if(initialGL(displaySetting["screen"]["width"], displaySetting["screen"]["height"]) != 0)          //初始化opengl es环境
	if(initialGL(1920, 720) != 0)          //初始化opengl es环境
    {
         fprintf(stderr,"opengl init unsucess!\n");
         //return -1;
    }
    printf("initialGL()::\n");
	auto glVer = glGetString(GL_VERSION);
	if (!glVer)
	{
		printf("glGetString failed: %d\n", glGetError());
	}
	else
	{
		printf("OpenGL version: %s\n", glGetString(GL_VERSION));
	}

    //if(getCameraDevice().initCamera() != 0)   //初始化视频采集设备
    {
         printf("can not initialize the camera device！\n");
         //return -1;
    }
    printf("getCameraDevice()::\n");

    if(initPanoramaParam() != 0){
        printf("can not initialize the camera param");
        //return -1;
    }


    printf("initPanoramaParam() success!\n");
    //keyboard_device::initDevice();                            //使用标准键盘设备
    printf("keyboard_device::initDevice() sucess!\n");
    CFinalScene scene;
	scene.setBirdEyeViewport(1280,
							 0,
							 640,
							 720);
	scene.setSingleViewport(0,
							0,
							1280,
							720);
	scene.set3DViewport(0,
						0,
						1280,
						720);
    scene.onStart();


	//std::cout << displaySetting << std::endl;
    //Calib_2D_Init();
    //Fbo_2D_Init();

    while(1)
    {
    	pthread_mutex_lock(&videocapture_lock);
        glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        int times = 0;
#if 0

        readFromDevice();
	#if 0
        if((times = getKeyTimes(MY_KEY_UP)) > 0)
            scene.onButtonUp(times);
        else if((times = getKeyTimes(MY_KEY_DOWN)) > 0)
            scene.onButtonDown(times);
        else if((times = getKeyTimes(MY_KEY_LEFT)) > 0)
            scene.onButtonLeft(times);
        else if((times = getKeyTimes(MY_KEY_RIGHT)) > 0)
            scene.onButtonRight(times);
		//        else if((times = getKeyTimes(MY_KEY_MENU)) > 0)
		//            scene.onButtonOK();
	#else
		if ((times = getKeyTimes('w')) > 0)
		{
			scene.onButtonMoveUp(times);
		}
		else if ((times = getKeyTimes('s')) > 0)
		{
			scene.onButtonMoveDown(times);
		}
		else if ((times = getKeyTimes('a')) > 0)
		{
			scene.onButtonMoveLeft(times);
		}
		else if ((times = getKeyTimes('d')) > 0)
		{
			scene.onButtonMoveRight(times);
		}
	#endif
		else if ((times = getKeyTimes('i')) > 0)
		{
			scene.onButtonZoomIn(times);
		}
		else if ((times = getKeyTimes('o')) > 0)
		{
			scene.onButtonZoomOut(times);
		}
		else if ((times = getKeyTimes('z')) > 0)
		{
			scene.onButtonSwitchMode(SceneMode::Scene2D);
		}
		else if ((times = getKeyTimes('x')) > 0)
		{
			scene.onButtonSwitchMode(SceneMode::Scene3D);
		}
		else if ((times = getKeyTimes('c')) > 0)
		{
			scene.onButtonSwitchMode(SceneMode::SceneSingle);
		}
		else if ((times = getKeyTimes('v')) > 0)
		{
			scene.onButtonSwitchMode(SceneMode::Scene2DWith3D);
		}
		else if ((times = getKeyTimes('b')) > 0)
		{
			scene.onButtonSwitchMode(SceneMode::Scene2DWithSingle);
		}
		else if ((times = getKeyTimes('n')) > 0)
		{
			scene.onButtonSwitchMode(SceneMode::Scene2DWithZoomFront);
		}
		else if ((times = getKeyTimes('m')) > 0)
		{
			scene.onButtonSwitchMode(SceneMode::Scene2DWithZoomRear);
		}
		else if ((times = getKeyTimes('<')) > 0)
		{
			scene.onButtonSwitchMode(SceneMode::Scene2DWithZoomLeft);
		}
		else if ((times = getKeyTimes('>')) > 0)
		{
			scene.onButtonSwitchMode(SceneMode::Scene2DWithZoomRight);
		}
		else if ((times = getKeyTimes('?')) > 0)
		{
			scene.onButtonSwitchMode(SceneMode::Scene2DWithZoomLeftRight);
		}
		else if ((times = getKeyTimes('[')) > 0)
		{
			scene.onButtonTurnLeft(times);
		}
		else if ((times = getKeyTimes(']')) > 0)
		{
			scene.onButtonTurnRight(times);
		}
        else if((times = getKeyTimes(MY_KEY_RETURN)) > 0)
            break;
        for(int i = 0;i < 9;i++){
            if(times = getKeyTimes('0' + i) > 0){
                scene.onMessage('0' + i);
            }
        }
#endif
		pthread_cond_wait(&videocapture_cond, &videocapture_lock);
        //printf("scene.renderToSreen()\n");
        Timer<> timer;
		timer.start();
		scene.renderToSreen();
        flushToScreen();
		timer.stop();
		//std::cout << "rendering: " << timer.get() << std::endl;
		//dump(1200, 800, 800, 1080-800);
		pthread_mutex_unlock(&videocapture_lock);	
    }
	//程序执行完毕，做一些释放资源的操作
    scene.onStop();
    //getCameraDevice().deinitCamera();
    //keyboard_device::deinitDevice();
    deinitGL();
    return 0;
	#if 0
	readParam();
    if(initialGL() != 0)         // //��ʼ��opengl es����
    {
         fprintf(stderr,"opengl init unsucess!\n");
         return 0;
    }
    printf("initialGL()::\n");
   //if(getCameraDevice().initCamera() != 0)   //��ʼ����Ƶ�ɼ��豸
   // {
   //     printf("can not initialize the camera device\n");
   //     return 0;
   //}
   //printf("getCameraDevice()::\n");
    if(initPanoramaParam() != 0){
        printf("can not initialize the camera param");
        return 0;
    }
    printf("initPanoramaParam() success!\n");
    //keyboard_device::initDevice();                           //ʹ�ñ�׼�����豸
    //printf("keyboard_device::initDevice() sucess!\n");
    CFinalScene scene;
    scene.onStart();

    //Calib_2D_Init();

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT);
		#if 0
        int times = 0;
        readFromDevice();
		usleep(20000);
        if((times = getKeyTimes(MY_KEY_UP)) > 0)
            scene.onButtonUp(times);
        if((times = getKeyTimes(MY_KEY_DOWN)) > 0)
            scene.onButtonDown(times);
        if((times = getKeyTimes(MY_KEY_LEFT)) > 0)
            scene.onButtonLeft(times);
        if((times = getKeyTimes(MY_KEY_RIGHT)) > 0)
            scene.onButtonRight(times);
        if((times = getKeyTimes(MY_KEY_MENU)) > 0)
            scene.onButtonOK();
        if((times = getKeyTimes(MY_KEY_RETURN)) > 0)
            break;
        for(int i = 0;i < 9;i++){
            if(times = getKeyTimes('0' + i) > 0){
                scene.onMessage('0' + i);
            }
        }
		#endif
        //printf("scene.renderToSreen()\n");
        scene.renderToSreen();
        flushToScreen();

    }
	  //����ִ����ϣ���һЩ�ͷ���Դ�Ĳ���
    scene.onStop();
    getCameraDevice().deinitCamera();
    keyboard_device::deinitDevice();
    deinitGL();
    return 0;
	#endif
}

int32_t DdrBuffersAlloc(void** appFbsVirtual,
                        uint32_t*  apFbsPhysical,
                        size_t aMemSize,
                        uint32_t aBufferCnt
                       )
{
  int32_t lRet    = 0;

  // allocate buffers & get physical addresses
  for(uint32_t i = 0; i < aBufferCnt; i++)
  {
    appFbsVirtual[i] = OAL_MemoryAllocFlag(
                          aMemSize,
                          OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|
                          OAL_MEMORY_FLAG_CONTIGUOUS);

    if( appFbsVirtual[i] == NULL)
    {
      lRet = -1;
      break;
    }
    apFbsPhysical[i] = (uint32_t)(uintptr_t)OAL_MemoryReturnAddress(
                          appFbsVirtual[i],
                          ACCESS_PHY); // get physical address

    memset(appFbsVirtual[i], 0x00, aMemSize);
#if defined(__STANDALONE__) && defined(__ARMV8)
    //flush_dcache_range(appFbsVirtual[i], aMemSize);
#endif // #ifdef __STANDALONE__
  } // for all framebuffers

  if(lRet != 0)
  {
    DdrBuffersFree(appFbsVirtual, apFbsPhysical, aBufferCnt);
  }

  return lRet;
} // DdrBuffersAlloc()

//***************************************************************************

void DdrBuffersFree(void** appFbsVirtual,
                    uint32_t*  apFbsPhysical,
                    uint32_t aBufferCnt
                   )
{
  for(uint32_t i = 0; i < aBufferCnt; i++)
  {
    if(appFbsVirtual[i] != NULL)
    {
      OAL_MemoryFree(appFbsVirtual[i]);
    } // if buffer allocated

    appFbsVirtual[i]   = NULL;
    apFbsPhysical[i]   = 0;
  } // for all framebuffers
} // DdrBuffersFree()



//***************************************************************************

void SeqEventCallBack(uint32_t aEventType, void* apUserVal)
{
  static uint32_t slFrmCnt = 0;

  if(aEventType == SEQ_MSG_TYPE_FRAMEDONE)
  {
    //printf("Frame done message arrived #%u. User value %d\n",
    //       slFrmCnt++,
    //       *((int32_t*)apUserVal));
  } // if frame done arrived
} // SeqEventCallBack()

//***************************************************************************

#ifndef __STANDALONE__
void SigintHandler(int)
{
  sStop = true;
} // SigintHandler()

//***************************************************************************

int32_t SigintSetup()
{
  int32_t lRet = SEQ_LIB_SUCCESS;

  // prepare internal signal handler
  struct sigaction lSa;
  memset(&lSa, 0, sizeof(lSa));
  lSa.sa_handler = SigintHandler;

  if( sigaction(SIGINT, &lSa, NULL) != 0)
  {
    VDB_LOG_ERROR("Failed to register signal handler.\n");
    lRet = SEQ_LIB_FAILURE;
  } // if signal not registered

  return lRet;
} // SigintSetup()

//***************************************************************************
#endif // #ifndef __STANDALONE__

int Ipc_Create()
{
  int	 shmid1,shmid2,shmid3,shmid4,shmid5,shmid6,shmid7;
  //���������ڎ� ���൱�ڎ��Č����Č��������򎎜�
  shmid1 = shmget(0x2234, sizeof(CamData), IPC_CREAT | 0666);
  if (shmid1 == -1)
  {
	  perror("shmget 1 err");
	  return errno;
  }
  printf("shmid1:%d \n", shmid1);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  CamData_Ptr =(CamData *)shmat(shmid1, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (CamData_Ptr == (void *)-1 )
  {
	  perror("shmat 1 err");
	  return errno;
  }

  shmid2 = shmget(0x3234, sizeof(ParkingPlace), IPC_CREAT | 0666);
  if (shmid2 == -1)
  {
	  perror("shmget 2 err");
	  return errno;
  }
  printf("shmid2:%d \n", shmid2);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  ParkingPlace_Ptr =(ParkingPlace *)shmat(shmid2, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (ParkingPlace_Ptr == (void *)-1 )
  {
	  perror("shmat 2 err");
	  return errno;
  }

  shmid3 = shmget(0x4234, sizeof(McuSendDada), IPC_CREAT | 0666);
  if (shmid3 == -1)
  {
	  perror("shmget 3 err");
	  return errno;
  }
  printf("shmid3:%d \n", shmid3);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  McuSendDada_Ptr =(McuSendDada *)shmat(shmid3, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (McuSendDada_Ptr == (void *)-1 )
  {
	  perror("shmat 3 err");
	  return errno;
  }

  shmid4 = shmget(0x5234, sizeof(PCWRITEDATA), IPC_CREAT | 0666);
  if (shmid4 == -1)
  {
	  perror("shmget 4 err");
	  return errno;
  }
  printf("shmid4:%d \n", shmid4);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  PCWRITEDATA_Ptr =(PCWRITEDATA *)shmat(shmid4, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (PCWRITEDATA_Ptr == (void *)-1 )
  {
	  perror("shmat 4 err");
	  return errno;
  }

  shmid5 = shmget(0x6234, sizeof(BasicData), IPC_CREAT | 0666);
  if (shmid5 == -1)
  {
	  perror("shmget 5 err");
	  return errno;
  }
  printf("shmid5:%d \n", shmid5);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  BasicData_Ptr =(BasicData *)shmat(shmid5, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (BasicData_Ptr == (void *)-1 )
  {
	  perror("shmat 5 err");
	  return errno;
  }

  shmid6 = shmget(0x7234, sizeof(LanelineData), IPC_CREAT | 0666);
  if (shmid6 == -1)
  {
	  perror("shmget 6 err");
	  return errno;
  }
  printf("shmid6:%d \n", shmid6);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  LanelineData_Ptr =(LanelineData *)shmat(shmid6, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (LanelineData_Ptr == (void *)-1 )
  {
	  perror("shmat 6 err");
	  return errno;
  }

  shmid7 = shmget(0x8234, sizeof(ObstacleData), IPC_CREAT | 0666);
  if (shmid7 == -1)
  {
	  perror("shmget 7 err");
	  return errno;
  }
  printf("shmid7:%d \n", shmid7);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  ObstacleData_Ptr =(ObstacleData *)shmat(shmid7, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (ObstacleData_Ptr == (void *)-1 )
  {
	  perror("shmat 7 err");
	  return errno;
  }

}

  unsigned long long GetTime(void)
{
	Soc_tt=GetNowTimeUs();
	Soc_tt=Soc_tt/1000;
	return McuTimeData + (Soc_tt - Mcu_tt);

}

unsigned long long GetNowTimeUs(void)
 {
	 struct timeval tv;
	 gettimeofday(&tv, NULL);
	 return (unsigned long long)tv.tv_sec * 1000000 + tv.tv_usec;
}