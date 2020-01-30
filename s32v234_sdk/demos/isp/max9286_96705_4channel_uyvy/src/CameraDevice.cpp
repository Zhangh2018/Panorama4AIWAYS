#include "CameraDevice.h"
#include "const_value.h"
#include "max9286_96705_4_c.h"

#include <opencv2/core.hpp>

//摄像头读取设备类
using namespace const_value;
int32_t ddrBuffersAlloc(void** appFbsVirtual, uint32_t*  apFbsPhysical,size_t aMemSize, uint32_t aBufferCnt);
void ddrBuffersFree(void** appFbsVirtual, uint32_t*  apFbsPhysical,uint32_t aBufferCnt);
CCameraVedioDevice::CCameraVedioDevice()
{
    m_lpFdma = NULL;
    m_lpGrabber = NULL;
}
CCameraVedioDevice::~CCameraVedioDevice()
{
	deinitCamera();
}
int CCameraVedioDevice::initCamera()
{
    int lRes = OAL_Initialize();
    //printf("lRes = %d\n",lRes);
    lRes = sdi::Initialize(0);
    //printf("lRes = %d\n",lRes);
    m_lpGrabber = new(sdi_grabber);
    m_lpGrabber->ProcessSet(gpGraph, &gGraphMetadata);
    m_lpFdma = (sdi_FdmaIO*)m_lpGrabber->IoGet(SEQ_OTHRIX_FDMA);
    for(int i = 0; i < CAMERANUM; i++)
    {
        SDI_ImageDescriptor lFrmDesc = SDI_ImageDescriptor(SRC_IMAGE_WIDTH, SRC_IMAGE_HEIGHT, YUV422Stream_UYVY);
        m_lpFdma->DdrBufferDescSet(i, lFrmDesc);
    }
    m_lpFdma->DdrBuffersAlloc(SRC_FRAME_BUFFERS);
    m_lpGrabber->PreStart();
}
void CCameraVedioDevice::deinitCamera()
{
    m_lpGrabber->Release();
    delete m_lpGrabber;
    sdi::Close(0);
    OAL_Deinitialize();
}
//从摄像头设备index中取出一个缓冲区
vsdk::SUMat &CCameraVedioDevice::getBuffer(int index)
{
   m_lFrame[index] = m_lpGrabber->FramePop(index);
   return m_lFrame[index].mUMat;
}
//归还缓冲区
void CCameraVedioDevice::giveBackBuffer(int index)
{
    m_lpGrabber->FramePush(m_lFrame[index]);
}
//开始采集
int CCameraVedioDevice::startCapture(int width,int height)
{
    m_lpGrabber->Start();
}
//直接从文件中读取yuvy格式的图像，并返回
#if 1
vsdk::SUMat &CCameraVedioDevice::getImageBuffer(int index)
{
    static vsdk::SUMat *uyvyMat[4] = {NULL};
    if(uyvyMat[index] == NULL)
	{
            uyvyMat[index] = new vsdk::SUMat(SRC_IMAGE_HEIGHT,SRC_IMAGE_WIDTH,CV_8UC2);
        //yvyMat[index] = new vsdk::UMat(SRC_IMAGE_HEIGHT,SRC_IMAGE_WIDTH,CV_8UC2);
        vsdk::SMat mat = uyvyMat[index]->getMat(vsdk::ACCESS_RW | OAL_USAGE_NONCACHED);
        uint8_t *ptr = mat.data;
        static const char *direction[4] = {"front","back","left","right"};
        char filename[200];
            sprintf(filename,"/data/opengl_new/UYVY/%s.yuv",direction[index]);
            FILE *ifp = fopen(filename,"rb");
            fread(ptr,SRC_IMAGE_WIDTH * SRC_IMAGE_HEIGHT * 2,1,ifp);
            fclose(ifp);
    }		
    return *uyvyMat[index];
}
#endif
 //   vsdk::UMat &get_Calib_ImageBuffer(int index);
#if 0
vsdk::SUMat &CCameraVedioDevice::get_Calib_ImageBuffer(int index,unsigned char *buffer)
{
    static vsdk::SUMat *uyvyMat[1] = {NULL};
    if(uyvyMat[index] == NULL){
            uyvyMat[index] = new vsdk::SUMat(SRC_IMAGE_HEIGHT,SRC_IMAGE_WIDTH,CV_8UC2);
        vsdk::SMat mat = uyvyMat[index]->getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
        uint8_t *ptr = mat.data;

        //memcpy(ptr,buffer,720*1280*2);
    }
    return *uyvyMat[0];
}
#endif
void CCameraVedioDevice::giveBackImageBuffer(int index)
{

}
void CCameraVedioDevice::stopCapture()
{
    m_lpGrabber->Stop();
}
//分配缓冲区资源
int32_t ddrBuffersAlloc(void** appFbsVirtual,uint32_t*  apFbsPhysical,size_t aMemSize,uint32_t aBufferCnt)
{
    int32_t lRet = 0;
    // 分配缓冲区和物理地址
    for(uint32_t i = 0; i < aBufferCnt; i++)
    {
        appFbsVirtual[i] = OAL_MemoryAllocFlag(aMemSize,OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
        if(appFbsVirtual[i] == NULL)
        {
            lRet = -1;
            break;
        }
        apFbsPhysical[i] = (uint32_t)(uintptr_t)OAL_MemoryReturnAddress(appFbsVirtual[i],ACCESS_PHY); // get physical address
        memset(appFbsVirtual[i], 0x00, aMemSize);
    }
    if(lRet != 0)
    {
        ddrBuffersFree(appFbsVirtual, apFbsPhysical, aBufferCnt);
    }
    return lRet;
}
//释放缓冲区资源
void ddrBuffersFree(void** appFbsVirtual,uint32_t*  apFbsPhysical,uint32_t aBufferCnt)
{
    for(uint32_t i = 0; i < aBufferCnt; i++)
    {
        if(appFbsVirtual[i] != NULL)
        {
            OAL_MemoryFree(appFbsVirtual[i]);
        }
        appFbsVirtual[i]   = NULL;
        apFbsPhysical[i]   = 0;
    }
}
CCameraVedioDevice &getCameraDevice()
{
    static CCameraVedioDevice cameraDevice;
    return cameraDevice;
}
