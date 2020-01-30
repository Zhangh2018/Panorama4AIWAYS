#ifndef CAMERADEVICE_H
#define CAMERADEVICE_H

#include "sdi.hpp"
#include "oal.h"
#include "frame_output_v234fb.h"

typedef vsdk::SUMat MatType;

class CCameraVedioDevice
{
public:
    CCameraVedioDevice();
    virtual ~CCameraVedioDevice();
    MatType &getBuffer(int index);
    MatType &getImageBuffer(int index);

    MatType &get_Calib_ImageBuffer(int index,unsigned char *buffer);


    void giveBackBuffer(int index);
    void giveBackImageBuffer(int index);
    int startCapture(int width,int height);
    void stopCapture();
    int initCamera();
    void deinitCamera();

protected:
private:
    sdi_FdmaIO *m_lpFdma;
    sdi_grabber *m_lpGrabber;
    SDI_Frame m_lFrame[4];

    void realtimeCapture();
};
CCameraVedioDevice &getCameraDevice();

#endif // CAMERADEVICE_H
