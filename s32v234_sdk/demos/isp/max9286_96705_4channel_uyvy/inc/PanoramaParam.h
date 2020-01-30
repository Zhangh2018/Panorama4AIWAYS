#ifndef _CAMERA_PARAM_H__
#define _CAMERA_PARAM_H__
#include "ImageUtilStruct.h"
#include "CModifyableParam.h"
#include <string>
extern CCarParam g_carParam;
extern CGlobalSetParam g_globalParam;
extern float g_luminParam[3];
extern float g_globalCompensate[4];
extern C3DPanoramaParam  g_3dParam;

int initPanoramaParam();
void deinitPanoramaParam();

void setCarParam(const CCarParam &param);
void setGlobalParam(const CGlobalSetParam &param);
void setGlobalLuminParam(float luminParam[4]);

#endif

