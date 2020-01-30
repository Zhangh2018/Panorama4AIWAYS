#include "PanoramaParam.h"
#include "ImageUtilStruct.h"
#include "GLES2/gl2.h"
#include <memory.h>
#include "const_value.h"
#include "camera_model.h"
using namespace const_value;
CCarParam g_carParam;
CGlobalSetParam g_globalParam;
C3DPanoramaParam  g_3dParam;
float g_luminParam[3] = {0.0};
float g_globalCompensate[4] = {0,0,0,0};

int loadLocalLuminParam()
{
    FILE *ifp = fopen("/data/opengl_new/param/lumin_param.txt","r");
    if(ifp == NULL)
    {
        fprintf(stderr,"can not open the file: param/lumin_param.txt\n");
        return -1;
    }
    fscanf(ifp,"luminParam:%f%f%f",&g_luminParam[0],&g_luminParam[1],&g_luminParam[2]);
    fclose(ifp);
    return 0;
}
void saveLocalLuminParam()
{
    FILE *ofp = fopen("/data/opengl_new/param/lumin_param.txt","r");
    fprintf(ofp,"luminParam: %f\t%f\t%f\n",g_luminParam[0],g_luminParam[1],g_luminParam[2]);
    fclose(ofp);
}
int initPanoramaParam()
{
    std::string directionName[4] = {"front","rear","left","right"};
    load_ocam_params();
    if(loadPanoramaCarParam("/data/opengl_new/param/car_param.txt",g_carParam) == -1){
        return -1;
    }
    if(loadPanoramaGlobalParam("/data/opengl_new/param/global_param.txt",g_globalParam) == -1){
        return -1;
    }
    if(loadPanorama3DParam("/data/opengl_new/param/panorama3d_param.txt",g_3dParam) == -1){
        return -1;
    }
    if(loadLocalLuminParam() == -1){
        return -1;
    }
    return 0;
}
void deinitPanoramaParam()
{
    savePanorama3DParam("/data/opengl_new/param/panorama3d_param.txt",g_3dParam);
}
void setCarParam(const CCarParam &param)
{
    g_carParam = param;
}
void setGlobalParam(const CGlobalSetParam &param)
{
    g_globalParam = param;
}
void setGlobalLuminParam(float luminParam[4])
{
    memcpy(g_globalCompensate,luminParam,sizeof(float) * 4);
}

