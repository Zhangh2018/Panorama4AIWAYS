#ifndef _CMODIFYABLE_PARAM_H__
#define _CMODIFYABLE_PARAM_H__
#include <stdio.h>
struct CCalibrationParam
{
    float m_clothLength;
    float m_frontClothHeight;
    float m_leftInnerClothWidth;
    float m_leftOuterClothWidth;
    CCalibrationParam(float clothLength = 0,float frontClothHeight = 0,float leftInnerClothWidth = 0,float leftOuterClothWidth = 0)
    {
        m_clothLength = clothLength;
        m_frontClothHeight = frontClothHeight;
        m_leftInnerClothWidth = leftInnerClothWidth;
        m_leftOuterClothWidth = leftOuterClothWidth;
    }
};
struct C3DPanoramaParam
{
        float m_mergeMinAngle;
        float m_mergeMaxAngle;
        float m_innerRadius;
        float m_outerRadius;
        float m_cameraHeight;
        float m_observerHeight;
        float m_sizePerGrid;
        int m_reGenerateModel;
    C3DPanoramaParam(float mergeMinAngle = 0,float mergeMaxAngle = 0,float innerRadius = 0,
                     float outerRadius = 0,float cameraHeight = 0,float observerHeight = 0,float sizePerGrid = 0,int reGenerateModel = 0)
    {
        m_mergeMinAngle = mergeMinAngle;
        m_mergeMaxAngle = mergeMaxAngle;
        m_innerRadius = innerRadius;
        m_outerRadius = outerRadius;
        m_cameraHeight = cameraHeight;
        m_observerHeight = observerHeight;
        m_sizePerGrid = sizePerGrid;
        m_reGenerateModel = reGenerateModel;
    }
};
struct CGlobalSetParam
{
    int m_themeColor;
    int m_isColsePanorama;
    int m_isResponseTurn;
    int m_paranoraScale;
    int m_displayMode;
    int m_recordTime;
    int m_delayTime;
    int m_lightness;
    CGlobalSetParam(int themeColor = 0,int isColsePanorama = 0,int isResponseTurn = 0,int paranoraScale = 0,int displayMode = 0)
    {
        m_themeColor = themeColor;
        m_isColsePanorama = isColsePanorama;
        m_isResponseTurn = isResponseTurn;
        m_paranoraScale = paranoraScale;
        m_displayMode = displayMode;
    }
};
struct CCarParam
{
    float m_carLength;
    float m_carWidth;
    float m_carHeight;
    float m_frontBlank;
    float m_rearBlank;
    int m_isDisplayBlank;
    CCarParam(float carLength = 0.0,float carWidth = 0.0,float carHeight = 0.0,float frontBlank = 0,float rearBlank = 0,int isDisplay = 0)
    {
        m_carLength = carLength;
        m_carWidth = carWidth;
        m_carHeight = carHeight;
        m_frontBlank = frontBlank;
        m_rearBlank = rearBlank;
        m_isDisplayBlank = isDisplay;
    }
};

int loadPanoramaCarParam(const char *filename,CCarParam &param);
int printPanoramaCarParam(FILE *ofp,const CCarParam &param);
int savePanoramaCarParam(const char *filename,const CCarParam &param);

int loadPanorama3DParam(const char *filename,C3DPanoramaParam& param);
int printPanorama3DParam(FILE *ofp,const C3DPanoramaParam& param);
int savePanorama3DParam(const char *filename,const C3DPanoramaParam& param);

int loadPanoramaGlobalParam(const char *filename,CGlobalSetParam& param);
int printPanoramaGlobalParam(FILE *ofp,const CGlobalSetParam& param);
int savePanoramaGlobalParam(const char *filename,const CGlobalSetParam& param);

#endif
