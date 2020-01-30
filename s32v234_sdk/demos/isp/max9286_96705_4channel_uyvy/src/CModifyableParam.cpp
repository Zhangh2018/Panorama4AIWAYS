#include "CModifyableParam.h"
#include <stdio.h>
int loadPanorama3DParam(const char *filename,C3DPanoramaParam& param)
{
      FILE *ifp = fopen(filename,"r");
      if(ifp == NULL){
          fprintf(stderr,"can not open the file: %s\n",filename);
          return -1;
      }
      fscanf(ifp,"mergeMinAngle:%f\n",&param.m_mergeMinAngle);
      fscanf(ifp,"mergeMaxAngle:%f\n",&param.m_mergeMaxAngle);
      fscanf(ifp,"innerRadius:%f\n",&param.m_innerRadius);
      fscanf(ifp,"outerRadius:%f\n",&param.m_outerRadius);
      fscanf(ifp,"cameraHeight:%f\n",&param.m_cameraHeight);
      fscanf(ifp,"observerHeight:%f\n",&param.m_observerHeight);
      fscanf(ifp,"sizePerGrid:%f\n",&param.m_sizePerGrid);
      fscanf(ifp,"reGenrateModel:%d\n",&param.m_reGenerateModel);
      fclose(ifp);
      return 0;
}
int printPanorama3DParam(FILE *ofp,const C3DPanoramaParam& param)
{
      fprintf(ofp,"mergeMinAngle: %f\n",param.m_mergeMinAngle);
      fprintf(ofp,"mergeMaxAngle: %f\n",param.m_mergeMaxAngle);
      fprintf(ofp,"innerRadius: %f\n",param.m_innerRadius);
      fprintf(ofp,"outerRadius: %f\n",param.m_outerRadius);
      fprintf(ofp,"cameraHeight: %f\n",param.m_cameraHeight);
      fprintf(ofp,"observerHeight: %f\n",param.m_observerHeight);
      fprintf(ofp,"sizePerGrid: %f\n",param.m_sizePerGrid);
      fprintf(ofp,"reGenrateModel: %d\n",param.m_reGenerateModel);
      return 0;
}
int savePanorama3DParam(const char *filename,const C3DPanoramaParam& param)
{
      FILE *ofp = fopen(filename,"w");
      if(ofp == 0)
      {
           fprintf(stderr,"can not open the file: %s",filename);
           return -1;
      }
      printPanorama3DParam(ofp,param);
      fclose(ofp);
      return 0;
}
int loadPanoramaGlobalParam(const char *filename,CGlobalSetParam& param)
{
      FILE *ifp = fopen(filename,"r");
      if(ifp == NULL){
          fprintf(stderr,"can not open the file: %s\n",filename);
          return -1;
      }
      fscanf(ifp,"themeColor:%d\n",&param.m_themeColor);
      fscanf(ifp,"colsePanorama:%d\n",&param.m_isColsePanorama);
      fscanf(ifp,"isResponseTurn:%d\n",&param.m_isResponseTurn);
      fscanf(ifp,"paranoraScale:%d\n",&param.m_paranoraScale);
      fscanf(ifp,"displayMode:%d\n",&param.m_displayMode);
      fscanf(ifp,"recordTime:%d\n",&param.m_recordTime);
      fscanf(ifp,"delayTime:%d\n",&param.m_delayTime);
      fscanf(ifp,"lightness:%d\n",&param.m_lightness);
      fclose(ifp);
      return 0;
}
int printPanoramaGlobalParam(FILE *ofp,const CGlobalSetParam& param)
{
      fprintf(ofp,"themeColor: %d\n",param.m_themeColor);
      fprintf(ofp,"colsePanorama: %d\n",param.m_isColsePanorama);
      fprintf(ofp,"isResponseTurn: %d\n",param.m_isResponseTurn);
      fprintf(ofp,"paranoraScale: %d\n",param.m_paranoraScale);
      fprintf(ofp,"displayMode: %d\n",param.m_displayMode);
      fprintf(ofp,"recordTime: %d\n",param.m_recordTime);
      fprintf(ofp,"delayTime: %d\n",param.m_delayTime);
      fprintf(ofp,"lightness: %d\n",param.m_lightness);
      return 0;
}
int savePanoramaGlobalParam(const char *filename,const CGlobalSetParam& param)
{
      FILE *ofp = fopen(filename,"w");
      if(ofp == 0)
      {
           fprintf(stderr,"can not open the file: %s",filename);
           return -1;
      }
      printPanoramaGlobalParam(ofp,param);
      fclose(ofp);
      return 0;
}
int loadPanoramaCarParam(const char *filename,CCarParam &param)
{
     FILE *ifp = fopen(filename,"r");
     if(ifp == NULL){
          fprintf(stderr,"can not open the file: %s\n",filename);
          return -1;
      }
     fscanf(ifp,"carLength:%f\n",&param.m_carLength);
     fscanf(ifp,"carWidth:%f\n",&param.m_carWidth);
     fscanf(ifp,"carHeight:%f\n",&param.m_carHeight);
     fscanf(ifp,"frontBlank:%f\n",&param.m_frontBlank);
     fscanf(ifp,"rearBlank:%f\n",&param.m_rearBlank);
     fscanf(ifp,"isDisplayBlank:%d\n",&param.m_isDisplayBlank);
     fclose(ifp);
     return 0;
}
int printPanoramaCarParam(FILE *ofp,const CCarParam &param)
{
     fprintf(ofp,"carLength: %f\n",param.m_carLength);
     fprintf(ofp,"carWidth: %f\n",param.m_carWidth);
     fprintf(ofp,"carHeight: %f\n",param.m_carHeight);
     fprintf(ofp,"frontBlank: %f\n",param.m_frontBlank);
     fprintf(ofp,"rearBlank: %f\n",param.m_rearBlank);
     fprintf(ofp,"isDisplayBlank: %d\n",param.m_isDisplayBlank);
     return 0;
}
int savePanoramaCarParam(const char *filename,const CCarParam &param)
{
      FILE *ofp = fopen(filename,"w");
      if(ofp == 0)
      {
           fprintf(stderr,"can not open the file: %s",filename);
           return -1;
      }
      printPanoramaCarParam(ofp,param);
      fclose(ofp);
      return 0;
}

