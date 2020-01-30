#ifndef _IMAGE_BILT_H__
#define _IMAGE_BILT_H__
#include "GLES2/gl2.h"
int initImageBilt();
void imageBilt(GLuint textureID,float x,float y,float width,float height,int isLeftTop = 0);
void deinitImageBilt();
#endif

