#ifndef DYNAMICTEXTURE_H
#define DYNAMICTEXTURE_H
#include "IL/il.h"
#include "GLES2/gl2.h"

int initTextureEnvir();
int deinitTextureEnvir();

GLuint createStaticTexture(const char *textureFile);
GLuint createDynamicYUYVTexture(int width,int height,GLvoid **buffer);
void updateYUYVTexture(GLuint textureID,GLvoid *buffer,int width,int height);

GLuint createDynamicYUYVTexture2(int width,int height,GLvoid **buffer);
void updateYUYVTexture2(int Camera_Flag, GLuint textureID,GLvoid *buffer,int width,int height);

GLuint createDynamicRGBTexture(int width,int height,void *buffer);
void updateRGBTexture(GLuint textureID,char *buffer);





/***********************************************************************************/
void Calib_2D(int Camera_Flag, uint8_t *UYVY_DATA_BUFFER,int width,int height);
#endif // DYNAMICTEXTURE_H
