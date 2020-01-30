#include "TextureIL.h"
#include "IL/il.h"
#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <map>
typedef void (GL_APIENTRY *PFNGLTEXDIRECTVIV)(GLenum Target, GLsizei Width, GLsizei Height, GLenum Format, GLvoid ** Pixels);
typedef void (GL_APIENTRY *PFNGLTEXDIRECTINVALIDATEVIV)(GLenum Target);
typedef void (GL_APIENTRY *PFNEEGLTEXDIRECTVIVMAP) (GLenum Target, GLsizei Width, GLsizei Height, GLenum Format, GLvoid ** Pixels, const GLuint * Physical);
static PFNEEGLTEXDIRECTVIVMAP pFNEEglTexDirectVIVMap = NULL;
static PFNGLTEXDIRECTVIV pFNglTexDirectVIV = NULL;
static PFNGLTEXDIRECTINVALIDATEVIV pFNglTexDirectInvalidateVIV = NULL;
static std::map<GLuint,ILuint> dynamicTextures;
static int gIsILInit = 0;
int initTextureEnvir()
{
    if (pFNglTexDirectVIV == NULL)
    {
        pFNglTexDirectVIV = (PFNGLTEXDIRECTVIV)eglGetProcAddress("glTexDirectVIV");
        if (pFNglTexDirectVIV == NULL)
        {
            printf("Required extension not supported.\n");
            return -1;
        }
    }
    if (pFNglTexDirectInvalidateVIV == NULL)
    {
        pFNglTexDirectInvalidateVIV = (PFNGLTEXDIRECTINVALIDATEVIV)eglGetProcAddress("glTexDirectInvalidateVIV");
        if (pFNglTexDirectInvalidateVIV == NULL)
        {
            printf("Required extension not supported.\n");
            return -1;
        }
    }
    if (pFNEEglTexDirectVIVMap == NULL)
    {
        pFNEEglTexDirectVIVMap = (PFNEEGLTEXDIRECTVIVMAP)eglGetProcAddress("glTexDirectVIVMap");

        if (pFNEEglTexDirectVIVMap == NULL)
        {
            printf("Required extension not supported!\n");
            return -1;
        }
    }
    if(!gIsILInit)
    {
        ilInit();
        gIsILInit = 1;
    }
    return 0;
}
int deinitTextureEnvir()
{
    std::map<GLuint,ILuint>::iterator it = dynamicTextures.begin();
    for(;it != dynamicTextures.end();it++){
        ilDeleteImages(1, &(it->second));
        glDeleteTextures(1,&it->first);
    }
    if(gIsILInit)
    {
        ilShutDown();
        gIsILInit = 0;
    }
    return 0;
}
GLuint createStaticTexture(const char *textureFile)
{
    ILuint imageIds;
    ilGenImages(1, &imageIds);
    GLuint textureObject = 0;
    ilBindImage(imageIds); /* Binding of DevIL image name */
    ilEnable(IL_ORIGIN_SET);
    ilOriginFunc(IL_ORIGIN_LOWER_LEFT);
    ILboolean success = ilLoadImage((ILstring)(textureFile));
    if (success)
    {
        glGenTextures(1, &textureObject);
        glBindTexture(GL_TEXTURE_2D, textureObject);
        ilConvertImage(IL_RGBA, IL_UNSIGNED_BYTE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ilGetInteger(IL_IMAGE_WIDTH),ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGBA, GL_UNSIGNED_BYTE,ilGetData());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    }
    else
    {
        ILenum errorEnum = ilGetError();
        printf("errorEnum ilLoadImage: %i,%s\n", errorEnum,textureFile);
    }
    ilDeleteImages(1, &imageIds);
    return textureObject;
}
GLuint createDynamicYUYVTexture(int width,int height,GLvoid **buffer)
{
    void *planes[3] = {NULL};
    GLuint textureObject = 0;
    glGenTextures(1, &textureObject);
    glBindTexture(GL_TEXTURE_2D, textureObject);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    pFNglTexDirectVIV(GL_TEXTURE_2D,width,height,GL_VIV_UYVY,(GLvoid**)&planes);
    if (glGetError() != GL_NO_ERROR)
    {
        printf("pFNglTexDirectVIV error!\n");
    }
    *buffer = planes[0];
    return textureObject;
}
void updateYUYVTexture(GLuint textureID,GLvoid *buffer,int width,int height)
{
	glBindTexture(GL_TEXTURE_2D, textureID);
	pFNglTexDirectInvalidateVIV(GL_TEXTURE_2D);
	if (glGetError() != GL_NO_ERROR)
    {
        printf("pFNglTexDirectInvalidateVIV error,textureId = %d!\n",textureID);
    }
	return;
}
GLuint createDynamicRGBTexture(int width,int height,void *buffer)
{
    ILuint imageIds;
    ilGenImages(1, &imageIds);
    GLuint textureObject = 0;
    ilBindImage(imageIds); /* Binding of DevIL image name */
    ilTexImage(width,height,0,3,IL_RGB,IL_UNSIGNED_BYTE, buffer);
    glGenTextures(1, &textureObject);
    glBindTexture(GL_TEXTURE_2D, textureObject);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width,height, 0, GL_RGB, GL_UNSIGNED_BYTE,ilGetData());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    dynamicTextures[textureObject] = imageIds;
    return textureObject;
}
void updateRGBTexture(GLuint textureID,char *buffer)
{
    ILuint imageILID = dynamicTextures[textureID];
    ilEnable(IL_ORIGIN_SET);
    ilOriginFunc(IL_ORIGIN_UPPER_LEFT);
    ilBindImage(imageILID); /* Binding of DevIL image name */
    ilSetData(buffer);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB, GL_UNSIGNED_BYTE,ilGetData());
}

GLuint createDynamicYUYVTexture2(int width,int height,GLvoid **buffer)
{
    GLuint textureObject = 0;
    glGenTextures(1, &textureObject);
    glBindTexture(GL_TEXTURE_2D, textureObject);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    return textureObject;
}
void updateYUYVTexture2(int Camera_Flag,GLuint textureID,GLvoid *buffer,int width,int height)
{
        void* logical = buffer;
        GLuint physical = ~0U;
	glBindTexture(GL_TEXTURE_2D, textureID);
        pFNEEglTexDirectVIVMap(GL_TEXTURE_2D, width, height, GL_VIV_YUY2, &logical, &physical);
	pFNglTexDirectInvalidateVIV(GL_TEXTURE_2D);
}
