#ifndef _OGL_EGL_H_
#define _OGL_EGL_H_
#include <EGL/egl.h>


void initEGL(EGLNativeDisplayType nativeDisplay,EGLNativeWindowType windowType);

void deinitEGL(void);

void swapBuffers();

#endif
