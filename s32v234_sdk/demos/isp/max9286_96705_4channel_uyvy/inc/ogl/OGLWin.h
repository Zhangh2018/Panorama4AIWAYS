#ifndef _OGL_WINDOW_H_
#define _OGL_WINDOW_H_

#include <EGL/egl.h>

EGLNativeWindowType getNativeWindowType();
EGLNativeDisplayType getNativeDisplayType();
void destroyWindow();
int createWindow(int width, int height);
EGLNativeDisplayType fslGetNativeDisplay();
int getScreenWidth();
int getScreenHeight();

#endif
