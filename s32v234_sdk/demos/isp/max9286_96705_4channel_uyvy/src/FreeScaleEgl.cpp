#include "OGLEGL.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>
#include <string.h>

#include <EGL/egl.h>

static EGLDisplay g_egldisplay = 0;
static EGLSurface g_eglsurface = 0;

static void handle_egl_error(const char *name)
{
	EGLint error_code = eglGetError();
	printf("%s:error = %d,exit!\n", name, error_code);
	exit(EXIT_FAILURE);
}

void initEGL(EGLNativeDisplayType nativeDisplay, EGLNativeWindowType windowType)
{
	static const EGLint sConfigAttribs[] =
		{
			EGL_SAMPLES, 0,
			EGL_RED_SIZE, 8,
			EGL_GREEN_SIZE, 8,
			EGL_BLUE_SIZE, 8,
			EGL_ALPHA_SIZE, 8,
			EGL_DEPTH_SIZE, 16,
			EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
			EGL_NONE,
		};
	EGLint numconfigs;
	g_egldisplay = eglGetDisplay(nativeDisplay);
	eglInitialize(g_egldisplay, NULL, NULL);
	eglBindAPI(EGL_OPENGL_ES_API);
	if (eglGetError() != EGL_SUCCESS)
	{
		handle_egl_error("eglInitialize");
	}
	EGLConfig eglconfig;
	eglChooseConfig(g_egldisplay, sConfigAttribs, &eglconfig, 1, &numconfigs);
	if (eglGetError() != EGL_SUCCESS)
	{
		handle_egl_error("eglChooseConfig");
	}
	g_eglsurface = eglCreateWindowSurface(g_egldisplay, eglconfig, windowType, NULL);
	if (g_eglsurface == EGL_NO_SURFACE)
	{
		handle_egl_error("eglCreateWindowSurface");
	}
	EGLint ctxAttribs[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};
	EGLContext eglcontext = eglCreateContext(g_egldisplay, eglconfig, 0, ctxAttribs);
	if (eglcontext == EGL_NO_CONTEXT)
	{
		handle_egl_error("eglCreateContext");
	}
	eglMakeCurrent(g_egldisplay, g_eglsurface, g_eglsurface, eglcontext);
	if (eglGetError() != EGL_SUCCESS)
	{
		handle_egl_error("eglMakeCurrent");
	}
	else
	{
		printf("eglMakeCurrent OK\n");
	}
#if 1
	if (eglSwapInterval(g_egldisplay, 1) != EGL_TRUE)
	{
		handle_egl_error("eglSwapInterval");
	}
	else
	{
		printf("set eglSwapInterval to 1\n");
	}
	eglSwapInterval(g_egldisplay, 1);
#endif
}

void deinitEGL(void)
{
	eglMakeCurrent(g_egldisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
	assert(eglGetError() == EGL_SUCCESS);
	eglTerminate(g_egldisplay);
	assert(eglGetError() == EGL_SUCCESS);
}
void swapBuffers()
{
	eglSwapBuffers(g_egldisplay, g_eglsurface);
	assert(eglGetError() == EGL_SUCCESS);
}
