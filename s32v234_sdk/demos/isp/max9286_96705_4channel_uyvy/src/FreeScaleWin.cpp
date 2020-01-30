#include <cstdio>
#include "OGLWin.h"

static EGLNativeDisplayType g_nativeDisplay = 0;
static EGLNativeWindowType g_windowType = 0;
static int g_width = 0;
static int g_height = 0;

int getScreenWidth()
{
         return g_width;
}
int getScreenHeight()
{
         return g_height;
}
EGLNativeWindowType getNativeWindowType()
{
         return g_windowType;
}
EGLNativeDisplayType getNativeDisplayType()
{
         return g_nativeDisplay;
}
void destroyWindow()
{

}

int createWindow(int width, int height)
{
	g_width = width;
	g_height = height;


	g_nativeDisplay = fbGetDisplayByIndex(0);
	printf("fbGetdisplay return 0x%x\n", g_nativeDisplay);
	//fbGetDisplayGeometry(g_nativeDisplay, &g_width, &g_height);
	printf("Display w,h %d,%d\n",g_width, g_height);
	g_windowType  = fbCreateWindow(g_nativeDisplay, 0, 0,g_width, g_height);
	printf("fbCreateWindow return 0x%x\n", g_windowType);
	return 0;
}
