#include <stdio.h>
#include "ImageBilt.h"
#include "ShaderUtil.h"
#include "OGLWin.h"
static GLuint g_shaderProgram = 0;
static GLuint g_positionHandle = 0;     //顶点数组引用
static GLuint g_samplerHandle = 0;
static GLuint g_textCoordHandle = 0;
static GLuint g_imageSizeHandle = 0;
static GLuint g_imageBeginHandle = 0;

static GLuint g_postionVbo = 0;
static GLuint g_textureVbo[2] = {0};
static GLuint g_indexVbo = 0;

int initImageBilt()
{
	if(loadShaders("/data/opengl_new/shader/texture.vert","/data/opengl_new/shader/texture.frag",g_shaderProgram) != 0)
		return -1;
	g_positionHandle = glGetAttribLocation(g_shaderProgram, "aPosition");
	g_samplerHandle = glGetUniformLocation(g_shaderProgram,"sTexture");
	g_textCoordHandle = glGetAttribLocation(g_shaderProgram,"aTexture");
	g_imageBeginHandle = glGetUniformLocation(g_shaderProgram,"uImageBegin");
	g_imageSizeHandle = glGetUniformLocation(g_shaderProgram,"uImageSize");

	float vertexArray[] = {0,0,0,1,1,0,1,1};   //vertex array,(0,0),(0,1),(1,0),(1,1) perspectively
	float lbTextArray[] = {0,1,0,0,1,1,1,0};   //texture array,flip the source image
	float ltTextArray[] = {0,0,0,1,1,0,1,1};   //texture array
	GLubyte indexArray[] = {0, 1, 3,0, 3, 2};  //index array

	glGenBuffers(1, &g_postionVbo);
	glBindBuffer(GL_ARRAY_BUFFER, g_postionVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertexArray),vertexArray, GL_STATIC_DRAW);

	glGenBuffers(2,g_textureVbo);
	glBindBuffer(GL_ARRAY_BUFFER, g_textureVbo[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(lbTextArray),lbTextArray, GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, g_textureVbo[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(ltTextArray),ltTextArray, GL_STATIC_DRAW);

	glGenBuffers(1,&g_indexVbo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_indexVbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indexArray), indexArray, GL_STATIC_DRAW);
	printf("the image bilt init success!\n");
	return 0;
}
void imageBilt(GLuint textureID,float x,float y,float width,float height,int isLeftTop)
{
    //suppose screen size is (1,1),g_imageBeginHandle is position of image,g_imageSizeHandle is size of image
	glUseProgram(g_shaderProgram);
	glUniform2f(g_imageSizeHandle,width,height);
	glUniform2f(g_imageBeginHandle,x,y);
	glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glUniform1i(g_samplerHandle, 0);
	glBindBuffer(GL_ARRAY_BUFFER, g_postionVbo);
	glVertexAttribPointer(g_positionHandle, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ARRAY_BUFFER, g_textureVbo[isLeftTop]);
	glVertexAttribPointer(g_textCoordHandle, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_indexVbo);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnableVertexAttribArray(g_textCoordHandle);
	glEnableVertexAttribArray(g_positionHandle);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, 0);
	glDisableVertexAttribArray(g_positionHandle);
	glDisableVertexAttribArray(g_textCoordHandle);
	glDisable(GL_BLEND);
}
void deinitImageBilt()
{
	glDeleteBuffers(1,&g_postionVbo);
	glDeleteBuffers(2,g_textureVbo);
	glDeleteBuffers(1,&g_indexVbo);
}
