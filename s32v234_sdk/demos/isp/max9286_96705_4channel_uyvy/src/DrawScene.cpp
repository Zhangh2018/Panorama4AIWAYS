#include "DrawScene.h"
#include <stdlib.h>
//#include <math.h>
#include "const_value.h"
#include "OGLEGL.h"
#include "OGLWin.h"
#include "ImageBilt.h"
#include "TextureIL.h"
#include "GLES2/gl2.h"
#include "ShaderUtil.h"
#include <math.h>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include "dump.h"
#include "utils/Timer.hpp"

#include <GLES3/gl3.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>

#include <map>
#include <string>

using namespace const_value;
using namespace std;
using namespace cv;

#include "LUT.h"


#define FRONT_BACK_ROW  (50)
#define FRONT_BACK_COL  (100)
//#define CALIB_FB_ROW    (291)//(301)
//#define CALIB_FB_COL    (810)


#define LEFT_RIGHT_ROW  (200)
#define LEFT_RIGHT_COL  (50)
//#define CALIB_LR_ROW    (1080)
//#define CALIB_LR_COL    (311)


#define MERGE_FRONT_LEFT_ROW    (100)
#define MERGE_FRONT_LEFT_COL    (100)
//#define CALIB_FL_ROW            (291)//(301)
//#define CALIB_FL_COL            (311)
/***************SingleView*************/
#define SINGLE_VIEW_ROW    (20)
#define SINGLE_VIEW_COL    (20)
//#define CALIB_SINGLE_VIEW_ROW            (810)
//#define CALIB_SINGLE_VIEW_COL            (1110)
/***************SingleView*************/


int bevHeight = 0;
int bevWidth = 0;
int singleViewWidth = 0;
int singleViewHeight = 0;
int CALIB_FB_ROW = 0,
	CALIB_FB_COL = 0,
	CALIB_LR_ROW = 0,
	CALIB_LR_COL = 0,
	CALIB_FL_ROW = 0,
	CALIB_FL_COL = 0,
	CALIB_SINGLE_VIEW_ROW = 0,
	CALIB_SINGLE_VIEW_COL = 0;

cv::Rect carIconRect;

void setBirdEyeViewParams(const CalibLUT &lut)
{
	auto carRect = lut.header.carRealRect;

	bevWidth = lut.header.bev_img_width;
	bevHeight = lut.header.bev_img_height;

	CALIB_FB_ROW = carRect.y;
	CALIB_FB_COL = bevWidth;

	CALIB_LR_ROW = bevHeight;
	CALIB_LR_COL = carRect.x;

	CALIB_FL_ROW = carRect.y;
	CALIB_FL_COL = carRect.x;

	carIconRect = lut.header.car_Icon_Rect;
}

void setSingleViewParams(int width, int height)
{
	width = 1110;
	height = 810;
	singleViewWidth = width;
	singleViewHeight = height;
	CALIB_SINGLE_VIEW_ROW = height;
	CALIB_SINGLE_VIEW_COL = width;
}

glm::mat4 bevTransformMatrix(1.0);

extern float view2DScale;
extern float hOffset;
extern float vOffset;

/**************************************************************Init Value start************************************************************************/
//unmerged
GLuint m_vbo_front[4] = {0}, m_vbo_back[4] = {0},m_vbo_left[4] = {0},m_vbo_right[4] = {0};
float VertexPositions_Front[FRONT_BACK_COL*FRONT_BACK_ROW*6*3] ={0.0},VertexPositions_Back[FRONT_BACK_COL*FRONT_BACK_ROW*6*3] ={0.0};
float VertexPositions_Left[LEFT_RIGHT_ROW*LEFT_RIGHT_COL*6*3] ={0.0},VertexPositions_Right[LEFT_RIGHT_ROW*LEFT_RIGHT_COL*6*3] ={0.0};

float VertexColors_Front[FRONT_BACK_COL*FRONT_BACK_ROW*6*2] ={0.0},VertexColors_Back[FRONT_BACK_COL*FRONT_BACK_ROW*6*2] ={0.0};
float VertexColors_Left[LEFT_RIGHT_ROW*LEFT_RIGHT_COL*6*2] ={0.0},VertexColors_Right[LEFT_RIGHT_ROW*LEFT_RIGHT_COL*6*2] ={0.0};
unsigned int Index_Array_FB[FRONT_BACK_COL*FRONT_BACK_ROW*6] = {0},Index_Array_LR[LEFT_RIGHT_ROW*LEFT_RIGHT_COL*6] = {0};
//merge
GLuint m_vbo0_Merge[6] = {0},m_vbo1_Merge[6] = {0},m_vbo2_Merge[6] = {0},m_vbo3_Merge[6] = {0};
GLfloat VerPos_Merge[MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*6*3] ={0.0};
GLfloat VerColor1_Merge[MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*6*2] ={0.0},VerColor2_Merge[MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*6*2] ={0.0};
GLfloat Vertex_Weight[MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*6*2] ={0.0};
unsigned int IndexArray_Merge[MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*6] = {0};
/***********************************************************Init Value end*****************************************************************************/


int initialGL (int width, int height)
{
    using namespace const_value;
    if(createWindow(width, height) != 0)
        return -1;
    //printf("initialGL1()::\n");
    initEGL(getNativeDisplayType(),getNativeWindowType());
    //printf("initialGL2()::\n");
    initTextureEnvir();
    //printf("initialGL3()::\n");
    glClearColor( 0.0f, 0.0f, 0.0f, 0.0f);
    glViewport(0, 0, getScreenWidth(), getScreenHeight());
    glDisable(GL_DEPTH_TEST);
    initImageBilt();

	printf("============ GLES info begin =============\n");
	printf("GL_VERSION = %s\n", glGetString(GL_VERSION));
	printf("GL_VENDOR = %s\n", glGetString(GL_VENDOR));
	printf("GL_EXTENSIONS = %s\n", glGetString(GL_EXTENSIONS));
	printf("============ GLES info end =============\n");

    //printf("initialGL4()::\n");
    return 0;
}
int render4SrcImage(GLuint *textureIds,int FLAG_Test)
{
    if(FLAG_Test == 0)
        imageBilt(textureIds[0],0.4,0.0,0.5,0.8,1);
    else if(FLAG_Test == 1)
        imageBilt(textureIds[1],0.4,0.0,0.5,0.8,1);
    else if(FLAG_Test == 2)
        imageBilt(textureIds[2],0.4,0.0,0.5,0.8,1);
    else if(FLAG_Test == 3)
        imageBilt(textureIds[3],0.4,0.0,0.5,0.8,1);
}
//int renderBird_View(GLuint *textureIds,int Flag_Test);
int renderBird_View(GLuint *textureIds,int FLAG_Test)
{
    imageBilt(textureIds[0],0.0,0.0,0.4,1.0,1);
}
void Calib_2D_Init()
{
	//printf("ok\n");
/********************************************************************start Init*********************************************************************************/
	glGenBuffers(4, m_vbo_front);
	glGenBuffers(4, m_vbo_back);
	glGenBuffers(4, m_vbo_left);
	glGenBuffers(4, m_vbo_right);
	//merge
	glGenBuffers(6, m_vbo0_Merge);
	glGenBuffers(6, m_vbo1_Merge);
	glGenBuffers(6, m_vbo2_Merge);
	glGenBuffers(6, m_vbo3_Merge);
	for(int Camera_ID = 0;Camera_ID < 4;Camera_ID++)
	{
		switch(Camera_ID)
		{
			case 0:
				Calib_2D_Ver_FrontBack(Camera_ID,VertexPositions_Front,VertexColors_Front,Index_Array_FB,-1.0,1.0,2.0*CALIB_FB_COL/bevWidth,2.0*CALIB_FB_ROW/CALIB_LR_ROW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo_front[0]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*FRONT_BACK_ROW*FRONT_BACK_COL*3,VertexPositions_Front, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo_front[1]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*FRONT_BACK_ROW*FRONT_BACK_COL*2,VertexColors_Front, GL_STATIC_DRAW);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_front[2]);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*FRONT_BACK_ROW*FRONT_BACK_COL, Index_Array_FB, GL_STATIC_DRAW);//
				break;
			case 1:
				Calib_2D_Ver_FrontBack(Camera_ID,VertexPositions_Back,VertexColors_Back,Index_Array_FB,-1.0,2.0*CALIB_FB_ROW/CALIB_LR_ROW-1.0,2.0*CALIB_FB_COL/bevWidth,2.0*CALIB_FB_ROW/CALIB_LR_ROW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo_back[0]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*FRONT_BACK_ROW*FRONT_BACK_COL*3,VertexPositions_Back, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo_back[1]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*FRONT_BACK_ROW*FRONT_BACK_COL*2,VertexColors_Back, GL_STATIC_DRAW);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_back[2]);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*FRONT_BACK_ROW*FRONT_BACK_COL, Index_Array_FB, GL_STATIC_DRAW);//
				break;
			case 2:
				Calib_2D_Ver_FrontBack(Camera_ID,VertexPositions_Left,VertexColors_Left,Index_Array_LR,-1.0,1.0,2.0*CALIB_LR_COL/bevWidth,2.0*CALIB_LR_ROW/bevHeight);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo_left[0]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*LEFT_RIGHT_ROW*LEFT_RIGHT_COL*3,VertexPositions_Left, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo_left[1]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*LEFT_RIGHT_ROW*LEFT_RIGHT_COL*2,VertexColors_Left, GL_STATIC_DRAW);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_left[2]);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*LEFT_RIGHT_ROW*LEFT_RIGHT_COL, Index_Array_LR, GL_STATIC_DRAW);//
				break;
			case 3:
				Calib_2D_Ver_FrontBack(Camera_ID,VertexPositions_Right,VertexColors_Right,Index_Array_LR,-1.0+2.0*(CALIB_FB_COL-CALIB_LR_COL)/bevWidth,1.0,2.0*CALIB_LR_COL/bevWidth,2.0*CALIB_LR_ROW/bevHeight);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo_right[0]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*LEFT_RIGHT_ROW*LEFT_RIGHT_COL*3,VertexPositions_Right, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo_right[1]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*LEFT_RIGHT_ROW*LEFT_RIGHT_COL*2,VertexColors_Right, GL_STATIC_DRAW);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_right[2]);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*LEFT_RIGHT_ROW*LEFT_RIGHT_COL, Index_Array_LR, GL_STATIC_DRAW);
				break;
			default:
				printf("function : Calib_2D_Init() switch can't find proper Value.\n");
				break;
		}//end : switch(Camera_ID)
	}//end : for(int Camera_ID = 0;Camera_ID < 4;Camera_ID++)
//printf("function : Calib_2D_Init() switch can't find proper Value.\n");
/****************************************************************************************end Init**************************************************************/
/********************************************************************start Init*********************************************************************************/

	for(int Camera_ID = 0;Camera_ID < 4;Camera_ID++)
	{
		switch(Camera_ID)
		{
			case 0:
				Merge_Calib_2D_Ver_FrontBack(Camera_ID,VerPos_Merge,Vertex_Weight,VerColor1_Merge,VerColor2_Merge,IndexArray_Merge,-1.0,1.0,2.0*CALIB_LR_COL/bevWidth,2.0*CALIB_FB_ROW/bevHeight);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_Merge[0]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*3,VerPos_Merge, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_Merge[1]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*2,VerColor1_Merge, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_Merge[2]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*2,VerColor2_Merge, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_Merge[3]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL,Vertex_Weight, GL_STATIC_DRAW);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo0_Merge[4]);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL, IndexArray_Merge, GL_STATIC_DRAW);
				break;
			case 1:
				Merge_Calib_2D_Ver_FrontBack(Camera_ID,VerPos_Merge,Vertex_Weight,VerColor1_Merge,VerColor2_Merge,IndexArray_Merge,-1.0+2.0*(CALIB_FB_COL-CALIB_LR_COL)/bevWidth,1.0,2.0*CALIB_LR_COL/bevWidth,2.0*CALIB_FB_ROW/bevHeight);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_Merge[0]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*3,VerPos_Merge, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_Merge[1]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*2,VerColor1_Merge, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_Merge[2]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*2,VerColor2_Merge, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_Merge[3]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL,Vertex_Weight, GL_STATIC_DRAW);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo1_Merge[4]);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL, IndexArray_Merge, GL_STATIC_DRAW);
				break;
			case 2:
				Merge_Calib_2D_Ver_FrontBack(Camera_ID,VerPos_Merge,Vertex_Weight,VerColor1_Merge,VerColor2_Merge,IndexArray_Merge,-1.0,-1.0+2.0*CALIB_FB_ROW/bevHeight,2.0*CALIB_LR_COL/bevWidth,2.0*CALIB_FB_ROW/bevHeight);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_Merge[0]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*3,VerPos_Merge, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_Merge[1]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*2,VerColor1_Merge, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_Merge[2]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*2,VerColor2_Merge, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_Merge[3]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL,Vertex_Weight, GL_STATIC_DRAW);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo2_Merge[4]);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL, IndexArray_Merge, GL_STATIC_DRAW);
				break;
			case 3:
				Merge_Calib_2D_Ver_FrontBack(Camera_ID,VerPos_Merge,Vertex_Weight,VerColor1_Merge,VerColor2_Merge,IndexArray_Merge,-1.0+2.0*(CALIB_FB_COL-CALIB_LR_COL)/bevWidth,-1.0+2.0*CALIB_FB_ROW/bevHeight,2.0*CALIB_LR_COL/bevWidth,2.0*CALIB_FB_ROW/bevHeight);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_Merge[0]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*3,VerPos_Merge, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_Merge[1]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*2,VerColor1_Merge, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_Merge[2]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL*2,VerColor2_Merge, GL_STATIC_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_Merge[3]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL,Vertex_Weight, GL_STATIC_DRAW);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo3_Merge[4]);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL, IndexArray_Merge, GL_STATIC_DRAW);
				break;
			default:
				printf("function : Calib_2D_Init() switch can't find proper Value.\n");
				break;
		}//end : switch(Camera_ID)
	}//end : for(int Camera_ID = 0;Camera_ID < 4;Camera_ID++)
/********************************************************************end unmerge Init********************************************************************************/

}//end : void Calib_2D_Init()


void Generate_2D_View(GLuint fbo, GLuint Camera_Num_Flag,GLuint *textureID, const float* delta, const float* mask)
{
	static GLuint m_shaderProgram = 0;
	static GLuint m_positionHandle = 0;
	static GLuint m_colorHandle = 0;
	static GLuint m_textCoordHandle = 0;
	static GLuint m_samplerHandle = 0;
	static GLuint m_TransformMat = 0;
	static GLuint m_deltaHandle = 0;
	static GLuint m_transformMatrixHandle = 0;
	static GLuint m_maskHandle = 0;

	static GLuint count_num = 0;
	static bool Flag_cal = false;
	static float angle = 0.0f;
	angle = 3.141592654 / 2;
	// static float transformMatrix[16] = {
	// 	1, 0, 0, 0,
	// 	0, 1, 0, 0,
	// 	0, 0, 1, 0,
	// 	0, 0, 0, 1};
	//transformMatrix[0] = (GLfloat)cos(angle);
	//transformMatrix[5] = (GLfloat)cos(angle);
	//transformMatrix[1] = (GLfloat)sin(angle);
	//transformMatrix[4] = -(GLfloat)sin(angle);
	//angle += 3.1415926/2;
	if (!Flag_cal)
	{
		if (loadShaders("/data/opengl_new/shader/unmerge2d.vert", "/data/opengl_new/shader/unmerge2d.frag", m_shaderProgram) != 0)
			printf("return error at Generate_2D_View!!!\n");
		m_positionHandle = glGetAttribLocation(m_shaderProgram, "aPosition");
	 	m_colorHandle = glGetAttribLocation(m_shaderProgram, "a_Color");
		m_textCoordHandle = glGetAttribLocation(m_shaderProgram, "aTexCoord");
		m_samplerHandle = glGetUniformLocation(m_shaderProgram, "my_Sampler");
		//m_TransformMat = glGetUniformLocation(m_shaderProgram, "uProjMatrix");
		m_deltaHandle = glGetUniformLocation(m_shaderProgram, "uDelta");
		m_transformMatrixHandle = glGetUniformLocation(m_shaderProgram, "uTransformMatrix");
		m_maskHandle = glGetUniformLocation(m_shaderProgram, "uMask");
	}
	Flag_cal = true;
	count_num++;


	// glm::mat4 transformMatrix(1.0);
	// transformMatrix = glm::rotate(transformMatrix, glm::radians(-90.0f), glm::vec3(0.0, 0.0, 1.0));
	// transformMatrix = glm::scale(transformMatrix, glm::vec3(view2DScale, view2DScale, 1.0));
	// transformMatrix = glm::translate(transformMatrix, glm::vec3(hOffset, vOffset, 0.0));
	// for (int i = 0; i < 4; ++i)
	// {
	// 	for (int j = 0; j < 4; ++j)
	// 	{
	// 		printf("%5.3f\t", transformMatrix[j][i]);
	// 	}
	// 	printf("\n");
	// }
	// printf("=============================\n");

	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	if (Camera_Num_Flag == 0)
	{
		glUseProgram(m_shaderProgram);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textureID[0]);
		glUniform1i(m_samplerHandle, 0);

		glUniform1f(m_deltaHandle, delta[Camera_Num_Flag]);

		glUniformMatrix4fv(m_transformMatrixHandle, 1, GL_FALSE, (GLfloat*)glm::value_ptr(bevTransformMatrix));
		glUniform4fv(m_maskHandle, 1, mask);

		glBindBuffer(GL_ARRAY_BUFFER, m_vbo_front[0]);
		glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 3,
							  0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
		glBindBuffer(GL_ARRAY_BUFFER, m_vbo_front[1]);
		glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 2, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_front[2]);//

		glEnableVertexAttribArray(m_positionHandle);
		glEnableVertexAttribArray(m_textCoordHandle);
		glEnableVertexAttribArray(m_samplerHandle);
		glEnableVertexAttribArray(m_colorHandle);
		glDrawElements(GL_TRIANGLES, FRONT_BACK_COL * FRONT_BACK_ROW * 6, GL_UNSIGNED_INT, 0);
		glDisableVertexAttribArray(m_positionHandle);
		glDisableVertexAttribArray(m_textCoordHandle);
		glDisableVertexAttribArray(m_samplerHandle);
		glDisableVertexAttribArray(m_colorHandle);
	}
	else if (Camera_Num_Flag == 1)
	{
		glUseProgram(m_shaderProgram);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textureID[1]);
		glUniform1i(m_samplerHandle, 0);
		glUniform1f(m_deltaHandle, delta[Camera_Num_Flag]);

		glUniformMatrix4fv(m_transformMatrixHandle, 1, GL_FALSE, (GLfloat*)glm::value_ptr(bevTransformMatrix));
		glUniform4fv(m_maskHandle, 1, mask);

		glBindBuffer(GL_ARRAY_BUFFER, m_vbo_back[0]);
		glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 3,
							  0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
		glBindBuffer(GL_ARRAY_BUFFER, m_vbo_back[1]);
		glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 2, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_back[2]);//

		glEnableVertexAttribArray(m_positionHandle);
		glEnableVertexAttribArray(m_textCoordHandle);
		glEnableVertexAttribArray(m_samplerHandle);
		glEnableVertexAttribArray(m_colorHandle);
		glDrawElements(GL_TRIANGLES, FRONT_BACK_COL * FRONT_BACK_ROW * 6, GL_UNSIGNED_INT, 0);
		glDisableVertexAttribArray(m_positionHandle);
		glDisableVertexAttribArray(m_textCoordHandle);
		glDisableVertexAttribArray(m_samplerHandle);
		glDisableVertexAttribArray(m_colorHandle);
	}
	else if (Camera_Num_Flag == 2)
	{
		glUseProgram(m_shaderProgram);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textureID[2]);
		glUniform1i(m_samplerHandle, 0);
		glUniform1f(m_deltaHandle, delta[Camera_Num_Flag]);

		glUniformMatrix4fv(m_transformMatrixHandle, 1, GL_FALSE, (GLfloat*)glm::value_ptr(bevTransformMatrix));
		glUniform4fv(m_maskHandle, 1, mask);

		glBindBuffer(GL_ARRAY_BUFFER, m_vbo_left[0]);
		glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 3,
							  0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
		glBindBuffer(GL_ARRAY_BUFFER, m_vbo_left[1]);
		glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 2, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_left[2]);
		glEnableVertexAttribArray(m_positionHandle);
		glEnableVertexAttribArray(m_textCoordHandle);
		glEnableVertexAttribArray(m_samplerHandle);
		glEnableVertexAttribArray(m_colorHandle);
		glDrawElements(GL_TRIANGLES, 6 * LEFT_RIGHT_ROW * LEFT_RIGHT_COL, GL_UNSIGNED_INT, 0);
		glDisableVertexAttribArray(m_positionHandle);
		glDisableVertexAttribArray(m_textCoordHandle);
		glDisableVertexAttribArray(m_samplerHandle);
		glDisableVertexAttribArray(m_colorHandle);
	}
	else if (Camera_Num_Flag == 3)
	{
		glUseProgram(m_shaderProgram);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textureID[3]);
		glUniform1i(m_samplerHandle, 0);
		glUniform1f(m_deltaHandle, delta[Camera_Num_Flag]);

		glUniformMatrix4fv(m_transformMatrixHandle, 1, GL_FALSE, (GLfloat*)glm::value_ptr(bevTransformMatrix));
		glUniform4fv(m_maskHandle, 1, mask);

		glBindBuffer(GL_ARRAY_BUFFER, m_vbo_right[0]);
		glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 3,
							  0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
		glBindBuffer(GL_ARRAY_BUFFER, m_vbo_right[1]);
		glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 2, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_right[2]);//

		glEnableVertexAttribArray(m_positionHandle);
		glEnableVertexAttribArray(m_textCoordHandle);
		glEnableVertexAttribArray(m_samplerHandle);
		glEnableVertexAttribArray(m_colorHandle);
		glDrawElements(GL_TRIANGLES, 6 * LEFT_RIGHT_ROW * LEFT_RIGHT_COL, GL_UNSIGNED_INT, 0);
		glDisableVertexAttribArray(m_positionHandle);
		glDisableVertexAttribArray(m_textCoordHandle);
		glDisableVertexAttribArray(m_samplerHandle);
		glDisableVertexAttribArray(m_colorHandle);
	}
}

void Generate_2D_View_Merge(GLuint fbo, GLuint Camera_Num_Flag,GLuint *textureID, const float* delta, const float* mask)
{
	static GLuint m_shaderProgram = 0;
	static GLuint m_positionHandle = 0;
	static GLuint m_weightHandle = 0;
	static GLuint m_texCoord1Handle = 0;
	static GLuint m_texCoord2Handle = 0;
	static GLuint m_sampler1Handle = 0;
	static GLuint m_sampler2Handle = 0;
	static GLuint m_colorHandle = 0;
	static GLuint m_delta1Handle = 0;
	static GLuint m_delta2Handle = 0;
	static GLuint m_transformMatrixHandle = 0;
	static GLuint m_maskHandle = 0;

	static GLuint count_num = 0;
	static bool Flag_cal = false;
	static float angle = 3.141592654 / 2;
	// static float transformMatrix[16] = {
	// 	1, 0, 0, 0,
	// 	0, 1, 0, 0,
	// 	0, 0, 1, 0,
	// 	0, 0, 0, 1};
	//transformMatrix[0] = (GLfloat)cos(angle);
	//transformMatrix[5] = (GLfloat)cos(angle);
	//transformMatrix[1] = (GLfloat)sin(angle);
	//transformMatrix[4] = -(GLfloat)sin(angle);
	if (!Flag_cal)
	{
		if (loadShaders("/data/opengl_new/shader/merge2d.vert", "/data/opengl_new/shader/merge2d.frag", m_shaderProgram) != 0)
			printf("return error!!!\n");
//属性参数,分别为位置，距离和权重
		m_positionHandle = glGetAttribLocation(m_shaderProgram, "aPosition");
		m_colorHandle = glGetAttribLocation(m_shaderProgram, "a_Color");
		m_weightHandle = glGetAttribLocation(m_shaderProgram, "aWeight");
		m_texCoord1Handle = glGetAttribLocation(m_shaderProgram, "aTexCoord1");
		m_texCoord2Handle = glGetAttribLocation(m_shaderProgram, "aTexCoord2");
//纹理采样参数
		m_sampler1Handle = glGetUniformLocation(m_shaderProgram, "sTexture1");
		m_sampler2Handle = glGetUniformLocation(m_shaderProgram, "sTexture2");

		//m_TransformMat = glGetUniformLocation(m_shaderProgram, "uProjMatrix");

		m_delta1Handle = glGetUniformLocation(m_shaderProgram, "uDelta1");
		m_delta2Handle = glGetUniformLocation(m_shaderProgram, "uDelta2");

		m_transformMatrixHandle = glGetUniformLocation(m_shaderProgram, "uTransformMatrix");
		m_maskHandle = glGetUniformLocation(m_shaderProgram, "uMask");
	}
	Flag_cal = true;

	// glm::mat4 transformMatrix(1.0);
	// transformMatrix = glm::rotate(transformMatrix, glm::radians(-90.0f), glm::vec3(0.0, 0.0, 1.0));
	// transformMatrix = glm::scale(transformMatrix, glm::vec3(view2DScale, view2DScale, 1.0));
	// transformMatrix = glm::translate(transformMatrix, glm::vec3(hOffset, vOffset, 0.0));
	// for (int i = 0; i < 4; ++i)
	// {
	// 	for (int j = 0; j < 4; ++j)
	// 	{
	// 		printf("%5.3f\t", transformMatrix[j][i]);
	// 	}
	// 	printf("\n");
	// }
	// printf("=============================\n");


	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	if(Camera_Num_Flag == 0){
    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID[0]);
    glUniform1i(m_sampler1Handle, 0);
    glActiveTexture (GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textureID[2]);
    glUniform1i(m_sampler2Handle, 1);

	glUniform1f(m_delta1Handle, delta[0]);
	glUniform1f(m_delta2Handle, delta[2]);

	glUniformMatrix4fv(m_transformMatrixHandle, 1, GL_FALSE, (GLfloat*)glm::value_ptr(bevTransformMatrix));
		glUniform4fv(m_maskHandle, 1, mask);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_Merge[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_Merge[1]);
        glVertexAttribPointer(m_texCoord1Handle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_Merge[2]);
        glVertexAttribPointer(m_texCoord2Handle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_Merge[3]);
        glVertexAttribPointer(m_weightHandle, 1, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*1, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo0_Merge[4]);//
    }
    else if(Camera_Num_Flag == 1){
    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID[0]);
    glUniform1i(m_sampler1Handle, 0);
    glActiveTexture (GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textureID[3]);
    glUniform1i(m_sampler2Handle, 1);
	glUniform1f(m_delta1Handle, delta[0]);
	glUniform1f(m_delta2Handle, delta[3]);

	glUniformMatrix4fv(m_transformMatrixHandle, 1, GL_FALSE, (GLfloat*)glm::value_ptr(bevTransformMatrix));
		glUniform4fv(m_maskHandle, 1, mask);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_Merge[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_Merge[1]);
        glVertexAttribPointer(m_texCoord1Handle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_Merge[2]);
        glVertexAttribPointer(m_texCoord2Handle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_Merge[3]);
        glVertexAttribPointer(m_weightHandle, 1, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*1, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo1_Merge[4]);//
    }
    else if(Camera_Num_Flag == 2){
    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID[1]);
    glUniform1i(m_sampler1Handle, 0);
    glActiveTexture (GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textureID[2]);
    glUniform1i(m_sampler2Handle, 1);

	glUniform1f(m_delta1Handle, delta[1]);
	glUniform1f(m_delta2Handle, delta[2]);

	glUniformMatrix4fv(m_transformMatrixHandle, 1, GL_FALSE, (GLfloat*)glm::value_ptr(bevTransformMatrix));
		glUniform4fv(m_maskHandle, 1, mask);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_Merge[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_Merge[1]);
        glVertexAttribPointer(m_texCoord1Handle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_Merge[2]);
        glVertexAttribPointer(m_texCoord2Handle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_Merge[3]);
        glVertexAttribPointer(m_weightHandle, 1, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*1, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo2_Merge[4]);//
    }
    else if(Camera_Num_Flag == 3){
    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID[1]);
    glUniform1i(m_sampler1Handle, 0);
    glActiveTexture (GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textureID[3]);
    glUniform1i(m_sampler2Handle, 1);

	glUniform1f(m_delta1Handle, delta[1]);
	glUniform1f(m_delta2Handle, delta[3]);

	glUniformMatrix4fv(m_transformMatrixHandle, 1, GL_FALSE, (GLfloat*)glm::value_ptr(bevTransformMatrix));
		glUniform4fv(m_maskHandle, 1, mask);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_Merge[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_Merge[1]);
        glVertexAttribPointer(m_texCoord1Handle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_Merge[2]);
        glVertexAttribPointer(m_texCoord2Handle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_Merge[3]);
        glVertexAttribPointer(m_weightHandle, 1, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*1, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo3_Merge[4]);//
    }
    glEnableVertexAttribArray(m_positionHandle);
	glEnableVertexAttribArray(m_texCoord1Handle);
	glEnableVertexAttribArray(m_texCoord2Handle);
	glEnableVertexAttribArray(m_weightHandle);
	glEnableVertexAttribArray(m_colorHandle);
	glDrawElements(GL_TRIANGLES, 6*MERGE_FRONT_LEFT_ROW*MERGE_FRONT_LEFT_COL, GL_UNSIGNED_INT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_texCoord1Handle);
	glDisableVertexAttribArray(m_texCoord2Handle);
	glDisableVertexAttribArray(m_weightHandle);
	glDisableVertexAttribArray(m_colorHandle);
}
void Calib_2D_Ver_FrontBack(GLuint Camera_Num_Flag,GLfloat *Vertex_pos,GLfloat *Vertex_tex,unsigned int *Vertex_index,GLfloat START_X,GLfloat START_Y,GLfloat WIDTH,GLfloat HEIGHT)
{
   FILE *fp ;
   //printf("WIDTH HEIGHT::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::%f\t%f\n",WIDTH,HEIGHT);
   float temp_x1=START_X,temp_y1=START_Y,temp_x2,temp_y2;
   float X_JUMP = 1.0*WIDTH/FRONT_BACK_COL,Y_JUMP = 1.0*HEIGHT/FRONT_BACK_ROW;//Y_JUMP = (1.0*CALIB_FB_ROW/CALIB_LR_ROW)*2.0/FRONT_BACK_ROW;
   float X_JUMP_LR = 1.0*WIDTH/LEFT_RIGHT_COL,Y_JUMP_LR = 1.0*HEIGHT/LEFT_RIGHT_ROW;
   float X_COUNT,Y_COUNT;
    unsigned int* front_Calib = new unsigned int[CALIB_FB_ROW*CALIB_FB_COL];
    unsigned int* LR_Calib = new unsigned int[CALIB_LR_ROW*CALIB_LR_COL];
printf("Calib_2D_Ver_FrontBack(Camera_Num_Flag:%d) WIDTH:%f\tHEIGHT:%f\n",Camera_Num_Flag,WIDTH,HEIGHT);
    if(Camera_Num_Flag == 0){
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Front.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(front_Calib, sizeof(unsigned int), CALIB_FB_ROW*CALIB_FB_COL, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 1){
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Back.bin", "rb");//Lut_Back
	fseek(fp,0,SEEK_SET);
	fread(front_Calib, sizeof(unsigned int), CALIB_FB_ROW*CALIB_FB_COL, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 2){
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Left.bin", "rb");//Lut_Back
	fseek(fp,0,SEEK_SET);
	fread(LR_Calib, sizeof(unsigned int), CALIB_LR_ROW*CALIB_LR_COL, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 3){
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Right.bin", "rb");//Lut_Back
	fseek(fp,0,SEEK_SET);
	fread(LR_Calib, sizeof(unsigned int), CALIB_LR_ROW*CALIB_LR_COL, fp);
	fclose(fp);
    }
	//printf("Camera_Num_Flag:%d\tfind error333\n",Camera_Num_Flag);
   	/****************/
printf("(Camera_Num_Flag:%d) X_JUMP:%f\tY_JUMP:%f\n",Camera_Num_Flag,X_JUMP,Y_JUMP);
    if(Camera_Num_Flag == 0 || Camera_Num_Flag == 1){
	for(int i=0;i<FRONT_BACK_ROW;i++)
	{
		//temp_y1 = START_Y-i*Y_JUMP;
		temp_y1 = START_Y-1.0*i*HEIGHT/FRONT_BACK_ROW;
   	//printf("i,temp_y1::%d\t%f\n",i,temp_y1);
        for(int j=0;j<FRONT_BACK_COL;j++)
          {
            //temp_x1 =START_X+j*X_JUMP;WIDTH/FRONT_BACK_COL
	    temp_x1 =START_X+1.0*j*WIDTH/FRONT_BACK_COL;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+0] = temp_x1;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+1] = temp_y1;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+2] = 0;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+3] = temp_x1+X_JUMP;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+4] = temp_y1;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+5] = 0;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+6] = temp_x1+X_JUMP;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+7] = temp_y1-Y_JUMP;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+8] = 0;                  //FIRST TRIANGLE

            Vertex_pos[(i*FRONT_BACK_COL+j)*18+9] =  temp_x1;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+10] = temp_y1-Y_JUMP;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+11] = 0;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+12] = temp_x1+X_JUMP;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+13] = temp_y1-Y_JUMP;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+14] = 0;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+15] = temp_x1;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+16] = temp_y1;
            Vertex_pos[(i*FRONT_BACK_COL+j)*18+17] = 0;                 //SECOND TRIANGLE
          }
	}
    //X_COUNT = int(480.0/FRONT_BACK_ROW);//CALIB_FB_ROW
    X_COUNT = 1.0*CALIB_FB_ROW/FRONT_BACK_ROW;//CALIB_FB_ROW
    Y_COUNT = 1.0*CALIB_FB_COL/FRONT_BACK_COL;
    //printf("X_COUNT Y_COUNT%f\t%f\n",X_COUNT,Y_COUNT);
    for(int i=0;i<FRONT_BACK_ROW;i++)
	{
        for(int j=0;j<FRONT_BACK_COL;j++)
        {//             temp_y = (front_Calib[i*720+j]>>10)&0x7ff;
         //             temp_x = (front_Calib[i*720+j]>>21)&0x7ff;

            Vertex_tex[(i*FRONT_BACK_COL+j)*12+0] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*(Y_COUNT))]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*FRONT_BACK_COL+j)*12+1] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*(Y_COUNT))]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*FRONT_BACK_COL+j)*12+2] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*(Y_COUNT)+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*FRONT_BACK_COL+j)*12+3] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*(Y_COUNT)+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*FRONT_BACK_COL+j)*12+4] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*(Y_COUNT)+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*FRONT_BACK_COL+j)*12+5] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*(Y_COUNT)+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*FRONT_BACK_COL+j)*12+6] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*(Y_COUNT))]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*FRONT_BACK_COL+j)*12+7] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*(Y_COUNT))]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*FRONT_BACK_COL+j)*12+8] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*(Y_COUNT)+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*FRONT_BACK_COL+j)*12+9] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*(Y_COUNT)+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*FRONT_BACK_COL+j)*12+10] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*Y_COUNT)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*FRONT_BACK_COL+j)*12+11] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*Y_COUNT)]>>21)&0x7ff)/720.0;
        }//UPDATE X&Y
	}
//printf("hello error 1111\n");
	for(int i=0;i<FRONT_BACK_COL*FRONT_BACK_ROW*6;i++)
	{
        	Vertex_index[i] = i;
    	}
    }
    if(Camera_Num_Flag == 2 || Camera_Num_Flag == 3){//CALIB_FB_ROW*CALIB_FB_COL  #define LEFT_RIGHT_ROW  (128)#define LEFT_RIGHT_COL  (50)
	for(int i=0;i<LEFT_RIGHT_ROW;i++)//CALIB_LR_ROW*CALIB_LR_COL
	{
		//temp_y1 = START_Y-i*Y_JUMP_LR;
		temp_y1 = START_Y-1.0*i*HEIGHT/LEFT_RIGHT_ROW;
        for(int j=0;j<LEFT_RIGHT_COL;j++)
          {
            //temp_x1 =START_X+j*X_JUMP_LR;

	    temp_x1 =START_X+1.0*j*WIDTH/LEFT_RIGHT_COL;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+0] = temp_x1;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+1] = temp_y1;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+2] = 0;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+3] = temp_x1+X_JUMP_LR;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+4] = temp_y1;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+5] = 0;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+6] = temp_x1+X_JUMP_LR;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+7] = temp_y1-Y_JUMP_LR;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+8] = 0;                  //FIRST TRIANGLE

            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+9] =  temp_x1;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+10] = temp_y1-Y_JUMP_LR;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+11] = 0;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+12] = temp_x1+X_JUMP_LR;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+13] = temp_y1-Y_JUMP_LR;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+14] = 0;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+15] = temp_x1;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+16] = temp_y1;
            Vertex_pos[(i*LEFT_RIGHT_COL+j)*18+17] = 0;                 //SECOND TRIANGLE
          }
	}
    //X_COUNT = int(480.0/FRONT_BACK_ROW);//CALIB_FB_ROW
    X_COUNT = 1.0*CALIB_LR_ROW/LEFT_RIGHT_ROW;//CALIB_FB_ROW
    Y_COUNT = 1.0*CALIB_LR_COL/LEFT_RIGHT_COL;
    //printf("X_COUNT Y_COUNT%f\t%f\n",X_COUNT,Y_COUNT);
    for(int i=0;i<LEFT_RIGHT_ROW;i++)
	{
        for(int j=0;j<LEFT_RIGHT_COL;j++)
        {
            Vertex_tex[(i*LEFT_RIGHT_COL+j)*12+0] = ((LR_Calib[int(i*X_COUNT)*CALIB_LR_COL+int(j*(Y_COUNT))]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*LEFT_RIGHT_COL+j)*12+1] = ((LR_Calib[int(i*X_COUNT)*CALIB_LR_COL+int(j*(Y_COUNT))]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*LEFT_RIGHT_COL+j)*12+2] = ((LR_Calib[int(i*X_COUNT)*CALIB_LR_COL+int(j*(Y_COUNT)+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*LEFT_RIGHT_COL+j)*12+3] = ((LR_Calib[int(i*X_COUNT)*CALIB_LR_COL+int(j*(Y_COUNT)+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*LEFT_RIGHT_COL+j)*12+4] = ((LR_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_LR_COL+int(j*(Y_COUNT)+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*LEFT_RIGHT_COL+j)*12+5] = ((LR_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_LR_COL+int(j*(Y_COUNT)+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*LEFT_RIGHT_COL+j)*12+6] = ((LR_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_LR_COL+int(j*(Y_COUNT))]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*LEFT_RIGHT_COL+j)*12+7] = ((LR_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_LR_COL+int(j*(Y_COUNT))]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*LEFT_RIGHT_COL+j)*12+8] = ((LR_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_LR_COL+int(j*(Y_COUNT)+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*LEFT_RIGHT_COL+j)*12+9] = ((LR_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_LR_COL+int(j*(Y_COUNT)+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*LEFT_RIGHT_COL+j)*12+10] = ((LR_Calib[int(i*X_COUNT)*CALIB_LR_COL+int(j*Y_COUNT)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*LEFT_RIGHT_COL+j)*12+11] = ((LR_Calib[int(i*X_COUNT)*CALIB_LR_COL+int(j*Y_COUNT)]>>21)&0x7ff)/720.0;
        }//UPDATE X&Y
	}
	for(int i=0;i<LEFT_RIGHT_COL*LEFT_RIGHT_ROW*6;i++)
	{
        	Vertex_index[i] = i;
    	}
    }

    delete[] front_Calib;
    delete[] LR_Calib;
}
void Merge_Calib_2D_Ver_FrontBack(GLuint Camera_Num_Flag,GLfloat *Vertex_pos,GLfloat *Vertex_Weight,GLfloat *Vertex_tex1,GLfloat *Vertex_tex2,unsigned int *Vertex_index,GLfloat START_X,GLfloat START_Y,GLfloat WIDTH,GLfloat HEIGHT)
{
   	FILE *fp ;
   	float temp_x1=START_X,temp_y1=START_Y,temp_x2,temp_y2;
   	float X_JUMP = 1.0*WIDTH/MERGE_FRONT_LEFT_COL,Y_JUMP = 1.0*HEIGHT/MERGE_FRONT_LEFT_ROW;
   	//printf("X_JUMPM,Y_JUMPM::%f\t,%f\n",X_JUMP,Y_JUMP);
    float X_COUNT,Y_COUNT;
    int COL_OFFSET,ROW_OFFSET;
    /***************************************************///front  CALIB_FB_COL
    unsigned int* front_Calib = new unsigned int[CALIB_FB_ROW*CALIB_FB_COL];
    unsigned int* left_Calib = new unsigned int[CALIB_LR_ROW*CALIB_LR_COL];
    if(Camera_Num_Flag == 0){
	COL_OFFSET =0;
	ROW_OFFSET =0;
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Front.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(front_Calib, sizeof(unsigned int), CALIB_FB_ROW*CALIB_FB_COL, fp);
	fclose(fp);

        fp = fopen("/data/opengl_new/Lut_Calib/Lut_Left.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(left_Calib, sizeof(unsigned int), CALIB_LR_ROW*CALIB_LR_COL, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 1){
	COL_OFFSET =CALIB_FB_COL-CALIB_LR_COL;
	ROW_OFFSET =0;
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Front.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(front_Calib, sizeof(unsigned int), CALIB_FB_ROW*CALIB_FB_COL, fp);
	fclose(fp);

        fp = fopen("/data/opengl_new/Lut_Calib/Lut_Right.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(left_Calib, sizeof(unsigned int), CALIB_LR_ROW*CALIB_LR_COL, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 2){
	COL_OFFSET =0;
	ROW_OFFSET =CALIB_LR_ROW-CALIB_FB_ROW;
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Back.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(front_Calib, sizeof(unsigned int), CALIB_FB_ROW*CALIB_FB_COL, fp);
	fclose(fp);

        fp = fopen("/data/opengl_new/Lut_Calib/Lut_Left.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(left_Calib, sizeof(unsigned int), CALIB_LR_ROW*CALIB_LR_COL, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 3){
	COL_OFFSET =CALIB_FB_COL-CALIB_LR_COL;
	ROW_OFFSET =CALIB_LR_ROW-CALIB_FB_ROW;
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Back.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(front_Calib, sizeof(unsigned int), CALIB_FB_ROW*CALIB_FB_COL, fp);
	fclose(fp);

        fp = fopen("/data/opengl_new/Lut_Calib/Lut_Right.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(left_Calib, sizeof(unsigned int), CALIB_LR_ROW*CALIB_LR_COL, fp);
	fclose(fp);
    }
	//printf("COL_OFFSET,ROW_OFFSET::%d\t,%d\n",COL_OFFSET,ROW_OFFSET);
 	//printf("START_X,START_Y::%d\t,%d\n",COL_OFFSET,ROW_OFFSET);
   	/****************/
	for(int i=0;i<MERGE_FRONT_LEFT_ROW;i++)
	{
	        temp_y1 = START_Y-1.0*i*HEIGHT/MERGE_FRONT_LEFT_ROW;
        for(int j=0;j<MERGE_FRONT_LEFT_COL;j++)
        {
            temp_x1 =START_X+1.0*j*WIDTH/MERGE_FRONT_LEFT_COL;

            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+0] = temp_x1;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+1] = temp_y1;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+2] = 0;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+3] = temp_x1+X_JUMP;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+4] = temp_y1;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+5] = 0;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+6] = temp_x1+X_JUMP;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+7] = temp_y1-Y_JUMP;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+8] = 0;                  //FIRST TRIANGLE

            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+9] =  temp_x1;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+10] = temp_y1-Y_JUMP;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+11] = 0;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+12] = temp_x1+X_JUMP;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+13] = temp_y1-Y_JUMP;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+14] = 0;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+15] = temp_x1;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+16] = temp_y1;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL+j)*18+17] = 0;                 //SECOND TRIANGLE
                              //UPDATE X
        }//UPDATE X&Y
	}
    //X_COUNT = int(480.0/FRONT_BACK_ROW);//CALIB_FB_ROW
    X_COUNT = 1.0*CALIB_FL_ROW/MERGE_FRONT_LEFT_ROW;//CALIB_FB_ROW
    Y_COUNT = 1.0*CALIB_FL_COL/MERGE_FRONT_LEFT_COL;
    //printf("X_COUNT1 Y_COUNT1%f\t%f\n",X_COUNT,Y_COUNT);
    //for(int i=0;i<MERGE_FRONT_LEFT_ROW;i++)
    //for(int i=0;i<MERGE_FRONT_LEFT_ROW;i++)
    for(int i=0;i<MERGE_FRONT_LEFT_ROW;i++)
	{
        for(int j=0;j<MERGE_FRONT_LEFT_COL;j++)
        {
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL+j)*12+0] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*Y_COUNT)+COL_OFFSET]>>10)&0x7ff)/1280.0;
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL+j)*12+1] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*Y_COUNT)+COL_OFFSET]>>21)&0x7ff)/720.0;

            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL+j)*12+2] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET]>>10)&0x7ff)/1280.0;
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL+j)*12+3] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET]>>21)&0x7ff)/720.0;

            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL+j)*12+4] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET]>>10)&0x7ff)/1280.0;
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL+j)*12+5] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET]>>21)&0x7ff)/720.0;

            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL+j)*12+6] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*Y_COUNT)+COL_OFFSET]>>10)&0x7ff)/1280.0;
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL+j)*12+7] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*Y_COUNT)+COL_OFFSET]>>21)&0x7ff)/720.0;

            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL+j)*12+8] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET]>>10)&0x7ff)/1280.0;
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL+j)*12+9] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET]>>21)&0x7ff)/720.0;

            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL+j)*12+10] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*Y_COUNT+COL_OFFSET)]>>10)&0x7ff)/1280.0;
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL+j)*12+11] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*Y_COUNT+COL_OFFSET)]>>21)&0x7ff)/720.0;

        }//UPDATE X&Y
	}
    X_COUNT = 1.0*CALIB_FL_ROW/MERGE_FRONT_LEFT_ROW;//CALIB_FB_ROW
    Y_COUNT = 1.0*CALIB_FL_COL/MERGE_FRONT_LEFT_COL;
    //printf("X_COUNT2 Y_COUNT2%f\t%f\n",X_COUNT,Y_COUNT);
    //for(int i=0;i<MERGE_FRONT_LEFT_ROW;i++)
    //for(int i=0;i<MERGE_FRONT_LEFT_ROW;i++)
    for(int i=0;i<MERGE_FRONT_LEFT_ROW;i++)
	{
        for(int j=0;j<MERGE_FRONT_LEFT_COL;j++)
        {
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL+j)*12+0] = ((left_Calib[int(i*X_COUNT+ROW_OFFSET)*CALIB_LR_COL+int(j*Y_COUNT)]>>10)&0x7ff)/1280.0;
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL+j)*12+1] = ((left_Calib[int(i*X_COUNT+ROW_OFFSET)*CALIB_LR_COL+int(j*Y_COUNT)]>>21)&0x7ff)/720.0;

            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL+j)*12+2] = ((left_Calib[int(i*X_COUNT+ROW_OFFSET)*CALIB_LR_COL+int(j*Y_COUNT+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL+j)*12+3] = ((left_Calib[int(i*X_COUNT+ROW_OFFSET)*CALIB_LR_COL+int(j*Y_COUNT+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL+j)*12+4] = ((left_Calib[int(i*X_COUNT+X_COUNT-1+ROW_OFFSET)*CALIB_LR_COL+int(j*Y_COUNT+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL+j)*12+5] = ((left_Calib[int(i*X_COUNT+X_COUNT-1+ROW_OFFSET)*CALIB_LR_COL+int(j*Y_COUNT+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL+j)*12+6] = ((left_Calib[int(i*X_COUNT+X_COUNT-1+ROW_OFFSET)*CALIB_LR_COL+int(j*Y_COUNT)]>>10)&0x7ff)/1280.0;
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL+j)*12+7] = ((left_Calib[int(i*X_COUNT+X_COUNT-1+ROW_OFFSET)*CALIB_LR_COL+int(j*Y_COUNT)]>>21)&0x7ff)/720.0;

            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL+j)*12+8] = ((left_Calib[int(i*X_COUNT+X_COUNT-1+ROW_OFFSET)*CALIB_LR_COL+int(j*Y_COUNT+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL+j)*12+9] = ((left_Calib[int(i*X_COUNT+X_COUNT-1+ROW_OFFSET)*CALIB_LR_COL+int(j*Y_COUNT+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL+j)*12+10] = ((left_Calib[int(i*X_COUNT+ROW_OFFSET)*CALIB_LR_COL+int(j*Y_COUNT)]>>10)&0x7ff)/1280.0;
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL+j)*12+11] = ((left_Calib[int(i*X_COUNT+ROW_OFFSET)*CALIB_LR_COL+int(j*Y_COUNT)]>>21)&0x7ff)/720.0;
        }//UPDATE X&Y
	}

    X_COUNT = 1.0*CALIB_FL_ROW/MERGE_FRONT_LEFT_ROW;//CALIB_FB_ROW
    Y_COUNT = 1.0*CALIB_FL_COL/MERGE_FRONT_LEFT_COL;
    //printf("X_COUNT3 Y_COUNT3%f\t%f\n",X_COUNT,Y_COUNT);
    for(int i=0;i<MERGE_FRONT_LEFT_ROW;i++)
	{
        for(int j=0;j<MERGE_FRONT_LEFT_COL;j++)
        {//weight_merge = ((front_Calib[i*720+j])&0x3ff)/1023.0;
            Vertex_Weight[(i*MERGE_FRONT_LEFT_COL+j)*6+0] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*Y_COUNT)+COL_OFFSET])&0x3ff)/1023.0;

            Vertex_Weight[(i*MERGE_FRONT_LEFT_COL+j)*6+1] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET])&0x3ff)/1023.0;

            Vertex_Weight[(i*MERGE_FRONT_LEFT_COL+j)*6+2] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET])&0x3ff)/1023.0;

            Vertex_Weight[(i*MERGE_FRONT_LEFT_COL+j)*6+3] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*Y_COUNT)+COL_OFFSET])&0x3ff)/1023.0;

            Vertex_Weight[(i*MERGE_FRONT_LEFT_COL+j)*6+4] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET])&0x3ff)/1023.0;

            Vertex_Weight[(i*MERGE_FRONT_LEFT_COL+j)*6+5] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL+int(j*Y_COUNT)+COL_OFFSET])&0x3ff)/1023.0;
        }//UPDATE X&Y
	}
	for(int i=0;i<MERGE_FRONT_LEFT_COL*MERGE_FRONT_LEFT_ROW*6;i++)
	{
        Vertex_index[i] = i;
    }
	delete[] front_Calib;
	delete[] left_Calib;
}

void Generate_Car_Icon_Background()
{
	//drawFilledRect(carIconRect.x, carIconRect.y, carIconRect.width, carIconRect.height, viewWidth, viewHeight, bevTransformMatrix);
	static GLuint positionVBO = 0;
	static GLuint colorVBO = 0;
	static GLuint indexVBO = 0;

	static GLuint program = 0;
	static GLint positionAttr = 0;
	static GLint colorAttr = 0;
	static GLint transformMatrixUnif = 0;
	static bool isProgramLoad = false;

	if (!isProgramLoad)
	{
		if (loadShaders("/data/opengl_new/shader/line.vert", "/data/opengl_new/shader/line.frag", program) != 0)
		{
			throw std::runtime_error("couldn't load line.vert/line.frag");
		}
		positionAttr = glGetAttribLocation(program, "aPosition");
		colorAttr = glGetAttribLocation(program, "aColor");
		transformMatrixUnif = glGetUniformLocation(program, "uTransformMatrix");

		float rectX = carIconRect.x / (bevWidth / 2.0) - 1.0;
		float rectY = 1.0 - carIconRect.y / (bevHeight / 2.0);
		float rectWidth = 2.0 * carIconRect.width / bevWidth;
		float rectHeight = 2.0 * (carIconRect.height / bevHeight);

		std::vector<GLfloat> positions = {
			rectX, rectY,
			rectX, rectY - rectHeight,
			rectX + rectWidth, rectY - rectHeight,
			rectX + rectWidth, rectY
		};

		std::vector<GLfloat> color = {
			0.0, 0.0, 0.0, 1.0,
			0.0, 0.0, 0.0, 1.0,
			0.0, 0.0, 0.0, 1.0,
			0.0, 0.0, 0.0, 1.0
		};

		std::vector<GLushort> indices = {
			0, 1, 2, 0, 2, 3
		};

		glGenBuffers(1, &positionVBO);
		glGenBuffers(1, &colorVBO);
		glGenBuffers(1, &indexVBO);
		glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * positions.size(), positions.data(), GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, colorVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * color.size(), color.data(), GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLushort) * indices.size(), indices.data(), GL_STATIC_DRAW);
		isProgramLoad = true;
	}

	glUseProgram(program);
	glUniformMatrix4fv(transformMatrixUnif, 1, GL_FALSE, glm::value_ptr(bevTransformMatrix));

	glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
	glVertexAttribPointer(positionAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

	glBindBuffer(GL_ARRAY_BUFFER, colorVBO);
	glVertexAttribPointer(colorAttr, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO);

	glEnableVertexAttribArray(positionAttr);
	glEnableVertexAttribArray(colorAttr);

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0);

	glDisableVertexAttribArray(colorAttr);
	glDisableVertexAttribArray(positionAttr);
}

void Generate_Car_Icon(const glm::mat4& transform)
{
	static GLuint program;
	static GLint positionAttr;
	static GLint texCoordAttr;
	static GLint transformMatrixUnif;
	static GLint samplerUnif;

	static GLuint positionVBO;
	static GLuint texCoordVBO;
	static GLuint indexVBO;
	static GLuint tex;

	static cv::Mat icon;

	static bool isProgramLoad = false;
	if (!isProgramLoad)
	{
		if (loadShaders("/data/opengl_new/shader/image.vert", "/data/opengl_new/shader/image.frag", program) != 0)
		{
			throw std::runtime_error("");
		}

		positionAttr = glGetAttribLocation(program, "aPosition");
		texCoordAttr = glGetAttribLocation(program, "aTexCoord");
		samplerUnif = glGetUniformLocation(program, "uTexture");
		transformMatrixUnif = glGetUniformLocation(program, "uTransformMatrix");

		float widthScale = carIconRect.width  / 340.0;
		float heightScale = carIconRect.height / 840.0;

		float rectX = (carIconRect.x - widthScale * 40.0)  / (bevWidth / 2.0) - 1.0;
		float rectY = 1.0 - carIconRect.y / (bevHeight / 2.0);
		float rectWidth = 2.0 * (carIconRect.width + widthScale * 40.0 * 2) / bevWidth;
		float rectHeight = 2.0 * carIconRect.height / bevHeight;

		std::vector<GLfloat> positions = {
			rectX, rectY,
			rectX, rectY - rectHeight,
			rectX + rectWidth, rectY - rectHeight,
			rectX + rectWidth, rectY
		};

//		std::vector<GLfloat> positions = {
//			-1.0f, 1.0f,
//			-1.0f, -1.0f,
//			1.0f, -1.0f,
//			1.0f,  1.0f,
//		};

		std::vector<GLfloat> texCoords = {
			0.0f,  0.0f,        // TexCoord 0
			0.0f,  1.0f,        // TexCoord 1
			1.0f,  1.0f,        // TexCoord 2
			1.0f,  0.0f         // TexCoord 3
		};

		std::vector<GLushort> indices =
			{ 0, 1, 2, 0, 2, 3 };

		cv::Mat bgra = cv::imread("/data/opengl_new/car_icon.png", cv::IMREAD_UNCHANGED);
		cv::cvtColor(bgra, icon, cv::COLOR_BGRA2RGBA);
		//icon = bgra.clone();

//		cv::imshow("icon", icon);
//		cv::waitKey(0);

		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glGenTextures(1, &tex);
		glBindTexture(GL_TEXTURE_2D, tex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, icon.size().width, icon.size().height, 0, GL_RGBA, GL_UNSIGNED_BYTE, icon.data);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		glGenBuffers(1, &positionVBO);
		glGenBuffers(1, &texCoordVBO);
		glGenBuffers(1, &indexVBO);

		glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * positions.size(), positions.data(), GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * texCoords.size(), texCoords.data(), GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLushort) * indices.size(), indices.data(), GL_STATIC_DRAW);

		isProgramLoad = true;
	}

	glUseProgram(program);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, tex);
	glUniform1i(samplerUnif, 0);

	glUniformMatrix4fv(transformMatrixUnif, 1, GL_FALSE, glm::value_ptr(transform));

	glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
	glVertexAttribPointer(positionAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
	glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO);
	glVertexAttribPointer(texCoordAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO);

	glEnableVertexAttribArray(positionAttr);
	glEnableVertexAttribArray(texCoordAttr);

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0);

	glDisableVertexAttribArray(positionAttr);
	glDisableVertexAttribArray(texCoordAttr);

}


void Generate_Image_View(GLuint tex, int x, int y, int width, int height, const glm::mat4& transformMatrix)
{

	int oldViewport[4] = {0};
	glGetIntegerv(GL_VIEWPORT, oldViewport);

	glViewport(x, y, width, height);

	static GLuint program;
	static GLint positionAttr;
	static GLint texCoordAttr;
	static GLint transformMatrixUnif;
	static GLint samplerUnif;

	static GLuint positionVBO;
	static GLuint texCoordVBO;
	static GLuint indexVBO;

	static bool isProgramLoad = false;
	if (!isProgramLoad)
	{
		if (loadShaders("/data/opengl_new/shader/image.vert", "/data/opengl_new/shader/image.frag", program) != 0)
		{
			throw std::runtime_error("could not load image.vert/image.frag");
		}

		positionAttr = glGetAttribLocation(program, "aPosition");
		texCoordAttr = glGetAttribLocation(program, "aTexCoord");
		samplerUnif = glGetUniformLocation(program, "uTexture");
		transformMatrixUnif = glGetUniformLocation(program, "uTransformMatrix");

		std::vector<GLfloat> positions = {
			-1.0f, 1.0f,	// Position 0
			-1.0f, -1.0f,  // Position 1
			1.0f, -1.0f,	// Position 2
			1.0f,  1.0f,	// Position 3
		};
		
		std::vector<GLfloat> texCoords = {
			0.0f,  0.0f,        // TexCoord 0
			0.0f,  1.0f,        // TexCoord 1
			1.0f,  1.0f,        // TexCoord 2
			1.0f,  0.0f         // TexCoord 3
		};

		std::vector<GLushort> indices =
			{ 0, 1, 2, 0, 2, 3 };


		glGenBuffers(1, &positionVBO);
		glGenBuffers(1, &texCoordVBO);
		glGenBuffers(1, &indexVBO);

		glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * positions.size(), positions.
data(), GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * texCoords.size(), texCoords.
data(), GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLfloat) * indices.size(),
indices.data(), GL_STATIC_DRAW);

		isProgramLoad = true;
	}


	glUseProgram(program);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, tex);
	glUniform1i(samplerUnif, 0);

	glUniformMatrix4fv(transformMatrixUnif, 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0)));

	glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
	glVertexAttribPointer(positionAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
	glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO);
	glVertexAttribPointer(texCoordAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO);

	glEnableVertexAttribArray(positionAttr);
	glEnableVertexAttribArray(texCoordAttr);

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0);

	glDisableVertexAttribArray(positionAttr);
	glDisableVertexAttribArray(texCoordAttr);

	glViewport(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]);
}


std::map<std::string, GLuint> textureTable;
void Generate_Image_View(const char* filename, int x, int y, int width, int height)
{
	std::string fn(filename);
	if (textureTable.find(fn) == textureTable.end())
	{
		cv::Mat icon;
		cv::Mat bgra = cv::imread(filename, cv::IMREAD_COLOR);
		cv::cvtColor(bgra, icon, cv::COLOR_BGR2RGB);

		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		GLuint tex;
		glGenTextures(1, &tex);
		glBindTexture(GL_TEXTURE_2D, tex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, icon.size().width, icon.size().height, 0, GL_RGB, GL_UNSIGNED_BYTE, icon.data);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		textureTable[fn] = tex;
	}
	Generate_Image_View(textureTable[fn], x, y, width, height);
}



void Generate_SingleView(GLuint Camera_Num_Flag,GLuint *textureID, float* delta)
{
    static GLuint m_shaderProgram=0;
    static GLuint m_positionHandle=0;
    static GLuint m_colorHandle=0;
    static GLuint m_textCoordHandle=0;
    static GLuint m_samplerHandle=0;
	static GLuint m_deltaHandle=0;
    static GLuint count_num =0;
    static bool Flag_cal[4] ={false,false,false,false};

    static GLfloat VertexPositions_front[SINGLE_VIEW_COL*SINGLE_VIEW_ROW*6*3] ={0.0,1.0,0.0,
										1.0,1.0,0.0,
										1.0,0.0,0.0,
										0.0,0.0,0.0,
										1.0,0.0,0.0,
										0.0,1.0,0.0};
    static GLfloat VertexColors_front[SINGLE_VIEW_COL*SINGLE_VIEW_ROW*6*2] ={0.0,0.0,
										1.0,0.0,
										1.0,1.0,
										0.0,1.0,
										1.0,1.0,
										0.0,0.0};
    static GLfloat VertexPositions_back[SINGLE_VIEW_COL*SINGLE_VIEW_ROW*6*3] ={0.0};
    static GLfloat VertexColors_back[SINGLE_VIEW_COL*SINGLE_VIEW_ROW*6*2] ={0.0};
    static GLfloat VertexPositions_left[SINGLE_VIEW_COL*SINGLE_VIEW_ROW*6*3] ={0.0};
    static GLfloat VertexColors_left[SINGLE_VIEW_COL*SINGLE_VIEW_ROW*6*2] ={0.0};
    static GLfloat VertexPositions_right[SINGLE_VIEW_COL*SINGLE_VIEW_ROW*6*3] ={0.0};
    static GLfloat VertexColors_right[SINGLE_VIEW_COL*SINGLE_VIEW_ROW*6*2] ={0.0};
    static unsigned short indexArray[SINGLE_VIEW_COL*SINGLE_VIEW_ROW*6] = {0,1,2,3,4,5};
    if(!Flag_cal[Camera_Num_Flag]){
        if(loadShaders("/data/opengl_new/shader/single_view.vert","/data/opengl_new/shader/single_view.frag",m_shaderProgram) != 0)
            printf("return error11111!!!\n");
        m_positionHandle = glGetAttribLocation(m_shaderProgram, "aPosition");
        //m_colorHandle = glGetAttribLocation(m_shaderProgram, "a_Color");
        m_textCoordHandle = glGetAttribLocation(m_shaderProgram, "aTexCoord");
        m_samplerHandle = glGetUniformLocation(m_shaderProgram,"my_Sampler");
		m_deltaHandle = glGetUniformLocation(m_shaderProgram, "uDelta");
    }
    if(Camera_Num_Flag == 0 && Flag_cal[Camera_Num_Flag] ==false){
	Calib_2D_Single_View(Camera_Num_Flag,VertexPositions_front,VertexColors_front,indexArray,-1.0,1.0,2.0*singleViewWidth/singleViewWidth,2.0*singleViewHeight/singleViewHeight);
	int  a =8;
    }
    else if(Camera_Num_Flag == 1 && Flag_cal[Camera_Num_Flag] ==false){
	Calib_2D_Single_View(Camera_Num_Flag,VertexPositions_back,VertexColors_back,indexArray,-1.0,1.0,2.0*singleViewWidth/singleViewWidth,2.0*singleViewHeight/singleViewHeight);
    }
    else if(Camera_Num_Flag == 2 && Flag_cal[Camera_Num_Flag] ==false){
	Calib_2D_Single_View(Camera_Num_Flag,VertexPositions_left,VertexColors_left,indexArray,-1.0,1.0,2.0*singleViewWidth/singleViewWidth,2.0*singleViewHeight/singleViewHeight);
    }
    else if(Camera_Num_Flag == 3 && Flag_cal[Camera_Num_Flag] ==false){
	Calib_2D_Single_View(Camera_Num_Flag,VertexPositions_right,VertexColors_right,indexArray,-1.0,1.0,2.0*singleViewWidth/singleViewWidth,2.0*singleViewHeight/singleViewHeight);
    }
    //printf("count_num,VertexPositions::\t%d\t%f,\t%f\t%f\n",count_num,VertexPositions_front[3*count_num],VertexPositions_front[3*count_num+1],VertexPositions_front[3*count_num+2]);
    //printf("count_num,Vertex_tex1::\t\t%d\t%f,\t%f\n",count_num,VertexColors_front[2*count_num],VertexColors_front[2*count_num+1]);
    count_num++;
    if(count_num == SINGLE_VIEW_COL*SINGLE_VIEW_ROW*6)
        count_num = 0;
    static GLuint m_vbo_front1[3] = {0};
    static GLuint m_vbo_back1[3] = {0};
    static GLuint m_vbo_left1[3] = {0};
    static GLuint m_vbo_right1[3] = {0};
    if(m_vbo_front1[0]==0 && m_vbo_back1[0]==0)
    {
        glGenBuffers(3, m_vbo_front1);
        glGenBuffers(3, m_vbo_back1);
        glGenBuffers(3, m_vbo_left1);
        glGenBuffers(3, m_vbo_right1);
    }
        if(Camera_Num_Flag == 0 && Flag_cal[Camera_Num_Flag] ==false)
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_front1[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*SINGLE_VIEW_COL*SINGLE_VIEW_ROW*3,VertexPositions_front, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_front1[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*SINGLE_VIEW_COL*SINGLE_VIEW_ROW*2,VertexColors_front, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_front1[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*6*SINGLE_VIEW_COL*SINGLE_VIEW_ROW, indexArray, GL_STATIC_DRAW);
        }
        else if(Camera_Num_Flag == 1 && Flag_cal[Camera_Num_Flag] ==false)
        {
	    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_back1[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*SINGLE_VIEW_COL*SINGLE_VIEW_ROW*3,VertexPositions_back, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_back1[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*SINGLE_VIEW_COL*SINGLE_VIEW_ROW*2,VertexColors_back, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_back1[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*6*SINGLE_VIEW_COL*SINGLE_VIEW_ROW, indexArray, GL_STATIC_DRAW);
        }
        else if(Camera_Num_Flag == 2 && Flag_cal[Camera_Num_Flag] ==false)
        {
	    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_left1[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*SINGLE_VIEW_COL*SINGLE_VIEW_ROW*3,VertexPositions_left, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_left1[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*SINGLE_VIEW_COL*SINGLE_VIEW_ROW*2,VertexColors_left, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_left1[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*6*SINGLE_VIEW_COL*SINGLE_VIEW_ROW, indexArray, GL_STATIC_DRAW);
        }
        else if(Camera_Num_Flag == 3 && Flag_cal[Camera_Num_Flag] ==false)
        {
	    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_right1[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*SINGLE_VIEW_COL*SINGLE_VIEW_ROW*3,VertexPositions_right, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_right1[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*SINGLE_VIEW_COL*SINGLE_VIEW_ROW*2,VertexColors_right, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_right1[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*6*SINGLE_VIEW_COL*SINGLE_VIEW_ROW, indexArray, GL_STATIC_DRAW);
        }
     Flag_cal[Camera_Num_Flag] = true;
    if(Camera_Num_Flag == 0)
    {
    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID[0]);
    glUniform1i(m_samplerHandle, 0);
	glUniform1f(m_deltaHandle, delta[Camera_Num_Flag]);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_front1[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_front1[1]);
        glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_front1[2]);
    }
    else if(Camera_Num_Flag == 1)
    {
    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID[1]);
    glUniform1i(m_samplerHandle, 0);
	glUniform1f(m_deltaHandle, delta[Camera_Num_Flag]);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_back1[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_back1[1]);
        glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_back1[2]);
    }
    else if(Camera_Num_Flag == 2)
    {
    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID[2]);
    glUniform1i(m_samplerHandle, 0);
	glUniform1f(m_deltaHandle, delta[Camera_Num_Flag]);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_left1[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_left1[1]);
        glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_left1[2]);
    }
    else if(Camera_Num_Flag == 3)
    {
    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID[3]);
    glUniform1i(m_samplerHandle, 0);
	glUniform1f(m_deltaHandle, delta[Camera_Num_Flag]);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_right1[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_right1[1]);
        glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_right1[2]);
    }
	//printf("ffgggjjk : \t %d\n",Camera_Num_Flag);
    	glEnableVertexAttribArray(m_positionHandle);
	glEnableVertexAttribArray(m_textCoordHandle);
	glEnableVertexAttribArray(m_samplerHandle);
	//if(Camera_Num_Flag == 0)
		//glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0);
	//else
   	glDrawElements(GL_TRIANGLES, SINGLE_VIEW_COL*SINGLE_VIEW_ROW*6, GL_UNSIGNED_SHORT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_textCoordHandle);
	glDisableVertexAttribArray(m_samplerHandle);
}
void Calib_2D_Single_View(GLuint Camera_Num_Flag,GLfloat *Vertex_pos,GLfloat *Vertex_tex,unsigned short *Vertex_index,GLfloat START_X,GLfloat START_Y,GLfloat WIDTH,GLfloat HEIGHT)
{
   FILE *fp ;
   float temp_x1=START_X,temp_y1=START_Y;
   float X_JUMP = 1.0*WIDTH/SINGLE_VIEW_COL,Y_JUMP = 1.0*HEIGHT/SINGLE_VIEW_ROW;
   float X_COUNT,Y_COUNT;
   unsigned int* Calib = new unsigned int[CALIB_SINGLE_VIEW_ROW*CALIB_SINGLE_VIEW_COL*2];
    if(Camera_Num_Flag == 0){
    	fp = fopen("/data/opengl_new/Lut_Calib/single_view_front.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(Calib, sizeof(unsigned int), CALIB_SINGLE_VIEW_ROW*CALIB_SINGLE_VIEW_COL*2, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 1){
    	fp = fopen("/data/opengl_new/Lut_Calib/single_view_back.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(Calib, sizeof(unsigned int), CALIB_SINGLE_VIEW_ROW*CALIB_SINGLE_VIEW_COL*2, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 2){
    	fp = fopen("/data/opengl_new/Lut_Calib/single_view_left.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(Calib, sizeof(unsigned int), CALIB_SINGLE_VIEW_ROW*CALIB_SINGLE_VIEW_COL*2, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 3){
    	fp = fopen("/data/opengl_new/Lut_Calib/single_view_right.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(Calib, sizeof(unsigned int), CALIB_SINGLE_VIEW_ROW*CALIB_SINGLE_VIEW_COL*2, fp);
	fclose(fp);
    }
/****************/
	for(int i=0;i<SINGLE_VIEW_ROW;i++)
	{
		temp_y1 = START_Y-1.0*i*HEIGHT/SINGLE_VIEW_ROW;
        for(int j=0;j<SINGLE_VIEW_COL;j++)
          {
	    temp_x1 =START_X+1.0*j*WIDTH/SINGLE_VIEW_COL;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+0] = temp_x1;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+1] = temp_y1;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+2] = 0;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+3] = temp_x1+X_JUMP;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+4] = temp_y1;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+5] = 0;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+6] = temp_x1+X_JUMP;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+7] = temp_y1-Y_JUMP;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+8] = 0;                  //FIRST TRIANGLE

            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+9] =  temp_x1;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+10] = temp_y1-Y_JUMP;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+11] = 0;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+12] = temp_x1+X_JUMP;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+13] = temp_y1-Y_JUMP;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+14] = 0;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+15] = temp_x1;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+16] = temp_y1;
            Vertex_pos[(i*SINGLE_VIEW_COL+j)*18+17] = 0;                 //SECOND TRIANGLE
          }
	}
    X_COUNT = 1.0*CALIB_SINGLE_VIEW_ROW/SINGLE_VIEW_ROW;//CALIB_FB_ROW
    Y_COUNT = 1.0*CALIB_SINGLE_VIEW_COL/SINGLE_VIEW_COL;
    //printf("X_COUNT Y_COUNT%f\t%f\n",X_COUNT,Y_COUNT);
    for(int i=0;i<SINGLE_VIEW_ROW;i++)
	{
        for(int j=0;j<SINGLE_VIEW_COL;j++)
        {
            Vertex_tex[(i*SINGLE_VIEW_COL+j)*12+0] = ((Calib[(int(i*X_COUNT)*CALIB_SINGLE_VIEW_COL+int(j*Y_COUNT))*2]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*SINGLE_VIEW_COL+j)*12+1] = ((Calib[(int(i*X_COUNT)*CALIB_SINGLE_VIEW_COL+int(j*Y_COUNT))*2]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*SINGLE_VIEW_COL+j)*12+2] = ((Calib[(int(i*X_COUNT)*CALIB_SINGLE_VIEW_COL+int(j*Y_COUNT+Y_COUNT-1))*2]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*SINGLE_VIEW_COL+j)*12+3] = ((Calib[(int(i*X_COUNT)*CALIB_SINGLE_VIEW_COL+int(j*Y_COUNT+Y_COUNT-1))*2]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*SINGLE_VIEW_COL+j)*12+4] = ((Calib[(int(i*X_COUNT+X_COUNT-1)*CALIB_SINGLE_VIEW_COL+int(j*Y_COUNT+Y_COUNT-1))*2]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*SINGLE_VIEW_COL+j)*12+5] = ((Calib[(int(i*X_COUNT+X_COUNT-1)*CALIB_SINGLE_VIEW_COL+int(j*Y_COUNT+Y_COUNT-1))*2]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*SINGLE_VIEW_COL+j)*12+6] = ((Calib[(int(i*X_COUNT+X_COUNT-1)*CALIB_SINGLE_VIEW_COL+int(j*Y_COUNT))*2]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*SINGLE_VIEW_COL+j)*12+7] = ((Calib[(int(i*X_COUNT+X_COUNT-1)*CALIB_SINGLE_VIEW_COL+int(j*Y_COUNT))*2]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*SINGLE_VIEW_COL+j)*12+8] = ((Calib[(int(i*X_COUNT+X_COUNT-1)*CALIB_SINGLE_VIEW_COL+int(j*Y_COUNT+Y_COUNT-1))*2]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*SINGLE_VIEW_COL+j)*12+9] = ((Calib[(int(i*X_COUNT+X_COUNT-1)*CALIB_SINGLE_VIEW_COL+int(j*Y_COUNT+Y_COUNT-1))*2]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*SINGLE_VIEW_COL+j)*12+10] = ((Calib[(int(i*X_COUNT)*CALIB_SINGLE_VIEW_COL+int(j*Y_COUNT))*2]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*SINGLE_VIEW_COL+j)*12+11] = ((Calib[(int(i*X_COUNT)*CALIB_SINGLE_VIEW_COL+int(j*Y_COUNT))*2]>>21)&0x7ff)/720.0;
        }
	}
	for(int i=0;i<SINGLE_VIEW_COL*SINGLE_VIEW_ROW*6;i++)
	{
        	Vertex_index[i] = i;
	}
	delete[] Calib;
}

void Generate_Radar_Chart(GLuint Flag_Next,GLuint Num_Flag)
{
    static GLuint m_shaderProgram=0;
    static GLuint m_positionHandle=0;
    static GLuint m_colorHandle=0;
    static GLuint count_num =0;
    static bool Flag_cal=false;
    static unsigned short IndexArray[100000] = {0,1,2,3,4,5};
    static int Line_Num[2] = {0};
    static int Line_Num_static[2] = {0};
    static GLfloat Ver_Pos0[1000*3] = {0.0};
    static GLfloat Ver_Pos1[1000*3] = {0.0};
    static GLfloat Ver_Color0[1000*3] = {0.0};
    static GLfloat Ver_Color1[1000*3] = {0.0};
    static GLfloat Ver_Pos0_Static[100000*3] = {0.0};
    static GLfloat Ver_Color0_Static[100000*3] = {0.0};
    static GLfloat Ver_Pos1_Static[100000*3] = {0.0};
    static GLfloat Ver_Color1_Static[100000*3] = {0.0};

    static vector<Point2f> points;
    static vector<Point2f> points_fitted;
    Mat A;

    if(!Flag_cal)
    {
	if(loadShaders("/data/opengl_new/shader/radar_chart.vert","/data/opengl_new/shader/radar_chart.frag",m_shaderProgram) != 0)
		printf("return error : Generate_Radar_Chart()!!!\n");
        m_positionHandle = glGetAttribLocation(m_shaderProgram, "aPosition");
        m_colorHandle = glGetAttribLocation(m_shaderProgram, "a_Color");
    }
	Calib_2D_Single_Line(Flag_Next,Num_Flag,Line_Num,Ver_Pos0,Ver_Color0,IndexArray);
		Calib_2D_Single_Static_Line(0,Num_Flag,Line_Num_static,Ver_Pos0_Static,Ver_Color0_Static,IndexArray);
		Calib_2D_Single_Static_Line(1,Num_Flag,Line_Num_static,Ver_Pos1_Static,Ver_Color1_Static,IndexArray);
	for(int i = 0;i<Line_Num[0];i++)
	{
		Ver_Pos1[3*i+0] = Ver_Pos0[3*(i+Line_Num[1])+0];
		Ver_Pos1[3*i+1] = Ver_Pos0[3*(i+Line_Num[1])+1];
		Ver_Pos1[3*i+2] = Ver_Pos0[3*(i+Line_Num[1])+2];
		Ver_Color1[3*i+0] = Ver_Color0[3*(i+Line_Num[1])+0];
		Ver_Color1[3*i+1] = Ver_Color0[3*(i+Line_Num[1])+1];
		Ver_Color1[3*i+2] = Ver_Color0[3*(i+Line_Num[1])+2];
	}
	for(int i = 0;i<Line_Num[0];i++){
		points.push_back(Point2f(Ver_Pos1[3*i+0],Ver_Pos1[3*i+1]));
		//printf("Ver_Pos1[%d] = %f\t%f\n",i,Ver_Pos1[3*i+0],Ver_Pos1[3*i+1]);
	}
	polynomial_curve_fit(points, points_fitted,1, A);
	for(int i = 0;i<points_fitted.size();i++){
		//Ver_Pos1[3*i+0] = points_fitted[i].x;
		//Ver_Pos1[3*i+1] = points_fitted[i].y;
		//printf("points_fitted[%d].xy = %f %f\n",i,points_fitted[i].x,points_fitted[i].y);
	}
	//Line_Num[0] = points_fitted.size();
	points.clear();
	points_fitted.clear();

	//printf("size of point = %d\n",points.size());
	//printf("size of points_fitted = %d\n",points_fitted.size());
	//printf("Ver_Pos[%d] : %f\t %f\t %f\n",count_num,Ver_Pos0[3*count_num+0],Ver_Pos0[3*count_num+1],Ver_Pos0[3*count_num+2]);
	//printf("Line_Num_static[0]:%d\tLine_Num_static[1]:%d\n",Line_Num_static[0],Line_Num_static[1]);
    static GLuint m_vbo0[3] = {0,0,0};
    static GLuint m_vbo1[3] = {0,0,0};
    static GLuint m_vbo_front_static[3] = {0,0,0};
    static GLuint m_vbo_back_static[3] = {0,0,0};
    if(!Flag_cal)
        {
		glGenBuffers(3, m_vbo0);
		glGenBuffers(3, m_vbo1);
		glGenBuffers(3, m_vbo_front_static);
		glGenBuffers(3, m_vbo_back_static);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_front_static[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*Line_Num_static[0]*3,Ver_Pos0_Static, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_front_static[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*Line_Num_static[0]*3,Ver_Color0_Static, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_front_static[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*Line_Num_static[0], IndexArray, GL_STATIC_DRAW);

            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_back_static[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*Line_Num_static[1]*3,Ver_Pos1_Static, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_back_static[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*Line_Num_static[1]*3,Ver_Color1_Static, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_back_static[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*Line_Num_static[1], IndexArray, GL_STATIC_DRAW);
	}
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo0[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*Line_Num[1]*3,&(Ver_Pos0[0]), GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo0[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*Line_Num[1]*3,&(Ver_Color0[0]), GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo0[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*Line_Num[1], IndexArray, GL_STATIC_DRAW);

            glBindBuffer(GL_ARRAY_BUFFER, m_vbo1[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*Line_Num[0]*3,Ver_Pos1, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo1[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*Line_Num[0]*3,Ver_Color1, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo1[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*Line_Num[0], IndexArray, GL_STATIC_DRAW);

    Flag_cal=true;
    if(Flag_Next == 0){
    glUseProgram(m_shaderProgram);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_front_static[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_front_static[1]);
        glVertexAttribPointer(m_colorHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_front_static[2]);
    	glEnableVertexAttribArray(m_positionHandle);
    	glEnableVertexAttribArray(m_colorHandle);
	glDrawElements(GL_POINTS, Line_Num_static[0], GL_UNSIGNED_SHORT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_colorHandle);
    }
    else if(Flag_Next == 1){
    glUseProgram(m_shaderProgram);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_back_static[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_back_static[1]);
        glVertexAttribPointer(m_colorHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_back_static[2]);
    	glEnableVertexAttribArray(m_positionHandle);
    	glEnableVertexAttribArray(m_colorHandle);
	glDrawElements(GL_POINTS, Line_Num_static[1], GL_UNSIGNED_SHORT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_colorHandle);
    }

    glUseProgram(m_shaderProgram);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo0[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo0[1]);
        glVertexAttribPointer(m_colorHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo0[2]);
    	glLineWidth(20);
    	glEnableVertexAttribArray(m_positionHandle);
    	glEnableVertexAttribArray(m_colorHandle);
	glDrawElements(GL_LINE_STRIP, Line_Num[1], GL_UNSIGNED_SHORT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_colorHandle);

    glUseProgram(m_shaderProgram);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo1[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo1[1]);
        glVertexAttribPointer(m_colorHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo1[2]);
    	glEnableVertexAttribArray(m_positionHandle);
    	glEnableVertexAttribArray(m_colorHandle);
	glDrawElements(GL_LINE_STRIP, Line_Num[0], GL_UNSIGNED_SHORT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_colorHandle);

}
void Calib_2D_Single_Line(GLint Flag_Next,GLint Num_Line,GLint *Line_Num,GLfloat *Vertex_pos,GLfloat *Vertex_color,unsigned short *Vertex_index)
{
   	FILE *fp;
	static unsigned int Calib0[1000*63]={0};
	static unsigned int Calib1[1000*63]={0};
	int NUMBER =150;
	static bool Flag_Line = false;
	if(!Flag_Line){
    		fp = fopen("/data/opengl_new/Lut_Calib/single_view_front_dynamic_line.bin", "rb");
		fseek(fp,0,SEEK_SET);
		fread(Calib0, sizeof(unsigned int), NUMBER*63, fp);
		fclose(fp);

    		fp = fopen("/data/opengl_new/Lut_Calib/single_view_rear_dynamic_line.bin", "rb");
		fseek(fp,0,SEEK_SET);
		fread(Calib1, sizeof(unsigned int), 150*63, fp);
		fclose(fp);

		Flag_Line = true;
	}
	int k_flag =NUMBER*Num_Line;
	//printf("flag_next::%d\n",Flag_Next);
    if(Flag_Next == 0){
	Line_Num[1] = Calib0[NUMBER-1+k_flag];//youxian
	Line_Num[0] = Calib0[0+k_flag] - Calib0[NUMBER-1+k_flag];//zuoxian

	//printf("111Line_Num1 %d,Line_Num2 %d\n",Line_Num[0],Line_Num[1]);
	for(int i = 0;i<NUMBER-2;i++)
	{
//		Vertex_pos[3*i+0] = 1.0 - (CALIB_SINGLE_VIEW_COL-((Calib0[i+1+k_flag]>>10) & 0x7ff)) * 2 /singleViewWidth;
//		Vertex_pos[3*i+1] = (1.0 - ((Calib0[i+1+k_flag]>>21) & 0x7ff) * 2 / singleViewHeight);
		Vertex_pos[3*i+0] = 1.0 - ((Calib0[i+1+k_flag]>>10) & 0x7ff) / singleViewWidth;
		Vertex_pos[3*i+1] = 1.0 - ((Calib0[i+1+k_flag]>>21) & 0x7ff) / singleViewHeight;
		Vertex_pos[3*i+2] = 0.0;
		if((Calib0[i+1+k_flag] & 0x3) == 0){
			Vertex_color[3*i+0] = 1.0;
			Vertex_color[3*i+1] = 1.0;
			Vertex_color[3*i+2] = 0.0;
		}
		else if((Calib0[i+1+k_flag] & 0x3) == 1){
			Vertex_color[3*i+0] = 1.0;
			Vertex_color[3*i+1] = 1.0;
			Vertex_color[3*i+2] = 0.0;
		}
		else if((Calib0[i+1+k_flag] & 0x3) == 2){
			Vertex_color[3*i+0] = 1.0;
			Vertex_color[3*i+1] = 1.0;
			Vertex_color[3*i+2] = 0.0;
		}
	}
    }
    else if(Flag_Next == 1){
	int k_flag =150*Num_Line;
	Line_Num[1] = Calib1[149+k_flag];//youxian
	Line_Num[0] = Calib1[0+k_flag] - Calib1[149+k_flag];//zuoxian

	//printf("Line_Num1 %d,Line_Num2 %d\n",Line_Num[0],Line_Num[1]);
	for(int i = 0;i<148;i++)
	{
//		Vertex_pos[3*i+0] = 1.0-(CALIB_SINGLE_VIEW_COL-((Calib1[i+1+k_flag]>>10) & 0x7ff))*2/singleViewWidth;
//		Vertex_pos[3*i+1] = (1.0 - ((Calib1[i+1+k_flag]>>21) & 0x7ff)*2.0/singleViewHeight);
		Vertex_pos[3*i+0] = 1.0 - ((Calib1[i+1+k_flag]>>10) & 0x7ff) / singleViewWidth;
		Vertex_pos[3*i+1] = 1.0 - ((Calib1[i+1+k_flag]>>21) & 0x7ff) / singleViewHeight;
		Vertex_pos[3*i+2] = 0.0;
		if((Calib1[i+1+k_flag] & 0x3) == 0){
			Vertex_color[3*i+0] = 1.0;
			Vertex_color[3*i+1] = 0.0;
			Vertex_color[3*i+2] = 0.0;
		}
		else if((Calib1[i+1+k_flag] & 0x3) == 1){
			Vertex_color[3*i+0] = 1.0;
			Vertex_color[3*i+1] = 1.0;
			Vertex_color[3*i+2] = 0.0;
		}
		else if((Calib1[i+1+k_flag] & 0x3) == 2){
			Vertex_color[3*i+0] = 0.0;
			Vertex_color[3*i+1] = 1.0;
			Vertex_color[3*i+2] = 0.0;
		}
	}


    }
	for(int i = 0;i<1000;i++)
	{
		Vertex_index[i] = i;
	}
}
void Calib_2D_Single_Static_Line(GLint Flag_Next,GLint Num_Line,GLint *Line_Num,GLfloat *Vertex_pos,GLfloat *Vertex_color,unsigned short *Vertex_index)
{
   	FILE *fp;
	int NUMBER =100000;
	static unsigned int Calib0[100000]={0};
	static unsigned int Calib1[100000]={0};
	static bool Flag_Line = false;
	if(!Flag_Line){
    		fp = fopen("/data/opengl_new/Lut_Calib/single_view_front_static_line.bin", "rb");//single_view_front_static_line.bin
		fseek(fp,0,SEEK_SET);
		fread(Calib0, sizeof(unsigned int), NUMBER, fp);
		fclose(fp);
    		fp = fopen("/data/opengl_new/Lut_Calib/single_view_rear_static_line.bin", "rb");
		fseek(fp,0,SEEK_SET);
		fread(Calib1, sizeof(unsigned int), NUMBER, fp);
		fclose(fp);
	Line_Num[0] = Calib0[0];
	Line_Num[1] = Calib1[0];
		Flag_Line = true;

	}
	//printf("Calib0:data = %x\n",Calib0[1]);
    //printf("Line_Num[0]:%d\n",Line_Num[0]);
    //printf("Line_Num[1]:%d\n",Line_Num[1]);
    if(Flag_Next == 0){
	for(int i = 0;i<Line_Num[0];i++)
	{
//		Vertex_pos[3*i+0] = 1.0-(CALIB_SINGLE_VIEW_COL-((Calib0[i+1]>>10) & 0x7ff))*2/1920.0;
//		Vertex_pos[3*i+1] = (1.0 - ((Calib0[i+1]>>21) & 0x7ff)*2.0/1080.0);

		Vertex_pos[3 * i + 0] = (((float)CALIB_SINGLE_VIEW_COL / 2.0) - (float)((Calib0[i + 1] >> 10) & 0x07ff)) / (float)singleViewWidth * 2.0;
		Vertex_pos[3 * i + 1] = (((float)CALIB_SINGLE_VIEW_ROW / 2.0) - (float)((Calib0[i + 1] >> 21) & 0x07ff)) / (float)singleViewHeight * 2.0;

		Vertex_pos[3*i+2] = 0.0;
		if((Calib0[i+1] & 0x3) == 0){
			Vertex_color[3*i+0] = 1.0;
			Vertex_color[3*i+1] = 0.0;
			Vertex_color[3*i+2] = 0.0;
		}
		else if((Calib0[i+1] & 0x3) == 1){
			Vertex_color[3*i+0] = 1.0;
			Vertex_color[3*i+1] = 1.0;
			Vertex_color[3*i+2] = 0.0;
		}
		else if((Calib0[i+1] & 0x3) == 2){
			Vertex_color[3*i+0] = 0.0;
			Vertex_color[3*i+1] = 1.0;
			Vertex_color[3*i+2] = 0.0;
		}
	}
    }
    else if(Flag_Next == 1){
	for(int i = 0;i<Line_Num[1];i++)
	{
//		Vertex_pos[3*i+0] = 1.0-(CALIB_SINGLE_VIEW_COL-((Calib1[i+1]>>10) & 0x7ff))*2.0/1920.0-0.0;
//		Vertex_pos[3*i+1] = (1.0 - ((Calib1[i+1]>>21) & 0x7ff)*2.0/1080.0);

		Vertex_pos[3 * i + 0] = (((float)CALIB_SINGLE_VIEW_COL / 2.0) - (float)((Calib0[i + 1] >> 10) & 0x07ff)) / (float)singleViewWidth * 2.0;
		Vertex_pos[3 * i + 1] = (((float)CALIB_SINGLE_VIEW_ROW / 2.0) - (float)((Calib0[i + 1] >> 21) & 0x07ff)) / (float)singleViewHeight * 2.0;

		Vertex_pos[3*i+2] = 0.0;
		if((Calib1[i+1] & 0x3) == 0){
			Vertex_color[3*i+0] = 1.0;
			Vertex_color[3*i+1] = 0.0;
			Vertex_color[3*i+2] = 0.0;
		}
		else if((Calib1[i+1] & 0x3) == 1){
			Vertex_color[3*i+0] = 1.0;
			Vertex_color[3*i+1] = 1.0;
			Vertex_color[3*i+2] = 0.0;
		}
		else if((Calib1[i+1] & 0x3) == 2){
			Vertex_color[3*i+0] = 0.0;
			Vertex_color[3*i+1] = 1.0;
			Vertex_color[3*i+2] = 0.0;
		}
	}
    }
	for(int i = 0;i<100000;i++)
	{
		Vertex_index[i] = i;
	}
}
void Avm_ParkingTrail(GLint Flag_Next)
{
    /*****************************************************/
    static GLfloat Circle_Center[3] = {0.0,0.0,0.0};
    static GLfloat Circle_Radius = 0.5;
    static GLfloat Angles[2] = {3.1415926*45/180,3.1415926*135/180};
    static GLfloat Trail_Loc = 0.0;
    static int Num_Total =20;
    static int Num_Parking =10;
    /*****************************************************/

    static GLuint m_shaderProgram=0;
    static GLuint m_positionHandle=0;
    static GLuint m_colorHandle=0;
    static GLuint count_num = 0;


    static bool Flag_cal=false;

    static unsigned short IndexArray_Trail[20] = {0,1,2,3,4,5,6};

    static unsigned short IndexArray_Radar[(16*6+4+12)*6] = {0,1,2,3,4,5,6};//Radar
    static GLfloat Pos_Radar[(16*6+4+12)*5*3] = {0.0};
    static GLfloat Color_Radar[(16*6+4+12)*5*3] = {0.0};
    int Signal_Flag[16] = {6,5,4,3,2,1,6,1,2,3,4,5,6,1,4,6};
    int Signal_Color[16]= {2,1,2,2,2,1,2,1,2,1,2,1,2,1,1,2};
	count_num++;
	if(count_num > 200){
		//count_num = 0;
		//printf("modefy signal\n");
		Signal_Flag[0] = 1;
		Signal_Color[0] = 1;
		//Signal_Flag[3] = 4;
		Signal_Flag[6] = 1;
		Signal_Color[6] = 1;
		//Signal_Flag[7] = 4;
		if(count_num > 400)
			count_num = 0;
	}
	else{
		Signal_Flag[0] = 6;
		Signal_Color[0] = 2;
		Signal_Flag[6] = 6;
		Signal_Color[6] = 2;
	}
    Cal_Radar_Line(Signal_Flag,Signal_Color,Pos_Radar,Color_Radar,IndexArray_Radar);//Radar
    static int Line_Num[2] = {20,20};
    static GLfloat Ver_Pos_Left[20*3] = {
						185.9/960-1.0,1.0-365.0/540,0.0,
						190.9/960-1.0,1.0-320.3/540,0.0,
						206.2/960-1.0,1.0-274.6/540,0.0,
						224.5/960-1.0,1.0-244.7/540,0.0,
						238.7/960-1.0,1.0-228.2/540,0.0
					};
    static GLfloat Ver_Pos_Right[20*3] = {0.0};
    static GLfloat Ver_Color_Left[20*3] = {
						1.0,0.0,0.0,
						0.0,1.0,0.0,
						0.0,0.0,1.0,
						1.0,0.0,0.0,
						0.0,1.0,0.0,
						0.0,0.0,1.0};
    static GLfloat Ver_Color_Right[20*3] = {0.0};

    static unsigned short IndexArray[20] = {0};
    static GLfloat Ver_Pos_Parking[10*4*3] = {
						0.0,1.0,0.0,
						1.0,1.0,0.0,
						1.0,0.0,0.0,
						0.0,0.5,0.0,
						0.0,0.0,0.0,
						0.0,-1.0,0.0,
						1.0,-1.0,0.0,
						1.0,-0.5,0.0};//car location
    static GLfloat Ver_Color_Parking[10*4*3] = {
						1.0,0.0,0.0,
						0.0,1.0,0.0,
						0.0,0.0,1.0,
						1.0,0.0,0.0,
						1.0,0.0,0.0,
						0.0,1.0,0.0,
						0.0,0.0,1.0,
						1.0,0.0,0.0};
    if(!Flag_cal)
    {
	if(loadShaders("/data/opengl_new/shader/radar_chart.vert","/data/opengl_new/shader/radar_chart.frag",m_shaderProgram) != 0)
		printf("return error : Generate_Radar_Chart()!!!\n");
        m_positionHandle = glGetAttribLocation(m_shaderProgram, "aPosition");
        m_colorHandle = glGetAttribLocation(m_shaderProgram, "a_Color");
    }
	Cal_Parking(Ver_Pos_Parking,Ver_Color_Parking,IndexArray);

	//printf("Ver_Pos_Left: %d %f\t %f\t %f\n",count_num,Ver_Pos_Left[3*count_num+0],Ver_Pos_Left[3*count_num+1],Ver_Pos_Left[3*count_num+2]);
	//printf("Color: %f\t %f\t %f\n",Ver_Color_Left[3*count_num+0],Ver_Color_Left[3*count_num+1],Ver_Color_Left[3*count_num+2]);Pos_Radar
	//printf("Pos_Radar: %d %f\t %f\t %f\n",count_num,Pos_Radar[3*count_num+0],Pos_Radar[3*count_num+1],Pos_Radar[3*count_num+2]);
	//printf("Color_Radar: %d %f\t %f\t %f\n",count_num,Color_Radar[3*count_num+0],Color_Radar[3*count_num+1],Color_Radar[3*count_num+2]);


    static GLuint m_vbo_park[3] = {0,0,0};
    static GLuint m_vbo_trail0[3] = {0,0,0};
    static GLuint m_vbo_trail1[3] = {0,0,0};
    static GLuint m_vbo_radar[3] = {0,0,0};
    if(!Flag_cal){
	glGenBuffers(3, m_vbo_park);
	glGenBuffers(3, m_vbo_trail0);
	glGenBuffers(3, m_vbo_trail1);
	glGenBuffers(3, m_vbo_radar);
    }
    Flag_cal = true;
#if 0
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_radar[0]);//radar
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*(16*6+4+12)*5*3,Pos_Radar, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_radar[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*(16*6+4+12)*5*3,Color_Radar, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_radar[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*(16*6+4+12)*6, IndexArray_Radar, GL_STATIC_DRAW);
    glUseProgram(m_shaderProgram);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_radar[0]);//radar(16*6+4+12)*6-1
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_radar[1]);
        glVertexAttribPointer(m_colorHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_radar[2]);
    	glLineWidth(10);
    	glEnableVertexAttribArray(m_positionHandle);
    	glEnableVertexAttribArray(m_colorHandle);
		//glEnable(GL_PRIMITIVE_RESTART_FIXED_INDEX);
	glDrawElements(GL_LINE_STRIP, (16*6+4+12)*6, GL_UNSIGNED_SHORT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_colorHandle);
	//glDisable(GL_PRIMITIVE_RESTART_FIXED_INDEX);
#endif



            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_park[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*10*4*3,Ver_Pos_Parking, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_park[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*10*4*3,Ver_Color_Parking, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_park[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*10*5, IndexArray, GL_STATIC_DRAW);
/**************************************************Trail********************************************************/
	Circle_Center[0] = 0.0;
	Circle_Center[1] = 0.0;
    	Circle_Radius = 0.5;
	Angles[0] = 3.1415926*45/180;//
	Angles[1] = 3.1415926*135/180;
    	Cal_Trail(Circle_Center,Circle_Radius,Angles,Ver_Pos_Left,Ver_Color_Left,IndexArray_Trail);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_trail0[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*20*3,Ver_Pos_Left, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_trail0[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*20*3,Ver_Color_Left, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_trail0[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*20, IndexArray_Trail, GL_STATIC_DRAW);

	Circle_Center[0] = -0.5;
	Circle_Center[1] = -0.5;
    	Circle_Radius = 0.5;
	Angles[0] = 3.1415926*45/180;
	Angles[1] = 3.1415926*135/180;//
    	Cal_Trail(Circle_Center,Circle_Radius,Angles,Ver_Pos_Right,Ver_Color_Right,IndexArray_Trail);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_trail1[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*20*3,Ver_Pos_Right, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo_trail1[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*20*3,Ver_Color_Right, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_trail1[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*20, IndexArray_Trail, GL_STATIC_DRAW);
/**********************************************************************************************************************/
    glUseProgram(m_shaderProgram);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_park[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_park[1]);
        glVertexAttribPointer(m_colorHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_park[2]);
    	glLineWidth(10);
    	glEnableVertexAttribArray(m_positionHandle);
    	glEnableVertexAttribArray(m_colorHandle);

glEnable(GL_PRIMITIVE_RESTART_FIXED_INDEX);
	glDrawElements(GL_LINE_LOOP, 2*5, GL_UNSIGNED_SHORT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_colorHandle);
	glDisable(GL_PRIMITIVE_RESTART_FIXED_INDEX);


        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_trail1[0]);//right
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_trail1[1]);
        glVertexAttribPointer(m_colorHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_trail1[2]);
    	glLineWidth(10);
    	glEnableVertexAttribArray(m_positionHandle);
    	glEnableVertexAttribArray(m_colorHandle);
	glDrawElements(GL_LINE_STRIP, 20, GL_UNSIGNED_SHORT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_colorHandle);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_trail0[0]);//left
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo_trail0[1]);
        glVertexAttribPointer(m_colorHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_trail0[2]);
    	glLineWidth(10);
    	glEnableVertexAttribArray(m_positionHandle);
    	glEnableVertexAttribArray(m_colorHandle);
	glDrawElements(GL_LINE_STRIP, 20, GL_UNSIGNED_SHORT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_colorHandle);
}
void Cal_Radar_Line(int *Signal,int *Signal_Color,GLfloat *Ver_Pos,GLfloat *Ver_Color,unsigned short *Index)
{

	static int Label_Set[16]={60*0,60*1,60*2,60*3,60*4,60*5,60*6,60*7,60*8,60*9,60*10,60*11,60*12,60*13,60*14,60*15};
	static int Label_Valid[16]={6,6,6,8,8,6,6,6,6,6,6,12,12,6,6,6};
	int Label_Total[16]={0};
    	int Radar_Flag[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static bool Flag_Line = false;
	if(!Flag_Line){
		FILE *fp;
		int Radar_Line[16*12*5*2];
    		fp = fopen("/data/opengl_new/Lut_Calib/Radar_Line.bin", "rb");
		fseek(fp,0,SEEK_SET);
		fread(Radar_Line, sizeof(int), 16*12*5*2, fp);
		fclose(fp);
		for(int i = 0;i < 16;i++){
			if(i == 0)
				Label_Total[i] = 0;
			else
				Label_Total[i] = Label_Total[i-1] + Label_Valid[i-1]*5;
			//printf("Label_Total:%d\t%d\n",i,Label_Total[i]);
		}//

		for(int i = 0;i < 16;i++){
			for(int j = 0;j < Label_Valid[i]*5;j++){
				Ver_Pos[(Label_Total[i]+j)*3+0] = -1.0+Radar_Line[(Label_Set[i]+j)*2+0]/960.0;
				Ver_Pos[(Label_Total[i]+j)*3+1] = 1.0-Radar_Line[(Label_Set[i]+j)*2+1]/540.0;
				Ver_Pos[(Label_Total[i]+j)*3+2] = 0.0;
			if(i == 12 && j<5)
				printf("Ver_Pos::  %f\t%f\n",Ver_Pos[(Label_Total[i]+j)*3+0],Ver_Pos[(Label_Total[i]+j)*3+1]);
			}
		}
		for(int i = 0;i < 16;i++){
			for(int j = 0;j < Label_Valid[i]*5;j++){
				Ver_Color[(Label_Total[i]+j)*3+0] = 1.0;
				Ver_Color[(Label_Total[i]+j)*3+1] = 1.0;
				Ver_Color[(Label_Total[i]+j)*3+2] = 1.0;
			}
		}
		for(int i=0;i<16*6+4+12;i++)
		{
			Index[6*i+0] = 0xffff;
			Index[6*i+1] = 5*i+0;
			Index[6*i+2] = 5*i+1;
			Index[6*i+3] = 5*i+2;
			Index[6*i+4] = 5*i+3;
			Index[6*i+5] = 5*i+4;
		}
		//printf("index radar ok1\n");
		Flag_Line = true;
	//printf("Label_Total[0]000 = %d\n",Label_Total[0]);
	}
		for(int i = 0;i < 16;i++){
			if(i == 0)
				Label_Total[i] = 0;
			else
				Label_Total[i] = Label_Total[i-1] + Label_Valid[i-1]*5;
		}
		for(int i = 0;i < 16;i++){
			for(int j = 0;j < Label_Valid[i]*5;j++){
				Ver_Color[(Label_Total[i]+j)*3+0] = 1.0;
				Ver_Color[(Label_Total[i]+j)*3+1] = 1.0;
				Ver_Color[(Label_Total[i]+j)*3+2] = 1.0;
			}
		}
	//printf("Label_Total[0]111 = %d\n",Label_Total[0]);
		for(int i = 0;i < 16;i++){
		    if(Radar_Flag[i] != Signal[i]){
			//printf("Radar_Flag[i] = %d,Signal[i]= %d\n",Radar_Flag[i],Signal[i]);
			for(int j = (Signal[i]-1)*5;j < (Signal[i]-1)*5+5;j++){
			    if(Signal_Color[i] == 0){
				Ver_Color[(Label_Total[i]+j)*3+0] = 1.0;
				Ver_Color[(Label_Total[i]+j)*3+1] = 1.0;
				Ver_Color[(Label_Total[i]+j)*3+2] = 1.0;
			    }
			    else if(Signal_Color[i] == 1){
				Ver_Color[(Label_Total[i]+j)*3+0] = 1.0;
				Ver_Color[(Label_Total[i]+j)*3+1] = 1.0;
				Ver_Color[(Label_Total[i]+j)*3+2] = 0.0;
			    }
			    else if(Signal_Color[i] == 2){
				Ver_Color[(Label_Total[i]+j)*3+0] = 1.0;
				Ver_Color[(Label_Total[i]+j)*3+1] = 0.0;
				Ver_Color[(Label_Total[i]+j)*3+2] = 0.0;
			    }
			}
		    	Radar_Flag[i] = Signal[i];
		    }
		}
}
void Cal_Parking(GLfloat *Ver_Pos,GLfloat *Ver_Color,unsigned short *Index)
{
	for(int i=0;i<10;i++)
	{
		Index[5*i+0] = 0xffff;
		Index[5*i+1] = 4*i+0;
		Index[5*i+2] = 4*i+1;
		Index[5*i+3] = 4*i+2;
		Index[5*i+4] = 4*i+3;//index
	}
	for(int i=0;i<10*4;i++)
	{
		Ver_Color[3*i+0] = 1.0;
		Ver_Color[3*i+1] = 0.0;
		Ver_Color[3*i+2] = 0.0;//color
	}
}
void Cal_Trail(GLfloat *Center,GLfloat Radius,GLfloat *Angles,GLfloat *Pos_Loc,GLfloat *Color_Loc,unsigned short *IndexArray)
{
	//printf("Trail:\t %f\t %f\t %f\n",Angles[0],Angles[1],Radius);
	//float Theta = (Angles[1] - Angles[0])/20.0;
	for(int i = 0;i<20;i++)
	{
		Pos_Loc[3*i+0] = Radius*cos(Angles[0] + i*(Angles[1] - Angles[0])/20.0) + Center[0];
		Pos_Loc[3*i+1] = Radius*sin(Angles[0] + i*(Angles[1] - Angles[0])/20.0) + Center[1];
		Pos_Loc[3*i+2] = 0.0 - Center[2];//pos

		Color_Loc[3*i+0] = 1.0;
		Color_Loc[3*i+1] = 1.0;
		Color_Loc[3*i+2] = 0.0;//color
	}
	for(int i = 0;i<20;i++)
	{

	}
	for(int i = 0;i<20;i++)
	{
		IndexArray[i] = i;
	}

}
///////////////////////////////////////////////////////////////////

bool polynomial_curve_fit(vector<Point2f>& key_point, vector<Point2f>& poly_point,int n, Mat& A)
{
	double x_start,x_end,x_jump;
	x_start = key_point[0].x;
	x_end = key_point[key_point.size() - 1].x;
	x_jump = (x_end - x_start)/20.0;

	int N = key_point.size();
	Mat X = Mat::zeros(n + 1, n + 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
		for (int j = 0; j < n + 1; j++)
		{
			for (int k = 0; k < N; k++)
			{
				X.at<double>(i, j) = X.at<double>(i, j) +
					pow(key_point[k].x, i + j);
			}
		}
	}
	Mat Y = Mat::zeros(n + 1, 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
		for (int k = 0; k < N; k++)
		{
			Y.at<double>(i, 0) = Y.at<double>(i, 0) +
				pow(key_point[k].x, i) * key_point[k].y;
		}
	}
	A = Mat::zeros(n + 1, 1, CV_64FC1);
	//printf("enter solve\n");
	solve(X, Y, A,DECOMP_LU);
	//printf("end solve\n");
	//printf("x_start = %f x_end = %f x_jump = %f\n",x_start,x_end,x_jump);
	for (double x = 0.0; x < 1.0; x += 0.1)
	{
		double y = A.at<double>(0, 0) + A.at<double>(1, 0) * x;// + A.at<double>(2, 0)*pow(x, 2);// + A.at<double>(3, 0)*pow(x, 3);
		poly_point.push_back(Point2f(x, y));
		//printf("poly_point0: %f,%f\n",x,y);
		//printf("poly_point1: %f,%f\n",poly_point[0].x,poly_point[0].y);
	}
	//printf("size of poly_point = %d\n",poly_point.size());
	//printf("finish solve\n");
	return true;
}
int renderSingleSrcImage(GLuint textureId)
{
    imageBilt(textureId,0,0,1,1,1);
}
void flushToScreen()
{
	swapBuffers();
}
void setBackGround(int r,int g,int b,int a)
{
     glClearColor(r / 255.0, g / 255.0, b/ 255.0, a / 255.0);
}
void clearForeGround()
{
     glClear(GL_COLOR_BUFFER_BIT);
}
int deinitGL()
{
    deinitImageBilt();
    deinitTextureEnvir();
    deinitEGL();
    destroyWindow();
    printf("deinitGL completed!\n");
    return 0;
}

void set2DViewScale(float scale)
{
	view2DScale = scale;
}

void render2DView(GLuint tex, float scale)
{

}
