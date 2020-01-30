#ifndef _DRAW_SCENE_H__
#define _DRAW_SCENE_H__
#include "GLES2/gl2.h"

#include <iostream>
#include "opencv2/core.hpp"
#include <glm/glm.hpp>
#include "LUT.h"
using namespace std;


void setBirdEyeViewParams(const CalibLUT &lut);
void setSingleViewParams(int width, int height);

int initialGL(int width, int height);
int deinitGL();
void setBackGround(int r,int g,int b,int a);
void clearForeGround();
void flushToScreen();
int render4SrcImage(GLuint *textureIds,int Flag_Test);

int renderBird_View(GLuint *textureIds,int Flag_Test);
void Generate_SingleView(GLuint Camera_Num_Flag,GLuint *textureID, float* delta);

void Generate_Trail(float steeringAngle, float len, float width);

void Generate_2D_View(GLuint fbo, GLuint Camera_Num_Flag,GLuint *textureID, const float* delta, const float* mask);
void Generate_2D_View_Merge(GLuint fbo, GLuint Camera_Num_Flag,GLuint *textureID, const float* delta, const float* mask);
void Generate_Car_Icon(const glm::mat4& transform = glm::mat4(1.0));
void Generate_Car_Icon_Background();
void Generate_Image_View(const char* filename, int x, int y, int width, int height);
void Generate_Image_View(GLuint tex, int x, int y, int width, int height, const glm::mat4& transformMatrix = glm::mat4(1.0));


void Calib_2D_Ver_FrontBack(GLuint Camera_Num_Flag,GLfloat *Vertex_pos,GLfloat *Vertex_tex,unsigned int *Vertex_index,GLfloat START_X,GLfloat START_Y,GLfloat WIDTH,GLfloat HEIGHT);
void Merge_Calib_2D_Ver_FrontBack(GLuint Camera_Num_Flag,GLfloat *Vertex_pos,GLfloat *Vertex_Weight,GLfloat *Vertex_tex1,GLfloat *Vertex_tex2,unsigned int *Vertex_index,GLfloat START_X,GLfloat START_Y,GLfloat WIDTH,GLfloat HEIGHT);


void Calib_2D_Single_View(GLuint Camera_Num_Flag,GLfloat *Vertex_pos,GLfloat *Vertex_tex,unsigned short *Vertex_index,GLfloat START_X,GLfloat START_Y,GLfloat WIDTH,GLfloat HEIGHT);


int renderSingleSrcImage(GLuint textureId);
void Calib_2D_Init();
void Fbo_2D_Init();
//void Generate_Radar_Chart(GLuint Num_Flag);
void Generate_Radar_Chart(GLuint Flag_Next,GLuint Num_Flag);
void Calib_2D_Single_Line(GLint Flag_Next,GLint Num_Line,GLint *Line_Num,GLfloat *Vertex_pos,GLfloat *Vertex_color,unsigned short *Vertex_index);
void Calib_2D_Single_Static_Line(GLint Flag_Next,GLint Num_Line,GLint *Line_Num,GLfloat *Vertex_pos,GLfloat *Vertex_color,unsigned short *Vertex_index);
void Avm_ParkingTrail(GLint Flag_Next);
void Cal_Parking(GLfloat *Ver_Color,GLfloat *Ver_Pos,unsigned short *Index);
void Cal_Trail(GLfloat *Center,GLfloat Radius,GLfloat *Angles,GLfloat *Pos_Loc,GLfloat *Color_Loc,unsigned short *IndexArray);
void Test_Fbo(GLuint *TextureId);
void Test_Fbo_Render(GLuint *TextureId);

//
void Update_Radar_Data(GLuint Radar_Number,GLuint Radar_Level);
void Cal_Radar_Line(int *Signal,int *Signal_Color,GLfloat *Ver_Pos,GLfloat *Ver_Color,unsigned short *Index);
//
bool polynomial_curve_fit(vector<cv::Point2f>& key_point, vector<cv::Point2f>& poly_point,int n, cv::Mat& A);
//fbo
void Calib_2D_Ver_FrontBack_Fbo(GLuint Camera_Num_Flag,GLfloat *Vertex_pos,GLfloat *Vertex_tex,unsigned int *Vertex_index,GLfloat START_X,GLfloat START_Y,GLfloat WIDTH,GLfloat HEIGHT);
void Merge_Calib_2D_Ver_FrontBack_Fbo(GLuint Camera_Num_Flag,GLfloat *Vertex_pos,GLfloat *Vertex_Weight,GLfloat *Vertex_tex1,GLfloat *Vertex_tex2,unsigned int *Vertex_index,GLfloat START_X,GLfloat START_Y,GLfloat WIDTH,GLfloat HEIGHT);

void CreateTextureID_2D();

#endif // _DRAW_SCENE_H__
