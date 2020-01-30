#include <stdio.h>
#include "DrawScene.h"
#include "GLES3/gl3.h"
#include "ShaderUtil.h"
/******************************************************FBO value start********************************************************************/
#define FRONT_BACK_ROW_FBO  (100)
#define FRONT_BACK_COL_FBO  (200)
#define CALIB_FB_ROW_FBO    (198)
#define CALIB_FB_COL_FBO    (512)


#define LEFT_RIGHT_ROW_FBO  (200)
#define LEFT_RIGHT_COL_FBO  (100)
#define CALIB_LR_ROW_FBO    (512)
#define CALIB_LR_COL_FBO    (234)


#define MERGE_FRONT_LEFT_ROW_FBO    (100)
#define MERGE_FRONT_LEFT_COL_FBO    (100)
#define CALIB_FL_ROW_FBO            (198)
#define CALIB_FL_COL_FBO            (234)


#define GPU_IMAGE_WIDTH		(512)
#define GPU_IMAGE_HEIGHT	(512)
unsigned char IMAGE_AVM[GPU_IMAGE_WIDTH*GPU_IMAGE_HEIGHT*3] = {0};

GLuint Texture_Id;
GLuint Framebuffer_Id;
GLuint Renderbuffer_Id;
//unmerged
GLuint fbo_unmerge_front[4] = {0}, fbo_unmerge_back[4] = {0},fbo_unmerge_left[4] = {0},fbo_unmerge_right[4] = {0};
float VertexPos_fbo_Front[FRONT_BACK_COL_FBO*FRONT_BACK_ROW_FBO*6*3] ={0.0},VertexPos_fbo_Back[FRONT_BACK_COL_FBO*FRONT_BACK_ROW_FBO*6*3] ={0.0};
float VertexPos_fbo_Left[LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO*6*3] ={0.0},VertexPos_fbo_Right[LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO*6*3] ={0.0};

float VertexColors_fbo_Front[FRONT_BACK_COL_FBO*FRONT_BACK_ROW_FBO*6*2] ={0.0},VertexColors_fbo_Back[FRONT_BACK_COL_FBO*FRONT_BACK_ROW_FBO*6*2] ={0.0};
float VertexColors_fbo_Left[LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO*6*2] ={0.0},VertexColors_fbo_Right[LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO*6*2] ={0.0};
unsigned int Index_Array_fbo_FB[FRONT_BACK_COL_FBO*FRONT_BACK_ROW_FBO*6] = {0},Index_Array_fbo_LR[LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO*6] = {0};
//merge
GLuint m_vbo0_fbo_Merge[6] = {0},m_vbo1_fbo_Merge[6] = {0},m_vbo2_fbo_Merge[6] = {0},m_vbo3_fbo_Merge[6] = {0};
GLfloat VerPos_Merge_fbo[MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*6*3] ={0.0};
GLfloat VerColor1_fbo_Merge[MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*6*2] ={0.0},VerColor2_fbo_Merge[MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*6*2] ={0.0};
GLfloat Vertex_Weight_fbo[MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*6*2] ={0.0};
unsigned int IndexArray_fbo_Merge[MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*6] = {0};
/******************************************************FBO value end********************************************************************/
void Fbo_2D_Init()
{
	printf("fbo 2d init is ok!\n");
	//unmerge
        glGenBuffers(4, fbo_unmerge_front);
        glGenBuffers(4, fbo_unmerge_back);
        glGenBuffers(4, fbo_unmerge_left);
        glGenBuffers(4, fbo_unmerge_right);
	//merge
        glGenBuffers(6, m_vbo0_fbo_Merge);
	glGenBuffers(6, m_vbo1_fbo_Merge);
	glGenBuffers(6, m_vbo2_fbo_Merge);
	glGenBuffers(6, m_vbo3_fbo_Merge);
/*********************************************************unmerge****************************************************************************************/
	for(int Camera_ID = 0;Camera_ID < 4;Camera_ID++)
	{
	    switch(Camera_ID)
	    {
		case 0:
			Calib_2D_Ver_FrontBack_Fbo(Camera_ID,VertexPos_fbo_Front,VertexColors_fbo_Front,Index_Array_fbo_FB,-1.0,1.0,2.0*CALIB_FB_COL_FBO/1920,2.0*CALIB_FB_ROW_FBO/1080);
			glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_front[0]);
			glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*FRONT_BACK_ROW_FBO*FRONT_BACK_COL_FBO*3,VertexPos_fbo_Front, GL_STATIC_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_front[1]);
			glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*FRONT_BACK_ROW_FBO*FRONT_BACK_COL_FBO*2,VertexColors_fbo_Front, GL_STATIC_DRAW);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, fbo_unmerge_front[2]);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*FRONT_BACK_ROW_FBO*FRONT_BACK_COL_FBO, Index_Array_fbo_FB, GL_STATIC_DRAW);//
			break;
		case 1:
			Calib_2D_Ver_FrontBack_Fbo(Camera_ID,VertexPos_fbo_Back,VertexColors_fbo_Back,Index_Array_fbo_FB,-1.0,1.0-2.0*(CALIB_LR_ROW_FBO-CALIB_FB_ROW_FBO)/1080,2.0*CALIB_FB_COL_FBO/1920,2.0*CALIB_FB_ROW_FBO/1080);
	    		glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_back[0]);
            		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*FRONT_BACK_ROW_FBO*FRONT_BACK_COL_FBO*3,VertexPos_fbo_Back, GL_STATIC_DRAW);
            		glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_back[1]);
            		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*FRONT_BACK_ROW_FBO*FRONT_BACK_COL_FBO*2,VertexColors_fbo_Back, GL_STATIC_DRAW);
            		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, fbo_unmerge_back[2]);
            		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*FRONT_BACK_ROW_FBO*FRONT_BACK_COL_FBO, Index_Array_fbo_FB, GL_STATIC_DRAW);//
			break;
		case 2:
			Calib_2D_Ver_FrontBack_Fbo(Camera_ID,VertexPos_fbo_Left,VertexColors_fbo_Left,Index_Array_fbo_LR,-1.0,1.0,2.0*CALIB_LR_COL_FBO/1920,2.0*CALIB_LR_ROW_FBO/1080);
	    		glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_left[0]);
            		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO*3,VertexPos_fbo_Left, GL_STATIC_DRAW);
            		glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_left[1]);
            		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO*2,VertexColors_fbo_Left, GL_STATIC_DRAW);
            		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, fbo_unmerge_left[2]);
            		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO, Index_Array_fbo_LR, GL_STATIC_DRAW);//
			break;
		case 3:
			Calib_2D_Ver_FrontBack_Fbo(Camera_ID,VertexPos_fbo_Right,VertexColors_fbo_Right,Index_Array_fbo_LR,-1.0+2.0*(CALIB_FB_COL_FBO-CALIB_LR_COL_FBO)/1920,1.0,2.0*CALIB_LR_COL_FBO/1920,2.0*CALIB_LR_ROW_FBO/1080);
			glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_right[0]);
            		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO*3,VertexPos_fbo_Right, GL_STATIC_DRAW);
            		glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_right[1]);
            		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO*2,VertexColors_fbo_Right, GL_STATIC_DRAW);
            		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, fbo_unmerge_right[2]);
            		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO, Index_Array_fbo_LR, GL_STATIC_DRAW);
			break;
		default:
			printf("function : Calib_2D_Init() switch can't find proper Value.\n");
			break;
	    }//end : switch(Camera_ID)
	}//end : for(int Camera_ID = 0;Camera_ID < 4;Camera_ID++)
/*********************************************************merge****************************************************************************************/
	for(int Camera_ID = 0;Camera_ID < 4;Camera_ID++)
	{
	    switch(Camera_ID)
	    {
		case 0:			Merge_Calib_2D_Ver_FrontBack_Fbo(Camera_ID,VerPos_Merge_fbo,Vertex_Weight_fbo,VerColor1_fbo_Merge,VerColor2_fbo_Merge,IndexArray_fbo_Merge,-1.0,1.0,2.0*CALIB_LR_COL_FBO/1920,2.0*CALIB_FB_ROW_FBO/1080);
			glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_fbo_Merge[0]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*3,VerPos_Merge_fbo, GL_STATIC_DRAW);
        		glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_fbo_Merge[1]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*2,VerColor1_fbo_Merge, GL_STATIC_DRAW);
        		glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_fbo_Merge[2]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*2,VerColor2_fbo_Merge, GL_STATIC_DRAW);
        		glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_fbo_Merge[3]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO,Vertex_Weight_fbo, GL_STATIC_DRAW);
        		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo0_fbo_Merge[4]);
        		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO, IndexArray_fbo_Merge, GL_STATIC_DRAW);
			break;
		case 1:			Merge_Calib_2D_Ver_FrontBack_Fbo(Camera_ID,VerPos_Merge_fbo,Vertex_Weight_fbo,VerColor1_fbo_Merge,VerColor2_fbo_Merge,IndexArray_fbo_Merge,-1.0+2.0*(CALIB_FB_COL_FBO-CALIB_LR_COL_FBO)/1920,1.0,2.0*CALIB_LR_COL_FBO/1920,2.0*CALIB_FB_ROW_FBO/1080);
			glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_fbo_Merge[0]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*3,VerPos_Merge_fbo, GL_STATIC_DRAW);
        		glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_fbo_Merge[1]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*2,VerColor1_fbo_Merge, GL_STATIC_DRAW);
        		glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_fbo_Merge[2]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*2,VerColor2_fbo_Merge, GL_STATIC_DRAW);
        		glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_fbo_Merge[3]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO,Vertex_Weight_fbo, GL_STATIC_DRAW);
        		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo1_fbo_Merge[4]);
        		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO, IndexArray_fbo_Merge, GL_STATIC_DRAW);
			break;
		case 2:			Merge_Calib_2D_Ver_FrontBack_Fbo(Camera_ID,VerPos_Merge_fbo,Vertex_Weight_fbo,VerColor1_fbo_Merge,VerColor2_fbo_Merge,IndexArray_fbo_Merge,-1.0,1.0-2.0*(CALIB_LR_ROW_FBO-CALIB_FB_ROW_FBO)/1080,2.0*CALIB_LR_COL_FBO/1920,2.0*CALIB_FB_ROW_FBO/1080);
			glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_fbo_Merge[0]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*3,VerPos_Merge_fbo, GL_STATIC_DRAW);
        		glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_fbo_Merge[1]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*2,VerColor1_fbo_Merge, GL_STATIC_DRAW);
        		glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_fbo_Merge[2]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*2,VerColor2_fbo_Merge, GL_STATIC_DRAW);
        		glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_fbo_Merge[3]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO,Vertex_Weight_fbo, GL_STATIC_DRAW);
        		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo2_fbo_Merge[4]);
        		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO, IndexArray_fbo_Merge, GL_STATIC_DRAW);
			break;
		case 3:			Merge_Calib_2D_Ver_FrontBack_Fbo(Camera_ID,VerPos_Merge_fbo,Vertex_Weight_fbo,VerColor1_fbo_Merge,VerColor2_fbo_Merge,IndexArray_fbo_Merge,-1.0+2.0*(CALIB_FB_COL_FBO-CALIB_LR_COL_FBO)/1920,1.0-2.0*(CALIB_LR_ROW_FBO-CALIB_FB_ROW_FBO)/1080,2.0*CALIB_LR_COL_FBO/1920,2.0*CALIB_FB_ROW_FBO/1080);
			glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_fbo_Merge[0]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*3,VerPos_Merge_fbo, GL_STATIC_DRAW);
        		glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_fbo_Merge[1]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*2,VerColor1_fbo_Merge, GL_STATIC_DRAW);
        		glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_fbo_Merge[2]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO*2,VerColor2_fbo_Merge, GL_STATIC_DRAW);
        		glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_fbo_Merge[3]);
        		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO,Vertex_Weight_fbo, GL_STATIC_DRAW);
        		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo3_fbo_Merge[4]);
        		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO, IndexArray_fbo_Merge, GL_STATIC_DRAW);
			break;
		default:
			printf("function : Calib_2D_Init() switch can't find proper Value.\n");
			break;
	    }//end : switch(Camera_ID)
	}//end : for(int Camera_ID = 0;Camera_ID < 4;Camera_ID++)
}
void Calib_2D_Ver_FrontBack_Fbo(GLuint Camera_Num_Flag,GLfloat *Vertex_pos,GLfloat *Vertex_tex,unsigned int *Vertex_index,GLfloat START_X,GLfloat START_Y,GLfloat WIDTH,GLfloat HEIGHT)
{
	printf("fbo 2d init is ok Calib_2D_Ver_FrontBack_Fbo!\n");
   FILE *fp ;
   //printf("WIDTH HEIGHT::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::%f\t%f\n",WIDTH,HEIGHT);
   float temp_x1=START_X,temp_y1=START_Y,temp_x2,temp_y2;
   float X_JUMP = 1.0*WIDTH/FRONT_BACK_COL_FBO,Y_JUMP = 1.0*HEIGHT/FRONT_BACK_ROW_FBO;
   float X_JUMP_LR = 1.0*WIDTH/LEFT_RIGHT_COL_FBO,Y_JUMP_LR = 1.0*HEIGHT/LEFT_RIGHT_ROW_FBO;
   float X_COUNT,Y_COUNT;
    unsigned int front_Calib[CALIB_FB_ROW_FBO*CALIB_FB_COL_FBO]={0};
    unsigned int LR_Calib[CALIB_LR_ROW_FBO*CALIB_LR_COL_FBO]={0};
//printf("Calib_2D_Ver_FrontBack(Camera_Num_Flag:%d) WIDTH:%f\tHEIGHT:%f\n",Camera_Num_Flag,WIDTH,HEIGHT);
    if(Camera_Num_Flag == 0){
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Front_Fbo.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(front_Calib, sizeof(unsigned int), CALIB_FB_ROW_FBO*CALIB_FB_COL_FBO, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 1){
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Back_Fbo.bin", "rb");//Lut_Back
	fseek(fp,0,SEEK_SET);
	fread(front_Calib, sizeof(unsigned int), CALIB_FB_ROW_FBO*CALIB_FB_COL_FBO, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 2){
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Left_Fbo.bin", "rb");//Lut_Back
	fseek(fp,0,SEEK_SET);
	fread(LR_Calib, sizeof(unsigned int), CALIB_LR_ROW_FBO*CALIB_LR_COL_FBO, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 3){
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Right_Fbo.bin", "rb");//Lut_Back
	fseek(fp,0,SEEK_SET);
	fread(LR_Calib, sizeof(unsigned int), CALIB_LR_ROW_FBO*CALIB_LR_COL_FBO, fp);
	fclose(fp);
    }
	//printf("Camera_Num_Flag:%d\tfind error333\n",Camera_Num_Flag);
   	/****************/
printf("(Camera_Num_Flag:%d) X_JUMP:%f\tY_JUMP:%f\n",Camera_Num_Flag,X_JUMP,Y_JUMP);
    if(Camera_Num_Flag == 0 || Camera_Num_Flag == 1){
	for(int i=0;i<FRONT_BACK_ROW_FBO;i++)
	{
		//temp_y1 = START_Y-i*Y_JUMP;
		temp_y1 = START_Y-1.0*i*HEIGHT/FRONT_BACK_ROW_FBO;
   	//printf("i,temp_y1::%d\t%f\n",i,temp_y1);
        for(int j=0;j<FRONT_BACK_COL_FBO;j++)
          {
            //temp_x1 =START_X+j*X_JUMP;WIDTH/FRONT_BACK_COL
	    temp_x1 =START_X+1.0*j*WIDTH/FRONT_BACK_COL_FBO;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+0] = temp_x1;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+1] = temp_y1;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+2] = 0;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+3] = temp_x1+X_JUMP;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+4] = temp_y1;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+5] = 0;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+6] = temp_x1+X_JUMP;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+7] = temp_y1-Y_JUMP;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+8] = 0;                  //FIRST TRIANGLE

            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+9] =  temp_x1;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+10] = temp_y1-Y_JUMP;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+11] = 0;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+12] = temp_x1+X_JUMP;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+13] = temp_y1-Y_JUMP;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+14] = 0;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+15] = temp_x1;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+16] = temp_y1;
            Vertex_pos[(i*FRONT_BACK_COL_FBO+j)*18+17] = 0;                 //SECOND TRIANGLE
          }
	}
    //X_COUNT = int(480.0/FRONT_BACK_ROW);//CALIB_FB_ROW
    X_COUNT = 1.0*CALIB_FB_ROW_FBO/FRONT_BACK_ROW_FBO;//CALIB_FB_ROW
    Y_COUNT = 1.0*CALIB_FB_COL_FBO/FRONT_BACK_COL_FBO;
    //printf("X_COUNT Y_COUNT%f\t%f\n",X_COUNT,Y_COUNT);
    for(int i=0;i<FRONT_BACK_ROW_FBO;i++)
	{
        for(int j=0;j<FRONT_BACK_COL_FBO;j++)
        {//             temp_y = (front_Calib[i*720+j]>>10)&0x7ff;
         //             temp_x = (front_Calib[i*720+j]>>21)&0x7ff;

            Vertex_tex[(i*FRONT_BACK_COL_FBO+j)*12+0] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*(Y_COUNT))]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*FRONT_BACK_COL_FBO+j)*12+1] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*(Y_COUNT))]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*FRONT_BACK_COL_FBO+j)*12+2] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*(Y_COUNT)+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*FRONT_BACK_COL_FBO+j)*12+3] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*(Y_COUNT)+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*FRONT_BACK_COL_FBO+j)*12+4] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*(Y_COUNT)+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*FRONT_BACK_COL_FBO+j)*12+5] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*(Y_COUNT)+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*FRONT_BACK_COL_FBO+j)*12+6] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*(Y_COUNT))]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*FRONT_BACK_COL_FBO+j)*12+7] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*(Y_COUNT))]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*FRONT_BACK_COL_FBO+j)*12+8] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*(Y_COUNT)+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*FRONT_BACK_COL_FBO+j)*12+9] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*(Y_COUNT)+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*FRONT_BACK_COL_FBO+j)*12+10] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*Y_COUNT)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*FRONT_BACK_COL_FBO+j)*12+11] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*Y_COUNT)]>>21)&0x7ff)/720.0;
        }//UPDATE X&Y
	}
//printf("hello error 1111\n");
	for(int i=0;i<FRONT_BACK_COL_FBO*FRONT_BACK_ROW_FBO*6;i++)
	{
        	Vertex_index[i] = i;
    	}
    }
    if(Camera_Num_Flag == 2 || Camera_Num_Flag == 3){//CALIB_FB_ROW*CALIB_FB_COL  #define LEFT_RIGHT_ROW  (128)#define LEFT_RIGHT_COL  (50)
	for(int i=0;i<LEFT_RIGHT_ROW_FBO;i++)//CALIB_LR_ROW*CALIB_LR_COL
	{
		//temp_y1 = START_Y-i*Y_JUMP_LR;
		temp_y1 = START_Y-1.0*i*HEIGHT/LEFT_RIGHT_ROW_FBO;
        for(int j=0;j<LEFT_RIGHT_COL_FBO;j++)
          {
            //temp_x1 =START_X+j*X_JUMP_LR;

	    temp_x1 =START_X+1.0*j*WIDTH/LEFT_RIGHT_COL_FBO;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+0] = temp_x1;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+1] = temp_y1;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+2] = 0;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+3] = temp_x1+X_JUMP_LR;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+4] = temp_y1;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+5] = 0;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+6] = temp_x1+X_JUMP_LR;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+7] = temp_y1-Y_JUMP_LR;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+8] = 0;                  //FIRST TRIANGLE

            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+9] =  temp_x1;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+10] = temp_y1-Y_JUMP_LR;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+11] = 0;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+12] = temp_x1+X_JUMP_LR;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+13] = temp_y1-Y_JUMP_LR;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+14] = 0;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+15] = temp_x1;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+16] = temp_y1;
            Vertex_pos[(i*LEFT_RIGHT_COL_FBO+j)*18+17] = 0;                 //SECOND TRIANGLE
          }
	}
    //X_COUNT = int(480.0/FRONT_BACK_ROW);//CALIB_FB_ROW
    X_COUNT = 1.0*CALIB_LR_ROW_FBO/LEFT_RIGHT_ROW_FBO;//CALIB_FB_ROW
    Y_COUNT = 1.0*CALIB_LR_COL_FBO/LEFT_RIGHT_COL_FBO;
    //printf("X_COUNT Y_COUNT%f\t%f\n",X_COUNT,Y_COUNT);
    for(int i=0;i<LEFT_RIGHT_ROW_FBO;i++)
	{
        for(int j=0;j<LEFT_RIGHT_COL_FBO;j++)
        {
            Vertex_tex[(i*LEFT_RIGHT_COL_FBO+j)*12+0] = ((LR_Calib[int(i*X_COUNT)*CALIB_LR_COL_FBO+int(j*(Y_COUNT))]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*LEFT_RIGHT_COL_FBO+j)*12+1] = ((LR_Calib[int(i*X_COUNT)*CALIB_LR_COL_FBO+int(j*(Y_COUNT))]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*LEFT_RIGHT_COL_FBO+j)*12+2] = ((LR_Calib[int(i*X_COUNT)*CALIB_LR_COL_FBO+int(j*(Y_COUNT)+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*LEFT_RIGHT_COL_FBO+j)*12+3] = ((LR_Calib[int(i*X_COUNT)*CALIB_LR_COL_FBO+int(j*(Y_COUNT)+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*LEFT_RIGHT_COL_FBO+j)*12+4] = ((LR_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_LR_COL_FBO+int(j*(Y_COUNT)+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*LEFT_RIGHT_COL_FBO+j)*12+5] = ((LR_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_LR_COL_FBO+int(j*(Y_COUNT)+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*LEFT_RIGHT_COL_FBO+j)*12+6] = ((LR_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_LR_COL_FBO+int(j*(Y_COUNT))]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*LEFT_RIGHT_COL_FBO+j)*12+7] = ((LR_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_LR_COL_FBO+int(j*(Y_COUNT))]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*LEFT_RIGHT_COL_FBO+j)*12+8] = ((LR_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_LR_COL_FBO+int(j*(Y_COUNT)+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*LEFT_RIGHT_COL_FBO+j)*12+9] = ((LR_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_LR_COL_FBO+int(j*(Y_COUNT)+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex[(i*LEFT_RIGHT_COL_FBO+j)*12+10] = ((LR_Calib[int(i*X_COUNT)*CALIB_LR_COL_FBO+int(j*Y_COUNT)]>>10)&0x7ff)/1280.0;
            Vertex_tex[(i*LEFT_RIGHT_COL_FBO+j)*12+11] = ((LR_Calib[int(i*X_COUNT)*CALIB_LR_COL_FBO+int(j*Y_COUNT)]>>21)&0x7ff)/720.0;
        }//UPDATE X&Y
	}
	for(int i=0;i<LEFT_RIGHT_COL_FBO*LEFT_RIGHT_ROW_FBO*6;i++)
	{
        	Vertex_index[i] = i;
    	}
    }
}
void Merge_Calib_2D_Ver_FrontBack_Fbo(GLuint Camera_Num_Flag,GLfloat *Vertex_pos,GLfloat *Vertex_Weight,GLfloat *Vertex_tex1,GLfloat *Vertex_tex2,unsigned int *Vertex_index,GLfloat START_X,GLfloat START_Y,GLfloat WIDTH,GLfloat HEIGHT)
{
   	FILE *fp ;
   	float temp_x1=START_X,temp_y1=START_Y,temp_x2,temp_y2;
   	float X_JUMP = 1.0*WIDTH/MERGE_FRONT_LEFT_COL_FBO,Y_JUMP = 1.0*HEIGHT/MERGE_FRONT_LEFT_ROW_FBO;
   	//printf("X_JUMPM,Y_JUMPM::%f\t,%f\n",X_JUMP,Y_JUMP);
    float X_COUNT,Y_COUNT;
    int COL_OFFSET,ROW_OFFSET;
    /***************************************************///front  CALIB_FB_COL
    unsigned int front_Calib[CALIB_FB_ROW_FBO*CALIB_FB_COL_FBO]={0};
    unsigned int left_Calib[CALIB_LR_ROW_FBO*CALIB_LR_COL_FBO]={0};
    if(Camera_Num_Flag == 0){
	COL_OFFSET =0;
	ROW_OFFSET =0;
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Front_Fbo.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(front_Calib, sizeof(unsigned int), CALIB_FB_ROW_FBO*CALIB_FB_COL_FBO, fp);
	fclose(fp);

        fp = fopen("/data/opengl_new/Lut_Calib/Lut_Left_Fbo.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(left_Calib, sizeof(unsigned int), CALIB_LR_ROW_FBO*CALIB_LR_COL_FBO, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 1){
	COL_OFFSET =CALIB_FB_COL_FBO-CALIB_LR_COL_FBO;
	ROW_OFFSET =0;
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Front_Fbo.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(front_Calib, sizeof(unsigned int), CALIB_FB_ROW_FBO*CALIB_FB_COL_FBO, fp);
	fclose(fp);

        fp = fopen("/data/opengl_new/Lut_Calib/Lut_Right_Fbo.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(left_Calib, sizeof(unsigned int), CALIB_LR_ROW_FBO*CALIB_LR_COL_FBO, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 2){
	COL_OFFSET =0;
	ROW_OFFSET =CALIB_LR_ROW_FBO-CALIB_FB_ROW_FBO;
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Back_Fbo.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(front_Calib, sizeof(unsigned int), CALIB_FB_ROW_FBO*CALIB_FB_COL_FBO, fp);
	fclose(fp);

        fp = fopen("/data/opengl_new/Lut_Calib/Lut_Left_Fbo.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(left_Calib, sizeof(unsigned int), CALIB_LR_ROW_FBO*CALIB_LR_COL_FBO, fp);
	fclose(fp);
    }
    else if(Camera_Num_Flag == 3){
	COL_OFFSET =CALIB_FB_COL_FBO-CALIB_LR_COL_FBO;
	ROW_OFFSET =CALIB_LR_ROW_FBO-CALIB_FB_ROW_FBO;
    	fp = fopen("/data/opengl_new/Lut_Calib/Lut_Back_Fbo.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(front_Calib, sizeof(unsigned int), CALIB_FB_ROW_FBO*CALIB_FB_COL_FBO, fp);
	fclose(fp);

        fp = fopen("/data/opengl_new/Lut_Calib/Lut_Right_Fbo.bin", "rb");
	fseek(fp,0,SEEK_SET);
	fread(left_Calib, sizeof(unsigned int), CALIB_LR_ROW_FBO*CALIB_LR_COL_FBO, fp);
	fclose(fp);
    }
	//printf("COL_OFFSET,ROW_OFFSET::%d\t,%d\n",COL_OFFSET,ROW_OFFSET);
 	//printf("START_X,START_Y::%d\t,%d\n",COL_OFFSET,ROW_OFFSET);
   	/****************/
	for(int i=0;i<MERGE_FRONT_LEFT_ROW_FBO;i++)
	{
	        temp_y1 = START_Y-1.0*i*HEIGHT/MERGE_FRONT_LEFT_ROW_FBO;
        for(int j=0;j<MERGE_FRONT_LEFT_COL_FBO;j++)
        {
            temp_x1 =START_X+1.0*j*WIDTH/MERGE_FRONT_LEFT_COL_FBO;

            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+0] = temp_x1;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+1] = temp_y1;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+2] = 0;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+3] = temp_x1+X_JUMP;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+4] = temp_y1;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+5] = 0;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+6] = temp_x1+X_JUMP;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+7] = temp_y1-Y_JUMP;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+8] = 0;                  //FIRST TRIANGLE

            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+9] =  temp_x1;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+10] = temp_y1-Y_JUMP;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+11] = 0;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+12] = temp_x1+X_JUMP;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+13] = temp_y1-Y_JUMP;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+14] = 0;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+15] = temp_x1;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+16] = temp_y1;
            Vertex_pos[(i*MERGE_FRONT_LEFT_COL_FBO+j)*18+17] = 0;                 //SECOND TRIANGLE
                              //UPDATE X
        }//UPDATE X&Y
	}
    //X_COUNT = int(480.0/FRONT_BACK_ROW);//CALIB_FB_ROW
    X_COUNT = 1.0*CALIB_FL_ROW_FBO/MERGE_FRONT_LEFT_ROW_FBO;//CALIB_FB_ROW
    Y_COUNT = 1.0*CALIB_FL_COL_FBO/MERGE_FRONT_LEFT_COL_FBO;
    //printf("X_COUNT1 Y_COUNT1%f\t%f\n",X_COUNT,Y_COUNT);
    //for(int i=0;i<MERGE_FRONT_LEFT_ROW;i++)
    //for(int i=0;i<MERGE_FRONT_LEFT_ROW;i++)
    for(int i=0;i<MERGE_FRONT_LEFT_ROW_FBO;i++)
	{
        for(int j=0;j<MERGE_FRONT_LEFT_COL_FBO;j++)
        {
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+0] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*Y_COUNT)+COL_OFFSET]>>10)&0x7ff)/1280.0;
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+1] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*Y_COUNT)+COL_OFFSET]>>21)&0x7ff)/720.0;

            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+2] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET]>>10)&0x7ff)/1280.0;
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+3] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET]>>21)&0x7ff)/720.0;

            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+4] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET]>>10)&0x7ff)/1280.0;
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+5] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET]>>21)&0x7ff)/720.0;

            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+6] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*Y_COUNT)+COL_OFFSET]>>10)&0x7ff)/1280.0;
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+7] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*Y_COUNT)+COL_OFFSET]>>21)&0x7ff)/720.0;

            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+8] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET]>>10)&0x7ff)/1280.0;
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+9] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET]>>21)&0x7ff)/720.0;

            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+10] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*Y_COUNT+COL_OFFSET)]>>10)&0x7ff)/1280.0;
            Vertex_tex1[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+11] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*Y_COUNT+COL_OFFSET)]>>21)&0x7ff)/720.0;

        }//UPDATE X&Y
	}
    X_COUNT = 1.0*CALIB_FL_ROW_FBO/MERGE_FRONT_LEFT_ROW_FBO;//CALIB_FB_ROW
    Y_COUNT = 1.0*CALIB_FL_COL_FBO/MERGE_FRONT_LEFT_COL_FBO;
    //printf("X_COUNT2 Y_COUNT2%f\t%f\n",X_COUNT,Y_COUNT);
    //for(int i=0;i<MERGE_FRONT_LEFT_ROW;i++)
    //for(int i=0;i<MERGE_FRONT_LEFT_ROW;i++)
    for(int i=0;i<MERGE_FRONT_LEFT_ROW_FBO;i++)
	{
        for(int j=0;j<MERGE_FRONT_LEFT_COL_FBO;j++)
        {
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+0] = ((left_Calib[int(i*X_COUNT+ROW_OFFSET)*CALIB_LR_COL_FBO+int(j*Y_COUNT)]>>10)&0x7ff)/1280.0;
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+1] = ((left_Calib[int(i*X_COUNT+ROW_OFFSET)*CALIB_LR_COL_FBO+int(j*Y_COUNT)]>>21)&0x7ff)/720.0;

            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+2] = ((left_Calib[int(i*X_COUNT+ROW_OFFSET)*CALIB_LR_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+3] = ((left_Calib[int(i*X_COUNT+ROW_OFFSET)*CALIB_LR_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+4] = ((left_Calib[int(i*X_COUNT+X_COUNT-1+ROW_OFFSET)*CALIB_LR_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+5] = ((left_Calib[int(i*X_COUNT+X_COUNT-1+ROW_OFFSET)*CALIB_LR_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+6] = ((left_Calib[int(i*X_COUNT+X_COUNT-1+ROW_OFFSET)*CALIB_LR_COL_FBO+int(j*Y_COUNT)]>>10)&0x7ff)/1280.0;
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+7] = ((left_Calib[int(i*X_COUNT+X_COUNT-1+ROW_OFFSET)*CALIB_LR_COL_FBO+int(j*Y_COUNT)]>>21)&0x7ff)/720.0;

            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+8] = ((left_Calib[int(i*X_COUNT+X_COUNT-1+ROW_OFFSET)*CALIB_LR_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)]>>10)&0x7ff)/1280.0;
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+9] = ((left_Calib[int(i*X_COUNT+X_COUNT-1+ROW_OFFSET)*CALIB_LR_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)]>>21)&0x7ff)/720.0;

            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+10] = ((left_Calib[int(i*X_COUNT+ROW_OFFSET)*CALIB_LR_COL_FBO+int(j*Y_COUNT)]>>10)&0x7ff)/1280.0;
            Vertex_tex2[(i*MERGE_FRONT_LEFT_COL_FBO+j)*12+11] = ((left_Calib[int(i*X_COUNT+ROW_OFFSET)*CALIB_LR_COL_FBO+int(j*Y_COUNT)]>>21)&0x7ff)/720.0;
        }//UPDATE X&Y
	}

    X_COUNT = 1.0*CALIB_FL_ROW_FBO/MERGE_FRONT_LEFT_ROW_FBO;//CALIB_FB_ROW
    Y_COUNT = 1.0*CALIB_FL_COL_FBO/MERGE_FRONT_LEFT_COL_FBO;
    //printf("X_COUNT3 Y_COUNT3%f\t%f\n",X_COUNT,Y_COUNT);
    for(int i=0;i<MERGE_FRONT_LEFT_ROW_FBO;i++)
	{
        for(int j=0;j<MERGE_FRONT_LEFT_COL_FBO;j++)
        {//weight_merge = ((front_Calib[i*720+j])&0x3ff)/1023.0;
            Vertex_Weight[(i*MERGE_FRONT_LEFT_COL_FBO+j)*6+0] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*Y_COUNT)+COL_OFFSET])&0x3ff)/1023.0;

            Vertex_Weight[(i*MERGE_FRONT_LEFT_COL_FBO+j)*6+1] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET])&0x3ff)/1023.0;

            Vertex_Weight[(i*MERGE_FRONT_LEFT_COL_FBO+j)*6+2] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET])&0x3ff)/1023.0;

            Vertex_Weight[(i*MERGE_FRONT_LEFT_COL_FBO+j)*6+3] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*Y_COUNT)+COL_OFFSET])&0x3ff)/1023.0;

            Vertex_Weight[(i*MERGE_FRONT_LEFT_COL_FBO+j)*6+4] = ((front_Calib[int(i*X_COUNT+X_COUNT-1)*CALIB_FB_COL_FBO+int(j*Y_COUNT+Y_COUNT-1)+COL_OFFSET])&0x3ff)/1023.0;

            Vertex_Weight[(i*MERGE_FRONT_LEFT_COL_FBO+j)*6+5] = ((front_Calib[int(i*X_COUNT)*CALIB_FB_COL_FBO+int(j*Y_COUNT)+COL_OFFSET])&0x3ff)/1023.0;
        }//UPDATE X&Y
	}
	for(int i=0;i<MERGE_FRONT_LEFT_COL_FBO*MERGE_FRONT_LEFT_ROW_FBO*6;i++)
	{
        Vertex_index[i] = i;
    }

}

void Test_Fbo(GLuint *TextureId)
{
	static bool Flag_cal=false;
	//unmerge
	static GLuint m_shaderProgram=0;
    	static GLuint m_positionHandle=0;
	static GLuint m_colorHandle=0;
	static GLuint m_textCoordHandle=0;
    	static GLuint m_samplerHandle=0;
	//merge
	static GLuint m_shaderProgram_merge,m_positionHandle_merge,m_colorHandle_merge,m_weightHandle_merge;
	static GLuint m_texCoord1Handle_merge,m_texCoord2Handle_merge,m_sampler1Handle_merge,m_sampler2Handle_merge;


	//static GLuint Texture_Id;
	//static GLuint Framebuffer_Id;
	//static GLuint Renderbuffer_Id;
    static GLfloat Ver_Pos[3*3] = {
				0.0,0.0,0.0,
				1.0,1.0,0.0};
    static GLfloat Ver_Posline[6*3] = {
				-1.0,1.0,0.0,
				-1.0,-1.0,0.0,
				1.0,1.0,0.0,
				-1.0,-1.0,0.0,
				1.0,-1.0,0.0,
				1.0,1.0,0.0};//
    static GLfloat Ver_Color[6*3] = {
				0.0,1.0,0.0,
				0.0,0.0,1.0,
				1.0,0.0,0.0,
				0.0,1.0,0.0,
				0.0,0.0,1.0,
				1.0,0.0,0.0};
    static GLfloat Ver_Colorline[6*2] = {
				0.0,0.0,
				0.0,1.0,
				1.0,0.0,
				0.0,1.0,
				1.0,1.0,
				1.0,0.0};
    static unsigned short IndexArray[6] = {0,1,2,3,4,5};
    if(!Flag_cal)
    {
	//unmerge
	if(loadShaders("/data/opengl_new/shader/FBO.vert","/data/opengl_new/shader/FBO.frag",m_shaderProgram) != 0)
		printf("return error : Test_Fbo()!!!\n");
        m_positionHandle = glGetAttribLocation(m_shaderProgram, "aPosition");
        //m_colorHandle = glGetAttribLocation(m_shaderProgram, "a_Color");
        m_textCoordHandle = glGetAttribLocation(m_shaderProgram, "aTexCoord");
        m_samplerHandle = glGetUniformLocation(m_shaderProgram,"my_Sampler");

	//merge
    	if(loadShaders("/data/opengl_new/shader/FBO_merge.vert","/data/opengl_new/shader/FBO_merge.frag",m_shaderProgram_merge) != 0)
        	printf("return error FBO_merge!!!\n");
    	m_positionHandle_merge = glGetAttribLocation(m_shaderProgram_merge, "aPosition");
        m_colorHandle_merge = glGetAttribLocation(m_shaderProgram_merge, "a_Color");
    	m_weightHandle_merge = glGetAttribLocation(m_shaderProgram_merge,"aWeight");
    	m_texCoord1Handle_merge = glGetAttribLocation(m_shaderProgram_merge, "aTexCoord1");
   	m_texCoord2Handle_merge = glGetAttribLocation(m_shaderProgram_merge, "aTexCoord2");
    	m_sampler1Handle_merge = glGetUniformLocation(m_shaderProgram_merge,"sTexture1");
    	m_sampler2Handle_merge = glGetUniformLocation(m_shaderProgram_merge,"sTexture2");
	//generate tex id & framebuffer id & renderbuffer id
	glGenTextures(1,&Texture_Id);
	glGenFramebuffers(1,&Framebuffer_Id);
	glGenRenderbuffers(1,&Renderbuffer_Id);
    }


	//glDeleteTextures(1,&Texture_Id);
	//glDeleteFramebuffers(1,&Framebuffer_Id);
	//glDeleteRenderbuffers(1,&Renderbuffer_Id);



	glBindTexture(GL_TEXTURE_2D,Texture_Id);
	glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,GPU_IMAGE_WIDTH,GPU_IMAGE_HEIGHT,0,GL_RGB,GL_UNSIGNED_BYTE,NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);

	glBindRenderbuffer(GL_RENDERBUFFER,Renderbuffer_Id);
	glRenderbufferStorage(GL_RENDERBUFFER,GL_DEPTH_COMPONENT16,GPU_IMAGE_WIDTH,GPU_IMAGE_HEIGHT);

	glBindFramebuffer(GL_FRAMEBUFFER,Framebuffer_Id);
	glFramebufferTexture2D(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D,Texture_Id,0);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER,Renderbuffer_Id);
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		printf("glCheckFramebufferStatus is not GL_FRAMEBUFFER_COMPLETE\n");



static GLuint m_vbo[3] = {0,0,0};
    	if(!Flag_cal){
		glGenBuffers(3, m_vbo);
		//glGenBuffers(3, m_vbo1);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*FRONT_BACK_COL_FBO*FRONT_BACK_ROW_FBO*6*3,VertexPos_fbo_Front, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*FRONT_BACK_COL_FBO*FRONT_BACK_ROW_FBO*6*2,VertexColors_fbo_Front, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*6*FRONT_BACK_COL_FBO*FRONT_BACK_ROW_FBO, Index_Array_fbo_FB, GL_STATIC_DRAW);
		Flag_cal=true;
	}


    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);//front
	glBindTexture(GL_TEXTURE_2D, TextureId[0]);
    glUniform1i(m_samplerHandle, 0);
        glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_front[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_front[1]);
        glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, fbo_unmerge_front[2]);
    	glEnableVertexAttribArray(m_positionHandle);
	glEnableVertexAttribArray(m_textCoordHandle);
	glEnableVertexAttribArray(m_samplerHandle);
	glDrawElements(GL_TRIANGLES, FRONT_BACK_COL_FBO*FRONT_BACK_ROW_FBO*6, GL_UNSIGNED_INT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_textCoordHandle);
	glDisableVertexAttribArray(m_samplerHandle);

    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);//back
	glBindTexture(GL_TEXTURE_2D, TextureId[1]);
    glUniform1i(m_samplerHandle, 0);//m_vbo_back
        glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_back[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_back[1]);
        glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, fbo_unmerge_back[2]);
    	glEnableVertexAttribArray(m_positionHandle);
	glEnableVertexAttribArray(m_textCoordHandle);
	glEnableVertexAttribArray(m_samplerHandle);
	glDrawElements(GL_TRIANGLES, FRONT_BACK_COL_FBO*FRONT_BACK_ROW_FBO*6, GL_UNSIGNED_INT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_textCoordHandle);
	glDisableVertexAttribArray(m_samplerHandle);

    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);//left
	glBindTexture(GL_TEXTURE_2D, TextureId[2]);
    glUniform1i(m_samplerHandle, 0);//m_vbo_left LEFT_RIGHT_ROW*LEFT_RIGHT_COL
        glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_left[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_left[1]);
        glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, fbo_unmerge_left[2]);
    	glEnableVertexAttribArray(m_positionHandle);
	glEnableVertexAttribArray(m_textCoordHandle);
	glEnableVertexAttribArray(m_samplerHandle);
	glDrawElements(GL_TRIANGLES, LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO*6, GL_UNSIGNED_INT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_textCoordHandle);
	glDisableVertexAttribArray(m_samplerHandle);

    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);//right
	glBindTexture(GL_TEXTURE_2D, TextureId[3]);
    glUniform1i(m_samplerHandle, 0);//m_vbo_right LEFT_RIGHT_ROW*LEFT_RIGHT_COL
        glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_right[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, fbo_unmerge_right[1]);
        glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, fbo_unmerge_right[2]);
    	glEnableVertexAttribArray(m_positionHandle);
	glEnableVertexAttribArray(m_textCoordHandle);
	glEnableVertexAttribArray(m_samplerHandle);
	glDrawElements(GL_TRIANGLES, LEFT_RIGHT_ROW_FBO*LEFT_RIGHT_COL_FBO*6, GL_UNSIGNED_INT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_textCoordHandle);
	glDisableVertexAttribArray(m_samplerHandle);


    glUseProgram(m_shaderProgram_merge);//merge 0
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, TextureId[0]);
    glUniform1i(m_sampler1Handle_merge, 0);
    glActiveTexture (GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, TextureId[2]);
    glUniform1i(m_sampler2Handle_merge, 1);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_fbo_Merge[0]);
        glVertexAttribPointer(m_positionHandle_merge, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_fbo_Merge[1]);
        glVertexAttribPointer(m_texCoord1Handle_merge, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_fbo_Merge[2]);
        glVertexAttribPointer(m_texCoord2Handle_merge, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo0_fbo_Merge[3]);
        glVertexAttribPointer(m_weightHandle_merge, 1, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*1, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo0_fbo_Merge[4]);
    	glEnableVertexAttribArray(m_positionHandle_merge);
	glEnableVertexAttribArray(m_texCoord1Handle_merge);
	glEnableVertexAttribArray(m_texCoord2Handle_merge);
	glEnableVertexAttribArray(m_weightHandle_merge);
	glEnableVertexAttribArray(m_colorHandle_merge);
	glDrawElements(GL_TRIANGLES, 6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO, GL_UNSIGNED_INT, 0);
	glDisableVertexAttribArray(m_positionHandle_merge);
	glDisableVertexAttribArray(m_texCoord1Handle_merge);
	glDisableVertexAttribArray(m_texCoord2Handle_merge);
	glDisableVertexAttribArray(m_weightHandle_merge);
	glDisableVertexAttribArray(m_colorHandle_merge);

    glUseProgram(m_shaderProgram_merge);//merge 1
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, TextureId[0]);
    glUniform1i(m_sampler1Handle_merge, 0);
    glActiveTexture (GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, TextureId[3]);
    glUniform1i(m_sampler2Handle_merge, 1);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_fbo_Merge[0]);
        glVertexAttribPointer(m_positionHandle_merge, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_fbo_Merge[1]);
        glVertexAttribPointer(m_texCoord1Handle_merge, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_fbo_Merge[2]);
        glVertexAttribPointer(m_texCoord2Handle_merge, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo1_fbo_Merge[3]);
        glVertexAttribPointer(m_weightHandle_merge, 1, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*1, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo1_fbo_Merge[4]);
    	glEnableVertexAttribArray(m_positionHandle_merge);
	glEnableVertexAttribArray(m_texCoord1Handle_merge);
	glEnableVertexAttribArray(m_texCoord2Handle_merge);
	glEnableVertexAttribArray(m_weightHandle_merge);
	glEnableVertexAttribArray(m_colorHandle_merge);
	glDrawElements(GL_TRIANGLES, 6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO, GL_UNSIGNED_INT, 0);
	glDisableVertexAttribArray(m_positionHandle_merge);
	glDisableVertexAttribArray(m_texCoord1Handle_merge);
	glDisableVertexAttribArray(m_texCoord2Handle_merge);
	glDisableVertexAttribArray(m_weightHandle_merge);
	glDisableVertexAttribArray(m_colorHandle_merge);

    glUseProgram(m_shaderProgram_merge);//merge 2
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, TextureId[1]);
    glUniform1i(m_sampler1Handle_merge, 0);
    glActiveTexture (GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, TextureId[2]);
    glUniform1i(m_sampler2Handle_merge, 1);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_fbo_Merge[0]);
        glVertexAttribPointer(m_positionHandle_merge, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_fbo_Merge[1]);
        glVertexAttribPointer(m_texCoord1Handle_merge, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_fbo_Merge[2]);
        glVertexAttribPointer(m_texCoord2Handle_merge, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo2_fbo_Merge[3]);
        glVertexAttribPointer(m_weightHandle_merge, 1, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*1, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo2_fbo_Merge[4]);
    	glEnableVertexAttribArray(m_positionHandle_merge);
	glEnableVertexAttribArray(m_texCoord1Handle_merge);
	glEnableVertexAttribArray(m_texCoord2Handle_merge);
	glEnableVertexAttribArray(m_weightHandle_merge);
	glEnableVertexAttribArray(m_colorHandle_merge);
	glDrawElements(GL_TRIANGLES, 6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO, GL_UNSIGNED_INT, 0);
	glDisableVertexAttribArray(m_positionHandle_merge);
	glDisableVertexAttribArray(m_texCoord1Handle_merge);
	glDisableVertexAttribArray(m_texCoord2Handle_merge);
	glDisableVertexAttribArray(m_weightHandle_merge);
	glDisableVertexAttribArray(m_colorHandle_merge);

    glUseProgram(m_shaderProgram_merge);//merge 3
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, TextureId[1]);
    glUniform1i(m_sampler1Handle_merge, 0);
    glActiveTexture (GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, TextureId[3]);
    glUniform1i(m_sampler2Handle_merge, 1);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_fbo_Merge[0]);
        glVertexAttribPointer(m_positionHandle_merge, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_fbo_Merge[1]);
        glVertexAttribPointer(m_texCoord1Handle_merge, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_fbo_Merge[2]);
        glVertexAttribPointer(m_texCoord2Handle_merge, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo3_fbo_Merge[3]);
        glVertexAttribPointer(m_weightHandle_merge, 1, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*1, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo3_fbo_Merge[4]);
    	glEnableVertexAttribArray(m_positionHandle_merge);
	glEnableVertexAttribArray(m_texCoord1Handle_merge);
	glEnableVertexAttribArray(m_texCoord2Handle_merge);
	glEnableVertexAttribArray(m_weightHandle_merge);
	glEnableVertexAttribArray(m_colorHandle_merge);
	glDrawElements(GL_TRIANGLES, 6*MERGE_FRONT_LEFT_ROW_FBO*MERGE_FRONT_LEFT_COL_FBO, GL_UNSIGNED_INT, 0);
	glDisableVertexAttribArray(m_positionHandle_merge);
	glDisableVertexAttribArray(m_texCoord1Handle_merge);
	glDisableVertexAttribArray(m_texCoord2Handle_merge);
	glDisableVertexAttribArray(m_weightHandle_merge);
	glDisableVertexAttribArray(m_colorHandle_merge);

	//write image to files;
	//printf("before IMAGE_AVM data: %d, %d, %d\n",IMAGE_AVM[0],IMAGE_AVM[1],IMAGE_AVM[2]);
	//glBindTexture(GL_TEXTURE_2D, Texture_Id);
	//glGetTexImage(GL_TEXTURE_2D,0,GL_RGB,GL_UNSIGNED_BYTE,IMAGE_AVM);

	//printf("IMAGE_AVM data: %d, %d, %d\n",IMAGE_AVM[0],IMAGE_AVM[1],IMAGE_AVM[2]);
	//Mat image_avm(GPU_IMAGE_HEIGHT,GPU_IMAGE_WIDTH,CV_8UC3,Scalar(0,0,0));
	//printf("image size:%d %d\n",image_avm.rows,image_avm.cols);

	glReadPixels(0,0,GPU_IMAGE_WIDTH,GPU_IMAGE_HEIGHT,GL_RGB,GL_UNSIGNED_BYTE,IMAGE_AVM);
	static bool Image_flag = false;
	if(!Image_flag){
		FILE *fp;
    		fp = fopen("image_avm.bin", "wb+");
		fseek(fp,0,SEEK_SET);
		fwrite(IMAGE_AVM, sizeof(unsigned char), GPU_IMAGE_WIDTH*GPU_IMAGE_HEIGHT*3, fp);
		fclose(fp);
		Image_flag = true;
		printf("write data complete!!\n");
	}
	glBindFramebuffer(GL_FRAMEBUFFER,0);
int Size_size =0;
//glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE,&Size_size);
//printf("Size_size: %d\n",Size_size);
//printf("TextureId[0]:%d\tTexture_Id:%d\n",TextureId[0] ,Texture_Id);
TextureId[0] = Texture_Id;
}
void Test_Fbo_Render(GLuint *TextureId)
{
	static bool Flag_cal=false;
	static GLuint m_shaderProgram=0;
    	static GLuint m_positionHandle=0;
	static GLuint m_colorHandle=0;
	static GLuint m_textCoordHandle=0;
    	static GLuint m_samplerHandle=0;
	//static GLuint Texture_Id;
	static GLuint Framebuffer_Id;
	static GLuint Renderbuffer_Id;
    static GLfloat Ver_Pos[3*3] = {
				0.0,0.0,0.0,
				1.0,1.0,0.0};
    static GLfloat Ver_Posline[6*3] = {
				0.0,-1.0,0.0,
				0.0,0.0,0.0,
				1.0,0.0,0.0,
				0.0,-1.0,0.0,
				1.0,-1.0,0.0,
				1.0,0.0,0.0};
    static GLfloat Ver_Color[6*3] = {
				0.0,1.0,0.0,
				0.0,0.0,1.0,
				1.0,0.0,0.0,
				0.0,1.0,0.0,
				0.0,0.0,1.0,
				1.0,0.0,0.0};
    static GLfloat Ver_Colorline[6*2] = {
				0.0,1.0,
				0.0,0.0,
				1.0,0.0,
				0.0,1.0,
				1.0,1.0,
				1.0,0.0};
    static unsigned short IndexArray[6] = {0,1,2,3,4,5};

    if(!Flag_cal)
    {
	if(loadShaders("/data/opengl_new/shader/unmerge2d.vert","/data/opengl_new/shader/unmerge2d.frag",m_shaderProgram) != 0)
		printf("return error : Test_Fbo_Render()!!!\n");
        m_positionHandle = glGetAttribLocation(m_shaderProgram, "aPosition");
        //m_colorHandle = glGetAttribLocation(m_shaderProgram, "a_Color");
        m_textCoordHandle = glGetAttribLocation(m_shaderProgram, "aTexCoord");
        m_samplerHandle = glGetUniformLocation(m_shaderProgram,"my_Sampler");


    }
static GLuint m_vbo[3] = {0,0,0};
    	if(!Flag_cal){
		glGenBuffers(3, m_vbo);
		//glGenBuffers(3, m_vbo1);
	}
	Flag_cal=true;
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo[0]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*3,Ver_Posline, GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, m_vbo[1]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*2,Ver_Colorline, GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*6, IndexArray, GL_STATIC_DRAW);


    glUseProgram(m_shaderProgram);
    glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, Texture_Id);
    glUniform1i(m_samplerHandle, 0);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo[0]);
        glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*3, 0);//VERTEX_POS_SIZE * sizeof ( GLfloat )
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo[1]);
        glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, sizeof ( GLfloat )*2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo[2]);
    	//glLineWidth(20);
    	glEnableVertexAttribArray(m_positionHandle);
	glEnableVertexAttribArray(m_textCoordHandle);
	glEnableVertexAttribArray(m_samplerHandle);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_textCoordHandle);
	glDisableVertexAttribArray(m_samplerHandle);
}
