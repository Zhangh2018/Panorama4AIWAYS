/***********************************************************************
 *
 * Mario Created on   : 2019-06-11 18:02
 * Filename      : TransBot.h
 * Function      : 透明车底功能实现
 *
 *************************************************************************/

# pragma once

#include "DrawScene.h"
#include "ShaderUtil.h"
#include "GLES2/gl2.h"
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <opencv2/core.hpp>

// 里程计的数据结构
struct Encoder{
    int x;
    int y;
    int theta;
};

class TransBot{

    public:

        /**
         * @brief 透明车底初始化
         *
         * @param fbot
         * @param preTextureIDt             前一帧texture
         * @param currentTextureIDt         当前一帧texture
         * @param wholeCols                 拼接宽 单位像素:810
         * @param wholeRows                 拼接长 单位像素:1080
         * @param rowOffset                 (拼接宽 - 车辆宽)/2 单位像素: 311
         * @param wholeRows                 (拼接长 - 车辆长)/2 单位像素: 291
         */
        TransBot(GLuint fbot,
                 GLuint preTextureIDt,
                 GLuint currentTextureIDt,
                 int wholeColst = 810,
                 int wholeRowst = 1080,
                 int colOffsett = 311,
                 int rowOffsett = 291);

        /**
         * @brief 从里程计数据计算前后帧的变换,并进行贴图
         *
         * @param encoderData
         */
        void generateTransBot(Encoder &encoderData, const bool& reset = false);


        /**
         * @brief 消抖滤波法
         *
         * @param encoderData
         */
        void shakeOff(Encoder &encoderData);


        /**
         * @brief 一阶滞后滤波法
         *
         * @param encoderData
         */
        void firstOrderLag(Encoder &encoderData, float factor = 0.91667);

        /**
         * @brief 设置渲染变换矩阵
         *
         * @param bevTransformMatrixt
         */
        void setTransMatrix(const glm::mat4& bevTransformMatrixt);

    private:

        /**
         * @brief 由当前帧像素坐标,变换矩阵,得到前一帧的像素坐标
         *
         * @param p0x                       input: 当前帧像素坐标x
         * @param p0y                       input: 当前帧像素坐标y
         * @param TAB                       input: 前后帧转移矩阵T
         * @param pos0X                     output: 前一帧像素坐标x
         * @param pos0Y                     output: 前一像素坐标y
         */
		void transCur2Pre(
                     const float& p0x,
					 const float& p0y,
                     const cv::Mat_<float> TAB,
					 float& pos0X,
					 float& pos0Y);

        /**
         * @brief 由里程计数据生成前后两帧的坐标变换
         *
         * @param encoderData
         * @param TAB
         */
        void calTransformMatrix(
                     const Encoder& encoderData,
                     cv::Mat& TAB,
                     const bool& reset = false);

        GLuint fbo;
        GLuint preTextureID;
        GLuint currentTextureID;
        Encoder encoderData;

        GLuint vertexbuffer;
        GLuint uvbuffer;
        GLuint uvbufferOrigin;
        GLuint TextureID1;
        GLuint TextureID2;
        GLuint program;
        GLuint m_transformMatrixHandle;

        GLuint m_minXHandle;
        GLuint m_maxXHandle;
        GLuint m_minYHandle;
        GLuint m_maxYHandle;

        GLuint m_minXFHandle;
        GLuint m_maxXFHandle;
        GLuint m_minYFHandle;
        GLuint m_maxYFHandle;
        /**
        int encoderTMP1; // 保存最近里程计数据1
        float encoderTMP2; // 保存前一个里程计数据2
        int encoderTMP3; // 用于保证递增性
        */

		int encoderXP;
		int encoderYP;
		int encoderThetaP;

        bool resetP;

		int encoderThetaShakeOff;
		int NShakeOff;
        int NShakeOffThreshold;

        float wholeCols;
        float wholeRows;

        float rowOffset; // 291*0.95
        float colOffset; // 311*0.95

        float factor; // for merge area;

        GLfloat g_vertex_buffer_data[18];
        GLfloat g_uv_buffer_data[12];
        GLfloat g_uv_buffer_data_origin[12];

        glm::mat4 bevTransformMatrix;

};
