#ifndef _MATRIX_STATE_H__
#define _MATRIX_STATE_H__
#include <stack>
#include "MyMatrix.h"
class MatrixState
{
private:
	static std::stack<MyMatrix> m_matrixStack;
	static MyMatrix m_projMatrix;
	static MyMatrix m_viewMatrix;
	static MyMatrix m_curMatrix;
    static MyMatrix m_cameraMatrix;
	static MyMatrix m_finalMatrix;
	static float m_lightPos[3];
	static float m_cameraPos[3];
public:
	static void setInitStack();//获取不变换初始矩阵
	static MyMatrix &getProjMatrix();
	static MyMatrix &getViewMatrix();
	static MyMatrix &getCurMatrix();
	static MyMatrix &getFinalMatrix();
	static MyMatrix &getCameraMatrix();
	static void pushMatrix();
	static void popMatrix();
	static void applyMatrix(const MyMatrix &m);
	static void loadMatrix(const MyMatrix &m);
	static void applyTranslate(float x,float y,float z);
	static void translate(float x,float y,float z);
	static void rotate(float angle,float x,float y,float z);
	static void applyRotate(const float angle, const float x, const float y, const float z);
	static void scale(const float x, const float y, const float z);
	static void applyScale(const float x, const float y,const float z);
	static void setCamera(float cx,float cy,float cz,float tx,float ty,float tz,float upx,float upy,float upz);
	static void setCamera(const MyMatrix &matrix);
	static void setProjectFrustum(float left,float right, float bottom,float top, float near,float far);
	static void setProjectOrtho(float left,float right,float bottom,float top,float near,float far);
	static const float* getCameraPosition();
	static void setLightLocation(float x,float y,float z);
	static void printMatrixState();
};
#endif // _MATRIX_STATE_H__

