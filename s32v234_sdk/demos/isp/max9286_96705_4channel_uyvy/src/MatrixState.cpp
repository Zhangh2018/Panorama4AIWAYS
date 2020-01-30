#include "MatrixState.h"
#include "MyVector3D.h"
#include <fstream>
void MatrixState::setInitStack()
{
	m_curMatrix.loadIdentity();
	m_cameraMatrix.loadIdentity();
	m_projMatrix.loadIdentity();
}
MyMatrix &MatrixState::getProjMatrix()
{
	return m_projMatrix;
}
MyMatrix &MatrixState::getViewMatrix()
{
	m_viewMatrix = m_cameraMatrix * m_curMatrix;
	return m_viewMatrix;
}
MyMatrix &MatrixState::getCurMatrix()
{
	return m_curMatrix;
}
MyMatrix &MatrixState::getFinalMatrix()
{
	m_finalMatrix = m_projMatrix * m_cameraMatrix * m_curMatrix;
	return m_finalMatrix;
}
MyMatrix &MatrixState::getCameraMatrix()
{
	return m_cameraMatrix;
}
void MatrixState::applyMatrix(const MyMatrix &m)
{
	m_curMatrix *= m;
}
void MatrixState::loadMatrix(const MyMatrix &m)
{
	m_curMatrix = m;
}
void MatrixState::pushMatrix()
{
	m_matrixStack.push(m_curMatrix);
}
void MatrixState::popMatrix()
{
	m_curMatrix = m_matrixStack.top();
	m_matrixStack.pop();
}
void MatrixState::translate(float x,float y,float z)
{
	m_curMatrix.loadTranslate(x, y, z);
}
void MatrixState::rotate(float angle,float x,float y,float z)
{
	m_curMatrix.loadRotate(angle,x,y,z);
}
void MatrixState::scale(const float x, const float y, const float z)
{
	m_curMatrix.loadScale(x,y,z);
}
void MatrixState::applyTranslate(float x,float y,float z)
{
	m_curMatrix.applyTranslate(x,y,z);
}
void MatrixState::printMatrixState()
{
	std::ofstream ofp1("matrix/cameraMatrix.txt",std::ios::out);
	std::ofstream ofp2("matrix/curMatrix.txt",std::ios::out);
	std::ofstream ofp3("matrix/finalMatrix.txt",std::ios::out);
	std::ofstream ofp4("matrix/projMatrix.txt",std::ios::out);
	std::ofstream ofp5("matrix/viewMatrix.txt",std::ios::out);
	ofp1<<getCameraMatrix();
	ofp2<<getCurMatrix();
	ofp3<<getFinalMatrix();
	ofp4<<getProjMatrix();
	ofp5<<getViewMatrix();
}
const float* MatrixState::getCameraPosition()
{
	return m_cameraPos;
}
void MatrixState::applyRotate(const float angle, const float x, const float y, const float z)
{
	m_curMatrix.applyRotate(angle,x,y,z);
}
void MatrixState::applyScale(const float x, const float y,const float z)
{
	m_curMatrix.applyScale(x,y,z);
}
void MatrixState::setCamera(float cx,float cy,float cz,float tx,float ty,float tz,float upx,float upy,float upz)
{
	m_cameraMatrix.loadView(cx,cy,cz,tx,ty,tz,upx,upy,upz);
	m_cameraPos[0] = cx;
	m_cameraPos[1] = cy;
	m_cameraPos[2] = cz;
}
void MatrixState::setCamera(const MyMatrix &matrix)
{
   m_cameraMatrix = matrix;
}
void MatrixState::setProjectFrustum(float left,float right, float bottom,float top, float near,float far)
{
	m_projMatrix.loadFrustum(left,right,bottom,top,near,far);
}
void MatrixState::setProjectOrtho(float left,float right,float bottom,float top,float near,float far)
{
	m_projMatrix.loadOrtho(left,right,bottom,top,near,far);
}
void MatrixState::setLightLocation(float x,float y,float z)
{
	m_lightPos[0] = x;
	m_lightPos[1] = y;
	m_lightPos[2] = z;
}
float MatrixState::m_lightPos[3] = {0,0,0};
float MatrixState::m_cameraPos[3] = {0,0,0};
std::stack<MyMatrix> MatrixState::m_matrixStack;
MyMatrix MatrixState::m_projMatrix;
MyMatrix MatrixState::m_viewMatrix;
MyMatrix MatrixState::m_curMatrix;
MyMatrix MatrixState::m_cameraMatrix;
MyMatrix MatrixState::m_finalMatrix;
