#ifndef _TRACK_BALL_H__
#define _TRACK_BALL_H__
#include "MyVector3D.h"
#include "MyMatrix.h"
class TrackBall
{
public:
	TrackBall(int width,int height,float observeeHeight,float cameraHeight);
	~TrackBall(void);
public:
    float getCurCameraPhei(){return m_fai;}
    float getCurCameraTheta(){return m_theta;}
	void leftButtonDown(int x,int y);
	void leftButtonUp(int x,int y);
	void mouseMove(int x,int y);
	int isEnableRotate(){return m_leftButtonDown;}
	void incrementTheta(int times);
	void incrementPhei(int times);
	void setCurViewByAngle(float phei,float theta);
	MyMatrix getMatrixByThetaPhei(float phei,float theta);
	MyMatrix getMatrixByThetaPhei2(float phei,float theta);
	MyVector3D getCameraPos();
	MyVector3D getObserveePos();
	MyVector3D getCameraUpVec();
public:
	float m_theta,m_fai;
	int m_leftButtonDown;
	int m_width,m_height;
	int m_preX,m_preY;
	float m_observeeHeight;
	float m_cameraHeight;
private:
	//static const float PI;
	static const float RADIUSTOANGLE;
};
#endif // _TRACK_BALL_H__

