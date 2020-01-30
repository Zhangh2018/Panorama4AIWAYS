#include "TrackBall.h"
#include <cmath>
#include <iostream>
#include "data_type.h"
TrackBall::TrackBall(int width,int height,float observeeHeight,float cameraHeight)
{
    m_leftButtonDown = 0;
    m_width = width;
    m_height = height;
    m_observeeHeight = observeeHeight;
    m_cameraHeight = cameraHeight;
    m_fai = 0.0;
    m_theta = 0.0;
}
void TrackBall::setCurViewByAngle(float phei,float theta)
{
    m_fai = phei;
    m_theta = theta;
}
MyMatrix TrackBall::getMatrixByThetaPhei2(float phei,float theta)
{
    MyVector3D observeePos,cameraPos(0,0,m_cameraHeight),cameraUpVec;
    observeePos.x = (m_cameraHeight  - m_observeeHeight)* sin(phei) * sin(theta);
    observeePos.y = (m_cameraHeight  - m_observeeHeight) * sin(phei) * cos(theta);
    observeePos.z = m_observeeHeight + (m_cameraHeight  - m_observeeHeight) * (1.0 - cos(phei));

    MyVector3D eyePos = MyVector3D(cameraPos.x - observeePos.x,cameraPos.y - observeePos.y,cameraPos.z - observeePos.z).Normalize();

    if((eyePos - MyVector3D(0,0,1)).Length() < 0.0001){
        cameraUpVec =  MyVector3D(0,1,0);
    }else{
        MyVector3D axieVec = eyePos ^ MyVector3D(0,0,1);
        cameraUpVec = eyePos ^ axieVec;
        cameraUpVec.Normalize();
    }
    if(cameraUpVec.z < 0)
        cameraUpVec = -cameraUpVec;
    MyMatrix matrix;
    matrix.loadView(cameraPos.x,cameraPos.y,cameraPos.z,observeePos.x,observeePos.y,observeePos.z,cameraUpVec.x,cameraUpVec.y,cameraUpVec.z);
    return matrix;
}
MyMatrix TrackBall::getMatrixByThetaPhei(float phei,float theta)
{
    MyVector3D observeePos,cameraPos,cameraUpVec;
    cameraPos.x = m_cameraHeight* sin(phei) * sin(theta);
    cameraPos.y = m_cameraHeight * sin(phei) * cos(theta);
    cameraPos.z = m_cameraHeight * cos(phei);
    observeePos = (m_observeeHeight / m_cameraHeight) * cameraPos;
    MyVector3D eyePos = MyVector3D(cameraPos.x - observeePos.x,cameraPos.y - observeePos.y,cameraPos.z - observeePos.z).Normalize();

    if((eyePos - MyVector3D(0,0,1)).Length() < 0.0001){
        cameraUpVec =  MyVector3D(0,1,0);
    }else{
        MyVector3D axieVec = eyePos ^ MyVector3D(0,0,1);
        cameraUpVec = eyePos ^ axieVec;
        cameraUpVec.Normalize();
    }
    if(cameraUpVec.z < 0)
        cameraUpVec = -cameraUpVec;
    MyMatrix matrix;
    matrix.loadView(cameraPos.x,cameraPos.y,cameraPos.z,observeePos.x,observeePos.y,observeePos.z,cameraUpVec.x,cameraUpVec.y,cameraUpVec.z);
    return matrix;
}
static float clamp(float realVal,float minVal,float maxVal)
{
    if(realVal < minVal)
    {
        return minVal;
    }
    if(realVal > maxVal)
    {
        return maxVal;
    }
    return realVal;
}
void TrackBall::incrementTheta(int times)
{
    m_theta += times * PI / m_width;
    if(m_theta > PI)
        m_theta = m_theta - 2 * PI;
    else if(m_theta < -PI)
        m_theta = m_theta + 2 * PI;
}
void TrackBall::incrementPhei(int times)
{
    m_fai += times * PI / (4 * m_height);
    m_fai = clamp(m_fai,0,PI / 2);
}
TrackBall::~TrackBall(void)
{
}
MyVector3D TrackBall::getCameraPos()
{
    return MyVector3D(0,0,m_cameraHeight);
}
MyVector3D TrackBall::getObserveePos()
{
    MyVector3D observeePos;
    observeePos.x = (m_cameraHeight  - m_observeeHeight)* sin(m_fai) * sin(m_theta);
    observeePos.y = (m_cameraHeight  - m_observeeHeight) * sin(m_fai) * cos(m_theta);
    observeePos.z = m_observeeHeight + (m_cameraHeight  - m_observeeHeight) * (1.0 - cos(m_fai));
    return observeePos;
}
MyVector3D TrackBall::getCameraUpVec()
{
    MyVector3D cameraPos = getCameraPos();
    MyVector3D observeePos = getObserveePos();
    MyVector3D eyePos = MyVector3D(cameraPos.x - observeePos.x,cameraPos.y - observeePos.y,cameraPos.z - observeePos.z).Normalize();
    if((eyePos - MyVector3D(0,0,1)).Length() < 0.0001)
    {
        return MyVector3D(0,1,0);
    }
    MyVector3D axieVec = eyePos ^ MyVector3D(0,0,1);
    MyVector3D cameraUpVec = eyePos ^ axieVec;
    cameraUpVec.Normalize();
    if(cameraUpVec.z < 0)
        cameraUpVec = -cameraUpVec;
    return cameraUpVec;
}
void TrackBall::leftButtonDown(int x,int y)
{
    m_leftButtonDown = 1;
    m_preX = x;
    m_preY = y;
}
void TrackBall::leftButtonUp(int x,int y)
{
    m_leftButtonDown = 0;
}
void TrackBall::mouseMove(int x,int y)
{
    if(m_leftButtonDown)
    {
        m_fai += (y - m_preY) * PI / (4 * m_height);
        m_theta += (x - m_preX) * PI / m_width;
        m_fai = clamp(m_fai,0,PI / 2);
        if(m_theta > PI)
           m_theta = m_theta - 2 * PI;
        else if(m_theta < -PI)
           m_theta = m_theta + 2 * PI;
        m_preX = x;
        m_preY = y;
    }
}
//const float TrackBall::PI = 3.141592654;
const float TrackBall::RADIUSTOANGLE = 180 / PI; //TrackBall::PI;
