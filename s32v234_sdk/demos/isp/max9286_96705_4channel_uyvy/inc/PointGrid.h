#ifndef _POINT_GRID_H__
#define _POINT_GRID_H__
#include <vector>
#include "const_value.h"
#include "ImageUtilStruct.h"
#include "opencv2/core.hpp"
class PointGrid
{
public:
	PointGrid(float innerRadius,float outerRadius,float sizePerGrid,float carWidth,float carHeight,float mergeAngleMin,float mergeAngleMax);
	~PointGrid(void);
private:
//网格参数
	float m_innerRadius;
	float m_outerRadius;
    float m_lengthPerGrid;
	ImageSize<float> m_carSize;
	float m_mergeAngleMin;
	float m_mergeAngleMax;
//顶点数组
	float m_curY[const_value::REGION_NUM];
	std::vector<float> m_vertexArray[const_value::REGION_NUM];
	std::vector<float> m_distanceArray[const_value::REGION_NUM];
	std::vector<float> m_weightArray[const_value::REGION_NUM];
public:
	void testVertexArray(std::vector<unsigned short> (&indexArray)[8]);
	int generate2DGrid();
	int generate3DVertexArray(std::vector<float> (&positionArray)[8],std::vector<float> (&weightArray)[4]);
	int generate2DVertexArray(std::vector<float> (&positionArray)[8],std::vector<float> (&weightArray)[4]);
	int generate3DIndexArray(std::vector<unsigned short> (&indexArray)[8]);
	int generate2DIndexArray(std::vector<unsigned short> (&indexArray)[8]);
	void processColumn1(float yBegin,float yEnd,int flag);
	void processColumn2(float yBegin,float yEnd,int flag);
	void processColumn3(float yBegin,float yEnd,int flag);
	void processColumn4(float yBegin,float yEnd);
	void processOneLine(float xBegin,float xEnd,float yCoord,int regionLabel);
	static int generateIndexArray(const std::vector<float> &vertexArray,std::vector<unsigned short> &index,bool isPanorama3D);
	static int generate3DPositionArray(const std::vector<float> &vertexArray,const std::vector<float> &distanceArray,std::vector<float> &positionArray,float innerRadius,float outerRadius);
	static int generate2DPositionArray(const std::vector<float> &vertexArray,std::vector<float> &positionArray);
};
#endif // _POINT_GRID_H__

