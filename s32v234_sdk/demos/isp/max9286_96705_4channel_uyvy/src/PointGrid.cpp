#include "PointGrid.h"
#include <math.h>
#include <opencv2/core.hpp>

#pragma comment(lib,"opencv_core246d.lib")
#pragma comment(lib,"opencv_highgui246d.lib")
PointGrid::PointGrid(float innerRadius,float outerRadius,float sizePerGrid,float carWidth,float carHeight,float mergeAngleMin,float mergeAngleMax)
{
	m_innerRadius = innerRadius;
	m_outerRadius = outerRadius;
	m_lengthPerGrid = sizePerGrid;
	m_carSize.width = carWidth;
	m_carSize.height = carHeight;
	m_mergeAngleMin = mergeAngleMin;
	m_mergeAngleMax = mergeAngleMax;
	for(int i = 0;i < 8;i++)
		m_curY[i] = (carHeight / 2.0 + m_outerRadius) + 10;
}


PointGrid::~PointGrid(void)
{
}
int PointGrid::generate2DGrid()
{
	float angle1 = m_mergeAngleMax,angle2 = m_mergeAngleMin;
	float yBegin = m_carSize.height / 2.0 + m_outerRadius,yEnd = -m_carSize.height / 2.0 - m_outerRadius;
	float carLeft = -m_carSize.width / 2.0,carRight = m_carSize.width / 2.0,carFront = m_carSize.height / 2.0,carRear = -m_carSize.height / 2.0;
	float yTop1 = m_outerRadius * sin(angle1) + carFront,yTop2 = m_outerRadius * sin(angle2) + carFront;
	float yBottom1 = -m_outerRadius * sin(angle1) + carRear,yBottom2 = -m_outerRadius * sin(angle2) + carRear;
	float yCoord = 0.0;
	//��������1��ֻ��ǰ������
	processColumn1(yBegin,yTop1,0);
	//��������2����ǰ��ǰ��ǰ����������
	processColumn2(yTop1,yTop2,0);
	//��������3������ǰ��ǰ��ǰ�ң����������
	processColumn3(yTop2,carFront,0);
	//��������4����������������
	processColumn4(carFront,carRear);
	//��������5�����󣬺��󣬺󣬺��ң����������
	processColumn3(carRear,yBottom2,1);
	//��������6���к��󣬺󣬺�����������
	processColumn2(yBottom2,yBottom1,1);
	//��������7���к�1������
	processColumn1(yBottom1,yEnd,1);
	return 0;
}
int PointGrid::generate3DVertexArray(std::vector<float> (&positionArray)[8],std::vector<float> (&weightArray)[4])
{
         for(int i = 0;i < 4;i++)
	{
		weightArray[i].clear();
		weightArray[i].insert(weightArray[i].end(),m_weightArray[i + 4].begin(),m_weightArray[i + 4].end());
	}
	for(int i = 0;i < 8;i++)
	{
		generate3DPositionArray(m_vertexArray[i],m_distanceArray[i],positionArray[i],m_innerRadius,m_outerRadius);
	}
	return 0;
}
int PointGrid::generate2DVertexArray(std::vector<float> (&positionArray)[8],std::vector<float> (&weightArray)[4])
{
	for(int i = 0;i < 4;i++)
	{
		weightArray[i].clear();
		weightArray[i].insert(weightArray[i].end(),m_weightArray[i + 4].begin(),m_weightArray[i + 4].end());
	}
	for(int i = 0;i < 8;i++)
	{
		generate2DPositionArray(m_vertexArray[i],positionArray[i]);
	}
	return 0;
}
void PointGrid::processColumn1(float yBegin,float yEnd,int flag)
{
	float yCoord = 0.0;
	float carLeft = -m_carSize.width / 2.0,carRight = m_carSize.width / 2.0,carFront = m_carSize.height / 2.0,carRear = -m_carSize.height / 2.0;
	int regionLabel[2] = {const_value::FRONT_REGION,const_value::REAR_REGION};
	float carLoc[2] = {carFront,carRear};
	float LENGTH_TOLERANT = m_lengthPerGrid / 4;

	for(yCoord = yBegin;yCoord > yEnd + LENGTH_TOLERANT;yCoord -= m_lengthPerGrid)
	{
		float yDistToCar = fabs(carLoc[flag] - yCoord);
		float xBegin = carLeft - sqrt(m_outerRadius * m_outerRadius - yDistToCar * yDistToCar),xEnd = carRight + sqrt(m_outerRadius * m_outerRadius - yDistToCar * yDistToCar);
		processOneLine(xBegin,xEnd,yCoord,regionLabel[flag]);
	}

	yCoord = yEnd;
	float yDistToCar = fabs(carLoc[flag] - yCoord);
	float xBegin = carLeft - sqrt(m_outerRadius * m_outerRadius - yDistToCar * yDistToCar),xEnd = carRight + sqrt(m_outerRadius * m_outerRadius - yDistToCar * yDistToCar);
	processOneLine(xBegin,xEnd,yCoord,regionLabel[flag]);
}
void PointGrid::processColumn2(float yBegin,float yEnd,int flag)
{
	float yCoord = 0.0;
	float angle1 = m_mergeAngleMax,angle2 = m_mergeAngleMin;
	float carLeft = -m_carSize.width / 2.0,carRight = m_carSize.width / 2.0,carFront = m_carSize.height / 2.0,carRear = -m_carSize.height / 2.0;
	int regionLabel1[2] = {const_value::FRONT_LEFT_REGION,const_value::REAR_LEFT_REGION};
	int regionLabel2[2] = {const_value::FRONT_REGION,const_value::REAR_REGION};
	int regionLabel3[3] = {const_value::FRONT_RIGHT_REGION,const_value::REAR_RIGHT_REGION};
	float carLoc[2] = {carFront,carRear};
	float LENGTH_TOLERANT = m_lengthPerGrid / 4;

	for(yCoord = yBegin;yCoord > yEnd + LENGTH_TOLERANT;yCoord -= m_lengthPerGrid)
	{
		float yDistToCar = fabs(carLoc[flag] - yCoord);
		float xBegin = carLeft - sqrt(m_outerRadius * m_outerRadius - yDistToCar * yDistToCar),xEnd = carRight + sqrt(m_outerRadius * m_outerRadius - yDistToCar * yDistToCar);
		float xLeft1 = carLeft - yDistToCar / tan(angle1),xRight1 = carRight + yDistToCar / tan(angle1);
		processOneLine(xBegin,xLeft1,yCoord,regionLabel1[flag]);
		processOneLine(xLeft1,xRight1,yCoord,regionLabel2[flag]);
		processOneLine(xRight1,xEnd,yCoord,regionLabel3[flag]);
	}

	yCoord = yEnd;
	float yDistToCar = fabs(carLoc[flag] - yCoord);
	float xBegin = carLeft - sqrt(m_outerRadius * m_outerRadius - yDistToCar * yDistToCar),xEnd = carRight + sqrt(m_outerRadius * m_outerRadius - yDistToCar * yDistToCar);
	float xLeft1 = carLeft - yDistToCar / tan(angle1),xRight1 = carRight + yDistToCar / tan(angle1);
	processOneLine(xBegin,xLeft1,yCoord,regionLabel1[flag]);
	processOneLine(xLeft1,xRight1,yCoord,regionLabel2[flag]);
	processOneLine(xRight1,xEnd,yCoord,regionLabel3[flag]);
}
void PointGrid::processColumn3(float yBegin,float yEnd,int flag)
{
	float LENGTH_TOLERANT = m_lengthPerGrid / 4;
	float yCoord = 0.0;
	float angle1 = m_mergeAngleMax,angle2 = m_mergeAngleMin;
	float carLeft = -m_carSize.width / 2.0,carRight = m_carSize.width / 2.0,carFront = m_carSize.height / 2.0,carRear = -m_carSize.height / 2.0;
	int regionLabel1[2] = {const_value::FRONT_LEFT_REGION,const_value::REAR_LEFT_REGION};
	int regionLabel2[2] = {const_value::FRONT_REGION,const_value::REAR_REGION};
	int regionLabel3[3] = {const_value::FRONT_RIGHT_REGION,const_value::REAR_RIGHT_REGION};
	float carLoc[2] = {carFront,carRear};

	for(yCoord = yBegin;yCoord > yEnd + LENGTH_TOLERANT;yCoord -= m_lengthPerGrid)
	{
		float yDistToCar = fabs(carLoc[flag] - yCoord);
		float xBegin = carLeft - sqrt(m_outerRadius * m_outerRadius - yDistToCar * yDistToCar),xEnd = carRight + sqrt(m_outerRadius * m_outerRadius - yDistToCar * yDistToCar);
		float xLeft1 = carLeft - yDistToCar / tan(angle1),xRight1 = carRight + yDistToCar / tan(angle1);
		float xLeft2 = carLeft - yDistToCar / tan(angle2),xRight2 = carRight + yDistToCar / tan(angle2);
		processOneLine(xBegin,xLeft2,yCoord,const_value::LEFT_REGION);
		processOneLine(xLeft2,xLeft1,yCoord,regionLabel1[flag]);
		processOneLine(xLeft1,xRight1,yCoord,regionLabel2[flag]);
		processOneLine(xRight1,xRight2,yCoord,regionLabel3[flag]);
		processOneLine(xRight2,xEnd,yCoord,const_value::RIGHT_REGION);
	}
	yCoord = yEnd;
	float yDistToCar = fabs(carLoc[flag] - yCoord);
	float xBegin = carLeft - sqrt(m_outerRadius * m_outerRadius - yDistToCar * yDistToCar),xEnd = carRight + sqrt(m_outerRadius * m_outerRadius - yDistToCar * yDistToCar);
	float xLeft1 = carLeft - yDistToCar / tan(angle1),xRight1 = carRight + yDistToCar / tan(angle1);
	float xLeft2 = carLeft - yDistToCar / tan(angle2),xRight2 = carRight + yDistToCar / tan(angle2);
	processOneLine(xBegin,xLeft2,yCoord,const_value::LEFT_REGION);
	processOneLine(xLeft2,xLeft1,yCoord,regionLabel1[flag]);
	processOneLine(xLeft1,xRight1,yCoord,regionLabel2[flag]);
	processOneLine(xRight1,xRight2,yCoord,regionLabel3[flag]);
	processOneLine(xRight2,xEnd,yCoord,const_value::RIGHT_REGION);
}
void PointGrid::processColumn4(float yBegin,float yEnd)
{
	float LENGTH_TOLERANT = m_lengthPerGrid / 4;
	float yCoord = 0.0;
	float carLeft = -m_carSize.width / 2.0,carRight = m_carSize.width / 2.0;
	for(yCoord = yBegin;yCoord > yEnd + LENGTH_TOLERANT;yCoord -= m_lengthPerGrid)
	{
		float xBegin = carLeft - m_outerRadius,xEnd = carRight + m_outerRadius;
		processOneLine(xBegin,carLeft,yCoord,const_value::LEFT_REGION);
		processOneLine(carRight,xEnd,yCoord,const_value::RIGHT_REGION);
	}
	yCoord = yEnd;
	float xBegin = carLeft - m_outerRadius,xEnd = carRight + m_outerRadius;
	processOneLine(xBegin,carLeft,yCoord,const_value::LEFT_REGION);
	processOneLine(carRight,xEnd,yCoord,const_value::RIGHT_REGION);
}
void PointGrid::processOneLine(float xBegin,float xEnd,float yCoord,int regionLabel)
{
	if(fabs(m_curY[regionLabel] - yCoord) < 0.00001) return;
	m_curY[regionLabel] = yCoord;
	std::vector<float> points;
	float LENGTH_TOLERANT = m_lengthPerGrid / 16;
	float carLeft = -m_carSize.width / 2.0,carRight = m_carSize.width / 2.0,carFront = m_carSize.height / 2.0,carRear = -m_carSize.height / 2.0;
	float angle1 = m_mergeAngleMax,angle2 = m_mergeAngleMin;

	for(float xCoord = xBegin;xCoord < xEnd - LENGTH_TOLERANT;xCoord += m_lengthPerGrid)
	{
		points.push_back(xCoord);
		points.push_back(yCoord);
	}
	points.push_back(xEnd);
	points.push_back(yCoord);
	for(int i = 0;i < points.size() / 2;i++)
	{
		m_vertexArray[regionLabel].push_back(points[2 * i]);
		m_vertexArray[regionLabel].push_back(points[2 * i + 1]);
	}
	switch(regionLabel)
	{
	case const_value::FRONT_REGION:
		{
			for(int i = 0;i < points.size() / 2;i++)
			{
				if(points[2 * i] < carLeft)
					m_distanceArray[regionLabel].push_back(sqrt((points[2 * i] - carLeft) * (points[2 * i] - carLeft) + (points[2 * i + 1] - carFront) * (points[2 * i + 1] - carFront)));
				else if(points[2 * i] > carRight)
                    m_distanceArray[regionLabel].push_back(sqrt((points[2 * i] - carRight) * (points[2 * i] - carRight) + (points[2 * i + 1] - carFront) * (points[2 * i + 1] - carFront)));
				else
					m_distanceArray[regionLabel].push_back(points[2 * i + 1] - carFront);
			}
			break;
		}
	case const_value::REAR_REGION:
		{
			for(int i = 0;i < points.size() / 2;i++)
			{
				if(points[2 * i] < carLeft)
					m_distanceArray[regionLabel].push_back(sqrt((points[2 * i] - carLeft) * (points[2 * i] - carLeft) + (points[2 * i + 1] - carRear) * (points[2 * i + 1] - carRear)));
				else if(points[2 * i] > carRight)
                    m_distanceArray[regionLabel].push_back(sqrt((points[2 * i] - carRight) * (points[2 * i] - carRight) + (points[2 * i + 1] - carRear) * (points[2 * i + 1] - carRear)));
				else
					m_distanceArray[regionLabel].push_back(carRear - points[2 * i + 1]);
			}
			break;
		}
	case const_value::LEFT_REGION:
		{
			for(int i = 0;i < points.size() / 2;i++)
			{
				if(points[2 * i + 1] > carFront)
					m_distanceArray[regionLabel].push_back(sqrt((points[2 * i] - carLeft) * (points[2 * i] - carLeft) + (points[2 * i + 1] - carFront) * (points[2 * i + 1] - carFront)));
				else if(points[2 * i + 1] < carRear)
					m_distanceArray[regionLabel].push_back(sqrt((points[2 * i] - carLeft) * (points[2 * i] - carLeft) + (points[2 * i + 1] - carRear) * (points[2 * i + 1] - carRear)));
				else
					m_distanceArray[regionLabel].push_back(carLeft - points[2 * i]);
			}
			break;
		}
	case const_value::RIGHT_REGION:
		{
			for(int i = 0;i < points.size() / 2;i++)
			{
				if(points[2 * i + 1] > carFront)
					m_distanceArray[regionLabel].push_back(sqrt((points[2 * i] - carRight) * (points[2 * i] - carRight) + (points[2 * i + 1] - carFront) * (points[2 * i + 1] - carFront)));
				else if(points[2 * i + 1] < carRear)
					m_distanceArray[regionLabel].push_back(sqrt((points[2 * i] - carRight) * (points[2 * i] - carRight) + (points[2 * i + 1] - carRear) * (points[2 * i + 1] - carRear)));
				else
					m_distanceArray[regionLabel].push_back(points[2 * i] - carRight);
			}
			break;
		}
	case const_value::FRONT_LEFT_REGION:
		{
			for(int i = 0;i < points.size() / 2;i++)
			{
				float weight = 0.5;
				if(carFront != points[2 * i + 1])
				{
					float angle = atan((points[2 * i + 1] - carFront) / (carLeft - points[2 * i]));
					weight = (angle - angle2) / (angle1 - angle2);
				}
				m_distanceArray[regionLabel].push_back(sqrt((points[2 * i] - carLeft) * (points[2 * i] - carLeft) + (points[2 * i + 1] - carFront) * (points[2 * i + 1] - carFront)));
				m_weightArray[regionLabel].push_back(weight);
			}
			break;
		}
	case const_value::FRONT_RIGHT_REGION:
		{
			for(int i = 0;i < points.size() / 2;i++)
			{
				float weight = 0.5;
				if(carFront != points[2 * i + 1])
				{
					float angle = atan((points[2 * i + 1] - carFront) / (points[2 * i] - carRight));
					weight = (angle - angle2) / (angle1 - angle2);
				}
				m_distanceArray[regionLabel].push_back(sqrt((points[2 * i] - carRight) * (points[2 * i] - carRight) + (points[2 * i + 1] - carFront) * (points[2 * i + 1] - carFront)));
				m_weightArray[regionLabel].push_back(weight);
			}
			break;
		}
	case const_value::REAR_LEFT_REGION:
		{
			for(int i = 0;i < points.size() / 2;i++)
			{
				float weight = 0.5;
				if(carRear != points[2 * i + 1])
				{
					float angle = atan((carRear - points[2 * i + 1]) / (carLeft - points[2 * i]));
					weight = (angle - angle2) / (angle1 - angle2);
				}
				m_distanceArray[regionLabel].push_back(sqrt((points[2 * i] - carLeft) * (points[2 * i] - carLeft) + (points[2 * i + 1] - carRear) * (points[2 * i + 1] - carRear)));
				m_weightArray[regionLabel].push_back(weight);
			}
			break;
		}
	case const_value::REAR_RIGHT_REGION:
		{
			for(int i = 0;i < points.size() / 2;i++)
			{
				float weight = 0.5;
				if(carRear != points[2 * i + 1])
				{
					float angle = atan((carRear - points[2 * i + 1]) / (points[2 * i] - carRight));
					weight = (angle - angle2) / (angle1 - angle2);
				}
				m_distanceArray[regionLabel].push_back(sqrt((points[2 * i] - carRight) * (points[2 * i] - carRight) + (points[2 * i + 1] - carRear) * (points[2 * i + 1] - carRear)));
				m_weightArray[regionLabel].push_back(weight);
			}
			break;
		}
	default:
		break;
	}
}
int PointGrid::generateIndexArray(const std::vector<float> &vertexArray,std::vector<unsigned short> &index,bool isPanorama3D)
{
	int vertexNum = vertexArray.size() / 2;
	std::vector<unsigned short> indexArray1;
	std::vector<std::vector<unsigned short> > verRows;
	float curY = vertexArray[1],curRow = 0;
	verRows.push_back(std::vector<unsigned short>());
	verRows[curRow].push_back(0);
	for(int i = 1;i < vertexArray.size() / 2;i++)
	{
		if((curY - vertexArray[2 * i + 1]) > const_value::ESP)
		{
			curRow++;
			curY = vertexArray[2 * i + 1];
			verRows.push_back(std::vector<unsigned short>());
			verRows[curRow].push_back(i);
		}
		else
		{
			verRows[curRow].push_back(i);
		}
	}
	for(int i = 0;i < verRows.size() - 1;i++)
	{
		int minNum = std::min(verRows[i].size(),verRows[i + 1].size());
		for(int j = 0;j < minNum - 1;j++)
		{
			indexArray1.push_back(verRows[i][j]);
			indexArray1.push_back(verRows[i + 1][j]);
			indexArray1.push_back(verRows[i + 1][j + 1]);
			indexArray1.push_back(verRows[i][j]);
			indexArray1.push_back(verRows[i + 1][j + 1]);
			indexArray1.push_back(verRows[i][j + 1]);
		}
		if(verRows[i].size() > verRows[i + 1].size())
		{
			int lastIndex = verRows[i + 1].size() - 1;
			for(int k = lastIndex;k < verRows[i].size() - 1;k++)
			{
				indexArray1.push_back(verRows[i + 1][lastIndex]);
				indexArray1.push_back(verRows[i][k + 1]);
				indexArray1.push_back(verRows[i][k]);
			}
		}
		else if(verRows[i].size() < verRows[i + 1].size())
		{
			int lastIndex = verRows[i].size() - 1;
			for(int k = lastIndex;k < verRows[i + 1].size() - 1;k++)
			{
				indexArray1.push_back(verRows[i][lastIndex]);
				indexArray1.push_back(verRows[i + 1][k]);
				indexArray1.push_back(verRows[i + 1][k + 1]);
			}
		}
	}
	index.resize(indexArray1.size());
	if(indexArray1.size() != 0)
	     memcpy(&index[0],&indexArray1[0],sizeof(unsigned short) * indexArray1.size());
	return 0;
}
int PointGrid::generate3DIndexArray(std::vector<unsigned short> (&indexArray)[8])
{
	for(int i = 0;i < const_value::REGION_NUM;i++)
		generateIndexArray(m_vertexArray[i],indexArray[i],true);
	return 0;
}
int PointGrid::generate2DIndexArray(std::vector<unsigned short> (&indexArray)[8])
{
	for(int i = 0;i < const_value::REGION_NUM;i++)
		generateIndexArray(m_vertexArray[i],indexArray[i],false);
	return 0;
}
void PointGrid::testVertexArray(std::vector<unsigned short> (&indexArray)[8])
{
//	float carWidth = m_carSize.width;
//	float carHeight = m_carSize.height;
//	float outerRadius = m_outerRadius;
//	cv::Scalar color[8] = {cv::Scalar(0,0,255),cv::Scalar(0,0,255),cv::Scalar(0,0,255),cv::Scalar(0,0,255),
//	                       cv::Scalar(255,0,0),cv::Scalar(255,0,0),cv::Scalar(255,0,0),cv::Scalar(255,0,0)};
//	float width = (carWidth + outerRadius * 2) * 10 + 200;
//	float height = (carHeight + outerRadius * 2) * 10 + 200;
//	float xOffset = width / 2.0,yOffset = height / 2.0;
//    float xScale = 10.0,yScale = 10.0;
//	cv::Mat image(cv::Size(width,height),CV_8UC3,cv::Scalar(0,0,0,0));
//	for(int i = 0;i < 8;i++)
//	{
//		for(int j = 0;j < indexArray[i].size() / 6;j++)
//		{
//			float px[3],py[3];
//			cv::Point point[3];
//			for(int k = 0;k < 3;k++)
//			{
//				px[k] = m_vertexArray[i][indexArray[i][3 * j + k] * 2];
//				py[k] = m_vertexArray[i][indexArray[i][3 * j + k] * 2 + 1];
//				point[k] = cv::Point(int(px[k] * xScale + xOffset + 0.5),int(py[k] * yScale + yOffset + 0.5));
//			}
//			cv::line(image,point[0],point[1],color[i],1);
//			cv::line(image,point[1],point[2],color[i],1);
//			cv::line(image,point[2],point[0],color[i],1);
//		}
//	}
//	cv::imwrite("image/triangle.jpg",image);
}
int PointGrid::generate3DPositionArray(const std::vector<float> &vertexArray,const std::vector<float> &distanceArray,
                                       std::vector<float> &positionArray,float innerRadius,float outerRadius)
{
	//numPoints��ʾ�ж��ٸ��������Ҫ����߶ȣ�positionArray������������ά����
	int numPoints = distanceArray.size();
	positionArray.resize(numPoints * 3);
	float *lowPosition = &positionArray[0];
	float heightTallThanGround = sqrt(outerRadius * outerRadius - innerRadius * innerRadius);
	for(int i = 0;i < numPoints;i++)
	{
		lowPosition[3 * i] = vertexArray[2 * i];
		lowPosition[3 * i + 1] = vertexArray[2 * i + 1];
		if(distanceArray[i] < innerRadius)   //��������ڰ뾶��߶�Ϊ0
		{
			lowPosition[3 * i + 2] = 0;
		}
		else if(distanceArray[i] >= outerRadius)   //������������뾶���߶�Ϊ��ߵ�heightTallThanGround
                  {
                           lowPosition[3  * i + 2] = heightTallThanGround;
                  }
		else
		{
			lowPosition[3  * i + 2] = heightTallThanGround - sqrt(outerRadius * outerRadius - distanceArray[i] * distanceArray[i]);
		}
	}
	return 0;
}
int PointGrid::generate2DPositionArray(const std::vector<float> &vertexArray,std::vector<float> &positionArray)
{
	int numPoints = vertexArray.size() / 2;
	positionArray.resize(numPoints * 3);
	for(int i = 0;i < numPoints;i++)
	{
		positionArray[3 * i] = vertexArray[2 * i];
		positionArray[3 * i + 1] = vertexArray[2 * i + 1];
		positionArray[3 * i + 2] = 0;
	}
	return 0;
}
