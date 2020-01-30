#ifndef _POINT_MAP_H__
#define _POINT_MAP_H__
#include "opencv2/core.hpp"
class PointMap
{
public:
	PointMap(void);
	~PointMap(void);
public:
	int generateUnmergeTextCoord(const std::vector<float> pointArray[],std::vector<float> textCoordArray[]);
	int generateMergeTextCoord(const std::vector<float> pointArray[4],const std::vector<float> weightArray[4],std::vector<float> textCoordArray1[4],std::vector<float> textCoordArray2[4]);
};
#endif // _POINT_MAP_H__
