#include "PointMap.h"
#include <fstream>
#include <iostream>
#include "camera_model.h"
#include "const_value.h"
using namespace const_value;
PointMap::PointMap(void)
{
}
PointMap::~PointMap(void)
{
}
cv::Point2f getProjPoint(const cv::Point3f &objects,int index)
{
      Float64_t world_points[3] = {objects.x,objects.y,objects.z},camera_ray[3], uv[2];
      World_Ray_To_Cam_Ray(camera_ray, world_points, &Camera_Model[index]->camera_model_ext);
      Cam_Ray_To_Image_Point(uv, camera_ray, &(Camera_Model[index]->camera_model_int));
      return cv::Point2f(uv[1],uv[0]);
}
cv::Point2f getGroundPoint(const cv::Point3f &camPos,const cv::Point3f &points)
{
    double zDiff = camPos.z - points.z;
    cv::Point2f point = cv::Point(camPos.z * points.x - camPos.x * points.z,camPos.z * points.y - camPos.y * points.z);
    return cv::Point2f(point.x / zDiff,point.y / zDiff);
}
cv::Point3f getMergePoint(const cv::Point3f &point,const cv::Point3f &cameraPos1,const cv::Point3f &cameraPos2,float weight)
{
    float minHeight = std::min(cameraPos1.z,cameraPos2.z) * 0.9;
    if(point.z > 0)
    {
        return point;
    }
    else
    {
        cv::Point2f groundPoint1 = getGroundPoint(cameraPos1,point);
        cv::Point2f groundPoint2 = getGroundPoint(cameraPos2,point);
        cv::Point2f mergedPoint = weight * groundPoint1 + (1.0 - weight) * groundPoint2;
        return cv::Point3f(mergedPoint.x,mergedPoint.y,0.0);
    }
}
static float clamp(float realVal,float minVal,float maxVal)
{
    return (realVal < minVal ? minVal : (realVal > maxVal ? maxVal : realVal));
}
int PointMap::generateUnmergeTextCoord(const std::vector<float> pointArray[],std::vector<float> textCoordArray[])
{
    for(int j = 0; j < 4; j++)
    {
        int nPoints = pointArray[j].size() / 3;
        textCoordArray[j].resize(nPoints * 2);
        for(int i = 0; i < nPoints; i++)
        {
            cv::Point3f worldPoint(pointArray[j][3 * i],pointArray[j][3 * i + 1],pointArray[j][3 * i + 2]);
            cv::Point2f texCoord = getProjPoint(worldPoint,j);
            textCoordArray[j][2 * i] = clamp(texCoord.x / SRC_IMAGE_WIDTH,0.0,1.0);
            textCoordArray[j][2 * i + 1] = clamp(1 - texCoord.y / SRC_IMAGE_HEIGHT,0.0,1.0);
        }
    }
    return 0;
}
int PointMap::generateMergeTextCoord(const std::vector<float> pointArray[4],const std::vector<float> weightArray[4],std::vector<float> textCoordArray1[4],std::vector<float> textCoordArray2[4])
{
    int mergeIndex1[4] = {0,0,1,1},mergeIndex2[4] = {2,3,2,3};
    cv::Point3d camPos[4];
    for(int i = 0;i < 4;i++){
       camPos[i] = cv::Point3d(g_camPos[i][0],g_camPos[i][1],g_camPos[i][2]);
    }
    for(int j = 0; j <4; j++)
    {
        int nPoints = weightArray[j].size();
        textCoordArray1[j].resize(nPoints * 2);
        textCoordArray2[j].resize(nPoints * 2);
        for(int i = 0; i < nPoints; i++)
        {
            cv::Point3f worldPoint(pointArray[j][3 * i],pointArray[j][3 * i + 1],pointArray[j][3 * i + 2]);
            cv::Point3f mergePoint = getMergePoint(worldPoint,camPos[mergeIndex1[j]],camPos[mergeIndex2[j]],weightArray[j][i]);
            cv::Point2f texCoord1 = getProjPoint(mergePoint,mergeIndex1[j]);
            cv::Point2f texCoord2 = getProjPoint(mergePoint,mergeIndex2[j]);
            textCoordArray1[j][2 * i] = clamp(texCoord1.x / SRC_IMAGE_WIDTH,0.0,1.0);
            textCoordArray1[j][2 * i + 1] = clamp(1 - texCoord1.y / SRC_IMAGE_HEIGHT,0.0,1.0);
            textCoordArray2[j][2 * i] = clamp(texCoord2.x / SRC_IMAGE_WIDTH,0.0,1.0);
            textCoordArray2[j][2 * i + 1] = clamp(1 - texCoord2.y / SRC_IMAGE_HEIGHT,0.0,1.0);
        }
    }
    return 0;
}
