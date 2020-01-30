#pragma once

#include <memory>
#include <vector>
#include "LUT.h"

#include <opencv2/core.hpp>
#include <GLES2/gl2.h>
#include <glm/glm.hpp>

class BoxExpandView
{
public:
	BoxExpandView() = default;
	BoxExpandView(const BoxExpandView&) = default;
	BoxExpandView(int camId, const char* configPath);
	~BoxExpandView();
	void draw(GLuint textures, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));

private:
	void generateVertices(const cv::Mat_<cv::Point2i>& lut);
	cv::Mat_<cv::Point2i> generateBoxExpandLUT();

	void loadConfig(const char* path);
	static GLuint program;
	static GLint positionAttr;
	static GLint texCoordAttr;
	static GLint transformMatrixUnif;
	static GLint samplerUnif;

	bool isVerticesGenerated = false;
	GLuint positionVBO;
	GLuint texCoordVBO;
	int vertexCount;
	int camId;

	int srcWidth, srcHeight;
	int resultWidth, resultHeight;

	// box 展开中间区域的宽度，以像素计
	float boxCenterWidth;

	// 视角在 Y 轴上的平移量，以像素计
	float translationY;

	// box 展开中间区域的fov，以弧度计
	float fovHCenter;

	// box展开中的两边区域水平方向的 fov，以弧度计
	float fovHSide;

	// 垂直方向的 fov，以弧度计
	float fovV;
};